// Device identity, CMC and info / heartbeat multicast. See aoip_info.h.
//
// Structure and every constant follow FPGA project firmware/aoip_info.c
// and aoip_cmc.c. Where this file differs, it is because this device is a
// RECEIVER (8 rx channels, no tx) and those differences are called out.

#include "aoip_info.h"
#include "aoip_wire.h"
#include "eth_ts.h"
#include "ptpv1.h"
#include "rate.h"
#include "app_config.h"
#include "lwip/sockets.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "info";

#define MCAST_HDR_LEN       32
#define HEARTBEAT_MS        1000
// Announced CONTINUOUSLY, not as a burst at boot: DC rebuilds its list from
// these, and a device that stops announcing vanishes on the next Refresh
// (FPGA aoip_info.c, INFO_ANNOUNCE_MS).
#define INFO_ANNOUNCE_MS    3000

#define OP_CMC_DEVICE_ADVERTISEMENT  0x1001

static const uint8_t GRP_DEVINFO[4]   = {224, 0, 0, 231};
static const uint8_t GRP_HEARTBEAT[4] = {224, 0, 0, 233};

// The literal ASCII "Audinate" identifies the protocol, not the manufacturer.
static const char VENDOR[8] = {'A','u','d','i','n','a','t','e'};

static uint8_t      s_devid[8];
static int          s_sock = -1;          // bound to 8700: every info TX comes from it
static uint16_t     s_seq;
static SemaphoreHandle_t s_tx_lock;       // s_buf is shared by the rx and announce tasks
static uint8_t      s_buf[512];

const uint8_t *aoip_device_id(void) { return s_devid; }

static inline void put_u16(uint8_t *p, uint32_t at, uint16_t v)
{
    p[at] = (uint8_t)(v >> 8); p[at + 1] = (uint8_t)v;
}
static inline void put_u32(uint8_t *p, uint32_t at, uint32_t v)
{
    p[at] = (uint8_t)(v >> 24); p[at + 1] = (uint8_t)(v >> 16);
    p[at + 2] = (uint8_t)(v >> 8); p[at + 3] = (uint8_t)v;
}

// Fixed-width field, zero-padded, NO reserved terminator: a string of exactly
// `width` characters runs into the next field.
static void put_fixed(uint8_t *c, uint32_t at, uint32_t width, const char *s)
{
    uint32_t i = 0;
    for (; s[i] && i < width; i++) c[at + i] = (uint8_t)s[i];
    for (; i < width; i++)         c[at + i] = 0;
}

static uint32_t put_hdr_seq(uint8_t *p, uint16_t start_code, const uint8_t op[8],
                            uint16_t seq)
{
    put_u16(p, 0, start_code);
    put_u16(p, 2, 0);                         // total_length, patched by send
    put_u16(p, 4, seq);
    put_u16(p, 6, 0);                         // process id
    memcpy(p + 8, s_devid, 8);
    memcpy(p + 16, VENDOR, 8);
    memcpy(p + 24, op, 8);
    return MCAST_HDR_LEN;
}
static uint32_t put_hdr(uint8_t *p, uint16_t start_code, const uint8_t op[8])
{
    return put_hdr_seq(p, start_code, op, s_seq);
}

static void send_to(const uint8_t ip[4], uint16_t port, uint32_t n)
{
    put_u16(s_buf, 2, (uint16_t)n);
    struct sockaddr_in to = { .sin_family = AF_INET, .sin_port = htons(port) };
    memcpy(&to.sin_addr.s_addr, ip, 4);
    sendto(s_sock, s_buf, n, 0, (struct sockaddr *)&to, sizeof(to));
    s_seq++;
}

static bool our_ip(uint8_t ip[4], uint8_t mask[4], uint8_t gw[4])
{
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(eth_ts_netif(), &info) != ESP_OK || !info.ip.addr)
        return false;
    memcpy(ip, &info.ip.addr, 4);             // lwIP keeps these in network order
    if (mask) memcpy(mask, &info.netmask.addr, 4);
    if (gw)   memcpy(gw, &info.gw.addr, 4);
    return true;
}

// ---------------------------------------------------------------------------
// Heartbeat -> 224.0.0.233:8708, 1 Hz, start code 0xfffe
//
// Sub-records: u16 length, u16 type, u16 4, u16 content length, u16 seq, u16 0.
// All five types every real device sends. The AM2 on this bench, the closest
// analogue (a receiver), sends exactly 8000/8001/8002/8003/8004.
// ---------------------------------------------------------------------------

static void send_heartbeat(void)
{
    static const uint8_t op[8] = {0x00, 0x08, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00};
    uint8_t *p = s_buf;
    uint32_t n = put_hdr(p, 0xFFFE, op);

    // 0x8001: PTP clock frequency offset, ppb -- what DC plots in its histogram.
    put_u16(p, n, 16); put_u16(p, n + 2, 0x8001); put_u16(p, n + 4, 4);
    put_u16(p, n + 6, 4); put_u16(p, n + 8, s_seq); put_u16(p, n + 10, 0);
    put_u32(p, n + 12, (uint32_t)g_ptpv1.out_ppb);
    n += 16;

    // 0x8000: clock sync quality -- |offset| and mean path delay, ns. This is
    // what DC's Sync indicator reads. Reported honestly: green is earned by the
    // servo, not by the report.
    int64_t off = g_ptpv1.offset_ns;  if (off < 0) off = -off;
    if (off > 0xFFFFFFFFLL) off = 0xFFFFFFFFLL;
    int64_t pd = g_ptpv1.mean_path_delay_ns;  if (pd < 0) pd = 0;
    if (pd > 0xFFFFFFFFLL) pd = 0xFFFFFFFFLL;
    put_u16(p, n, 36); put_u16(p, n + 2, 0x8000); put_u16(p, n + 4, 4);
    put_u16(p, n + 6, 4); put_u16(p, n + 8, s_seq); put_u16(p, n + 10, 0);
    put_u16(p, n + 12, 0x0010); put_u16(p, n + 14, 0);
    put_u16(p, n + 16, 1);      put_u16(p, n + 18, 0x0010);
    put_u32(p, n + 20, (uint32_t)off); put_u32(p, n + 24, (uint32_t)pd);
    put_u32(p, n + 28, 0); put_u32(p, n + 32, 0);
    n += 36;

    // 0x8002: per-channel peaks, one byte each, tx first then rx, padded to 4.
    // Zero: honest "no meter". RECEIVER: 0 tx, AP_NCH rx.
    {
        const uint16_t ntx = 0, nrx = AP_NCH, npk = ntx + nrx;
        const uint16_t pad = (uint16_t)((4u - (npk & 3u)) & 3u);
        put_u16(p, n, (uint16_t)(24 + npk + pad)); put_u16(p, n + 2, 0x8002);
        put_u16(p, n + 4, 4); put_u16(p, n + 6, (uint16_t)(12 + npk));
        put_u16(p, n + 8, s_seq); put_u16(p, n + 10, 0);
        put_u16(p, n + 12, ntx); put_u16(p, n + 14, 0);
        put_u16(p, n + 16, nrx); put_u16(p, n + 18, 0);
        put_u16(p, n + 20, 24);  put_u16(p, n + 22, 0);
        memset(p + n + 24, 0, npk + pad);
        n += 24 + npk + pad;
    }

    // 0x8003 / 0x8004, last so a mis-sized count can only spoil themselves.
    // The count is the RX FLOW CAPACITY (AM2: 2, A16R: 32 -- FPGA measured).
    const uint16_t nflows = AP_MAX_FLOWS;
    put_u16(p, n, (uint16_t)(24 + 4 * nflows)); put_u16(p, n + 2, 0x8003);
    put_u16(p, n + 4, 4); put_u16(p, n + 6, (uint16_t)(12 + 4 * nflows));
    put_u16(p, n + 8, s_seq); put_u16(p, n + 10, 0);
    put_u16(p, n + 12, nflows); put_u16(p, n + 14, 0);
    put_u16(p, n + 16, 0x0018); put_u16(p, n + 18, 0);
    put_u32(p, n + 20, rate_hz());
    memset(p + n + 24, 0, 4 * nflows);        // TODO: measured per-flow latency
    n += 24 + 4 * nflows;

    put_u16(p, n, (uint16_t)(20 + 4 * nflows)); put_u16(p, n + 2, 0x8004);
    put_u16(p, n + 4, 4); put_u16(p, n + 6, (uint16_t)(8 + 4 * nflows));
    put_u16(p, n + 8, s_seq); put_u16(p, n + 10, 0);
    put_u16(p, n + 12, nflows); put_u16(p, n + 14, 0);
    put_u16(p, n + 16, 0x0014); put_u16(p, n + 18, 0);
    memset(p + n + 20, 0, 4 * nflows);
    n += 20 + 4 * nflows;

    send_to(GRP_HEARTBEAT, AOIP_PORT_HEARTBEAT, n);
}

// ---------------------------------------------------------------------------
// 0x0060 board info -- "AoIP Model" in DC (board name at 0x0c and 0x38).
// c[0xbb] = 0x1f: left 0, the device is flooded with info requests.
// ---------------------------------------------------------------------------

static void send_device_info(const uint8_t ip[4], uint16_t port)
{
    static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x60, 0, 0, 0, 0};
    uint32_t n = put_hdr(s_buf, 0xFFFF, op);
    uint8_t *c = s_buf + n;
    memset(c, 0, 200);
    c[0] = 4; c[1] = 1; c[2] = 0; c[3] = 6;           // firmware version
    c[4] = 4; c[5] = 1; c[6] = 0; c[7] = 3;           // hardware version
    c[0x16] = 0x10;                                   // has manufacturer name
    c[0x23] = 2; c[0x27] = 1; c[0x28] = 1;
    c[0xbb] = 0x1f;
    put_fixed(c, 0x0c, 8,  "NSerAoI");
    put_fixed(c, 0x38, 16, "N-Series AoIP");          // == mDNS router_info
    send_to(ip, port, n + 200);
}

// 0x00c0 product info -- Manufacturer, Model Name, Product Version in DC.
static void send_product_info(const uint8_t ip[4], uint16_t port)
{
    static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0xc0, 0, 0, 0, 0};
    uint32_t n = put_hdr(s_buf, 0xFFFF, op);
    uint8_t *c = s_buf + n;
    memset(c, 0, 336);
    put_fixed(c, 0x00, 8,  "NSeries");
    put_fixed(c, 0x08, 8,  "DAC8");
    c[0x1c] = 0; c[0x1d] = 0; c[0x1e] = 1; c[0x1f] = 0;   // product 0.1.0
    put_fixed(c, 0x2c, 16, "N-Series");
    put_fixed(c, 0xac, 16, "N-Series DAC8");               // Model Name, <= 16 chars
    send_to(ip, port, n + 336);
}

// 0x0011 network info -- Primary Address and Link Speed. THIS GATES ROUTING:
// without an address DC cannot send ARC. Request is 0x13, reply is 0x11.
static void send_network_info(const uint8_t ip_dst[4], uint16_t port)
{
    static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x11, 0, 0, 0, 0};
    uint8_t ip[4], mask[4], gw[4];
    if (!our_ip(ip, mask, gw)) return;

    uint32_t n = put_hdr(s_buf, 0xFFFF, op);
    uint8_t *c = s_buf + n;
    uint32_t o = 0;
    static const uint8_t lead[6] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    memcpy(c + o, lead, 6); o += 6;
    put_u16(c, o, 100); o += 2;               // 100 Mbit/s -- IP101 on this board
    put_u16(c, o, 1);   o += 2;
    memcpy(c + o, eth_ts_mac(), 6); o += 6;
    memcpy(c + o, ip, 4);   o += 4;
    memcpy(c + o, mask, 4); o += 4;
    memcpy(c + o, gw, 4);   o += 4;
    memcpy(c + o, gw, 4);   o += 4;           // DNS: none; inferno repeats gw
    memset(c + o, 0, 32); c[o + 1] = 0x18; c[o + 3] = 0x30; o += 32;
    send_to(ip_dst, port, n + o);
}

// ---------------------------------------------------------------------------
// 0x0020 clock stats -- what makes DC show us as a PTPv1 FOLLOWER of the right
// leader instead of "PTPv2 Domain 0 / Priority 0/0". Byte-for-byte a DVS
// (DC's "Follower Only", which is what a slave-only receiver is), patched:
//   [2..4]  0x0003 locked / 0x0001 not
//   [8..12] frequency offset ppb
//   12/20/28  MAC+0000 of us / grandmaster / parent   (NOT EUI-64)
//   [40]    0x0009 = IEEE 1588 SLAVE, the "Primary v1 Multicast" column
//   120/128/136  EUI-64 of us / leader / leader
// Always multicast, and a reply ECHOES the request's seq and opcode (only
// byte 3 changes, 0x21 -> 0x20), or DC does not ingest it.
// ---------------------------------------------------------------------------

static const uint8_t CLOCK_STATS_TMPL[148] = {
    0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xf1, 0x6a,
    0xc8, 0xa3, 0x62, 0xeb, 0xf8, 0xc8, 0x00, 0x00, 0x00, 0x1d, 0xc1, 0x2d,
    0x4a, 0x18, 0x00, 0x00, 0x00, 0x1d, 0xc1, 0x2d, 0x4a, 0x18, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x34, 0x00, 0x09, 0x00, 0x00, 0x02, 0x34, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x0c,
    0x00, 0x78, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x68, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x09, 0x00, 0x07, 0x00, 0x01, 0x00, 0x07, 0x00, 0x98, 0x00, 0x04,
    0xc8, 0xa3, 0x62, 0xff, 0xfe, 0xeb, 0xf8, 0xc8, 0x00, 0x1d, 0xc1, 0xff,
    0xfe, 0x2d, 0x4a, 0x18, 0x00, 0x1d, 0xc1, 0xff, 0xfe, 0x2d, 0x4a, 0x18,
    0x00, 0x01, 0x00, 0x00,
};

static void send_clock_stats(const uint8_t *req)
{
    if (!g_ptpv1.have_master) return;

    static const uint8_t op_default[8] = {0x07, 0x2a, 0x00, 0x20, 0, 0, 0, 0};
    uint8_t op[8];
    uint16_t seq;
    if (req) {
        memcpy(op, req + 24, 8);
        op[3] = 0x20;
        seq = (uint16_t)((req[4] << 8) | req[5]);
    } else {
        memcpy(op, op_default, 8);
        seq = s_seq;
    }

    uint32_t n = put_hdr_seq(s_buf, 0xFFFF, op, seq);
    uint8_t *c = s_buf + n;
    memcpy(c, CLOCK_STATS_TMPL, sizeof(CLOCK_STATS_TMPL));

    c[2] = 0x00; c[3] = g_ptpv1.locked ? 0x03 : 0x01;
    put_u32(c, 8, (uint32_t)g_ptpv1.out_ppb);

    const uint8_t *mu = g_ptpv1.master_uuid;
    memcpy(c + 12, eth_ts_mac(), 6); c[18] = 0; c[19] = 0;
    memcpy(c + 20, mu, 6);           c[26] = 0; c[27] = 0;
    memcpy(c + 28, mu, 6);           c[34] = 0; c[35] = 0;

    uint8_t eui[8] = { mu[0], mu[1], mu[2], 0xFF, 0xFE, mu[3], mu[4], mu[5] };
    memcpy(c + 120, s_devid, 8);
    memcpy(c + 128, eui, 8);
    memcpy(c + 136, eui, 8);

    send_to(GRP_DEVINFO, AOIP_PORT_INFO, n + sizeof(CLOCK_STATS_TMPL));
}

// 0x0080 sample rates, 0x0082 encodings, 0x1009, 0x0084. Declares only what
// this device can do: ONE rate (the one the boot pin chose) and 24-bit.
static void send_caps(const uint8_t ip[4], uint16_t port)
{
    {
        static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x80, 0, 0, 0, 0};
        uint32_t n = put_hdr(s_buf, 0xFFFF, op);
        uint8_t *c = s_buf + n;
        put_u16(c, 0, 0x0018); put_u16(c, 2, 1);
        put_u32(c, 4, rate_hz()); put_u32(c, 8, 0);
        put_u16(c, 12, 0); put_u16(c, 14, 0);
        put_u32(c, 16, rate_hz());
        send_to(ip, port, n + 20);
    }
    {
        static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x82, 0, 0, 0, 0};
        uint32_t n = put_hdr(s_buf, 0xFFFF, op);
        uint8_t *c = s_buf + n;
        put_u16(c, 0, 0x0018); put_u16(c, 2, 1);
        put_u32(c, 4, 24); put_u32(c, 8, 0);
        put_u16(c, 12, 2); put_u16(c, 14, 0);
        put_u32(c, 16, 24);
        send_to(ip, port, n + 20);
    }
    {
        static const uint8_t op[8] = {0x07, 0x2a, 0x10, 0x09, 0, 0, 0, 0};
        uint32_t n = put_hdr(s_buf, 0xFFFF, op);
        memset(s_buf + n, 0, 16);
        send_to(ip, port, n + 16);
    }
    {
        // Byte-identical on the A16R, the AM2 and DVS.
        static const uint8_t op[8] = {0x07, 0x2a, 0x00, 0x84, 0, 0, 0, 0};
        static const uint8_t body[60] = {
            0x00,0x30,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
            0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x04,
        };
        uint32_t n = put_hdr(s_buf, 0xFFFF, op);
        memcpy(s_buf + n, body, sizeof(body));
        send_to(ip, port, n + sizeof(body));
    }
}

// ---------------------------------------------------------------------------
// 0x0102 channel-change event (inferno protocol/mcast.rs
// make_channel_change_notification): u16 mask length, then a bitmask of
// 0-based channel indices, LSB first.
// ---------------------------------------------------------------------------

void aoip_info_notify_rx_change(uint32_t mask)
{
    if (s_sock < 0 || !mask) return;
    static const uint8_t op[8] = {0x07, 0x2a, 0x01, 0x02, 0, 0, 0, 0};
    const uint32_t nbytes = (AP_NCH + 7) / 8;
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    uint32_t n = put_hdr(s_buf, 0xFFFF, op);
    put_u16(s_buf, n, (uint16_t)nbytes); n += 2;
    for (uint32_t i = 0; i < nbytes; i++) s_buf[n++] = (uint8_t)(mask >> (8 * i));
    send_to(GRP_DEVINFO, AOIP_PORT_INFO, n);
    xSemaphoreGive(s_tx_lock);
}

// ---------------------------------------------------------------------------
// Requests on 8700. DC's requests are UNICAST to us, so the console is the
// only place they can be seen -- unknown ones are logged once each.
// ---------------------------------------------------------------------------

static void info_rx(const uint8_t *req, int len, const struct sockaddr_in *from)
{
    if (len < MCAST_HDR_LEN) return;
    uint8_t src[4];
    memcpy(src, &from->sin_addr.s_addr, 4);
    uint16_t sport = ntohs(from->sin_port);

    uint8_t q = req[24 + 3];
    switch (q) {
    case 0x60: case 0x61: send_device_info(src, sport);  break;
    case 0xc0: case 0xc1: send_product_info(src, sport); break;
    case 0x13:            send_network_info(src, sport); break;
    case 0x21:            send_clock_stats(req);         break;
    case 0x81: case 0x83: case 0x85: send_caps(src, sport); break;
    default: {
        static uint64_t seen;
        uint64_t bit = 1ULL << (q & 63);
        if (!(seen & bit)) {
            seen |= bit;
            ESP_LOGW(TAG, "unhandled info query 0x%02x (op %02x%02x%02x%02x%02x%02x%02x%02x) from "
                     IPSTR, q, req[24], req[25], req[26], req[27], req[28], req[29],
                     req[30], req[31], src[0], src[1], src[2], src[3]);
        }
        break;
    }
    }
}

// ---------------------------------------------------------------------------
// CMC, UDP 8800: one opcode, 0x1001. DC's FIRST unicast to a new device -- with
// no reply it never proceeds to ARC.
// ---------------------------------------------------------------------------

static void cmc_task(void *arg)
{
    (void)arg;
    static uint8_t rx[256], tx[64];
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(AOIP_PORT_CMC),
                             .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (s < 0 || bind(s, (struct sockaddr *)&a, sizeof(a)) != 0) {
        ESP_LOGE(TAG, "cmc: cannot bind %d", AOIP_PORT_CMC);
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int n = recvfrom(s, rx, sizeof(rx), 0, (struct sockaddr *)&from, &flen);
        if (n < (int)DRR_HDR_LEN) continue;
        if (dw_rd16(rx + 8) != 0 || dw_rd16(rx + 6) != OP_CMC_DEVICE_ADVERTISEMENT) {
            ESP_LOGW(TAG, "cmc: unhandled opcode 0x%04x", dw_rd16(rx + 6));
            continue;
        }
        uint8_t ip[4];
        if (!our_ip(ip, NULL, NULL)) continue;

        // DeviceAdvertisement. Checked against the AM2's reply to DC's own
        // request bytes, replayed from this bench:
        //   0000 001dc1fffea1723c 0001 0000 a9fe3d72 21fc 0000
        memcpy(tx, rx, DRR_HDR_LEN);          // echo start code, seq, opcode1
        uint32_t o = DRR_HDR_LEN;
        dw_wr16(tx + o, 0);        o += 2;    // process id
        memcpy(tx + o, s_devid, 8); o += 8;
        dw_wr16(tx + o, 1);        o += 2;
        dw_wr16(tx + o, 0);        o += 2;
        memcpy(tx + o, ip, 4);     o += 4;
        dw_wr16(tx + o, AOIP_PORT_INFO_REQ); o += 2;
        dw_wr16(tx + o, 0);        o += 2;
        dw_wr16(tx + 2, (uint16_t)o);
        dw_wr16(tx + 8, DRR_CODE_OK);
        sendto(s, tx, o, 0, (struct sockaddr *)&from, flen);
    }
}

static void info_rx_task(void *arg)
{
    (void)arg;
    static uint8_t rx[512];
    for (;;) {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int n = recvfrom(s_sock, rx, sizeof(rx), 0, (struct sockaddr *)&from, &flen);
        if (n <= 0) continue;
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        info_rx(rx, n, &from);
        xSemaphoreGive(s_tx_lock);
    }
}

static void announce_task(void *arg)
{
    (void)arg;
    int64_t next_announce = 0;
    for (;;) {
        uint8_t ip[4];
        if (our_ip(ip, NULL, NULL)) {
            xSemaphoreTake(s_tx_lock, portMAX_DELAY);
            int64_t now = esp_timer_get_time() / 1000;
            if (now >= next_announce) {
                send_device_info(GRP_DEVINFO, AOIP_PORT_INFO);
                send_product_info(GRP_DEVINFO, AOIP_PORT_INFO);
                send_network_info(GRP_DEVINFO, AOIP_PORT_INFO);
                // Unsolicited too: DC asks for clock stats only on a manual
                // refresh, so a boot-time "not locked" would otherwise stick.
                send_clock_stats(NULL);
                send_caps(GRP_DEVINFO, AOIP_PORT_INFO);
                next_announce = now + INFO_ANNOUNCE_MS;
            }
            send_heartbeat();
            xSemaphoreGive(s_tx_lock);
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}

esp_err_t aoip_info_start(void)
{
    const uint8_t *m = eth_ts_mac();
    uint8_t id[8] = { m[0], m[1], m[2], 0xFF, 0xFE, m[3], m[4], m[5] };
    memcpy(s_devid, id, 8);

    s_tx_lock = xSemaphoreCreateMutex();
    if (!s_tx_lock) return ESP_ERR_NO_MEM;

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(AOIP_PORT_INFO_REQ),
                             .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (s_sock < 0 || bind(s_sock, (struct sockaddr *)&a, sizeof(a)) != 0) {
        ESP_LOGE(TAG, "cannot bind %d", AOIP_PORT_INFO_REQ);
        return ESP_FAIL;
    }
    uint8_t ttl = 1;
    setsockopt(s_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    // 224.0.0.231 / .233 are already in the EMAC filter from main.c. Do NOT add
    // them again: the filter has few slots, and duplicates filled it so that
    // lwIP's own IGMP join failed ("failed to add MAC filter") on first bench.

    xTaskCreatePinnedToCore(cmc_task, "cmc", 3072, NULL,
                            AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    xTaskCreatePinnedToCore(info_rx_task, "info_rx", 4096, NULL,
                            AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    xTaskCreatePinnedToCore(announce_task, "info_tx", 4096, NULL,
                            AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    ESP_LOGI(TAG, "cmc:%d  info:%d -> 224.0.0.231:%d, heartbeat -> 224.0.0.233:%d  id %02x%02x%02x%02x%02x%02x%02x%02x",
             AOIP_PORT_CMC, AOIP_PORT_INFO_REQ, AOIP_PORT_INFO, AOIP_PORT_HEARTBEAT,
             id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
    return ESP_OK;
}
