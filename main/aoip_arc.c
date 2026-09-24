// ARC server, UDP 4440.
//
// Handlers and constants are ported from FPGA project firmware/aoip_arc.c,
// which was iterated against AoIP Controller, a RedNet AM2, an A16R and DVS.
// That file's comments are the reason for most of what is here; the ones that
// matter for a receiver are repeated inline. The shape differences are:
// no transmit channels, AP_NCH receive channels, and 0x3010 drives a real
// subscriber instead of only being remembered.

#include "aoip_arc.h"
#include "aoip_msg.h"
#include "aoip_wire.h"
#include "aoip_mdns.h"
#include "subscriber.h"
#include "media_clock.h"
#include "rate.h"
#include "app_config.h"
#include "lwip/sockets.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "arc";

#define PACKET_SIZE_SOFT_LIMIT  800
#define CODE_UNSUPPORTED        0x0022   // what real hardware returns

// ---------------------------------------------------------------------------
// 0x1100 / 0x1102 -- device property tables.
//
// Byte-exact replay of the RedNet AM2 on this bench (FPGA aoip_arc.c). As
// zeros (inferno's answer) DC misclassified the FPGA device as PTPv2 domain 0.
// The AM2 is a RECEIVER and DC's "Follower", so it is the right template here.
//
// 0x1102: u16 count, then count x (u16 key, u16 type).
// 0x1100: u16 (flags<<8 | count), then count x (u16 key, u16 value); keys with
//         bit 15 set hold an OFFSET (absolute, includes the 10-byte header)
//         into the data blob after the table. The latency capability lives
//         there:
//     0x8205 / 0x8301   current latency, ns
//     0x8306            MINIMUM supported latency -- what DC offers down to
// ---------------------------------------------------------------------------

static uint8_t arc_1100_body[202] = {
    0x24, 0x1f, 0x80, 0x20, 0x00, 0x9c, 0x80, 0x21, 0x00, 0xa0, 0x00, 0x22,
    0x00, 0x01, 0x00, 0x23, 0x00, 0x18, 0x00, 0x24, 0x00, 0x01, 0x80, 0x60,
    0x00, 0xb0, 0x00, 0x62, 0x00, 0x01, 0x00, 0x63, 0x00, 0x01, 0x02, 0x01,
    0x00, 0x01, 0x82, 0x04, 0x00, 0xb4, 0x82, 0x05, 0x00, 0xb8, 0x02, 0x0a,
    0x00, 0x00, 0x02, 0x0b, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x02, 0x11,
    0x00, 0x00, 0x02, 0x12, 0x00, 0x30, 0x02, 0x13, 0x00, 0x00, 0x02, 0x14,
    0x00, 0x00, 0x02, 0x22, 0x13, 0x8c, 0x83, 0x01, 0x00, 0xbc, 0x83, 0x06,
    0x00, 0xc0, 0x83, 0x02, 0x00, 0xc4, 0x83, 0x21, 0x00, 0xc8, 0x03, 0x10,
    0x00, 0x10, 0x03, 0x11, 0x00, 0x10, 0x03, 0x12, 0x00, 0x30, 0x03, 0x03,
    0x00, 0x02, 0x83, 0xf0, 0x00, 0xcc, 0x06, 0x01, 0x00, 0x00, 0x03, 0x09,
    0x00, 0x03, 0x02, 0x09, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xbb, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0x45,
    0x00, 0x00, 0x00, 0x0f, 0x42, 0x40, 0x00, 0x07, 0xa1, 0x20, 0x00, 0x07,
    0xa1, 0x20, 0x00, 0x03, 0xd0, 0x90, 0x01, 0x35, 0xf1, 0xb4, 0x00, 0x1e,
    0x84, 0x80, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t arc_1102_body[126] = {
    0x00, 0x1f, 0x80, 0x20, 0x00, 0x01, 0x80, 0x21, 0x00, 0x03, 0x00, 0x22,
    0x00, 0x03, 0x00, 0x23, 0x00, 0x03, 0x00, 0x24, 0x00, 0x01, 0x80, 0x60,
    0x00, 0x03, 0x00, 0x62, 0x00, 0x03, 0x00, 0x63, 0x00, 0x01, 0x02, 0x01,
    0x00, 0x03, 0x82, 0x04, 0x00, 0x03, 0x82, 0x05, 0x00, 0x03, 0x02, 0x0a,
    0x00, 0x01, 0x02, 0x0b, 0x00, 0x01, 0x02, 0x10, 0x00, 0x03, 0x02, 0x11,
    0x00, 0x03, 0x02, 0x12, 0x00, 0x03, 0x02, 0x13, 0x00, 0x01, 0x02, 0x14,
    0x00, 0x01, 0x02, 0x22, 0x00, 0x03, 0x83, 0x01, 0x00, 0x03, 0x83, 0x06,
    0x00, 0x01, 0x83, 0x02, 0x00, 0x01, 0x83, 0x21, 0x00, 0x01, 0x03, 0x10,
    0x00, 0x01, 0x03, 0x11, 0x00, 0x01, 0x03, 0x12, 0x00, 0x01, 0x03, 0x03,
    0x00, 0x03, 0x83, 0xf0, 0x00, 0x01, 0x06, 0x01, 0x00, 0x01, 0x03, 0x09,
    0x00, 0x01, 0x02, 0x09, 0x00, 0x01,
};

// Write a u32 into the 0x1100 data blob at the offset an offset-key holds.
static void arc_1100_patch_u32(uint16_t key, uint32_t v)
{
    uint32_t count = arc_1100_body[1];
    for (uint32_t i = 0; i < count; i++) {
        uint32_t at = 2 + i * 4;
        if (dw_rd16(arc_1100_body + at) != key) continue;
        uint32_t off = dw_rd16(arc_1100_body + at + 2);
        if (off < DRR_HDR_LEN) return;
        uint32_t bi = off - DRR_HDR_LEN;
        if (bi + 4 > sizeof(arc_1100_body)) return;
        dw_wr32(arc_1100_body + bi, v);
        return;
    }
}

// ---------------------------------------------------------------------------

static uint16_t put_common_descriptor(aoip_msg_t *m)
{
    uint16_t off = (uint16_t)m->len;
    aoip_msg_u32(m, rate_hz());
    aoip_msg_u8 (m, 1);
    aoip_msg_u8 (m, 1);
    aoip_msg_u16(m, AP_BITS_PER_SAMPLE);
    aoip_msg_u16(m, 0x400);
    aoip_msg_u16(m, AP_BITS_PER_SAMPLE);
    aoip_msg_u16(m, AP_BITS_PER_SAMPLE);
    aoip_msg_u16(m, 0xE);                    // pcm_type, as the FPGA sends
    return off;
}

// Paginated body: [0] space u8, [1] actual u8, [2..] space*item_size zeros,
// then strings. Request start index (1-based) is at content[2..4].
typedef struct { uint32_t items_at, item_size, space, actual; } page_t;

static void page_begin(aoip_msg_t *m, page_t *pg, uint32_t item_size, uint32_t space)
{
    if (space > 255) space = 255;
    pg->item_size = item_size; pg->space = space; pg->actual = 0;
    aoip_msg_u8(m, (uint8_t)space);
    aoip_msg_u8(m, 0);
    pg->items_at = m->len;
    aoip_msg_zeros(m, item_size * space);
}
static uint8_t *page_slot(aoip_msg_t *m, page_t *pg)
{
    return m->buf + pg->items_at + pg->actual * pg->item_size;
}
static void page_end(aoip_msg_t *m, page_t *pg)
{
    m->buf[pg->items_at - 1] = (uint8_t)pg->actual;   // THE ACTUAL COUNT
}

// Copy a NUL-terminated string at absolute packet offset `off`, bounded.
static void req_str(char *dst, uint32_t cap, const uint8_t *pkt, uint32_t len, uint16_t off)
{
    dst[0] = 0;
    if (off < DRR_HDR_LEN || off >= len) return;
    uint32_t i = 0;
    while (off + i < len && pkt[off + i] && i + 1 < cap) { dst[i] = (char)pkt[off + i]; i++; }
    dst[i] = 0;
}

// ---------------------------------------------------------------------------
// Request log: every DISTINCT opcode, with its body, once. DC polls, so a
// per-request budget is exhausted in seconds and hides everything asked later
// (FPGA aoip_arc.c). Also logs every subscription and latency set.
// ---------------------------------------------------------------------------

static void log_first(uint16_t op, const struct sockaddr_in *from,
                      const uint8_t *p, int n, uint16_t code)
{
    static uint16_t seen[48];
    static int nseen;
    for (int i = 0; i < nseen; i++) if (seen[i] == op) return;
    if (nseen < (int)(sizeof(seen) / sizeof(seen[0]))) seen[nseen++] = op;

    char hex[2 * 40 + 1];
    int m = n < 40 ? n : 40;
    for (int i = 0; i < m; i++) sprintf(hex + 2 * i, "%02x", p[i]);
    hex[2 * m] = 0;
    ESP_LOGI(TAG, "op 0x%04x from " IPSTR " -> code 0x%04x  req %s%s", op,
             IP2STR((const esp_ip4_addr_t *)&from->sin_addr.s_addr), code,
             hex, n > 40 ? "..." : "");
}

// ---------------------------------------------------------------------------

static uint32_t handle(const uint8_t *req, uint32_t len, uint8_t *buf, uint16_t *code_out)
{
    uint16_t opcode = aoip_req_opcode1(req);
    const uint8_t *content = req + AOIP_HDR_LEN;
    uint32_t clen = len - AOIP_HDR_LEN;

    aoip_msg_t m;
    aoip_msg_begin(&m, buf, req);
    uint16_t code = AOIP_CODE_OK;

    switch (opcode) {

    case ARC_OP_CHANNEL_COUNTS: {
        // Layout per inferno channels_and_flows_count::Response; capability
        // bytes are the AM2's (a receiver), read off this bench:
        //   0d f9 0000 0002 0000 0000 0008 0000 0002 0000 0001 0001 + 12 zero
        aoip_msg_u8 (&m, 0x0d);
        aoip_msg_u8 (&m, 0xf9);
        aoip_msg_u16(&m, 0);                 // tx channels
        aoip_msg_u16(&m, AP_NCH);            // rx channels
        aoip_msg_u16(&m, 0);
        aoip_msg_u16(&m, 0);                 // max channels in (tx) flow
        aoip_msg_u16(&m, 8);
        aoip_msg_u16(&m, 0);                 // max tx flows
        aoip_msg_u16(&m, AP_MAX_FLOWS);      // max rx flows
        aoip_msg_u16(&m, 0);
        aoip_msg_u16(&m, 1);
        aoip_msg_u16(&m, 1);
        aoip_msg_zeros(&m, 12);
        break;
    }

    case ARC_OP_GET_DEVICE_NAME:
        aoip_msg_str(&m, aoip_mdns_hostname());
        break;

    case ARC_OP_GET_DEVICE_NAMES: {
        // The layout real devices emit (A16R and AM2 byte-identical in
        // structure), NOT inferno's -- FPGA aoip_arc.c. 48-byte header, 2 pad,
        // fixed 32-byte friendly and factory fields, then board and revision.
        // DC parses this for latency detail; the wrong shape gives "Cannot
        // retrieve Device Latency".
        const char *name = aoip_mdns_hostname();
        uint32_t head = m.len;
        aoip_msg_zeros(&m, 48);
        aoip_msg_zeros(&m, 2);
        uint16_t friendly = (uint16_t)m.len;
        uint32_t nl = strnlen(name, 31);
        aoip_msg_bytes(&m, name, nl); aoip_msg_zeros(&m, 32 - nl);
        uint16_t factory = (uint16_t)m.len;
        aoip_msg_bytes(&m, name, nl); aoip_msg_zeros(&m, 32 - nl);
        uint16_t board    = aoip_msg_str(&m, "N-Series AoIP");
        uint16_t revision = aoip_msg_str(&m, ":705");

        aoip_msg_patch_u16(&m, head +  0, 0x001c);
        aoip_msg_patch_u16(&m, head +  2, 0x001c);
        aoip_msg_patch_u16(&m, head +  4, 0x0028);
        aoip_msg_patch_u16(&m, head +  6, board);
        aoip_msg_patch_u16(&m, head +  8, revision);
        aoip_msg_patch_u16(&m, head + 18, 0x0500);
        aoip_msg_patch_u16(&m, head + 20, friendly);
        aoip_msg_patch_u16(&m, head + 22, factory);
        aoip_msg_patch_u16(&m, head + 24, friendly);
        aoip_msg_patch_u16(&m, head + 30, 0x0a0a);
        aoip_msg_patch_u16(&m, head + 34, 0x0404);   // router 4.4.0 == mDNS
        aoip_msg_patch_u16(&m, head + 38, 0x2809);   // arcp 2.8.9   == mDNS
        aoip_msg_patch_u16(&m, head + 40, 0x0204);
        aoip_msg_patch_u16(&m, head + 42, 0x1200);
        aoip_msg_patch_u16(&m, head + 44, 0x1004);
        break;
    }

    case ARC_OP_GET_TX_CHANNELS:        // 0x2000
    case 0x2010:                        // tx friendly names
    case 0x2200:                        // tx flows
    case 0x2204:                        // tx flow detail -- MUST be OK+empty, an
                                        // error made DC loop at ~1 kHz (FPGA)
    case 0x3200: {                      // rx flows. TODO: describe active flows
        page_t pg;
        page_begin(&m, &pg, 8, 0);
        page_end(&m, &pg);
        break;
    }

    case ARC_OP_GET_RX_CHANNELS: {
        // 20-byte items (the tx descriptor is 8; mirroring it crashed DC).
        uint16_t start = (clen >= 4) ? aoip_req_u16(content, 2) : 1;
        if (start == 0) start = 1;

        page_t pg;
        page_begin(&m, &pg, 20, AP_NCH);
        uint16_t common = 0;
        uint16_t idx = start;
        for (; idx <= AP_NCH && pg.actual < pg.space; idx++) {
            if (!common) common = put_common_descriptor(&m);
            const rx_channel_t *ch = subscriber_channel((uint8_t)(idx - 1));
            uint16_t name_off = aoip_msg_str(&m, ch->friendly);
            uint16_t tx_off = 0, host_off = 0;
            if (ch->tx_channel[0]) {
                tx_off   = aoip_msg_str(&m, ch->tx_channel);
                host_off = ch->tx_device[0] ? aoip_msg_str(&m, ch->tx_device) : 0;
            }
            uint8_t *slot = page_slot(&m, &pg);
            dw_wr16(slot + 0,  idx);
            dw_wr16(slot + 2,  6);
            dw_wr16(slot + 4,  common);
            dw_wr16(slot + 6,  tx_off);
            dw_wr16(slot + 8,  host_off);
            dw_wr16(slot + 10, name_off);
            dw_wr32(slot + 12, ch->status);
            dw_wr32(slot + 16, 0);
            pg.actual++;
            if (m.len >= PACKET_SIZE_SOFT_LIMIT) { idx++; break; }
        }
        page_end(&m, &pg);
        if (idx <= AP_NCH) code = AOIP_CODE_MORE;
        break;
    }

    case ARC_OP_RENAME_RX_CHANNELS: {   // 0x3001: u8 space, u8 actual, {u16 ch, u16 name_off}
        uint8_t n = clen >= 2 ? content[1] : 0;
        bool any = false;
        for (uint8_t i = 0; i < n; i++) {
            uint32_t at = 2 + (uint32_t)i * 4;
            if (at + 4 > clen) break;
            uint16_t ch = aoip_req_u16(content, at);
            char nm[32];
            req_str(nm, sizeof(nm), req, len, aoip_req_u16(content, at + 2));
            if (ch >= 1 && ch <= AP_NCH && nm[0]) {
                subscriber_rename((uint8_t)(ch - 1), nm);
                ESP_LOGI(TAG, "rename Rx%u -> '%s'", ch, nm);
                any = true;
            }
        }
        if (!any) code = CODE_UNSUPPORTED;
        break;
    }

    case ARC_OP_SET_SUBSCRIPTIONS: {
        // Captured from DC on the FPGA bench:
        //   02 01 | 0001 0034 0037 | ... "01" 00 "RedNetA16R" 00
        //   space actual | { our rx ch (1-based), tx chan off, tx host off }
        // Empty names = unsubscribe. Answer OK with no content (as DVS does).
        uint8_t n = clen >= 2 ? content[1] : 0;
        for (uint8_t i = 0; i < n; i++) {
            uint32_t at = 2 + (uint32_t)i * 6;
            if (at + 6 > clen) break;
            uint16_t ch = aoip_req_u16(content, at);
            char nm[32], host[32];
            req_str(nm,   sizeof(nm),   req, len, aoip_req_u16(content, at + 2));
            req_str(host, sizeof(host), req, len, aoip_req_u16(content, at + 4));
            if (ch >= 1 && ch <= AP_NCH) {
                ESP_LOGI(TAG, "subscribe Rx%u <- '%s'@'%s'", ch, nm, host);
                subscriber_set((uint8_t)(ch - 1), nm, host);
            }
        }
        break;
    }

    case 0x3014: {
        // Unsubscribe (network-audio-controller): content[4..6] = local channel.
        if (clen >= 6) {
            uint16_t ch = aoip_req_u16(content, 4);
            if (ch >= 1 && ch <= AP_NCH) subscriber_set((uint8_t)(ch - 1), "", "");
        }
        break;
    }

    case 0x3300:
        // "necessary to avoid 'clock domain mismatch' error in DC" (inferno);
        // the AM2 on this bench answers exactly this.
        aoip_msg_u16(&m, 0x3800); aoip_msg_u16(&m, 0x38fd);
        aoip_msg_u16(&m, 0x38fe); aoip_msg_u16(&m, 0x38ff);
        break;

    case 0x1100: {
        // Current latency tracks the setting; the MINIMUM is ours, not the
        // AM2's 1 ms -- this receiver's floor is DMA depth + margin, so DC must
        // not offer below it.
        uint32_t cur_ns = mclk_get_latency_us() * 1000u;
        uint32_t min_ns = rate_get()->latency_min_us * 1000u;
        arc_1100_patch_u32(0x8205, cur_ns);
        arc_1100_patch_u32(0x8301, cur_ns);
        arc_1100_patch_u32(0x8306, min_ns);
        aoip_msg_bytes(&m, arc_1100_body, sizeof(arc_1100_body));
        break;
    }

    case 0x1102:
        aoip_msg_bytes(&m, arc_1102_body, sizeof(arc_1102_body));
        break;

    case ARC_OP_SET_LATENCY: {
        // 0x1101, found by mirroring DC's Latency tab on the FPGA bench:
        // 30 bytes, the latency in NANOSECONDS twice at [22..26] and [26..30].
        if (clen < 30) { code = CODE_UNSUPPORTED; break; }
        uint32_t v1 = dw_rd32(content + 22), v2 = dw_rd32(content + 26);
        if (v1 != v2 || v1 < 100000u || v1 > 40000000u) {
            ESP_LOGW(TAG, "0x1101 latency rejected: %u / %u", (unsigned)v1, (unsigned)v2);
            code = CODE_UNSUPPORTED;
            break;
        }
        mclk_set_latency_us(v1 / 1000u);
        ESP_LOGI(TAG, "latency set to %u us (now %u us)", (unsigned)(v1 / 1000u),
                 (unsigned)mclk_get_latency_us());
        aoip_msg_bytes(&m, content, clen);       // echo, as the FPGA does
        break;
    }

    case 0x2320:
    case 0x4100:
        code = 0x30;
        break;

    case 0x2032:
        aoip_msg_u16(&m, 0);                     // A16R and AM2 both answer this
        break;

    default:
        // Every real device replies to every request; 0x22 is what they send
        // for an unsupported opcode.
        code = CODE_UNSUPPORTED;
        break;
    }

    *code_out = code;
    return aoip_msg_finish(&m, code);
}

static void arc_task(void *arg)
{
    (void)arg;
    static uint8_t rx[1024];
    static uint8_t tx[1024];

    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(AOIP_PORT_ARC),
    };
    if (s < 0 || bind(s, (struct sockaddr *)&a, sizeof(a)) != 0) {
        ESP_LOGE(TAG, "cannot bind %d", AOIP_PORT_ARC);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "listening on %d", AOIP_PORT_ARC);

    for (;;) {
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        int n = recvfrom(s, rx, sizeof(rx), 0, (struct sockaddr *)&from, &flen);
        if (n < (int)AOIP_HDR_LEN) continue;
        if (aoip_req_opcode2(rx) != 0) continue;      // not a request

        uint16_t code;
        uint32_t rlen = handle(rx, (uint32_t)n, tx, &code);
        log_first(aoip_req_opcode1(rx), &from, rx, n, code);
        sendto(s, tx, rlen, 0, (struct sockaddr *)&from, flen);
    }
}

esp_err_t aoip_arc_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(arc_task, "arc", 6144, NULL,
                                            AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
