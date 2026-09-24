#include "eth_ts.h"
#include "app_config.h"
#include "aoip_wire.h"
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "soc/soc_caps.h"
#if SOC_EMAC_IEEE1588V2_SUPPORTED
#include "soc/emac_ptp_struct.h"
#endif
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "eth_ts";

static esp_eth_handle_t  s_eth;
static esp_eth_mac_t    *s_mac;
static esp_eth_phy_t    *s_phy;
static esp_netif_t      *s_netif;
static uint8_t           s_our_mac[6];
static volatile bool     s_link_up;
static eth_ts_ptp_cb_t   s_ptp_cb;
static int32_t           s_rate_ppb;

// ---------------------------------------------------------------------------
// RX path: lift PTPv1 out, pass everything else to lwIP
// ---------------------------------------------------------------------------
//
// This hook replaces the default stack input so that a frame can be examined
// WITH its hardware timestamp before lwIP ever sees it. Audio does not come
// through here -- it goes to a normal UDP socket -- because audio does not need
// a receive timestamp. Only PTP does.

static inline bool is_ptpv1_frame(const uint8_t *f, uint32_t len,
                                  const uint8_t **payload, uint32_t *plen,
                                  uint16_t *dport)
{
    if (len < 14 + 20 + 8) return false;
    if (dw_rd16(f + 12) != 0x0800) return false;          // IPv4
    const uint8_t *ip = f + 14;
    if ((ip[0] >> 4) != 4) return false;
    uint32_t ihl = (ip[0] & 0x0F) * 4;
    if (ip[9] != 17) return false;                        // UDP
    uint32_t dst = dw_rd32(ip + 16);
    if (dst != PTP1_GROUP_IP && dst != (PTP1_GROUP_IP + 1)) return false;
    const uint8_t *udp = ip + ihl;
    if (udp + 8 > f + len) return false;
    uint16_t dp = dw_rd16(udp + 2);
    if (dp != PTP1_EVENT_PORT && dp != PTP1_GENERAL_PORT) return false;
    uint32_t ulen = dw_rd16(udp + 4);
    if (ulen < 8) return false;
    *payload = udp + 8;
    *plen    = ulen - 8;
    *dport   = dp;
    return true;
}

// IDF 5.5 (esp_eth_mac_esp.c, emac_esp32_rx_task): `info` is an
// eth_mac_time_t*. When the descriptor carries no stamp the driver writes
// 0 s / 0 ns rather than signalling it, so an all-zero stamp is "none".
// The PTP clock is initialised to 0 and a real stamp can be 0/0 only in the
// first nanosecond after enable, which is not worth distinguishing.
static bool extract_rx_ts(void *info, eth_ts_time_t *out)
{
    if (!info) return false;
    const eth_mac_time_t *t = (const eth_mac_time_t *)info;
    out->seconds     = t->seconds;
    out->nanoseconds = t->nanoseconds;
    return t->seconds != 0 || t->nanoseconds != 0;
}

static volatile uint32_t s_rx_frames;      // every frame the EMAC delivered
static uint32_t s_rx_kicks, s_rx_restarts, s_phy_resets;
static uint32_t s_rx_dma_state, s_rx_missed, s_rx_fifo_ovf;

static esp_err_t stack_input_info(esp_eth_handle_t eth, uint8_t *buffer,
                                  uint32_t length, void *priv, void *info)
{
    (void)eth;
    s_rx_frames++;
    const uint8_t *payload; uint32_t plen; uint16_t dport;
    if (s_ptp_cb && is_ptpv1_frame(buffer, length, &payload, &plen, &dport)) {
        eth_ts_ptp_frame_t f;
        f.len = plen > sizeof(f.payload) ? sizeof(f.payload) : plen;
        memcpy(f.payload, payload, f.len);
        f.dst_port = dport;
        f.ts_valid = extract_rx_ts(info, &f.ts);
        s_ptp_cb(&f);
        // Fall through: lwIP is welcome to it too. Nothing there listens on
        // 319/320, so this costs one socket lookup and keeps the path simple.
    }
    return esp_netif_receive((esp_netif_t *)priv, buffer, length, NULL);
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

void eth_ts_log_eee(void);

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        s_link_up = true;
        ESP_LOGI(TAG, "link up");
        eth_ts_log_eee();
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        s_link_up = false;
        ESP_LOGW(TAG, "link down");
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

esp_err_t eth_ts_init(void)
{
    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    // EMAC RECEIVE TASK ABOVE EVERYTHING. IDF's default is 15, unpinned --
    // below the I2S (23), audio RX (22) and PTP (21) tasks on core 0. Starved
    // during a burst (two flows overlapping in a make-before-break repatch),
    // the RX DMA ran out of descriptors and suspended, and since IDF wakes
    // this task only on "frame received", receive never restarted: bench, RX
    // dead for good while TX (heartbeats) carried on. It only copies frames
    // out; running it first costs nothing.
    mac_cfg.rx_task_prio = 24;
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr    = AP_ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = AP_PIN_ETH_PHY_RST;

    eth_esp32_emac_config_t esp32_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_cfg.smi_gpio.mdc_num  = AP_PIN_ETH_MDC;
    esp32_cfg.smi_gpio.mdio_num = AP_PIN_ETH_MDIO;
    // EXTERNAL 50 MHz into GPIO50. If this were EMAC_CLK_OUT the RMII clock
    // would be generated from the internal APLL, the APLL would be unavailable
    // as the media clock, and mclk_hw.c would have nothing to trim. The
    // Waveshare ESP32-P4-ETH wires an external oscillator, which is why this
    // design is possible on this board.
    esp32_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    // On the P4 this is set here, not in Kconfig: CONFIG_ETH_RMII_CLK_* is
    // ESP32-only in IDF 5.5 and silently absent on this target.
    esp32_cfg.clock_config.rmii.clock_gpio = (emac_rmii_clock_gpio_t)AP_PIN_ETH_REF_CLK;

    s_mac = esp_eth_mac_new_esp32(&esp32_cfg, &mac_cfg);
    if (!s_mac) return ESP_FAIL;
    s_phy = esp_eth_phy_new_ip101(&phy_cfg);
    if (!s_phy) return ESP_FAIL;

    esp_eth_config_t cfg = ETH_DEFAULT_CONFIG(s_mac, s_phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&cfg, &s_eth));

    ESP_ERROR_CHECK(esp_read_mac(s_our_mac, ESP_MAC_ETH));
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth, ETH_CMD_S_MAC_ADDR, s_our_mac));

    // --- IEEE 1588 ---------------------------------------------------------
    //
    // IDF 5.5 has no Kconfig switch for this; the capability is
    // SOC_EMAC_IEEE1588V2_SUPPORTED and the API is esp_eth_ioctl().
    //
    // Stamping ALL frames is the part that matters for AoIP. IDF's
    // emac_hal_ptp_start() enables only PTP-over-Ethernet (L2) and PTPv2
    // parsing, and never sets en_proc_ptp_ipv4_udp or en_ts4all -- so as
    // shipped, PTPv1 over UDP 319/320 is never stamped and every Sync would
    // land in ptp_no_hw_ts. IDF exposes no ioctl for it, so we set TSENALL in
    // the register directly, after the driver has finished its own setup.
#if SOC_EMAC_IEEE1588V2_SUPPORTED
    {
        bool on = true;
        esp_err_t err = esp_eth_ioctl(s_eth, ETH_MAC_ESP_CMD_PTP_ENABLE, &on);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "PTP timestamping unavailable (%s). Without hardware "
                          "timestamps this device cannot hold a media clock.",
                     esp_err_to_name(err));
            return err;
        }
        // Stamp ONLY PTPv1 event messages over UDP/IPv4 -- NOT every frame.
        //
        // This used to set en_ts4all, which stamps all frames, 1600+ audio
        // packets a second included. Under the bursts of a flow change the
        // MTL RX FIFO read controller hung in its "reading frame status /
        // timestamp" state (EMACDEBUG RRCSTS = 2, RXFSTS = 3: FIFO full), the
        // FIFO filled, and every later frame was dropped before the DMA --
        // receive dead for good, TX alive, and no EMAC or PHY restart
        // recovering it. The classifier below selects Sync (slave, event
        // messages, IEEE 1588-2002 format) and stamps a few frames a second.
        typeof(EMAC_PTP.timestamp_ctrl) tc = EMAC_PTP.timestamp_ctrl;
        tc.en_ts4all                    = 0;
        tc.en_ptp_pkg_proc_ver2_fmt     = 0;     // PTPv1 (1588-2002) messages
        tc.en_proc_ptp_ether_frm        = 0;     // not PTP-over-Ethernet
        tc.en_proc_ptp_ipv6_udp         = 0;
        tc.en_proc_ptp_ipv4_udp         = 1;     // PTP over UDP/IPv4, ports 319/320
        tc.en_ts_snap_event_msg         = 1;     // event messages only
        tc.en_snap_msg_relevant_master  = 0;     // slave: stamp Sync
        tc.sel_snap_type                = 0;
        EMAC_PTP.timestamp_ctrl = tc;
        ESP_LOGI(TAG, "IEEE 1588 timestamping enabled: PTPv1 Sync over UDP/IPv4 only");
    }
#else
#error "This target's EMAC has no IEEE 1588 unit -- a software timestamp is not good enough for a media clock."
#endif

    // 802.3x flow control: OFF.
    //
    // It was on, because the AES67 work on this chip measured multicast frame
    // loss without it. But with IDF's hardware flow control the MAC sends
    // PAUSE whenever its RX FIFO fills, and a receive stall during a burst
    // (flow changes: two flows overlapping, a flow stopped while packets are
    // still in flight) became a DEADLOCK: the MAC kept pausing the switch, the
    // switch forwarded nothing to this port, nothing drained the FIFO. On the
    // bench every patch change could kill receive for good -- transmit still
    // working, and no EMAC or PHY reset on our side recovering it. Without
    // flow control an overload drops frames instead, which the jitter buffer
    // and the flow liveness check handle.
    bool fc = false;
    esp_eth_ioctl(s_eth, ETH_CMD_S_FLOW_CTRL, &fc);

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_netif = esp_netif_new(&netif_cfg);
    ESP_ERROR_CHECK(esp_netif_attach(s_netif, esp_eth_new_netif_glue(s_eth)));

    // Take over the input path so PTP frames can be seen with their timestamps.
    ESP_ERROR_CHECK(esp_eth_update_input_path_info(s_eth, stack_input_info, s_netif));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               eth_event_handler, NULL));
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// ENERGY-EFFICIENT ETHERNET OFF.
//
// With EEE (802.3az) the link drops into low-power idle between frames and the
// next frame pays a ~30 us wake-up. On a link carrying only PTP that is every
// Sync; once audio flows at 1500 pps the link never sleeps and the penalty
// vanishes. Bench trace: t2 - t1 fell by 36 us the moment the flow came up,
// the boot-time path delay (39 us, measured idle) no longer matched, lock was
// lost -- and before the servo fix, the kick from it froze the servo into the
// "71 s step". A PTP slave wants a link whose delay does not depend on load.
//
// MMD 7 register 60 is our EEE advertisement; 61 is the partner's. Clause-22
// indirect access through registers 13 (control) and 14 (address/data).
// ---------------------------------------------------------------------------

static esp_err_t phy_rd(uint32_t reg, uint32_t *v)
{
    esp_eth_phy_reg_rw_data_t d = { .reg_addr = reg, .reg_value_p = v };
    return esp_eth_ioctl(s_eth, ETH_CMD_READ_PHY_REG, &d);
}
static esp_err_t phy_wr(uint32_t reg, uint32_t v)
{
    esp_eth_phy_reg_rw_data_t d = { .reg_addr = reg, .reg_value_p = &v };
    return esp_eth_ioctl(s_eth, ETH_CMD_WRITE_PHY_REG, &d);
}
static esp_err_t mmd_rd(uint16_t dev, uint16_t reg, uint32_t *v)
{
    phy_wr(13, dev); phy_wr(14, reg); phy_wr(13, 0x4000 | dev);
    return phy_rd(14, v);
}
static esp_err_t mmd_wr(uint16_t dev, uint16_t reg, uint16_t v)
{
    phy_wr(13, dev); phy_wr(14, reg); phy_wr(13, 0x4000 | dev);
    return phy_wr(14, v);
}

static void eee_disable(void)
{
    uint32_t adv = 0xFFFF, lp = 0xFFFF;
    mmd_rd(7, 60, &adv);
    mmd_rd(7, 61, &lp);
    mmd_wr(7, 60, 0);
    uint32_t now = 0xFFFF;
    mmd_rd(7, 60, &now);
    ESP_LOGI(TAG, "EEE advert 0x%04x -> 0x%04x (partner last 0x%04x)",
             (unsigned)adv, (unsigned)now, (unsigned)lp);
}

// ---------------------------------------------------------------------------
// RECEIVE WATCHDOG. PTP alone brings ~8 frames/s, so a second with none while
// the link is up means receive has stalled. First try a KICK: wake IDF's
// emac_rx task so it drains whatever descriptors it is sitting on, and poke
// the DMA's receive-poll register so a suspended DMA looks again. If that
// does not bring frames back, restart the EMAC (which also drops and
// re-acquires the IP -- ~10 s -- but beats a dead device).
// ---------------------------------------------------------------------------
#include "soc/emac_dma_struct.h"
#include "soc/emac_mac_struct.h"

static void rx_watchdog_task(void *arg)
{
    (void)arg;
    uint32_t last = s_rx_frames;
    int idle_ms = 0;
    TaskHandle_t emac_rx = NULL;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        uint32_t now = s_rx_frames;
        if (now != last || !s_link_up) { last = now; idle_ms = 0; continue; }
        idle_ms += 250;
        if (idle_ms == 1000) {
            // What does the MAC see? RX DMA state + frames it had to drop.
            uint32_t st = EMAC_DMA.dmastatus.recv_proc_state;
            uint32_t mf = EMAC_DMA.dmamissedfr.missed_fc;       // read clears
            uint32_t of = EMAC_DMA.dmamissedfr.overflow_fc;
            s_rx_dma_state = st; s_rx_missed += mf; s_rx_fifo_ovf += of;
            ESP_LOGW(TAG, "RX idle 1 s: dma state %u (3 wait, 4 SUSPENDED no-desc), "
                          "missed %u, fifo overflow %u", (unsigned)st, (unsigned)mf, (unsigned)of);
        }
        if (idle_ms == 1000 || idle_ms == 2000) {
            if (!emac_rx) emac_rx = xTaskGetHandle("emac_rx");
            if (emac_rx) xTaskNotifyGive(emac_rx);
            EMAC_DMA.dmarxpolldemand = 1;
            s_rx_kicks++;
            ESP_LOGW(TAG, "no RX for %d ms -- kicked emac_rx (#%u)", idle_ms,
                     (unsigned)s_rx_kicks);
        } else if (idle_ms == 4000) {
            s_rx_restarts++;
            ESP_LOGE(TAG, "RX still dead -- restarting EMAC (#%u)", (unsigned)s_rx_restarts);
            esp_eth_stop(s_eth);
            esp_eth_start(s_eth);
        } else if (idle_ms >= 8000) {
            // Even an EMAC restart did not help: the silence is below the MAC.
            // Soft-reset the PHY (BMCR bit 15), put EEE back off, renegotiate.
            s_phy_resets++;
            ESP_LOGE(TAG, "RX dead after EMAC restart -- PHY soft reset (#%u)",
                     (unsigned)s_phy_resets);
            phy_wr(0, 0x8000);
            vTaskDelay(pdMS_TO_TICKS(50));
            mmd_wr(7, 60, 0);
            phy_wr(0, 0x1200);                 // AN enable + restart
            idle_ms = 0;
        }
    }
}

uint32_t eth_ts_rx_kicks(void)    { return s_rx_kicks; }
uint32_t eth_ts_rx_restarts(void) { return s_rx_restarts; }
uint32_t eth_ts_phy_resets(void)  { return s_phy_resets; }
// Raw MAC receive-side registers, healthy vs dead.
void eth_ts_mac_dump(void)
{
    printf("  mac       config %08x  frame-filter %08x  debug %08x\n",
           (unsigned)EMAC_MAC.gmacconfig.val, (unsigned)EMAC_MAC.gmacff.val,
           (unsigned)EMAC_MAC.emacdebug.val);
    printf("  mac addr0 %08x %08x\n", (unsigned)EMAC_MAC.emacaddr0high.val,
           (unsigned)EMAC_MAC.emacaddr0low);
    for (int i = 0; i < 15; i++) {
        if (EMAC_MAC.emacaddr[i].emacaddrhigh.address_enable)
            printf("  mac addr%-2d %08x %08x\n", i + 1,
                   (unsigned)EMAC_MAC.emacaddr[i].emacaddrhigh.val,
                   (unsigned)EMAC_MAC.emacaddr[i].emacaddrlow);
    }
}

void eth_ts_rx_diag(uint32_t *dma_state_now, uint32_t *missed, uint32_t *fifo_ovf)
{
    *dma_state_now = EMAC_DMA.dmastatus.recv_proc_state;
    uint32_t mf = EMAC_DMA.dmamissedfr.missed_fc, of = EMAC_DMA.dmamissedfr.overflow_fc;
    s_rx_missed += mf; s_rx_fifo_ovf += of;
    *missed = s_rx_missed; *fifo_ovf = s_rx_fifo_ovf;
}

esp_err_t eth_ts_start(void)
{
    eee_disable();                   // before autonegotiation starts
    esp_err_t err = esp_eth_start(s_eth);
    xTaskCreatePinnedToCore(rx_watchdog_task, "rx_wdog", 3072, NULL, 10, NULL, 1);
    return err;
}

// Log the partner's EEE ability once the link is up (diagnostic).
void eth_ts_log_eee(void)
{
    uint32_t adv = 0xFFFF, lp = 0xFFFF;
    mmd_rd(7, 60, &adv);
    mmd_rd(7, 61, &lp);
    ESP_LOGI(TAG, "EEE after link: ours 0x%04x, partner 0x%04x -> %s", (unsigned)adv,
             (unsigned)lp, (adv & lp & 0x0006) ? "EEE ACTIVE" : "EEE off");
}
bool eth_ts_link_up(void)    { return s_link_up; }
const uint8_t *eth_ts_mac(void) { return s_our_mac; }
esp_netif_t *eth_ts_netif(void) { return s_netif; }
void eth_ts_register_ptp_cb(eth_ts_ptp_cb_t cb) { s_ptp_cb = cb; }

// ---------------------------------------------------------------------------
// PTP clock
// ---------------------------------------------------------------------------

esp_err_t eth_ts_get_time(eth_ts_time_t *t)
{
    eth_mac_time_t mt;
    esp_err_t err = esp_eth_ioctl(s_eth, ETH_MAC_ESP_CMD_G_PTP_TIME, &mt);
    if (err != ESP_OK) return err;
    t->seconds     = mt.seconds;
    t->nanoseconds = mt.nanoseconds;
    return ESP_OK;
}

esp_err_t eth_ts_set_time(const eth_ts_time_t *t)
{
    eth_mac_time_t mt = { .seconds = t->seconds, .nanoseconds = t->nanoseconds };
    return esp_eth_ioctl(s_eth, ETH_MAC_ESP_CMD_S_PTP_TIME, &mt);
}

esp_err_t eth_ts_step_time(int64_t delta_ns)
{
    eth_ts_time_t t;
    esp_err_t err = eth_ts_get_time(&t);
    if (err != ESP_OK) return err;

    int64_t now = (int64_t)t.seconds * 1000000000LL + t.nanoseconds + delta_ns;
    if (now < 0) now = 0;
    t.seconds     = (uint32_t)(now / 1000000000LL);
    t.nanoseconds = (uint32_t)(now % 1000000000LL);
    return eth_ts_set_time(&t);
}

// Fine rate control: the MAC's addend register. This is the same actuator the
// FPGA's gPTP servo drove (gptp.c, the 52-bit TSU addend) -- note that it
// disciplines the PTP TIME BASE, not the audio clock. Two separate actuators,
// two separate loops, and they must not be crossed: pointing the media clock
// at the PTP phase term broke audio twice on the FPGA.
esp_err_t eth_ts_set_rate_ppb(int32_t ppb)
{
    s_rate_ppb = ppb;
    // ETH_MAC_ESP_CMD_ADJ_PTP_TIME, not ..._ADJ_PTP_FREQ: in IDF 5.5 the former
    // (emac_hal_ptp_adj_inc) sets the addend to base * 1e9 / (1e9 - ppb) --
    // ABSOLUTE ppb against the addend captured at first call, which is what
    // the servo emits. ADJ_PTP_FREQ multiplies the CURRENT addend by a double,
    // so calling it every update would compound. Positive ppb = faster, which
    // matches ptp_servo's sign (offset = slave - master, output = -gain * offset).
    return esp_eth_ioctl(s_eth, ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &ppb);
}

int32_t eth_ts_get_rate_ppb(void) { return s_rate_ppb; }

// ---------------------------------------------------------------------------
// Raw UDP multicast transmit with TX timestamp
// ---------------------------------------------------------------------------

static uint16_t ip_checksum(const uint8_t *p, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i + 1 < len; i += 2) sum += dw_rd16(p + i);
    if (len & 1) sum += (uint32_t)p[len - 1] << 8;
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return (uint16_t)~sum;
}

esp_err_t eth_ts_send_udp_mcast(uint32_t dst_ip, uint16_t dst_port, uint16_t src_port,
                                const uint8_t *payload, uint32_t len, uint8_t tos,
                                uint8_t ttl, eth_ts_time_t *tx_ts)
{
    uint8_t frame[14 + 20 + 8 + 256];
    if (len > sizeof(frame) - 42) return ESP_ERR_INVALID_SIZE;

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(s_netif, &ip) != ESP_OK) return ESP_FAIL;
    uint32_t src_ip = ntohl(ip.ip.addr);

    // Destination MAC for an IPv4 multicast group: 01:00:5e + low 23 bits.
    frame[0] = 0x01; frame[1] = 0x00; frame[2] = 0x5e;
    frame[3] = (uint8_t)((dst_ip >> 16) & 0x7F);
    frame[4] = (uint8_t)((dst_ip >> 8) & 0xFF);
    frame[5] = (uint8_t)(dst_ip & 0xFF);
    memcpy(frame + 6, s_our_mac, 6);
    dw_wr16(frame + 12, 0x0800);

    uint8_t *ipp = frame + 14;
    uint32_t total = 20 + 8 + len;
    memset(ipp, 0, 20);
    ipp[0] = 0x45;
    ipp[1] = tos;
    dw_wr16(ipp + 2, (uint16_t)total);
    ipp[8] = ttl;
    ipp[9] = 17;                       // UDP
    dw_wr32(ipp + 12, src_ip);
    dw_wr32(ipp + 16, dst_ip);
    dw_wr16(ipp + 10, ip_checksum(ipp, 20));

    uint8_t *udp = ipp + 20;
    dw_wr16(udp + 0, src_port);
    dw_wr16(udp + 2, dst_port);
    dw_wr16(udp + 4, (uint16_t)(8 + len));
    dw_wr16(udp + 6, 0);               // checksum optional over IPv4
    memcpy(udp + 8, payload, len);

    uint32_t frame_len = 14 + total;

    // IDF 5.5: `ctrl` is an eth_mac_time_t* the driver fills after polling the
    // descriptor; on timeout it writes 0/0 and still returns ESP_OK, so a
    // zero stamp must be treated as a failed send for PTP purposes.
    eth_mac_time_t ts = {0};
    esp_err_t err = esp_eth_transmit_ctrl_vargs(s_eth, &ts, 2, frame, frame_len);
    if (err == ESP_OK && ts.seconds == 0 && ts.nanoseconds == 0) err = ESP_ERR_TIMEOUT;
    if (err == ESP_OK && tx_ts) {
        tx_ts->seconds     = ts.seconds;
        tx_ts->nanoseconds = ts.nanoseconds;
    }
    return err;
}

// ---------------------------------------------------------------------------

esp_err_t eth_ts_mcast_allow(uint32_t group_ip)
{
    uint8_t mac[6] = { 0x01, 0x00, 0x5e,
                       (uint8_t)((group_ip >> 16) & 0x7F),
                       (uint8_t)((group_ip >> 8) & 0xFF),
                       (uint8_t)(group_ip & 0xFF) };
    // *** PORTING POINT *** -- ESP-IDF's per-group MAC filter ioctl. If the
    // build has none, the EMAC will pass all multicast and the software filter
    // carries the whole flood. That is survivable at 8 channels but it is
    // exactly the condition RX_GATE.md measured as 21% control-plane loss.
    ESP_LOGI(TAG, "mcast filter += %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return esp_eth_ioctl(s_eth, ETH_CMD_ADD_MAC_FILTER, mac);
}
