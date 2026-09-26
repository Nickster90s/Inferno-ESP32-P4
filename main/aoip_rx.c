#include "aoip_rx.h"
#include "rate.h"
#include "aoip_wire.h"
#include "jitterbuf.h"
#include "telem.h"
#include "media_clock.h"
#include "ptpv1.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <errno.h>

static const char *TAG = "aoip_rx";

typedef struct {
    int      sock;
    bool     active;
    uint8_t  nslots;
    int8_t   slot_to_ch[AP_MAX_CH_PER_FLOW];
    uint16_t fpp;
    uint16_t port;
    uint32_t packets;
    volatile int32_t lat_max;       // max (now - timestamp) at receipt, samples
    struct sockaddr_in src;         // where this flow's audio comes FROM
    bool     have_src;
} flow_t;

// THE FLOW KEEPALIVE. Two bytes, 0x13 0x37, sent every 250 ms FROM the audio
// receive socket TO the transmitter's audio source address (inferno
// flows_rx.rs KEEPALIVE_CONTENT / KEEPALIVE_INTERVAL). A transmitter drops a
// flow after 4 s without one (flows_tx.rs KEEPALIVE_TIMEOUT_SECONDS).
//
// Control-port opcode 0x0102 is NOT the keepalive -- it re-sets a flow's
// channel list. First bench run: DVS accepted every flow request, streamed for
// ~2 s, then answered 0x0102 with 0x0103 "expired", forever.
#define KEEPALIVE_US        250000
static const uint8_t KEEPALIVE[2] = { 0x13, 0x37 };

static flow_t s_flows[AP_MAX_FLOWS];
// Guards nslots / slot_to_ch against aoip_rx_remap_flow() on the other core.
static portMUX_TYPE s_map_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_peak_in[AP_NCH];     // max |sample|, 24-bit, since last take
static volatile uint32_t s_peak_hb[AP_NCH];     // the same, for the heartbeat's meters
static aoip_rx_stats_t s_st;
static uint8_t s_rxbuf[1600];

static void handle_packet(flow_t *fl, const uint8_t *buf, int len)
{
    aoip_audio_hdr_t h;
    if (!aoip_audio_parse(buf, (uint32_t)len, &h)) {
        if (len < AOIP_AUDIO_HDR_LEN + 3) s_st.short_pkt++;
        else s_st.bad_magic++;
        return;
    }

    // Snapshot the slot map: a patch can remap it from the control core.
    uint8_t nslots;
    int8_t  map[AP_MAX_CH_PER_FLOW];
    taskENTER_CRITICAL(&s_map_mux);
    nslots = fl->nslots;
    memcpy(map, fl->slot_to_ch, sizeof(map));
    taskEXIT_CRITICAL(&s_map_mux);

    // The packet must be EXACTLY fpp frames of nslots channels. Checking only
    // divisibility would let a packet in the previous layout through during a
    // remap -- 2 slots x 32 frames is also 1 slot x 64 -- and scramble it.
    uint32_t per_frame = nslots * 3;
    if (per_frame == 0 || h.sample_bytes != per_frame * fl->fpp) {
        s_st.wrong_len++;
        return;
    }
    uint32_t nframes = fl->fpp;

    uint64_t idx = aoip_ts_to_samples(h.seconds, h.subsec_samples, rate_hz());

    // ACTUAL LATENCY, the controller's number: now - timestamp at receipt,
    // kept as a running max until the heartbeat takes it (inferno
    // flows_rx.rs:122; FPGA docs/LATENCY.md 8). A timestamp in the future
    // clamps to 0, as there. Only while PTP is LOCKED, and nothing over
    // 50 ms (the protocol maximum is 40): across a PTP step or phase shift `now`
    // jumps and the difference is the step, not the network (seen: 100-144 ms
    // in the seconds after lock).
    uint64_t now;
    if (g_ptpv1.locked && mclk_now_samples(&now)) {
        int64_t d = (int64_t)(now - idx);
        if (d > (int64_t)(rate_hz() / 20)) d = 0;
        if (d > fl->lat_max) fl->lat_max = (int32_t)d;
        if (d > s_st.lat_max_samples) s_st.lat_max_samples = (int32_t)d;
    }

    jb_write(idx, nframes, nslots, map, h.samples);

    // Input meter: what the transmitter actually sent. Silence here and the
    // fault is upstream (nothing playing into that AoIP channel).
    for (uint32_t s = 0; s < nslots; s++) {
        int8_t ch = map[s];
        if (ch < 0 || ch >= AP_NCH) continue;
        uint32_t pk = s_peak_in[ch];
        for (uint32_t f = 0; f < nframes; f++) {
            const uint8_t *p = h.samples + (f * nslots + s) * 3;
            int32_t v = (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8)) >> 8;
            uint32_t a = (uint32_t)(v < 0 ? -v : v);
            if (a > pk) pk = a;
        }
        s_peak_in[ch] = pk;
        if (pk > s_peak_hb[ch]) s_peak_hb[ch] = pk;
    }

    fl->packets++;
    s_st.packets++;
}

static void send_keepalives(void)
{
    for (int i = 0; i < AP_MAX_FLOWS; i++) {
        flow_t *fl = &s_flows[i];
        if (!fl->active || fl->sock < 0 || !fl->have_src) continue;
        // CHECKED: an unsent keepalive is invisible otherwise, and four
        // seconds of them is a dropped flow on the transmitter.
        if (sendto(fl->sock, KEEPALIVE, sizeof(KEEPALIVE), 0,
                   (struct sockaddr *)&fl->src, sizeof(fl->src)) == sizeof(KEEPALIVE)) {
            s_st.ka_sent++;
        } else {
            s_st.ka_failed++;
            s_st.ka_last_errno = errno;
        }
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    int64_t next_ka = 0;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now >= next_ka) {
            send_keepalives();
            next_ka = now + KEEPALIVE_US;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = -1;
        for (int i = 0; i < AP_MAX_FLOWS; i++) {
            if (s_flows[i].active && s_flows[i].sock >= 0) {
                FD_SET(s_flows[i].sock, &rfds);
                if (s_flows[i].sock > maxfd) maxfd = s_flows[i].sock;
            }
        }
        if (maxfd < 0) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 20000 };
        int n = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (n <= 0) continue;

        for (int i = 0; i < AP_MAX_FLOWS; i++) {
            flow_t *fl = &s_flows[i];
            if (!fl->active || fl->sock < 0 || !FD_ISSET(fl->sock, &rfds)) continue;
            // Drain the socket rather than taking one packet per select: at
            // fpp=16 a flow delivers 3000 pps and a select round-trip per
            // packet is wasted work on the one task that must never be late.
            for (;;) {
                struct sockaddr_in from;
                socklen_t flen = sizeof(from);
                int len = recvfrom(fl->sock, s_rxbuf, sizeof(s_rxbuf), MSG_DONTWAIT,
                                   (struct sockaddr *)&from, &flen);
                if (len <= 0) break;
                if (!fl->have_src) {
                    fl->src = from;
                    fl->have_src = true;
                    send_keepalives();          // first one immediately
                }
                handle_packet(fl, s_rxbuf, len);
            }
        }
    }
}

esp_err_t aoip_rx_bind_flow(uint8_t idx, uint8_t nslots, const int8_t *slot_to_ch,
                             uint16_t fpp, uint16_t *out_port)
{
    if (idx >= AP_MAX_FLOWS || nslots == 0 || nslots > AP_MAX_CH_PER_FLOW)
        return ESP_ERR_INVALID_ARG;

    flow_t *fl = &s_flows[idx];
    aoip_rx_unbind_flow(idx);

    fl->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fl->sock < 0) return ESP_FAIL;

    int rcvbuf = 32 * 1024;
    setsockopt(fl->sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in a = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(AOIP_RX_AUDIO_PORT + idx),
    };
    if (bind(fl->sock, (struct sockaddr *)&a, sizeof(a)) != 0) {
        close(fl->sock);
        fl->sock = -1;
        return ESP_FAIL;
    }

    fl->nslots = nslots;
    memcpy(fl->slot_to_ch, slot_to_ch, nslots);
    for (int i = nslots; i < AP_MAX_CH_PER_FLOW; i++) fl->slot_to_ch[i] = -1;
    fl->fpp     = fpp;
    fl->port    = AOIP_RX_AUDIO_PORT + idx;
    fl->packets = 0;
    fl->have_src = false;
    fl->active  = true;

    if (out_port) *out_port = fl->port;
    ESP_LOGI(TAG, "flow %u bound: %u slots, fpp %u, port %u",
             idx, nslots, fpp, fl->port);
    return ESP_OK;
}

esp_err_t aoip_rx_remap_flow(uint8_t idx, uint8_t nslots, const int8_t *slot_to_ch)
{
    if (idx >= AP_MAX_FLOWS || nslots == 0 || nslots > AP_MAX_CH_PER_FLOW)
        return ESP_ERR_INVALID_ARG;
    flow_t *fl = &s_flows[idx];
    if (!fl->active) return ESP_ERR_INVALID_STATE;
    taskENTER_CRITICAL(&s_map_mux);
    fl->nslots = nslots;
    memcpy(fl->slot_to_ch, slot_to_ch, nslots);
    for (int i = nslots; i < AP_MAX_CH_PER_FLOW; i++) fl->slot_to_ch[i] = -1;
    taskEXIT_CRITICAL(&s_map_mux);
    ESP_LOGI(TAG, "flow %u remapped: %u slots", idx, nslots);
    return ESP_OK;
}

esp_err_t aoip_rx_unbind_flow(uint8_t idx)
{
    if (idx >= AP_MAX_FLOWS) return ESP_ERR_INVALID_ARG;
    flow_t *fl = &s_flows[idx];
    fl->active = false;
    if (fl->sock >= 0) { close(fl->sock); fl->sock = -1; }
    return ESP_OK;
}

void aoip_rx_get_stats(aoip_rx_stats_t *out) { *out = s_st; }

void aoip_rx_take_peaks(uint32_t out[AP_NCH])
{
    for (int c = 0; c < AP_NCH; c++) { out[c] = s_peak_in[c]; s_peak_in[c] = 0; }
}

// A second, independent accumulator: telemetry and the heartbeat each take
// (and reset) their own, so neither steals the other's peaks.
void aoip_rx_take_peaks_hb(uint32_t out[AP_NCH])
{
    for (int c = 0; c < AP_NCH; c++) { out[c] = s_peak_hb[c]; s_peak_hb[c] = 0; }
}

uint32_t aoip_rx_take_latency(uint8_t idx)
{
    if (idx >= AP_MAX_FLOWS || !s_flows[idx].active) return 0;
    int32_t v = __atomic_exchange_n(&s_flows[idx].lat_max, 0, __ATOMIC_RELAXED);
    return v > 0 ? (uint32_t)v : 0;
}

uint32_t aoip_rx_flow_packets(uint8_t idx)
{
    return idx < AP_MAX_FLOWS ? s_flows[idx].packets : 0;
}

esp_err_t aoip_rx_start(void)
{
    for (int i = 0; i < AP_MAX_FLOWS; i++) s_flows[i].sock = -1;
    BaseType_t ok = xTaskCreatePinnedToCore(rx_task, "aoip_rx", 4096, NULL,
                                            AP_PRIO_RX, NULL, AP_CORE_AUDIO);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
