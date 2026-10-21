#include "aoip_rx.h"
#include "rate.h"
#include "aoip_wire.h"
#include "jitterbuf.h"
#include "telem.h"
#include "media_clock.h"
#include "ptpv1.h"
#include "eth_ts.h"
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
    uint32_t missed;                // packets lost on the way or dropped late (0x8004)
    uint64_t next_idx;              // timestamp the next packet should carry
    bool     have_next;
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
// 100 ms, not inferno's 250: a Prodigy (Brooklyn-3) transmitter drops a flow
// after ~1 s without one (measured from a PC: no keepalive -> audio stopped
// after 1.1 s, "0x0103" on stop), so 250 ms left four chances. The DAC8 lost
// its flow 7 times in 30 minutes; each time the keepalive log showed a ~940 ms
// gap while the receive loop was held up elsewhere. Hence also their own task.
#define KEEPALIVE_US        100000
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
    // clamps to 0, as there. Nothing over 50 ms (the protocol maximum is 40):
    // across a PTP step or phase shift `now` jumps and the difference is the
    // step, not the network (seen: 100-144 ms in the seconds after lock).
    // NOT gated on "locked": a brief unlock froze the controller's latency
    // display while the audio played on (mclk_now_samples needs a master).
    uint64_t now;
    if (mclk_now_samples(&now)) {
        int64_t d = (int64_t)(now - idx);
        if (d > (int64_t)(rate_hz() / 20)) d = 0;
        if (d > fl->lat_max) fl->lat_max = (int32_t)d;
        if (d > s_st.lat_max_samples) s_st.lat_max_samples = (int32_t)d;
    }

    // MISSED PACKETS, cumulative per flow -- the heartbeat's 0x8004 word,
    // which a RedNet AM2 counts up (57, then 62 while 0x8003 moved) and the
    // controller's Latency tab shows beside the latency. Counted: packets that
    // never arrived (a timestamp jump of more than one packet) and packets
    // that arrived too late to play.
    if (fl->have_next && (int64_t)(idx - fl->next_idx) > 0) {
        uint64_t gap = (idx - fl->next_idx) / nframes;
        fl->missed += gap > 1000 ? 1000 : (uint32_t)gap;
    }
    if (!fl->have_next || (int64_t)(idx - fl->next_idx) >= 0) {
        fl->next_idx = idx + nframes;
        fl->have_next = true;
    }
    if (!jb_write(idx, nframes, nslots, map, h.samples)) fl->missed++;

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

// ---------------------------------------------------------------------------
// FAST PATH. eth_ts.c hands us a unicast UDP frame to one of our flow ports in
// the emac_rx task, straight after the DMA delivered it. We decode it there:
// the same handle_packet() the socket path uses, into the same ring. What is
// gone is lwIP's tcpip thread, the socket, select() and this task's wake-up.
//
// The socket stays open, bound, for SENDING: the flow keepalive has to come
// from the audio port. It is never read while the fast path is on. And
// nothing here may call lwIP -- emac_rx feeds lwIP, so a send from here could
// wait on the very thread that waits on us. The first packet's keepalive is
// therefore flagged to the keepalive task instead of sent.
// ---------------------------------------------------------------------------
static volatile bool s_fast = true;
static volatile bool s_ka_now;
static volatile bool s_src_note;

void aoip_rx_set_fast_path(bool on)
{
    s_fast = on;
    ESP_LOGW(TAG, "audio path: %s", on ? "FAST (EMAC task)" : "socket (lwIP)");
}
bool aoip_rx_fast_path(void) { return s_fast; }

static bool fast_input(uint32_t src_ip_be, uint16_t src_port, uint16_t dst_port,
                       const uint8_t *payload, uint32_t len)
{
    if (!s_fast) return false;
    uint16_t idx = (uint16_t)(dst_port - AOIP_RX_AUDIO_PORT);
    if (idx >= AP_MAX_FLOWS) return false;
    flow_t *fl = &s_flows[idx];
    if (!fl->active || fl->sock < 0) return false;   // not ours now: let lwIP answer
    if (!fl->have_src) {
        fl->src.sin_family = AF_INET;
        fl->src.sin_addr.s_addr = src_ip_be;
        fl->src.sin_port = htons(src_port);
        __atomic_store_n(&fl->have_src, true, __ATOMIC_RELEASE);
        s_ka_now = true;                             // the keepalive task sends it
    } else if (fl->src.sin_addr.s_addr != src_ip_be || fl->src.sin_port != htons(src_port)) {
        // DIAGNOSTIC: a second stream into this port (a stale flow on the
        // transmitter?) -- the keepalive only goes to the latched source.
        s_st.src_other++;
        s_st.src_other_ip = src_ip_be;
        s_st.src_other_port = src_port;
        s_src_note = true;
    }
    handle_packet(fl, payload, (int)len);
    s_st.fast_packets++;
    return true;
}

static void send_keepalives(void)
{
    static int64_t last_ok;
    for (int i = 0; i < AP_MAX_FLOWS; i++) {
        flow_t *fl = &s_flows[i];
        if (!fl->active || fl->sock < 0 || !fl->have_src) continue;
        int64_t t0 = esp_timer_get_time();
        // CHECKED: an unsent keepalive is invisible otherwise, and four
        // seconds of them is a dropped flow on the transmitter.
        if (sendto(fl->sock, KEEPALIVE, sizeof(KEEPALIVE), 0,
                   (struct sockaddr *)&fl->src, sizeof(fl->src)) == sizeof(KEEPALIVE)) {
            s_st.ka_sent++;
            int64_t t1 = esp_timer_get_time();
            if (t1 - t0 > s_st.ka_send_max_us) s_st.ka_send_max_us = (uint32_t)(t1 - t0);
            if (last_ok) {
                int64_t gap = t1 - last_ok;
                if (gap > s_st.ka_gap_max_us) s_st.ka_gap_max_us = (uint32_t)gap;
                if (gap > 500000) {
                    s_st.ka_gaps_500ms++;
                    ESP_LOGW(TAG, "keepalive gap %lld ms (sendto took %lld ms)",
                             (long long)(gap / 1000), (long long)((t1 - t0) / 1000));
                }
            }
            last_ok = t1;
        } else {
            s_st.ka_failed++;
            s_st.ka_last_errno = errno;
        }
    }
}

// KEEPALIVES, on their own task: nothing the receive loop waits on (select,
// a socket drain, the log) may delay them.
static volatile uint32_t s_ka_loops;

static void ka_task(void *arg)
{
    (void)arg;
    int64_t next_ka = 0;
    for (;;) {
        s_ka_loops++;
        int64_t now = esp_timer_get_time();
        if (s_ka_now || now >= next_ka) {
            s_ka_now = false;
            send_keepalives();
            next_ka = now + KEEPALIVE_US;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// STALL DETECTOR (diagnostic). Core 0 sometimes stops running everything
// below some priority for ~1 s (keepalive gap and select() both 940-1260 ms),
// and the Prodigy drops the flow. From core 1, sample which task core 0 is
// running every 5 ms; when the keepalive task has not looped for 300 ms, say
// who had the core.
#define STALL_SLOTS 6
static void stall_task(void *arg)
{
    (void)arg;
    uint32_t last_loops = s_ka_loops;
    int64_t last_move = esp_timer_get_time();
    TaskHandle_t h[STALL_SLOTS] = {0};
    uint32_t c[STALL_SLOTS] = {0};
    bool in_stall = false;
    uint32_t f0 = eth_ts_rx_frames(), a0 = s_st.fast_packets;
    uint64_t hk0, bt0;
    eth_ts_hook_times(&hk0, &bt0);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5));
        int64_t now = esp_timer_get_time();
        uint32_t l = s_ka_loops;
        if (l != last_loops) {
            if (in_stall) {
                char buf[160]; int n = 0;
                for (int i = 0; i < STALL_SLOTS && h[i]; i++)
                    n += snprintf(buf + n, sizeof(buf) - n, " %s:%u", pcTaskGetName(h[i]),
                                  (unsigned)(c[i] * 5));
                uint64_t hk, bt;
                eth_ts_hook_times(&hk, &bt);
                uint32_t nf = eth_ts_rx_frames() - f0;
                ESP_LOGW(TAG, "core-0 stall %lld ms, %u frames (%u audio), per frame: hook %u us, driver %u us; core 0 ran (ms):%s",
                         (long long)((now - last_move) / 1000), (unsigned)nf,
                         (unsigned)(s_st.fast_packets - a0),
                         (unsigned)(nf ? (hk - hk0) / nf : 0), (unsigned)(nf ? (bt - bt0) / nf : 0), buf);
                in_stall = false;
            }
            last_loops = l; last_move = now;
            f0 = eth_ts_rx_frames(); a0 = s_st.fast_packets;
            eth_ts_hook_times(&hk0, &bt0);
            memset(h, 0, sizeof(h)); memset(c, 0, sizeof(c));
            continue;
        }
        if (now - last_move > 300000) in_stall = true;
        TaskHandle_t cur = xTaskGetCurrentTaskHandleForCore(0);
        for (int i = 0; i < STALL_SLOTS; i++) {
            if (h[i] == cur) { c[i]++; break; }
            if (!h[i]) { h[i] = cur; c[i] = 1; break; }
        }
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    for (;;) {
        static int64_t last_note;
        if (s_src_note && esp_timer_get_time() - last_note > 1000000) {
            s_src_note = false;
            last_note = esp_timer_get_time();
            uint32_t ip = ntohl(s_st.src_other_ip);
            ESP_LOGW(TAG, "audio from another source %u.%u.%u.%u:%u (%u so far)",
                     (unsigned)(ip >> 24), (unsigned)(ip >> 16 & 255), (unsigned)(ip >> 8 & 255),
                     (unsigned)(ip & 255), (unsigned)s_st.src_other_port, (unsigned)s_st.src_other);
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
        int64_t t_sel = esp_timer_get_time();
        int n = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        t_sel = esp_timer_get_time() - t_sel;
        if (t_sel > 200000)
            ESP_LOGW(TAG, "select() blocked %lld ms (20 ms timeout)", (long long)(t_sel / 1000));
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
                    s_ka_now = true;            // first one immediately
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
    fl->missed = 0;
    fl->have_next = false;
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

uint32_t aoip_rx_flow_missed(uint8_t idx)
{
    return (idx < AP_MAX_FLOWS && s_flows[idx].active) ? s_flows[idx].missed : 0;
}

uint32_t aoip_rx_flow_packets(uint8_t idx)
{
    return idx < AP_MAX_FLOWS ? s_flows[idx].packets : 0;
}

esp_err_t aoip_rx_start(void)
{
    for (int i = 0; i < AP_MAX_FLOWS; i++) s_flows[i].sock = -1;
    eth_ts_register_audio_cb(fast_input);
    BaseType_t ok = xTaskCreatePinnedToCore(rx_task, "aoip_rx", 4096, NULL,
                                            AP_PRIO_RX, NULL, AP_CORE_AUDIO);
    if (ok == pdPASS)
        // Core 0, just above the receive loop. NOT the control core: sent
        // from core 1 ten times a second, the keepalives pinned emac_rx (core
        // 0, prio 24) at 100% within 10 s of the flow starting -- IDLE0 never
        // ran, task watchdog every 5 s, PTP lost.
        ok = xTaskCreatePinnedToCore(ka_task, "aoip_ka", 3072, NULL,
                                     AP_PRIO_RX + 1, NULL, AP_CORE_AUDIO);
    if (ok == pdPASS)
        ok = xTaskCreatePinnedToCore(stall_task, "stall_mon", 3072, NULL,
                                     AP_PRIO_CONTROL + 1, NULL, AP_CORE_CONTROL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
