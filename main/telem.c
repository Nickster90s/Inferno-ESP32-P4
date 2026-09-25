#include "telem.h"
#include "reqlog.h"
#include "eth_ts.h"
#include "esp_system.h"
#include <math.h>
#include <stdlib.h>
#include "app_config.h"
#include "rate.h"
#include "ptpv1.h"
#include "media_clock.h"
#include "mclk_hw.h"
#include "jitterbuf.h"
#include "aoip_rx.h"
#include "subscriber.h"
#include "flows_client.h"
#include "audio_out.h"
#include "aoip_wire.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "telem";

#define TELEM_PORT_STREAM   7778
#define TELEM_PORT_STATS    7779
#define RING_N              256

typedef struct {
    uint32_t t_ms;
    uint8_t  type;
    uint8_t  pad;
    uint16_t flags;
    int32_t  a, b, c, d;
} rec_t;

static rec_t   s_ring[RING_N];
static volatile uint32_t s_wr, s_rd;
static uint32_t s_dest_ip;
static uint16_t s_dest_port = TELEM_PORT_STREAM;

void telem_set_dest(uint32_t ip, uint16_t port)
{
    s_dest_ip   = ip;
    s_dest_port = port ? port : TELEM_PORT_STREAM;
}

void telem_push(uint8_t type, uint16_t flags, int32_t a, int32_t b, int32_t c, int32_t d)
{
    uint32_t w = s_wr;
    uint32_t next = (w + 1) % RING_N;
    if (next == s_rd) return;          // full: drop, never block
    rec_t *r = &s_ring[w];
    r->t_ms  = (uint32_t)(esp_timer_get_time() / 1000);
    r->type  = type;
    r->flags = flags;
    r->a = a; r->b = b; r->c = c; r->d = d;
    s_wr = next;
}

// ---------------------------------------------------------------------------
// Stats reply -- SELF-DESCRIBING, parsed by name.
//
// Emitted as newline-separated "key=value" text rather than a packed struct.
// That is deliberate and it costs nothing at 1 Hz: a packed reply is exactly
// what produced two wrong conclusions from stale hand-counted offsets, and
// growing one by 8 bytes once killed a port outright for reasons never found.
// ---------------------------------------------------------------------------

static int build_stats(char *out, size_t cap)
{
    jb_stats_t jb;         jb_get_stats(&jb);
    aoip_rx_stats_t rx;   aoip_rx_get_stats(&rx);
    subscriber_stats_t sub; subscriber_get_stats(&sub);

    int n = snprintf(out, cap,
        "v=3\n"
        "uptime_ms=%u\n"
        "reset_reason=%d\n"
        "eth_rx_kicks=%u\n"
        "eth_rx_restarts=%u\neth_fifo_hang_reboots=%u\ndma_isr_gaps=%u\n"
        "eth_phy_resets=%u\n"
        "rate_hz=%u\n"
        "rate_pin=%d\n"
        "rate_pinned=%d\n"
        "rate_fpp=%u\n"
        "ptp_locked=%d\n"
        "ptp_have_master=%d\n"
        "ptp_offset_ns=%lld\n"
        "ptp_path_delay_ns=%lld\n"
        "ptp_rate_ppb=%d\n"
        "ptp_rx_sync=%u\n"
        "ptp_rx_followup=%u\n"
        "ptp_rx_delay_resp=%u\n"
        "ptp_lost_followup=%u\n"
        "ptp_orphan_followup=%u\n"
        "ptp_mispair=%u\n"
        "ptp_no_hw_ts=%u\n"
        "ptp_steps=%u\n"
        "ptp_path_rejected=%u\n"
        "ptp_phase_shifts=%u\n"
        "ptp_out_ppb=%d\n"
        "ptp_adj_failed=%u\n"
        "ptp_adj_err=%d\n"
        "mclk_armed=%d\n"
        "mclk_anchored=%d\n"
        "mclk_error_frames=%d\n"
        "mclk_ppb_target=%d\n"
        "mclk_ppb_applied=%d\n"
        "mclk_ppb_ff=%d\n"
        "mclk_lsb_ppb=%u\n"
        "mclk_anchors=%u\n"
        "mclk_reset_steps=%u\n"
        "mclk_latency_us=%u\nmclk_latency_cfg_us=%u\n"
        "jb_level_frames=%d\n"
        "jb_underrun_frames=%u\n"
        "jb_pkt_written=%u\n"
        "jb_pkt_late=%u\n"
        "jb_pkt_future=%u\n"
        "jb_playout_steps=%u\n"
        "rx_packets=%u\n"
        "rx_bad_magic=%u\n"
        "rx_short=%u\n"
        "rx_wrong_len=%u\n"
        "rx_ka_sent=%u\n"
        "rx_ka_failed=%u\n"
        "rx_ka_errno=%d\nrx_lat_max_samples=%d\n"
        "fc_last_op=%u\n"
        "fc_last_code=%u\n"
        "fc_refused=%u\n"
        "fc_timeouts=%u\n"
        "flows_active=%u\n"
        "flow_req_ok=%u\n"
        "flow_req_failed=%u\n"
        "flow_ka_ok=%u\n"
        "flow_ka_lost=%u\n"
        "resolve_failed=%u\n"
        "flow_fallbacks=%u\n"
        "audio_blocks=%u\n",
        (unsigned)(esp_timer_get_time() / 1000),
        // esp_reset_reason_t: 1 power-on, 3 software, 4 PANIC, 5 int WDT,
        // 6 task WDT, 7 other WDT, 9 brown-out. Anything but 1/3 after a
        // silent reboot is a crash, and says which kind.
        (int)esp_reset_reason(),
        (unsigned)eth_ts_rx_kicks(), (unsigned)eth_ts_rx_restarts(), (unsigned)eth_ts_fifo_hang_reboots(), (unsigned)jb.isr_gaps, (unsigned)eth_ts_phy_resets(),
        (unsigned)rate_hz(), rate_pin_level(), (int)rate_is_pinned(),
        (unsigned)rate_get()->fpp,
        g_ptpv1.locked, g_ptpv1.have_master,
        (long long)g_ptpv1.offset_ns, (long long)g_ptpv1.mean_path_delay_ns,
        (int)g_ptpv1.rate_ppb,
        (unsigned)g_ptpv1.rx_sync, (unsigned)g_ptpv1.rx_followup,
        (unsigned)g_ptpv1.rx_delay_resp, (unsigned)g_ptpv1.lost_followup,
        (unsigned)g_ptpv1.orphan_followup, (unsigned)g_ptpv1.mispair,
        (unsigned)g_ptpv1.no_hw_ts, (unsigned)g_ptpv1.step_count,
        (unsigned)g_ptpv1.path_delay_rejected,
        (unsigned)g_ptpv1.phase_shifts,
        (int)g_ptpv1.out_ppb, (unsigned)g_ptpv1.rate_adj_failed,
        (int)g_ptpv1.rate_adj_last_err,
        g_mclk.armed, g_mclk.anchored, (int)g_mclk.error_frames,
        (int)g_mclk.ppb_target, (int)g_mclk.ppb_applied, (int)g_mclk.ppb_ff,
        (unsigned)mclk_hw_lsb_ppb(), (unsigned)g_mclk.anchors,
        (unsigned)g_mclk.reset_steps, (unsigned)mclk_get_latency_us(), (unsigned)mclk_get_config_latency_us(),
        (int)jb.level_frames, (unsigned)jb.frames_underrun,
        (unsigned)jb.pkt_written, (unsigned)jb.pkt_late, (unsigned)jb.pkt_future,
        (unsigned)jb.playout_steps,
        (unsigned)rx.packets, (unsigned)rx.bad_magic, (unsigned)rx.short_pkt,
        (unsigned)rx.wrong_len,
        (unsigned)rx.ka_sent, (unsigned)rx.ka_failed, (int)rx.ka_last_errno, (int)rx.lat_max_samples,
        (unsigned)g_flows_diag.last_opcode, (unsigned)g_flows_diag.last_code,
        (unsigned)g_flows_diag.refused, (unsigned)g_flows_diag.timeouts,
        (unsigned)sub.flows_active, (unsigned)sub.requests_ok,
        (unsigned)sub.requests_failed, (unsigned)sub.keepalives_ok,
        (unsigned)sub.keepalives_lost, (unsigned)sub.resolve_failed, (unsigned)sub.fallbacks,
        (unsigned)audio_out_blocks());
    if (n < 0 || (size_t)n >= cap) return n;

    // Level meters, dBFS per DAC channel; -999 = digital silence.
    uint32_t pin[AP_NCH], pout[AP_NCH];
    aoip_rx_take_peaks(pin);
    audio_out_take_peaks(pout);
    const char *key[2] = { "peak_in_dbfs=", "peak_out_dbfs=" };
    for (int k = 0; k < 2; k++) {
        n += snprintf(out + n, cap - n, "%s", key[k]);
        for (int c = 0; c < AP_NCH && (size_t)n < cap; c++) {
            uint32_t v = k ? pout[c] : pin[c];
            double fs = k ? 2147483648.0 : 8388608.0;
            int db = v ? (int)lround(20.0 * log10(v / fs)) : -999;
            n += snprintf(out + n, cap - n, "%d%s", db, c + 1 < AP_NCH ? "," : "\n");
        }
    }
    if ((size_t)n < cap)
        n += snprintf(out + n, cap - n, "dma_isr_max_us=%u\n", (unsigned)jb_take_isr_max_us());
    return n;
}

static void telem_task(void *arg)
{
    (void)arg;
    int stream = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int stats  = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    struct sockaddr_in a = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(TELEM_PORT_STATS),
    };
    bind(stats, (struct sockaddr *)&a, sizeof(a));

    static char txt[2048];
    uint8_t batch[sizeof(rec_t) * 16 + 4];

    for (;;) {
        // Drain the event ring to the collector.
        if (s_dest_ip) {
            uint32_t n = 0;
            batch[0] = 2;                  // record format version
            batch[1] = sizeof(rec_t);
            uint32_t off = 4;
            while (s_rd != s_wr && n < 16) {
                memcpy(batch + off, &s_ring[s_rd], sizeof(rec_t));
                off += sizeof(rec_t);
                s_rd = (s_rd + 1) % RING_N;
                n++;
            }
            if (n) {
                batch[2] = (uint8_t)n;
                batch[3] = 0;
                struct sockaddr_in d = {
                    .sin_family = AF_INET,
                    .sin_addr.s_addr = htonl(s_dest_ip),
                    .sin_port = htons(s_dest_port),
                };
                sendto(stream, batch, off, 0, (struct sockaddr *)&d, sizeof(d));
            }
        } else {
            s_rd = s_wr;                   // nobody listening; don't back up
        }

        // Answer stats queries.
        struct sockaddr_in from;
        socklen_t flen = sizeof(from);
        char q[8];
        int qn = recvfrom(stats, q, sizeof(q), MSG_DONTWAIT,
                          (struct sockaddr *)&from, &flen);
        if (qn > 0 && q[0] == 'F') {
            q[qn < (int)sizeof(q) ? qn : (int)sizeof(q) - 1] = 0;
            mclk_force_latency_us((uint32_t)atoi(q + 1));
            int len = snprintf(txt, sizeof(txt), "latency %u us\n", (unsigned)mclk_get_latency_us());
            sendto(stats, txt, (size_t)len, 0, (struct sockaddr *)&from, flen);
        } else if (qn > 0 && q[0] == 'L') {
            // Request log (reqlog.c): what controllers have been asking.
            int len = reqlog_dump(txt, sizeof(txt));
            sendto(stats, txt, (size_t)(len > 0 ? len : 1), 0, (struct sockaddr *)&from, flen);
        } else if (qn > 0) {
            int len = build_stats(txt, sizeof(txt));
            if (len > 0) {
                sendto(stats, txt, (size_t)len, 0, (struct sockaddr *)&from, flen);
                // The querier is presumably also where the stream should go.
                if (!s_dest_ip) s_dest_ip = ntohl(from.sin_addr.s_addr);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t telem_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(telem_task, "telem", 6144, NULL,
                                            AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    ESP_LOGI(TAG, "stats on udp/%d, stream to udp/%d", TELEM_PORT_STATS, TELEM_PORT_STREAM);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
