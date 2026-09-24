// PTPv1 slave. Wire format: see aoip_wire.h, confirmed byte for byte against a
// RedNet A16R by the FPGA project.

#include "ptpv1.h"
#include "ptp_servo.h"
#include "eth_ts.h"
#include "aoip_wire.h"
#include "app_config.h"
#include "telem.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "ptpv1";

// PTP TRACE: per-sample timestamps to the console, for a time window after
// boot. OFF by default -- each line blocks this core-0 task for ~10 ms of
// UART, and with it on the audio flow was lost six times in two minutes.
// Build with -DAP_PTP_TRACE=1 (and adjust the window) to use it.
#ifndef AP_PTP_TRACE
#define AP_PTP_TRACE    0
#endif
#define TRACE_FROM_MS   15000
#define TRACE_TO_MS     35000
static inline bool tracing(void)
{
#if AP_PTP_TRACE
    int64_t ms = esp_timer_get_time() / 1000;
    return ms >= TRACE_FROM_MS && ms <= TRACE_TO_MS;
#else
    return false;
#endif
}


ptpv1_state_t g_ptpv1;

#define DELAY_REQ_MS        4000
#define DELAY_REQ_FAST_MS   250     // until the path delay is measured: 4 samples in ~1 s
#define PD_MIN_SAMPLES      4

#define SMALL_STEP_NS   100000     // below this a step does not re-anchor the audio

static QueueHandle_t s_q;
static ptp_servo_t   s_servo;
static uint8_t       s_uuid[6];
static uint16_t      s_port_id = 1;
static uint16_t      s_delay_req_seq;

// Sync/FollowUp pairing
static int64_t  s_t1, s_t2, s_t3, s_t4;
static bool     s_have_t1, s_have_t2, s_have_t3;
static bool     s_awaiting_followup;
static uint16_t s_pending_sync_seq;

// Path delay
static int64_t  s_path_delay_acc;
static uint32_t s_path_delay_n;
static int64_t  s_last_ms;          // latest t2 - t1, paired with t4 - t3
static bool     s_snapped;          // post-estimate phase snap done
static bool     s_path_fresh;       // path delay measured AFTER the frequency fix
static bool     s_have_ms;

// A step moves our clock under every timestamp taken before it: a t3 or a
// (t2 - t1) from before the step no longer pairs with anything after it, and
// an average containing them is wrong until it slides out.
static void path_delay_invalidate(void)
{
    s_path_fresh = false;
    s_have_t3 = false;
    s_have_ms = false;
    s_path_delay_acc = 0;
    s_path_delay_n   = 0;
}

static inline int64_t ts_ns(uint32_t sec, uint32_t nsec)
{
    return (int64_t)sec * 1000000000LL + nsec;
}

static void on_ptp_frame(const eth_ts_ptp_frame_t *f)
{
    // Runs on the Ethernet RX task. Copy and get out -- everything expensive
    // happens in the PTP task.
    if (s_q) xQueueSend(s_q, f, 0);
}

// ---------------------------------------------------------------------------

static void send_delay_req(void)
{
    uint8_t buf[PTP1_HDR_LEN + PTP1_SYNC_BODY_LEN];
    uint32_t n = ptp1_put_header(buf, s_uuid, s_port_id, PTP1_CTRL_DELAY_REQ,
                                 PTP1_PORT_TYPE_EVENT, s_delay_req_seq);
    memset(buf + n, 0, PTP1_SYNC_BODY_LEN);   // DelayReq shares Sync's body
    n += PTP1_SYNC_BODY_LEN;

    eth_ts_time_t tx;
    if (eth_ts_send_udp_mcast(PTP1_GROUP_IP, PTP1_EVENT_PORT, PTP1_EVENT_PORT,
                              buf, n, PTP1_TOS, 1, &tx) == ESP_OK) {
        s_t3 = ts_ns(tx.seconds, tx.nanoseconds);
        s_have_t3 = true;
        g_ptpv1.tx_delay_req++;
    }
    s_delay_req_seq++;
}

static void apply_servo(int64_t offset_ns)
{
    int64_t step_ns = 0;
    int32_t ppb = ptp_servo_update(&s_servo, offset_ns, s_t2, &step_ns);

    if (step_ns) {
        eth_ts_step_time(step_ns);
        eth_ts_set_rate_ppb(ppb);            // the servo's rate estimate, P dropped
        g_ptpv1.out_ppb = ppb;
        // The path changed (shift) or pairings straddle the step: re-measure,
        // at 4 Hz, holding the rate meanwhile.
        path_delay_invalidate();
        if (step_ns > SMALL_STEP_NS || step_ns < -SMALL_STEP_NS) {
            g_ptpv1.step_count++;            // media clock re-anchors on this
            ESP_LOGW(TAG, "step %lld ns (#%u)", (long long)step_ns,
                     (unsigned)g_ptpv1.step_count);
        } else {
            // Under 100 us is < 5 samples: the audio absorbs it as phase
            // error. Re-anchoring for it would put a click in the output.
            g_ptpv1.phase_shifts++;
            ESP_LOGI(TAG, "phase shift %lld ns corrected, lock kept (#%u)",
                     (long long)step_ns, (unsigned)g_ptpv1.phase_shifts);
        }
    } else {
        g_ptpv1.out_ppb = ppb;
        esp_err_t err = eth_ts_set_rate_ppb(ppb);
        if (err != ESP_OK) {
            g_ptpv1.rate_adj_failed++;
            g_ptpv1.rate_adj_last_err = err;
        }
    }

    // Milestones, for timing acquisition (Sync red -> green in DC).
    static bool was_est, was_locked;
    if (s_servo.est_done && !was_est)
        ESP_LOGI(TAG, "freq estimate: %d ppb", (int)s_servo.rate_ppb);
    if (s_servo.locked && !was_locked)
        ESP_LOGI(TAG, "LOCKED  offset %lld ns  rate %d ppb", (long long)offset_ns,
                 (int)s_servo.rate_ppb);
    if (!s_servo.locked && was_locked) ESP_LOGW(TAG, "lock lost");
    was_est = s_servo.est_done; was_locked = s_servo.locked;

    g_ptpv1.locked        = s_servo.locked;
    g_ptpv1.offset_ns     = s_servo.offset_ns;
    g_ptpv1.rate_ppb      = s_servo.rate_ppb;
    g_ptpv1.servo_updates = s_servo.updates;

    telem_push(TELEM_T_PTP, g_ptpv1.locked ? TELEM_F_LOCKED : 0,
               (int32_t)s_servo.filtered_ns, (int32_t)offset_ns,
               ppb, (int32_t)g_ptpv1.mean_path_delay_ns);
}

static void compute_offset(void)
{
    if (!s_have_t1 || !s_have_t2) return;

    // offset = (t2 - t1) - path_delay
    int64_t raw = s_t2 - s_t1;
    if (tracing())
        ESP_LOGI("trace", "SYNC t1=%lld t2=%lld raw=%lld path=%lld off=%lld",
                 (long long)s_t1, (long long)s_t2, (long long)raw,
                 (long long)g_ptpv1.mean_path_delay_ns,
                 (long long)(raw - g_ptpv1.mean_path_delay_ns));
    s_last_ms = raw;
    s_have_ms = true;
    int64_t offset = raw - g_ptpv1.mean_path_delay_ns;

    // ACQUISITION ORDER, each stage as early as it can run:
    //
    //  1. Step, as soon as the first Sync pairs -- no IP or path delay needed.
    //  2. Frequency estimate on the RAW t2 - t1. The path delay is a constant,
    //     so the slope is the same, and the estimate no longer waits for it
    //     (it used to: a path delay arriving mid-window reads as ~20 ppm).
    //  3. Hold until the path delay is known (it would otherwise appear as a
    //     ~35 us phase jump), then SNAP the phase with one small step rather
    //     than making the servo walk out everything that built up meanwhile --
    //     that walk was 25 of the 47 s to lock on the bench.
    //  4. Servo.
    bool path_known = g_ptpv1.mean_path_delay_ns != 0 && s_path_fresh;
    if (!s_servo.est_done) {
        s_snapped = false;
        apply_servo(raw);
        if (s_servo.est_done) {
            // PATH SAMPLES FROM BEFORE THE FREQUENCY FIX ARE BIASED. A path
            // sample pairs a Sync (t2 - t1) with a Delay_Req (t4 - t3) up to
            // 250 ms apart; at 35 ppm uncorrected that is several us each, and
            // the average carried them: 31 us measured at boot against a
            // steady 24 us, and the offset crept 7 us as they aged out -- lock
            // lost 6 s after first locking. Re-measure now (4 x 250 ms); the
            // old mean stays in use meanwhile.
            s_path_delay_acc = 0;
            s_path_delay_n   = 0;
            s_path_fresh     = false;
        }
    } else if (!path_known) {
        g_ptpv1.offset_ns = offset;          // coasting on the estimate
    } else if (!s_snapped) {
        s_snapped = true;
        if (offset > 5000 || offset < -5000) {
            eth_ts_step_time(-offset);
            // Small step: only in-flight pairings are stale. The path delay
            // average was measured correctly and stays -- dropping it would
            // cost another second.
            s_have_t3 = false;
            s_have_ms = false;
            s_servo.median_count = 0;
            s_servo.median_pos = 0;
            g_ptpv1.step_count++;            // media clock re-anchors on this
            ESP_LOGI(TAG, "phase snap %lld ns", (long long)-offset);
        } else {
            apply_servo(offset);
        }
    } else {
        apply_servo(offset);
    }

    s_have_t1 = s_have_t2 = false;
}

static void handle_sync(const eth_ts_ptp_frame_t *f)
{
    const uint8_t *p = f->payload;
    uint16_t seq   = dw_rd16(p + 30);
    uint8_t  flags = p[35];
    const uint8_t *body = p + PTP1_HDR_LEN;
    uint32_t blen = f->len - PTP1_HDR_LEN;
    if (blen < 8) return;

    if (g_ptpv1.rx_sync++ == 0) ESP_LOGI(TAG, "first Sync");

    // Whoever sends Sync is the Leader. No BMCA: we are slave-only, so we
    // simply follow the source of Sync.
    memcpy(g_ptpv1.master_uuid, p + 22, 6);
    g_ptpv1.master_port_id = dw_rd16(p + 28);
    g_ptpv1.have_master    = true;

    if (!f->ts_valid) { g_ptpv1.no_hw_ts++; return; }
    s_t2 = ts_ns(f->ts.seconds, f->ts.nanoseconds);
    s_have_t2 = true;

    // A NEW SYNC WHILE THE PREVIOUS ONE IS STILL UNANSWERED means its FollowUp
    // never arrived. Record it BEFORE clobbering the pending state -- on the
    // FPGA this was overwritten in silence, discarding the single event that
    // explains the offset excursion which follows it.
    if (s_awaiting_followup) {
        g_ptpv1.lost_followup++;
        telem_push(TELEM_T_PTP, TELEM_F_NO_FOLLOWUP, s_pending_sync_seq,
                   (int32_t)seq, (int32_t)g_ptpv1.lost_followup, 0);
    }

    if (flags & PTP1_FLAG_PTP_ASSIST) {
        s_pending_sync_seq  = seq;
        s_awaiting_followup = true;
        s_have_t1 = false;
    } else {
        s_t1 = ts_ns(dw_rd32(body), dw_rd32(body + 4));
        s_have_t1 = true;
        s_awaiting_followup = false;
        compute_offset();
    }
}

static void handle_followup(const eth_ts_ptp_frame_t *f)
{
    const uint8_t *body = f->payload + PTP1_HDR_LEN;
    if (f->len < PTP1_HDR_LEN + PTP1_FOLLOWUP_BODY_LEN) return;
    g_ptpv1.rx_followup++;

    // PTPv1 Follow_Up body: [0..2] reserved, [2..4] associatedSequenceId,
    // [4..12] preciseOriginTimestamp (u32 sec + u32 nsec).
    uint16_t assoc = dw_rd16(body + 2);

    if (!s_awaiting_followup) { g_ptpv1.orphan_followup++; return; }
    if (assoc != s_pending_sync_seq) {
        g_ptpv1.mispair++;
        telem_push(TELEM_T_PTP, TELEM_F_MISPAIR, s_pending_sync_seq,
                   (int32_t)assoc, (int32_t)g_ptpv1.mispair, 0);
        return;
    }

    s_t1 = ts_ns(dw_rd32(body + 4), dw_rd32(body + 8));
    s_have_t1 = true;
    s_awaiting_followup = false;
    compute_offset();
}

static void handle_delay_resp(const eth_ts_ptp_frame_t *f)
{
    const uint8_t *body = f->payload + PTP1_HDR_LEN;
    if (f->len < PTP1_HDR_LEN + PTP1_DELAY_RESP_BODY_LEN) return;
    g_ptpv1.rx_delay_resp++;

    // Body: [0..8] delayReceiptTimestamp, [9] commTech, [10..16] requesting
    // uuid, [16..18] requesting portId, [18..20] requesting sequenceId.
    if (memcmp(body + 10, s_uuid, 6) != 0) return;
    if (dw_rd16(body + 16) != s_port_id) return;
    if (!s_have_t3) return;

    s_t4 = ts_ns(dw_rd32(body), dw_rd32(body + 4));
    s_have_t3 = false;
    if (!s_have_ms) return;

    // path_delay = ((t2 - t1) + (t4 - t3)) / 2, averaged. BOTH halves: t4 - t3
    // alone is path_delay MINUS the offset, and before lock the offset is
    // seconds. Feeding that back as the path delay made the first bench run
    // step 8 times in a minute and report a path of -1.6 s.
    int64_t d = (s_last_ms + (s_t4 - s_t3)) / 2;
    if (tracing())
        ESP_LOGI("trace", "DRESP t3=%lld t4=%lld sm=%lld ms=%lld d=%lld n=%u acc=%lld",
                 (long long)s_t3, (long long)s_t4, (long long)(s_t4 - s_t3),
                 (long long)s_last_ms, (long long)d, (unsigned)s_path_delay_n,
                 (long long)s_path_delay_acc);
    if (d < 0 || d > 10000000) {            // not a LAN; a stale pairing
        g_ptpv1.path_delay_rejected++;
        return;
    }
    s_path_delay_acc += d;
    s_path_delay_n++;
    if (s_path_delay_n >= PD_MIN_SAMPLES) {
        if (s_servo.est_done && !s_path_fresh) {
            s_path_fresh = true;
            ESP_LOGI(TAG, "path delay (post-estimate): %lld ns",
                     (long long)(s_path_delay_acc / (int64_t)s_path_delay_n));
        }
        if (g_ptpv1.mean_path_delay_ns == 0)
            ESP_LOGI(TAG, "path delay known: %lld ns", (long long)(s_path_delay_acc / (int64_t)s_path_delay_n));
        g_ptpv1.mean_path_delay_ns = s_path_delay_acc / (int64_t)s_path_delay_n;
        if (s_path_delay_n > 16) {          // slide, don't accumulate forever
            s_path_delay_acc /= 2;
            s_path_delay_n   /= 2;
        }
    }
}

// ---------------------------------------------------------------------------

static void ptp_task(void *arg)
{
    (void)arg;
    eth_ts_ptp_frame_t f;
    int64_t next_dreq_us = 0;

    for (;;) {
        if (xQueueReceive(s_q, &f, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (ptp1_header_ok(f.payload, f.len)) {
                const uint8_t *uuid = f.payload + 22;
                if (memcmp(uuid, s_uuid, 6) != 0) {     // ignore our own echo
                    if (tracing()) {
                        const uint8_t *u = f.payload + 22;
                        ESP_LOGI("trace", "RX ctrl=%u from %02x%02x%02x%02x%02x%02x seq=%u",
                                 f.payload[32], u[0], u[1], u[2], u[3], u[4], u[5],
                                 (unsigned)dw_rd16(f.payload + 30));
                    }
                    switch (f.payload[32]) {
                    case PTP1_CTRL_SYNC:       handle_sync(&f);       break;
                    case PTP1_CTRL_FOLLOWUP:   handle_followup(&f);   break;
                    case PTP1_CTRL_DELAY_RESP: handle_delay_resp(&f); break;
                    default:                   g_ptpv1.rx_other++;    break;
                    }
                }
            }
        }

        int64_t now = esp_timer_get_time();
        {
            static int64_t last_mon, last_ptp, last_cpu;
            if (tracing() && now - last_mon >= 1000000) {
                eth_ts_time_t t;
                if (eth_ts_get_time(&t) == ESP_OK) {
                    int64_t ptp = ts_ns(t.seconds, t.nanoseconds);
                    int64_t cpu = esp_timer_get_time() * 1000;
                    if (last_mon)
                        ESP_LOGI("trace", "CLK dptp-dcpu=%lld ns over %lld ms  rate=%d",
                                 (long long)((ptp - last_ptp) - (cpu - last_cpu)),
                                 (long long)((cpu - last_cpu) / 1000000),
                                 (int)eth_ts_get_rate_ppb());
                    last_ptp = ptp; last_cpu = cpu;
                }
                last_mon = now;
            }
        }
        if (g_ptpv1.have_master && now >= next_dreq_us) {
            send_delay_req();
            uint32_t period = (s_path_delay_n >= PD_MIN_SAMPLES)
                            ? DELAY_REQ_MS : DELAY_REQ_FAST_MS;
            next_dreq_us = now + (int64_t)period * 1000;
        }
    }
}

esp_err_t ptpv1_start(void)
{
    memset(&g_ptpv1, 0, sizeof(g_ptpv1));
    memcpy(s_uuid, eth_ts_mac(), 6);
    ptp_servo_init(&s_servo, 7);

    s_q = xQueueCreate(32, sizeof(eth_ts_ptp_frame_t));
    if (!s_q) return ESP_ERR_NO_MEM;

    eth_ts_register_ptp_cb(on_ptp_frame);
    eth_ts_mcast_allow(PTP1_GROUP_IP);

    BaseType_t ok = xTaskCreatePinnedToCore(ptp_task, "ptpv1", 4096, NULL,
                                            AP_PRIO_PTP, NULL, AP_CORE_AUDIO);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
