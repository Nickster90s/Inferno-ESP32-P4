#include "media_clock.h"
#include "rate.h"
#include "mclk_hw.h"
#include "jitterbuf.h"
#include "ptpv1.h"
#include "eth_ts.h"
#include "app_config.h"
#include "telem.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "mclk";

mclk_state_t g_mclk;

static uint32_t s_latency_us = AP_LATENCY_US_DEFAULT;   // effective
static uint32_t s_cfg_us     = AP_LATENCY_US_DEFAULT;   // controller's setting
static uint32_t s_floor_us;                             // transmitters' demand

#include "nvs.h"
#define LAT_NVS_NS   "latency"
#define LAT_NVS_KEY  "us"

static uint32_t effective_us(void)
{
    uint32_t us = s_cfg_us > s_floor_us ? s_cfg_us : s_floor_us;
    if (us < rate_get()->latency_min_us) us = rate_get()->latency_min_us;
    if (us > AP_LATENCY_US_MAX) us = AP_LATENCY_US_MAX;
    return us;
}
static uint32_t s_block_count;
static uint32_t s_blocks_per_update;
static uint32_t s_last_ptp_steps;

esp_err_t mclk_init(uint32_t dma_depth_frames)
{
    g_mclk.dma_depth_frames = dma_depth_frames;
    nvs_handle_t h; uint32_t v;
    if (nvs_open(LAT_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u32(h, LAT_NVS_KEY, &v) == ESP_OK && v) s_cfg_us = v;
        nvs_close(h);
    }
    s_latency_us = effective_us();
    g_mclk.latency_frames   = (uint32_t)((uint64_t)s_latency_us * rate_hz() / 1000000);

    // One servo update every AP_MCLK_UPDATE_HZ; the audio task calls us once
    // per I2S block. Blocks/s is AP_BLOCKS_PER_S by construction at either
    // rate, but derive it rather than assume it.
    uint32_t blocks_per_s = rate_hz() / rate_get()->dma_frames;
    s_blocks_per_update = blocks_per_s / AP_MCLK_UPDATE_HZ;
    if (s_blocks_per_update == 0) s_blocks_per_update = 1;

    return mclk_hw_init();
}

bool mclk_now_samples(uint64_t *out)
{
    eth_ts_time_t t;
    if (!g_ptpv1.have_master) return false;
    if (eth_ts_get_time(&t) != ESP_OK) return false;

    // AoIP's timestamp is (seconds, subsec_samples) on the SAME timeline PTP
    // carries, so the conversion is exact arithmetic, not an estimate.
    uint32_t hz = rate_hz();
    *out = (uint64_t)t.seconds * hz
         + (uint64_t)t.nanoseconds * hz / 1000000000ULL;
    return true;
}

static void apply_latency(void)
{
    uint32_t us = effective_us();
    if (us == s_latency_us) return;          // no re-anchor, no glitch
    s_latency_us = us;
    g_mclk.latency_frames = (uint32_t)((uint64_t)us * rate_hz() / 1000000);
    ESP_LOGI(TAG, "latency -> %u us (%u frames; configured %u, floor %u)",
             (unsigned)us, (unsigned)g_mclk.latency_frames,
             (unsigned)s_cfg_us, (unsigned)s_floor_us);
    mclk_anchor();
}

void mclk_set_latency_us(uint32_t us)
{
    if (us > AP_LATENCY_US_MAX) us = AP_LATENCY_US_MAX;
    if (us != s_cfg_us) {
        s_cfg_us = us;
        nvs_handle_t h;
        if (nvs_open(LAT_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u32(h, LAT_NVS_KEY, us);
            nvs_commit(h);
            nvs_close(h);
        }
    }
    apply_latency();
}

void mclk_set_latency_floor_us(uint32_t us)
{
    s_floor_us = us;
    apply_latency();
}

uint32_t mclk_get_config_latency_us(void) { return s_cfg_us; }

uint32_t mclk_get_latency_us(void) { return s_latency_us; }

void mclk_anchor(void)
{
    uint64_t now;
    if (!mclk_now_samples(&now)) return;

    // The frame handed to DMA now is converted dma_depth frames later, so the
    // pointer sits that much AHEAD of the sample the DAC is emitting. Constant
    // offset, folded into the target once here rather than in the error term.
    uint64_t playout = now - g_mclk.latency_frames + g_mclk.dma_depth_frames;
    jb_reset(playout);

    g_mclk.integral = 0;
    g_mclk.anchored = true;
    g_mclk.anchors++;
    s_last_ptp_steps = g_ptpv1.step_count;
    ESP_LOGI(TAG, "anchor #%u at sample %llu (latency %u frames)",
             (unsigned)g_mclk.anchors, (unsigned long long)playout,
             (unsigned)g_mclk.latency_frames);
    telem_push(TELEM_T_MCLK, TELEM_F_ANCHOR, (int32_t)g_mclk.anchors, 0, 0, 0);
}

// Actuator = feed-forward (crystal error, from PTP) + slewed phase term.
static void apply(void)
{
    mclk_hw_set_ppb(g_mclk.ppb_ff + g_mclk.ppb_applied);
}

void mclk_arm(bool on)
{
    g_mclk.armed = on;
    if (!on) { g_mclk.ppb_applied = 0; apply(); }
    ESP_LOGI(TAG, "discipline %s", on ? "ARMED" : "disarmed");
}

bool mclk_is_armed(void) { return g_mclk.armed; }

void mclk_tick(void)
{
    if (++s_block_count < s_blocks_per_update) return;
    s_block_count = 0;

    if (!g_ptpv1.locked) {
        // PTP is the timeline. Without it the error term is meaningless, so
        // hold the last rate rather than servoing on noise.
        return;
    }

    // Feed-forward runs whether or not the phase loop is armed: disarmed now
    // means "rate-matched to the Leader, phase free", not "raw crystal".
    int32_t ff = g_ptpv1.rate_ppb;
    if (ff >  AP_MCLK_FF_PPB_MAX) ff =  AP_MCLK_FF_PPB_MAX;
    if (ff < -AP_MCLK_FF_PPB_MAX) ff = -AP_MCLK_FF_PPB_MAX;
    g_mclk.ppb_ff = ff;
    apply();

    // A PTP phase step moved absolute time out from under the anchor.
    if (g_ptpv1.step_count != s_last_ptp_steps) {
        ESP_LOGW(TAG, "PTP stepped (%u -> %u) -- re-anchoring",
                 (unsigned)s_last_ptp_steps, (unsigned)g_ptpv1.step_count);
        mclk_anchor();
        return;
    }

    if (!g_mclk.anchored) { mclk_anchor(); return; }

    uint64_t now;
    if (!mclk_now_samples(&now)) return;

    uint64_t target  = now - g_mclk.latency_frames + g_mclk.dma_depth_frames;
    uint64_t playout = jb_playout_idx();
    int64_t  err     = (int64_t)target - (int64_t)playout;   // signed, always

    g_mclk.error_frames = (int32_t)err;

    if (err > AP_MCLK_RESET_FRAMES || err < -AP_MCLK_RESET_FRAMES) {
        // Beyond anything the rate loop can walk back in reasonable time.
        // Stepping is audible; if this fires in steady state the fault is
        // upstream of the servo, so it is counted and reported, never silent.
        ESP_LOGW(TAG, "phase error %lld frames exceeds band -- stepping playout",
                 (long long)err);
        jb_step_playout(err);
        g_mclk.integral = 0;
        g_mclk.reset_steps++;
        telem_push(TELEM_T_MCLK, TELEM_F_STEP, (int32_t)err,
                   (int32_t)g_mclk.reset_steps, 0, 0);
        return;
    }

    if (!g_mclk.armed) {
        telem_push(TELEM_T_MCLK, 0, (int32_t)err, 0, g_mclk.ppb_ff, 0);
        return;
    }

    g_mclk.integral += err;
    if (g_mclk.integral >  1000000) g_mclk.integral =  1000000;
    if (g_mclk.integral < -1000000) g_mclk.integral = -1000000;

    int64_t ppb = (int64_t)err * AP_MCLK_KP_NUM
                + (g_mclk.integral * AP_MCLK_KI_NUM) / 1000;

    if (ppb >  AP_MCLK_PPB_CLAMP) ppb =  AP_MCLK_PPB_CLAMP;
    if (ppb < -AP_MCLK_PPB_CLAMP) ppb = -AP_MCLK_PPB_CLAMP;
    g_mclk.ppb_target = (int32_t)ppb;

    // SLEW LIMIT. This is the constraint whose absence cost two bench sessions
    // on the FPGA (MCR_REPLACEMENT.md): "both earlier attempts stepped the NCO
    // at main-loop rate and produced underrun storms... the missing constraint
    // was slew rate, not the choice of rate estimate."
    int32_t step_max = AP_MCLK_SLEW_PPB_PER_S / AP_MCLK_UPDATE_HZ;
    if (step_max < 1) step_max = 1;
    int32_t cur = g_mclk.ppb_applied;         // the PHASE term only
    int32_t d   = g_mclk.ppb_target - cur;
    if (d >  step_max) d =  step_max;
    if (d < -step_max) d = -step_max;

    g_mclk.ppb_applied = cur + d;
    apply();

    jb_stats_t jst;
    jb_get_stats(&jst);
    telem_push(TELEM_T_MCLK, TELEM_F_LOCKED, (int32_t)err, g_mclk.ppb_applied,
               jst.level_frames, (int32_t)jst.frames_underrun);
}
