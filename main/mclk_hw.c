#include "mclk_hw.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include <string.h>

static const char *TAG = "mclk_hw";

// ---------------------------------------------------------------------------
// APLL model
// ---------------------------------------------------------------------------
//
//   f_apll = f_xtal * (4 + SDM/2^16) / (2 * (odiv + 2))    (ESP32-P4, IDF 5.5
//   SDM    = sdm2 * 2^16 + sdm1 * 2^8 + sdm0                rtc_clk.c; sdm2 is 6 bits)
//
// Relative rate change per LSB of SDM:
//
//   df/f = dSDM / (4 * 2^16 + SDM)
//
// so with a typical SDM near 3.8e5 one LSB is about 1/644218 = 1.55 ppm.
// That is the number that makes dithering mandatory rather than an
// optimisation.

#define SDM_BASE_OFFSET   (4u << 16)      // the "4 +" term, in SDM units

// Commanded rate, and the actuator state.
static volatile int32_t  s_ppb;
static uint32_t          s_sdm_base;      // SDM word at nominal rate
static uint32_t          s_odiv;
static int64_t           s_target_q16;    // desired SDM, 16 fractional bits
static uint32_t          s_sdm_applied;
static int64_t           s_dither_acc;    // sigma-delta accumulator, Q16
static esp_timer_handle_t s_dither_timer;
static uint32_t          s_lsb_ppb = 1550;

// ---------------------------------------------------------------------------
// Low-level write
// ---------------------------------------------------------------------------
//
// IDF 5.5 on the P4 offers rtc_clk_apll_coeff_set(), but it re-runs the APLL
// calibration and busy-waits for it on every call -- not something to do from
// a 1 kHz ISR, and a recalibration is exactly the kind of event that could
// glitch the clock we are trying to keep clean. So only the DSDM fraction is
// written (below). A trim of tens of ppm stays well inside the VCO band the
// I2S driver's initial calibration picked.
//
// *** UNVERIFIED ON SILICON *** -- whether a DSDM change takes effect cleanly
// without recalibration is the first thing to scope at bring-up step 8.
//
// If this proves unusable, build with AP_MCLK_BACKEND=MCLK_BACKEND_EXTERNAL
// and drive an outboard generator from mclk_hw_ext_write() instead.

#if AP_MCLK_BACKEND == MCLK_BACKEND_APLL
#include "hal/clk_tree_ll.h"
#include "esp_private/regi2c_ctrl.h"

// Write ONLY the DSDM bytes that changed. clk_ll_apll_set_config() also
// rewrites the whole SDM_STOP register (0x09 then 0x49 -- a modulator stop and
// restart) and the output divider, six analog-bus writes per call. From a
// 1 kHz ISR on the audio core that ran the DAC 0.84% SLOW (47,596 frames/s
// measured against 48,000) and cost a third of the incoming audio packets on
// the first bench run with feed-forward. A dither step normally changes DSDM0
// alone: one write.
static uint32_t s_sdm_hw;       // what the DSDM registers hold now

static inline void apll_write_sdm(uint32_t sdm)
{
    uint32_t diff = sdm ^ s_sdm_hw;
    if (diff & 0x3F0000) REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM2, (sdm >> 16) & 0x3F);
    if (diff & 0x00FF00) REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM1, (sdm >> 8) & 0xFF);
    if (diff & 0x0000FF) REGI2C_WRITE_MASK(I2C_APLL, I2C_APLL_DSDM0, sdm & 0xFF);
    s_sdm_hw = sdm;
}
#else
// Hook for an external generator. Convert `sdm` (still expressed in the model
// above, so the servo and telemetry stay identical) into whatever your part
// wants -- an Si5351 fractional divider, or a DAC code on a VCXO's trim pin.
__attribute__((weak)) void mclk_hw_ext_write(uint32_t sdm) { (void)sdm; }
static inline void apll_write_sdm(uint32_t sdm) { mclk_hw_ext_write(sdm); }
#endif

// ---------------------------------------------------------------------------
// Sigma-delta render
// ---------------------------------------------------------------------------
//
// This is the port of the FPGA's nco_render(). The commanded rate is carried in
// fixed point; the integer part is written and the fraction is accumulated, so
// the LSB toggles at whatever duty makes the MEAN exact.
//
// FPGA project README.md records why this matters: the FPGA's servo "is
// structurally blind" to actuator rounding, because it measures the crystal
// against the Leader, not how coarsely that estimate is rendered into an
// integer. The loop reported itself converged, with zero trips and a healthy
// ring, while the clock ran permanently fast. Do not remove the dither and
// expect the servo to notice.

static void IRAM_ATTR dither_cb(void *arg)
{
    (void)arg;
    int64_t target = s_target_q16;
    int64_t whole  = target >> 16;
    int64_t frac   = target & 0xFFFF;

    s_dither_acc += frac;
    if (s_dither_acc >= 0x10000) {
        s_dither_acc -= 0x10000;
        whole += 1;
    }

    uint32_t sdm = (uint32_t)(whole < 0 ? 0 : (whole > 0x3FFFFF ? 0x3FFFFF : whole));
    if (sdm != s_sdm_applied) {
        s_sdm_applied = sdm;
        apll_write_sdm(sdm);
    }
}

// ---------------------------------------------------------------------------

void mclk_hw_set_ppb(int32_t ppb)
{
    if (ppb >  AP_MCLK_TOTAL_PPB_MAX) ppb =  AP_MCLK_TOTAL_PPB_MAX;
    if (ppb < -AP_MCLK_TOTAL_PPB_MAX) ppb = -AP_MCLK_TOTAL_PPB_MAX;
    s_ppb = ppb;

    // dSDM = ppb * (2^18 + SDM_base) / 1e9, carried with 16 fractional bits.
    int64_t span = (int64_t)SDM_BASE_OFFSET + s_sdm_base;
    int64_t delta_q16 = ((int64_t)ppb * span * 65536) / 1000000000LL;
    s_target_q16 = ((int64_t)s_sdm_base << 16) + delta_q16;
}

int32_t  mclk_hw_get_ppb(void)  { return s_ppb; }
uint32_t mclk_hw_lsb_ppb(void)  { return s_lsb_ppb; }

esp_err_t mclk_hw_init(void)
{
    // Read back what the I2S driver actually programmed. Doing it this way
    // rather than recomputing from the nominal rate means the servo trims the
    // clock that is really running, including whatever rounding the driver did.
    s_odiv     = 0;
    s_sdm_base = 0;

#if AP_MCLK_BACKEND == MCLK_BACKEND_APLL
    uint32_t o, s0, s1, s2;
    clk_ll_apll_get_config(&o, &s0, &s1, &s2);
    s_odiv     = o;
    s_sdm_base = (s2 << 16) | (s1 << 8) | s0;
    s_sdm_hw   = s_sdm_base;
#endif

    if (s_sdm_base == 0) {
        ESP_LOGE(TAG, "could not read back the APLL coefficients -- the media "
                      "clock has no actuator. See the PORTING POINT comments.");
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_lsb_ppb = (uint32_t)(1000000000ULL / (SDM_BASE_OFFSET + s_sdm_base));
    ESP_LOGI(TAG, "APLL odiv=%u sdm=%u  -> actuator LSB = %u ppb",
             (unsigned)s_odiv, (unsigned)s_sdm_base, (unsigned)s_lsb_ppb);
    if (s_lsb_ppb > 200) {
        ESP_LOGW(TAG, "actuator LSB is %u ppb; the dither is carrying the "
                      "accuracy. Expect ~%u ppb of rate wander at %d Hz.",
                 (unsigned)s_lsb_ppb, (unsigned)s_lsb_ppb, MCLK_DITHER_HZ);
    }

    s_target_q16  = ((int64_t)s_sdm_base) << 16;
    s_sdm_applied = s_sdm_base;
    s_dither_acc  = 0;

    const esp_timer_create_args_t args = {
        .callback = dither_cb,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "mclk_dither",
    };
    esp_err_t err = esp_timer_create(&args, &s_dither_timer);
    if (err != ESP_OK) return err;
    return esp_timer_start_periodic(s_dither_timer, 1000000 / MCLK_DITHER_HZ);
}
