#include "rate.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "rate";

// Latency margin over the DMA depth.
//
// The playout pointer sits at (now - latency + dma_depth), so the ring is only
// read for samples that have had (latency - dma_depth) to arrive. Below that
// the device underruns on every block with PTP locked and every other counter
// healthy -- which is why this is derived and not a number someone picked.
#define LATENCY_MARGIN_US   500

static const rate_profile_t s_profiles[] = {
    {
        .hz = AP_RATE_48K,
        .fpp = AP_RATE_48K / AP_BLOCKS_PER_S,          // 16
        .dma_frames = AP_RATE_48K / AP_BLOCKS_PER_S,   // 16
        // SCKI = BCK = 256 fs and 24-bit HIGH-SPEED I2S TDM: the combination
        // verified on this breakout by ../stm32/ESP32-P4/p4-uac-pcm1690.
        .scki_fs = 256,                                 // 12.288 MHz, = BCK
        .bck_fs = AP_BCK_FS,                            // 12.288 MHz
        .pcm1690_fmt = 0x08,                            // 24-bit high-speed I2S TDM
        .label = "48 kHz",
    },
    {
        .hz = AP_RATE_96K,
        .fpp = AP_RATE_96K / AP_BLOCKS_PER_S,          // 32
        .dma_frames = AP_RATE_96K / AP_BLOCKS_PER_S,   // 32
        .scki_fs = 256,                                 // 24.576 MHz -- same!
        .bck_fs = AP_BCK_FS,                            // 24.576 MHz
        // 24-bit HIGH-SPEED I2S mode TDM. The only PCM1690 format that carries
        // 8 channels at 96 kHz on DIN1 alone; plain TDM moves to DIN1/2.
        .pcm1690_fmt = 0x08,
        .label = "96 kHz",
    },
};

static rate_profile_t s_current;
static int s_pin_level = -1;

const rate_profile_t *rate_get(void) { return &s_current; }
bool rate_is_pinned(void) { return AP_RATE_PINNED; }
int  rate_pin_level(void) { return s_pin_level; }

esp_err_t rate_select(void)
{
    uint32_t want = AP_RATE_DEFAULT;

#if AP_PIN_RATE_SEL >= 0
    // Pull-up, so an unfitted switch reads high and gives 48 kHz -- the
    // conservative bring-up rate, and the one whose BCK has 100% of timing
    // margin rather than 1.7%.
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << AP_PIN_RATE_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rate pin %d unusable (%s) -- falling back to %u Hz",
                 AP_PIN_RATE_SEL, esp_err_to_name(err), (unsigned)want);
    } else {
        // Settle: the pin was just switched to input with a weak pull-up, and
        // a long lead to a panel switch has capacitance.
        esp_rom_delay_us(1000);
        s_pin_level = gpio_get_level(AP_PIN_RATE_SEL);
        if (!AP_RATE_PINNED) {
            want = s_pin_level ? AP_RATE_48K : AP_RATE_96K;
        }
    }
#endif

    for (size_t i = 0; i < sizeof(s_profiles) / sizeof(s_profiles[0]); i++) {
        if (s_profiles[i].hz == want) {
            s_current = s_profiles[i];
            s_current.dma_depth_frames = s_current.dma_frames * AP_DMA_DESC_NUM;
            s_current.dma_depth_us =
                (uint32_t)((uint64_t)s_current.dma_depth_frames * 1000000 / s_current.hz);
            s_current.latency_min_us = s_current.dma_depth_us + LATENCY_MARGIN_US;

            ESP_LOGW(TAG, "===== %s =====", s_current.label);
            ESP_LOGI(TAG, "  source      %s%s",
                     AP_RATE_PINNED ? "build-time -DSAMPLE_RATE" : "selection pin",
                     (AP_RATE_PINNED && s_pin_level >= 0) ? " (PIN IGNORED)" : "");
#if AP_PIN_RATE_SEL >= 0
            ESP_LOGI(TAG, "  pin %-3d     %s -> %s", AP_PIN_RATE_SEL,
                     s_pin_level < 0 ? "unread" : (s_pin_level ? "high/open" : "low/closed"),
                     s_pin_level < 0 ? "-" : (s_pin_level ? "48 kHz" : "96 kHz"));
#endif
            ESP_LOGI(TAG, "  BCK         %d fs = %.3f MHz",
                     s_current.bck_fs, s_current.bck_fs * (double)s_current.hz / 1e6);
            ESP_LOGI(TAG, "  SCKI        %d fs = %.3f MHz (fixed, aux I2S)",
                     s_current.scki_fs, s_current.scki_fs * (double)s_current.hz / 1e6);
            ESP_LOGI(TAG, "  PCM1690 fmt 0x%02x (%s TDM)", s_current.pcm1690_fmt,
                     s_current.pcm1690_fmt == 0x08 ? "24-bit high-speed I2S" : "24-bit I2S");
            ESP_LOGI(TAG, "  fpp %u, %u pps, DMA %u x %u = %u frames / %u us",
                     s_current.fpp, (unsigned)(s_current.hz / s_current.fpp),
                     s_current.dma_frames, AP_DMA_DESC_NUM,
                     s_current.dma_depth_frames, (unsigned)s_current.dma_depth_us);
            ESP_LOGI(TAG, "  latency     %u..%u us",
                     (unsigned)s_current.latency_min_us, AP_LATENCY_US_MAX);

            // SCKI IS BCK (audio_out.c route_bck_to_scki), and the APLL must
            // reach 2 x mclk_multiple x fs within its 125 MHz ceiling. That
            // caps this design at 48 kHz: 96 kHz needs 196 MHz (the verified
            // ../stm32 UAC firmware documents the same ceiling).
            if (s_current.scki_fs != s_current.bck_fs ||
                2ULL * AP_I2S_MCLK_MULTIPLE * s_current.hz > 125000000ULL) {
                ESP_LOGE(TAG, "%s is not possible with SCKI = BCK from the APLL "
                              "(needs %.1f MHz, ceiling 125 MHz)", s_current.label,
                         2.0 * AP_I2S_MCLK_MULTIPLE * s_current.hz / 1e6);
                return ESP_ERR_INVALID_STATE;
            }
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "no profile for %u Hz", (unsigned)want);
    return ESP_ERR_INVALID_ARG;
}
