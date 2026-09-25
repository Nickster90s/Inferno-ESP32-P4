#include "rate.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "rate";

// Arrival margin on top of the DMA's lead (read-ahead + I2S FIFO).
//
// The DMA plays the ring directly (jitterbuf.h), so a packet stamped T only has
// to land before the DMA reads slot T + latency. It is sent fpp samples after
// T (0.33 ms at 48/fpp16 and 96/fpp32) and crosses the network and our
// receive path. 0.5 ms covers that for now; stage 2 (audio decoded in the EMAC
// task) is what lets it shrink.
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
        // 32-frame DMA blocks: 3 x 32 = 1.0 ms of DMA depth, 1.5 ms minimum
        // latency. 16-frame blocks (0.5 ms depth, which would allow a 1 ms
        // minimum) were tried and FAILED: the I2S task's slack drops to
        // 0.33 ms, emac_rx (priority 24) preempts it for ~0.2 ms through
        // receive bursts, and the bench showed I2S underruns, write stalls up
        // to 100 ms, and PTP losing lock. At 32 frames: 0 underruns.
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

// ---------------------------------------------------------------------------
// Rate chosen from the controller, kept in NVS. Precedence at boot:
//   -DSAMPLE_RATE (pinned)  >  NVS "rate" (set from the controller)  >  pin
// ---------------------------------------------------------------------------
#include "nvs.h"
#define RATE_NVS_NS   "rate"
#define RATE_NVS_KEY  "hz"

static uint32_t nvs_rate(void)
{
    nvs_handle_t h; uint32_t v = 0;
    if (nvs_open(RATE_NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
    if (nvs_get_u32(h, RATE_NVS_KEY, &v) != ESP_OK) v = 0;
    nvs_close(h);
    return v;
}

bool rate_supported(uint32_t hz)
{
    for (size_t i = 0; i < sizeof(s_profiles) / sizeof(s_profiles[0]); i++)
        if (s_profiles[i].hz == hz) return true;
    return false;
}

esp_err_t rate_request(uint32_t hz)
{
    if (AP_RATE_PINNED) return ESP_ERR_NOT_SUPPORTED;
    if (!rate_supported(hz)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(RATE_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_u32(h, RATE_NVS_KEY, hz);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static const char *s_source = "default";

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
            s_source = "selection pin";
        }
    }
#endif
    if (AP_RATE_PINNED) {
        s_source = "build-time -DSAMPLE_RATE";
    } else {
        uint32_t v = nvs_rate();
        if (v && rate_supported(v)) { want = v; s_source = "controller (NVS)"; }
    }

    for (size_t i = 0; i < sizeof(s_profiles) / sizeof(s_profiles[0]); i++) {
        if (s_profiles[i].hz == want) {
            s_current = s_profiles[i];
            s_current.dma_depth_frames = AP_DMA_AHEAD_FRAMES + AP_I2S_FIFO_FRAMES;
            s_current.dma_depth_us =
                (uint32_t)((uint64_t)s_current.dma_depth_frames * 1000000 / s_current.hz);
            s_current.latency_min_us = s_current.dma_depth_us + LATENCY_MARGIN_US;

            ESP_LOGW(TAG, "===== %s =====", s_current.label);
            ESP_LOGI(TAG, "  source      %s%s", s_source,
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
            ESP_LOGI(TAG, "  fpp %u, %u pps; DMA plays the ring: %u x %u frames, lead %u frames / %u us",
                     s_current.fpp, (unsigned)(s_current.hz / s_current.fpp),
                     (unsigned)(AP_RING_FRAMES / s_current.dma_frames), s_current.dma_frames,
                     s_current.dma_depth_frames, (unsigned)s_current.dma_depth_us);
            ESP_LOGI(TAG, "  latency     %u..%u us",
                     (unsigned)s_current.latency_min_us, AP_LATENCY_US_MAX);

            // SCKI IS BCK (audio_out.c route_bck_to_scki). 96 kHz is reached
            // with a BCK divider of 2 off the 48 kHz APLL -- see audio_out.c.
            if (s_current.scki_fs != s_current.bck_fs) {
                ESP_LOGE(TAG, "%s: SCKI must equal BCK", s_current.label);
                return ESP_ERR_INVALID_STATE;
            }
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "no profile for %u Hz", (unsigned)want);
    return ESP_ERR_INVALID_ARG;
}
