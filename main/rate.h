// The selected sample rate, and everything that follows from it.
//
// The rate is chosen ONCE at boot -- from AP_PIN_RATE_SEL, or pinned by
// -DSAMPLE_RATE -- and is then constant for the life of the run. Nothing here
// changes afterwards, so callers may cache the pointer.
//
// It is a runtime value rather than a #define for one reason: the same binary
// then serves both rates, so a board can be re-jumpered without a rebuild and
// a field unit cannot end up running firmware that disagrees with its own
// switch. Buffers are sized for 96 kHz in app_config.h and under-filled at
// 48 kHz; nothing is allocated from the rate.

#ifndef RATE_H
#define RATE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "app_config.h"

typedef struct {
    uint32_t hz;                // 48000 or 96000
    uint16_t fpp;               // frames per packet we request  (hz / 3000)
    uint16_t dma_frames;        // frames per DMA descriptor     (hz / 3000)
    uint16_t dma_depth_frames;  // dma_frames * AP_DMA_DESC_NUM
    uint32_t dma_depth_us;
    uint16_t scki_fs;           // 512 at 48k, 256 at 96k -- same Hz either way
    uint16_t bck_fs;            // 256, always
    uint8_t  pcm1690_fmt;       // 0x06 TDM, 0x08 high-speed TDM
    uint32_t latency_min_us;    // dma_depth_us + margin
    const char *label;
} rate_profile_t;

// Read the selection pin and latch the profile. Call this FIRST in app_main,
// before any audio or network hardware is configured.
// Call after nvs_flash_init(): a rate set from the controller lives in NVS.
esp_err_t rate_select(void);

// Rates this build can run: 48 and 96 kHz (SCKI = BCK; 96k via BCK divider 2, audio_out.c).
bool rate_supported(uint32_t hz);

// Store `hz` as the rate to boot at; takes effect after a restart.
esp_err_t rate_request(uint32_t hz);

const rate_profile_t *rate_get(void);

static inline uint32_t rate_hz(void) { return rate_get()->hz; }

// True when the rate came from -DSAMPLE_RATE rather than from the pin.
bool rate_is_pinned(void);

// What the pin actually read, for the boot banner and telemetry -- reported
// even when a build-time override ignored it, because "the switch says 96 and
// the firmware is running 48" is a confusing failure to debug from the outside.
int rate_pin_level(void);

#endif // RATE_H
