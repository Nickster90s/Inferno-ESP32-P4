// TI PCM1690 -- 8-channel 24-bit audio DAC, SPI or I2C control (detected),
// TDM data in.
//
// The part supports ten input formats; the one used here is 24-bit I2S-mode
// TDM at single rate, which carries all eight channels on DIN1 alone. That is
// why one ESP32-P4 I2S port is enough and no channel splitting is needed.
//
// Clocking: the PCM1690 is a SLAVE. SCKI, BCK and LRCK all come from the P4's
// APLL, so disciplining the APLL disciplines the conversion rate directly --
// there is no second oscillator anywhere in the audio path.

#ifndef PCM1690_H
#define PCM1690_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t pcm1690_init(void);

// Release reset and configure. Call only once MCLK/SCKI is already running.
esp_err_t pcm1690_start(void);

esp_err_t pcm1690_set_mute(bool mute);
esp_err_t pcm1690_write(uint8_t reg, uint8_t val);   // bench: one raw register write

// Log what each DAC pin actually reads (clock ~50%, static 0/100%); also
// written to out (if not NULL) as text. Returns the text length.
int pcm1690_pin_check(char *out, size_t n);
esp_err_t pcm1690_set_attenuation(uint8_t ch, uint8_t code);  // ch 0..7, 0xFF = 0 dB

#endif // PCM1690_H
