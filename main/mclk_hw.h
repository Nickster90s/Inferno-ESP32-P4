// Media-clock actuator: the one knob that makes the DAC run at the network's
// idea of 48 kHz rather than its own.
//
// Two backends. Pick with AP_MCLK_BACKEND.
//
//   APLL      the ESP32-P4's audio PLL drives MCLK/BCK/LRCK and the PCM1690 is
//             the slave. Zero extra parts. Trimmed by dithering the PLL's
//             sigma-delta word.
//
//   EXTERNAL  an outboard generator (Si5351, or a VCXO + trim DAC) drives the
//             DAC's SCKI and the P4's I2S runs as a SLAVE off it. More parts,
//             far better jitter, and it sidesteps every APLL question below.
//
// WHY YOU MAY WANT THE EXTERNAL BACKEND, stated plainly:
//
// One LSB of the APLL sigma-delta word is worth roughly 1.5 ppm here -- about
// six times coarser than the FPGA's NCO increment, whose 242 ppb LSB was itself
// enough to produce a permanent +0.35 ppm drift and a receiver's latency
// indicator walking to grey (FPGA project DRIFT_HANDOFF.md). The fix there, and
// the fix here, is to sigma-delta the fractional part so the MEAN rate is exact
// -- see mclk_hw_render(). But the mean being exact does not make the
// instantaneous rate clean: the LSB toggles at MCLK_DITHER_HZ, so the clock
// carries ~1.5 ppm of square-wave rate dither. At 1 kHz that is ~1.5 ns of
// phase wander, which a jitter buffer does not care about and a good DAC does.
//
// You built a PCM1690 deliberately. If the measured output does not satisfy
// you, the answer is not a servo tweak -- it is this #define.

#ifndef MCLK_HW_H
#define MCLK_HW_H

#include <stdint.h>
#include "esp_err.h"

#define MCLK_BACKEND_APLL       0
#define MCLK_BACKEND_EXTERNAL   1

#ifndef AP_MCLK_BACKEND
#define AP_MCLK_BACKEND MCLK_BACKEND_APLL
#endif

// How fast the fractional part is dithered. Faster = less phase wander per
// step; too fast and the PLL is being written more often than it can settle.
#define MCLK_DITHER_HZ  1000

esp_err_t mclk_hw_init(void);

// Set the commanded rate offset. The servo calls this; nothing else should.
void mclk_hw_set_ppb(int32_t ppb);

int32_t mclk_hw_get_ppb(void);

// ppb per LSB of the actuator, for telemetry and for judging whether a
// residual drift is servo error or actuator quantisation. The FPGA project
// spent two sessions on exactly that distinction.
uint32_t mclk_hw_lsb_ppb(void);

#endif // MCLK_HW_H
