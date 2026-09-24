// Streaming telemetry.
//
// TELEMETRY_AND_PTP.md is the reason this exists and is written FIRST rather
// than last. Verbatim from it:
//
//   "A snapshot cannot show a servo."
//   "NEVER hand-index the reply. Emit a version + field count, and have the
//    host parse by name. Two wrong conclusions came from stale offsets."
//   "Log STATE TRANSITIONS, not just levels. Levels hide events."
//   "Watch the reply size: growing the stats reply 200 -> 208 bytes silently
//    killed the port."
//
// Four wrong conclusions on the FPGA -- a reverted working fix, a misread
// packet rate, a 25x improvement credited to code that never executed, and a
// PTP lock cycling for hours unnoticed -- all traced to having no way to see
// inside the running device. The instrument was the bottleneck. Do not treat
// this file as optional scaffolding.

#ifndef TELEM_H
#define TELEM_H

#include <stdint.h>
#include "esp_err.h"

#define TELEM_T_PTP     1
#define TELEM_T_MCLK    2
#define TELEM_T_FLOW    3

#define TELEM_F_LOCKED       (1u << 0)
#define TELEM_F_NO_FOLLOWUP  (1u << 1)
#define TELEM_F_MISPAIR      (1u << 2)
#define TELEM_F_ANCHOR       (1u << 3)
#define TELEM_F_STEP         (1u << 4)

// Pushed from anywhere, including the audio task. Lock-free, drops on overflow
// rather than blocking -- telemetry must never be able to stall audio.
void telem_push(uint8_t type, uint16_t flags, int32_t a, int32_t b, int32_t c, int32_t d);

esp_err_t telem_start(void);

// Send the stream to this host. 0 disables.
void telem_set_dest(uint32_t ip, uint16_t port);

#endif // TELEM_H
