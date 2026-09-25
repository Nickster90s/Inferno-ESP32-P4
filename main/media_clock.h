// Media clock: the loop that makes the DAC convert the sample the network says
// it should be converting, and keeps doing it.
//
// One controller, one buffer. That constraint is the whole design.
// MCR_REPLACEMENT.md records what happens when it is violated: on the FPGA the
// USB wrapper's async feedback turned out to be a PI servo on ring level, so
// disciplining the NCO underneath it put TWO controllers on one buffer and
// produced underrun storms twice. Here there is exactly one actuator (the
// media clock) and one observable (playout phase). Do not add a second.

#ifndef MEDIA_CLOCK_H
#define MEDIA_CLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool     armed;            // servo is acting on the clock
    bool     anchored;         // playout pointer has been placed on the timeline
    int32_t  error_frames;     // last phase error (target - playout)
    int32_t  ppb_target;       // what the PI loop wants
    int32_t  ppb_applied;      // phase term after the slew limiter
    int32_t  ppb_ff;           // feed-forward: crystal error from the PTP servo
    int64_t  integral;
    uint32_t anchors;          // re-anchor count
    uint32_t reset_steps;      // error exceeded the band; pointer was stepped
    uint32_t dma_depth_frames;
    uint32_t latency_frames;
} mclk_state_t;

extern mclk_state_t g_mclk;

esp_err_t mclk_init(uint32_t dma_depth_frames);

// Absolute AoIP sample index corresponding to "now" on the PTP timeline.
// Returns false if PTP has never produced a usable time.
bool mclk_now_samples(uint64_t *out);

// Place the playout pointer on the timeline. Called once when PTP first locks,
// and again after any PTP phase step.
//
// A STEP AFTER THE ANCHOR IS NOT A SMALL PROBLEM. ptpv1.h: "the media clock is
// a free-running counter anchored ONCE to PTP, so a step after that anchor
// leaves it on the old timeline permanently -- audio that is out of sync from
// the moment the stream starts, with every counter healthy." That is why
// ptpv1 exports step_count and why this is called when it moves.
void mclk_anchor(void);

void mclk_arm(bool on);
bool mclk_is_armed(void);

// Latency, three numbers -- as a real device keeps them apart:
//   CONFIGURED  the device setting, chosen in the controller (0x1101), kept in
//               NVS. What the controller shows as "Device Latency"; anything
//               else it calls "custom".
//   FLOOR       what the transmitters we receive from ask for (their
//               advertised latency). A DVS asks for 4 ms or more.
//   EFFECTIVE   max(configured, floor, this build's minimum) -- what playout
//               actually runs at, and what a receive flow reports.
void     mclk_set_latency_us(uint32_t us);       // configured; persisted
uint32_t mclk_get_config_latency_us(void);
void     mclk_set_latency_floor_us(uint32_t us); // from the subscriber
uint32_t mclk_get_latency_us(void);              // effective

// BENCH ONLY: force the effective latency, ignoring configured and floor
// (still clamped to the minimum). 0 turns it off. Telemetry port 7779, "F<us>".
// For finding the real floor below what a transmitter such as a DVS demands.
void mclk_force_latency_us(uint32_t us);

// Called by audio_out's service task at AP_MCLK_UPDATE_HZ.
void mclk_tick(void);

#endif // MEDIA_CLOCK_H
