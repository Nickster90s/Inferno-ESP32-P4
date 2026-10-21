// PI servo on PTP offset, with the median filter and the lock hysteresis.
//
// Gains and thresholds are lifted from FPGA project firmware/ptpv1.c,
// where they were tuned against a real AoIP Leader. They are a starting point
// here, not a guarantee: that loop drove a 52-bit TSU addend on a 57 MHz FPGA
// and this one drives the ESP32-P4 EMAC's addend. Expect to retune, and expect
// the telemetry to be how you do it.

#ifndef PTP_SERVO_H
#define PTP_SERVO_H

#include <stdint.h>
#include <stdbool.h>

// PTPv1 has NO correctionField -- 802.1AS transparent clocks add switch
// residence time and PTPv1 has no equivalent, so queueing delay lands directly
// in the offset measurement. Hundreds of ns to low microseconds through a
// switch, not tens of ns. The thresholds reflect that.
// Hysteresis wide enough for PTPv1 through a switch: no correction field, so
// queueing shows up as microseconds of offset jitter. At 2 us / 5 us x 4 the
// bench flapped (offset sitting at 1.5-1.9 us, a short burst over 5 us
// unlocked it, and relocking needed 8 Syncs under 2 us) -- Sync red in the
// controller and the latency display frozen while the audio was fine.
#define PS_LOCK_NS          3000
#define PS_UNLOCK_NS        10000
#define PS_LOCK_STREAK      8
#define PS_UNLOCK_STREAK    8
// LOW-PERCENTILE WINDOW instead of median + outlier gate. Queueing only ever
// makes a Sync LATE, i.e. the offset reads too HIGH; the true offset is the low
// edge of the scatter. On our 100 Mbit port 8% of Syncs land 1-10 us late (one
// small audio frame ahead of them) in runs of up to 11. The old gate judged
// samples against one filtered value; a spike that slipped into the median
// froze that value, every correct sample after it was then "an outlier", the
// servo ran blind for 15-25 s, the clock walked 8-20 us off and was "phase
// shifted" back with an overshoot -- bench 2026-10-13: 4 shifts and 5 lock
// losses in 3 h, one of them 9 s of late packets.
#define PS_WIN_LOCKED       32          // 8 s of Syncs while locked
#define PS_WIN_ACQ          8           // 2 s while acquiring (the rate is still off)
#define PS_WIN_RANK         1           // 2nd lowest: one early glitch cannot drag it
// A JUMP: the window's low edge itself beyond PS_JUMP_NS for PS_JUMP_N Syncs
// (2 s). Late Syncs cannot do that -- they never move the low edge -- so it is
// the path or the Leader's phase: at a flow start the Leader-to-us delay drops
// ~36 us and stays. Taken as one small phase step with rate and lock kept; fed
// to the PI instead it lost lock and swung the rate estimate by 5.8 ppm
// (simulated), and the audio clock follows that rate. Three within 30 s is not
// a jump but a clock that will not settle: unlock and re-acquire.
#define PS_JUMP_NS          8000
#define PS_JUMP_N           8
#define PS_JUMP_STRIKES     3
#define PS_STEP_NS          1000000LL   // 1 ms, per statime's step_threshold

typedef struct {
    bool     locked;
    int64_t  offset_ns;        // last raw offset
    int64_t  filtered_ns;      // low edge of the window
    int32_t  rate_ppb;         // the INTEGRAL only -- the rate estimate
    int64_t  integral_ns;
    uint32_t updates;
    uint32_t steps;

    // Window of offsets with the servo's own phase moves taken out, so that
    // samples seconds apart compare (see ptp_servo.c).
    int64_t  win[PS_WIN_LOCKED];
    uint8_t  win_pos, win_count;
    uint8_t  win_n4;           // window in use, x4 (it grows after lock)
    int64_t  comp_ns;          // phase the P term has moved the clock by
    int64_t  last_t_ns;
    uint8_t  lock_streak, unlock_streak;
    uint8_t  jump_run, jump_strikes;
    int64_t  last_jump_t_ns;
    uint32_t shifts;           // jumps corrected by a small step, lock kept

    // Frequency pre-estimate (see ptp_servo.c). Before steering, measure how
    // fast the offset drifts and seed the integrator with it.
    bool     est_done;
    uint8_t  est_n;
    int64_t  est_t0_ns, est_off0_ns;
    double   est_sx, est_sy, est_sxx, est_sxy;   // least-squares sums
    int32_t  applied_ppb;      // last value returned to the caller
} ptp_servo_t;

void    ptp_servo_init(ptp_servo_t *s);
// Forget the filter window: after the clock was stepped under it.
void    ptp_servo_clear_filter(ptp_servo_t *s);
void    ptp_servo_reset(ptp_servo_t *s);

// Feed one offset measurement. Returns the ppb the caller should apply to the
// clock's rate, and sets *step_ns non-zero when a step is called for: a coarse
// step (|offset| > PS_STEP_NS, lock dropped) or a small JUMP correction while
// locked (lock kept). Either way the caller should re-measure the path delay.
// `t_ns` is the local receive time of the Sync the offset came from.
int32_t ptp_servo_update(ptp_servo_t *s, int64_t offset_ns, int64_t t_ns,
                         int64_t *step_ns);

#endif // PTP_SERVO_H
