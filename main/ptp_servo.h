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

#define PS_MEDIAN_MAX       9

// PTPv1 has NO correctionField -- 802.1AS transparent clocks add switch
// residence time and PTPv1 has no equivalent, so queueing delay lands directly
// in the offset measurement. Hundreds of ns to low microseconds through a
// switch, not tens of ns. The thresholds reflect that.
#define PS_LOCK_NS          2000
#define PS_UNLOCK_NS        5000
#define PS_LOCK_STREAK      8
#define PS_UNLOCK_STREAK    4
#define PS_STEP_NS          1000000LL   // 1 ms, per statime's step_threshold

typedef struct {
    bool     locked;
    int64_t  offset_ns;        // last raw offset
    int64_t  filtered_ns;      // after the median
    int32_t  rate_ppb;         // the INTEGRAL only -- the rate estimate
    int64_t  integral_ns;
    uint32_t updates;
    uint32_t outliers;
    uint32_t steps;

    uint8_t  median_n;
    int64_t  median_buf[PS_MEDIAN_MAX];
    uint8_t  median_pos, median_count;
    uint8_t  lock_streak, unlock_streak;
    uint8_t  outlier_run;      // consecutive rejected samples
    int64_t  orun[4];          // the first SHIFT_N of them
    uint32_t shifts;           // phase shifts corrected by a small step

    // Frequency pre-estimate (see ptp_servo.c). Before steering, measure how
    // fast the offset drifts and seed the integrator with it.
    bool     est_done;
    uint8_t  est_n;
    int64_t  est_t0_ns, est_off0_ns;
    double   est_sx, est_sy, est_sxx, est_sxy;   // least-squares sums
    int32_t  applied_ppb;      // last value returned to the caller
} ptp_servo_t;

void    ptp_servo_init(ptp_servo_t *s, uint8_t median_n);
void    ptp_servo_reset(ptp_servo_t *s);

// Feed one offset measurement. Returns the ppb the caller should apply to the
// clock's rate, and sets *step_ns non-zero when a step is called for: a coarse
// step (|offset| > PS_STEP_NS, lock dropped) or a small PHASE SHIFT correction
// while locked (lock kept; the caller should re-measure the path delay).
// `t_ns` is the local receive time of the Sync the offset came from.
int32_t ptp_servo_update(ptp_servo_t *s, int64_t offset_ns, int64_t t_ns,
                         int64_t *step_ns);

#endif // PTP_SERVO_H
