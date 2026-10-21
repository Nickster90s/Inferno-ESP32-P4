#include "ptp_servo.h"
#include <string.h>

// Gains: FPGA project firmware/ptpv1.c. Sync arrives at ~4 Hz on an AoIP
// network rather than gPTP's 8 Hz, which is why the integral gain is scaled
// down from the gPTP value.
//
// LOCKED gains. The FPGA's (kp 0.072 / 0.2 fast, ki 0.0009) held a thermally
// settled crystal, but this board's crystal WARMS UP after power-on and walks
// from ~34.5 to ~41.3 ppm over the first minute. Tracking a ramp that fast at
// ki 0.0009 needs a standing phase error of tens of us, far past the 5 us
// unlock threshold: the bench lost lock 2 s after first locking. Simulated
// against that warm-up with this bench's jitter: FPGA gains 16 lock losses in
// 8 runs, these 0, steady-state sd 0.57 us vs 0.45 us.
#define KP_NUM          300
#define KP_DEN          1000
#define KP_FAST_NUM     500         // when |offset| > 1 us
#define KI_NUM          20000
#define KI_DEN          1000000
#define INTEGRAL_MAX    100000000LL // +-100 ms

// ACQUISITION gains, used only while unlocked. The gains above are the FPGA's
// steady-state tuning and they are ~50x softer than linuxptp's (kp 0.7,
// ki 0.3/s): any error in the initial frequency seed parks the offset at
// bias/kp and the integrator walks it out with a ~60 s time constant. First
// bench runs: Sync red for 200-300 s. Simulated with this bench's measured
// jitter: 166 s with the soft gains alone,
// 10-20 s with these during acquisition and the soft ones once locked.
#define KP_ACQ_NUM      500         // /KP_DEN
#define KP_ACQ_FAST_NUM 700
#define KI_ACQ_NUM      40000       // /KI_DEN, per Sync
void ptp_servo_init(ptp_servo_t *s)
{
    memset(s, 0, sizeof(*s));
}

void ptp_servo_reset(ptp_servo_t *s)
{
    memset(s, 0, sizeof(*s));
}

void ptp_servo_clear_filter(ptp_servo_t *s)
{
    s->win_pos = 0;
    s->win_count = 0;
    s->comp_ns = 0;
}

// LOW-PERCENTILE FILTER (see ptp_servo.h for why not a median).
//
// The samples are stored with the servo's own phase correction taken out:
// comp_ns is the phase the P term (output minus rate estimate) has moved the
// clock by since the window was cleared, so `offset - comp` of a sample 8 s old
// compares with one from now. What it cannot take out is the rate estimate's
// own error, which is why the window is short while acquiring and grows only
// once locked (a ~1 ppm rate error right after lock is 1 us/s of ramp, and the
// low edge of a full 8 s window lags a rising ramp by all of it -- simulated:
// 10 us excursions after lock with the full window from the start, 3.7 us with
// the window growing over 24 s).
static int64_t window_push(ptp_servo_t *s, int64_t offset_ns)
{
    s->win[s->win_pos] = offset_ns - s->comp_ns;
    s->win_pos = (uint8_t)((s->win_pos + 1) % PS_WIN_LOCKED);
    if (s->win_count < PS_WIN_LOCKED) s->win_count++;

    if (s->locked) {
        if (s->win_n4 < PS_WIN_LOCKED * 4) s->win_n4++;   // +1 sample per 4 Syncs
    } else {
        s->win_n4 = PS_WIN_ACQ * 4;
    }
    uint8_t n = s->win_n4 / 4;
    if (n < PS_WIN_ACQ) n = PS_WIN_ACQ;
    if (n > s->win_count) n = s->win_count;

    // The n newest, sorted.
    int64_t tmp[PS_WIN_LOCKED] = {0};    // = {0}: -O2 cannot see n >= 1
    for (uint8_t i = 0; i < n; i++)
        tmp[i] = s->win[(s->win_pos + PS_WIN_LOCKED - 1 - i) % PS_WIN_LOCKED];
    for (uint8_t i = 1; i < n; i++) {
        int64_t k = tmp[i];
        int j = (int)i - 1;
        while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = k;
    }
    uint8_t r = n > PS_WIN_RANK ? PS_WIN_RANK : (uint8_t)(n - 1);
    return tmp[r] + s->comp_ns;
}

// FREQUENCY PRE-ESTIMATE.
//
// The first P4 bench run locked after ~4-5 minutes, with DC's Sync red all
// that time: the crystal is 43.5 ppm off, and the integrator had to climb
// there at KI's pace while the offset was worked down. linuxptp does what this
// does -- watch the offset drift for a couple of seconds WITHOUT steering, and
// the slope IS the frequency error. Seed the integrator with it and the P term
// only has a phase to remove.
//
// Relative to what is applied now, so it is equally valid after a step.
#define EST_MIN_NS      4000000000LL    // 4 s of drift (~16 Syncs)
#define EST_MIN_N       12

// Least-squares slope, not two end points: PTPv1 offsets through a switch
// jitter by several us (no correctionField), and a two-point slope over 2 s
// turned that into a 16 ppm mis-seed on the first try (59 vs 43.5 ppm).
static int32_t estimate(ptp_servo_t *s, int64_t offset_ns, int64_t t_ns)
{
    if (s->est_n == 0) {
        s->est_t0_ns  = t_ns;
        s->est_off0_ns = offset_ns;
        s->est_sx = s->est_sy = s->est_sxx = s->est_sxy = 0;
    }
    s->est_n++;
    double x = (double)(t_ns - s->est_t0_ns) * 1e-9;          // s
    double y = (double)(offset_ns - s->est_off0_ns);          // ns
    s->est_sx += x; s->est_sy += y; s->est_sxx += x * x; s->est_sxy += x * y;

    int64_t dt = t_ns - s->est_t0_ns;
    if (dt < EST_MIN_NS || s->est_n < EST_MIN_N) return s->applied_ppb;

    double n = s->est_n;
    double den = n * s->est_sxx - s->est_sx * s->est_sx;
    if (den <= 0) return s->applied_ppb;
    double slope = (n * s->est_sxy - s->est_sx * s->est_sy) / den;   // ns/s = ppb

    // offset = slave - master. Slave slow -> offset falls -> need +ppb.
    int64_t total = (int64_t)s->applied_ppb - (int64_t)slope;
    if (total >  100000) total =  100000;      // == the integral clamp below
    if (total < -100000) total = -100000;

    s->integral_ns = -(total * KI_DEN) / KI_ACQ_NUM;    // unlocked: ACQ gain
    s->rate_ppb    = (int32_t)total;
    s->est_done    = true;
    s->applied_ppb = (int32_t)total;
    return s->applied_ppb;
}

int32_t ptp_servo_update(ptp_servo_t *s, int64_t offset_ns, int64_t t_ns,
                         int64_t *step_ns)
{
    *step_ns = 0;
    s->offset_ns = offset_ns;

    // A large offset is a step, not something to servo out.
    if (offset_ns > PS_STEP_NS || offset_ns < -PS_STEP_NS) {
        *step_ns = -offset_ns;
        s->integral_ns = 0;
        ptp_servo_clear_filter(s);
        s->last_t_ns = 0;
        s->steps++;
        s->locked = false;
        s->lock_streak = 0;
        s->est_done = false;                // re-measure from the new phase
        s->est_n = 0;
        // DROP THE P TERM. After a step the phase it was correcting is gone,
        // and keeping it deadlocked the servo on the bench: a phase excursion
        // saturated the output at -200 ppm, the step kept that rate, the
        // clock then ran 240 ppm off and hit the 1 ms step threshold again
        // before the 4 s re-estimate could finish -- 52 steps, Sync red.
        // Fall back to the rate estimate (the integral), which is a frequency.
        s->applied_ppb = s->rate_ppb;
        return s->applied_ppb;
    }

    if (!s->est_done) return estimate(s, offset_ns, t_ns);

    // The phase the P term moved the clock by since the last Sync: what the
    // last output applied beyond the rate estimate, for that long.
    if (s->last_t_ns && t_ns > s->last_t_ns && t_ns - s->last_t_ns < 2000000000LL)
        s->comp_ns += ((int64_t)(s->applied_ppb - s->rate_ppb) * (t_ns - s->last_t_ns))
                      / 1000000000LL;
    s->last_t_ns = t_ns;

    int64_t f = window_push(s, offset_ns);

    s->filtered_ns = f;

    if (s->locked) {
        int64_t m = f < 0 ? -f : f;
        if (m > PS_JUMP_NS) {
            if (++s->jump_run < PS_JUMP_N) {
                s->applied_ppb = s->rate_ppb;   // hold the rate, keep it out of the PI
                return s->applied_ppb;
            }
            s->jump_run = 0;
            if (s->last_jump_t_ns && t_ns - s->last_jump_t_ns < 30000000000LL)
                s->jump_strikes++;
            else
                s->jump_strikes = 1;
            s->last_jump_t_ns = t_ns;
            if (s->jump_strikes < PS_JUMP_STRIKES) {
                *step_ns = -f;
                ptp_servo_clear_filter(s);
                s->shifts++;
                s->applied_ppb = s->rate_ppb;
                return s->applied_ppb;
            }
            s->jump_strikes = 0;
            s->locked = false;
            s->lock_streak = 0;
            s->integral_ns = -((int64_t)s->rate_ppb * KI_DEN) / KI_ACQ_NUM;
        } else {
            s->jump_run = 0;
        }
    }

    bool big = (f > 1000 || f < -1000);
    int64_t p_num  = s->locked ? (big ? KP_FAST_NUM : KP_NUM)
                               : (big ? KP_ACQ_FAST_NUM : KP_ACQ_NUM);
    int64_t ki_num = s->locked ? KI_NUM : KI_ACQ_NUM;
    s->integral_ns += f;
    if (s->integral_ns >  INTEGRAL_MAX) s->integral_ns =  INTEGRAL_MAX;
    if (s->integral_ns < -INTEGRAL_MAX) s->integral_ns = -INTEGRAL_MAX;

    // The integral alone is the RATE estimate. ptpv1.h is emphatic about why
    // this is exported separately: "the proportional term is phase correction,
    // and feeding it into an audio sample rate is audible wander, not drift
    // correction." Nothing downstream of here may use the P term.
    int64_t integral_ppb = -(s->integral_ns * ki_num) / KI_DEN;
    if (integral_ppb >  100000) integral_ppb =  100000;
    if (integral_ppb < -100000) integral_ppb = -100000;
    s->rate_ppb = (int32_t)integral_ppb;

    int64_t prop_ppb = -(f * p_num) / KP_DEN;
    // The P term is phase correction; +-50 ppm pulls 1 ms in within ~20 s and
    // cannot drag the clock to the output clamp on one bad measurement.
    if (prop_ppb >  50000) prop_ppb =  50000;
    if (prop_ppb < -50000) prop_ppb = -50000;
    int64_t out = integral_ppb + prop_ppb;
    if (out >  200000) out =  200000;
    if (out < -200000) out = -200000;

    int64_t mag = f < 0 ? -f : f;
    if (!s->locked) {
        if (mag < PS_LOCK_NS) {
            if (++s->lock_streak >= PS_LOCK_STREAK) {
                s->locked = true; s->unlock_streak = 0;
                // Gain switch: rescale so the rate estimate is continuous.
                s->integral_ns = -((int64_t)s->rate_ppb * KI_DEN) / KI_NUM;
            }
        } else s->lock_streak = 0;
    } else {
        if (mag > PS_UNLOCK_NS) {
            if (++s->unlock_streak >= PS_UNLOCK_STREAK) {
                s->locked = false; s->lock_streak = 0;
                s->integral_ns = -((int64_t)s->rate_ppb * KI_DEN) / KI_ACQ_NUM;
            }
        } else s->unlock_streak = 0;
    }

    s->updates++;
    s->applied_ppb = (int32_t)out;
    return (int32_t)out;
}
