#ifndef APP_FSM_FSM_TYPES_H
#define APP_FSM_FSM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ── FSM Input Struct (FSM_TRANSITION_SPEC.md §2) ───────────────── */
typedef struct {
    /* From EKF */
    float alt_m;            /* Altitude AGL (m), from ekf.x[0]                */
    float vel_mps;          /* Vertical velocity (m/s), from ekf.x[1]         */
                            /*   positive = ascending, negative = descending   */

    /* From attitude estimator / IMU */
    float vert_accel_g;     /* Vertical acceleration (g), body-Z projected     */
                            /*   onto NED-down via attitude quaternion.         */
                            /*   Positive = upward accel (thrust), ~0 = free   */
                            /*   fall, negative = deceleration                 */
    bool  antenna_up;       /* True if rocket is upright (antenna pointing up) */
                            /*   Determined by Madgwick filter gravity vector: */
                            /*   body Z-axis within +/-30 deg of NED up       */

    /* Timing */
    float flight_time_s;    /* Seconds since mission start (0 if not launched) */

    /* Configuration (from MC upload) */
    float main_deploy_alt_m;     /* Main chute deployment altitude AGL (m)     */
    float drogue_fail_vel_mps;   /* Drogue failure descent rate threshold (m/s)*/
                                 /*   Stored positive. Compared against EKF    */
                                 /*   vertical velocity (negative = descending)*/
                                 /*   Triggers if vel_mps < -this_value        */
    float drogue_fail_time_s;    /* Sustain time for drogue failure detect (s) */
    uint8_t apogee_pyro_ch;      /* Pyro channel for apogee/drogue (0-indexed) */
    uint8_t main_pyro_ch;        /* Pyro channel for main chute (0-indexed)    */
    uint16_t apogee_fire_dur_ms; /* Apogee pyro fire duration (ms)             */
    uint16_t main_fire_dur_ms;   /* Main pyro fire duration (ms)               */
} fsm_input_t;

/* ── Dwell Timer (FSM_TRANSITION_SPEC.md §3) ─────────────────────── */
typedef struct {
    uint32_t start_ms;      /* fsm_get_tick() when condition first met */
    bool     active;        /* condition currently sustained            */
} dwell_timer_t;

/* ── Threshold Constants (FSM_TRANSITION_SPEC.md §4.0) ───────────── */

/* Launch detection (PAD -> BOOST) */
#define FSM_LAUNCH_ACCEL_G          2.0f        /* Vertical accel threshold (g)   */
#define FSM_LAUNCH_ACCEL_DWELL_MS   100         /* Accel sustain time (ms)        */
#define FSM_LAUNCH_VEL_MPS          15.0f       /* EKF velocity threshold (m/s)   */

/* Burnout detection (BOOST -> COAST) */
#define FSM_BURNOUT_ACCEL_G         0.0f        /* Accel below this = burnout     */
#define FSM_BURNOUT_DWELL_MS        100         /* Sustain time (ms)              */

/* Sustain re-light (COAST -> BOOST) */
#define FSM_SUSTAIN_ACCEL_G         3.0f        /* Re-light accel threshold (g)   */
#define FSM_SUSTAIN_DWELL_MS        100         /* Sustain time (ms)              */

/* Apogee detection (COAST -> APOGEE) */
#define FSM_APOGEE_VEL_MPS          0.0f        /* Velocity <= this = apogee      */
#define FSM_APOGEE_VEL_DWELL_MS     25          /* Sustain time (ms)              */
#define FSM_APOGEE_MIN_FLIGHT_S     5.0f        /* Minimum flight time gate (s)   */

/* Landing detection (MAIN -> LANDED) */
#define FSM_LANDED_VEL_MPS          1.0f        /* |velocity| below this (m/s)    */
#define FSM_LANDED_ALT_DELTA_M      2.0f        /* Altitude change < this (m)     */
#define FSM_LANDED_DWELL_MS         3000        /* Sustain time (ms)              */

/* Low-power auto-timer (LANDED -> RECOVERY) */
#define FSM_LANDED_TO_LOWPOWER_S    300         /* 5 minutes after landing        */

/* ── Pyro Channel Exclusion Mask (FSM_TRANSITION_SPEC.md §6.2) ──── */
/* Set bit N to exclude channel N from auto-arm and auto-fire.        */
/* Example: 0x0C = exclude channels 2 and 3 (0-indexed).             */
#ifndef PYRO_EXCLUDE_MASK
#define PYRO_EXCLUDE_MASK   0x00    /* default: no channels excluded */
#endif

#endif /* APP_FSM_FSM_TYPES_H */
