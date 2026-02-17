/**
 * flight_fsm.c -- Flight state machine with sensor-driven transitions.
 *
 * Implements debounced transitions for all flight phases:
 *   PAD -> BOOST -> COAST/COAST_1 -> (optional SUSTAIN -> COAST_2)
 *   -> APOGEE (transient) -> DROGUE -> MAIN -> LANDED
 *
 * Preserves sim flight behavior from Level 1.
 */

#ifndef HOST_TEST
#include "stm32h7xx_hal.h"
#else
/* HAL_GetTick provided by test_shim.h */
extern uint32_t HAL_GetTick(void);
#endif

#include "flight_fsm.h"
#include "flight_constants.h"
#include <math.h>
#include <string.h>

/* Forward declaration -- implemented in tlm_manager.c (stubbed in test_shim.h) */
extern int tlm_queue_event(uint8_t type, uint16_t data);

/* ── Internal state ─────────────────────────────────────────────── */
static flight_context_t s_ctx;

/* Simulated flight */
static bool     s_sim_active;
static uint32_t s_sim_start_ms;

/* ── Sim flight profile (time_s, alt_m, vel_mps) ─────────────── */
typedef struct {
    float time_s;
    float alt_m;
    float vel_mps;
    fsm_state_t state;
} sim_waypoint_t;

static const sim_waypoint_t s_sim_profile[] = {
    {   0.0f,    0.0f,    0.0f, FSM_STATE_PAD     },
    {   0.5f,    5.0f,   50.0f, FSM_STATE_BOOST   },
    {   3.0f,  600.0f,  300.0f, FSM_STATE_BOOST   },
    {   3.5f,  750.0f,  250.0f, FSM_STATE_COAST   },
    {  12.0f, 5150.0f,    0.0f, FSM_STATE_APOGEE  },
    {  12.5f, 5100.0f,  -20.0f, FSM_STATE_DROGUE  },
    {  45.0f,  300.0f,   -8.0f, FSM_STATE_MAIN    },
    {  90.0f,   10.0f,   -5.0f, FSM_STATE_RECOVERY},
    { 180.0f,    0.0f,    0.0f, FSM_STATE_LANDED  },
};
#define SIM_PROFILE_LEN  (sizeof(s_sim_profile) / sizeof(s_sim_profile[0]))

/* ── Helpers ────────────────────────────────────────────────────── */
static void transition_to(fsm_state_t new_state, uint32_t now_ms)
{
    if (new_state == s_ctx.state) {
        return;
    }
    s_ctx.state = new_state;
    s_ctx.state_entry_ms = now_ms;
    tlm_queue_event(FC_EVT_STATE, (uint16_t)new_state);
    if (new_state == FSM_STATE_APOGEE) {
        tlm_queue_event(FC_EVT_APOGEE, 0);
    }
}

static float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

/* ── Sensor-driven tick logic ───────────────────────────────────── */
static void fsm_sensor_tick(const sensor_input_t *input)
{
    uint32_t now = input->timestamp_ms;
    uint32_t time_in_state = now - s_ctx.state_entry_ms;

    switch (s_ctx.state) {

    /* ── PAD -> BOOST ────────────────────────────────────── */
    case FSM_STATE_PAD: {
        bool condition = (input->accel_body_ms2[2] > LAUNCH_ACCEL_THRESHOLD_MS2) &&
                         (input->ekf_vel_mps > 0.0f);
        if (condition) {
            if (!s_ctx.launch_debouncing) {
                s_ctx.launch_debouncing = true;
                s_ctx.launch_debounce_start_ms = now;
            } else if (now - s_ctx.launch_debounce_start_ms >= LAUNCH_DEBOUNCE_MS) {
                s_ctx.launch_debouncing = false;
                s_ctx.launch_detected = true;
                s_ctx.launch_timestamp_ms = now;
                s_ctx.mission_started = true;
                s_ctx.mission_start_ms = now;
                s_ctx.motor_count = 0;  /* first motor still burning */
                transition_to(FSM_STATE_BOOST, now);
            }
        } else {
            s_ctx.launch_debouncing = false;
        }
        break;
    }

    /* ── BOOST -> COAST / COAST_1 ────────────────────────── */
    case FSM_STATE_BOOST: {
        bool condition = (input->accel_body_ms2[2] < BURNOUT_ACCEL_THRESHOLD_MS2) &&
                         (time_in_state >= BURNOUT_MIN_BOOST_TIME_MS) &&
                         (input->ekf_vel_mps > BURNOUT_MIN_VELOCITY_MPS);
        if (condition) {
            if (!s_ctx.burnout_debouncing) {
                s_ctx.burnout_debouncing = true;
                s_ctx.burnout_debounce_start_ms = now;
            } else if (now - s_ctx.burnout_debounce_start_ms >= BURNOUT_DEBOUNCE_MS) {
                s_ctx.burnout_debouncing = false;
                s_ctx.motor_count++;
                s_ctx.last_burnout_ms = now;
                tlm_queue_event(FC_EVT_BURNOUT, (uint16_t)s_ctx.motor_count);
                if (s_ctx.motors_expected >= 2) {
                    transition_to(FSM_STATE_COAST_1, now);
                } else {
                    transition_to(FSM_STATE_COAST, now);
                }
            }
        } else {
            s_ctx.burnout_debouncing = false;
        }
        break;
    }

    /* ── COAST_1 -> SUSTAIN (multi-stage staging) ────────── */
    case FSM_STATE_COAST_1: {
        /* Check staging first */
        bool staging_condition = (input->accel_body_ms2[2] > STAGING_ACCEL_THRESHOLD_MS2) &&
                                (s_ctx.motors_expected >= 2) &&
                                (s_ctx.motor_count == 1);
        if (staging_condition) {
            if (!s_ctx.burnout_debouncing) {
                s_ctx.burnout_debouncing = true;
                s_ctx.burnout_debounce_start_ms = now;
            } else if (now - s_ctx.burnout_debounce_start_ms >= STAGING_DEBOUNCE_MS) {
                s_ctx.burnout_debouncing = false;
                tlm_queue_event(FC_EVT_STAGING, 0);
                transition_to(FSM_STATE_SUSTAIN, now);
                break;
            }
        } else {
            s_ctx.burnout_debouncing = false;
        }
        /* Fall through to apogee check (COAST_1 is a coast state) */
        goto check_apogee;
    }

    /* ── SUSTAIN -> COAST_2 ──────────────────────────────── */
    case FSM_STATE_SUSTAIN: {
        bool condition = (input->accel_body_ms2[2] < BURNOUT_ACCEL_THRESHOLD_MS2) &&
                         (time_in_state >= BURNOUT_MIN_BOOST_TIME_MS) &&
                         (input->ekf_vel_mps > BURNOUT_MIN_VELOCITY_MPS);
        if (condition) {
            if (!s_ctx.burnout_debouncing) {
                s_ctx.burnout_debouncing = true;
                s_ctx.burnout_debounce_start_ms = now;
            } else if (now - s_ctx.burnout_debounce_start_ms >= BURNOUT_DEBOUNCE_MS) {
                s_ctx.burnout_debouncing = false;
                s_ctx.motor_count++;
                s_ctx.last_burnout_ms = now;
                tlm_queue_event(FC_EVT_BURNOUT, (uint16_t)s_ctx.motor_count);
                transition_to(FSM_STATE_COAST_2, now);
            }
        } else {
            s_ctx.burnout_debouncing = false;
        }
        break;
    }

    /* ── COAST / COAST_2 -> APOGEE (via apogee voting) ───── */
    case FSM_STATE_COAST:
    case FSM_STATE_COAST_2:
    check_apogee: {
        /* Update peak trackers */
        if (input->ekf_alt_m > s_ctx.max_ekf_alt_m) {
            s_ctx.max_ekf_alt_m = input->ekf_alt_m;
        }
        if (input->baro_alt_agl_m > s_ctx.max_baro_alt_m) {
            s_ctx.max_baro_alt_m = input->baro_alt_agl_m;
        }

        /* 2-of-3 voting */
        uint8_t votes = 0;
        if (input->ekf_vel_mps < APOGEE_VEL_THRESHOLD_MPS)
            votes++;
        if (input->ekf_alt_m < s_ctx.max_ekf_alt_m - APOGEE_ALT_MARGIN_M)
            votes++;
        if (input->baro_alt_agl_m < s_ctx.max_baro_alt_m - APOGEE_ALT_MARGIN_M)
            votes++;

        bool condition = (votes >= 2) &&
                         (time_in_state >= APOGEE_MIN_COAST_TIME_MS) &&
                         (s_ctx.max_ekf_alt_m > APOGEE_MIN_ALTITUDE_M);

        if (condition) {
            if (!s_ctx.apogee_debouncing) {
                s_ctx.apogee_debouncing = true;
                s_ctx.apogee_debounce_start_ms = now;
            } else if (now - s_ctx.apogee_debounce_start_ms >= APOGEE_DEBOUNCE_MS) {
                s_ctx.apogee_debouncing = false;
                s_ctx.apogee_detected = true;
                s_ctx.apogee_timestamp_ms = now;
                /* APOGEE is transient: immediately transition to DROGUE */
                transition_to(FSM_STATE_APOGEE, now);
                transition_to(FSM_STATE_DROGUE, now);
            }
        } else {
            s_ctx.apogee_debouncing = false;
        }
        break;
    }

    /* ── APOGEE: transient, should not dwell ─────────────── */
    case FSM_STATE_APOGEE:
        /* Immediately transition to DROGUE (should already be done) */
        transition_to(FSM_STATE_DROGUE, now);
        break;

    /* ── DROGUE / MAIN -> LANDED ─────────────────────────── */
    case FSM_STATE_DROGUE:
    case FSM_STATE_MAIN: {
        /* Guard: time since apogee */
        if (!s_ctx.apogee_detected) {
            break;
        }
        if (now - s_ctx.apogee_timestamp_ms < LANDING_MIN_TIME_AFTER_APOGEE_MS) {
            break;
        }

        float vel_abs = input->ekf_vel_mps;
        if (vel_abs < 0.0f) vel_abs = -vel_abs;

        bool condition = (vel_abs < LANDING_VEL_THRESHOLD_MPS);

        if (condition) {
            if (!s_ctx.landing_debouncing) {
                s_ctx.landing_debouncing = true;
                s_ctx.landing_debounce_start_ms = now;
                s_ctx.landing_ref_alt_m = input->ekf_alt_m;
            } else {
                /* Check altitude stability relative to reference */
                float alt_diff = input->ekf_alt_m - s_ctx.landing_ref_alt_m;
                if (alt_diff < 0.0f) alt_diff = -alt_diff;
                if (alt_diff > LANDING_ALT_STABLE_M) {
                    /* Altitude drifted too far -- reset */
                    s_ctx.landing_debouncing = false;
                } else if (now - s_ctx.landing_debounce_start_ms >= LANDING_DEBOUNCE_MS) {
                    s_ctx.landing_debouncing = false;
                    s_ctx.landing_timestamp_ms = now;
                    transition_to(FSM_STATE_LANDED, now);
                }
            }
        } else {
            s_ctx.landing_debouncing = false;
        }
        break;
    }

    /* ── RECOVERY / TUMBLE / LANDED: no transitions ──────── */
    case FSM_STATE_RECOVERY:
    case FSM_STATE_TUMBLE:
    case FSM_STATE_LANDED:
    default:
        break;
    }
}

/* ── Peak tracking (every tick, after transitions) ──────────────── */
static void update_peaks(const sensor_input_t *input)
{
    if (!s_ctx.mission_started) {
        return;
    }

    if (input->ekf_alt_m > s_ctx.max_altitude_m) {
        s_ctx.max_altitude_m = input->ekf_alt_m;
    }
    if (input->ekf_vel_mps > s_ctx.max_velocity_mps) {
        s_ctx.max_velocity_mps = input->ekf_vel_mps;
    }
    float accel_nose = input->accel_body_ms2[2];
    if (accel_nose > s_ctx.max_accel_mps2) {
        s_ctx.max_accel_mps2 = accel_nose;
    }
    if (input->tilt_deg > s_ctx.peak_tilt_deg) {
        s_ctx.peak_tilt_deg = input->tilt_deg;
    }
    /* Independent peak trackers for apogee voting (also updated in check_apogee) */
    if (input->ekf_alt_m > s_ctx.max_ekf_alt_m) {
        s_ctx.max_ekf_alt_m = input->ekf_alt_m;
    }
    if (input->baro_alt_agl_m > s_ctx.max_baro_alt_m) {
        s_ctx.max_baro_alt_m = input->baro_alt_agl_m;
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

void flight_fsm_init(uint8_t motors_expected)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = FSM_STATE_PAD;
    s_ctx.state_entry_ms = HAL_GetTick();
    s_ctx.motors_expected = motors_expected;
    s_sim_active = false;
    s_sim_start_ms = 0;
}

fsm_state_t flight_fsm_tick(const sensor_input_t *input)
{
    if (s_sim_active) {
        uint32_t now = HAL_GetTick();
        float sim_t = (float)(now - s_sim_start_ms) / 1000.0f;

        /* Find the target state for current sim time */
        fsm_state_t target = FSM_STATE_PAD;
        for (int i = (int)SIM_PROFILE_LEN - 1; i >= 0; i--) {
            if (sim_t >= s_sim_profile[i].time_s) {
                target = s_sim_profile[i].state;
                break;
            }
        }

        /* Transition if needed */
        if (target != s_ctx.state) {
            transition_to(target, now);
        }

        /* End sim when we reach LANDED and stay for 5s */
        if (s_ctx.state == FSM_STATE_LANDED) {
            float landed_time = sim_t - s_sim_profile[SIM_PROFILE_LEN - 1].time_s;
            if (landed_time > 5.0f) {
                s_sim_active = false;
            }
        }
    } else {
        /* Real sensor-driven transitions */
        fsm_sensor_tick(input);
        update_peaks(input);
    }

    return s_ctx.state;
}

const flight_context_t *flight_fsm_get_context(void)
{
    return &s_ctx;
}

fsm_state_t flight_fsm_get_state(void)
{
    return s_ctx.state;
}

float flight_fsm_get_time_s(void)
{
    if (!s_ctx.mission_started) {
        return 0.0f;
    }
    return (float)(HAL_GetTick() - s_ctx.mission_start_ms) / 1000.0f;
}

void flight_fsm_force_state(fsm_state_t new_state)
{
    if (new_state > FSM_STATE_LANDED) {
        return;
    }
    uint32_t now = HAL_GetTick();
    if (!s_ctx.mission_started && new_state != FSM_STATE_PAD) {
        s_ctx.mission_started = true;
        s_ctx.mission_start_ms = now;
    }
    transition_to(new_state, now);
}

void flight_fsm_notify_pyro_event(uint8_t pyro_role, uint32_t now_ms)
{
    /* pyro_role: 0=Apogee, 2=Main, 4=Ignition */
    if (pyro_role == 2) {  /* PYRO_ROLE_MAIN */
        /* Transition DROGUE -> MAIN when main chute fires */
        if (s_ctx.state == FSM_STATE_DROGUE) {
            transition_to(FSM_STATE_MAIN, now_ms);
        }
    }
    /* PYRO_ROLE_IGNITION (4): no FSM action -- staging detected via accel */
    /* PYRO_ROLE_APOGEE (0): no FSM action -- already transitioned */
}

void flight_fsm_sim_start(void)
{
    uint32_t now = HAL_GetTick();
    s_sim_active = true;
    s_sim_start_ms = now;
    s_ctx.mission_started = true;
    s_ctx.mission_start_ms = now;
    transition_to(FSM_STATE_PAD, now);
}

void flight_fsm_sim_stop(void)
{
    s_sim_active = false;
    transition_to(FSM_STATE_PAD, HAL_GetTick());
}

bool flight_fsm_sim_active(void)
{
    return s_sim_active;
}

void flight_fsm_sim_get_state(fc_telem_state_t *out)
{
    if (!s_sim_active) {
        return;
    }

    float sim_t = (float)(HAL_GetTick() - s_sim_start_ms) / 1000.0f;

    /* Find bounding waypoints and interpolate */
    int lo = 0;
    int hi = (int)SIM_PROFILE_LEN - 1;

    for (int i = 0; i < (int)SIM_PROFILE_LEN - 1; i++) {
        if (sim_t >= s_sim_profile[i].time_s &&
            sim_t < s_sim_profile[i + 1].time_s) {
            lo = i;
            hi = i + 1;
            break;
        }
    }

    /* Clamp if past last waypoint */
    if (sim_t >= s_sim_profile[SIM_PROFILE_LEN - 1].time_s) {
        lo = (int)SIM_PROFILE_LEN - 1;
        hi = lo;
    }

    float t = 0.0f;
    if (lo != hi) {
        float dt = s_sim_profile[hi].time_s - s_sim_profile[lo].time_s;
        if (dt > 0.0f) {
            t = (sim_t - s_sim_profile[lo].time_s) / dt;
        }
    }

    out->alt_m   = lerp(s_sim_profile[lo].alt_m,   s_sim_profile[hi].alt_m,   t);
    out->vel_mps = lerp(s_sim_profile[lo].vel_mps, s_sim_profile[hi].vel_mps, t);
    out->flight_time_s = sim_t;

    /* Slowly rotating quaternion: yaw at 10 deg/s */
    float yaw_rad = sim_t * 10.0f * 0.0174532925f;
    float half_yaw = yaw_rad * 0.5f;
    out->quat[0] = cosf(half_yaw);  /* w */
    out->quat[1] = 0.0f;            /* x */
    out->quat[2] = 0.0f;            /* y */
    out->quat[3] = sinf(half_yaw);  /* z */
}

float flight_fsm_tilt_from_quat(const float q[4])
{
    float qx = q[1], qy = q[2];
    float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
    /* cos_tilt = R[2][2]. Body +Z (nose) in NED.
     * Nose up: cos_tilt = -1 -> tilt = 0 deg
     * Nose horizontal: cos_tilt = 0 -> tilt = 90 deg */
    float clamped = cos_tilt;
    if (clamped > 1.0f) clamped = 1.0f;
    if (clamped < -1.0f) clamped = -1.0f;
    float tilt_rad = acosf(-clamped);
    return tilt_rad * (180.0f / 3.14159265f);
}
