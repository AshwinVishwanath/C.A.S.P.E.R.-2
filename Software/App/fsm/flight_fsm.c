#include "flight_fsm.h"
#include "stm32h7xx_hal.h"
#include "pyro_manager.h"
#include <math.h>
#include <string.h>

/* Forward declaration — implemented in tlm_manager.c */
extern int tlm_queue_event(uint8_t type, uint16_t data);

/* ── Internal state ─────────────────────────────────────────────── */
static fsm_state_t s_state;
static uint32_t    s_state_entry_ms;
static uint32_t    s_mission_start_ms;
static bool        s_mission_started;

/* Simulated flight */
static bool        s_sim_active;
static uint32_t    s_sim_start_ms;

/* Bench test mode — holds FSM state, no auto-transitions */
static bool        s_bench_mode;

/* Launch detection (Item 10) */
static uint8_t s_lsm_launch_count;

/* Landing detection (Item 8) */
static float    s_land_baro_ref;
static uint32_t s_land_window_start_ms;
static bool     s_land_window_active;

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
static void transition_to(fsm_state_t new_state)
{
    if (new_state == s_state) {
        return;
    }
    s_state = new_state;
    s_state_entry_ms = HAL_GetTick();
    tlm_queue_event(FC_EVT_STATE, (uint16_t)new_state);
    if (new_state == FSM_STATE_APOGEE) {
        tlm_queue_event(FC_EVT_APOGEE, 0);
    }
}

static float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

/* ── Landing detection (Item 8) ──────────────────────────────────── */
#define LANDED_ALT_THRESH  5.0f    /* m — baro altitude change threshold */
#define LANDED_VEL_THRESH  1.0f    /* m/s — EKF velocity threshold       */
#define T_LANDED_SUSTAIN   5000    /* ms — 5 seconds sustained            */
#define LANDED_MIN_DESCENT 10000   /* ms — min time in descent state      */

static void check_landing(const fc_telem_state_t *state)
{
    uint32_t now = HAL_GetTick();

    /* Guard: must be in descent state for at least 10s */
    if ((now - s_state_entry_ms) < LANDED_MIN_DESCENT)
        return;

    bool baro_stable = fabsf(state->baro_alt_m - s_land_baro_ref) < LANDED_ALT_THRESH;
    bool vel_zero    = fabsf(state->vel_mps) < LANDED_VEL_THRESH;

    if (baro_stable && vel_zero) {
        if (!s_land_window_active) {
            s_land_window_active = true;
            s_land_window_start_ms = now;
            s_land_baro_ref = state->baro_alt_m;
        } else if ((now - s_land_window_start_ms) >= T_LANDED_SUSTAIN) {
            transition_to(FSM_STATE_LANDED);
            pyro_mgr_disarm_all();
        }
    } else {
        s_land_window_active = false;
        s_land_baro_ref = state->baro_alt_m;
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

void flight_fsm_init(void)
{
    s_state = FSM_STATE_PAD;
    s_state_entry_ms = HAL_GetTick();
    s_mission_start_ms = 0;
    s_mission_started = false;
    s_sim_active = false;
    s_sim_start_ms = 0;
    s_bench_mode = false;
    s_lsm_launch_count = 0;
    s_land_baro_ref = 0.0f;
    s_land_window_start_ms = 0;
    s_land_window_active = false;
}

fsm_state_t flight_fsm_tick(const fc_telem_state_t *state)
{
    /* Bench mode: hold state, no auto-transitions */
    if (s_bench_mode) return s_state;

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
        if (target != s_state) {
            transition_to(target);
        }

        /* End sim when we reach LANDED and stay for 5s */
        if (s_state == FSM_STATE_LANDED) {
            float landed_time = sim_t - s_sim_profile[SIM_PROFILE_LEN - 1].time_s;
            if (landed_time > 5.0f) {
                s_sim_active = false;
            }
        }

        return s_state;
    }

    /* ── Real sensor-based transitions ── */
    switch (s_state) {
    case FSM_STATE_PAD: {
        /* Item 10: Dual-sensor launch confirmation */
        bool lsm_trigger = (state->accel_mag_g > 3.0f);
        if (lsm_trigger)
            s_lsm_launch_count++;
        else
            s_lsm_launch_count = 0;

        bool lsm_confirmed = (s_lsm_launch_count >= 2);
        /* Degrade to LSM-only if ADXL not available */
        bool adxl_ok = !state->adxl_available || state->adxl_activity;

        if (lsm_confirmed && adxl_ok) {
            s_mission_started = true;
            s_mission_start_ms = HAL_GetTick();
            transition_to(FSM_STATE_BOOST);
        }
        break;
    }

    case FSM_STATE_DROGUE:
    case FSM_STATE_MAIN:
    case FSM_STATE_RECOVERY:
    case FSM_STATE_TUMBLE:
        /* Item 8: ZUPT-based landing detection */
        check_landing(state);
        break;

    default:
        break;
    }

    return s_state;
}

fsm_state_t flight_fsm_get_state(void)
{
    return s_state;
}

float flight_fsm_get_time_s(void)
{
    if (!s_mission_started) {
        return 0.0f;
    }
    return (float)(HAL_GetTick() - s_mission_start_ms) / 1000.0f;
}

void flight_fsm_force_state(fsm_state_t new_state)
{
    if (new_state > FSM_STATE_LANDED) {
        return;
    }
    if (!s_mission_started && new_state != FSM_STATE_PAD) {
        s_mission_started = true;
        s_mission_start_ms = HAL_GetTick();
    }
    transition_to(new_state);
}

void flight_fsm_sim_start(void)
{
    s_sim_active = true;
    s_sim_start_ms = HAL_GetTick();
    s_mission_started = true;
    s_mission_start_ms = s_sim_start_ms;
    transition_to(FSM_STATE_PAD);
}

void flight_fsm_sim_stop(void)
{
    s_sim_active = false;
    transition_to(FSM_STATE_PAD);
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

    /* Slowly rolling quaternion: spin about nose (body Y) at 10 deg/s */
    float roll_rad = sim_t * 10.0f * 0.0174532925f;
    float half_roll = roll_rad * 0.5f;
    out->quat[0] = cosf(half_roll);  /* w */
    out->quat[1] = 0.0f;             /* x */
    out->quat[2] = sinf(half_roll);  /* y — nose axis */
    out->quat[3] = 0.0f;             /* z */
}

void flight_fsm_set_bench_mode(bool on)
{
    s_bench_mode = on;
}

bool flight_fsm_bench_active(void)
{
    return s_bench_mode;
}
