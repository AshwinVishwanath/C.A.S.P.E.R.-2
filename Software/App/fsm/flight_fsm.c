#include "flight_fsm.h"
#include "fsm_util.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <string.h>

#include "pyro_manager.h"

/* Forward declaration — implemented in tlm_manager.c */
extern int  tlm_queue_event(uint8_t type, uint16_t data);

/* ── Internal state (spec S8) ──────────────────────────────────── */
static fsm_state_t s_state;
static uint32_t    s_state_entry_ms;
static uint32_t    s_mission_start_ms;
static bool        s_mission_started;

/* Sim + bench */
static bool        s_sim_active;
static uint32_t    s_sim_start_ms;
static bool        s_bench_mode;

/* Dwell timers (spec S8) */
static dwell_timer_t s_launch_accel_dwell;
static dwell_timer_t s_burnout_dwell;
static dwell_timer_t s_sustain_relight_dwell;
static dwell_timer_t s_apogee_vel_dwell;
static dwell_timer_t s_drogue_fail_dwell;
static dwell_timer_t s_landed_dwell;

/* Peak tracking */
static float   s_peak_accel_g;
static float   s_peak_alt_m;

/* Landing detection */
static float   s_landed_alt_ref;

/* Multi-stage */
static uint8_t s_stage_count;

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
    if (new_state == s_state) return;
    s_state = new_state;
    s_state_entry_ms = fsm_get_tick();
    tlm_queue_event(FC_EVT_STATE, (uint16_t)new_state);

    if (new_state == FSM_STATE_BOOST) {
        pyro_mgr_auto_arm_flight();
    }
}

static float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

/* ── Public API ─────────────────────────────────────────────────── */

void flight_fsm_init(void)
{
    s_state = FSM_STATE_PAD;
    s_state_entry_ms = fsm_get_tick();
    s_mission_start_ms = 0;
    s_mission_started = false;
    s_sim_active = false;
    s_sim_start_ms = 0;
    s_bench_mode = false;

    memset(&s_launch_accel_dwell, 0, sizeof(dwell_timer_t));
    memset(&s_burnout_dwell, 0, sizeof(dwell_timer_t));
    memset(&s_sustain_relight_dwell, 0, sizeof(dwell_timer_t));
    memset(&s_apogee_vel_dwell, 0, sizeof(dwell_timer_t));
    memset(&s_drogue_fail_dwell, 0, sizeof(dwell_timer_t));
    memset(&s_landed_dwell, 0, sizeof(dwell_timer_t));

    s_peak_accel_g = 0.0f;
    s_peak_alt_m = 0.0f;
    s_landed_alt_ref = 0.0f;
    s_stage_count = 0;
}

void flight_fsm_reset(void)
{
    flight_fsm_init();
}

fsm_state_t flight_fsm_tick(const fsm_input_t *in)
{
    /* Bench mode: hold state */
    if (s_bench_mode) return s_state;

    /* Sim mode: run scripted profile (uses internal timing, ignores 'in') */
    if (s_sim_active) {
        uint32_t now = fsm_get_tick();
        float sim_t = (float)(now - s_sim_start_ms) / 1000.0f;

        fsm_state_t target = FSM_STATE_PAD;
        for (int i = (int)SIM_PROFILE_LEN - 1; i >= 0; i--) {
            if (sim_t >= s_sim_profile[i].time_s) {
                target = s_sim_profile[i].state;
                break;
            }
        }
        if (target != s_state) transition_to(target);
        if (s_state == FSM_STATE_LANDED) {
            float landed_time = sim_t - s_sim_profile[SIM_PROFILE_LEN - 1].time_s;
            if (landed_time > 5.0f) s_sim_active = false;
        }
        return s_state;
    }

    /* ── Sensor-driven transitions ── */
    switch (s_state) {

    case FSM_STATE_PAD: {
        /* S4.1: antenna_up AND accel > 2g sustained 100ms AND vel > 15 */
        bool accel_sustained = dwell_check(&s_launch_accel_dwell,
                                            in->vert_accel_g > FSM_LAUNCH_ACCEL_G,
                                            FSM_LAUNCH_ACCEL_DWELL_MS);
        if (in->antenna_up && accel_sustained && in->vel_mps > FSM_LAUNCH_VEL_MPS) {
            s_mission_started = true;
            s_mission_start_ms = fsm_get_tick();
            s_peak_accel_g = 0.0f;  /* reset for BOOST tracking */
            transition_to(FSM_STATE_BOOST);
        }
        break;
    }

    case FSM_STATE_BOOST: {
        /* S4.2: Track peak accel, detect burnout */
        if (in->vert_accel_g > s_peak_accel_g)
            s_peak_accel_g = in->vert_accel_g;

        if (dwell_check(&s_burnout_dwell,
                         in->vert_accel_g < FSM_BURNOUT_ACCEL_G,
                         FSM_BURNOUT_DWELL_MS)) {
            tlm_queue_event(FC_EVT_BURNOUT,
                            (uint16_t)(s_peak_accel_g * 1000.0f));
            s_peak_alt_m = 0.0f;  /* reset for COAST tracking */
            transition_to(FSM_STATE_COAST);
        }
        break;
    }

    case FSM_STATE_COAST: {
        /* Track peak altitude */
        if (in->alt_m > s_peak_alt_m)
            s_peak_alt_m = in->alt_m;

        /* S4.4: Check sustain re-light FIRST */
        if (dwell_check(&s_sustain_relight_dwell,
                         in->vert_accel_g > FSM_SUSTAIN_ACCEL_G,
                         FSM_SUSTAIN_DWELL_MS)) {
            s_stage_count++;
            tlm_queue_event(FC_EVT_STAGING, (uint16_t)s_stage_count);
            s_peak_accel_g = 0.0f;  /* reset peak for new burn */
            transition_to(FSM_STATE_BOOST);
            break;  /* Do NOT evaluate apogee */
        }

        /* S4.3: Check apogee */
        if (in->flight_time_s > FSM_APOGEE_MIN_FLIGHT_S &&
            dwell_check(&s_apogee_vel_dwell,
                         in->vel_mps <= FSM_APOGEE_VEL_MPS,
                         FSM_APOGEE_VEL_DWELL_MS)) {
            tlm_queue_event(FC_EVT_APOGEE,
                            (uint16_t)(s_peak_alt_m / 10.0f));
            pyro_mgr_auto_fire(in->apogee_pyro_ch, in->apogee_fire_dur_ms);
            transition_to(FSM_STATE_APOGEE);
        }
        break;
    }

    case FSM_STATE_APOGEE: {
        /* S4.5: Normal path -- altitude deploy FIRST */
        if (in->alt_m <= in->main_deploy_alt_m) {
            pyro_mgr_auto_fire(in->main_pyro_ch, in->main_fire_dur_ms);
            transition_to(FSM_STATE_MAIN);
            break;
        }

        /* S4.6: Backup -- drogue failure detection */
        float drogue_fail_dwell_ms = in->drogue_fail_time_s * 1000.0f;
        if (dwell_check(&s_drogue_fail_dwell,
                         in->vel_mps < -(in->drogue_fail_vel_mps),
                         (uint32_t)drogue_fail_dwell_ms)) {
            tlm_queue_event(FC_EVT_ERROR, ERR_DROGUE_FAIL);
            pyro_mgr_auto_fire(in->main_pyro_ch, in->main_fire_dur_ms);
            transition_to(FSM_STATE_MAIN);
        }
        break;
    }

    case FSM_STATE_MAIN: {
        /* S4.7: Landing detection */
        bool vel_ok = fabsf(in->vel_mps) < FSM_LANDED_VEL_MPS;

        /* Capture altitude reference when dwell begins */
        if (vel_ok && !s_landed_dwell.active) {
            s_landed_alt_ref = in->alt_m;
        }

        bool alt_ok = fabsf(in->alt_m - s_landed_alt_ref) < FSM_LANDED_ALT_DELTA_M;

        if (dwell_check(&s_landed_dwell,
                         vel_ok && alt_ok,
                         FSM_LANDED_DWELL_MS)) {
            transition_to(FSM_STATE_LANDED);
            pyro_mgr_disarm_all();
        }
        break;
    }

    case FSM_STATE_LANDED: {
        /* S4.8: Auto-timer to recovery */
        if ((fsm_get_tick() - s_state_entry_ms) >= (uint32_t)(FSM_LANDED_TO_LOWPOWER_S * 1000)) {
            transition_to(FSM_STATE_RECOVERY);
        }
        break;
    }

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
    if (!s_mission_started) return 0.0f;
    return (float)(fsm_get_tick() - s_mission_start_ms) / 1000.0f;
}

void flight_fsm_force_state(fsm_state_t new_state)
{
    if (new_state > FSM_STATE_LANDED) {
        return;
    }
    if (!s_mission_started && new_state != FSM_STATE_PAD) {
        s_mission_started = true;
        s_mission_start_ms = fsm_get_tick();
    }
    transition_to(new_state);
}

void flight_fsm_sim_start(void)
{
    s_sim_active = true;
    s_sim_start_ms = fsm_get_tick();
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

    float sim_t = (float)(fsm_get_tick() - s_sim_start_ms) / 1000.0f;

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
    out->quat[2] = sinf(half_roll);  /* y -- nose axis */
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
