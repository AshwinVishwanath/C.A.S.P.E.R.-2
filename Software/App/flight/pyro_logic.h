#ifndef PYRO_LOGIC_H
#define PYRO_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_context.h"
#include "flight_constants.h"

/* -- Pyro channel roles ----------------------------------------- */
typedef enum {
    PYRO_ROLE_APOGEE   = 0,
    PYRO_ROLE_APOGEE_BACKUP = 1,
    PYRO_ROLE_MAIN     = 2,
    PYRO_ROLE_MAIN_BACKUP   = 3,
    PYRO_ROLE_IGNITION = 4,
    PYRO_ROLE_NONE     = 0xFF
} pyro_role_t;

/* -- Altitude source for trigger -------------------------------- */
typedef enum {
    ALT_SOURCE_EKF  = 0,
    ALT_SOURCE_BARO = 1
} alt_source_t;

/* -- Backup mode ------------------------------------------------ */
typedef enum {
    BACKUP_NONE     = 0,
    BACKUP_TIMER    = 1,   /* fire N seconds after primary */
    BACKUP_ALTITUDE = 2    /* fire at altitude */
} backup_mode_t;

/* -- Per-channel configuration ---------------------------------- */
typedef struct {
    pyro_role_t   role;
    alt_source_t  alt_source;
    float         deploy_alt_m;          /* for MAIN / MAIN_BACKUP */
    float         fire_delay_s;          /* delay after trigger (0 = immediate) */
    float         max_flight_angle_deg;  /* for IGNITION: max tilt before lockout */
    backup_mode_t backup_mode;
    float         backup_timer_s;        /* seconds after primary fires */
    float         backup_alt_m;          /* altitude for backup trigger */
    uint8_t       primary_channel;       /* 0-3: which channel is this one's primary (for backup) */
} pyro_channel_config_t;

/* -- Per-channel runtime state ---------------------------------- */
typedef struct {
    bool fired;                /* one-shot: once true, never re-evaluated */
    bool armed;                /* set by caller (from pyro_manager arm state) */
    bool continuity;           /* set by caller (from pyro_manager continuity) */
    bool window_open;          /* eval window currently open */
    bool window_closed_permanently; /* FSM moved past window */
    bool flight_angle_violated;     /* latching: once true, stays true (IGNITION only) */
    bool trigger_met;          /* trigger condition met (waiting for fire_delay) */
    uint32_t trigger_time_ms;  /* when trigger condition was first met */
    bool backup_started;       /* primary fired, backup timer running */
    uint32_t backup_start_ms;  /* when backup timer started */
} pyro_channel_state_t;

/* -- Action output ---------------------------------------------- */
typedef struct {
    bool fire;           /* true = fire this channel NOW */
    uint8_t channel;     /* 0-3: which physical channel */
    pyro_role_t role;    /* role for FSM notification */
} pyro_action_t;

/* -- Pyro logic context ----------------------------------------- */
#define PYRO_MAX_CHANNELS 4
#define PYRO_MAX_ACTIONS  4

typedef struct {
    pyro_channel_config_t config[PYRO_MAX_CHANNELS];
    pyro_channel_state_t  state[PYRO_MAX_CHANNELS];
    uint8_t num_channels;
    pyro_action_t actions[PYRO_MAX_ACTIONS];
    uint8_t num_actions;  /* filled by tick */
} pyro_logic_t;

/* -- API -------------------------------------------------------- */

/**
 * Initialize pyro logic with default safe config (no channels armed).
 */
void pyro_logic_init(pyro_logic_t *pl);

/**
 * Evaluate all channels for the current tick.
 * Pure logic -- does NOT call hardware. Fills pl->actions[].
 * Caller must iterate pl->actions[0..num_actions-1] and call:
 *   pyro_mgr_set_arm(ch, true)
 *   pyro_mgr_fire(ch)
 *   flight_fsm_notify_pyro_event(role, now_ms)
 *
 * fsm_state: current FSM state
 * ctx: flight context (for apogee_timestamp_ms, etc.)
 * input: current sensor data
 * now_ms: current timestamp
 */
void pyro_logic_tick(pyro_logic_t *pl, fsm_state_t fsm_state,
                     const flight_context_t *ctx, const sensor_input_t *input,
                     uint32_t now_ms);

#endif /* PYRO_LOGIC_H */
