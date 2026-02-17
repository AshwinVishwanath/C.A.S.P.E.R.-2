/*
 * pyro_logic.c -- Per-channel pyro trigger evaluation (Level 2)
 *
 * PURE LOGIC: no hardware calls, no HAL includes. All state lives in
 * pyro_logic_t passed by the caller. The caller (main.c) is responsible
 * for executing the returned fire actions via pyro_mgr_fire() and
 * flight_fsm_notify_pyro_event().
 */

#include "pyro_logic.h"
#include <string.h>

/* -- Helpers ---------------------------------------------------- */

/**
 * Get the altitude value for a channel based on its configured source.
 */
static float get_channel_altitude(const pyro_channel_config_t *config,
                                  const sensor_input_t *input)
{
    if (config->alt_source == ALT_SOURCE_BARO) {
        return input->baro_alt_agl_m;
    }
    return input->ekf_alt_m;
}

/**
 * Check if an FSM state is a coast variant (COAST, COAST_1, COAST_2).
 */
static bool is_coast_state(fsm_state_t s)
{
    return (s == FSM_STATE_COAST ||
            s == FSM_STATE_COAST_1 ||
            s == FSM_STATE_COAST_2);
}

/* -- Eval window logic ------------------------------------------ */

/**
 * Determine if the evaluation window is currently open for the given
 * role and FSM state. Also detect if the window has permanently closed
 * (FSM advanced past the window states).
 *
 * Returns true if window is open, false otherwise.
 * Sets *permanently_closed = true if the FSM has moved past the window.
 */
static bool eval_window_check(pyro_role_t role, fsm_state_t fsm_state,
                               bool *permanently_closed)
{
    *permanently_closed = false;

    switch (role) {
    case PYRO_ROLE_APOGEE:
        /* Open during COAST, COAST_1, COAST_2 */
        if (is_coast_state(fsm_state)) {
            return true;
        }
        /* If past coast states (APOGEE or later), window is permanently closed */
        if (fsm_state >= FSM_STATE_APOGEE) {
            *permanently_closed = true;
        }
        return false;

    case PYRO_ROLE_APOGEE_BACKUP:
        /* Open during COAST, COAST_1, COAST_2, DROGUE, MAIN states
         * (backup can fire after primary, which fires at apogee during descent) */
        if (is_coast_state(fsm_state) ||
            fsm_state == FSM_STATE_APOGEE ||
            fsm_state == FSM_STATE_DROGUE ||
            fsm_state == FSM_STATE_MAIN) {
            return true;
        }
        if (fsm_state >= FSM_STATE_LANDED) {
            *permanently_closed = true;
        }
        return false;

    case PYRO_ROLE_MAIN:
        /* Open during DROGUE, MAIN */
        if (fsm_state == FSM_STATE_DROGUE || fsm_state == FSM_STATE_MAIN) {
            return true;
        }
        if (fsm_state >= FSM_STATE_LANDED) {
            *permanently_closed = true;
        }
        return false;

    case PYRO_ROLE_MAIN_BACKUP:
        /* Open during DROGUE, MAIN */
        if (fsm_state == FSM_STATE_DROGUE || fsm_state == FSM_STATE_MAIN) {
            return true;
        }
        if (fsm_state >= FSM_STATE_LANDED) {
            *permanently_closed = true;
        }
        return false;

    case PYRO_ROLE_IGNITION:
        /* Open during COAST_1 ONLY */
        if (fsm_state == FSM_STATE_COAST_1) {
            return true;
        }
        /* If past COAST_1 (SUSTAIN, COAST_2, APOGEE, ...), permanently closed */
        if (fsm_state >= FSM_STATE_SUSTAIN) {
            *permanently_closed = true;
        }
        /* Also closed if we jumped from COAST_1 directly to APOGEE */
        if (fsm_state >= FSM_STATE_APOGEE) {
            *permanently_closed = true;
        }
        return false;

    case PYRO_ROLE_NONE:
    default:
        return false;
    }
}

/* -- Emit action ------------------------------------------------ */

static void emit_action(pyro_logic_t *pl, uint8_t channel, pyro_role_t role)
{
    if (pl->num_actions >= PYRO_MAX_ACTIONS) {
        return;
    }
    pyro_action_t *a = &pl->actions[pl->num_actions];
    a->fire = true;
    a->channel = channel;
    a->role = role;
    pl->num_actions++;
}

/* -- Start backup timers for backup channels -------------------- */

/**
 * When a primary channel fires, scan all channels for backups that
 * reference the firing channel as their primary. Start their backup
 * timers.
 */
static void start_backup_timers(pyro_logic_t *pl, uint8_t fired_channel,
                                uint32_t now_ms)
{
    for (uint8_t i = 0; i < pl->num_channels; i++) {
        pyro_channel_config_t *cfg = &pl->config[i];
        pyro_channel_state_t *st = &pl->state[i];

        /* Skip if not a backup role */
        if (cfg->role != PYRO_ROLE_APOGEE_BACKUP &&
            cfg->role != PYRO_ROLE_MAIN_BACKUP) {
            continue;
        }

        /* Skip if already fired or backup already started */
        if (st->fired || st->backup_started) {
            continue;
        }

        /* Check if this backup references the fired channel */
        if (cfg->primary_channel == fired_channel) {
            st->backup_started = true;
            st->backup_start_ms = now_ms;
        }
    }
}

/* -- Per-channel evaluation ------------------------------------- */

static void eval_channel(pyro_logic_t *pl, uint8_t ch_idx,
                         fsm_state_t fsm_state,
                         const flight_context_t *ctx,
                         const sensor_input_t *input,
                         uint32_t now_ms)
{
    pyro_channel_config_t *cfg = &pl->config[ch_idx];
    pyro_channel_state_t *st = &pl->state[ch_idx];

    /* One-shot: if already fired, skip entirely */
    if (st->fired) {
        return;
    }

    /* If window is permanently closed, never fire */
    if (st->window_closed_permanently) {
        return;
    }

    /* ROLE_NONE: skip */
    if (cfg->role == PYRO_ROLE_NONE) {
        return;
    }

    /* Determine eval window status */
    bool permanently_closed = false;
    bool window_open = eval_window_check(cfg->role, fsm_state,
                                          &permanently_closed);

    if (permanently_closed) {
        st->window_closed_permanently = true;
        st->window_open = false;
        return;
    }

    st->window_open = window_open;

    /* Latching flight angle check (IGNITION only, every tick while window open) */
    if (cfg->role == PYRO_ROLE_IGNITION && window_open) {
        if (cfg->max_flight_angle_deg > 0.0f) {
            if (input->tilt_deg > cfg->max_flight_angle_deg) {
                st->flight_angle_violated = true;
            }
        }
    }

    /* If window is not open, nothing more to do (but don't close permanently) */
    if (!window_open) {
        /* For backup channels, we still evaluate backup timers even if
         * the window check function says "not open" -- but the backup
         * is triggered by backup_started flag, not the window. For
         * non-backup roles, just return. */
        if (cfg->role != PYRO_ROLE_APOGEE_BACKUP &&
            cfg->role != PYRO_ROLE_MAIN_BACKUP) {
            return;
        }
        /* For backup roles, if backup hasn't started, nothing to do */
        if (!st->backup_started) {
            return;
        }
    }

    /* ---- Evaluate trigger conditions by role ---- */
    bool trigger = false;

    switch (cfg->role) {
    case PYRO_ROLE_APOGEE:
        /* Fires when FSM has reached APOGEE (apogee_detected is set
         * by the FSM, which transitions through APOGEE to DROGUE).
         * Condition: fsm_state >= FSM_STATE_APOGEE */
        trigger = (fsm_state >= FSM_STATE_APOGEE);
        break;

    case PYRO_ROLE_APOGEE_BACKUP:
        /* Backup timer mode: fires backup_timer_s after primary fired.
         * Backup altitude mode: fires when alt <= backup_alt_m.
         * Only evaluates after backup_started is set by primary firing. */
        if (!st->backup_started) {
            trigger = false;
            break;
        }
        if (cfg->backup_mode == BACKUP_TIMER) {
            uint32_t elapsed_ms = now_ms - st->backup_start_ms;
            trigger = (elapsed_ms >= (uint32_t)(cfg->backup_timer_s * 1000.0f));
        } else if (cfg->backup_mode == BACKUP_ALTITUDE) {
            float alt = get_channel_altitude(cfg, input);
            trigger = (alt <= cfg->backup_alt_m);
        }
        break;

    case PYRO_ROLE_MAIN:
        /* Fires when altitude <= deploy_alt_m.
         * Guard: at least 3s after apogee timestamp. */
        if (!ctx->apogee_detected) {
            trigger = false;
            break;
        }
        if (now_ms - ctx->apogee_timestamp_ms < MAIN_DEPLOY_MIN_TIME_AFTER_APOGEE_MS) {
            trigger = false;
            break;
        }
        {
            float alt = get_channel_altitude(cfg, input);
            trigger = (alt <= cfg->deploy_alt_m);
        }
        break;

    case PYRO_ROLE_MAIN_BACKUP:
        /* Same backup logic as APOGEE_BACKUP */
        if (!st->backup_started) {
            trigger = false;
            break;
        }
        if (cfg->backup_mode == BACKUP_TIMER) {
            uint32_t elapsed_ms = now_ms - st->backup_start_ms;
            trigger = (elapsed_ms >= (uint32_t)(cfg->backup_timer_s * 1000.0f));
        } else if (cfg->backup_mode == BACKUP_ALTITUDE) {
            float alt = get_channel_altitude(cfg, input);
            trigger = (alt <= cfg->backup_alt_m);
        }
        break;

    case PYRO_ROLE_IGNITION:
        /* Fires when in COAST_1, no flight angle violation */
        if (fsm_state != FSM_STATE_COAST_1) {
            trigger = false;
            break;
        }
        if (st->flight_angle_violated) {
            trigger = false;
            break;
        }
        trigger = true;
        break;

    case PYRO_ROLE_NONE:
    default:
        return;
    }

    /* ---- Fire delay logic ---- */
    if (trigger) {
        if (!st->trigger_met) {
            /* First tick trigger is met */
            st->trigger_met = true;
            st->trigger_time_ms = now_ms;
        }

        /* Check if fire delay has elapsed */
        if (cfg->fire_delay_s > 0.0f) {
            uint32_t delay_ms = (uint32_t)(cfg->fire_delay_s * 1000.0f);
            if (now_ms - st->trigger_time_ms < delay_ms) {
                return;  /* still waiting */
            }
        }

        /* Fire! */
        st->fired = true;
        emit_action(pl, ch_idx, cfg->role);

        /* Start backup timers for any backup channels referencing this one */
        start_backup_timers(pl, ch_idx, now_ms);

    } else {
        /* Trigger condition no longer met -- reset fire delay */
        st->trigger_met = false;
    }
}

/* -- Public API ------------------------------------------------- */

void pyro_logic_init(pyro_logic_t *pl)
{
    memset(pl, 0, sizeof(*pl));
    for (uint8_t i = 0; i < PYRO_MAX_CHANNELS; i++) {
        pl->config[i].role = PYRO_ROLE_NONE;
    }
}

void pyro_logic_tick(pyro_logic_t *pl, fsm_state_t fsm_state,
                     const flight_context_t *ctx, const sensor_input_t *input,
                     uint32_t now_ms)
{
    /* Clear actions from previous tick */
    pl->num_actions = 0;

    /* Evaluate each configured channel */
    for (uint8_t i = 0; i < pl->num_channels; i++) {
        eval_channel(pl, i, fsm_state, ctx, input, now_ms);
    }
}
