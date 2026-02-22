#ifndef APP_FSM_FLIGHT_FSM_H
#define APP_FSM_FLIGHT_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "tlm_types.h"

/**
 * Initialize the flight state machine. Starts in PAD state.
 */
void flight_fsm_init(void);

/**
 * Tick the FSM. Call every superloop iteration.
 * Handles simulated flight progression and state transitions.
 *
 * @param state  Current telemetry state (used for sensor-based transitions — TODO)
 * @return       Current FSM state
 */
fsm_state_t flight_fsm_tick(const fc_telem_state_t *state);

/**
 * Get current FSM state.
 */
fsm_state_t flight_fsm_get_state(void);

/**
 * Get mission elapsed time in seconds.
 * Returns 0.0 if not yet launched.
 */
float flight_fsm_get_time_s(void);

/**
 * Force FSM to a specific state (for MC/debug-driven transitions).
 * Emits FC_EVT_STATE event via tlm_queue_event.
 */
void flight_fsm_force_state(fsm_state_t new_state);

/**
 * Start simulated flight sequence (0xD0 debug command).
 * Scripted PAD→BOOST→COAST→APOGEE→DROGUE→MAIN→RECOVERY→LANDED.
 */
void flight_fsm_sim_start(void);

/**
 * Stop simulated flight and return to PAD.
 */
void flight_fsm_sim_stop(void);

/**
 * Check if a simulated flight is active.
 */
bool flight_fsm_sim_active(void);

/**
 * Get simulated flight telemetry values.
 * Only valid when sim is active. Fills alt_m, vel_mps, quat.
 */
void flight_fsm_sim_get_state(fc_telem_state_t *out);

/**
 * Enable/disable bench test mode.
 * When on, flight_fsm_tick() holds state (no auto-transitions).
 * State can only be changed via flight_fsm_force_state().
 */
void flight_fsm_set_bench_mode(bool on);

/**
 * Check if bench test mode is active.
 */
bool flight_fsm_bench_active(void);

#endif /* APP_FSM_FLIGHT_FSM_H */
