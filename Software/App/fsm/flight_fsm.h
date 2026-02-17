#ifndef APP_FSM_FLIGHT_FSM_H
#define APP_FSM_FLIGHT_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_context.h"
#include "tlm_types.h"

/**
 * Initialize the flight state machine. Starts in PAD state.
 *
 * @param motors_expected  1 for single stage, 2 for two stage.
 */
void flight_fsm_init(uint8_t motors_expected);

/**
 * Tick the FSM. Call at sensor rate (833 Hz from IMU data-ready).
 * When sim is active, uses sim script (existing behavior).
 * When sim is NOT active, uses real sensor data from `input`.
 * Does NOT fire pyros -- that is pyro_logic's job.
 *
 * @param input  Current sensor snapshot
 * @return       Current FSM state
 */
fsm_state_t flight_fsm_tick(const sensor_input_t *input);

/**
 * Get read-only pointer to flight context (for telemetry, pyro_logic, etc.)
 */
const flight_context_t *flight_fsm_get_context(void);

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
 * Notification from pyro_logic when a channel fires.
 * Used for DROGUE->MAIN transition.
 *
 * @param pyro_role  0=Apogee, 2=Main, 4=Ignition
 * @param now_ms     Current timestamp
 */
void flight_fsm_notify_pyro_event(uint8_t pyro_role, uint32_t now_ms);

/* -- Sim flight (preserved from existing code) -- */
void flight_fsm_sim_start(void);
void flight_fsm_sim_stop(void);
bool flight_fsm_sim_active(void);
void flight_fsm_sim_get_state(fc_telem_state_t *out);

/**
 * Compute tilt from vertical in degrees from a quaternion.
 * Body +Z is the nose axis.
 *
 * @param q  Quaternion [w, x, y, z]
 * @return   Tilt angle in degrees (0 = vertical nose-up, 90 = horizontal)
 */
float flight_fsm_tilt_from_quat(const float q[4]);

#endif /* APP_FSM_FLIGHT_FSM_H */
