#ifndef APP_PYRO_PYRO_MANAGER_H
#define APP_PYRO_PYRO_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/**
 * Initialize the pyro manager and low-level pyro hardware.
 * Calls casper_pyro_init() internally — do NOT call casper_pyro_init() separately.
 */
void pyro_mgr_init(ADC_HandleTypeDef *hadc1,
                    ADC_HandleTypeDef *hadc2,
                    ADC_HandleTypeDef *hadc3);

/**
 * Periodic tick (~10 Hz). Calls casper_pyro_tick() for ADC/LED/auto-stop,
 * checks arm timeout, updates firing state.
 */
void pyro_mgr_tick(void);

/**
 * Set arm state for a single channel.
 *
 * @param channel  Channel number (1–4 per PRD convention)
 * @param armed    true to arm, false to disarm
 * @return         0 on success, -1 on invalid channel or precondition failure
 */
int pyro_mgr_set_arm(uint8_t channel, bool armed);

/**
 * Check continuity on a channel.
 *
 * @param channel  Channel number (1–4)
 * @return         true if continuity detected
 */
bool pyro_mgr_has_continuity(uint8_t channel);

/**
 * Fire a channel.
 * Requires: channel armed AND continuity AND (test_mode OR FSM past PAD).
 *
 * @param channel     Channel number (1–4)
 * @param duration_ms Fire duration in milliseconds (capped at PYRO_MAX_FIRE_MS)
 * @return            0 on success, -1 on precondition failure
 */
int pyro_mgr_fire(uint8_t channel, uint16_t duration_ms);

/**
 * Get arm bitmap. Bits 0–3 correspond to channels 1–4.
 */
uint8_t pyro_mgr_get_arm_bitmap(void);

/**
 * Get continuity bitmap. Bits 0–3 correspond to channels 1–4.
 */
uint8_t pyro_mgr_get_cont_bitmap(void);

/**
 * Check if any channel is currently firing.
 */
bool pyro_mgr_is_firing(void);

/**
 * Disarm all channels and stop all active fires.
 */
void pyro_mgr_disarm_all(void);

/**
 * Set/clear test mode. In test mode, fire is allowed from MC.
 */
void pyro_mgr_set_test_mode(bool enable);

/**
 * Check if test mode is active.
 */
bool pyro_mgr_is_test_mode(void);

/**
 * Auto-arm all flight channels on BOOST entry.
 * Arms channels 0-3 that have continuity and are not excluded.
 * Skips already-armed channels (idempotent).
 * Emits FC_EVT_ARM for each newly armed channel.
 * Called from FSM transition_to(BOOST).
 */
void pyro_mgr_auto_arm_flight(void);

/**
 * FSM-triggered pyro fire (0-indexed channels).
 * Unlike pyro_mgr_fire() (1-indexed, CAC path), this is for FSM auto-fire.
 * Enforces PAD lockout, channel exclusion, arm check, continuity check.
 *
 * @param ch          Channel number (0-3, 0-indexed)
 * @param duration_ms Fire duration in milliseconds (capped at PYRO_MAX_FIRE_MS)
 * @return            0 on success, -1 on precondition failure
 */
int pyro_mgr_auto_fire(uint8_t ch, uint16_t duration_ms);

#endif /* APP_PYRO_PYRO_MANAGER_H */
