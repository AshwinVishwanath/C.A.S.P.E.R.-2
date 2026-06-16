/**
 * @file  casper_pwm.h
 * @brief Portable PWM interface for the buzzer tone output.
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 *
 * On Casper 2 this wraps TIM4 CH3 PWM (__HAL_TIM_SET_AUTORELOAD /
 * __HAL_TIM_SET_COMPARE / HAL_TIM_PWM_Start / Stop).
 *
 * The abstraction is intentionally minimal — only the three operations
 * the buzzer module actually needs.  Frequency and duty-cycle are passed
 * as raw ARR and CCR values so the caller (buzzer.c) controls the tone
 * without the board layer having to know the timer clock frequency.
 * The board layer simply writes the values to the timer registers.
 */

#ifndef CASPER_PWM_H
#define CASPER_PWM_H

#include "casper_types.h"

/**
 * @brief Start the PWM output (timer channel active, no sound until set).
 *
 * Equivalent to HAL_TIM_PWM_Start().  After this call the PWM pin toggles
 * at the rate determined by the timer's ARR/CCR values.  If casper_pwm_tone_set()
 * has not been called yet, the default post-init timer values apply.
 *
 * @param pwm  Opaque PWM handle.
 */
void casper_pwm_tone_start(casper_pwm_t *pwm);

/**
 * @brief Update the PWM frequency and duty cycle.
 *
 * Sets the timer auto-reload register (ARR) and the compare register (CCR)
 * for the buzzer channel.  Both registers are updated atomically (or as
 * close as the board permits without glitching the output).
 *
 * Relationship:
 *   f_tone  = f_timer_clock / (arr + 1)
 *   duty_%  = (ccr * 100) / (arr + 1)
 *
 * The caller (buzzer.c) is responsible for computing arr / ccr from the
 * desired frequency and duty cycle given the board's timer clock.
 * The board layer stores the timer clock frequency inside the
 * casper_pwm_t struct so that buzzer.c can query it if needed, but the
 * current API is value-pass only.
 *
 * @param pwm  Opaque PWM handle.
 * @param arr  New auto-reload value (timer period − 1).
 * @param ccr  New compare value (controls duty cycle).
 */
void casper_pwm_tone_set(casper_pwm_t *pwm, uint32_t arr, uint32_t ccr);

/**
 * @brief Stop the PWM output (pin driven low / idle).
 *
 * Equivalent to HAL_TIM_PWM_Stop().  Silences the buzzer.
 *
 * @param pwm  Opaque PWM handle.
 */
void casper_pwm_tone_off(casper_pwm_t *pwm);

#endif /* CASPER_PWM_H */
