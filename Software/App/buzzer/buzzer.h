#ifndef APP_BUZZER_BUZZER_H
#define APP_BUZZER_BUZZER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/**
 * Initialize the buzzer driver. Starts TIM4 PWM on CH3, silent.
 * Must be called after MX_TIM4_Init().
 */
void buzzer_init(TIM_HandleTypeDef *htim);

/**
 * Non-blocking tick. Call from the superloop every iteration.
 * Advances the beep state machine using HAL_GetTick().
 */
void buzzer_tick(void);

/**
 * Queue N beeps at a given volume.
 *
 * @param pct        Duty cycle 0-100 (volume). 0 = silent, 50 = loudest.
 * @param count      Number of beeps to play.
 * @param on_ms      Tone-on duration per beep (ms).
 * @param period_ms  Total period per beep (on + off). off = period - on.
 */
void buzzer_beep_n(uint8_t pct, uint8_t count,
                   uint16_t on_ms, uint16_t period_ms);

#endif /* APP_BUZZER_BUZZER_H */
