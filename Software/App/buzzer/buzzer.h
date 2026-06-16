#ifndef APP_BUZZER_BUZZER_H
#define APP_BUZZER_BUZZER_H

#include "casper_port.h"   /* casper_pwm_t, casper_millis — no HAL types */
#include <stdint.h>

/**
 * Initialize the buzzer driver. Starts the PWM channel once, silent.
 * Must be called after the board layer has configured the timer.
 *
 * @param pwm  Opaque PWM handle (e.g. &BSP_PWM_BUZZ on Casper 2).
 */
void buzzer_init(casper_pwm_t *pwm);

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
