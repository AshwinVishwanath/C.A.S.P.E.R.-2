/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Buzzer Driver
 *  SUMMARY:  TIM4-driven piezo for arm/disarm/state audible cues.
 * ============================================================ */
#include "buzzer.h"
#include <stddef.h>   /* NULL */

/* ── Timer constants ─────────────────────────────────────────────── */
/*
 * TIM4 clock on Casper 2: APB1 timer clock = 216 MHz (SYSCLK/2 * 2).
 * ARR = (clock / frequency) - 1  = (216 000 000 / 6000) - 1 = 35999.
 * These values are board-specific constants stored here so that buzzer.c
 * (a portable module) computes arr/ccr and passes them to the board layer
 * via casper_pwm_tone_set().  A future board with a different timer clock
 * only needs to change these two constants.
 */
#define BUZZER_CLK_HZ   216000000UL
#define BUZZER_FREQ_HZ  6000U
#define BUZZER_ARR      ((BUZZER_CLK_HZ / BUZZER_FREQ_HZ) - 1U)  /* 35999 */

/* ── State machine ───────────────────────────────────────────────── */
typedef enum {
    BZ_IDLE,
    BZ_BEEP_ON,
    BZ_BEEP_OFF,
} bz_state_t;

static casper_pwm_t *s_pwm;
static bz_state_t s_state;
static uint32_t   s_step_start;    /* casper_millis() when current step began */
static uint8_t    s_duty_pct;      /* 0-100 duty cycle                        */
static uint8_t    s_beeps_left;    /* beeps remaining (including current)      */
static uint16_t   s_on_ms;         /* tone-on duration                        */
static uint16_t   s_off_ms;        /* silence duration (period - on)          */

/* ── Hardware helpers ────────────────────────────────────────────── */

/*
 * buzzer_tone_on: program ARR and CCR for the requested volume, then
 * the PWM channel (already started by buzzer_init) produces the tone.
 *
 * Piezo max acoustic output is at 50 % duty; we map the 0–100 caller
 * input to 0–50 % duty by dividing by 200 instead of 100.
 *   ccr = (ARR * pct) / 200
 */
static void buzzer_tone_on(uint8_t pct)
{
    uint32_t ccr = ((uint32_t)BUZZER_ARR * pct) / 200U;
    casper_pwm_tone_set(s_pwm, BUZZER_ARR, ccr);
}

/*
 * buzzer_tone_off: silence without stopping the PWM channel.
 * Setting CCR to 0 holds the output low while keeping the timer running so
 * that buzzer_tone_on() can resume instantly without restarting the timer.
 */
static void buzzer_tone_off(void)
{
    casper_pwm_tone_set(s_pwm, BUZZER_ARR, 0U);
}

/* ── Public API ──────────────────────────────────────────────────── */
void buzzer_init(casper_pwm_t *pwm)
{
    s_pwm        = pwm;
    s_state      = BZ_IDLE;
    s_beeps_left = 0;

    /* Start the PWM channel once; it stays running for the lifetime of the
     * firmware.  Silence is achieved by driving CCR to 0, not by stopping
     * the timer. */
    casper_pwm_tone_start(s_pwm);
    buzzer_tone_off();
}

void buzzer_tick(void)
{
    if (s_pwm == NULL || s_state == BZ_IDLE) return;

    uint32_t now     = casper_millis();
    uint32_t elapsed = now - s_step_start;

    switch (s_state) {
    case BZ_BEEP_ON:
        if (elapsed >= s_on_ms) {
            buzzer_tone_off();
            s_beeps_left--;
            if (s_beeps_left == 0) {
                s_state = BZ_IDLE;
            } else {
                s_state      = BZ_BEEP_OFF;
                s_step_start = now;
            }
        }
        break;

    case BZ_BEEP_OFF:
        if (elapsed >= s_off_ms) {
            buzzer_tone_on(s_duty_pct);
            s_state      = BZ_BEEP_ON;
            s_step_start = now;
        }
        break;

    default:
        break;
    }
}

void buzzer_beep_n(uint8_t pct, uint8_t count,
                   uint16_t on_ms, uint16_t period_ms)
{
    if (s_pwm == NULL || count == 0) return;
    if (pct > 100) pct = 100;
    if (on_ms > period_ms) on_ms = period_ms;

    s_duty_pct   = pct;
    s_beeps_left = count;
    s_on_ms      = on_ms;
    s_off_ms     = period_ms - on_ms;

    if (pct == 0) {
        /* 0 % = silent, no point beeping */
        buzzer_tone_off();
        s_state = BZ_IDLE;
        return;
    }

    buzzer_tone_on(pct);
    s_state      = BZ_BEEP_ON;
    s_step_start = casper_millis();
}
