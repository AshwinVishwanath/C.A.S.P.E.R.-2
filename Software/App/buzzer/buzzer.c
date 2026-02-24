#include "buzzer.h"

/* ── Timer constants ─────────────────────────────────────────────── */
#define TIM4_CLK_HZ          216000000UL
#define BUZZER_FREQ_HZ       6000U
#define BUZZER_ARR           ((TIM4_CLK_HZ / BUZZER_FREQ_HZ) - 1U)  /* 35999 */
#define BUZZER_TIM_CHANNEL   TIM_CHANNEL_3

/* ── State machine ───────────────────────────────────────────────── */
typedef enum {
    BZ_IDLE,
    BZ_BEEP_ON,
    BZ_BEEP_OFF,
} bz_state_t;

static TIM_HandleTypeDef *s_htim;
static bz_state_t s_state;
static uint32_t   s_step_start;    /* HAL_GetTick() when current step began */
static uint8_t    s_duty_pct;      /* 0-100 duty cycle                      */
static uint8_t    s_beeps_left;    /* beeps remaining (including current)    */
static uint16_t   s_on_ms;         /* tone-on duration                      */
static uint16_t   s_off_ms;        /* silence duration (period - on)        */

/* ── Hardware helpers ────────────────────────────────────────────── */
static void buzzer_tone_on(uint8_t pct)
{
    /* Piezo max acoustic output at 50% duty; map 0-100 input → 0-50% duty */
    uint32_t ccr = ((uint32_t)BUZZER_ARR * pct) / 200U;
    __HAL_TIM_SET_AUTORELOAD(s_htim, BUZZER_ARR);
    __HAL_TIM_SET_COMPARE(s_htim, BUZZER_TIM_CHANNEL, ccr);
    __HAL_TIM_SET_COUNTER(s_htim, 0);
}

static void buzzer_tone_off(void)
{
    __HAL_TIM_SET_COMPARE(s_htim, BUZZER_TIM_CHANNEL, 0);
}

/* ── Public API ──────────────────────────────────────────────────── */
void buzzer_init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;
    s_state = BZ_IDLE;
    s_beeps_left = 0;

    HAL_TIM_PWM_Start(s_htim, BUZZER_TIM_CHANNEL);
    buzzer_tone_off();
}

void buzzer_tick(void)
{
    if (s_htim == NULL || s_state == BZ_IDLE) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_step_start;

    switch (s_state) {
    case BZ_BEEP_ON:
        if (elapsed >= s_on_ms) {
            buzzer_tone_off();
            s_beeps_left--;
            if (s_beeps_left == 0) {
                s_state = BZ_IDLE;
            } else {
                s_state = BZ_BEEP_OFF;
                s_step_start = now;
            }
        }
        break;

    case BZ_BEEP_OFF:
        if (elapsed >= s_off_ms) {
            buzzer_tone_on(s_duty_pct);
            s_state = BZ_BEEP_ON;
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
    if (s_htim == NULL || count == 0) return;
    if (pct > 100) pct = 100;
    if (on_ms > period_ms) on_ms = period_ms;

    s_duty_pct   = pct;
    s_beeps_left = count;
    s_on_ms      = on_ms;
    s_off_ms     = period_ms - on_ms;

    if (pct == 0) {
        /* 0% = silent, no point beeping */
        buzzer_tone_off();
        s_state = BZ_IDLE;
        return;
    }

    buzzer_tone_on(pct);
    s_state      = BZ_BEEP_ON;
    s_step_start = HAL_GetTick();
}
