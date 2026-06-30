/* test_buzzer.c — characterization + edge-case tests for App/buzzer/buzzer.c
 *
 * GOLDEN SPEC captured from the CURRENT (pre-seam) implementation:
 *
 *   #define TIM4_CLK_HZ     216000000UL
 *   #define BUZZER_FREQ_HZ  6000U
 *   #define BUZZER_ARR      ((TIM4_CLK_HZ / BUZZER_FREQ_HZ) - 1U)  == 35999
 *
 *   tone_on(pct):  ccr = ((uint32_t)BUZZER_ARR * pct) / 200U;
 *                  ARR <- 35999 ; CCR <- ccr ; counter <- 0
 *                  (input 0..100 maps to 0..50% duty: pct/200 of ARR)
 *   tone_off():    CCR <- 0   (timer keeps running, channel stays armed)
 *
 *   init():        start PWM channel once, then tone_off() -> silent.
 *
 *   beep_n(pct,count,on_ms,period_ms):
 *      - count==0  -> no-op
 *      - pct>100   -> clamped to 100
 *      - on_ms>period_ms -> on_ms clamped to period_ms (=> off_ms==0)
 *      - off_ms = period_ms - on_ms
 *      - pct==0    -> tone_off(), state=IDLE, return (silent, no beep)
 *      - else      -> tone_on(pct), state=BEEP_ON, step_start=now
 *
 *   tick() state machine (driven by casper_millis()):
 *      BEEP_ON : elapsed>=on_ms  -> tone_off(); beeps_left--;
 *                                   if 0 -> IDLE else -> BEEP_OFF, step_start=now
 *      BEEP_OFF: elapsed>=off_ms -> tone_on(duty); BEEP_ON, step_start=now
 *      IDLE / null htim -> no-op
 *
 * ----------------------------------------------------------------------------
 * TARGET (post-seam) API this suite is written against:
 *   void buzzer_init(casper_pwm_t *pwm);
 *   void buzzer_tick(void);
 *   void buzzer_beep_n(uint8_t pct, uint8_t count, uint16_t on_ms,
 *                      uint16_t period_ms);
 *
 *   tone_on  -> casper_pwm_tone_set(pwm, BUZZER_ARR, ccr)   (ccr as above)
 *   tone_off -> silence (CCR -> 0). The behaviour-preserving mapping is
 *               casper_pwm_tone_set(pwm, BUZZER_ARR, 0); init also calls
 *               casper_pwm_tone_start(pwm) exactly once.
 *
 * These tests are RED until buzzer.c is migrated to the seam (it currently
 * uses TIM_HandleTypeDef / HAL). They are the golden gate for the refactor.
 * ----------------------------------------------------------------------------
 */
#include "test.h"
#include "board_mock.h"
#include "buzzer.h"

/* Golden constants — must equal the driver's compile-time values. */
#define G_ARR        35999u
#define G_CCR(pct)   (((uint32_t)G_ARR * (uint32_t)(pct)) / 200u)

/* ----- expected CCR table (computed offline, pinned here) ----------------- */
/*  pct :   1    50    100                                                    */
/*  ccr : 179  8999  17999                                                    */

/* =========================================================================
 *  init: starts PWM once, leaves buzzer silent (CCR 0), state IDLE.
 * ========================================================================= */
TEST(init_starts_pwm_silent) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");

    buzzer_init(&pwm);

    /* PWM channel must be running after init. */
    ASSERT_EQ_INT(mock_pwm_started(&pwm), 1);
    /* Silent: last CCR written is 0. */
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);
    /* ARR was programmed to the buzzer period. */
    ASSERT_EQ_U(mock_pwm_last_arr(&pwm), G_ARR);

    /* Idle: tick must not touch the PWM at all. */
    int before = mock_pwm_call_count(&pwm);
    mock_advance_ms(1000);
    buzzer_tick();
    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), before);
}

/* =========================================================================
 *  beep_n: first tone-on programs the golden ARR/CCR for the given volume.
 * ========================================================================= */
TEST(beep_tone_on_golden_ccr_50pct) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);
    mock_set_millis(1000);

    buzzer_beep_n(50 /*pct*/, 1 /*count*/, 100 /*on_ms*/, 200 /*period_ms*/);

    ASSERT_EQ_U(mock_pwm_last_arr(&pwm), G_ARR);
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);  /* G_CCR(50) */
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), G_CCR(50));
}

TEST(beep_tone_on_golden_ccr_100pct) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);

    buzzer_beep_n(100, 1, 50, 100);

    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 17999u);  /* G_CCR(100) */
}

TEST(beep_tone_on_golden_ccr_1pct) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);

    buzzer_beep_n(1, 1, 10, 20);

    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 179u);   /* G_CCR(1): truncated */
}

/* =========================================================================
 *  pct > 100 clamps to 100 (CCR == G_CCR(100), NOT overflowing past 50% duty)
 * ========================================================================= */
TEST(beep_pct_clamped_to_100) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);

    buzzer_beep_n(200 /*over*/, 1, 50, 100);

    /* Clamp to 100 -> CCR 17999, i.e. 50% duty, the piezo max. */
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), G_CCR(100));
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 17999u);
}

/* =========================================================================
 *  pct == 0: silent, no beep queued; state stays IDLE.
 * ========================================================================= */
TEST(beep_zero_pct_is_silent_idle) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);

    buzzer_beep_n(0 /*pct*/, 5, 50, 100);

    /* Silent: CCR forced to 0. */
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);

    /* IDLE: ticking forever must never produce a tone. */
    int before = mock_pwm_call_count(&pwm);
    for (int i = 0; i < 20; i++) { mock_advance_ms(50); buzzer_tick(); }
    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), before);
}

/* =========================================================================
 *  count == 0: complete no-op (no PWM writes after init).
 * ========================================================================= */
TEST(beep_zero_count_noop) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);
    int before = mock_pwm_call_count(&pwm);

    buzzer_beep_n(50, 0 /*count*/, 50, 100);

    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), before);

    int after_tick = mock_pwm_call_count(&pwm);
    mock_advance_ms(1000);
    buzzer_tick();
    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), after_tick);
}

/* =========================================================================
 *  Single beep timing: tone stays on until on_ms, then off; then IDLE.
 *  on_ms=100, period_ms=300 -> off_ms=200, count=1.
 * ========================================================================= */
TEST(single_beep_timing) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);
    mock_set_millis(1000);

    buzzer_beep_n(50, 1, 100 /*on*/, 300 /*period*/);
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);   /* ON */

    /* t=1099: still within on window (elapsed 99 < 100) -> no change. */
    mock_set_millis(1099);
    buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);

    /* t=1100: elapsed==100 >= on_ms -> tone off, last beep -> IDLE. */
    mock_set_millis(1100);
    buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);

    /* Now IDLE: further ticks (even past the would-be period) do nothing. */
    int after_off = mock_pwm_call_count(&pwm);
    mock_set_millis(5000);
    buzzer_tick();
    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), after_off);
}

/* =========================================================================
 *  Multi-beep cadence: 3 beeps, on=50, period=200 (off=150).
 *  Pin the full ON/OFF transition sequence and the final IDLE.
 * ========================================================================= */
TEST(multi_beep_cadence) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);

    const uint32_t t0 = 10000;
    mock_set_millis(t0);
    buzzer_beep_n(50, 3 /*count*/, 50 /*on*/, 200 /*period*/);
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);  /* beep1 ON */

    /* --- beep1 ON window: ends at t0+50 --- */
    mock_set_millis(t0 + 49); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);  /* still on */
    mock_set_millis(t0 + 50); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);     /* off, enter OFF gap */

    /* --- OFF gap: off_ms=150, ends at (t0+50)+150 = t0+200 --- */
    mock_set_millis(t0 + 199); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);     /* still silent */
    mock_set_millis(t0 + 200); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);  /* beep2 ON */

    /* --- beep2 ON: ends 50ms after step_start (t0+200) = t0+250 --- */
    mock_set_millis(t0 + 250); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);     /* off, OFF gap */

    /* --- OFF gap ends at t0+250+150 = t0+400 --- */
    mock_set_millis(t0 + 400); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);  /* beep3 ON */

    /* --- beep3 ON ends at t0+450; this is the LAST beep -> IDLE --- */
    mock_set_millis(t0 + 450); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);     /* final off */

    /* IDLE: no further activity. */
    int after = mock_pwm_call_count(&pwm);
    mock_set_millis(t0 + 100000); buzzer_tick();
    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), after);
}

/* =========================================================================
 *  Boundary: on_ms > period_ms -> on_ms clamped to period_ms, off_ms==0.
 *  With off_ms==0 every OFF step re-fires instantly on the next tick, so the
 *  buzzer is effectively continuous between beeps.
 *  count=2, on=500, period=100 -> on clamped to 100, off 0.
 * ========================================================================= */
TEST(on_gt_period_clamped_zero_off) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);
    mock_set_millis(0);

    buzzer_beep_n(50, 2, 500 /*on>period*/, 100 /*period*/);
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);   /* beep1 ON */

    /* on_ms clamped to 100: at elapsed 100 the first beep ends. */
    mock_set_millis(100); buzzer_tick();
    /* beep1 off -> beeps_left=1 -> BEEP_OFF, step_start=100.
       off_ms==0 so this very state would fire on the NEXT tick. The tick that
       turned it off does not also re-arm (single state transition per tick),
       so right now CCR is 0. */
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);

    /* next tick at same/elapsed>=0 -> OFF gap satisfied -> beep2 ON. */
    mock_set_millis(100); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);   /* beep2 ON */

    /* beep2 ends at 100+100=200, last beep -> IDLE. */
    mock_set_millis(200); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);

    int after = mock_pwm_call_count(&pwm);
    mock_set_millis(10000); buzzer_tick();
    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), after);
}

/* =========================================================================
 *  Re-trigger: calling beep_n mid-cadence overrides the in-flight pattern.
 * ========================================================================= */
TEST(retrigger_overrides_pattern) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);
    mock_set_millis(0);

    buzzer_beep_n(50, 5, 50, 200);
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 8999u);

    /* Override with a louder, single short beep partway through. */
    mock_set_millis(20);
    buzzer_beep_n(100, 1, 30, 60);
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 17999u);  /* new volume immediately */

    /* New pattern: single beep, ends 30ms after t=20 -> t=50 -> IDLE. */
    mock_set_millis(50); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);

    int after = mock_pwm_call_count(&pwm);
    mock_set_millis(1000); buzzer_tick();
    ASSERT_EQ_INT(mock_pwm_call_count(&pwm), after);  /* IDLE, count was 1 */
}

/* =========================================================================
 *  Tick exactly at the boundary uses >= (inclusive), verified via on window
 *  where elapsed==on_ms fires. (Already exercised; explicit guard here.)
 * ========================================================================= */
TEST(boundary_is_inclusive) {
    mock_reset();
    casper_pwm_t pwm = mock_pwm_make("BUZZ");
    buzzer_init(&pwm);
    mock_set_millis(0);

    buzzer_beep_n(50, 1, 10, 100);
    /* elapsed == on_ms (10) must trigger off (>=, not >). */
    mock_set_millis(10); buzzer_tick();
    ASSERT_EQ_U(mock_pwm_last_ccr(&pwm), 0u);
}

int main(void) {
    RUN(init_starts_pwm_silent);
    RUN(beep_tone_on_golden_ccr_50pct);
    RUN(beep_tone_on_golden_ccr_100pct);
    RUN(beep_tone_on_golden_ccr_1pct);
    RUN(beep_pct_clamped_to_100);
    RUN(beep_zero_pct_is_silent_idle);
    RUN(beep_zero_count_noop);
    RUN(single_beep_timing);
    RUN(multi_beep_cadence);
    RUN(on_gt_period_clamped_zero_off);
    RUN(retrigger_overrides_pattern);
    RUN(boundary_is_inclusive);
    return test_summary();
}
