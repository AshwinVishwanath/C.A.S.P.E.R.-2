/* test_pyro_adc.c — characterization + edge-case tests for App/pyro/casper_pyro.c
 *
 * GOLDEN SPEC captured from the CURRENT (pre-migration) HAL implementation of
 * casper_pyro.c on branch optimization-2026-06-09.  These tests pin the exact
 * numeric conversions, GPIO sequencing, ADC channel->raw mapping, continuity
 * threshold, and auto-stop timing so the upcoming HAL->seam refactor is
 * provably behavior-preserving.
 *
 * ---------------------------------------------------------------------------
 *  RED STATE NOTICE
 * ---------------------------------------------------------------------------
 * These tests are written against the TARGET (migrated) public API, which is
 * NOT YET IMPLEMENTED.  They WILL NOT COMPILE until casper_pyro.{c,h} is
 * migrated off the STM32 HAL onto the casper_port seam.  This is the intended
 * RED state for a characterization-first refactor.  Do NOT modify the driver
 * to make these pass; the implementation must converge to this API/behavior.
 *
 *  TARGET API (post-migration) — derived from the approved plan:
 *    "Pyro keeps a table, so it uses board-provided arrays
 *     (BSP_ADC_CONT[], BSP_PIN_PY[], BSP_PIN_LED[])."
 *
 *    void casper_pyro_init(casper_pyro_t *p,
 *                          casper_adc_t *adc[PYRO_NUM_CHANNELS],
 *                          casper_pin_t  fire[PYRO_NUM_CHANNELS],
 *                          casper_pin_t  led[PYRO_NUM_CHANNELS]);
 *    bool casper_pyro_fire(casper_pyro_t *p, uint8_t ch, uint32_t duration_ms);
 *    void casper_pyro_stop(casper_pyro_t *p, uint8_t ch);
 *    void casper_pyro_stop_all(casper_pyro_t *p);
 *    void casper_pyro_tick(casper_pyro_t *p);
 *
 *  GOLDEN BEHAVIORAL FACTS (from current casper_pyro.c):
 *    - PYRO_NUM_CHANNELS        == 4
 *    - PYRO_CONTINUITY_THRESHOLD == 8000
 *    - continuity[i] = (adc_raw[i] > 8000)     (STRICTLY greater; 8000 -> false)
 *    - tick() reads ALL 4 continuity ADCs into adc_raw[i], in channel order 0..3
 *    - tick() drives each LED pin: HIGH iff continuity[i], else LOW
 *    - fire(ch): firing[ch]=true, fire_start_ms[ch]=now, fire_duration_ms=dur,
 *                fire pin -> HIGH (SET). returns false (and no-op) if ch>=4.
 *    - stop(ch): fire pin -> LOW (RESET), firing[ch]=false. no-op if ch>=4.
 *    - stop_all(): stops all 4 channels.
 *    - init(): all fire pins forced LOW; state zeroed.
 *    - tick() auto-stop: if firing[i] && (now - fire_start_ms[i] >= dur) -> stop.
 *                        Boundary is INCLUSIVE (>=), uses unsigned wrap math.
 */
#include "test.h"
#include "board_mock.h"
#include "casper_pyro.h"

/* ---------------------------------------------------------------------------
 *  Board-table fixtures.
 *
 *  The migrated init takes board-provided tables.  We construct concrete,
 *  per-channel ADC handles and pin descriptors here (the host equivalent of
 *  BSP_ADC_CONT[], BSP_PIN_PY[], BSP_PIN_LED[]).  Pin port/mask values are
 *  arbitrary-but-unique so the mock GPIO table keys them distinctly.
 * ------------------------------------------------------------------------- */

/* One logical ADC channel per pyro continuity input (logical index 0..3).
 * The current driver maps CH -> (adc_idx, adc_channel); after migration the
 * board collapses that into one opaque casper_adc_t per logical channel, so
 * the driver just reads adc[i].
 *
 * NOTE on the type: casper_adc_t is an OPAQUE / incomplete type inside App/
 * code (its struct is defined only in the board/mock).  App/ may therefore
 * only hold POINTERS to it — so the table the driver receives is a pointer
 * array (casper_adc_t *adc[]), and the board provides &BSP_ADC_CONT[i].
 * The test file CAN hold the concrete structs because it includes
 * board_mock.h (which completes the type); it then passes their addresses. */
static casper_adc_t  s_adc[PYRO_NUM_CHANNELS];           /* concrete handles  */
static casper_adc_t *s_adc_ptr[PYRO_NUM_CHANNELS];       /* pointer table     */

/* Distinct fire pins (PY1..PY4) and LED pins (CONT_YN_1..4). */
static casper_pin_t s_fire[PYRO_NUM_CHANNELS] = {
    { (void*)0xF000, 1u << 0 },
    { (void*)0xF000, 1u << 1 },
    { (void*)0xF000, 1u << 2 },
    { (void*)0xF000, 1u << 3 },
};
static casper_pin_t s_led[PYRO_NUM_CHANNELS] = {
    { (void*)0x1ED0, 1u << 0 },
    { (void*)0x1ED0, 1u << 1 },
    { (void*)0x1ED0, 1u << 2 },
    { (void*)0x1ED0, 1u << 3 },
};

/* Reset the whole world: mock globals, ADC queues, fresh handles.
 *
 * TARGET init signature (board-table form, matching plan's BSP_*[] arrays).
 * casper_adc_t is opaque in App/, so the ADC table is a POINTER array:
 *   casper_pyro_init(p,
 *                    casper_adc_t *adc[PYRO_NUM_CHANNELS],  // pointer table
 *                    casper_pin_t  fire[PYRO_NUM_CHANNELS],
 *                    casper_pin_t  led[PYRO_NUM_CHANNELS]);
 */
static void setup(casper_pyro_t *p)
{
    mock_reset();
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        casper_adc_t fresh = mock_adc_make("ADC_CONT");
        s_adc[i]     = fresh;
        s_adc_ptr[i] = &s_adc[i];
    }
    casper_pyro_init(p, s_adc_ptr, s_fire, s_led);
}

/* ===========================================================================
 *  init()
 * ========================================================================= */

/* GOLDEN: init forces every fire pin LOW and zeroes all firing state. */
TEST(init_forces_all_fire_pins_low) {
    casper_pyro_t p;
    setup(&p);
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        ASSERT_EQ_INT(mock_gpio_get_state(s_fire[i]), CASPER_PIN_LOW);
        ASSERT_TRUE(p.firing[i] == false);
        ASSERT_EQ_INT(p.adc_raw[i], 0);
        ASSERT_TRUE(p.continuity[i] == false);
    }
}

/* ===========================================================================
 *  continuity threshold  (golden: adc_raw > 8000)
 * ========================================================================= */

/* GOLDEN: a reading strictly above 8000 -> continuity true; LED HIGH. */
TEST(continuity_above_threshold_true) {
    casper_pyro_t p;
    setup(&p);
    /* Queue one raw value per channel for a single tick. */
    mock_adc_push(&s_adc[0], 8001);   /* just above */
    mock_adc_push(&s_adc[1], 4095);   /* mid */
    mock_adc_push(&s_adc[2], 65535);  /* max 16-bit */
    mock_adc_push(&s_adc[3], 9000);
    casper_pyro_tick(&p);

    ASSERT_EQ_INT(p.adc_raw[0], 8001);
    ASSERT_EQ_INT(p.adc_raw[1], 4095);
    ASSERT_EQ_INT(p.adc_raw[2], 65535);
    ASSERT_EQ_INT(p.adc_raw[3], 9000);

    /* 8001>8000 true; 4095<8000 false; 65535>8000 true; 9000>8000 true */
    ASSERT_TRUE(p.continuity[0] == true);
    ASSERT_TRUE(p.continuity[1] == false);
    ASSERT_TRUE(p.continuity[2] == true);
    ASSERT_TRUE(p.continuity[3] == true);

    ASSERT_EQ_INT(mock_gpio_get_state(s_led[0]), CASPER_PIN_HIGH);
    ASSERT_EQ_INT(mock_gpio_get_state(s_led[1]), CASPER_PIN_LOW);
    ASSERT_EQ_INT(mock_gpio_get_state(s_led[2]), CASPER_PIN_HIGH);
    ASSERT_EQ_INT(mock_gpio_get_state(s_led[3]), CASPER_PIN_HIGH);
}

/* GOLDEN BOUNDARY: exactly 8000 is NOT continuity (strict >, not >=). */
TEST(continuity_exact_threshold_is_false) {
    casper_pyro_t p;
    setup(&p);
    mock_adc_push(&s_adc[0], 8000);   /* == threshold -> false */
    mock_adc_push(&s_adc[1], 7999);   /* below */
    mock_adc_push(&s_adc[2], 0);      /* zero */
    mock_adc_push(&s_adc[3], 8000);
    casper_pyro_tick(&p);

    ASSERT_TRUE(p.continuity[0] == false);
    ASSERT_TRUE(p.continuity[1] == false);
    ASSERT_TRUE(p.continuity[2] == false);
    ASSERT_TRUE(p.continuity[3] == false);

    for (int i = 0; i < PYRO_NUM_CHANNELS; i++)
        ASSERT_EQ_INT(mock_gpio_get_state(s_led[i]), CASPER_PIN_LOW);
}

/* ===========================================================================
 *  ADC channel -> raw mapping (each logical channel reads its OWN ADC)
 * ========================================================================= */

/* GOLDEN: per-channel values are not cross-wired — channel i reads adc[i]. */
TEST(adc_channel_mapping_is_per_channel) {
    casper_pyro_t p;
    setup(&p);
    mock_adc_push(&s_adc[0], 1000);
    mock_adc_push(&s_adc[1], 2000);
    mock_adc_push(&s_adc[2], 3000);
    mock_adc_push(&s_adc[3], 4000);
    casper_pyro_tick(&p);
    const uint16_t gold[4] = { 1000, 2000, 3000, 4000 };
    ASSERT_EQ_MEM(p.adc_raw, gold, sizeof gold);
}

/* GOLDEN: each tick performs exactly one fresh read per channel (FIFO),
 * so two ticks consume two queued values per channel in order. */
TEST(adc_read_is_one_per_channel_per_tick) {
    casper_pyro_t p;
    setup(&p);
    /* tick 1 values */
    mock_adc_push(&s_adc[0], 100);
    mock_adc_push(&s_adc[1], 200);
    mock_adc_push(&s_adc[2], 300);
    mock_adc_push(&s_adc[3], 400);
    /* tick 2 values */
    mock_adc_push(&s_adc[0], 9001);
    mock_adc_push(&s_adc[1], 9002);
    mock_adc_push(&s_adc[2], 9003);
    mock_adc_push(&s_adc[3], 9004);

    casper_pyro_tick(&p);
    ASSERT_EQ_INT(p.adc_raw[0], 100);
    ASSERT_TRUE(p.continuity[0] == false);

    casper_pyro_tick(&p);
    ASSERT_EQ_INT(p.adc_raw[0], 9001);
    ASSERT_EQ_INT(p.adc_raw[3], 9004);
    ASSERT_TRUE(p.continuity[3] == true);
}

/* EDGE: empty ADC queue -> mock returns 0 -> continuity false, LED LOW. */
TEST(adc_empty_queue_reads_zero) {
    casper_pyro_t p;
    setup(&p);
    /* No mock_adc_push; queues empty. */
    casper_pyro_tick(&p);
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        ASSERT_EQ_INT(p.adc_raw[i], 0);
        ASSERT_TRUE(p.continuity[i] == false);
        ASSERT_EQ_INT(mock_gpio_get_state(s_led[i]), CASPER_PIN_LOW);
    }
}

/* ===========================================================================
 *  fire() / stop() GPIO sequencing
 * ========================================================================= */

/* GOLDEN: fire(ch) drives that fire pin HIGH and sets firing state. */
TEST(fire_sets_pin_high_and_state) {
    casper_pyro_t p;
    setup(&p);
    mock_set_millis(1234);
    bool ok = casper_pyro_fire(&p, 2, 50);
    ASSERT_TRUE(ok == true);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[2]), CASPER_PIN_HIGH);
    ASSERT_TRUE(p.firing[2] == true);
    ASSERT_EQ_U(p.fire_start_ms[2], 1234u);
    ASSERT_EQ_U(p.fire_duration_ms[2], 50u);
    /* Other channels untouched. */
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[0]), CASPER_PIN_LOW);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[1]), CASPER_PIN_LOW);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[3]), CASPER_PIN_LOW);
    ASSERT_TRUE(p.firing[0] == false);
}

/* GOLDEN: stop(ch) drives the fire pin LOW and clears firing state. */
TEST(stop_sets_pin_low_and_clears_state) {
    casper_pyro_t p;
    setup(&p);
    casper_pyro_fire(&p, 1, 100);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[1]), CASPER_PIN_HIGH);
    casper_pyro_stop(&p, 1);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[1]), CASPER_PIN_LOW);
    ASSERT_TRUE(p.firing[1] == false);
}

/* GOLDEN: stop_all() drives every fire pin LOW and clears all firing. */
TEST(stop_all_clears_every_channel) {
    casper_pyro_t p;
    setup(&p);
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++)
        casper_pyro_fire(&p, i, 100);
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++)
        ASSERT_EQ_INT(mock_gpio_get_state(s_fire[i]), CASPER_PIN_HIGH);

    casper_pyro_stop_all(&p);
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++) {
        ASSERT_EQ_INT(mock_gpio_get_state(s_fire[i]), CASPER_PIN_LOW);
        ASSERT_TRUE(p.firing[i] == false);
    }
}

/* ===========================================================================
 *  Invalid channel guards (ch >= PYRO_NUM_CHANNELS)
 * ========================================================================= */

/* GOLDEN: fire() with ch>=4 returns false and writes no pin. */
TEST(fire_invalid_channel_rejected) {
    casper_pyro_t p;
    setup(&p);
    ASSERT_TRUE(casper_pyro_fire(&p, 4, 100) == false);
    ASSERT_TRUE(casper_pyro_fire(&p, 5, 100) == false);
    ASSERT_TRUE(casper_pyro_fire(&p, 255, 100) == false);
    /* No fire pin should have gone high. */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++)
        ASSERT_EQ_INT(mock_gpio_get_state(s_fire[i]), CASPER_PIN_LOW);
}

/* GOLDEN: stop() with ch>=4 is a silent no-op (no crash, no pin write). */
TEST(stop_invalid_channel_noop) {
    casper_pyro_t p;
    setup(&p);
    casper_pyro_fire(&p, 0, 100);
    casper_pyro_stop(&p, 4);    /* out of range -> no-op */
    casper_pyro_stop(&p, 200);
    /* Channel 0 still firing (untouched by invalid stops). */
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[0]), CASPER_PIN_HIGH);
    ASSERT_TRUE(p.firing[0] == true);
}

/* ===========================================================================
 *  Auto-stop timing in tick()
 * ========================================================================= */

/* GOLDEN: a fire auto-stops once (now - start) >= duration. Boundary is
 * inclusive: at exactly start+duration it stops. */
TEST(autostop_inclusive_boundary) {
    casper_pyro_t p;
    setup(&p);
    mock_set_millis(1000);
    casper_pyro_fire(&p, 0, 50);     /* start=1000, dur=50 */
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[0]), CASPER_PIN_HIGH);

    /* t=1049: 49 < 50 -> still firing */
    mock_set_millis(1049);
    casper_pyro_tick(&p);
    ASSERT_TRUE(p.firing[0] == true);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[0]), CASPER_PIN_HIGH);

    /* t=1050: 50 >= 50 -> auto-stop fires this tick */
    mock_set_millis(1050);
    casper_pyro_tick(&p);
    ASSERT_TRUE(p.firing[0] == false);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[0]), CASPER_PIN_LOW);
}

/* GOLDEN: well past the duration the channel is stopped. */
TEST(autostop_after_duration) {
    casper_pyro_t p;
    setup(&p);
    mock_set_millis(500);
    casper_pyro_fire(&p, 3, 20);     /* start=500, dur=20 */

    mock_set_millis(600);            /* 100 >= 20 */
    casper_pyro_tick(&p);
    ASSERT_TRUE(p.firing[3] == false);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[3]), CASPER_PIN_LOW);
}

/* GOLDEN: a zero-duration fire auto-stops on the very next tick
 * (now - start >= 0 is always true). */
TEST(autostop_zero_duration) {
    casper_pyro_t p;
    setup(&p);
    mock_set_millis(2000);
    casper_pyro_fire(&p, 1, 0);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[1]), CASPER_PIN_HIGH);
    /* Same-tick: now==start, 0>=0 -> stop. */
    casper_pyro_tick(&p);
    ASSERT_TRUE(p.firing[1] == false);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[1]), CASPER_PIN_LOW);
}

/* GOLDEN: auto-stop is independent per channel; only the expired one stops. */
TEST(autostop_is_per_channel_independent) {
    casper_pyro_t p;
    setup(&p);
    mock_set_millis(1000);
    casper_pyro_fire(&p, 0, 10);     /* expires at 1010 */
    casper_pyro_fire(&p, 1, 1000);   /* expires at 2000 */

    mock_set_millis(1010);
    casper_pyro_tick(&p);
    ASSERT_TRUE(p.firing[0] == false);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[0]), CASPER_PIN_LOW);
    ASSERT_TRUE(p.firing[1] == true);
    ASSERT_EQ_INT(mock_gpio_get_state(s_fire[1]), CASPER_PIN_HIGH);
}

/* GOLDEN: a non-firing channel is never auto-stopped / never spuriously
 * driven; tick only touches LED pins for non-firing channels. */
TEST(tick_does_not_disturb_idle_fire_pins) {
    casper_pyro_t p;
    setup(&p);
    /* No fires.  Push continuity so LEDs move, but fire pins must stay LOW. */
    mock_adc_push(&s_adc[0], 9000);
    casper_pyro_tick(&p);
    ASSERT_EQ_INT(mock_gpio_get_state(s_led[0]), CASPER_PIN_HIGH);
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++)
        ASSERT_EQ_INT(mock_gpio_get_state(s_fire[i]), CASPER_PIN_LOW);
}

/* GOLDEN: LED state tracks continuity across ticks (re-evaluated each tick,
 * so a drop in reading turns the LED back off). */
TEST(led_tracks_continuity_across_ticks) {
    casper_pyro_t p;
    setup(&p);
    mock_adc_push(&s_adc[0], 9000);   /* tick1: continuity */
    mock_adc_push(&s_adc[1], 0);
    mock_adc_push(&s_adc[2], 0);
    mock_adc_push(&s_adc[3], 0);
    casper_pyro_tick(&p);
    ASSERT_EQ_INT(mock_gpio_get_state(s_led[0]), CASPER_PIN_HIGH);

    mock_adc_push(&s_adc[0], 100);    /* tick2: lost continuity */
    mock_adc_push(&s_adc[1], 0);
    mock_adc_push(&s_adc[2], 0);
    mock_adc_push(&s_adc[3], 0);
    casper_pyro_tick(&p);
    ASSERT_EQ_INT(mock_gpio_get_state(s_led[0]), CASPER_PIN_LOW);
}

int main(void) {
    RUN(init_forces_all_fire_pins_low);
    RUN(continuity_above_threshold_true);
    RUN(continuity_exact_threshold_is_false);
    RUN(adc_channel_mapping_is_per_channel);
    RUN(adc_read_is_one_per_channel_per_tick);
    RUN(adc_empty_queue_reads_zero);
    RUN(fire_sets_pin_high_and_state);
    RUN(stop_sets_pin_low_and_clears_state);
    RUN(stop_all_clears_every_channel);
    RUN(fire_invalid_channel_rejected);
    RUN(stop_invalid_channel_noop);
    RUN(autostop_inclusive_boundary);
    RUN(autostop_after_duration);
    RUN(autostop_zero_duration);
    RUN(autostop_is_per_channel_independent);
    RUN(tick_does_not_disturb_idle_fire_pins);
    RUN(led_tracks_continuity_across_ticks);
    return test_summary();
}
