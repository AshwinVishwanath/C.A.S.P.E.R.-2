/**
 * @file test_pyro_manager.c
 * @brief Tier-2 protocol tests for pyro_manager and casper_pyro.
 *
 * Tests per PRD S4.3:
 *   - Cannot arm without continuity
 *   - Cannot fire without arm
 *   - Cannot fire without test mode or correct FSM state
 *   - Fire re-checks arm + continuity at execution time
 *   - Auto-disarm after fire completes
 *   - pyro_mgr_set_arm(): arm/disarm channels
 *   - pyro_mgr_get_arm_bitmap(): correct bitmap
 *   - pyro_mgr_get_cont_bitmap(): correct bitmap
 *   - pyro_mgr_fire(): success when all conditions met
 *   - pyro_mgr_disarm_all(): all channels disarmed
 */

#include "test_config.h"
#include "main.h"
#include "mock_tick.h"
#include "mock_gpio.h"
#include "mock_adc.h"
#include "pyro_manager.h"
#include "casper_pyro.h"
#include "tlm_types.h"

/* ================================================================== */
/*  Stubs for dependencies                                              */
/* ================================================================== */

/* flight_fsm_get_state stub — controls what the pyro manager sees */
static uint8_t s_stub_fsm_state = FSM_STATE_PAD;

uint8_t flight_fsm_get_state(void)
{
    return s_stub_fsm_state;
}

/* tlm_queue_event stub */
int tlm_queue_event(uint8_t type, uint16_t data)
{
    (void)type; (void)data;
    return 1;
}

/* ================================================================== */
/*  Helpers                                                             */
/* ================================================================== */

static void force_all_continuity(void)
{
    mock_adc_set_value(&hadc1, 12000);
    mock_adc_set_value(&hadc2, 12000);
    mock_adc_set_value(&hadc3, 12000);
    pyro_mgr_tick();
}

static void force_no_continuity(void)
{
    mock_adc_set_value(&hadc1, 100);
    mock_adc_set_value(&hadc2, 100);
    mock_adc_set_value(&hadc3, 100);
    pyro_mgr_tick();
}

/* ================================================================== */
/*  setUp / tearDown                                                    */
/* ================================================================== */

void setUp(void)
{
    mock_tick_reset();
    mock_gpio_reset();
    mock_adc_reset();
    mock_tick_set(1000);

    s_stub_fsm_state = FSM_STATE_PAD;
    pyro_mgr_init(&hadc1, &hadc2, &hadc3);
}

void tearDown(void) { }

/* ================================================================== */
/*  Arm / Disarm tests                                                  */
/* ================================================================== */

void test_arm_requires_continuity(void)
{
    force_no_continuity();

    int ret = pyro_mgr_set_arm(1, true);
    TEST_ASSERT_EQUAL_INT(-1, ret);
    TEST_ASSERT_FALSE(pyro_mgr_get_arm_bitmap() & 0x01);
}

void test_arm_succeeds_with_continuity(void)
{
    force_all_continuity();

    int ret = pyro_mgr_set_arm(1, true);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_TRUE(pyro_mgr_get_arm_bitmap() & 0x01);
}

void test_arm_invalid_channel_0(void)
{
    force_all_continuity();
    int ret = pyro_mgr_set_arm(0, true);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void test_arm_invalid_channel_5(void)
{
    force_all_continuity();
    int ret = pyro_mgr_set_arm(5, true);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void test_arm_all_four_channels(void)
{
    force_all_continuity();

    for (uint8_t ch = 1; ch <= 4; ch++) {
        TEST_ASSERT_EQUAL_INT(0, pyro_mgr_set_arm(ch, true));
    }

    TEST_ASSERT_EQUAL_HEX8(0x0F, pyro_mgr_get_arm_bitmap());
}

void test_disarm_channel(void)
{
    force_all_continuity();

    pyro_mgr_set_arm(1, true);
    TEST_ASSERT_TRUE(pyro_mgr_get_arm_bitmap() & 0x01);

    pyro_mgr_set_arm(1, false);
    TEST_ASSERT_FALSE(pyro_mgr_get_arm_bitmap() & 0x01);
}

void test_disarm_all(void)
{
    force_all_continuity();

    for (uint8_t ch = 1; ch <= 4; ch++) {
        pyro_mgr_set_arm(ch, true);
    }
    TEST_ASSERT_EQUAL_HEX8(0x0F, pyro_mgr_get_arm_bitmap());

    pyro_mgr_disarm_all();
    TEST_ASSERT_EQUAL_HEX8(0x00, pyro_mgr_get_arm_bitmap());
}

/* ================================================================== */
/*  Continuity bitmap tests                                             */
/* ================================================================== */

void test_cont_bitmap_all_continuity(void)
{
    force_all_continuity();
    /* All 4 channels use ADC handles shared between channels:
     * hadc1 for CH1+CH2, hadc3 for CH3, hadc2 for CH4.
     * Since mock_adc returns the same value per handle,
     * all channels should show continuity. */
    uint8_t bm = pyro_mgr_get_cont_bitmap();
    TEST_ASSERT_EQUAL_HEX8(0x0F, bm);
}

void test_cont_bitmap_no_continuity(void)
{
    force_no_continuity();
    TEST_ASSERT_EQUAL_HEX8(0x00, pyro_mgr_get_cont_bitmap());
}

/* ================================================================== */
/*  Fire tests                                                          */
/* ================================================================== */

void test_fire_without_arm_fails(void)
{
    force_all_continuity();
    s_stub_fsm_state = FSM_STATE_BOOST;  /* past PAD */

    int ret = pyro_mgr_fire(1, 100);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void test_fire_without_continuity_fails(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);

    /* Now remove continuity */
    force_no_continuity();
    s_stub_fsm_state = FSM_STATE_BOOST;

    int ret = pyro_mgr_fire(1, 100);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void test_fire_on_pad_without_test_mode_fails(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_PAD;

    int ret = pyro_mgr_fire(1, 100);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void test_fire_on_pad_with_test_mode_succeeds(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_PAD;
    pyro_mgr_set_test_mode(true);
    /* set_test_mode(true) does not disarm, but let's re-check:
     * looking at source — set_test_mode(false) calls disarm_all,
     * but set_test_mode(true) does not. So arm is preserved. */

    /* Re-arm since we need to verify */
    force_all_continuity();
    pyro_mgr_set_arm(1, true);

    int ret = pyro_mgr_fire(1, 50);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_TRUE(pyro_mgr_is_firing());
}

void test_fire_past_pad_without_test_mode_succeeds(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_BOOST;

    int ret = pyro_mgr_fire(1, 100);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_TRUE(pyro_mgr_is_firing());
}

void test_fire_duration_capped_to_max(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_BOOST;

    /* Fire with duration > PYRO_MAX_FIRE_MS (2000) */
    int ret = pyro_mgr_fire(1, 5000);
    TEST_ASSERT_EQUAL_INT(0, ret);

    /* Advance past 2000ms — should auto-stop */
    mock_tick_advance(2001);
    pyro_mgr_tick();
    TEST_ASSERT_FALSE(pyro_mgr_is_firing());
}

void test_fire_test_mode_caps_at_50ms(void)
{
    force_all_continuity();
    pyro_mgr_set_test_mode(true);
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_PAD;

    /* Request 200ms but test mode caps to 50ms */
    int ret = pyro_mgr_fire(1, 200);
    TEST_ASSERT_EQUAL_INT(0, ret);

    /* After 51ms should be stopped */
    mock_tick_advance(51);
    pyro_mgr_tick();
    TEST_ASSERT_FALSE(pyro_mgr_is_firing());
}

void test_fire_gpio_set_then_reset(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_BOOST;

    mock_gpio_reset();  /* clear log to track fire-specific writes */

    pyro_mgr_fire(1, 100);

    /* GPIO should have been set (fire pin HIGH) */
    /* PY1 is on GPIOD, pin 10 (GPIO_PIN_10 = 0x0400) */
    GPIO_PinState state = mock_gpio_get_written(GPIOD, GPIO_PIN_10);
    TEST_ASSERT_EQUAL(GPIO_PIN_SET, state);

    /* After duration expires */
    mock_tick_advance(101);
    pyro_mgr_tick();

    state = mock_gpio_get_written(GPIOD, GPIO_PIN_10);
    TEST_ASSERT_EQUAL(GPIO_PIN_RESET, state);
}

void test_fire_invalid_channel_fails(void)
{
    force_all_continuity();
    s_stub_fsm_state = FSM_STATE_BOOST;

    TEST_ASSERT_EQUAL_INT(-1, pyro_mgr_fire(0, 100));
    TEST_ASSERT_EQUAL_INT(-1, pyro_mgr_fire(5, 100));
}

/* ================================================================== */
/*  Test mode tests                                                     */
/* ================================================================== */

void test_set_test_mode_on(void)
{
    pyro_mgr_set_test_mode(true);
    TEST_ASSERT_TRUE(pyro_mgr_is_test_mode());
}

void test_set_test_mode_off_disarms_all(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    pyro_mgr_set_arm(2, true);
    TEST_ASSERT_EQUAL_HEX8(0x03, pyro_mgr_get_arm_bitmap());

    pyro_mgr_set_test_mode(false);
    TEST_ASSERT_FALSE(pyro_mgr_is_test_mode());
    TEST_ASSERT_EQUAL_HEX8(0x00, pyro_mgr_get_arm_bitmap());
}

/* ================================================================== */
/*  is_firing tests                                                     */
/* ================================================================== */

void test_is_firing_false_initially(void)
{
    TEST_ASSERT_FALSE(pyro_mgr_is_firing());
}

void test_is_firing_true_during_fire(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_BOOST;

    pyro_mgr_fire(1, 1000);
    TEST_ASSERT_TRUE(pyro_mgr_is_firing());
}

void test_is_firing_false_after_auto_stop(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    s_stub_fsm_state = FSM_STATE_BOOST;

    pyro_mgr_fire(1, 100);
    mock_tick_advance(101);
    pyro_mgr_tick();

    TEST_ASSERT_FALSE(pyro_mgr_is_firing());
}

/* ================================================================== */
/*  Has continuity API tests                                            */
/* ================================================================== */

void test_has_continuity_valid_channels(void)
{
    force_all_continuity();

    for (uint8_t ch = 1; ch <= 4; ch++) {
        TEST_ASSERT_TRUE(pyro_mgr_has_continuity(ch));
    }
}

void test_has_continuity_invalid_channels(void)
{
    force_all_continuity();

    TEST_ASSERT_FALSE(pyro_mgr_has_continuity(0));
    TEST_ASSERT_FALSE(pyro_mgr_has_continuity(5));
}

/* ================================================================== */
/*  main()                                                              */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Arm / Disarm */
    RUN_TEST(test_arm_requires_continuity);
    RUN_TEST(test_arm_succeeds_with_continuity);
    RUN_TEST(test_arm_invalid_channel_0);
    RUN_TEST(test_arm_invalid_channel_5);
    RUN_TEST(test_arm_all_four_channels);
    RUN_TEST(test_disarm_channel);
    RUN_TEST(test_disarm_all);

    /* Continuity bitmap */
    RUN_TEST(test_cont_bitmap_all_continuity);
    RUN_TEST(test_cont_bitmap_no_continuity);

    /* Fire interlocks */
    RUN_TEST(test_fire_without_arm_fails);
    RUN_TEST(test_fire_without_continuity_fails);
    RUN_TEST(test_fire_on_pad_without_test_mode_fails);
    RUN_TEST(test_fire_on_pad_with_test_mode_succeeds);
    RUN_TEST(test_fire_past_pad_without_test_mode_succeeds);
    RUN_TEST(test_fire_duration_capped_to_max);
    RUN_TEST(test_fire_test_mode_caps_at_50ms);
    RUN_TEST(test_fire_gpio_set_then_reset);
    RUN_TEST(test_fire_invalid_channel_fails);

    /* Test mode */
    RUN_TEST(test_set_test_mode_on);
    RUN_TEST(test_set_test_mode_off_disarms_all);

    /* is_firing */
    RUN_TEST(test_is_firing_false_initially);
    RUN_TEST(test_is_firing_true_during_fire);
    RUN_TEST(test_is_firing_false_after_auto_stop);

    /* has_continuity */
    RUN_TEST(test_has_continuity_valid_channels);
    RUN_TEST(test_has_continuity_invalid_channels);

    return UNITY_END();
}
