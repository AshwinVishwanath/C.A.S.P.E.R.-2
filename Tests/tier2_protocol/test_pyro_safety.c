/**
 * @file  test_pyro_safety.c
 * @brief Tier-2 safety tests for pyro auto-arm, auto-fire, PAD lockout,
 *        and channel exclusion.
 *
 * Tests per CASPER_FSM_PRD Level 3 (23 tests):
 *   - PAD lockout on auto_fire
 *   - Channel exclusion
 *   - Auto-arm on BOOST entry
 *   - Auto-fire preconditions
 *   - Integration with FSM transitions
 *
 * Compiled with -DHIL_MODE -DUNIT_TEST -DPYRO_EXCLUDE_MASK=0x04
 * (channel 2 excluded for exclusion tests).
 */

#include "test_config.h"
#include "main.h"
#include "mock_tick.h"
#include "mock_gpio.h"
#include "mock_adc.h"
#include "pyro_manager.h"
#include "casper_pyro.h"
#include "flight_fsm.h"
#include "fsm_util.h"
#include "tlm_types.h"

/* ================================================================== */
/*  Event log                                                          */
/* ================================================================== */
#define EVENT_LOG_SIZE 64
static struct { uint8_t type; uint16_t data; } s_event_log[EVENT_LOG_SIZE];
static int s_event_count = 0;

int tlm_queue_event(uint8_t type, uint16_t data)
{
    if (s_event_count < EVENT_LOG_SIZE) {
        s_event_log[s_event_count].type = type;
        s_event_log[s_event_count].data = data;
        s_event_count++;
    }
    return 1;
}

static void clear_events(void)
{
    s_event_count = 0;
}

static bool has_event(uint8_t type, uint16_t data)
{
    for (int i = 0; i < s_event_count; i++) {
        if (s_event_log[i].type == type && s_event_log[i].data == data)
            return true;
    }
    return false;
}

static bool has_event_type(uint8_t type)
{
    for (int i = 0; i < s_event_count; i++) {
        if (s_event_log[i].type == type)
            return true;
    }
    return false;
}

static int count_events(uint8_t type)
{
    int count = 0;
    for (int i = 0; i < s_event_count; i++) {
        if (s_event_log[i].type == type)
            count++;
    }
    return count;
}

/* ================================================================== */
/*  Helpers                                                            */
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

/* Drive FSM to a specific state via force_state */
static void drive_fsm_to(fsm_state_t target)
{
    flight_fsm_force_state(target);
}

/* ================================================================== */
/*  setUp / tearDown                                                   */
/* ================================================================== */

void setUp(void)
{
    mock_tick_reset();
    mock_gpio_reset();
    mock_adc_reset();
    mock_tick_set(1000);
    fsm_set_tick(1000);

    flight_fsm_init();
    pyro_mgr_init(&hadc1, &hadc2, &hadc3);
    clear_events();
}

void tearDown(void) { }

/* ================================================================== */
/*  TEST_PAD_LOCK: PAD lockout tests (5 tests)                        */
/* ================================================================== */

void TEST_PAD_LOCK_01_auto_fire_returns_neg1_on_pad(void)
{
    /* FSM starts in PAD. Arm channel 0 manually for test. */
    force_all_continuity();
    /* Can't use auto_arm (it checks state too), so use the CAC path */
    pyro_mgr_set_arm(1, true);  /* 1-indexed for CAC path */

    int ret = pyro_mgr_auto_fire(0, 1000);  /* 0-indexed for FSM path */
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void TEST_PAD_LOCK_02_auto_fire_succeeds_on_boost(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);  /* arm ch 0 (1-indexed) */
    drive_fsm_to(FSM_STATE_BOOST);
    clear_events();

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);
}

void TEST_PAD_LOCK_03_auto_fire_succeeds_on_coast(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    drive_fsm_to(FSM_STATE_COAST);
    clear_events();

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);
}

void TEST_PAD_LOCK_04_auto_fire_succeeds_on_apogee(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    drive_fsm_to(FSM_STATE_APOGEE);
    clear_events();

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);
}

void TEST_PAD_LOCK_05_auto_fire_succeeds_on_main(void)
{
    force_all_continuity();
    pyro_mgr_set_arm(1, true);
    drive_fsm_to(FSM_STATE_MAIN);
    clear_events();

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);
}

/* ================================================================== */
/*  TEST_EXCLUDE: Channel exclusion tests (4 tests)                    */
/*  Compiled with PYRO_EXCLUDE_MASK=0x04 (channel 2 excluded)         */
/* ================================================================== */

void TEST_EXCLUDE_01_excluded_ch_not_armed_by_auto_arm(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    clear_events();

    pyro_mgr_auto_arm_flight();

    /* Channel 2 (bit 2 of bitmap) should NOT be armed */
    uint8_t bitmap = pyro_mgr_get_arm_bitmap();
    TEST_ASSERT_FALSE(bitmap & 0x04);  /* bit 2 = channel 2 */
}

void TEST_EXCLUDE_02_excluded_ch_rejected_by_auto_fire(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);

    /* Manually arm channel 2 via CAC path (1-indexed: ch 3) */
    pyro_mgr_set_arm(3, true);

    int ret = pyro_mgr_auto_fire(2, 1000);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void TEST_EXCLUDE_03_non_excluded_ch_armed_by_auto_arm(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    clear_events();

    pyro_mgr_auto_arm_flight();

    /* Channels 0, 1, 3 should be armed (not ch 2) */
    uint8_t bitmap = pyro_mgr_get_arm_bitmap();
    TEST_ASSERT_TRUE(bitmap & 0x01);   /* ch 0 */
    TEST_ASSERT_TRUE(bitmap & 0x02);   /* ch 1 */
    TEST_ASSERT_TRUE(bitmap & 0x08);   /* ch 3 */
}

void TEST_EXCLUDE_04_non_excluded_ch_accepted_by_auto_fire(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);

    /* Arm non-excluded channels */
    pyro_mgr_auto_arm_flight();
    clear_events();

    /* Fire channel 0 (not excluded) */
    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);
}

/* ================================================================== */
/*  TEST_AUTOARM: Auto-arm tests (5 tests)                             */
/* ================================================================== */

void TEST_AUTOARM_01_arms_all_with_continuity(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    clear_events();

    pyro_mgr_auto_arm_flight();

    /* All non-excluded channels with continuity should be armed */
    /* With PYRO_EXCLUDE_MASK=0x04: channels 0,1,3 armed, 2 not */
    uint8_t bitmap = pyro_mgr_get_arm_bitmap();
    TEST_ASSERT_EQUAL_HEX8(0x0B, bitmap);  /* 0b1011 = ch 0,1,3 */
}

void TEST_AUTOARM_02_skips_no_continuity(void)
{
    force_no_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    clear_events();

    pyro_mgr_auto_arm_flight();

    /* No channels should be armed (no continuity) */
    TEST_ASSERT_EQUAL_HEX8(0x00, pyro_mgr_get_arm_bitmap());
}

void TEST_AUTOARM_03_idempotent_already_armed(void)
{
    force_all_continuity();
    /* Use COAST (not BOOST) to avoid transition_to auto-arming */
    drive_fsm_to(FSM_STATE_COAST);

    /* Pre-arm channel 0 (CAC path, 1-indexed: ch 1) */
    pyro_mgr_set_arm(1, true);
    clear_events();

    /* Auto-arm should not re-emit FC_EVT_ARM for channel 0 (already armed) */
    pyro_mgr_auto_arm_flight();

    /* Check that FC_EVT_ARM was NOT emitted for channel 0 */
    bool ch0_armed_event = has_event(FC_EVT_ARM, (0 << 8) | 0x01);
    TEST_ASSERT_FALSE(ch0_armed_event);

    /* But ch 1 and ch 3 should have ARM events */
    bool ch1_armed_event = has_event(FC_EVT_ARM, (1 << 8) | 0x01);
    bool ch3_armed_event = has_event(FC_EVT_ARM, (3 << 8) | 0x01);
    TEST_ASSERT_TRUE(ch1_armed_event);
    TEST_ASSERT_TRUE(ch3_armed_event);
}

void TEST_AUTOARM_04_emits_arm_events(void)
{
    force_all_continuity();
    /* Use COAST (not BOOST) to avoid transition_to auto-arming */
    drive_fsm_to(FSM_STATE_COAST);
    clear_events();

    pyro_mgr_auto_arm_flight();

    /* Should emit FC_EVT_ARM for channels 0, 1, 3 (not 2, excluded) */
    int arm_count = count_events(FC_EVT_ARM);
    TEST_ASSERT_EQUAL_INT(3, arm_count);

    /* Verify event data format: (ch << 8) | 0x01 */
    TEST_ASSERT_TRUE(has_event(FC_EVT_ARM, (0 << 8) | 0x01));
    TEST_ASSERT_TRUE(has_event(FC_EVT_ARM, (1 << 8) | 0x01));
    TEST_ASSERT_TRUE(has_event(FC_EVT_ARM, (3 << 8) | 0x01));
    /* Channel 2 excluded — no ARM event */
    TEST_ASSERT_FALSE(has_event(FC_EVT_ARM, (2 << 8) | 0x01));
}

void TEST_AUTOARM_05_skips_excluded_channels(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    clear_events();

    pyro_mgr_auto_arm_flight();

    /* Channel 2 should NOT be armed (excluded) */
    TEST_ASSERT_FALSE(pyro_mgr_get_arm_bitmap() & 0x04);
    /* No ARM event for channel 2 */
    TEST_ASSERT_FALSE(has_event(FC_EVT_ARM, (2 << 8) | 0x01));
}

/* ================================================================== */
/*  TEST_AUTOFIRE: Auto-fire precondition tests (7 tests)              */
/* ================================================================== */

void TEST_AUTOFIRE_01_succeeds_armed_continuity_not_pad(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    pyro_mgr_auto_arm_flight();
    clear_events();

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_TRUE(pyro_mgr_is_firing());
}

void TEST_AUTOFIRE_02_fails_not_armed(void)
{
    force_all_continuity();
    /* Use COAST (not BOOST) to avoid transition_to auto-arming */
    drive_fsm_to(FSM_STATE_COAST);
    /* Don't arm */

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void TEST_AUTOFIRE_03_fails_no_continuity(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    pyro_mgr_auto_arm_flight();

    /* Remove continuity */
    force_no_continuity();

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void TEST_AUTOFIRE_04_fails_invalid_channel(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    pyro_mgr_auto_arm_flight();

    int ret = pyro_mgr_auto_fire(4, 1000);
    TEST_ASSERT_EQUAL_INT(-1, ret);

    ret = pyro_mgr_auto_fire(255, 1000);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void TEST_AUTOFIRE_05_fails_already_firing(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    pyro_mgr_auto_arm_flight();

    /* Fire channel 0 */
    pyro_mgr_auto_fire(0, 1000);
    clear_events();

    /* Try to fire again while still firing */
    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(-1, ret);
}

void TEST_AUTOFIRE_06_caps_duration(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    pyro_mgr_auto_arm_flight();
    clear_events();

    /* Request 5000ms but should cap at PYRO_MAX_FIRE_MS (2000) */
    int ret = pyro_mgr_auto_fire(0, 5000);
    TEST_ASSERT_EQUAL_INT(0, ret);

    /* After 2001ms, fire should auto-stop */
    mock_tick_advance(2001);
    pyro_mgr_tick();
    TEST_ASSERT_FALSE(pyro_mgr_is_firing());
}

void TEST_AUTOFIRE_07_emits_pyro_event(void)
{
    force_all_continuity();
    drive_fsm_to(FSM_STATE_BOOST);
    pyro_mgr_auto_arm_flight();
    clear_events();

    pyro_mgr_auto_fire(1, 1000);

    /* Should emit FC_EVT_PYRO with data = (ch << 8) | (duration & 0xFF) */
    /* ch=1, duration=1000, 1000 & 0xFF = 0xE8 */
    TEST_ASSERT_TRUE(has_event(FC_EVT_PYRO,
                               ((uint16_t)1 << 8) | (1000 & 0xFF)));
}

/* ================================================================== */
/*  TEST_INTEGRATION: Integration tests (2 tests)                      */
/* ================================================================== */

void TEST_INTEGRATION_01_boost_auto_arms_then_apogee_fires(void)
{
    /* Drive FSM from PAD to BOOST — should auto-arm */
    force_all_continuity();

    /* Simulate launch: set up fsm_input_t and tick */
    fsm_set_tick(0);
    mock_tick_set(0);
    flight_fsm_init();
    pyro_mgr_init(&hadc1, &hadc2, &hadc3);
    force_all_continuity();
    clear_events();

    /* Force to BOOST to trigger auto-arm via transition_to */
    drive_fsm_to(FSM_STATE_BOOST);

    /* Verify channels were auto-armed (excluding ch 2) */
    uint8_t bitmap = pyro_mgr_get_arm_bitmap();
    TEST_ASSERT_EQUAL_HEX8(0x0B, bitmap);  /* ch 0, 1, 3 */

    /* Now simulate apogee fire on channel 0 */
    clear_events();
    drive_fsm_to(FSM_STATE_APOGEE);

    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_PYRO));
}

void TEST_INTEGRATION_02_full_flight_arm_fire_disarm(void)
{
    fsm_set_tick(0);
    mock_tick_set(0);
    flight_fsm_init();
    pyro_mgr_init(&hadc1, &hadc2, &hadc3);
    force_all_continuity();
    clear_events();

    /* BOOST: auto-arm */
    drive_fsm_to(FSM_STATE_BOOST);
    TEST_ASSERT_EQUAL_HEX8(0x0B, pyro_mgr_get_arm_bitmap());

    /* COAST */
    drive_fsm_to(FSM_STATE_COAST);

    /* APOGEE: fire apogee channel (ch 0) */
    drive_fsm_to(FSM_STATE_APOGEE);
    clear_events();
    int ret = pyro_mgr_auto_fire(0, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);

    /* Let fire complete */
    mock_tick_advance(1001);
    fsm_set_tick(fsm_get_tick() + 1001);
    pyro_mgr_tick();

    /* MAIN: fire main channel (ch 1) */
    drive_fsm_to(FSM_STATE_MAIN);
    clear_events();
    ret = pyro_mgr_auto_fire(1, 1000);
    TEST_ASSERT_EQUAL_INT(0, ret);

    /* Let fire complete */
    mock_tick_advance(1001);
    fsm_set_tick(fsm_get_tick() + 1001);
    pyro_mgr_tick();

    /* LANDED: disarm all */
    drive_fsm_to(FSM_STATE_LANDED);
    pyro_mgr_disarm_all();
    TEST_ASSERT_EQUAL_HEX8(0x00, pyro_mgr_get_arm_bitmap());
}

/* ================================================================== */
/*  main()                                                             */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* PAD lockout */
    RUN_TEST(TEST_PAD_LOCK_01_auto_fire_returns_neg1_on_pad);
    RUN_TEST(TEST_PAD_LOCK_02_auto_fire_succeeds_on_boost);
    RUN_TEST(TEST_PAD_LOCK_03_auto_fire_succeeds_on_coast);
    RUN_TEST(TEST_PAD_LOCK_04_auto_fire_succeeds_on_apogee);
    RUN_TEST(TEST_PAD_LOCK_05_auto_fire_succeeds_on_main);

    /* Channel exclusion (PYRO_EXCLUDE_MASK=0x04, ch 2 excluded) */
    RUN_TEST(TEST_EXCLUDE_01_excluded_ch_not_armed_by_auto_arm);
    RUN_TEST(TEST_EXCLUDE_02_excluded_ch_rejected_by_auto_fire);
    RUN_TEST(TEST_EXCLUDE_03_non_excluded_ch_armed_by_auto_arm);
    RUN_TEST(TEST_EXCLUDE_04_non_excluded_ch_accepted_by_auto_fire);

    /* Auto-arm */
    RUN_TEST(TEST_AUTOARM_01_arms_all_with_continuity);
    RUN_TEST(TEST_AUTOARM_02_skips_no_continuity);
    RUN_TEST(TEST_AUTOARM_03_idempotent_already_armed);
    RUN_TEST(TEST_AUTOARM_04_emits_arm_events);
    RUN_TEST(TEST_AUTOARM_05_skips_excluded_channels);

    /* Auto-fire */
    RUN_TEST(TEST_AUTOFIRE_01_succeeds_armed_continuity_not_pad);
    RUN_TEST(TEST_AUTOFIRE_02_fails_not_armed);
    RUN_TEST(TEST_AUTOFIRE_03_fails_no_continuity);
    RUN_TEST(TEST_AUTOFIRE_04_fails_invalid_channel);
    RUN_TEST(TEST_AUTOFIRE_05_fails_already_firing);
    RUN_TEST(TEST_AUTOFIRE_06_caps_duration);
    RUN_TEST(TEST_AUTOFIRE_07_emits_pyro_event);

    /* Integration */
    RUN_TEST(TEST_INTEGRATION_01_boost_auto_arms_then_apogee_fires);
    RUN_TEST(TEST_INTEGRATION_02_full_flight_arm_fire_disarm);

    return UNITY_END();
}
