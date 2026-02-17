/*
 * Test harness for pyro logic (Level 2).
 * Compiles on host PC with:
 *   gcc -DHOST_TEST -Wall -Werror \
 *       -I../flight -I../fsm -I../telemetry -I. \
 *       test_pyro_logic.c ../flight/pyro_logic.c -lm -o test_pyro_logic
 */
#ifndef HOST_TEST
#define HOST_TEST
#endif
#include "test_shim.h"
#include "pyro_logic.h"
#include <string.h>

/* -- Helper: build a sensor_input_t with defaults --------------- */
static sensor_input_t make_input(float ekf_alt, float ekf_vel,
                                  float baro_alt, float tilt,
                                  uint32_t time_ms)
{
    sensor_input_t si;
    memset(&si, 0, sizeof(si));
    si.ekf_alt_m = ekf_alt;
    si.ekf_vel_mps = ekf_vel;
    si.baro_alt_agl_m = baro_alt;
    si.tilt_deg = tilt;
    si.timestamp_ms = time_ms;
    return si;
}

/* -- Helper: build a default flight_context_t ------------------- */
static flight_context_t make_ctx(void)
{
    flight_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    return ctx;
}

/* -- Helper: configure a single-channel pyro_logic_t ------------ */
static void setup_single_channel(pyro_logic_t *pl, uint8_t ch_idx,
                                  pyro_role_t role)
{
    pyro_logic_init(pl);
    pl->num_channels = ch_idx + 1;
    memset(&pl->config[ch_idx], 0, sizeof(pyro_channel_config_t));
    pl->config[ch_idx].role = role;
    pl->config[ch_idx].alt_source = ALT_SOURCE_EKF;
}

/* ================================================================
 * Test 1: Default init -- no channels, no actions after tick
 * ================================================================ */
static void test_1_default_init(void)
{
    TEST_BEGIN("Test 1: Default init");

    pyro_logic_t pl;
    pyro_logic_init(&pl);

    ASSERT_EQ(pl.num_channels, 0, "num_channels should be 0 after init");
    ASSERT_EQ(pl.num_actions, 0, "num_actions should be 0 after init");

    /* Tick with no channels configured */
    flight_context_t ctx = make_ctx();
    sensor_input_t si = make_input(0.0f, 0.0f, 0.0f, 0.0f, 1000);

    pyro_logic_tick(&pl, FSM_STATE_PAD, &ctx, &si, 1000);
    ASSERT_EQ(pl.num_actions, 0, "num_actions should be 0 with no channels");

    /* Verify all configs are ROLE_NONE */
    for (int i = 0; i < PYRO_MAX_CHANNELS; i++) {
        ASSERT_EQ(pl.config[i].role, PYRO_ROLE_NONE, "All roles should be NONE");
    }

    TEST_END("Test 1: Default init");
}

/* ================================================================
 * Test 2: Apogee fires in COAST -> APOGEE transition
 * ================================================================ */
static void test_2_apogee_fires(void)
{
    TEST_BEGIN("Test 2: Apogee fires in COAST->APOGEE");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_APOGEE);

    flight_context_t ctx = make_ctx();
    sensor_input_t si = make_input(1000.0f, -5.0f, 1000.0f, 0.0f, 10000);

    /* In COAST state, apogee not yet detected */
    ctx.state = FSM_STATE_COAST;
    ctx.apogee_detected = false;

    pyro_logic_tick(&pl, FSM_STATE_COAST, &ctx, &si, 10000);
    ASSERT_EQ(pl.num_actions, 0, "No action in COAST (apogee not reached in FSM)");

    /* FSM transitions to DROGUE (APOGEE is transient) */
    ctx.state = FSM_STATE_DROGUE;
    ctx.apogee_detected = true;
    ctx.apogee_timestamp_ms = 10500;

    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 10500);
    ASSERT_EQ(pl.num_actions, 1, "Should fire on apogee detection");
    ASSERT_EQ(pl.actions[0].channel, 0, "Should fire channel 0");
    ASSERT_EQ(pl.actions[0].role, PYRO_ROLE_APOGEE, "Role should be APOGEE");
    ASSERT_TRUE(pl.state[0].fired, "Channel should be marked as fired");

    /* Tick again -- should NOT fire again (one-shot) */
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 10600);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire again (one-shot)");

    TEST_END("Test 2: Apogee fires in COAST->APOGEE");
}

/* ================================================================
 * Test 3: Apogee backup timer fires N seconds after primary
 * ================================================================ */
static void test_3_apogee_backup_timer(void)
{
    TEST_BEGIN("Test 3: Apogee backup timer");

    pyro_logic_t pl;
    pyro_logic_init(&pl);
    pl.num_channels = 2;

    /* CH0: Primary Apogee */
    memset(&pl.config[0], 0, sizeof(pyro_channel_config_t));
    pl.config[0].role = PYRO_ROLE_APOGEE;
    pl.config[0].alt_source = ALT_SOURCE_EKF;

    /* CH1: Apogee Backup, 3s timer, references CH0 as primary */
    memset(&pl.config[1], 0, sizeof(pyro_channel_config_t));
    pl.config[1].role = PYRO_ROLE_APOGEE_BACKUP;
    pl.config[1].backup_mode = BACKUP_TIMER;
    pl.config[1].backup_timer_s = 3.0f;
    pl.config[1].primary_channel = 0;
    pl.config[1].alt_source = ALT_SOURCE_EKF;

    flight_context_t ctx = make_ctx();
    sensor_input_t si = make_input(1000.0f, -5.0f, 1000.0f, 0.0f, 20000);

    /* FSM at DROGUE (apogee detected) */
    ctx.state = FSM_STATE_DROGUE;
    ctx.apogee_detected = true;
    ctx.apogee_timestamp_ms = 20000;

    /* Primary fires */
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 20000);
    ASSERT_EQ(pl.num_actions, 1, "Primary should fire");
    ASSERT_EQ(pl.actions[0].channel, 0, "Primary is CH0");
    ASSERT_TRUE(pl.state[0].fired, "Primary marked as fired");
    ASSERT_TRUE(pl.state[1].backup_started, "Backup timer should start");

    /* 2s later: backup should NOT fire yet */
    si.timestamp_ms = 22000;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 22000);
    ASSERT_EQ(pl.num_actions, 0, "Backup should not fire at 2s (< 3s)");

    /* 3s later: backup SHOULD fire */
    si.timestamp_ms = 23000;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 23000);
    ASSERT_EQ(pl.num_actions, 1, "Backup should fire at 3s");
    ASSERT_EQ(pl.actions[0].channel, 1, "Backup is CH1");
    ASSERT_EQ(pl.actions[0].role, PYRO_ROLE_APOGEE_BACKUP, "Role should be APOGEE_BACKUP");

    TEST_END("Test 3: Apogee backup timer");
}

/* ================================================================
 * Test 4: Main fires at altitude in DROGUE
 * ================================================================ */
static void test_4_main_fires_at_altitude(void)
{
    TEST_BEGIN("Test 4: Main fires at altitude");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_MAIN);
    pl.config[0].deploy_alt_m = 300.0f;

    flight_context_t ctx = make_ctx();
    ctx.state = FSM_STATE_DROGUE;
    ctx.apogee_detected = true;
    ctx.apogee_timestamp_ms = 20000;

    /* High altitude: no fire */
    sensor_input_t si = make_input(350.0f, -10.0f, 350.0f, 0.0f, 25000);
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 25000);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire above deploy altitude");

    /* Below deploy altitude: fire */
    si = make_input(290.0f, -10.0f, 290.0f, 0.0f, 26000);
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 26000);
    ASSERT_EQ(pl.num_actions, 1, "Should fire below deploy altitude");
    ASSERT_EQ(pl.actions[0].role, PYRO_ROLE_MAIN, "Role should be MAIN");

    TEST_END("Test 4: Main fires at altitude");
}

/* ================================================================
 * Test 5: Main altitude guard -- won't fire within 3s of apogee
 * ================================================================ */
static void test_5_main_altitude_guard(void)
{
    TEST_BEGIN("Test 5: Main altitude guard (3s after apogee)");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_MAIN);
    pl.config[0].deploy_alt_m = 300.0f;

    flight_context_t ctx = make_ctx();
    ctx.state = FSM_STATE_DROGUE;
    ctx.apogee_detected = true;
    ctx.apogee_timestamp_ms = 20000;

    /* Alt below threshold, but only 2s after apogee: should NOT fire */
    sensor_input_t si = make_input(250.0f, -10.0f, 250.0f, 0.0f, 22000);
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 22000);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire (only 2s after apogee, need 3s)");

    /* 2.9s after apogee: still should NOT fire */
    si.timestamp_ms = 22900;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 22900);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire at 2.9s after apogee");

    /* 3s after apogee: should fire */
    si.timestamp_ms = 23000;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 23000);
    ASSERT_EQ(pl.num_actions, 1, "Should fire at 3s after apogee");

    TEST_END("Test 5: Main altitude guard (3s after apogee)");
}

/* ================================================================
 * Test 6: Ignition fires in COAST_1
 * ================================================================ */
static void test_6_ignition_fires(void)
{
    TEST_BEGIN("Test 6: Ignition fires in COAST_1");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_IGNITION);
    pl.config[0].max_flight_angle_deg = 30.0f;

    flight_context_t ctx = make_ctx();
    ctx.state = FSM_STATE_COAST_1;
    ctx.motor_count = 1;
    ctx.mission_started = true;

    /* In COAST_1, tilt within limit: should fire */
    sensor_input_t si = make_input(800.0f, 100.0f, 800.0f, 5.0f, 10000);
    pyro_logic_tick(&pl, FSM_STATE_COAST_1, &ctx, &si, 10000);
    ASSERT_EQ(pl.num_actions, 1, "Should fire in COAST_1 with conditions met");
    ASSERT_EQ(pl.actions[0].role, PYRO_ROLE_IGNITION, "Role should be IGNITION");

    TEST_END("Test 6: Ignition fires in COAST_1");
}

/* ================================================================
 * Test 7: Ignition blocked by flight angle (latching)
 * ================================================================ */
static void test_7_ignition_blocked_by_angle(void)
{
    TEST_BEGIN("Test 7: Ignition blocked by flight angle");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_IGNITION);
    pl.config[0].max_flight_angle_deg = 30.0f;

    flight_context_t ctx = make_ctx();
    ctx.state = FSM_STATE_COAST_1;
    ctx.motor_count = 1;
    ctx.mission_started = true;

    /* First tick: tilt exceeds limit -> latching violation */
    sensor_input_t si = make_input(800.0f, 100.0f, 800.0f, 35.0f, 10000);
    pyro_logic_tick(&pl, FSM_STATE_COAST_1, &ctx, &si, 10000);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire (tilt > max_flight_angle)");
    ASSERT_TRUE(pl.state[0].flight_angle_violated,
                "flight_angle_violated should be latched");

    /* Tilt drops back to safe, but violation is latching: still no fire */
    si = make_input(800.0f, 100.0f, 800.0f, 5.0f, 10100);
    pyro_logic_tick(&pl, FSM_STATE_COAST_1, &ctx, &si, 10100);
    ASSERT_EQ(pl.num_actions, 0, "Should still not fire (latching violation)");

    /* Confirm it never fires */
    si.timestamp_ms = 11000;
    pyro_logic_tick(&pl, FSM_STATE_COAST_1, &ctx, &si, 11000);
    ASSERT_EQ(pl.num_actions, 0, "Still blocked, permanently");

    TEST_END("Test 7: Ignition blocked by flight angle");
}

/* ================================================================
 * Test 8: Window closure permanent -- APOGEE channel in LANDED
 * ================================================================ */
static void test_8_window_closure_permanent(void)
{
    TEST_BEGIN("Test 8: Window closure permanent");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_APOGEE);

    flight_context_t ctx = make_ctx();
    sensor_input_t si = make_input(0.0f, 0.0f, 0.0f, 0.0f, 100000);

    /* In COAST: window should be open */
    pyro_logic_tick(&pl, FSM_STATE_COAST, &ctx, &si, 100000);
    ASSERT_TRUE(pl.state[0].window_open, "Window should be open in COAST");
    ASSERT_TRUE(!pl.state[0].window_closed_permanently,
                "Window should not be permanently closed yet");

    /* Jump directly to LANDED state (skipping everything) */
    pyro_logic_tick(&pl, FSM_STATE_LANDED, &ctx, &si, 100100);
    ASSERT_TRUE(pl.state[0].window_closed_permanently,
                "Window should be permanently closed in LANDED");
    ASSERT_EQ(pl.num_actions, 0, "Should never fire after window closed");

    /* Even with apogee_detected, should never fire */
    ctx.apogee_detected = true;
    pyro_logic_tick(&pl, FSM_STATE_LANDED, &ctx, &si, 100200);
    ASSERT_EQ(pl.num_actions, 0, "Should never fire, window permanently closed");
    ASSERT_TRUE(!pl.state[0].fired, "Should NOT be marked as fired");

    TEST_END("Test 8: Window closure permanent");
}

/* ================================================================
 * Test 9: One-shot -- fires once, subsequent ticks produce no action
 * ================================================================ */
static void test_9_one_shot(void)
{
    TEST_BEGIN("Test 9: One-shot");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_APOGEE);

    flight_context_t ctx = make_ctx();
    ctx.state = FSM_STATE_DROGUE;
    ctx.apogee_detected = true;
    ctx.apogee_timestamp_ms = 20000;

    sensor_input_t si = make_input(1000.0f, -5.0f, 1000.0f, 0.0f, 20100);

    /* First tick: fires */
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 20100);
    ASSERT_EQ(pl.num_actions, 1, "Should fire first time");
    ASSERT_TRUE(pl.state[0].fired, "Should be marked as fired");

    /* Second tick: should NOT fire again */
    si.timestamp_ms = 20200;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 20200);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire again (one-shot)");

    /* Third tick: still no */
    si.timestamp_ms = 20300;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 20300);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire again (one-shot, tick 3)");

    TEST_END("Test 9: One-shot");
}

/* ================================================================
 * Test 10: Fire delay -- APOGEE with 2s delay
 * ================================================================ */
static void test_10_fire_delay(void)
{
    TEST_BEGIN("Test 10: Fire delay");

    pyro_logic_t pl;
    setup_single_channel(&pl, 0, PYRO_ROLE_APOGEE);
    pl.config[0].fire_delay_s = 2.0f;

    flight_context_t ctx = make_ctx();
    ctx.state = FSM_STATE_DROGUE;
    ctx.apogee_detected = true;
    ctx.apogee_timestamp_ms = 20000;

    /* Trigger met, but fire_delay = 2s */
    sensor_input_t si = make_input(1000.0f, -5.0f, 1000.0f, 0.0f, 20000);

    /* Apogee condition met -- trigger_met should be set, but no fire yet */
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 20000);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire immediately (2s delay)");
    ASSERT_TRUE(pl.state[0].trigger_met, "Trigger should be marked as met");

    /* 1s later: still no fire */
    si.timestamp_ms = 21000;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 21000);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire at 1s (< 2s delay)");

    /* 1.9s later: still no fire */
    si.timestamp_ms = 21900;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 21900);
    ASSERT_EQ(pl.num_actions, 0, "Should not fire at 1.9s (< 2s delay)");

    /* 2s later: FIRE */
    si.timestamp_ms = 22000;
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 22000);
    ASSERT_EQ(pl.num_actions, 1, "Should fire at 2s delay");
    ASSERT_TRUE(pl.state[0].fired, "Channel should be marked as fired");

    TEST_END("Test 10: Fire delay");
}

/* ================================================================
 * Test 11: Backup altitude -- MAIN_BACKUP fires at backup_alt_m
 * ================================================================ */
static void test_11_backup_altitude(void)
{
    TEST_BEGIN("Test 11: Backup altitude");

    pyro_logic_t pl;
    pyro_logic_init(&pl);
    pl.num_channels = 2;

    /* CH0: Primary Main at 300m */
    memset(&pl.config[0], 0, sizeof(pyro_channel_config_t));
    pl.config[0].role = PYRO_ROLE_MAIN;
    pl.config[0].deploy_alt_m = 300.0f;
    pl.config[0].alt_source = ALT_SOURCE_EKF;

    /* CH1: Main Backup, altitude mode at 200m, references CH0 */
    memset(&pl.config[1], 0, sizeof(pyro_channel_config_t));
    pl.config[1].role = PYRO_ROLE_MAIN_BACKUP;
    pl.config[1].backup_mode = BACKUP_ALTITUDE;
    pl.config[1].backup_alt_m = 200.0f;
    pl.config[1].primary_channel = 0;
    pl.config[1].alt_source = ALT_SOURCE_EKF;

    flight_context_t ctx = make_ctx();
    ctx.state = FSM_STATE_DROGUE;
    ctx.apogee_detected = true;
    ctx.apogee_timestamp_ms = 20000;

    /* Primary fires at 290m (below 300m deploy alt, 5s after apogee) */
    sensor_input_t si = make_input(290.0f, -10.0f, 290.0f, 0.0f, 25000);
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 25000);
    ASSERT_EQ(pl.num_actions, 1, "Primary should fire");
    ASSERT_EQ(pl.actions[0].channel, 0, "Primary is CH0");
    ASSERT_TRUE(pl.state[1].backup_started, "Backup timer/alt should start");

    /* Altitude at 250m: backup should NOT fire yet */
    si = make_input(250.0f, -10.0f, 250.0f, 0.0f, 26000);
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 26000);
    ASSERT_EQ(pl.num_actions, 0, "Backup should not fire at 250m (> 200m)");

    /* Altitude at 190m: backup SHOULD fire */
    si = make_input(190.0f, -10.0f, 190.0f, 0.0f, 27000);
    pyro_logic_tick(&pl, FSM_STATE_DROGUE, &ctx, &si, 27000);
    ASSERT_EQ(pl.num_actions, 1, "Backup should fire at 190m (< 200m)");
    ASSERT_EQ(pl.actions[0].channel, 1, "Backup is CH1");
    ASSERT_EQ(pl.actions[0].role, PYRO_ROLE_MAIN_BACKUP, "Role should be MAIN_BACKUP");

    TEST_END("Test 11: Backup altitude");
}

/* -- Main ------------------------------------------------------- */
int main(void)
{
    printf("=== Pyro Logic Test Harness ===\n\n");

    test_1_default_init();
    test_2_apogee_fires();
    test_3_apogee_backup_timer();
    test_4_main_fires_at_altitude();
    test_5_main_altitude_guard();
    test_6_ignition_fires();
    test_7_ignition_blocked_by_angle();
    test_8_window_closure_permanent();
    test_9_one_shot();
    test_10_fire_delay();
    test_11_backup_altitude();

    TEST_SUMMARY();
}
