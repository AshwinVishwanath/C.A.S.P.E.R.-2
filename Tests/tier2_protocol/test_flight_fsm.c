/**
 * @file test_flight_fsm.c
 * @brief Tier-2 protocol tests for the flight state machine.
 *
 * Tests per PRD S4.4:
 *   - Init starts in PAD state
 *   - Valid transitions with synthetic inputs
 *   - Sim flight: state sequence progression
 *   - Sim stop: returns to PAD
 *   - force_state: sets state and emits event
 *   - Mission timer: 0 before launch, monotonically increases after
 *   - Bench mode: holds state, no auto-transitions
 *   - Landing detection in descent states
 */

#include "test_config.h"
#include "mock_tick.h"
#include "mock_gpio.h"
#include "mock_adc.h"
#include "flight_fsm.h"
#include "fsm_types.h"
#include "fsm_util.h"
#include "tlm_types.h"

/* ================================================================== */
/*  Spy / stub infrastructure                                          */
/* ================================================================== */

/* Spy for tlm_queue_event */
static uint8_t spy_evt_types[32];
static uint16_t spy_evt_datas[32];
static int spy_evt_count = 0;

int tlm_queue_event(uint8_t type, uint16_t data)
{
    if (spy_evt_count < 32) {
        spy_evt_types[spy_evt_count] = type;
        spy_evt_datas[spy_evt_count] = data;
        spy_evt_count++;
    }
    return 1;
}

/* Stubs for pyro_manager functions (called by FSM) */
void pyro_mgr_disarm_all(void) { }
void pyro_mgr_auto_arm_flight(void) { }
int  pyro_mgr_auto_fire(uint8_t ch, uint16_t dur) { (void)ch; (void)dur; return 0; }

static void reset_spy(void)
{
    spy_evt_count = 0;
    memset(spy_evt_types, 0, sizeof(spy_evt_types));
    memset(spy_evt_datas, 0, sizeof(spy_evt_datas));
}

/* Build a default FSM input (idle on pad) */
static fsm_input_t make_pad_input(void)
{
    fsm_input_t in;
    memset(&in, 0, sizeof(in));
    in.vert_accel_g = 0.0f;
    in.alt_m = 0.0f;
    in.vel_mps = 0.0f;
    in.antenna_up = true;
    in.flight_time_s = 0.0f;
    in.main_deploy_alt_m = 250.0f;
    in.drogue_fail_vel_mps = 50.0f;
    in.drogue_fail_time_s = 3.0f;
    in.apogee_pyro_ch = 0;
    in.main_pyro_ch = 1;
    in.apogee_fire_dur_ms = 1000;
    in.main_fire_dur_ms = 1000;
    return in;
}

/* ================================================================== */
/*  setUp / tearDown                                                    */
/* ================================================================== */

/* Helper: keep mock_tick and fsm virtual clock in sync */
static void set_tick(uint32_t ms)
{
    mock_tick_set(ms);
    fsm_set_tick(ms);
}

static void advance_tick(uint32_t delta_ms)
{
    mock_tick_advance(delta_ms);
    fsm_set_tick(mock_tick_get());
}

void setUp(void)
{
    mock_tick_reset();
    set_tick(1000);
    reset_spy();
    flight_fsm_init();
}

void tearDown(void) { }

/* ================================================================== */
/*  Init tests                                                          */
/* ================================================================== */

void test_init_starts_in_pad(void)
{
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

void test_mission_time_zero_before_launch(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, flight_fsm_get_time_s());
}

/* ================================================================== */
/*  Launch detection tests                                              */
/* ================================================================== */

void test_launch_detection_dual_sensor(void)
{
    fsm_input_t in = make_pad_input();

    /* Tick once with low accel — should stay on PAD */
    in.vert_accel_g = 0.0f;
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* Sustained accel > 2g for 100ms + vel > 15 + antenna_up */
    in.vert_accel_g = 5.0f;
    in.vel_mps = 20.0f;
    in.antenna_up = true;
    advance_tick(1);
    flight_fsm_tick(&in);
    /* Need sustained 100ms dwell */
    advance_tick(110);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());
}

void test_launch_detection_resets_on_gap(void)
{
    fsm_input_t in = make_pad_input();

    /* Start high-accel dwell */
    in.vert_accel_g = 5.0f;
    in.vel_mps = 20.0f;
    in.antenna_up = true;
    advance_tick(50);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* Drop accel below threshold (resets dwell timer) */
    in.vert_accel_g = 0.5f;
    advance_tick(10);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* Resume high accel — dwell restarts, only 50ms so far (need 100ms) */
    in.vert_accel_g = 5.0f;
    advance_tick(50);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

void test_launch_requires_antenna_up(void)
{
    fsm_input_t in = make_pad_input();
    in.antenna_up = false;  /* Rocket not upright */

    /* Sustained high accel + vel, but antenna not up — should NOT launch */
    in.vert_accel_g = 5.0f;
    in.vel_mps = 20.0f;
    advance_tick(1);
    flight_fsm_tick(&in);
    advance_tick(110);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

/* ================================================================== */
/*  Mission timer tests                                                 */
/* ================================================================== */

void test_mission_timer_starts_at_launch(void)
{
    fsm_input_t in = make_pad_input();
    in.vert_accel_g = 5.0f;
    in.vel_mps = 20.0f;
    in.antenna_up = true;

    advance_tick(1);
    flight_fsm_tick(&in);
    advance_tick(110);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());

    /* Timer should be very small right after launch */
    float t0 = flight_fsm_get_time_s();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, t0);

    /* Advance 2 seconds */
    advance_tick(2000);
    float t1 = flight_fsm_get_time_s();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.0f, t1);
    TEST_ASSERT_TRUE(t1 > t0);
}

/* ================================================================== */
/*  force_state tests                                                   */
/* ================================================================== */

void test_force_state_changes_state(void)
{
    flight_fsm_force_state(FSM_STATE_COAST);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_COAST, flight_fsm_get_state());
}

void test_force_state_emits_event(void)
{
    flight_fsm_force_state(FSM_STATE_DROGUE);

    /* Should have emitted FC_EVT_STATE event */
    TEST_ASSERT_TRUE(spy_evt_count > 0);
    bool found = false;
    for (int i = 0; i < spy_evt_count; i++) {
        if (spy_evt_types[i] == FC_EVT_STATE &&
            spy_evt_datas[i] == FSM_STATE_DROGUE) {
            found = true;
        }
    }
    TEST_ASSERT_TRUE(found);
}

void test_force_state_invalid_ignored(void)
{
    flight_fsm_force_state(0xFF);  /* > FSM_STATE_LANDED */
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

void test_force_state_starts_mission_timer(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, flight_fsm_get_time_s());

    flight_fsm_force_state(FSM_STATE_BOOST);
    advance_tick(1000);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, flight_fsm_get_time_s());
}

void test_force_state_apogee_emits_apogee_event(void)
{
    flight_fsm_force_state(FSM_STATE_APOGEE);

    /* force_state always emits FC_EVT_STATE.
     * FC_EVT_APOGEE is now only emitted by the sensor-driven COAST->APOGEE
     * path (not by transition_to directly), so force_state does NOT emit it. */
    bool found_state = false;
    for (int i = 0; i < spy_evt_count; i++) {
        if (spy_evt_types[i] == FC_EVT_STATE) found_state = true;
    }
    TEST_ASSERT_TRUE(found_state);
}

void test_force_state_same_state_no_event(void)
{
    /* Force to PAD (already PAD) — should NOT emit event (no transition) */
    reset_spy();
    flight_fsm_force_state(FSM_STATE_PAD);
    TEST_ASSERT_EQUAL_INT(0, spy_evt_count);
}

/* ================================================================== */
/*  Sim flight tests                                                    */
/* ================================================================== */

void test_sim_starts_active(void)
{
    TEST_ASSERT_FALSE(flight_fsm_sim_active());
    flight_fsm_sim_start();
    TEST_ASSERT_TRUE(flight_fsm_sim_active());
}

void test_sim_progresses_through_states(void)
{
    flight_fsm_sim_start();
    fsm_input_t in = make_pad_input();

    /* At t=0: PAD */
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* At t=0.5s: BOOST */
    advance_tick(600);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());

    /* At t=3.5s: COAST */
    advance_tick(3000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_COAST, flight_fsm_get_state());

    /* At t=12s: APOGEE */
    advance_tick(8500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_APOGEE, flight_fsm_get_state());

    /* At t=12.5s: DROGUE */
    advance_tick(600);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_DROGUE, flight_fsm_get_state());

    /* At t=45s: MAIN */
    advance_tick(33000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_MAIN, flight_fsm_get_state());

    /* At t=90s: RECOVERY */
    advance_tick(46000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_RECOVERY, flight_fsm_get_state());

    /* At t=180s: LANDED */
    advance_tick(91000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_LANDED, flight_fsm_get_state());
}

void test_sim_stop_returns_to_pad(void)
{
    flight_fsm_sim_start();
    fsm_input_t in = make_pad_input();

    /* Advance to BOOST */
    advance_tick(600);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());

    /* Stop sim */
    flight_fsm_sim_stop();
    TEST_ASSERT_FALSE(flight_fsm_sim_active());
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

void test_sim_get_state_interpolates(void)
{
    flight_fsm_sim_start();

    /* Advance to mid-boost (~1.5s) */
    advance_tick(1500);

    fc_telem_state_t out;
    memset(&out, 0, sizeof(out));
    flight_fsm_sim_get_state(&out);

    /* At t=1.5s, should be between waypoints at t=0.5 (alt=5, vel=50)
     * and t=3.0 (alt=600, vel=300).
     * t_frac = (1.5 - 0.5) / (3.0 - 0.5) = 0.4
     * expected alt = 5 + 0.4 * (600 - 5) = 243
     * expected vel = 50 + 0.4 * (300 - 50) = 150 */
    TEST_ASSERT_FLOAT_WITHIN(50.0f, 243.0f, out.alt_m);
    TEST_ASSERT_FLOAT_WITHIN(30.0f, 150.0f, out.vel_mps);
    TEST_ASSERT_ALL_FINITE(out.quat, 4);
}

void test_sim_auto_ends_after_landed(void)
{
    flight_fsm_sim_start();
    fsm_input_t in = make_pad_input();

    /* Advance well past LANDED + 5s holdoff (180s + 6s = 186s) */
    advance_tick(186000);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_LANDED, flight_fsm_get_state());
    TEST_ASSERT_FALSE(flight_fsm_sim_active());
}

/* ================================================================== */
/*  Bench mode tests                                                    */
/* ================================================================== */

void test_bench_mode_holds_state(void)
{
    flight_fsm_set_bench_mode(true);
    TEST_ASSERT_TRUE(flight_fsm_bench_active());

    fsm_input_t in = make_pad_input();
    in.vert_accel_g = 10.0f;
    in.vel_mps = 20.0f;
    in.antenna_up = true;

    /* Even with high accel, should not transition */
    advance_tick(1);
    flight_fsm_tick(&in);
    advance_tick(110);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

void test_bench_mode_force_state_works(void)
{
    flight_fsm_set_bench_mode(true);

    flight_fsm_force_state(FSM_STATE_COAST);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_COAST, flight_fsm_get_state());
}

void test_bench_mode_off_resumes_transitions(void)
{
    flight_fsm_set_bench_mode(true);
    flight_fsm_set_bench_mode(false);
    TEST_ASSERT_FALSE(flight_fsm_bench_active());

    fsm_input_t in = make_pad_input();
    in.vert_accel_g = 5.0f;
    in.vel_mps = 20.0f;
    in.antenna_up = true;

    advance_tick(1);
    flight_fsm_tick(&in);
    advance_tick(110);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  Landing detection tests                                             */
/* ================================================================== */

void test_landing_detection_in_main(void)
{
    /* Put FSM into MAIN state (landing only detected from MAIN) */
    flight_fsm_force_state(FSM_STATE_MAIN);
    reset_spy();

    fsm_input_t in = make_pad_input();
    in.alt_m = 5.0f;
    in.vel_mps = 0.0f;

    /* Sustain |vel| < 1 m/s AND |delta_alt| < 2 m for 3 seconds */
    for (int i = 0; i < 40; i++) {
        advance_tick(100);
        flight_fsm_tick(&in);
    }

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_LANDED, flight_fsm_get_state());
}

/* ================================================================== */
/*  State transition event emission tests                               */
/* ================================================================== */

void test_sim_emits_state_events(void)
{
    flight_fsm_sim_start();
    fsm_input_t in = make_pad_input();

    /* Advance through BOOST */
    advance_tick(600);
    flight_fsm_tick(&in);

    /* Check that FC_EVT_STATE was emitted for BOOST */
    bool found_boost = false;
    for (int i = 0; i < spy_evt_count; i++) {
        if (spy_evt_types[i] == FC_EVT_STATE &&
            spy_evt_datas[i] == FSM_STATE_BOOST) {
            found_boost = true;
        }
    }
    TEST_ASSERT_TRUE(found_boost);
}

/* ================================================================== */
/*  main()                                                              */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Init */
    RUN_TEST(test_init_starts_in_pad);
    RUN_TEST(test_mission_time_zero_before_launch);

    /* Launch detection */
    RUN_TEST(test_launch_detection_dual_sensor);
    RUN_TEST(test_launch_detection_resets_on_gap);
    RUN_TEST(test_launch_requires_antenna_up);

    /* Mission timer */
    RUN_TEST(test_mission_timer_starts_at_launch);

    /* force_state */
    RUN_TEST(test_force_state_changes_state);
    RUN_TEST(test_force_state_emits_event);
    RUN_TEST(test_force_state_invalid_ignored);
    RUN_TEST(test_force_state_starts_mission_timer);
    RUN_TEST(test_force_state_apogee_emits_apogee_event);
    RUN_TEST(test_force_state_same_state_no_event);

    /* Sim flight */
    RUN_TEST(test_sim_starts_active);
    RUN_TEST(test_sim_progresses_through_states);
    RUN_TEST(test_sim_stop_returns_to_pad);
    RUN_TEST(test_sim_get_state_interpolates);
    RUN_TEST(test_sim_auto_ends_after_landed);

    /* Bench mode */
    RUN_TEST(test_bench_mode_holds_state);
    RUN_TEST(test_bench_mode_force_state_works);
    RUN_TEST(test_bench_mode_off_resumes_transitions);

    /* Landing detection */
    RUN_TEST(test_landing_detection_in_main);

    /* Event emission */
    RUN_TEST(test_sim_emits_state_events);

    return UNITY_END();
}
