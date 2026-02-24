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

/* Stub for pyro_mgr_disarm_all (called by FSM on LANDED) */
void pyro_mgr_disarm_all(void) { }

static void reset_spy(void)
{
    spy_evt_count = 0;
    memset(spy_evt_types, 0, sizeof(spy_evt_types));
    memset(spy_evt_datas, 0, sizeof(spy_evt_datas));
}

/* Build a default telem state (idle on pad) */
static fc_telem_state_t make_pad_state(void)
{
    fc_telem_state_t s;
    memset(&s, 0, sizeof(s));
    s.accel_mag_g = 1.0f;
    s.baro_alt_m = 0.0f;
    s.vel_mps = 0.0f;
    s.adxl_available = true;
    s.adxl_activity = false;
    return s;
}

/* ================================================================== */
/*  setUp / tearDown                                                    */
/* ================================================================== */

void setUp(void)
{
    mock_tick_reset();
    mock_tick_set(1000);
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
    fc_telem_state_t state = make_pad_state();

    /* Tick once with low accel — should stay on PAD */
    state.accel_mag_g = 1.0f;
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* Two consecutive ticks with accel > 3g AND adxl activity */
    state.accel_mag_g = 5.0f;
    state.adxl_activity = true;
    mock_tick_advance(1);
    flight_fsm_tick(&state);
    /* Need 2 consecutive > 3g samples */
    mock_tick_advance(1);
    flight_fsm_tick(&state);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());
}

void test_launch_detection_resets_on_gap(void)
{
    fc_telem_state_t state = make_pad_state();

    /* One high-accel tick */
    state.accel_mag_g = 5.0f;
    state.adxl_activity = true;
    mock_tick_advance(1);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* One low-accel tick (resets counter) */
    state.accel_mag_g = 1.0f;
    mock_tick_advance(1);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* One more high-accel tick (counter = 1, need 2) */
    state.accel_mag_g = 5.0f;
    mock_tick_advance(1);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

void test_launch_degraded_mode_no_adxl(void)
{
    fc_telem_state_t state = make_pad_state();
    state.adxl_available = false;  /* ADXL372 not available */

    /* Two consecutive high-accel ticks — should still launch (degraded mode) */
    state.accel_mag_g = 5.0f;
    mock_tick_advance(1);
    flight_fsm_tick(&state);
    mock_tick_advance(1);
    flight_fsm_tick(&state);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  Mission timer tests                                                 */
/* ================================================================== */

void test_mission_timer_starts_at_launch(void)
{
    fc_telem_state_t state = make_pad_state();
    state.accel_mag_g = 5.0f;
    state.adxl_activity = true;

    mock_tick_advance(1);
    flight_fsm_tick(&state);
    mock_tick_advance(1);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());

    /* Timer should be very small right after launch */
    float t0 = flight_fsm_get_time_s();
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, t0);

    /* Advance 2 seconds */
    mock_tick_advance(2000);
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
    mock_tick_advance(1000);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, flight_fsm_get_time_s());
}

void test_force_state_apogee_emits_apogee_event(void)
{
    flight_fsm_force_state(FSM_STATE_APOGEE);

    /* Should emit both FC_EVT_STATE and FC_EVT_APOGEE */
    bool found_state = false;
    bool found_apogee = false;
    for (int i = 0; i < spy_evt_count; i++) {
        if (spy_evt_types[i] == FC_EVT_STATE) found_state = true;
        if (spy_evt_types[i] == FC_EVT_APOGEE) found_apogee = true;
    }
    TEST_ASSERT_TRUE(found_state);
    TEST_ASSERT_TRUE(found_apogee);
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
    fc_telem_state_t state = make_pad_state();

    /* At t=0: PAD */
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* At t=0.5s: BOOST */
    mock_tick_advance(600);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());

    /* At t=3.5s: COAST */
    mock_tick_advance(3000);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_COAST, flight_fsm_get_state());

    /* At t=12s: APOGEE */
    mock_tick_advance(8500);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_APOGEE, flight_fsm_get_state());

    /* At t=12.5s: DROGUE */
    mock_tick_advance(600);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_DROGUE, flight_fsm_get_state());

    /* At t=45s: MAIN */
    mock_tick_advance(33000);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_MAIN, flight_fsm_get_state());

    /* At t=90s: RECOVERY */
    mock_tick_advance(46000);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_RECOVERY, flight_fsm_get_state());

    /* At t=180s: LANDED */
    mock_tick_advance(91000);
    flight_fsm_tick(&state);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_LANDED, flight_fsm_get_state());
}

void test_sim_stop_returns_to_pad(void)
{
    flight_fsm_sim_start();
    fc_telem_state_t state = make_pad_state();

    /* Advance to BOOST */
    mock_tick_advance(600);
    flight_fsm_tick(&state);
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
    mock_tick_advance(1500);

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
    fc_telem_state_t state = make_pad_state();

    /* Advance well past LANDED + 5s holdoff (180s + 6s = 186s) */
    mock_tick_advance(186000);
    flight_fsm_tick(&state);

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

    fc_telem_state_t state = make_pad_state();
    state.accel_mag_g = 10.0f;
    state.adxl_activity = true;

    /* Even with high accel, should not transition */
    mock_tick_advance(1);
    flight_fsm_tick(&state);
    mock_tick_advance(1);
    flight_fsm_tick(&state);

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

    fc_telem_state_t state = make_pad_state();
    state.accel_mag_g = 5.0f;
    state.adxl_activity = true;

    mock_tick_advance(1);
    flight_fsm_tick(&state);
    mock_tick_advance(1);
    flight_fsm_tick(&state);

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  Landing detection tests                                             */
/* ================================================================== */

void test_landing_detection_in_drogue(void)
{
    /* Put FSM into DROGUE state */
    flight_fsm_force_state(FSM_STATE_DROGUE);
    reset_spy();

    fc_telem_state_t state = make_pad_state();
    state.baro_alt_m = 50.0f;
    state.vel_mps = 0.0f;

    /* Landing detection requires LANDED_MIN_DESCENT (10s) in descent state first */
    mock_tick_advance(11000);

    /* Then T_LANDED_SUSTAIN (5s) of stable baro + low velocity */
    for (int i = 0; i < 60; i++) {
        mock_tick_advance(100);
        flight_fsm_tick(&state);
    }

    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_LANDED, flight_fsm_get_state());
}

/* ================================================================== */
/*  State transition event emission tests                               */
/* ================================================================== */

void test_sim_emits_state_events(void)
{
    flight_fsm_sim_start();
    fc_telem_state_t state = make_pad_state();

    /* Advance through BOOST */
    mock_tick_advance(600);
    flight_fsm_tick(&state);

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
    RUN_TEST(test_launch_degraded_mode_no_adxl);

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
    RUN_TEST(test_landing_detection_in_drogue);

    /* Event emission */
    RUN_TEST(test_sim_emits_state_events);

    return UNITY_END();
}
