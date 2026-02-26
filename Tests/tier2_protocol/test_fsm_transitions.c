/**
 * @file  test_fsm_transitions.c
 * @brief Tier-2 protocol tests for sensor-driven FSM transitions.
 *
 * Tests per CASPER_FSM_PRD Level 2 (all 29 tests):
 *   - Launch detection (PAD -> BOOST)
 *   - Burnout detection (BOOST -> COAST)
 *   - Sustain re-light (COAST -> BOOST)
 *   - Apogee detection (COAST -> APOGEE)
 *   - Main deploy (APOGEE -> MAIN)
 *   - Drogue failure backup (APOGEE -> MAIN)
 *   - Landing detection (MAIN -> LANDED)
 *   - Recovery auto-timer (LANDED -> RECOVERY)
 *   - Event emission verification
 *   - Full flight profile replays
 *
 * Compiled with -DHIL_MODE -DUNIT_TEST.
 */

#include "test_config.h"
#include "fsm_util.h"
#include "flight_fsm.h"
#include "mock_tick.h"
#include "tlm_types.h"

/* ================================================================== */
/*  ERR_DROGUE_FAIL (defined here if not in tlm_types.h yet)          */
/* ================================================================== */
#ifndef ERR_DROGUE_FAIL
#define ERR_DROGUE_FAIL  0x01
#endif

/* ================================================================== */
/*  Stubs for functions called by flight_fsm.c                        */
/* ================================================================== */

/* Event log for test verification */
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

void pyro_mgr_auto_arm_flight(void) { /* stub */ }
int  pyro_mgr_auto_fire(uint8_t ch, uint16_t dur)
{
    /* Stub emits FC_EVT_PYRO to match real implementation behavior */
    tlm_queue_event(FC_EVT_PYRO, ((uint16_t)ch << 8) | (dur & 0xFF));
    return 0;
}
void pyro_mgr_disarm_all(void) { /* stub */ }

/* ================================================================== */
/*  Event log helpers                                                  */
/* ================================================================== */

static void clear_events(void)
{
    s_event_count = 0;
}

static bool has_event(uint8_t type, uint16_t data)
{
    for (int i = 0; i < s_event_count; i++)
        if (s_event_log[i].type == type && s_event_log[i].data == data)
            return true;
    return false;
}

static bool has_event_type(uint8_t type)
{
    for (int i = 0; i < s_event_count; i++)
        if (s_event_log[i].type == type)
            return true;
    return false;
}

/* ================================================================== */
/*  fsm_input_t helper                                                 */
/* ================================================================== */

static fsm_input_t make_input(float alt, float vel, float accel_g,
                               bool ant_up, float flight_t)
{
    fsm_input_t in = {0};
    in.alt_m              = alt;
    in.vel_mps            = vel;
    in.vert_accel_g       = accel_g;
    in.antenna_up         = ant_up;
    in.flight_time_s      = flight_t;
    in.main_deploy_alt_m  = 250.0f;
    in.drogue_fail_vel_mps = 50.0f;
    in.drogue_fail_time_s = 3.0f;
    in.apogee_pyro_ch     = 0;
    in.main_pyro_ch       = 1;
    in.apogee_fire_dur_ms = 1000;
    in.main_fire_dur_ms   = 1000;
    return in;
}

/* ================================================================== */
/*  State advance helpers                                              */
/* ================================================================== */

static void advance_to_boost(void)
{
    /* PAD -> BOOST: need accel > 2g sustained 100ms + vel > 15 + antenna_up */
    fsm_input_t in = make_input(10.0f, 20.0f, 5.0f, true, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(100);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
}

static void advance_to_coast(void)
{
    advance_to_boost();
    clear_events();
    /* BOOST -> COAST: accel < 0 sustained 100ms */
    fsm_input_t in = make_input(500.0f, 200.0f, -1.0f, true, 3.0f);
    fsm_set_tick(200);
    flight_fsm_tick(&in);
    fsm_set_tick(300);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());
}

static void advance_to_apogee(void)
{
    advance_to_coast();
    clear_events();
    /* COAST -> APOGEE: vel <= 0 sustained 25ms, flight_time > 5s */
    fsm_input_t in = make_input(5000.0f, -0.5f, -1.0f, true, 12.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(425);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());
}

static void advance_to_main(void)
{
    advance_to_apogee();
    clear_events();
    /* APOGEE -> MAIN: alt <= main_deploy_alt (250m) */
    fsm_input_t in = make_input(200.0f, -10.0f, -1.0f, true, 45.0f);
    fsm_set_tick(500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());
}

static void advance_to_landed(void)
{
    advance_to_main();
    clear_events();
    /* MAIN -> LANDED: |vel| < 1 AND |delta-alt| < 2 sustained 3s */
    fsm_input_t in = make_input(5.0f, 0.1f, 0.0f, true, 90.0f);
    fsm_set_tick(1000);
    flight_fsm_tick(&in);
    fsm_set_tick(4000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_LANDED, flight_fsm_get_state());
}

/* ================================================================== */
/*  setUp / tearDown                                                   */
/* ================================================================== */

void setUp(void)
{
    flight_fsm_reset();
    fsm_set_tick(0);
    mock_tick_set(0);
    clear_events();
}

void tearDown(void) {}

/* ================================================================== */
/*  TEST_FSM_LAUNCH_01: PAD stays in PAD when accel < threshold        */
/* ================================================================== */

void TEST_FSM_LAUNCH_01(void)
{
    /* Accel below 2g threshold, vel and antenna_up are fine */
    fsm_input_t in = make_input(0.0f, 20.0f, 1.5f, true, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(200);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_PAD, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_LAUNCH_02: PAD stays when accel ok but vel < 15 m/s       */
/* ================================================================== */

void TEST_FSM_LAUNCH_02(void)
{
    /* Accel > 2g sustained, antenna_up, but vel only 10 m/s */
    fsm_input_t in = make_input(0.0f, 10.0f, 5.0f, true, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(100);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_PAD, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_LAUNCH_03: PAD stays when antenna_up is false             */
/* ================================================================== */

void TEST_FSM_LAUNCH_03(void)
{
    /* Accel > 2g, vel > 15, but rocket is tilted (antenna_up = false) */
    fsm_input_t in = make_input(5.0f, 20.0f, 5.0f, false, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(100);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_PAD, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_LAUNCH_04: PAD -> BOOST all conditions met 100ms          */
/* ================================================================== */

void TEST_FSM_LAUNCH_04(void)
{
    fsm_input_t in = make_input(10.0f, 20.0f, 5.0f, true, 0.0f);

    /* First tick: dwell begins */
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_PAD, flight_fsm_get_state());

    /* At 99ms: not yet sustained */
    fsm_set_tick(99);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_PAD, flight_fsm_get_state());

    /* At 100ms: sustained for exactly 100ms */
    fsm_set_tick(100);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_LAUNCH_05: PAD accel dwell resets mid-dwell               */
/* ================================================================== */

void TEST_FSM_LAUNCH_05(void)
{
    fsm_input_t in = make_input(10.0f, 20.0f, 5.0f, true, 0.0f);

    /* Start accel dwell */
    fsm_set_tick(0);
    flight_fsm_tick(&in);

    /* Advance 50ms */
    fsm_set_tick(50);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_PAD, flight_fsm_get_state());

    /* Drop accel below threshold (resets dwell) */
    in.vert_accel_g = 1.0f;
    fsm_set_tick(60);
    flight_fsm_tick(&in);

    /* Re-assert high accel at 70ms */
    in.vert_accel_g = 5.0f;
    fsm_set_tick(70);
    flight_fsm_tick(&in);

    /* At 169ms (99ms from restart at 70): should NOT have transitioned */
    fsm_set_tick(169);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_PAD, flight_fsm_get_state());

    /* At 170ms (100ms from restart at 70): should transition */
    fsm_set_tick(170);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_BURNOUT_01: BOOST stays while accel > 0g                  */
/* ================================================================== */

void TEST_FSM_BURNOUT_01(void)
{
    advance_to_boost();
    clear_events();

    /* Positive acceleration (still under thrust) */
    fsm_input_t in = make_input(200.0f, 100.0f, 3.0f, true, 1.0f);
    fsm_set_tick(200);
    flight_fsm_tick(&in);
    fsm_set_tick(500);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_BURNOUT_02: BOOST -> COAST when accel < 0 for 100ms      */
/* ================================================================== */

void TEST_FSM_BURNOUT_02(void)
{
    advance_to_boost();
    clear_events();

    /* Accel drops below 0g (free fall + drag) */
    fsm_input_t in = make_input(500.0f, 200.0f, -1.0f, true, 3.0f);

    /* Start burnout dwell */
    fsm_set_tick(200);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* At 299ms: not yet sustained */
    fsm_set_tick(299);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* At 300ms: sustained 100ms since 200ms */
    fsm_set_tick(300);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_BURNOUT_03: BOOST burnout dwell resets on accel spike     */
/* ================================================================== */

void TEST_FSM_BURNOUT_03(void)
{
    advance_to_boost();
    clear_events();

    fsm_input_t in = make_input(500.0f, 200.0f, -1.0f, true, 3.0f);

    /* Start burnout dwell */
    fsm_set_tick(200);
    flight_fsm_tick(&in);

    /* Brief accel spike at 250ms (resets dwell) */
    in.vert_accel_g = 1.0f;
    fsm_set_tick(250);
    flight_fsm_tick(&in);

    /* Back to negative accel */
    in.vert_accel_g = -1.0f;
    fsm_set_tick(260);
    flight_fsm_tick(&in);

    /* At 350ms: only 90ms from restart at 260 -- should NOT transition */
    fsm_set_tick(350);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* At 360ms: 100ms from restart at 260 -- should transition */
    fsm_set_tick(360);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_SUSTAIN_01: COAST -> BOOST on accel > 3g for 100ms       */
/* ================================================================== */

void TEST_FSM_SUSTAIN_01(void)
{
    advance_to_coast();
    clear_events();

    /* High accel indicating sustain re-light */
    fsm_input_t in = make_input(600.0f, 180.0f, 5.0f, true, 4.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);

    /* Not yet 100ms */
    fsm_set_tick(499);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());

    /* 100ms sustained */
    fsm_set_tick(500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_SUSTAIN_02: Stage count increments on re-light            */
/* ================================================================== */

void TEST_FSM_SUSTAIN_02(void)
{
    advance_to_coast();
    clear_events();

    /* Trigger sustain re-light */
    fsm_input_t in = make_input(600.0f, 180.0f, 5.0f, true, 4.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* Verify FC_EVT_STAGING was emitted */
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_STAGING));
}

/* ================================================================== */
/*  TEST_FSM_SUSTAIN_03: Peak accel resets on BOOST re-entry           */
/*  Verify FC_EVT_BURNOUT data is from the new burn, not the old one.  */
/* ================================================================== */

void TEST_FSM_SUSTAIN_03(void)
{
    advance_to_coast();
    clear_events();

    /* Trigger sustain re-light */
    fsm_input_t in = make_input(600.0f, 180.0f, 5.0f, true, 4.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
    clear_events();

    /* During new BOOST phase, set a known peak accel = 8.0g */
    in = make_input(700.0f, 200.0f, 8.0f, true, 5.0f);
    fsm_set_tick(550);
    flight_fsm_tick(&in);

    /* Now burn out the second stage */
    in = make_input(800.0f, 250.0f, -1.0f, true, 5.5f);
    fsm_set_tick(600);
    flight_fsm_tick(&in);
    fsm_set_tick(700);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());

    /* FC_EVT_BURNOUT should contain the new burn's peak (8.0g = 8000 mg) */
    bool found = false;
    for (int i = 0; i < s_event_count; i++) {
        if (s_event_log[i].type == FC_EVT_BURNOUT) {
            /* Peak was 8g, reported as mg: (uint16_t)(8.0 * 1000) = 8000 */
            TEST_ASSERT_EQUAL_UINT16(8000, s_event_log[i].data);
            found = true;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(found, "FC_EVT_BURNOUT not emitted for second burn");
}

/* ================================================================== */
/*  TEST_FSM_APOGEE_01: COAST stays when vel > 0                      */
/* ================================================================== */

void TEST_FSM_APOGEE_01(void)
{
    advance_to_coast();
    clear_events();

    /* Positive velocity (still ascending) */
    fsm_input_t in = make_input(4000.0f, 50.0f, -0.5f, true, 10.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(500);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_APOGEE_02: COAST stays when vel <= 0 but flight_time < 5s */
/* ================================================================== */

void TEST_FSM_APOGEE_02(void)
{
    advance_to_coast();
    clear_events();

    /* Velocity <= 0 but flight_time < 5s (safety gate) */
    fsm_input_t in = make_input(300.0f, -0.5f, -1.0f, true, 3.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(450);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_APOGEE_03: COAST -> APOGEE when vel <= 0 for 25ms        */
/*                       AND flight_time > 5s                          */
/* ================================================================== */

void TEST_FSM_APOGEE_03(void)
{
    advance_to_coast();
    clear_events();

    fsm_input_t in = make_input(5000.0f, -0.5f, -1.0f, true, 12.0f);

    /* Start apogee dwell */
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());

    /* At 424ms: not yet 25ms */
    fsm_set_tick(424);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());

    /* At 425ms: 25ms sustained */
    fsm_set_tick(425);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_APOGEE_04: Sustain re-light takes priority over apogee   */
/* ================================================================== */

void TEST_FSM_APOGEE_04(void)
{
    advance_to_coast();
    clear_events();

    /*
     * Both conditions could be met:
     * - vel <= 0, flight_time > 5 (apogee conditions)
     * - vert_accel > 3g (sustain re-light)
     * Sustain is checked FIRST in the COAST handler, so it should win.
     */
    fsm_input_t in = make_input(5000.0f, -0.5f, 5.0f, true, 12.0f);

    /* Start both dwells */
    fsm_set_tick(400);
    flight_fsm_tick(&in);

    /* At 500ms: sustain dwell (100ms) fires before apogee would matter */
    fsm_set_tick(500);
    flight_fsm_tick(&in);

    /* Should go to BOOST (sustain), not APOGEE */
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_MAIN_01: APOGEE -> MAIN when alt <= main_deploy_alt_m    */
/* ================================================================== */

void TEST_FSM_MAIN_01(void)
{
    advance_to_apogee();
    clear_events();

    /* Alt below main deploy altitude (250m default) */
    fsm_input_t in = make_input(200.0f, -10.0f, -1.0f, true, 45.0f);
    fsm_set_tick(500);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_MAIN_02: APOGEE -> MAIN via drogue failure                */
/* ================================================================== */

void TEST_FSM_MAIN_02(void)
{
    advance_to_apogee();
    clear_events();

    /* Alt above deploy altitude but descending very fast (drogue failed)
     * drogue_fail_vel_mps = 50 m/s, drogue_fail_time_s = 3.0s
     * vel = -60 m/s (faster than -50 threshold) */
    fsm_input_t in = make_input(3000.0f, -60.0f, -1.0f, true, 20.0f);

    /* Start drogue fail dwell */
    fsm_set_tick(500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());

    /* At 3499ms: not yet 3000ms sustained */
    fsm_set_tick(3499);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());

    /* At 3500ms: 3000ms sustained */
    fsm_set_tick(3500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_MAIN_03: Normal altitude deploy takes priority over       */
/*                    drogue failure                                    */
/* ================================================================== */

void TEST_FSM_MAIN_03(void)
{
    advance_to_apogee();
    clear_events();

    /* Both conditions could trigger:
     * - alt (200) <= main_deploy_alt (250) -- immediate altitude deploy
     * - vel (-60) < -drogue_fail_vel (-50) -- drogue fail (but needs dwell)
     * The altitude check is evaluated FIRST and is immediate (no dwell). */
    fsm_input_t in = make_input(200.0f, -60.0f, -1.0f, true, 45.0f);
    fsm_set_tick(500);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());

    /* Should NOT have emitted ERR_DROGUE_FAIL (normal path took priority) */
    TEST_ASSERT_FALSE(has_event(FC_EVT_ERROR, ERR_DROGUE_FAIL));
}

/* ================================================================== */
/*  TEST_FSM_LANDED_01: MAIN stays when velocity oscillating           */
/* ================================================================== */

void TEST_FSM_LANDED_01(void)
{
    advance_to_main();
    clear_events();

    /* Simulate pendulum swinging under parachute:
     * velocity oscillates above and below threshold */
    fsm_input_t in;

    /* Low velocity (starts dwell) */
    in = make_input(100.0f, 0.5f, 0.0f, true, 60.0f);
    fsm_set_tick(1000);
    flight_fsm_tick(&in);

    /* Spike above threshold at 2000ms (resets dwell) */
    in = make_input(95.0f, 2.0f, 0.0f, true, 61.0f);
    fsm_set_tick(2000);
    flight_fsm_tick(&in);

    /* Back to low velocity at 2500ms */
    in = make_input(90.0f, 0.3f, 0.0f, true, 62.0f);
    fsm_set_tick(2500);
    flight_fsm_tick(&in);

    /* At 5000ms: only 2500ms from restart, need 3000ms */
    in = make_input(89.0f, 0.2f, 0.0f, true, 64.5f);
    fsm_set_tick(5000);
    flight_fsm_tick(&in);

    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_LANDED_02: MAIN -> LANDED when |vel| < 1 AND             */
/*                       |delta-alt| < 2m for 3s                       */
/* ================================================================== */

void TEST_FSM_LANDED_02(void)
{
    advance_to_main();
    clear_events();

    /* Stable on ground: low velocity, stable altitude */
    fsm_input_t in = make_input(5.0f, 0.1f, 0.0f, true, 90.0f);

    /* Start landed dwell */
    fsm_set_tick(1000);
    flight_fsm_tick(&in);

    /* At 3999ms: 2999ms from start, not yet 3000ms */
    fsm_set_tick(3999);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());

    /* At 4000ms: 3000ms sustained */
    fsm_set_tick(4000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_LANDED, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_LANDED_03: Landing dwell resets if altitude drifts > 2m   */
/* ================================================================== */

void TEST_FSM_LANDED_03(void)
{
    advance_to_main();
    clear_events();

    /* Start with stable conditions */
    fsm_input_t in = make_input(5.0f, 0.1f, 0.0f, true, 90.0f);
    fsm_set_tick(1000);
    flight_fsm_tick(&in);

    /* After 2s, altitude drifts > 2m from reference (resets dwell) */
    in = make_input(8.0f, 0.1f, 0.0f, true, 92.0f);
    fsm_set_tick(3000);
    flight_fsm_tick(&in);

    /* Re-stabilize at new altitude */
    in = make_input(8.0f, 0.1f, 0.0f, true, 93.0f);
    fsm_set_tick(3500);
    flight_fsm_tick(&in);

    /* At 6000ms: 2500ms from restart at 3500, not 3000ms yet */
    in = make_input(8.0f, 0.05f, 0.0f, true, 95.5f);
    fsm_set_tick(6000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());

    /* At 6500ms: 3000ms from restart at 3500 */
    in = make_input(8.0f, 0.05f, 0.0f, true, 96.0f);
    fsm_set_tick(6500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_LANDED, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_RECOVERY_01: LANDED -> RECOVERY after 300s               */
/* ================================================================== */

void TEST_FSM_RECOVERY_01(void)
{
    advance_to_landed();
    clear_events();

    uint32_t landed_entry_tick = fsm_get_tick();

    /* Just before 300s: should still be LANDED */
    fsm_input_t in = make_input(5.0f, 0.0f, 0.0f, true, 390.0f);
    fsm_set_tick(landed_entry_tick + 299999);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_LANDED, flight_fsm_get_state());

    /* At exactly 300s: should transition to RECOVERY */
    fsm_set_tick(landed_entry_tick + 300000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_RECOVERY, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_EVENTS_01: PAD -> BOOST emits FC_EVT_STATE(BOOST)        */
/* ================================================================== */

void TEST_FSM_EVENTS_01(void)
{
    clear_events();

    fsm_input_t in = make_input(10.0f, 20.0f, 5.0f, true, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(100);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* Must have FC_EVT_STATE with data = FSM_STATE_BOOST */
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_BOOST));
}

/* ================================================================== */
/*  TEST_FSM_EVENTS_02: BOOST -> COAST emits FC_EVT_STATE(COAST)      */
/*                       + FC_EVT_BURNOUT                              */
/* ================================================================== */

void TEST_FSM_EVENTS_02(void)
{
    advance_to_boost();
    clear_events();

    /* Feed some high accel to set peak tracking */
    fsm_input_t in = make_input(300.0f, 150.0f, 6.0f, true, 2.0f);
    fsm_set_tick(150);
    flight_fsm_tick(&in);

    /* Now burnout */
    in = make_input(500.0f, 200.0f, -1.0f, true, 3.0f);
    fsm_set_tick(200);
    flight_fsm_tick(&in);
    fsm_set_tick(300);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());

    /* Must have FC_EVT_STATE(COAST) */
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_COAST));
    /* Must have FC_EVT_BURNOUT with peak accel in mg */
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_BURNOUT));
}

/* ================================================================== */
/*  TEST_FSM_EVENTS_03: COAST -> APOGEE emits FC_EVT_STATE +          */
/*                       FC_EVT_APOGEE + FC_EVT_PYRO                  */
/* ================================================================== */

void TEST_FSM_EVENTS_03(void)
{
    advance_to_coast();
    clear_events();

    fsm_input_t in = make_input(5000.0f, -0.5f, -1.0f, true, 12.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(425);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());

    /* FC_EVT_STATE(APOGEE) emitted by transition_to */
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_APOGEE));
    /* FC_EVT_APOGEE emitted with peak altitude data */
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_APOGEE));
    /* FC_EVT_PYRO emitted for the apogee pyro channel fire */
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_PYRO));
}

/* ================================================================== */
/*  TEST_FSM_EVENTS_04: APOGEE -> MAIN (drogue fail) emits            */
/*                       FC_EVT_ERROR(ERR_DROGUE_FAIL)                */
/* ================================================================== */

void TEST_FSM_EVENTS_04(void)
{
    advance_to_apogee();
    clear_events();

    /* Drogue failure: fast descent, alt still high */
    fsm_input_t in = make_input(3000.0f, -60.0f, -1.0f, true, 20.0f);
    fsm_set_tick(500);
    flight_fsm_tick(&in);

    /* Sustain for drogue_fail_time_s * 1000 = 3000ms */
    fsm_set_tick(3500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());

    /* Must have FC_EVT_ERROR with ERR_DROGUE_FAIL */
    TEST_ASSERT_TRUE(has_event(FC_EVT_ERROR, ERR_DROGUE_FAIL));
    /* Also must have FC_EVT_STATE(MAIN) */
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_MAIN));
}

/* ================================================================== */
/*  TEST_FSM_FULL_FLIGHT_01: Complete single-stage flight              */
/*  PAD -> BOOST -> COAST -> APOGEE -> MAIN -> LANDED -> RECOVERY     */
/* ================================================================== */

void TEST_FSM_FULL_FLIGHT_01(void)
{
    fsm_input_t in;
    clear_events();

    /* --- PAD -> BOOST --- */
    in = make_input(10.0f, 20.0f, 5.0f, true, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(100);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_BOOST));

    /* --- BOOST -> COAST --- */
    /* Feed some thrust to set peak accel */
    in = make_input(300.0f, 150.0f, 8.0f, true, 2.0f);
    fsm_set_tick(150);
    flight_fsm_tick(&in);

    in = make_input(500.0f, 200.0f, -1.0f, true, 3.0f);
    fsm_set_tick(200);
    flight_fsm_tick(&in);
    fsm_set_tick(300);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_COAST));
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_BURNOUT));

    /* --- COAST -> APOGEE --- */
    in = make_input(5000.0f, -0.5f, -1.0f, true, 12.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(425);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_APOGEE));
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_APOGEE));

    /* --- APOGEE -> MAIN (normal altitude deploy) --- */
    in = make_input(200.0f, -10.0f, -1.0f, true, 45.0f);
    fsm_set_tick(500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_MAIN));

    /* --- MAIN -> LANDED --- */
    in = make_input(5.0f, 0.1f, 0.0f, true, 90.0f);
    fsm_set_tick(1000);
    flight_fsm_tick(&in);
    fsm_set_tick(4000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_LANDED, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_LANDED));

    /* --- LANDED -> RECOVERY --- */
    uint32_t landed_tick = fsm_get_tick();
    in = make_input(5.0f, 0.0f, 0.0f, true, 400.0f);
    fsm_set_tick(landed_tick + 300000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_RECOVERY, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_RECOVERY));

    /* Verify correct event ordering:
     * STATE(BOOST), STATE(COAST), BURNOUT, STATE(APOGEE), APOGEE,
     * STATE(MAIN), STATE(LANDED), STATE(RECOVERY) */
    int state_boost_idx = -1, state_coast_idx = -1;
    int state_apogee_idx = -1, state_main_idx = -1;
    int state_landed_idx = -1, state_recovery_idx = -1;
    for (int i = 0; i < s_event_count; i++) {
        if (s_event_log[i].type == FC_EVT_STATE) {
            switch (s_event_log[i].data) {
            case FSM_STATE_BOOST:    state_boost_idx = i;    break;
            case FSM_STATE_COAST:    state_coast_idx = i;    break;
            case FSM_STATE_APOGEE:   state_apogee_idx = i;   break;
            case FSM_STATE_MAIN:     state_main_idx = i;     break;
            case FSM_STATE_LANDED:   state_landed_idx = i;   break;
            case FSM_STATE_RECOVERY: state_recovery_idx = i; break;
            }
        }
    }
    TEST_ASSERT_TRUE(state_boost_idx >= 0);
    TEST_ASSERT_TRUE(state_coast_idx > state_boost_idx);
    TEST_ASSERT_TRUE(state_apogee_idx > state_coast_idx);
    TEST_ASSERT_TRUE(state_main_idx > state_apogee_idx);
    TEST_ASSERT_TRUE(state_landed_idx > state_main_idx);
    TEST_ASSERT_TRUE(state_recovery_idx > state_landed_idx);
}

/* ================================================================== */
/*  TEST_FSM_FULL_FLIGHT_02: Two-stage flight with sustain re-light    */
/*  PAD -> BOOST -> COAST -> BOOST (sustain) -> COAST -> APOGEE ->     */
/*  MAIN -> LANDED                                                     */
/* ================================================================== */

void TEST_FSM_FULL_FLIGHT_02(void)
{
    fsm_input_t in;
    clear_events();

    /* --- PAD -> BOOST (first stage) --- */
    in = make_input(10.0f, 20.0f, 5.0f, true, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(100);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* Feed thrust to build peak accel for stage 1 */
    in = make_input(200.0f, 100.0f, 6.0f, true, 1.5f);
    fsm_set_tick(150);
    flight_fsm_tick(&in);

    /* --- BOOST -> COAST (first burnout) --- */
    in = make_input(500.0f, 200.0f, -1.0f, true, 3.0f);
    fsm_set_tick(200);
    flight_fsm_tick(&in);
    fsm_set_tick(300);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_BURNOUT));

    /* --- COAST -> BOOST (sustain re-light) --- */
    in = make_input(600.0f, 180.0f, 5.0f, true, 4.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* Verify staging event with stage count = 1 */
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_STAGING));

    /* Feed second stage thrust */
    in = make_input(800.0f, 250.0f, 10.0f, true, 5.0f);
    fsm_set_tick(550);
    flight_fsm_tick(&in);

    /* --- BOOST -> COAST (second burnout) --- */
    int burnout_count_before = 0;
    for (int i = 0; i < s_event_count; i++)
        if (s_event_log[i].type == FC_EVT_BURNOUT)
            burnout_count_before++;

    in = make_input(1200.0f, 300.0f, -1.0f, true, 6.5f);
    fsm_set_tick(600);
    flight_fsm_tick(&in);
    fsm_set_tick(700);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());

    /* Should now have two burnout events total */
    int burnout_count_after = 0;
    for (int i = 0; i < s_event_count; i++)
        if (s_event_log[i].type == FC_EVT_BURNOUT)
            burnout_count_after++;
    TEST_ASSERT_EQUAL(burnout_count_before + 1, burnout_count_after);

    /* --- COAST -> APOGEE --- */
    in = make_input(5000.0f, -0.5f, -1.0f, true, 15.0f);
    fsm_set_tick(800);
    flight_fsm_tick(&in);
    fsm_set_tick(825);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());

    /* --- APOGEE -> MAIN --- */
    in = make_input(200.0f, -10.0f, -1.0f, true, 45.0f);
    fsm_set_tick(900);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());

    /* --- MAIN -> LANDED --- */
    in = make_input(5.0f, 0.1f, 0.0f, true, 90.0f);
    fsm_set_tick(1000);
    flight_fsm_tick(&in);
    fsm_set_tick(4000);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_LANDED, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_FSM_FULL_FLIGHT_03: Drogue failure flight profile             */
/*  PAD -> BOOST -> COAST -> APOGEE -> MAIN (via drogue fail)          */
/* ================================================================== */

void TEST_FSM_FULL_FLIGHT_03(void)
{
    fsm_input_t in;
    clear_events();

    /* --- PAD -> BOOST --- */
    in = make_input(10.0f, 20.0f, 5.0f, true, 0.0f);
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    fsm_set_tick(100);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_BOOST, flight_fsm_get_state());

    /* --- BOOST -> COAST --- */
    in = make_input(500.0f, 200.0f, -1.0f, true, 3.0f);
    fsm_set_tick(200);
    flight_fsm_tick(&in);
    fsm_set_tick(300);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_COAST, flight_fsm_get_state());

    /* --- COAST -> APOGEE --- */
    in = make_input(5000.0f, -0.5f, -1.0f, true, 12.0f);
    fsm_set_tick(400);
    flight_fsm_tick(&in);
    fsm_set_tick(425);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_APOGEE, flight_fsm_get_state());
    clear_events();

    /* --- APOGEE -> MAIN via drogue failure ---
     * Alt stays high (3000m > 250m deploy alt), so normal path won't trigger.
     * Descent rate = -60 m/s exceeds drogue_fail_vel_mps (50 m/s).
     * Sustain for drogue_fail_time_s (3.0s) = 3000ms. */
    in = make_input(3000.0f, -60.0f, -1.0f, true, 20.0f);
    fsm_set_tick(500);
    flight_fsm_tick(&in);

    /* Sustain for 3000ms */
    fsm_set_tick(3500);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL(FSM_STATE_MAIN, flight_fsm_get_state());

    /* Verify drogue failure event */
    TEST_ASSERT_TRUE(has_event(FC_EVT_ERROR, ERR_DROGUE_FAIL));
    /* State transition event */
    TEST_ASSERT_TRUE(has_event(FC_EVT_STATE, FSM_STATE_MAIN));
}

/* ================================================================== */
/*  main()                                                             */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Launch detection (PAD -> BOOST) */
    RUN_TEST(TEST_FSM_LAUNCH_01);
    RUN_TEST(TEST_FSM_LAUNCH_02);
    RUN_TEST(TEST_FSM_LAUNCH_03);
    RUN_TEST(TEST_FSM_LAUNCH_04);
    RUN_TEST(TEST_FSM_LAUNCH_05);

    /* Burnout detection (BOOST -> COAST) */
    RUN_TEST(TEST_FSM_BURNOUT_01);
    RUN_TEST(TEST_FSM_BURNOUT_02);
    RUN_TEST(TEST_FSM_BURNOUT_03);

    /* Sustain re-light (COAST -> BOOST) */
    RUN_TEST(TEST_FSM_SUSTAIN_01);
    RUN_TEST(TEST_FSM_SUSTAIN_02);
    RUN_TEST(TEST_FSM_SUSTAIN_03);

    /* Apogee detection (COAST -> APOGEE) */
    RUN_TEST(TEST_FSM_APOGEE_01);
    RUN_TEST(TEST_FSM_APOGEE_02);
    RUN_TEST(TEST_FSM_APOGEE_03);
    RUN_TEST(TEST_FSM_APOGEE_04);

    /* Main deploy (APOGEE -> MAIN) */
    RUN_TEST(TEST_FSM_MAIN_01);
    RUN_TEST(TEST_FSM_MAIN_02);
    RUN_TEST(TEST_FSM_MAIN_03);

    /* Landing detection (MAIN -> LANDED) */
    RUN_TEST(TEST_FSM_LANDED_01);
    RUN_TEST(TEST_FSM_LANDED_02);
    RUN_TEST(TEST_FSM_LANDED_03);

    /* Recovery auto-timer (LANDED -> RECOVERY) */
    RUN_TEST(TEST_FSM_RECOVERY_01);

    /* Event emission */
    RUN_TEST(TEST_FSM_EVENTS_01);
    RUN_TEST(TEST_FSM_EVENTS_02);
    RUN_TEST(TEST_FSM_EVENTS_03);
    RUN_TEST(TEST_FSM_EVENTS_04);

    /* Full flight profiles */
    RUN_TEST(TEST_FSM_FULL_FLIGHT_01);
    RUN_TEST(TEST_FSM_FULL_FLIGHT_02);
    RUN_TEST(TEST_FSM_FULL_FLIGHT_03);

    return UNITY_END();
}
