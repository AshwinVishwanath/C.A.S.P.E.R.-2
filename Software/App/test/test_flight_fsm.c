/*
 * Test harness for flight FSM state transitions.
 * Compiles on host PC with: gcc -DHOST_TEST -Wall -Werror \
 *     -I../flight -I../fsm -I../telemetry -I. \
 *     test_flight_fsm.c ../fsm/flight_fsm.c -lm -o test_fsm
 */
#define HOST_TEST
#include "test_shim.h"
#include "flight_fsm.h"
#include "flight_constants.h"
#include <string.h>

/* ── Helper: build a sensor_input_t with defaults ──────────────── */
static sensor_input_t make_input(float accel_z, float ekf_alt, float ekf_vel,
                                  float baro_alt, float tilt, uint32_t time_ms)
{
    sensor_input_t si;
    memset(&si, 0, sizeof(si));
    si.accel_body_ms2[2] = accel_z;
    si.ekf_alt_m = ekf_alt;
    si.ekf_vel_mps = ekf_vel;
    si.baro_alt_agl_m = baro_alt;
    si.tilt_deg = tilt;
    si.timestamp_ms = time_ms;
    return si;
}

/* ── Helper: tick the FSM at 1ms intervals for duration_ms ───────── */
static void tick_for(sensor_input_t *si, uint32_t duration_ms)
{
    uint32_t end = si->timestamp_ms + duration_ms;
    while (si->timestamp_ms < end) {
        si->timestamp_ms++;
        test_set_tick(si->timestamp_ms);
        flight_fsm_tick(si);
    }
}

/* ══════════════════════════════════════════════════════════════════
 * Test 1: Normal single-stage flight
 * ══════════════════════════════════════════════════════════════════ */
static void test_1_single_stage(void)
{
    TEST_BEGIN("Test 1: Normal single-stage flight");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(1);

    /* PAD: 5 seconds of idle */
    sensor_input_t si = make_input(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    tick_for(&si, 5000);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_PAD, "Should stay PAD during idle");

    /* T=5.0s: acceleration spike to 60 m/s2 (6g), vel rising */
    si.accel_body_ms2[2] = 60.0f;
    si.ekf_vel_mps = 5.0f;
    si.ekf_alt_m = 1.0f;
    si.baro_alt_agl_m = 1.0f;
    tick_for(&si, 499);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_PAD, "Should still be PAD during 500ms debounce");

    /* T=5.5s: debounce complete */
    si.ekf_vel_mps = 30.0f;
    si.ekf_alt_m = 10.0f;
    si.baro_alt_agl_m = 10.0f;
    tick_for(&si, 2);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_BOOST, "Should transition to BOOST after 500ms");

    const flight_context_t *ctx = flight_fsm_get_context();
    ASSERT_TRUE(ctx->launch_detected, "launch_detected should be true");
    ASSERT_TRUE(ctx->mission_started, "mission_started should be true");

    /* T=5.5-8s: continue boosting */
    si.ekf_vel_mps = 200.0f;
    si.ekf_alt_m = 1000.0f;
    si.baro_alt_agl_m = 1000.0f;
    tick_for(&si, 2000);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_BOOST, "Should stay BOOST while accel high");

    /* T=8.0s: accel drops to 5 m/s2, vel=200 */
    si.accel_body_ms2[2] = 5.0f;
    si.ekf_vel_mps = 200.0f;
    tick_for(&si, 199);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_BOOST, "Should stay BOOST during burnout debounce");

    /* T=8.2s: burnout debounce complete */
    tick_for(&si, 2);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST, "Should transition to COAST after burnout");
    ASSERT_EQ(ctx->motor_count, 1, "motor_count should be 1");

    /* T=8.2-20s: coasting, vel decreasing, alt still rising then falling */
    for (uint32_t t = 0; t < 4000; t++) {
        si.timestamp_ms++;
        test_set_tick(si.timestamp_ms);
        float frac = (float)t / 4000.0f;
        si.ekf_vel_mps = 200.0f - 200.0f * frac;     /* 200 -> 0 */
        si.ekf_alt_m = 1000.0f + 800.0f * frac;       /* 1000 -> 1800 peak */
        si.baro_alt_agl_m = si.ekf_alt_m;
        flight_fsm_tick(&si);
    }

    /* Peak altitude ~1800. Now velocity goes negative, alt drops */
    for (uint32_t t = 0; t < 4000; t++) {
        si.timestamp_ms++;
        test_set_tick(si.timestamp_ms);
        float frac = (float)t / 4000.0f;
        si.ekf_vel_mps = -frac * 20.0f;               /* 0 -> -20 */
        si.ekf_alt_m = 1800.0f - 50.0f * frac;        /* 1800 -> 1750 */
        si.baro_alt_agl_m = si.ekf_alt_m;
        flight_fsm_tick(&si);
    }
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST, "Should still be COAST (min coast time check)");

    /* Now trigger apogee: vel clearly negative, alt falling past margin */
    si.ekf_vel_mps = -5.0f;
    si.ekf_alt_m = 1750.0f;
    si.baro_alt_agl_m = 1750.0f;
    tick_for(&si, 301);  /* 300ms debounce */
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Should be DROGUE (APOGEE is transient)");
    ASSERT_TRUE(ctx->apogee_detected, "apogee_detected should be true");

    /* Descending in DROGUE */
    for (uint32_t t = 0; t < 50000; t += 100) {
        si.timestamp_ms += 100;
        test_set_tick(si.timestamp_ms);
        si.ekf_vel_mps = -5.0f;
        si.ekf_alt_m -= 0.5f;
        si.baro_alt_agl_m = si.ekf_alt_m;
        flight_fsm_tick(&si);
    }
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Should stay DROGUE while descending");

    /* Landing: vel approaches 0, alt stable */
    si.ekf_vel_mps = 0.5f;
    si.ekf_alt_m = 2.0f;
    si.baro_alt_agl_m = 2.0f;
    tick_for(&si, 5001);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_LANDED, "Should transition to LANDED after 5s stable");

    TEST_END("Test 1: Normal single-stage flight");
}

/* ══════════════════════════════════════════════════════════════════
 * Test 2: Two-stage flight
 * ══════════════════════════════════════════════════════════════════ */
static void test_2_two_stage(void)
{
    TEST_BEGIN("Test 2: Two-stage flight");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(2);

    /* Launch */
    sensor_input_t si = make_input(60.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0);
    tick_for(&si, 501);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_BOOST, "Should be BOOST after launch");

    /* Boost for 2.5s then burnout */
    si.ekf_vel_mps = 200.0f;
    si.ekf_alt_m = 500.0f;
    si.baro_alt_agl_m = 500.0f;
    tick_for(&si, 2500);

    /* Burnout: accel drops */
    si.accel_body_ms2[2] = 5.0f;
    tick_for(&si, 201);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST_1, "Should be COAST_1 for 2-stage");

    const flight_context_t *ctx = flight_fsm_get_context();
    ASSERT_EQ(ctx->motor_count, 1, "motor_count should be 1 after first burnout");

    /* Coast for 2s, then staging: second motor ignites */
    si.accel_body_ms2[2] = 0.0f;
    si.ekf_vel_mps = 180.0f;
    tick_for(&si, 2000);

    si.accel_body_ms2[2] = 40.0f;  /* second motor! */
    tick_for(&si, 201);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_SUSTAIN, "Should be SUSTAIN after staging");

    /* Sustain for 2.5s then second burnout */
    si.ekf_vel_mps = 300.0f;
    si.ekf_alt_m = 2000.0f;
    si.baro_alt_agl_m = 2000.0f;
    tick_for(&si, 2500);

    si.accel_body_ms2[2] = 5.0f;
    tick_for(&si, 201);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST_2, "Should be COAST_2 after second burnout");
    ASSERT_EQ(ctx->motor_count, 2, "motor_count should be 2");

    /* Coast to apogee */
    si.ekf_vel_mps = -5.0f;
    si.ekf_alt_m = 4990.0f;
    si.baro_alt_agl_m = 4990.0f;
    tick_for(&si, 2001);  /* min coast time */

    /* Trigger apogee voting */
    tick_for(&si, 301);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Should be DROGUE after apogee");

    TEST_END("Test 2: Two-stage flight");
}

/* ══════════════════════════════════════════════════════════════════
 * Test 3: False launch rejection (accel < 500ms)
 * ══════════════════════════════════════════════════════════════════ */
static void test_3_false_launch(void)
{
    TEST_BEGIN("Test 3: False launch rejection");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(1);

    sensor_input_t si = make_input(50.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0);

    /* 400ms of high accel, then drops */
    tick_for(&si, 400);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_PAD, "Should still be PAD (400ms < 500ms debounce)");

    si.accel_body_ms2[2] = 0.0f;
    si.ekf_vel_mps = 0.0f;
    tick_for(&si, 200);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_PAD, "Should stay PAD after false spike");

    TEST_END("Test 3: False launch rejection");
}

/* ══════════════════════════════════════════════════════════════════
 * Test 4: Apogee voting -- single sensor disagreement (2-of-3)
 * ══════════════════════════════════════════════════════════════════ */
static void test_4_apogee_voting_2of3(void)
{
    TEST_BEGIN("Test 4: Apogee voting - 2 of 3 agree");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(1);

    /* Quick launch + burnout to get to COAST */
    sensor_input_t si = make_input(60.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0);
    tick_for(&si, 501);  /* PAD -> BOOST */
    si.ekf_vel_mps = 200.0f;
    si.ekf_alt_m = 500.0f;
    si.baro_alt_agl_m = 500.0f;
    tick_for(&si, 500);
    si.accel_body_ms2[2] = 5.0f;
    tick_for(&si, 201);  /* BOOST -> COAST */
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST, "Setup: should be COAST");

    /* Rise to peak */
    si.ekf_vel_mps = 100.0f;
    si.ekf_alt_m = 3000.0f;
    si.baro_alt_agl_m = 3000.0f;
    tick_for(&si, 2001);  /* exceed min coast time */

    /* Vote A: EKF vel descending, Vote B: EKF alt falling, Vote C: baro still rising */
    si.ekf_vel_mps = -3.0f;              /* Vote A: YES */
    si.ekf_alt_m = 2990.0f;              /* Vote B: YES (< peak - 2) */
    si.baro_alt_agl_m = 3005.0f;         /* Vote C: NO (still rising) */
    tick_for(&si, 301);

    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Should transition with 2-of-3 votes");

    TEST_END("Test 4: Apogee voting - 2 of 3 agree");
}

/* ══════════════════════════════════════════════════════════════════
 * Test 5: Apogee voting -- only 1 sensor agrees
 * ══════════════════════════════════════════════════════════════════ */
static void test_5_apogee_voting_1of3(void)
{
    TEST_BEGIN("Test 5: Apogee voting - only 1 of 3, no transition");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(1);

    /* Launch + burnout to COAST */
    sensor_input_t si = make_input(60.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0);
    tick_for(&si, 501);
    si.ekf_vel_mps = 200.0f;
    si.ekf_alt_m = 500.0f;
    si.baro_alt_agl_m = 500.0f;
    tick_for(&si, 500);
    si.accel_body_ms2[2] = 5.0f;
    tick_for(&si, 201);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST, "Setup: COAST");

    /* Rise to peak */
    si.ekf_vel_mps = 100.0f;
    si.ekf_alt_m = 3000.0f;
    si.baro_alt_agl_m = 3000.0f;
    tick_for(&si, 2001);

    /* Only 1 vote: EKF vel descending, but alt and baro still rising */
    si.ekf_vel_mps = -2.0f;              /* Vote A: YES */
    si.ekf_alt_m = 3001.0f;              /* Vote B: NO (above peak) */
    si.baro_alt_agl_m = 3001.0f;         /* Vote C: NO (above peak) */
    tick_for(&si, 500);

    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST, "Should stay COAST with only 1-of-3");

    TEST_END("Test 5: Apogee voting - only 1 of 3, no transition");
}

/* ══════════════════════════════════════════════════════════════════
 * Test 6: Landing stability reset
 * ══════════════════════════════════════════════════════════════════ */
static void test_6_landing_reset(void)
{
    TEST_BEGIN("Test 6: Landing stability reset");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(1);

    /* Fast-forward to DROGUE with known apogee time */
    sensor_input_t si = make_input(60.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0);
    tick_for(&si, 501);  /* BOOST */
    si.ekf_vel_mps = 200.0f;
    si.ekf_alt_m = 500.0f;
    si.baro_alt_agl_m = 500.0f;
    tick_for(&si, 500);
    si.accel_body_ms2[2] = 5.0f;
    tick_for(&si, 201);  /* COAST */
    si.ekf_vel_mps = 100.0f;
    si.ekf_alt_m = 3000.0f;
    si.baro_alt_agl_m = 3000.0f;
    tick_for(&si, 2001);
    si.ekf_vel_mps = -5.0f;
    si.ekf_alt_m = 2990.0f;
    si.baro_alt_agl_m = 2990.0f;
    tick_for(&si, 301);  /* DROGUE */
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Setup: DROGUE");

    /* Wait for landing guard (10s after apogee) */
    si.ekf_vel_mps = -5.0f;
    si.ekf_alt_m = 100.0f;
    si.baro_alt_agl_m = 100.0f;
    tick_for(&si, 10001);

    /* Attempt 1: stable for 4 seconds */
    si.ekf_vel_mps = 0.5f;
    si.ekf_alt_m = 5.0f;
    si.baro_alt_agl_m = 5.0f;
    tick_for(&si, 4000);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Should still be DROGUE (4s < 5s)");

    /* Gust bumps velocity */
    si.ekf_vel_mps = 3.0f;
    tick_for(&si, 100);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Reset: still DROGUE after gust");

    /* Attempt 2: stable for 5 seconds */
    si.ekf_vel_mps = 0.5f;
    si.ekf_alt_m = 5.0f;
    tick_for(&si, 5001);
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_LANDED, "Should be LANDED after second 5s attempt");

    TEST_END("Test 6: Landing stability reset");
}

/* ══════════════════════════════════════════════════════════════════
 * Test 7: Guard -- no apogee below 20m
 * ══════════════════════════════════════════════════════════════════ */
static void test_7_no_apogee_below_20m(void)
{
    TEST_BEGIN("Test 7: No apogee below 20m");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(1);

    /* Low-altitude launch */
    sensor_input_t si = make_input(60.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0);
    tick_for(&si, 501);  /* BOOST */

    /* Short boost */
    si.ekf_vel_mps = 15.0f;
    si.ekf_alt_m = 10.0f;
    si.baro_alt_agl_m = 10.0f;
    tick_for(&si, 500);
    si.accel_body_ms2[2] = 5.0f;
    tick_for(&si, 201);  /* COAST */
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST, "Setup: COAST");

    /* Peak at 15m then descend */
    si.ekf_alt_m = 15.0f;
    si.baro_alt_agl_m = 15.0f;
    si.ekf_vel_mps = 5.0f;
    tick_for(&si, 2001);  /* min coast time */

    si.ekf_vel_mps = -5.0f;
    si.ekf_alt_m = 12.0f;
    si.baro_alt_agl_m = 12.0f;
    tick_for(&si, 500);  /* well past debounce */

    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST, "Should NOT transition (max_alt < 20m)");

    TEST_END("Test 7: No apogee below 20m");
}

/* ══════════════════════════════════════════════════════════════════
 * Test 8: Staging fails -- still reaches apogee
 * ══════════════════════════════════════════════════════════════════ */
static void test_8_staging_fails(void)
{
    TEST_BEGIN("Test 8: Staging fails, still reaches apogee");
    test_set_tick(0);
    test_clear_events();
    flight_fsm_init(2);

    /* Launch */
    sensor_input_t si = make_input(60.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0);
    tick_for(&si, 501);  /* BOOST */

    /* Burnout */
    si.ekf_vel_mps = 200.0f;
    si.ekf_alt_m = 500.0f;
    si.baro_alt_agl_m = 500.0f;
    tick_for(&si, 500);
    si.accel_body_ms2[2] = 5.0f;
    tick_for(&si, 201);  /* COAST_1 */
    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST_1, "Should be COAST_1");

    const flight_context_t *ctx = flight_fsm_get_context();

    /* NO staging spike -- ignition failed. Coast continues. */
    si.accel_body_ms2[2] = 0.0f;
    si.ekf_vel_mps = 150.0f;
    si.ekf_alt_m = 2000.0f;
    si.baro_alt_agl_m = 2000.0f;
    tick_for(&si, 3000);

    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_COAST_1, "Should stay COAST_1 (no staging)");

    /* Apogee in COAST_1 */
    si.ekf_vel_mps = -5.0f;
    si.ekf_alt_m = 1990.0f;
    si.baro_alt_agl_m = 1990.0f;
    tick_for(&si, 301);

    ASSERT_EQ(flight_fsm_get_state(), FSM_STATE_DROGUE, "Should reach DROGUE from COAST_1");
    ASSERT_EQ(ctx->motor_count, 1, "motor_count should stay 1 (staging failed)");

    TEST_END("Test 8: Staging fails, still reaches apogee");
}

/* ── Main ───────────────────────────────────────────────────────── */
int main(void)
{
    printf("=== Flight FSM Test Harness ===\n\n");

    test_1_single_stage();
    test_2_two_stage();
    test_3_false_launch();
    test_4_apogee_voting_2of3();
    test_5_apogee_voting_1of3();
    test_6_landing_reset();
    test_7_no_apogee_below_20m();
    test_8_staging_fails();

    TEST_SUMMARY();
}
