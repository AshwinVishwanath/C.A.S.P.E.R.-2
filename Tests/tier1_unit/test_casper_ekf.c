/**
 * @file test_casper_ekf.c
 * @brief Tier-1 unit tests for the 4-state vertical EKF (casper_ekf).
 *
 * Tests cover: init, predict, baro update, ZUPT, Mach gating, GPS stubs.
 * Reference: PRD S3.2
 */

#include "test_config.h"
#include "casper_ekf.h"

#define G_ACCEL         9.80665f
#define EKF_DT          0.0024f
#define R_BARO_NOM      0.5f
#define BARO_GATE_K2    25.0f
#define P_FLOOR_BB      0.01f
#define MACH_GATE_ON    0.40f
#define MACH_GATE_OFF   0.35f

static casper_ekf_t ekf;

void setUp(void)
{
    casper_ekf_init(&ekf);
}

void tearDown(void) { }

/* ================================================================
 *  INIT TESTS
 * ================================================================ */

static void test_init_state_zero(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-12f, 0.0f, ekf.x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-12f, 0.0f, ekf.x[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-12f, 0.0f, ekf.x[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-12f, 0.0f, ekf.x[3]);
}

static void test_init_P_diagonal(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f,   ekf.P[0 * 4 + 0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, ekf.P[1 * 4 + 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.025f, ekf.P[2 * 4 + 2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.75f,  ekf.P[3 * 4 + 3]);
}

static void test_init_P_offdiag_zero(void)
{
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (r != c) {
                /* Q has off-diag [0][1] and [1][0] populated, so P0 off-diag
                 * are only from the memset(0) at init — they should be zero. */
                TEST_ASSERT_FLOAT_WITHIN(1e-12f, 0.0f, ekf.P[r * 4 + c]);
            }
        }
    }
}

static void test_init_baro_not_gated(void)
{
    TEST_ASSERT_FALSE(ekf.baro_gated);
}

static void test_init_dt(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, EKF_DT, ekf.dt);
}

/* ================================================================
 *  PREDICT TESTS
 * ================================================================ */

static void test_predict_zero_accel_stationary(void)
{
    /* Level, stationary: ned_accel = [0, 0, +9.80665] (Z-up frame).
     * a_up = 9.80665 - 9.80665 - 0 = 0. State should remain zero. */
    float ned[3] = {0.0f, 0.0f, G_ACCEL};
    float x0 = ekf.x[0];
    float x1 = ekf.x[1];

    casper_ekf_predict(&ekf, ned, EKF_DT);

    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x0, ekf.x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x1, ekf.x[1]);
}

static void test_predict_P_grows_with_Q(void)
{
    float P00_before = ekf.P[0];
    float ned[3] = {0.0f, 0.0f, G_ACCEL};

    casper_ekf_predict(&ekf, ned, EKF_DT);

    /* P should grow because Q is added */
    TEST_ASSERT_TRUE(ekf.P[0] > P00_before);
}

static void test_predict_known_vertical_accel(void)
{
    /* 10 m/s^2 upward net: ned_accel[2] = 9.80665 + 10 = 19.80665 */
    float ned[3] = {0.0f, 0.0f, G_ACCEL + 10.0f};
    float dt = EKF_DT;

    casper_ekf_predict(&ekf, ned, dt);

    /* vel += 10 * 0.0024 = 0.024 */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f * dt, ekf.x[1]);

    /* alt += 0.5 * 10 * 0.0024^2 = 2.88e-5 */
    float expected_alt = 0.5f * 10.0f * dt * dt;
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, expected_alt, ekf.x[0]);
}

static void test_predict_P_symmetry_after_100_steps(void)
{
    float ned[3] = {0.0f, 0.0f, G_ACCEL + 5.0f};

    for (int i = 0; i < 100; i++)
        casper_ekf_predict(&ekf, ned, EKF_DT);

    TEST_ASSERT_MATRIX_SYMMETRIC(ekf.P, 4, 1e-5f);
}

static void test_predict_P_diagonal_positive_after_many_cycles(void)
{
    float ned[3] = {0.0f, 0.0f, G_ACCEL + 2.0f};

    for (int i = 0; i < 10000; i++) {
        casper_ekf_predict(&ekf, ned, EKF_DT);
        /* Inject baro update every 4th step to keep P bounded */
        if (i % 4 == 0)
            casper_ekf_update_baro(&ekf, ekf.x[0]);
    }

    TEST_ASSERT_DIAG_POSITIVE(ekf.P, 4);
}

/* ================================================================
 *  BARO UPDATE TESTS
 * ================================================================ */

static void test_baro_zero_innovation(void)
{
    /* State is zero, baro reads zero: no change to state */
    float x0_before = ekf.x[0];
    float P00_before = ekf.P[0];

    casper_ekf_update_baro(&ekf, 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x0_before, ekf.x[0]);
    /* P should shrink (or stay same) since update reduces uncertainty */
    TEST_ASSERT_TRUE(ekf.P[0] <= P00_before + 1e-10f);
}

static void test_baro_5m_innovation(void)
{
    /* baro reads 5m while state is 0: altitude should correct toward 5.
     * With initial P0, S = P[0][0] + P[3][3] + 2*P[0][3] + R
     *                    = 0.1 + 0.75 + 0 + 0.5 = 1.35.
     * Innovation gate: |innov|^2 <= 25*S = 33.75 → |innov| <= 5.81.
     * 10m would be gated (100 > 33.75), so use 5m (25 <= 33.75). */
    casper_ekf_update_baro(&ekf, 5.0f);

    TEST_ASSERT_TRUE(ekf.x[0] > 0.0f);
    TEST_ASSERT_TRUE(ekf.x[0] <= 5.0f);
}

static void test_baro_joseph_symmetry_after_1000(void)
{
    float ned[3] = {0.0f, 0.0f, G_ACCEL + 1.0f};

    for (int i = 0; i < 1000; i++) {
        casper_ekf_predict(&ekf, ned, EKF_DT);
        casper_ekf_update_baro(&ekf, ekf.x[0] + 0.1f);
    }

    TEST_ASSERT_MATRIX_SYMMETRIC(ekf.P, 4, 1e-4f);
}

static void test_baro_innovation_gate_rejects(void)
{
    /* Set up small P so that S is small, then inject a huge innovation.
     * With P0 and R=0.5, S ~ P[0][0] + P[3][3] + 2*P[0][3] + R
     * = 0.1 + 0.75 + 0 + 0.5 = 1.35.
     * Gate: innovation^2 <= 25 * S = 33.75 → |innovation| <= 5.81
     * So 100m should be rejected. */
    float x_before[4];
    memcpy(x_before, ekf.x, sizeof(x_before));

    casper_ekf_update_baro(&ekf, 100.0f);

    /* State should remain unchanged (innovation gated) */
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x_before[0], ekf.x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x_before[1], ekf.x[1]);
}

static void test_baro_nan_rejected(void)
{
    float x_before[4];
    memcpy(x_before, ekf.x, sizeof(x_before));

    casper_ekf_update_baro(&ekf, NAN);

    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x_before[0], ekf.x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x_before[1], ekf.x[1]);
}

static void test_baro_inf_rejected(void)
{
    float x_before[4];
    memcpy(x_before, ekf.x, sizeof(x_before));

    casper_ekf_update_baro(&ekf, INFINITY);

    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x_before[0], ekf.x[0]);
}

static void test_baro_P_floor_baro_bias(void)
{
    /* Drive P[3][3] down with many updates, verify floor */
    float ned[3] = {0.0f, 0.0f, G_ACCEL};

    for (int i = 0; i < 500; i++) {
        casper_ekf_predict(&ekf, ned, EKF_DT);
        casper_ekf_update_baro(&ekf, 0.0f);
    }

    TEST_ASSERT_TRUE(ekf.P[3 * 4 + 3] >= P_FLOOR_BB);
}

/* ================================================================
 *  MACH GATING TESTS
 * ================================================================ */

/* Helper: compute speed of sound at altitude */
static float speed_of_sound(float alt)
{
    float alt_clamp = alt > 0.0f ? alt : 0.0f;
    float T_K = 288.15f - 0.0065f * alt_clamp;
    if (T_K < 216.65f) T_K = 216.65f;
    return sqrtf(1.4f * 287.058f * T_K);
}

static void test_mach_gate_below_035(void)
{
    /* Set velocity to Mach 0.30 — gate should NOT be active */
    float a_snd = speed_of_sound(0.0f);
    float vel_target = 0.30f * a_snd;

    /* Inject velocity by direct state manipulation for testing */
    ekf.x[1] = vel_target;

    float ned[3] = {0.0f, 0.0f, G_ACCEL};
    casper_ekf_predict(&ekf, ned, EKF_DT);

    TEST_ASSERT_FALSE(ekf.baro_gated);
}

static void test_mach_gate_above_040(void)
{
    float a_snd = speed_of_sound(0.0f);
    float vel_target = 0.45f * a_snd;

    ekf.x[1] = vel_target;

    float ned[3] = {0.0f, 0.0f, G_ACCEL};
    casper_ekf_predict(&ekf, ned, EKF_DT);

    TEST_ASSERT_TRUE(ekf.baro_gated);
}

static void test_mach_gate_hysteresis(void)
{
    float a_snd = speed_of_sound(0.0f);

    /* Go above ON threshold */
    ekf.x[1] = 0.45f * a_snd;
    float ned[3] = {0.0f, 0.0f, G_ACCEL};
    casper_ekf_predict(&ekf, ned, EKF_DT);
    TEST_ASSERT_TRUE(ekf.baro_gated);

    /* Drop to between OFF and ON (0.37) — should STAY gated (hysteresis) */
    ekf.x[1] = 0.37f * a_snd;
    casper_ekf_predict(&ekf, ned, EKF_DT);
    TEST_ASSERT_TRUE(ekf.baro_gated);

    /* Drop below OFF threshold */
    ekf.x[1] = 0.30f * a_snd;
    casper_ekf_predict(&ekf, ned, EKF_DT);
    TEST_ASSERT_FALSE(ekf.baro_gated);
}

static void test_mach_gate_baro_suppressed(void)
{
    /* Gate the baro */
    float a_snd = speed_of_sound(0.0f);
    ekf.x[1] = 0.45f * a_snd;
    float ned[3] = {0.0f, 0.0f, G_ACCEL};
    casper_ekf_predict(&ekf, ned, EKF_DT);
    TEST_ASSERT_TRUE(ekf.baro_gated);

    /* Baro update should be ignored while gated */
    float x_before[4];
    memcpy(x_before, ekf.x, sizeof(x_before));

    casper_ekf_update_baro(&ekf, 100.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x_before[0], ekf.x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, x_before[1], ekf.x[1]);
}

static void test_mach_ungate_bias_reset(void)
{
    float a_snd = speed_of_sound(0.0f);
    float ned[3] = {0.0f, 0.0f, G_ACCEL};

    /* Set nonzero biases */
    ekf.x[2] = 0.5f;
    ekf.x[3] = 2.0f;

    /* Gate */
    ekf.x[1] = 0.45f * a_snd;
    casper_ekf_predict(&ekf, ned, EKF_DT);
    TEST_ASSERT_TRUE(ekf.baro_gated);

    /* Record that we were gated */
    casper_ekf_update_baro(&ekf, 0.0f);  /* sets baro_prev_gated */

    /* Un-gate */
    ekf.x[1] = 0.20f * a_snd;
    casper_ekf_predict(&ekf, ned, EKF_DT);
    TEST_ASSERT_FALSE(ekf.baro_gated);

    /* First baro update after un-gate: should reset biases */
    casper_ekf_update_baro(&ekf, ekf.x[0]);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, ekf.x[2]);
    /* x[3] might not be exactly zero after the update itself, but the reset
     * happens before the measurement update. Just check P inflation. */
    TEST_ASSERT_TRUE(ekf.P[2 * 4 + 2] > 0.5f);  /* inflated from P_UNGATE_ACCEL_BIAS */
}

static void test_mach_ungate_R_inflated_then_nominal(void)
{
    float a_snd = speed_of_sound(0.0f);
    float ned[3] = {0.0f, 0.0f, G_ACCEL};

    /* Gate */
    ekf.x[1] = 0.45f * a_snd;
    casper_ekf_predict(&ekf, ned, EKF_DT);
    casper_ekf_update_baro(&ekf, 0.0f);  /* records prev_gated */

    /* Un-gate */
    ekf.x[1] = 0.20f * a_snd;
    casper_ekf_predict(&ekf, ned, EKF_DT);

    /* After 10 baro updates (ungate recovery), R should return to nominal.
     * We can't directly observe R, but we can check that after 20 updates
     * the convergence is tighter than during the inflated phase. */
    float alt_after_5 = 0.0f;
    for (int i = 0; i < 5; i++) {
        casper_ekf_predict(&ekf, ned, EKF_DT);
        casper_ekf_update_baro(&ekf, 5.0f);
    }
    alt_after_5 = ekf.x[0];

    /* Do 10 more updates (past ungate count) */
    for (int i = 0; i < 10; i++) {
        casper_ekf_predict(&ekf, ned, EKF_DT);
        casper_ekf_update_baro(&ekf, 5.0f);
    }
    float alt_after_15 = ekf.x[0];

    /* After more nominal-R updates, altitude should converge closer to 5m */
    TEST_ASSERT_TRUE(fabsf(alt_after_15 - 5.0f) < fabsf(alt_after_5 - 5.0f));
}

/* ================================================================
 *  ZUPT TESTS
 * ================================================================ */

static void test_zupt_corrects_velocity(void)
{
    /* Give EKF a nonzero velocity, then ZUPT should pull it toward 0 */
    ekf.x[1] = 1.0f;

    casper_ekf_update_zupt(&ekf);

    TEST_ASSERT_TRUE(fabsf(ekf.x[1]) < 1.0f);
}

static void test_zupt_repeated_converges(void)
{
    ekf.x[1] = 5.0f;

    for (int i = 0; i < 100; i++)
        casper_ekf_update_zupt(&ekf);

    /* Velocity should be very close to 0 after many ZUPTs */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ekf.x[1]);
}

static void test_zupt_does_not_gate(void)
{
    /* ZUPT uses INFINITY as gate threshold — should never reject */
    ekf.x[1] = 1000.0f;
    float v_before = ekf.x[1];

    casper_ekf_update_zupt(&ekf);

    /* Should still correct, even with massive innovation */
    TEST_ASSERT_TRUE(fabsf(ekf.x[1]) < fabsf(v_before));
}

/* ================================================================
 *  GPS STUB TESTS
 * ================================================================ */

static void test_gps_alt_stub_no_crash(void)
{
    casper_ekf_update_gps_alt(&ekf, 100.0f);
    /* No crash = pass. State should be unchanged. */
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, ekf.x[0]);
}

static void test_gps_vel_stub_no_crash(void)
{
    casper_ekf_update_gps_vel(&ekf, -5.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, ekf.x[1]);
}

/* ================================================================
 *  EXTENDED / STRESS TESTS
 * ================================================================ */

static void test_predict_no_nan_after_many_steps(void)
{
    float ned[3] = {0.0f, 0.0f, G_ACCEL + 50.0f};

    for (int i = 0; i < 5000; i++) {
        casper_ekf_predict(&ekf, ned, EKF_DT);
        if (i % 10 == 0)
            casper_ekf_update_baro(&ekf, ekf.x[0]);
    }

    TEST_ASSERT_ALL_FINITE(ekf.x, 4);
    TEST_ASSERT_ALL_FINITE(ekf.P, 16);
}

static void test_negative_accel_predicts_downward(void)
{
    /* Accel < g means decelerating or falling: a_up negative */
    float ned[3] = {0.0f, 0.0f, G_ACCEL - 5.0f};
    casper_ekf_predict(&ekf, ned, EKF_DT);

    /* Velocity should be negative (downward) */
    TEST_ASSERT_TRUE(ekf.x[1] < 0.0f);
}

/* ================================================================
 *  MAIN
 * ================================================================ */

int main(void)
{
    UNITY_BEGIN();

    /* Init */
    RUN_TEST(test_init_state_zero);
    RUN_TEST(test_init_P_diagonal);
    RUN_TEST(test_init_P_offdiag_zero);
    RUN_TEST(test_init_baro_not_gated);
    RUN_TEST(test_init_dt);

    /* Predict */
    RUN_TEST(test_predict_zero_accel_stationary);
    RUN_TEST(test_predict_P_grows_with_Q);
    RUN_TEST(test_predict_known_vertical_accel);
    RUN_TEST(test_predict_P_symmetry_after_100_steps);
    RUN_TEST(test_predict_P_diagonal_positive_after_many_cycles);

    /* Baro update */
    RUN_TEST(test_baro_zero_innovation);
    RUN_TEST(test_baro_5m_innovation);
    RUN_TEST(test_baro_joseph_symmetry_after_1000);
    RUN_TEST(test_baro_innovation_gate_rejects);
    RUN_TEST(test_baro_nan_rejected);
    RUN_TEST(test_baro_inf_rejected);
    RUN_TEST(test_baro_P_floor_baro_bias);

    /* Mach gating */
    RUN_TEST(test_mach_gate_below_035);
    RUN_TEST(test_mach_gate_above_040);
    RUN_TEST(test_mach_gate_hysteresis);
    RUN_TEST(test_mach_gate_baro_suppressed);
    RUN_TEST(test_mach_ungate_bias_reset);
    RUN_TEST(test_mach_ungate_R_inflated_then_nominal);

    /* ZUPT */
    RUN_TEST(test_zupt_corrects_velocity);
    RUN_TEST(test_zupt_repeated_converges);
    RUN_TEST(test_zupt_does_not_gate);

    /* GPS stubs */
    RUN_TEST(test_gps_alt_stub_no_crash);
    RUN_TEST(test_gps_vel_stub_no_crash);

    /* Stress */
    RUN_TEST(test_predict_no_nan_after_many_steps);
    RUN_TEST(test_negative_accel_predicts_downward);

    return UNITY_END();
}
