/**
 * @file test_casper_gyro_int.c
 * @brief Tier-1 unit tests for the deprecated RK4 gyro integrator (casper_gyro_int).
 *
 * Module is deprecated but still present in the codebase. Tests verify
 * initialization, integration correctness, bias estimation, and numerical stability.
 */

#include "test_config.h"
#include "casper_gyro_int.h"
#include "casper_quat.h"

#define G_ACCEL 9.80665f

static casper_gyro_int_t gi;

void setUp(void)
{
    /* Init from level accel: Y-nose convention, pad accel = [0, +g, 0] */
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    casper_gyro_int_init(&gi, accel);
}

void tearDown(void) { }

/* ================================================================
 *  INIT TESTS
 * ================================================================ */

static void test_init_quaternion_approximately_upright(void)
{
    /* With accel [0, g, 0], from_accel should produce a quaternion that
     * maps body Y (nose) to NED Z (up). The quaternion should be unit norm. */
    float norm = sqrtf(gi.q[0]*gi.q[0] + gi.q[1]*gi.q[1] +
                       gi.q[2]*gi.q[2] + gi.q[3]*gi.q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
}

static void test_init_quaternion_from_gravity(void)
{
    /* Verify that the rotation matrix from the init quaternion maps body
     * [0,1,0] (nose = Y) approximately to NED [0,0,1] (up). */
    float R[9];
    casper_quat_to_rotmat(gi.q, R);

    /* R * [0,1,0] = second column of R */
    float body_y_in_ned[3] = {R[1], R[4], R[7]};

    /* Should be approximately [0, 0, 1] */
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, body_y_in_ned[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, body_y_in_ned[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, body_y_in_ned[2]);
}

static void test_init_bias_zero(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, gi.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, gi.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, gi.gyro_bias[2]);
}

static void test_init_lpf_cutoff(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 50.0f, gi.lpf_cutoff_hz);
}

static void test_init_not_launched(void)
{
    TEST_ASSERT_FALSE(gi.launched);
}

/* ================================================================
 *  INTEGRATION TESTS
 * ================================================================ */

static void test_constant_rotation_integrates(void)
{
    /* Apply 1 rad/s about body X for 1 second.
     * Must first complete calibration (launch) to freeze bias. */
    gi.launched = true;  /* Skip calibration for this test */

    float q0[4];
    memcpy(q0, gi.q, sizeof(q0));

    float gyro[3]  = {1.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 833; i++)
        casper_gyro_int_update(&gi, gyro, accel, dt);

    /* Angle between q0 and q1: angle = 2 * acos(|dot|) */
    float dot = fabsf(q0[0]*gi.q[0] + q0[1]*gi.q[1] +
                       q0[2]*gi.q[2] + q0[3]*gi.q[3]);
    float angle_deg = 2.0f * acosf(dot) * RAD_TO_DEG;

    /* 1 radian = 57.296 degrees. Allow 2 degree tolerance for LPF settling. */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 57.296f, angle_deg);
}

static void test_zero_gyro_no_rotation(void)
{
    gi.launched = true;

    float q0[4];
    memcpy(q0, gi.q, sizeof(q0));

    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 1000; i++)
        casper_gyro_int_update(&gi, gyro, accel, dt);

    /* Quaternion should be unchanged */
    TEST_ASSERT_QUAT_EQUAL(q0, gi.q, 1e-4f);
}

static void test_multi_axis_rotation(void)
{
    gi.launched = true;

    float gyro[3]  = {0.5f, 0.3f, -0.2f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 833; i++)
        casper_gyro_int_update(&gi, gyro, accel, dt);

    /* Just verify quaternion is still unit norm and finite */
    float norm = sqrtf(gi.q[0]*gi.q[0] + gi.q[1]*gi.q[1] +
                       gi.q[2]*gi.q[2] + gi.q[3]*gi.q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
    TEST_ASSERT_ALL_FINITE(gi.q, 4);
}

/* ================================================================
 *  BIAS ESTIMATION TESTS
 * ================================================================ */

static void test_bias_convergence_on_pad(void)
{
    /* Inject constant gyro bias of 0.01 rad/s on all axes */
    float gyro[3]  = {0.01f, -0.02f, 0.015f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Run 5000 updates on pad (not launched) */
    for (int i = 0; i < 5000; i++)
        casper_gyro_int_update(&gi, gyro, accel, dt);

    /* Bias should converge toward the injected values */
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.01f,  gi.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, -0.02f, gi.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.015f, gi.gyro_bias[2]);
}

static void test_bias_frozen_after_cal_complete(void)
{
    /* The module transitions to launched after cal_samples (25000) */
    float gyro[3]  = {0.01f, 0.01f, 0.01f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Run until calibration completes */
    for (int i = 0; i < 25100; i++)
        casper_gyro_int_update(&gi, gyro, accel, dt);

    TEST_ASSERT_TRUE(gi.launched);

    float bias_frozen[3];
    memcpy(bias_frozen, gi.gyro_bias, sizeof(bias_frozen));

    /* Feed different gyro data — bias should not change */
    float gyro2[3] = {0.5f, -0.5f, 0.5f};
    for (int i = 0; i < 1000; i++)
        casper_gyro_int_update(&gi, gyro2, accel, dt);

    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bias_frozen[0], gi.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bias_frozen[1], gi.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bias_frozen[2], gi.gyro_bias[2]);
}

/* ================================================================
 *  NUMERICAL STABILITY TESTS
 * ================================================================ */

static void test_no_nan_after_many_steps(void)
{
    gi.launched = true;

    float gyro[3]  = {0.5f, -0.3f, 0.8f};
    float accel[3] = {1.0f, G_ACCEL, -0.5f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 50000; i++)
        casper_gyro_int_update(&gi, gyro, accel, dt);

    TEST_ASSERT_ALL_FINITE(gi.q, 4);
    TEST_ASSERT_ALL_FINITE(gi.gyro_filtered, 3);
    TEST_ASSERT_ALL_FINITE(gi.gyro_bias, 3);
    TEST_ASSERT_ALL_FINITE(gi.att_sigma, 3);
}

static void test_unit_norm_preserved(void)
{
    gi.launched = true;

    float gyro[3]  = {2.0f, -1.5f, 3.0f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 10000; i++)
        casper_gyro_int_update(&gi, gyro, accel, dt);

    float norm = sqrtf(gi.q[0]*gi.q[0] + gi.q[1]*gi.q[1] +
                       gi.q[2]*gi.q[2] + gi.q[3]*gi.q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, norm);
}

static void test_uncertainty_grows_monotonically(void)
{
    gi.launched = true;

    float gyro[3]  = {0.1f, 0.0f, 0.0f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    float prev_sigma = gi.att_sigma[0];

    for (int i = 0; i < 100; i++) {
        casper_gyro_int_update(&gi, gyro, accel, dt);
        TEST_ASSERT_TRUE(gi.att_sigma[0] >= prev_sigma);
        prev_sigma = gi.att_sigma[0];
    }
}

/* ================================================================
 *  LPF TESTS
 * ================================================================ */

static void test_set_lpf_cutoff(void)
{
    casper_gyro_int_set_lpf_cutoff(&gi, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, gi.lpf_cutoff_hz);
}

/* ================================================================
 *  DIFFERENT INITIAL ORIENTATIONS
 * ================================================================ */

static void test_init_from_tilted_accel(void)
{
    /* Tilted ~30 deg: accel has X component */
    float accel_tilted[3] = {G_ACCEL * sinf(30.0f * DEG_TO_RAD),
                              G_ACCEL * cosf(30.0f * DEG_TO_RAD),
                              0.0f};
    casper_gyro_int_t gi2;
    casper_gyro_int_init(&gi2, accel_tilted);

    float norm = sqrtf(gi2.q[0]*gi2.q[0] + gi2.q[1]*gi2.q[1] +
                       gi2.q[2]*gi2.q[2] + gi2.q[3]*gi2.q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
    TEST_ASSERT_ALL_FINITE(gi2.q, 4);
}

/* ================================================================
 *  MAIN
 * ================================================================ */

int main(void)
{
    UNITY_BEGIN();

    /* Init */
    RUN_TEST(test_init_quaternion_approximately_upright);
    RUN_TEST(test_init_quaternion_from_gravity);
    RUN_TEST(test_init_bias_zero);
    RUN_TEST(test_init_lpf_cutoff);
    RUN_TEST(test_init_not_launched);

    /* Integration */
    RUN_TEST(test_constant_rotation_integrates);
    RUN_TEST(test_zero_gyro_no_rotation);
    RUN_TEST(test_multi_axis_rotation);

    /* Bias estimation */
    RUN_TEST(test_bias_convergence_on_pad);
    RUN_TEST(test_bias_frozen_after_cal_complete);

    /* Numerical stability */
    RUN_TEST(test_no_nan_after_many_steps);
    RUN_TEST(test_unit_norm_preserved);
    RUN_TEST(test_uncertainty_grows_monotonically);

    /* LPF */
    RUN_TEST(test_set_lpf_cutoff);

    /* Different initial orientations */
    RUN_TEST(test_init_from_tilted_accel);

    return UNITY_END();
}
