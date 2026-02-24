/**
 * @file test_casper_attitude.c
 * @brief Tier-1 unit tests for the Mahony/RK4 attitude estimator (casper_attitude).
 *
 * Tests cover: static init, pad phase, flight phase, RK4 integration.
 * Reference: PRD S3.3
 */

#include "test_config.h"
#include "casper_attitude.h"
#include "casper_quat.h"

#define G_ACCEL     9.80665f

static casper_attitude_t att;
static casper_att_config_t cfg;

static void default_config(casper_att_config_t *c)
{
    c->Kp_grav            = 1.0f;
    c->Kp_mag_pad         = 0.5f;
    c->Kp_mag_flight      = 0.3f;
    c->Ki                 = 0.01f;
    c->gyro_lpf_cutoff_hz = 50.0f;
    c->mag_update_hz      = 10.0f;
    c->launch_accel_g     = 3.0f;
}

void setUp(void)
{
    default_config(&cfg);
    casper_att_init(&att, &cfg);
}

void tearDown(void) { }

/* ================================================================
 *  STATIC INIT TESTS
 * ================================================================ */

static void test_static_init_level_accel(void)
{
    /* Body frame: Y = nose (up on pad). Level accel = [0, +g, 0].
     * Static init computes:
     *   pitch = atan2(-ax, sqrt(ay^2 + az^2)) = atan2(0, g) = 0
     *   roll  = atan2(ay, az) = atan2(g, 0) = pi/2
     * The ZYX Euler decomposition then gives:
     *   roll_deg  = euler[1] (body Y spin)  = 0
     *   pitch_deg = euler[2] (body X tilt)  = 90 (the body X-rotation
     *              that maps body Y to NED Z)
     *   yaw_deg   = euler[0] (body Z heading) = 0 */
    float accel[3] = {0.0f, G_ACCEL, 0.0f};

    /* Feed enough samples to complete static init (timeout at 10s = 8330 samples) */
    bool done = false;
    for (int i = 0; i < 8400 && !done; i++) {
        done = casper_att_static_init(&att, accel, NULL);
    }
    TEST_ASSERT_TRUE(done);

    float roll_deg, pitch_deg, yaw_deg;
    casper_att_get_euler(&att, &roll_deg, &pitch_deg, &yaw_deg);

    /* Pitch (body X tilt) = 90 deg for nose-up on pad */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, pitch_deg);
}

static void test_static_init_no_mag_fallback(void)
{
    /* Without mag data, should timeout and set mag_available=false */
    float accel[3] = {0.0f, G_ACCEL, 0.0f};

    bool done = false;
    for (int i = 0; i < 8400 && !done; i++) {
        done = casper_att_static_init(&att, accel, NULL);
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_FALSE(att.mag_available);
}

static void test_static_init_with_mag_completes_early(void)
{
    /* Provide mag data every ~9th call (93 Hz vs 833 Hz).
     * 500 mag samples at every-9th = 4500 accel samples. */
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float mag[3]   = {20.0f, 0.0f, -40.0f};

    bool done = false;
    int count = 0;
    for (int i = 0; i < 10000 && !done; i++) {
        const float *mag_ptr = (i % 9 == 0) ? mag : NULL;
        done = casper_att_static_init(&att, accel, mag_ptr);
        count++;
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_TRUE(att.mag_available);
    /* Should complete before timeout (8330 samples) */
    TEST_ASSERT_TRUE(count < 8330);
}

static void test_static_init_yaw_zero_without_mag(void)
{
    float accel[3] = {0.0f, G_ACCEL, 0.0f};

    bool done = false;
    for (int i = 0; i < 8400 && !done; i++) {
        done = casper_att_static_init(&att, accel, NULL);
    }

    float roll_deg, pitch_deg, yaw_deg;
    casper_att_get_euler(&att, &roll_deg, &pitch_deg, &yaw_deg);

    /* Without mag, yaw should be set to 0 */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, yaw_deg);
}

/* ================================================================
 *  PAD PHASE TESTS
 * ================================================================ */

static void complete_static_init(casper_attitude_t *a)
{
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    for (int i = 0; i < 8400; i++)
        casper_att_static_init(a, accel, NULL);
}

static void test_pad_gyro_bias_convergence(void)
{
    complete_static_init(&att);

    /* Inject constant bias of 0.01 rad/s on all axes */
    float gyro[3]  = {0.01f, 0.01f, 0.01f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 5000; i++)
        casper_att_update(&att, gyro, accel, NULL, dt);

    /* Bias should converge toward 0.01 */
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.01f, att.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.01f, att.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.01f, att.gyro_bias[2]);
}

static void test_pad_gravity_correction(void)
{
    complete_static_init(&att);

    /* Tilt the body by ~5 degrees: add small X-accel */
    float tilt_rad = 5.0f * DEG_TO_RAD;
    float accel[3] = {G_ACCEL * sinf(tilt_rad), G_ACCEL * cosf(tilt_rad), 0.0f};
    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Record initial quaternion */
    float q0[4];
    casper_att_get_quaternion(&att, q0);

    /* Run pad phase updates — gravity correction should adjust quaternion */
    for (int i = 0; i < 2000; i++)
        casper_att_update(&att, gyro, accel, NULL, dt);

    float q1[4];
    casper_att_get_quaternion(&att, q1);

    /* Quaternion should have changed from initial */
    float dot = fabsf(q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3]);
    TEST_ASSERT_TRUE(dot < 0.9999f);
}

/* ================================================================
 *  FLIGHT PHASE TESTS
 * ================================================================ */

static void test_launch_detection_at_3g(void)
{
    complete_static_init(&att);

    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Below threshold: 2.5g */
    float accel_low[3] = {0.0f, 2.5f * G_ACCEL, 0.0f};
    casper_att_update(&att, gyro, accel_low, NULL, dt);
    TEST_ASSERT_FALSE(att.launched);

    /* Above threshold: 3.5g */
    float accel_high[3] = {0.0f, 3.5f * G_ACCEL, 0.0f};
    casper_att_update(&att, gyro, accel_high, NULL, dt);
    TEST_ASSERT_TRUE(att.launched);
}

static void test_gravity_correction_disabled_in_flight(void)
{
    complete_static_init(&att);

    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Launch the rocket */
    float accel_launch[3] = {0.0f, 5.0f * G_ACCEL, 0.0f};
    casper_att_update(&att, gyro, accel_launch, NULL, dt);
    TEST_ASSERT_TRUE(att.launched);

    /* Now give a tilted accel — should NOT correct because we're in flight */
    float q_before[4];
    casper_att_get_quaternion(&att, q_before);

    float accel_tilted[3] = {5.0f, G_ACCEL, 0.0f};
    for (int i = 0; i < 100; i++)
        casper_att_update(&att, gyro, accel_tilted, NULL, dt);

    float q_after[4];
    casper_att_get_quaternion(&att, q_after);

    /* Quaternion should be nearly unchanged (only gyro integration, which is zero) */
    float dot = fabsf(q_before[0]*q_after[0] + q_before[1]*q_after[1] +
                       q_before[2]*q_after[2] + q_before[3]*q_after[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, dot);
}

static void test_rk4_constant_rotation(void)
{
    complete_static_init(&att);

    /* Force launch so we get pure gyro integration (no gravity correction) */
    float accel_launch[3] = {0.0f, 5.0f * G_ACCEL, 0.0f};
    float gyro_zero[3]    = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f / 833.0f;
    casper_att_update(&att, gyro_zero, accel_launch, NULL, dt);
    TEST_ASSERT_TRUE(att.launched);

    /* Record initial quaternion */
    float q0[4];
    casper_att_get_quaternion(&att, q0);

    /* Apply 1 rad/s about body X for 1 second (833 steps) */
    float gyro_rot[3]  = {1.0f, 0.0f, 0.0f};
    float accel_nom[3] = {0.0f, G_ACCEL, 0.0f};

    for (int i = 0; i < 833; i++)
        casper_att_update(&att, gyro_rot, accel_nom, NULL, dt);

    /* After 1 second at 1 rad/s, should have rotated ~57.3 degrees (1 radian).
     * The angle between q0 and q1: angle = 2 * acos(|dot(q0,q1)|) */
    float q1[4];
    casper_att_get_quaternion(&att, q1);

    float dot = fabsf(q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3]);
    float angle_rad = 2.0f * acosf(dot);
    float angle_deg = angle_rad * RAD_TO_DEG;

    /* 1 radian = 57.296 degrees. Allow 2 degree tolerance for LPF + bias effects. */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 57.296f, angle_deg);
}

static void test_bias_frozen_at_launch(void)
{
    complete_static_init(&att);

    /* Feed some bias before launch */
    float gyro_biased[3] = {0.02f, 0.02f, 0.02f};
    float accel[3]       = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 1000; i++)
        casper_att_update(&att, gyro_biased, accel, NULL, dt);

    float bias_before[3];
    memcpy(bias_before, att.gyro_bias, sizeof(bias_before));

    /* Launch */
    float accel_launch[3] = {0.0f, 5.0f * G_ACCEL, 0.0f};
    casper_att_update(&att, gyro_biased, accel_launch, NULL, dt);
    TEST_ASSERT_TRUE(att.launched);

    /* Feed more data — bias should NOT change */
    for (int i = 0; i < 500; i++)
        casper_att_update(&att, gyro_biased, accel, NULL, dt);

    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bias_before[0], att.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bias_before[1], att.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, bias_before[2], att.gyro_bias[2]);
}

static void test_quaternion_unit_norm_maintained(void)
{
    complete_static_init(&att);

    float gyro[3]  = {0.5f, -0.3f, 0.2f};
    float accel[3] = {0.5f, G_ACCEL, -0.2f};
    float dt = 1.0f / 833.0f;

    for (int i = 0; i < 5000; i++)
        casper_att_update(&att, gyro, accel, NULL, dt);

    float q[4];
    casper_att_get_quaternion(&att, q);
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
}

static void test_no_nan_after_many_updates(void)
{
    complete_static_init(&att);

    float gyro[3]  = {0.1f, -0.05f, 0.3f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Launch first */
    float accel_launch[3] = {0.0f, 4.0f * G_ACCEL, 0.0f};
    casper_att_update(&att, gyro, accel_launch, NULL, dt);

    for (int i = 0; i < 10000; i++)
        casper_att_update(&att, gyro, accel, NULL, dt);

    TEST_ASSERT_ALL_FINITE(att.q, 4);
    TEST_ASSERT_ALL_FINITE(att.gyro_bias, 3);
    TEST_ASSERT_ALL_FINITE(att.gyro_filtered, 3);
}

static void test_ignition_gate(void)
{
    complete_static_init(&att);

    /* Add a gate from t=1s for 2s duration */
    casper_att_add_gate(&att, 1.0f, 2.0f);

    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, G_ACCEL, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Launch to enter flight phase */
    float accel_launch[3] = {0.0f, 5.0f * G_ACCEL, 0.0f};
    casper_att_update(&att, gyro, accel_launch, NULL, dt);

    /* The gate mechanism is internal — just verify it doesn't crash */
    float mag[3] = {20.0f, 0.0f, -40.0f};
    for (int i = 0; i < 5000; i++)
        casper_att_update(&att, gyro, accel, mag, dt);

    TEST_ASSERT_ALL_FINITE(att.q, 4);
}

/* ================================================================
 *  MAIN
 * ================================================================ */

int main(void)
{
    UNITY_BEGIN();

    /* Static init */
    RUN_TEST(test_static_init_level_accel);
    RUN_TEST(test_static_init_no_mag_fallback);
    RUN_TEST(test_static_init_with_mag_completes_early);
    RUN_TEST(test_static_init_yaw_zero_without_mag);

    /* Pad phase */
    RUN_TEST(test_pad_gyro_bias_convergence);
    RUN_TEST(test_pad_gravity_correction);

    /* Flight phase */
    RUN_TEST(test_launch_detection_at_3g);
    RUN_TEST(test_gravity_correction_disabled_in_flight);
    RUN_TEST(test_rk4_constant_rotation);
    RUN_TEST(test_bias_frozen_at_launch);

    /* Robustness */
    RUN_TEST(test_quaternion_unit_norm_maintained);
    RUN_TEST(test_no_nan_after_many_updates);
    RUN_TEST(test_ignition_gate);

    return UNITY_END();
}
