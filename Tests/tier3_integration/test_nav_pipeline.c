/**
 * @file test_nav_pipeline.c
 * @brief Tier-3 integration tests for the nav pipeline (PRD S5.2).
 *
 * Tests the attitude + EKF pipeline working together:
 *   1. Static IMU data -> verify no drift
 *   2. Constant rotation -> verify EKF uses rotated accel
 *   3. Sensor-to-body frame remap verification
 */

#include "test_config.h"
#include "casper_ekf.h"
#include "casper_attitude.h"
#include "casper_quat.h"

#define G_ACCEL 9.80665f

static casper_ekf_t ekf;
static casper_attitude_t att;
static casper_att_config_t att_cfg;

static void init_attitude_default(void)
{
    att_cfg.Kp_grav = 1.0f;
    att_cfg.Kp_mag_pad = 0.0f;
    att_cfg.Kp_mag_flight = 0.0f;
    att_cfg.Ki = 0.0f;
    att_cfg.gyro_lpf_cutoff_hz = 50.0f;
    att_cfg.mag_update_hz = 10.0f;
    att_cfg.launch_accel_g = 3.0f;
    casper_att_init(&att, &att_cfg);
}

void setUp(void)
{
    init_attitude_default();
    casper_ekf_init(&ekf);
}

void tearDown(void) { }

/* ================================================================
 *  TEST 1: Static IMU data -- no drift
 * ================================================================ */

static void test_static_imu_no_drift(void)
{
    /*
     * Feed 100 identical IMU samples representing a rocket sitting
     * vertically on the pad.
     *
     * CASPER-2 body frame: +Z = thrust (up on pad).
     * Accel on pad: [0, 0, +9.81] m/s^2 (gravity measured along thrust axis).
     * Gyro: [0, 0, 0] rad/s.
     *
     * After attitude static init converges, the EKF should see
     * ned_accel_z ~ +9.81 (Z-up frame), so a_up ~ 0.
     * EKF state should remain near zero.
     */
    float accel_pad[3] = {0.0f, 0.0f, G_ACCEL};
    float gyro_zero[3] = {0.0f, 0.0f, 0.0f};
    float dt = 0.0012f; /* 833 Hz */

    /* Static init: feed enough samples to complete */
    for (int i = 0; i < 10000; i++)
        casper_att_static_init(&att, accel_pad, NULL);

    TEST_ASSERT_TRUE_MESSAGE(att.init_complete,
        "Attitude static init should complete");

    /* Force launched so attitude runs RK4 path */
    att.launched = true;
    att.e_int[0] = att.e_int[1] = att.e_int[2] = 0.0f;

    /* Run 100 IMU cycles through attitude + EKF */
    int imu_subsample = 0;
    float ned_accum[3] = {0};

    for (int i = 0; i < 100; i++) {
        casper_att_update(&att, gyro_zero, accel_pad, NULL, dt);

        /* Rotate accel to NED */
        float R[9];
        casper_quat_to_rotmat(att.q, R);
        float ned[3];
        ned[0] = R[0]*accel_pad[0] + R[1]*accel_pad[1] + R[2]*accel_pad[2];
        ned[1] = R[3]*accel_pad[0] + R[4]*accel_pad[1] + R[5]*accel_pad[2];
        ned[2] = R[6]*accel_pad[0] + R[7]*accel_pad[1] + R[8]*accel_pad[2];

        /* 416 Hz EKF decimation */
        if (imu_subsample == 0) {
            ned_accum[0] = ned[0];
            ned_accum[1] = ned[1];
            ned_accum[2] = ned[2];
            imu_subsample = 1;
        } else {
            float ned_avg[3];
            ned_avg[0] = 0.5f * (ned_accum[0] + ned[0]);
            ned_avg[1] = 0.5f * (ned_accum[1] + ned[1]);
            ned_avg[2] = 0.5f * (ned_accum[2] + ned[2]);
            casper_ekf_predict(&ekf, ned_avg, 2.0f * dt);
            imu_subsample = 0;
        }
    }

    /* EKF altitude should not have drifted significantly */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ekf.x[0]); /* altitude */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ekf.x[1]); /* velocity */
}

/* ================================================================
 *  TEST 2: Constant rotation -- EKF uses rotated accel
 * ================================================================ */

static void test_constant_rotation_affects_ekf(void)
{
    /*
     * Apply a known constant rotation rate about body X (roll).
     * This tilts the rocket so the thrust-axis accel component (body Z)
     * is no longer fully aligned with NED Z-up. The EKF should see
     * reduced vertical accel, leading to velocity becoming negative
     * (the rocket is "falling" since accel < g).
     *
     * We skip static init and directly set the quaternion to identity
     * (body Z = NED Z-up) then let the gyro rotate it.
     */
    float accel_pad[3] = {0.0f, 0.0f, G_ACCEL};
    float gyro_roll[3] = {0.5f, 0.0f, 0.0f}; /* 0.5 rad/s about body X */
    float dt = 0.0012f;

    /* Force init complete with identity quaternion */
    att.init_complete = true;
    att.q[0] = 1.0f; att.q[1] = 0.0f; att.q[2] = 0.0f; att.q[3] = 0.0f;
    att.launched = true;

    /* Run 200 IMU cycles (~ 0.24s) */
    int imu_subsample = 0;
    float ned_accum[3] = {0};

    for (int i = 0; i < 200; i++) {
        casper_att_update(&att, gyro_roll, accel_pad, NULL, dt);

        float R[9];
        casper_quat_to_rotmat(att.q, R);
        float ned[3];
        ned[0] = R[0]*accel_pad[0] + R[1]*accel_pad[1] + R[2]*accel_pad[2];
        ned[1] = R[3]*accel_pad[0] + R[4]*accel_pad[1] + R[5]*accel_pad[2];
        ned[2] = R[6]*accel_pad[0] + R[7]*accel_pad[1] + R[8]*accel_pad[2];

        if (imu_subsample == 0) {
            ned_accum[0] = ned[0];
            ned_accum[1] = ned[1];
            ned_accum[2] = ned[2];
            imu_subsample = 1;
        } else {
            float ned_avg[3];
            ned_avg[0] = 0.5f * (ned_accum[0] + ned[0]);
            ned_avg[1] = 0.5f * (ned_accum[1] + ned[1]);
            ned_avg[2] = 0.5f * (ned_accum[2] + ned[2]);
            casper_ekf_predict(&ekf, ned_avg, 2.0f * dt);
            imu_subsample = 0;
        }
    }

    /*
     * After ~0.24s of 0.5 rad/s roll, the rocket has tilted ~6.9 degrees.
     * NED-Z component of accel = g*cos(6.9 deg) < g.
     * So EKF sees a_up < 0, velocity should have gone negative.
     */
    TEST_ASSERT_TRUE_MESSAGE(ekf.x[1] < 0.0f,
        "Tilted rocket should show negative EKF velocity (falling)");

    /* Quaternion should still be normalized */
    float qn = sqrtf(att.q[0]*att.q[0] + att.q[1]*att.q[1]
                    + att.q[2]*att.q[2] + att.q[3]*att.q[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, qn);
}

/* ================================================================
 *  TEST 3: Sensor-to-body frame remap
 * ================================================================ */

static void test_body_frame_z_up_on_pad(void)
{
    /*
     * CASPER-2 convention: +Z = thrust axis (up on pad).
     * On pad, accel = [0, 0, +g] in body frame.
     *
     * casper_quat_from_accel([0, 0, +g]) should produce a quaternion
     * where body Z maps to NED Z-up (identity or equivalent rotation).
     *
     * After rotating [0,0,+g] by this quaternion to NED, we should get
     * ned_z ~ +g (Z-up frame convention).
     */
    float accel_pad[3] = {0.0f, 0.0f, G_ACCEL};
    float q[4];
    casper_quat_from_accel(accel_pad, q);

    float R[9];
    casper_quat_to_rotmat(q, R);

    float ned[3];
    ned[0] = R[0]*accel_pad[0] + R[1]*accel_pad[1] + R[2]*accel_pad[2];
    ned[1] = R[3]*accel_pad[0] + R[4]*accel_pad[1] + R[5]*accel_pad[2];
    ned[2] = R[6]*accel_pad[0] + R[7]*accel_pad[1] + R[8]*accel_pad[2];

    /* NED Z should be +g (up) */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, G_ACCEL, ned[2]);
    /* NED X and Y should be near zero */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ned[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ned[1]);
}

static void test_body_frame_remap_consistency(void)
{
    /*
     * Old FC body frame: +Y = thrust, accel on pad ~ [~0, +g, ~0]
     * CASPER-2 body frame: +Z = thrust
     *
     * Remap: casper_x = old_x, casper_y = old_z, casper_z = old_y
     *
     * Given old FC pad accel [0.5, 9.8, -0.1]:
     *   casper accel = [0.5, -0.1, 9.8]
     *
     * After quat_from_accel on the remapped accel, NED Z should ~ +g.
     */
    float old_accel[3] = {0.5f, 9.8f, -0.1f};
    float casper_accel[3] = {old_accel[0], old_accel[2], old_accel[1]};

    float q[4];
    casper_quat_from_accel(casper_accel, q);

    float R[9];
    casper_quat_to_rotmat(q, R);

    float ned[3];
    ned[0] = R[0]*casper_accel[0] + R[1]*casper_accel[1] + R[2]*casper_accel[2];
    ned[1] = R[3]*casper_accel[0] + R[4]*casper_accel[1] + R[5]*casper_accel[2];
    ned[2] = R[6]*casper_accel[0] + R[7]*casper_accel[1] + R[8]*casper_accel[2];

    float accel_mag = sqrtf(casper_accel[0]*casper_accel[0]
                          + casper_accel[1]*casper_accel[1]
                          + casper_accel[2]*casper_accel[2]);

    /* NED Z should be approximately the full magnitude (pointing up) */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, accel_mag, ned[2]);
}

static void test_no_nan_after_pipeline(void)
{
    /*
     * Run attitude + EKF for 500 cycles with realistic data.
     * Verify no NaN in any output.
     */
    float accel[3] = {0.0f, 0.0f, G_ACCEL + 50.0f}; /* boost */
    float gyro[3]  = {0.01f, 0.02f, -0.01f};
    float dt = 0.0012f;

    /* Quick init */
    float init_accel[3] = {0.0f, 0.0f, G_ACCEL};
    for (int i = 0; i < 10000; i++)
        casper_att_static_init(&att, init_accel, NULL);
    att.launched = true;
    att.e_int[0] = att.e_int[1] = att.e_int[2] = 0.0f;

    int imu_sub = 0;
    float ned_accum[3] = {0};

    for (int i = 0; i < 500; i++) {
        casper_att_update(&att, gyro, accel, NULL, dt);

        float R[9];
        casper_quat_to_rotmat(att.q, R);
        float ned[3];
        ned[0] = R[0]*accel[0] + R[1]*accel[1] + R[2]*accel[2];
        ned[1] = R[3]*accel[0] + R[4]*accel[1] + R[5]*accel[2];
        ned[2] = R[6]*accel[0] + R[7]*accel[1] + R[8]*accel[2];

        if (imu_sub == 0) {
            ned_accum[0] = ned[0]; ned_accum[1] = ned[1]; ned_accum[2] = ned[2];
            imu_sub = 1;
        } else {
            float avg[3] = {
                0.5f * (ned_accum[0] + ned[0]),
                0.5f * (ned_accum[1] + ned[1]),
                0.5f * (ned_accum[2] + ned[2])
            };
            casper_ekf_predict(&ekf, avg, 2.0f * dt);
            if (i % 10 == 0)
                casper_ekf_update_baro(&ekf, ekf.x[0]);
            imu_sub = 0;
        }
    }

    TEST_ASSERT_ALL_FINITE(att.q, 4);
    TEST_ASSERT_ALL_FINITE(ekf.x, 4);
    TEST_ASSERT_ALL_FINITE(ekf.P, 16);
    TEST_ASSERT_DIAG_POSITIVE(ekf.P, 4);
}

/* ================================================================
 *  MAIN
 * ================================================================ */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_static_imu_no_drift);
    RUN_TEST(test_constant_rotation_affects_ekf);
    RUN_TEST(test_body_frame_z_up_on_pad);
    RUN_TEST(test_body_frame_remap_consistency);
    RUN_TEST(test_no_nan_after_pipeline);
    return UNITY_END();
}
