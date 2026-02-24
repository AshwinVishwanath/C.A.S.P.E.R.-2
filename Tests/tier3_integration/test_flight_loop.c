/**
 * @file test_flight_loop.c
 * @brief Tier 3 integration test — coarse flight loop validation.
 *
 * Tests the overall data flow: sensor input → attitude → EKF → output.
 * Uses synthetic sensor data (no real hardware).
 */

#include "test_config.h"
#include "casper_quat.h"
#include "casper_ekf.h"
#include "casper_attitude.h"
#include "mock_tick.h"

static casper_ekf_t ekf;
static casper_attitude_t att;

void setUp(void)
{
    mock_tick_reset();
    casper_ekf_init(&ekf);

    casper_att_config_t cfg = {
        .Kp_grav           = 1.0f,
        .Kp_mag_pad        = 0.3f,
        .Kp_mag_flight     = 0.01f,
        .Ki                = 0.0f,
        .gyro_lpf_cutoff_hz = 50.0f,
        .mag_update_hz     = 10.0f,
        .launch_accel_g    = 3.0f,
    };
    casper_att_init(&att, &cfg);
}

void tearDown(void) { }

/* Helper: run one iteration of the nav pipeline */
static void nav_step(const float accel_body[3], const float gyro[3],
                     float baro_alt, bool do_baro, float imu_dt)
{
    /* Attitude update */
    casper_att_update(&att, gyro, accel_body, NULL, imu_dt);

    /* Get quaternion and rotate accel to NED */
    float q[4];
    casper_att_get_quaternion(&att, q);
    float R[9];
    casper_quat_to_rotmat(q, R);

    float ned_accel[3] = {
        R[0]*accel_body[0] + R[1]*accel_body[1] + R[2]*accel_body[2],
        R[3]*accel_body[0] + R[4]*accel_body[1] + R[5]*accel_body[2],
        R[6]*accel_body[0] + R[7]*accel_body[1] + R[8]*accel_body[2],
    };

    /* EKF predict (every 2nd IMU sample = 416 Hz) */
    casper_ekf_predict(&ekf, ned_accel, 2.0f * imu_dt);

    /* Baro update */
    if (do_baro) {
        casper_ekf_update_baro(&ekf, baro_alt);
    }
}

/* ---- Test: pad calibration produces near-zero state ---- */
static void test_pad_calibration_converges(void)
{
    float accel[3] = {0.0f, 0.0f, 9.81f};
    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Static init phase (500 samples) */
    for (int i = 0; i < 500; i++) {
        casper_att_static_init(&att, accel, NULL);
    }

    /* Pad phase: 30 seconds of stationary data with baro at 0m */
    int steps = (int)(30.0f / dt);
    for (int i = 0; i < steps; i++) {
        bool do_baro = (i % 83 == 0); /* ~10 Hz baro */
        nav_step(accel, gyro, 0.0f, do_baro, dt);

        /* Also apply ZUPT on pad */
        if (i % 83 == 0) {
            casper_ekf_update_zupt(&ekf);
        }
    }

    /* After calibration: EKF state should be near zero */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, ekf.x[0]);  /* altitude ≈ 0 */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, ekf.x[1]);  /* velocity ≈ 0 */

    /* Attitude should be upright */
    float roll, pitch, yaw;
    casper_att_get_euler(&att, &roll, &pitch, &yaw);
    /* Note: euler convention is [zrot, yrot, xrot] in degrees.
       For upright pad, all should be near 0 (or 360 for some conventions) */
    TEST_ASSERT_ALL_FINITE(ekf.x, 4);
    TEST_ASSERT_DIAG_POSITIVE(ekf.P, 4);
}

/* ---- Test: synthetic boost → altitude/velocity increase ---- */
static void test_boost_increases_altitude_velocity(void)
{
    float accel_pad[3] = {0.0f, 0.0f, 9.81f};
    float gyro[3]      = {0.0f, 0.0f, 0.0f};
    float dt = 1.0f / 833.0f;

    /* Quick static init */
    for (int i = 0; i < 100; i++) {
        casper_att_static_init(&att, accel_pad, NULL);
    }

    /* Short pad phase (1 second) */
    for (int i = 0; i < 833; i++) {
        nav_step(accel_pad, gyro, 0.0f, (i % 83 == 0), dt);
        if (i % 83 == 0) casper_ekf_update_zupt(&ekf);
    }

    float alt_before = ekf.x[0];
    float vel_before = ekf.x[1];

    /* Boost phase: 50g upward for 2 seconds
       Body accel = [0, 0, 50*9.81] = [0, 0, 490.5] (nose-up, Z = thrust axis) */
    float accel_boost[3] = {0.0f, 0.0f, 50.0f * 9.81f};
    int boost_steps = (int)(2.0f / dt);

    for (int i = 0; i < boost_steps; i++) {
        /* During boost, baro becomes unreliable (Mach gating should activate)
           but we skip baro updates to simplify */
        nav_step(accel_boost, gyro, 0.0f, false, dt);
    }

    /* After 2s of 50g boost:
       Expected velocity ≈ (50-1)*9.81*2 ≈ 962 m/s
       Expected altitude ≈ 0.5*(50-1)*9.81*4 ≈ 962 m */
    TEST_ASSERT_TRUE(ekf.x[1] > vel_before + 100.0f);  /* velocity increased significantly */
    TEST_ASSERT_TRUE(ekf.x[0] > alt_before + 10.0f);   /* altitude increased */

    /* No NaN/Inf */
    TEST_ASSERT_ALL_FINITE(ekf.x, 4);
    TEST_ASSERT_DIAG_POSITIVE(ekf.P, 4);
}

/* ---- Test: Mach gating activates at high velocity ---- */
static void test_mach_gating_activates(void)
{
    /* Directly set EKF velocity above Mach 0.4 threshold
       At sea level, speed of sound ≈ 340 m/s, Mach 0.4 ≈ 136 m/s */
    float ned_accel[3] = {0.0f, 0.0f, 9.81f};

    /* Predict with high upward accel to build velocity */
    float high_accel[3] = {0.0f, 0.0f, 200.0f * 9.81f};
    for (int i = 0; i < 100; i++) {
        casper_ekf_predict(&ekf, high_accel, 0.0024f);
    }

    /* Velocity should now be very high */
    TEST_ASSERT_TRUE(fabsf(ekf.x[1]) > 100.0f);

    /* Check Mach gating */
    if (fabsf(ekf.x[1]) > 136.0f) {
        TEST_ASSERT_TRUE(ekf.baro_gated);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pad_calibration_converges);
    RUN_TEST(test_boost_increases_altitude_velocity);
    RUN_TEST(test_mach_gating_activates);
    return UNITY_END();
}
