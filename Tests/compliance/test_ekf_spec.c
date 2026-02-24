/**
 * @file test_ekf_spec.c
 * @brief EKF and attitude constant compliance tests (PRD 7.1).
 *
 * Validates that EKF tuning knobs, initial covariance, Mach gate thresholds,
 * and attitude configuration defaults match the EKF_SPEC v2.1.
 */

#include "test_config.h"
#include "casper_ekf.h"
#include "casper_attitude.h"

/* ── Test fixtures ────────────────────────────────────────────────────── */

static casper_ekf_t ekf;
static casper_attitude_t att;

void setUp(void)
{
    casper_ekf_init(&ekf);
}

void tearDown(void) { }

/* ═══════════════════════════════════════════════════════════════════════
 *  EKF constant validation
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_ekf_dt_416hz(void)
{
    /* EKF predict rate = 416 Hz => dt = 1/416 = 0.002403.. ~ 0.0024 */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0024f, ekf.dt);
}

static void test_ekf_P0_diagonal(void)
{
    /* P0 diagonal: [alt=0.1, vel=0.001, accel_bias=0.025, baro_bias=0.75] */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f,   ekf.P[0 * 4 + 0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, ekf.P[1 * 4 + 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.025f, ekf.P[2 * 4 + 2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.75f,  ekf.P[3 * 4 + 3]);
}

static void test_ekf_P0_off_diagonal_zero(void)
{
    /* Off-diagonal P0 entries must be zero */
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (r != c) {
                TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, ekf.P[r * 4 + c]);
            }
        }
    }
}

static void test_ekf_R_baro(void)
{
    /* R_BARO = 0.5 m^2 */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, ekf.R_baro);
}

static void test_ekf_initial_state_zero(void)
{
    /* State vector should be zero after init */
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, ekf.x[i]);
    }
}

static void test_ekf_not_gated_at_init(void)
{
    /* Baro should not be gated at boot */
    TEST_ASSERT_FALSE(ekf.baro_gated);
    TEST_ASSERT_FALSE(ekf.baro_prev_gated);
}

static void test_ekf_Phi_structure(void)
{
    /* Phi = [1  dt  -0.5*dt^2  0]
     *       [0  1   -dt        0]
     *       [0  0    1         0]
     *       [0  0    0         1] */
    float dt = 0.0024f;
    float dt2h = 0.5f * dt * dt;

    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 1.0f,  ekf.Phi[0 * 4 + 0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, dt,    ekf.Phi[0 * 4 + 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, -dt2h, ekf.Phi[0 * 4 + 2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[0 * 4 + 3]);

    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[1 * 4 + 0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 1.0f,  ekf.Phi[1 * 4 + 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, -dt,   ekf.Phi[1 * 4 + 2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[1 * 4 + 3]);

    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[2 * 4 + 0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[2 * 4 + 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 1.0f,  ekf.Phi[2 * 4 + 2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[2 * 4 + 3]);

    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[3 * 4 + 0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[3 * 4 + 1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.0f,  ekf.Phi[3 * 4 + 2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-8f, 1.0f,  ekf.Phi[3 * 4 + 3]);
}

static void test_ekf_Q_positive_diagonal(void)
{
    /* Process noise Q diagonal must be strictly positive */
    TEST_ASSERT_TRUE(ekf.Q[0 * 4 + 0] > 0.0f);
    TEST_ASSERT_TRUE(ekf.Q[1 * 4 + 1] > 0.0f);
    TEST_ASSERT_TRUE(ekf.Q[2 * 4 + 2] > 0.0f);
    TEST_ASSERT_TRUE(ekf.Q[3 * 4 + 3] > 0.0f);
}

static void test_ekf_Q_symmetric(void)
{
    /* Q must be symmetric */
    TEST_ASSERT_MATRIX_SYMMETRIC(ekf.Q, 4, 1e-12f);
}

static void test_ekf_zupt_threshold_defined(void)
{
    /* EKF_ZUPT_THRESHOLD should be 0.3 m/s^2 */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, (float)EKF_ZUPT_THRESHOLD);
}

static void test_ekf_gps_stubs_compile(void)
{
    /* GPS update stubs must exist and be callable (no-ops) */
    casper_ekf_update_gps_alt(&ekf, 100.0f);
    casper_ekf_update_gps_vel(&ekf, -5.0f);

    /* State should be unchanged (stubs are no-ops) */
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1e-10f, 0.0f, ekf.x[i]);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Mach gate behavioral test
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_ekf_mach_gate_hysteresis(void)
{
    /* Mach gate ON at 0.40, OFF at 0.35.
     * Speed of sound at sea level ~340.3 m/s.
     * Mach 0.40 => ~136 m/s, Mach 0.35 => ~119 m/s.
     *
     * We drive the EKF velocity high enough to trigger gating,
     * then bring it down to verify hysteresis. */

    /* Start not gated */
    TEST_ASSERT_FALSE(ekf.baro_gated);

    /* Set velocity above Mach 0.40 * 340 = 136 m/s (use 150 m/s) */
    ekf.x[1] = 150.0f;
    float ned_accel[3] = {0.0f, 0.0f, 9.80665f}; /* stationary in Z-up */
    casper_ekf_predict(&ekf, ned_accel, 0.0024f);
    TEST_ASSERT_TRUE(ekf.baro_gated);

    /* Reduce velocity to Mach 0.37 (~126 m/s) — still above OFF threshold */
    ekf.x[1] = 126.0f;
    casper_ekf_predict(&ekf, ned_accel, 0.0024f);
    TEST_ASSERT_TRUE(ekf.baro_gated);

    /* Reduce velocity below Mach 0.35 (~119 m/s) — should un-gate */
    ekf.x[1] = 100.0f;
    casper_ekf_predict(&ekf, ned_accel, 0.0024f);
    TEST_ASSERT_FALSE(ekf.baro_gated);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Attitude config defaults
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_attitude_config_defaults(void)
{
    /* Test the documented default config values */
    casper_att_config_t config = {
        .Kp_grav            = 1.0f,
        .Kp_mag_pad         = 0.5f,
        .Kp_mag_flight      = 0.3f,
        .Ki                 = 0.01f,
        .gyro_lpf_cutoff_hz = 50.0f,
        .mag_update_hz      = 10.0f,
        .launch_accel_g     = 3.0f,
    };

    casper_att_init(&att, &config);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f,  att.config.launch_accel_g);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 50.0f, att.config.gyro_lpf_cutoff_hz);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 10.0f, att.config.mag_update_hz);
}

static void test_attitude_identity_quaternion_at_init(void)
{
    casper_att_config_t config = {
        .Kp_grav = 1.0f, .Kp_mag_pad = 0.5f, .Kp_mag_flight = 0.3f,
        .Ki = 0.01f, .gyro_lpf_cutoff_hz = 50.0f,
        .mag_update_hz = 10.0f, .launch_accel_g = 3.0f,
    };
    casper_att_init(&att, &config);

    float q[4];
    casper_att_get_quaternion(&att, q);

    /* Identity quaternion: [1, 0, 0, 0] */
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, q[3]);
}

static void test_attitude_not_launched_at_init(void)
{
    casper_att_config_t config = {
        .Kp_grav = 1.0f, .Kp_mag_pad = 0.5f, .Kp_mag_flight = 0.3f,
        .Ki = 0.01f, .gyro_lpf_cutoff_hz = 50.0f,
        .mag_update_hz = 10.0f, .launch_accel_g = 3.0f,
    };
    casper_att_init(&att, &config);

    TEST_ASSERT_FALSE(att.launched);
    TEST_ASSERT_FALSE(att.init_complete);
}

static void test_attitude_max_gates(void)
{
    /* CASPER_ATT_MAX_GATES = 4 */
    TEST_ASSERT_EQUAL_INT(4, CASPER_ATT_MAX_GATES);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    UNITY_BEGIN();

    /* EKF constants */
    RUN_TEST(test_ekf_dt_416hz);
    RUN_TEST(test_ekf_P0_diagonal);
    RUN_TEST(test_ekf_P0_off_diagonal_zero);
    RUN_TEST(test_ekf_R_baro);
    RUN_TEST(test_ekf_initial_state_zero);
    RUN_TEST(test_ekf_not_gated_at_init);
    RUN_TEST(test_ekf_Phi_structure);
    RUN_TEST(test_ekf_Q_positive_diagonal);
    RUN_TEST(test_ekf_Q_symmetric);
    RUN_TEST(test_ekf_zupt_threshold_defined);
    RUN_TEST(test_ekf_gps_stubs_compile);

    /* Mach gate behavior */
    RUN_TEST(test_ekf_mach_gate_hysteresis);

    /* Attitude config */
    RUN_TEST(test_attitude_config_defaults);
    RUN_TEST(test_attitude_identity_quaternion_at_init);
    RUN_TEST(test_attitude_not_launched_at_init);
    RUN_TEST(test_attitude_max_gates);

    return UNITY_END();
}
