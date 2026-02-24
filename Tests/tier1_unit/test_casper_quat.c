/**
 * @file test_casper_quat.c
 * @brief Unit tests for casper_quat quaternion operations (PRD S3.1).
 *
 * Convention: q[4] = {w, x, y, z}, Hamilton product, scalar-first, body-to-NED.
 */

#include "test_config.h"
#include "casper_quat.h"

/* ------------------------------------------------------------------ */
void setUp(void) { }
void tearDown(void) { }

/* ---- Helpers ---- */
static const float Q_ID[4] = {1.0f, 0.0f, 0.0f, 0.0f};

/* 90-degree rotation about Z:  cos(45)=sin(45)=sqrt(2)/2 */
static const float SQRT2_2 = 0.70710678118f;

/* ================================================================== */
/*  Hamilton multiply                                                  */
/* ================================================================== */

void test_quat_identity_multiply(void)
{
    float r[4];
    casper_quat_mult(Q_ID, Q_ID, r);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 1.0f, r[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[3]);
}

void test_quat_z90_multiply(void)
{
    /* q_z90 = [cos(45), 0, 0, sin(45)] = [sqrt2/2, 0, 0, sqrt2/2] */
    float q_z90[4] = {SQRT2_2, 0.0f, 0.0f, SQRT2_2};

    /* q_z90 * q_z90 should give 180-degree Z rotation: [0, 0, 0, 1] */
    float r[4];
    casper_quat_mult(q_z90, q_z90, r);

    float q_z180[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    TEST_ASSERT_QUAT_EQUAL(q_z180, r, 1e-6f);
}

/* Hamilton convention: i*j=k, j*k=i, k*i=j */
void test_quat_hamilton_ij_eq_k(void)
{
    /* Pure quaternion i = [0,1,0,0], j = [0,0,1,0] */
    float qi[4] = {0.0f, 1.0f, 0.0f, 0.0f};
    float qj[4] = {0.0f, 0.0f, 1.0f, 0.0f};
    float r[4];
    casper_quat_mult(qi, qj, r);

    /* Expected: k = [0,0,0,1] */
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 1.0f, r[3]);
}

void test_quat_hamilton_jk_eq_i(void)
{
    float qj[4] = {0.0f, 0.0f, 1.0f, 0.0f};
    float qk[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float r[4];
    casper_quat_mult(qj, qk, r);

    /* Expected: i = [0,1,0,0] */
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 1.0f, r[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[3]);
}

void test_quat_hamilton_ki_eq_j(void)
{
    float qk[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float qi[4] = {0.0f, 1.0f, 0.0f, 0.0f};
    float r[4];
    casper_quat_mult(qk, qi, r);

    /* Expected: j = [0,0,1,0] */
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 1.0f, r[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, r[3]);
}

/* ================================================================== */
/*  Normalize                                                          */
/* ================================================================== */

void test_quat_normalize_unit(void)
{
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    casper_quat_normalize(q);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 1.0f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, q[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, q[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, q[3]);
}

void test_quat_normalize_non_unit(void)
{
    float q[4] = {2.0f, 0.0f, 0.0f, 0.0f};
    casper_quat_normalize(q);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 1.0f, q[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, q[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, q[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.0f, q[3]);
}

void test_quat_normalize_near_zero(void)
{
    /* Near-zero quaternion: should not produce NaN/Inf.
     * The implementation guards norm > 1e-10, so [1e-20,0,0,0]
     * has norm 1e-20 which is below threshold. The quaternion
     * should be left unchanged (guard prevents division). */
    float q[4] = {1e-20f, 0.0f, 0.0f, 0.0f};
    casper_quat_normalize(q);
    TEST_ASSERT_ALL_FINITE(q, 4);
}

/* ================================================================== */
/*  Rotation matrix                                                    */
/* ================================================================== */

void test_quat_rotmat_identity(void)
{
    float R[9];
    casper_quat_to_rotmat(Q_ID, R);

    /* Expected: I3 */
    float I3[9] = {1,0,0, 0,1,0, 0,0,1};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7f, I3, R, 9);
}

void test_quat_rotmat_90_pitch(void)
{
    /* 90-degree rotation about Y axis:
     * q = [cos(45), 0, sin(45), 0] = [sqrt2/2, 0, sqrt2/2, 0]
     * Expected rotation matrix for body-to-NED 90-deg Y:
     *   [0  0  1]
     *   [0  1  0]
     *   [-1 0  0]
     */
    float q_y90[4] = {SQRT2_2, 0.0f, SQRT2_2, 0.0f};
    float R[9];
    casper_quat_to_rotmat(q_y90, R);

    float expected[9] = {
         0.0f,  0.0f,  1.0f,
         0.0f,  1.0f,  0.0f,
        -1.0f,  0.0f,  0.0f
    };
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6f, expected, R, 9);
}

/* ================================================================== */
/*  From accelerometer                                                 */
/* ================================================================== */

void test_quat_from_accel_level(void)
{
    /* accel = [0, 0, 9.81] => body Z pointing up => roll=0, pitch=0
     * (using the Y-nose convention: pitch = atan2(-ax, sqrt(ay^2+az^2)),
     *  roll = atan2(ay, az))
     * pitch = atan2(0, 9.81) = 0
     * roll  = atan2(0, 9.81) = 0
     * => q = from_euler(0, 0, 0) = identity */
    float accel[3] = {0.0f, 0.0f, 9.81f};
    float q[4];
    casper_quat_from_accel(accel, q);

    /* Convert to euler and check roll/pitch near 0 */
    float euler[3];
    casper_quat_to_euler(q, euler);

    /* euler[0]=bodyZ(heading), euler[1]=bodyY(nose spin), euler[2]=bodyX(tilt) */
    /* With accel=[0,0,g], pitch and roll should be ~0 degrees */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, euler[1]); /* body Y rotation */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, euler[2]); /* body X rotation */
}

void test_quat_from_accel_tilted(void)
{
    /* accel = [0, 4.9, 8.5] => tilted about body X
     * pitch = atan2(0, sqrt(4.9^2+8.5^2)) = 0
     * roll  = atan2(4.9, 8.5) ~ 30 deg */
    float accel[3] = {0.0f, 4.9f, 8.5f};
    float q[4];
    casper_quat_from_accel(accel, q);

    /* Should produce a valid, finite quaternion */
    TEST_ASSERT_ALL_FINITE(q, 4);

    float euler[3];
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_ALL_FINITE(euler, 3);

    /* The roll (body X) should be approximately atan2(4.9, 8.5) ~ 30 deg.
     * Use generous tolerance for indirect measurement. */
    float expected_roll_deg = atan2f(4.9f, 8.5f) * RAD_TO_DEG;
    TEST_ASSERT_FLOAT_WITHIN(0.5f, expected_roll_deg, euler[2]);
}

/* ================================================================== */
/*  Euler round-trip                                                   */
/* ================================================================== */

void test_euler_round_trip(void)
{
    /* euler -> quat -> euler should return the original values */
    float roll_deg  = 15.0f;
    float pitch_deg = 25.0f;
    float yaw_deg   = 45.0f;

    float roll_rad  = roll_deg  * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float yaw_rad   = yaw_deg   * DEG_TO_RAD;

    float q[4];
    casper_quat_from_euler(roll_rad, pitch_rad, yaw_rad, q);

    float euler[3];
    casper_quat_to_euler(q, euler);

    /* euler[0] = body Z = yaw, euler[1] = body Y = pitch, euler[2] = body X = roll */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, yaw_deg,   euler[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, pitch_deg,  euler[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, roll_deg,   euler[2]);
}

void test_euler_round_trip_negative(void)
{
    float roll_deg  = -30.0f;
    float pitch_deg = -10.0f;
    float yaw_deg   = -60.0f;

    float q[4];
    casper_quat_from_euler(roll_deg * DEG_TO_RAD,
                           pitch_deg * DEG_TO_RAD,
                           yaw_deg * DEG_TO_RAD, q);

    float euler[3];
    casper_quat_to_euler(q, euler);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, yaw_deg,   euler[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, pitch_deg,  euler[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, roll_deg,   euler[2]);
}

void test_euler_gimbal_lock(void)
{
    /* pitch = 89.9 degrees => near gimbal lock, should not produce NaN */
    float roll_rad  = 0.0f;
    float pitch_rad = 89.9f * DEG_TO_RAD;
    float yaw_rad   = 0.0f;

    float q[4];
    casper_quat_from_euler(roll_rad, pitch_rad, yaw_rad, q);
    TEST_ASSERT_ALL_FINITE(q, 4);

    float euler[3];
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_ALL_FINITE(euler, 3);

    /* pitch (euler[1]) should be close to 89.9 */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 89.9f, euler[1]);
}

/* ================================================================== */
/*  main()                                                             */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Hamilton multiply */
    RUN_TEST(test_quat_identity_multiply);
    RUN_TEST(test_quat_z90_multiply);
    RUN_TEST(test_quat_hamilton_ij_eq_k);
    RUN_TEST(test_quat_hamilton_jk_eq_i);
    RUN_TEST(test_quat_hamilton_ki_eq_j);

    /* Normalize */
    RUN_TEST(test_quat_normalize_unit);
    RUN_TEST(test_quat_normalize_non_unit);
    RUN_TEST(test_quat_normalize_near_zero);

    /* Rotation matrix */
    RUN_TEST(test_quat_rotmat_identity);
    RUN_TEST(test_quat_rotmat_90_pitch);

    /* From accelerometer */
    RUN_TEST(test_quat_from_accel_level);
    RUN_TEST(test_quat_from_accel_tilted);

    /* Euler round-trip */
    RUN_TEST(test_euler_round_trip);
    RUN_TEST(test_euler_round_trip_negative);
    RUN_TEST(test_euler_gimbal_lock);

    return UNITY_END();
}
