/**
 * @file test_casper_quat.c
 * @brief Unit tests for casper_quat quaternion operations (PRD S3.1).
 *
 * Convention: q[4] = {w, x, y, z}, Hamilton product, scalar-first, body-to-NED.
 *
 * Body frame: +Y = nose (up on pad), +X = starboard, +Z = toward operator.
 * Nominal nose-up quaternion q0 = [sqrt2/2, -sqrt2/2, 0, 0].
 *
 * ZXY Euler output layout (euler[3]):
 *   euler[0] = yaw   (body Z, outer),  atan2 full range
 *   euler[1] = roll  (body Y, inner),  atan2 full +/-180  <- nose spin axis
 *   euler[2] = pitch (body X, middle), asin +/-90 singular
 *
 * Key property verified by the combined tests:
 *   A pure spin about the nose (body Y) changes ONLY roll.
 *   Pitch and yaw are invariant under nose-spin.
 */

#include "test_config.h"
#include "casper_quat.h"

/* ------------------------------------------------------------------ */
void setUp(void) { }
void tearDown(void) { }

/* ---- Helpers ---- */
static const float Q_ID[4] = {1.0f, 0.0f, 0.0f, 0.0f};

/* sqrt(2)/2 */
static const float SQRT2_2 = 0.70710678118f;

/* Nominal nose-up attitude: Rx(-90 deg) */
static const float Q0[4] = {0.70710678f, -0.70710678f, 0.0f, 0.0f};

#define DEG2RAD(d) ((d) * 3.14159265358979f / 180.0f)

/* --- Body-axis elementary rotation quaternions (angle in degrees) --- */
/* Rx(t): rotation about body X (fore/aft tilt / pitch) */
static void make_Rx(float deg, float q[4])
{
    float h = DEG2RAD(deg) * 0.5f;
    q[0] = cosf(h); q[1] = sinf(h); q[2] = 0.0f; q[3] = 0.0f;
}

/* Ry(t): rotation about body Y (nose spin / roll) */
static void make_Ry(float deg, float q[4])
{
    float h = DEG2RAD(deg) * 0.5f;
    q[0] = cosf(h); q[1] = 0.0f; q[2] = sinf(h); q[3] = 0.0f;
}

/* Rz(t): rotation about body Z (side tilt / yaw) */
static void make_Rz(float deg, float q[4])
{
    float h = DEG2RAD(deg) * 0.5f;
    q[0] = cosf(h); q[1] = 0.0f; q[2] = 0.0f; q[3] = sinf(h);
}

/* Tolerance for euler tests (degrees) */
#define EULER_TOL 0.5f

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
    float qi[4] = {0.0f, 1.0f, 0.0f, 0.0f};
    float qj[4] = {0.0f, 0.0f, 1.0f, 0.0f};
    float r[4];
    casper_quat_mult(qi, qj, r);

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
    /* Near-zero quaternion: norm guard prevents division, no NaN/Inf. */
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

    float I3[9] = {1,0,0, 0,1,0, 0,0,1};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-7f, I3, R, 9);
}

void test_quat_rotmat_90_pitch(void)
{
    /* 90-degree rotation about Y axis:
     * q = [cos(45), 0, sin(45), 0] = [sqrt2/2, 0, sqrt2/2, 0]
     * Expected rotation matrix:
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
/*  ZXY Euler: tilt-from-vertical (EDIT B spec tests)                 */
/*                                                                     */
/*  Test quaternions are built as  q = Q0 x Raxis(angle).             */
/*  casper_quat_to_euler internally removes Q0 and decomposes with     */
/*  intrinsic Z-X-Y (roll innermost = full range).                    */
/* ================================================================== */

/* --- single-axis: nominal vertical -> all zeros --- */
void test_euler_q0_nominal(void)
{
    float euler[3];
    casper_quat_to_euler(Q0, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL, 0.0f, euler[0]); /* yaw   */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL, 0.0f, euler[1]); /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL, 0.0f, euler[2]); /* pitch */
}

/* --- single-axis: pure roll (body Y spin), full range --- */
void test_euler_pure_roll_p90(void)
{
    float ry[4]; make_Ry(90.0f, ry);
    float q[4];  casper_quat_mult(Q0, ry, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  90.0f, euler[1]); /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[2]); /* pitch */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[0]); /* yaw   */
}

void test_euler_pure_roll_p170(void)
{
    /* 170 deg tests full atan2 range, beyond asin limit */
    float ry[4]; make_Ry(170.0f, ry);
    float q[4];  casper_quat_mult(Q0, ry, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL, 170.0f, euler[1]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[2]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[0]);
}

void test_euler_pure_roll_n90(void)
{
    float ry[4]; make_Ry(-90.0f, ry);
    float q[4];  casper_quat_mult(Q0, ry, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL, -90.0f, euler[1]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[2]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[0]);
}

/* --- single-axis: pure pitch (body X fore/aft tilt) --- */
void test_euler_pure_pitch_p20(void)
{
    float rx[4]; make_Rx(20.0f, rx);
    float q[4];  casper_quat_mult(Q0, rx, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  20.0f, euler[2]); /* pitch */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[1]); /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[0]); /* yaw   */
}

/* --- single-axis: pure yaw (body Z side tilt) --- */
void test_euler_pure_yaw_p20(void)
{
    float rz[4]; make_Rz(20.0f, rz);
    float q[4];  casper_quat_mult(Q0, rz, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  20.0f, euler[0]); /* yaw   */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[1]); /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[2]); /* pitch */
}

/* ================================================================== */
/*  Combined: tilt MUST be invariant under nose-spin                  */
/* ================================================================== */

void test_euler_combined_pitch20_roll60(void)
{
    /* q = Q0 x Rx(20) x Ry(60) -> pitch=20, roll=60, yaw~0 */
    float rx[4]; make_Rx(20.0f, rx);
    float ry[4]; make_Ry(60.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rx, tmp);
    casper_quat_mult(tmp, ry, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  20.0f, euler[2]); /* pitch unchanged by spin */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  60.0f, euler[1]); /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[0]); /* yaw~0 */
}

void test_euler_combined_pitch20_roll150(void)
{
    /* Full-range roll while pitch is invariant */
    float rx[4]; make_Rx(20.0f, rx);
    float ry[4]; make_Ry(150.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rx, tmp);
    casper_quat_mult(tmp, ry, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  20.0f, euler[2]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL, 150.0f, euler[1]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[0]);
}

void test_euler_combined_yaw15_roll90(void)
{
    /* q = Q0 x Rz(15) x Ry(90) -> yaw=15, roll=90, pitch~0 */
    float rz[4]; make_Rz(15.0f, rz);
    float ry[4]; make_Ry(90.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rz, tmp);
    casper_quat_mult(tmp, ry, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  15.0f, euler[0]); /* yaw unchanged by spin */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,  90.0f, euler[1]); /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[2]); /* pitch~0 */
}

void test_euler_combined_neg_pitch_neg_roll(void)
{
    /* q = Q0 x Rx(-25) x Ry(-120) -> pitch=-25, roll=-120, yaw~0 */
    float rx[4]; make_Rx(-25.0f, rx);
    float ry[4]; make_Ry(-120.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rx, tmp);
    casper_quat_mult(tmp, ry, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL, -25.0f, euler[2]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,-120.0f, euler[1]);
    TEST_ASSERT_FLOAT_WITHIN(EULER_TOL,   0.0f, euler[0]);
}

/* --- near-singularity guard: pitch -> +/-90 must not produce NaN --- */
void test_euler_near_singularity(void)
{
    /* q = Q0 x Rx(89.9) -> pitch~89.9, no NaN/Inf */
    float rx[4]; make_Rx(89.9f, rx);
    float q[4];  casper_quat_mult(Q0, rx, q);
    float euler[3]; casper_quat_to_euler(q, euler);
    TEST_ASSERT_ALL_FINITE(euler, 3);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 89.9f, euler[2]);
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

    /* ZXY Euler: single-axis */
    RUN_TEST(test_euler_q0_nominal);
    RUN_TEST(test_euler_pure_roll_p90);
    RUN_TEST(test_euler_pure_roll_p170);
    RUN_TEST(test_euler_pure_roll_n90);
    RUN_TEST(test_euler_pure_pitch_p20);
    RUN_TEST(test_euler_pure_yaw_p20);

    /* ZXY Euler: combined — tilt invariant under spin */
    RUN_TEST(test_euler_combined_pitch20_roll60);
    RUN_TEST(test_euler_combined_pitch20_roll150);
    RUN_TEST(test_euler_combined_yaw15_roll90);
    RUN_TEST(test_euler_combined_neg_pitch_neg_roll);
    RUN_TEST(test_euler_near_singularity);

    return UNITY_END();
}
