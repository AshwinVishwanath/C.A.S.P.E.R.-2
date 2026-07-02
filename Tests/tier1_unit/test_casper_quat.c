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

void test_quat_from_accel_finite(void)
{
    /* casper_quat_from_accel must produce a finite unit quaternion. */
    float accel[3] = {0.0f, 4.9f, 8.5f};
    float q[4];
    casper_quat_from_accel(accel, q);
    TEST_ASSERT_ALL_FINITE(q, 4);
    float euler[3];
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_ALL_FINITE(euler, 3);
}

/* ── Tilt-from-vertical euler convention helpers ─────────────────────
 * casper_quat_to_euler uses a Y-nose tilt-from-vertical convention:
 *   euler[0]=yaw (bodyZ, heading), euler[1]=roll (bodyY, nose spin,
 *   full ±180), euler[2]=pitch (bodyX, fore/aft tilt).  Nose-up nominal
 *   q0=[sqrt2/2,-sqrt2/2,0,0] -> (0,0,0). Pitch/yaw are invariant under
 *   nose-spin (roll changes only). */
static const float Q0_NOSE_UP[4] = { 0.70710678f, -0.70710678f, 0.0f, 0.0f };

/* q_out = q0 (x) R_axis(theta_deg), axis: 0=X (pitch), 1=Y (roll/spin), 2=Z (yaw) */
static void build_dev_quat(int axis, float theta_deg, float q_out[4])
{
    float h = theta_deg * DEG_TO_RAD * 0.5f;
    float r[4] = { cosf(h), 0.0f, 0.0f, 0.0f };
    r[axis + 1] = sinf(h);
    casper_quat_mult(Q0_NOSE_UP, r, q_out);
}

void test_tilt_nose_up_is_zero(void)
{
    float euler[3];
    casper_quat_to_euler(Q0_NOSE_UP, euler);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, euler[0]); /* yaw   */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, euler[1]); /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, euler[2]); /* pitch */
}

void test_tilt_nose_spin_is_roll(void)
{
    float q[4], euler[3];
    build_dev_quat(1, 90.0f, q);              /* pure spin about nose (Y) */
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, euler[1]);  /* roll = spin */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f,  euler[2]);  /* pitch unchanged */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f,  euler[0]);  /* yaw unchanged */

    build_dev_quat(1, 170.0f, q);             /* full range, no wrap before 180 */
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 170.0f, euler[1]);
}

/* ================================================================== */
/*  Euler — tilt-from-vertical single-axis + combined                  */
/* ================================================================== */

void test_tilt_pitch_is_bodyX(void)
{
    float q[4], euler[3];
    build_dev_quat(0, 20.0f, q);              /* fore/aft tilt about body X */
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, euler[2]);  /* pitch */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f,  euler[1]);  /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f,  euler[0]);  /* yaw   */
}

void test_tilt_yaw_is_bodyZ(void)
{
    float q[4], euler[3];
    build_dev_quat(2, 20.0f, q);              /* side tilt about body Z */
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, euler[0]);  /* yaw   */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f,  euler[1]);  /* roll  */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f,  euler[2]);  /* pitch */
}

void test_tilt_invariant_under_spin(void)
{
    /* fore/aft tilt 20deg, THEN spin 60deg about the nose: pitch must stay
     * at 20 (invariant), roll = spin, yaw ~ 0. This is the key property. */
    float qtilt[4], rspin[4], q[4], euler[3];
    build_dev_quat(0, 20.0f, qtilt);          /* q0 (x) Rx(20) */
    float h = 60.0f * DEG_TO_RAD * 0.5f;
    rspin[0] = cosf(h); rspin[1] = 0.0f; rspin[2] = sinf(h); rspin[3] = 0.0f; /* Ry(60) */
    casper_quat_mult(qtilt, rspin, q);        /* q0 (x) Rx(20) (x) Ry(60) */
    casper_quat_to_euler(q, euler);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, euler[2]);  /* pitch invariant under spin */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 60.0f, euler[1]);  /* roll = spin */
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f,  euler[0]);  /* yaw ~ 0 */
}

void test_tilt_near_singularity_finite(void)
{
    /* pitch ~ 89.9deg (nose horizontal) — must stay finite, no NaN. */
    float q[4], euler[3];
    build_dev_quat(0, 89.9f, q);
    casper_quat_to_euler(q, euler);
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

    /* From accelerometer */
    RUN_TEST(test_quat_from_accel_finite);

    /* Euler — tilt-from-vertical convention */
    RUN_TEST(test_tilt_nose_up_is_zero);
    RUN_TEST(test_tilt_nose_spin_is_roll);
    RUN_TEST(test_tilt_pitch_is_bodyX);
    RUN_TEST(test_tilt_yaw_is_bodyZ);
    RUN_TEST(test_tilt_invariant_under_spin);
    RUN_TEST(test_tilt_near_singularity_finite);

    return UNITY_END();
}
