/* test_casper_quat.c — characterization + property tests for App/nav/casper_quat.c
 *
 * Test categories:
 *   GOLDEN  — pin unchanged behaviour of mult, normalize, rotmat, from_euler.
 *   SPEC    — verify the ZXY tilt-from-vertical euler decomposition (EDIT B).
 *
 * Body frame: +Y = nose (up on pad), +X = starboard, +Z = toward operator.
 * Nominal attitude: q0 = [sqrt2/2, -sqrt2/2, 0, 0].
 * euler[0]=yaw(bodyZ), euler[1]=roll(bodyY,inner), euler[2]=pitch(bodyX,middle).
 */
#include "test.h"
#include "casper_quat.h"

#define TOL 1e-5f
#define EULER_TOL 0.5f   /* degrees, for ZXY euler tests */

/* sqrt(2)/2 */
#define SQRT2_2 0.70710678f

/* Nominal nose-up attitude */
static const float Q0[4] = {SQRT2_2, -SQRT2_2, 0.0f, 0.0f};

#define DEG2RAD(d) ((d) * 3.14159265358979f / 180.0f)

static void make_Rx(float deg, float q[4])
{
    float h = DEG2RAD(deg) * 0.5f;
    q[0]=cosf(h); q[1]=sinf(h); q[2]=0.0f; q[3]=0.0f;
}
static void make_Ry(float deg, float q[4])
{
    float h = DEG2RAD(deg) * 0.5f;
    q[0]=cosf(h); q[1]=0.0f; q[2]=sinf(h); q[3]=0.0f;
}
static void make_Rz(float deg, float q[4])
{
    float h = DEG2RAD(deg) * 0.5f;
    q[0]=cosf(h); q[1]=0.0f; q[2]=0.0f; q[3]=sinf(h);
}

/* ================================================================== */
/*  GOLDEN: unchanged operations                                       */
/* ================================================================== */

TEST(quat_mult_golden) {
    float a[4]={0.5f,0.5f,-0.5f,0.5f}, b[4]={0.7071068f,0.0f,0.7071068f,0.0f}, r[4];
    casper_quat_mult(a,b,r);
    float g[4]={0.7071068f,0.0f,0.0f,0.7071068f};
    for(int i=0;i<4;i++) ASSERT_NEAR(r[i], g[i], TOL);
}

TEST(quat_mult_identity) {
    float a[4]={0.18257f,0.36515f,0.54772f,0.73030f}, id[4]={1,0,0,0}, r[4];
    casper_quat_mult(a,id,r);
    for(int i=0;i<4;i++) ASSERT_NEAR(r[i], a[i], TOL);
}

TEST(quat_normalize_golden) {
    float q[4]={1.0f,2.0f,3.0f,4.0f};
    casper_quat_normalize(q);
    float g[4]={0.1825742f,0.3651484f,0.5477226f,0.7302967f};
    for(int i=0;i<4;i++) ASSERT_NEAR(q[i], g[i], TOL);
    ASSERT_NEAR(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3], 1.0f, TOL);
}

TEST(quat_from_euler_golden) {
    float q[4];
    casper_quat_from_euler(0.3f,-0.2f,1.1f,q);
    float g[4]={0.8309424f,0.1783589f,-0.0064355f,0.5269548f};
    for(int i=0;i<4;i++) ASSERT_NEAR(q[i], g[i], TOL);
}

TEST(quat_rotmat_golden_and_orthonormal) {
    float q[4]; casper_quat_from_euler(0.3f,-0.2f,1.1f,q);
    float R[9]; casper_quat_to_rotmat(q,R);
    float g[9]={0.4445544f,-0.8780339f,0.1772791f,
                0.8734425f,0.3810134f,-0.3031945f,
                0.1986693f,0.2896295f,0.9362934f};
    for(int i=0;i<9;i++) ASSERT_NEAR(R[i], g[i], TOL);
    /* rows orthonormal */
    for(int i=0;i<3;i++){
        float n=0; for(int k=0;k<3;k++) n+=R[i*3+k]*R[i*3+k];
        ASSERT_NEAR(n,1.0f,1e-4f);
    }
}

/* ================================================================== */
/*  SPEC: ZXY tilt-from-vertical euler decomposition (EDIT B)         */
/*  q = Q0 x Raxis(angle),  casper_quat_to_euler -> expected angles.  */
/* ================================================================== */

TEST(euler_q0_nominal) {
    /* Nominal nose-up: all angles zero */
    float e[3]; casper_quat_to_euler(Q0, e);
    ASSERT_NEAR(e[0], 0.0f, EULER_TOL); /* yaw   */
    ASSERT_NEAR(e[1], 0.0f, EULER_TOL); /* roll  */
    ASSERT_NEAR(e[2], 0.0f, EULER_TOL); /* pitch */
}

TEST(euler_pure_roll_p90) {
    float ry[4]; make_Ry(90.0f, ry);
    float q[4];  casper_quat_mult(Q0, ry, q);
    float e[3];  casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[1],  90.0f, EULER_TOL); /* roll  */
    ASSERT_NEAR(e[2],   0.0f, EULER_TOL); /* pitch */
    ASSERT_NEAR(e[0],   0.0f, EULER_TOL); /* yaw   */
}

TEST(euler_pure_roll_p170) {
    float ry[4]; make_Ry(170.0f, ry);
    float q[4];  casper_quat_mult(Q0, ry, q);
    float e[3];  casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[1], 170.0f, EULER_TOL); /* full-range roll */
    ASSERT_NEAR(e[2],   0.0f, EULER_TOL);
    ASSERT_NEAR(e[0],   0.0f, EULER_TOL);
}

TEST(euler_pure_roll_n90) {
    float ry[4]; make_Ry(-90.0f, ry);
    float q[4];  casper_quat_mult(Q0, ry, q);
    float e[3];  casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[1], -90.0f, EULER_TOL);
    ASSERT_NEAR(e[2],   0.0f, EULER_TOL);
    ASSERT_NEAR(e[0],   0.0f, EULER_TOL);
}

TEST(euler_pure_pitch_p20) {
    float rx[4]; make_Rx(20.0f, rx);
    float q[4];  casper_quat_mult(Q0, rx, q);
    float e[3];  casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[2],  20.0f, EULER_TOL); /* pitch */
    ASSERT_NEAR(e[1],   0.0f, EULER_TOL); /* roll  */
    ASSERT_NEAR(e[0],   0.0f, EULER_TOL); /* yaw   */
}

TEST(euler_pure_yaw_p20) {
    float rz[4]; make_Rz(20.0f, rz);
    float q[4];  casper_quat_mult(Q0, rz, q);
    float e[3];  casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[0],  20.0f, EULER_TOL); /* yaw   */
    ASSERT_NEAR(e[1],   0.0f, EULER_TOL); /* roll  */
    ASSERT_NEAR(e[2],   0.0f, EULER_TOL); /* pitch */
}

/* Combined: pitch invariant under nose spin */
TEST(euler_combined_pitch20_roll60) {
    float rx[4]; make_Rx(20.0f, rx);
    float ry[4]; make_Ry(60.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rx, tmp);
    casper_quat_mult(tmp, ry, q);
    float e[3]; casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[2],  20.0f, EULER_TOL); /* pitch unchanged by 60-deg spin */
    ASSERT_NEAR(e[1],  60.0f, EULER_TOL); /* roll */
    ASSERT_NEAR(e[0],   0.0f, EULER_TOL); /* yaw~0 */
}

TEST(euler_combined_pitch20_roll150) {
    float rx[4]; make_Rx(20.0f, rx);
    float ry[4]; make_Ry(150.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rx, tmp);
    casper_quat_mult(tmp, ry, q);
    float e[3]; casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[2],  20.0f, EULER_TOL);
    ASSERT_NEAR(e[1], 150.0f, EULER_TOL);
    ASSERT_NEAR(e[0],   0.0f, EULER_TOL);
}

TEST(euler_combined_yaw15_roll90) {
    /* yaw-tilt invariant under nose spin */
    float rz[4]; make_Rz(15.0f, rz);
    float ry[4]; make_Ry(90.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rz, tmp);
    casper_quat_mult(tmp, ry, q);
    float e[3]; casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[0],  15.0f, EULER_TOL); /* yaw unchanged by 90-deg spin */
    ASSERT_NEAR(e[1],  90.0f, EULER_TOL); /* roll */
    ASSERT_NEAR(e[2],   0.0f, EULER_TOL); /* pitch~0 */
}

TEST(euler_combined_neg_pitch_neg_roll) {
    float rx[4]; make_Rx(-25.0f, rx);
    float ry[4]; make_Ry(-120.0f, ry);
    float tmp[4], q[4];
    casper_quat_mult(Q0, rx, tmp);
    casper_quat_mult(tmp, ry, q);
    float e[3]; casper_quat_to_euler(q, e);
    ASSERT_NEAR(e[2],  -25.0f, EULER_TOL);
    ASSERT_NEAR(e[1], -120.0f, EULER_TOL);
    ASSERT_NEAR(e[0],    0.0f, EULER_TOL);
}

TEST(euler_near_singularity) {
    /* pitch -> 90 (nose horizontal), must remain finite */
    float rx[4]; make_Rx(89.9f, rx);
    float q[4];  casper_quat_mult(Q0, rx, q);
    float e[3];  casper_quat_to_euler(q, e);
    /* all finite */
    ASSERT_TRUE(isfinite(e[0]));
    ASSERT_TRUE(isfinite(e[1]));
    ASSERT_TRUE(isfinite(e[2]));
    ASSERT_NEAR(e[2], 89.9f, 0.5f);
}

/* ================================================================== */
int main(void){
    /* golden */
    RUN(quat_mult_golden);
    RUN(quat_mult_identity);
    RUN(quat_normalize_golden);
    RUN(quat_from_euler_golden);
    RUN(quat_rotmat_golden_and_orthonormal);

    /* ZXY euler spec — single-axis */
    RUN(euler_q0_nominal);
    RUN(euler_pure_roll_p90);
    RUN(euler_pure_roll_p170);
    RUN(euler_pure_roll_n90);
    RUN(euler_pure_pitch_p20);
    RUN(euler_pure_yaw_p20);

    /* ZXY euler spec — combined (tilt invariant under spin) */
    RUN(euler_combined_pitch20_roll60);
    RUN(euler_combined_pitch20_roll150);
    RUN(euler_combined_yaw15_roll90);
    RUN(euler_combined_neg_pitch_neg_roll);
    RUN(euler_near_singularity);

    return test_summary();
}
