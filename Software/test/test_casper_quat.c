/* test_casper_quat.c — characterization + property tests for App/nav/casper_quat.c
 * Golden values captured from the ORIGINAL implementation. */
#include "test.h"
#include "casper_quat.h"

#define TOL 1e-5f

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

TEST(quat_euler_roundtrip) {
    float q[4]; casper_quat_from_euler(0.3f,-0.2f,1.1f,q);
    float e[3]; casper_quat_to_euler(q,e);
    /* e = [bodyZ(yaw), bodyY(pitch), bodyX(roll)] in degrees */
    ASSERT_NEAR(e[0], 63.0253563f, 1e-3f);
    ASSERT_NEAR(e[1], -11.4591560f, 1e-3f);
    ASSERT_NEAR(e[2], 17.1887379f, 1e-3f);
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

int main(void){
    RUN(quat_mult_golden);
    RUN(quat_mult_identity);
    RUN(quat_normalize_golden);
    RUN(quat_from_euler_golden);
    RUN(quat_euler_roundtrip);
    RUN(quat_rotmat_golden_and_orthonormal);
    return test_summary();
}
