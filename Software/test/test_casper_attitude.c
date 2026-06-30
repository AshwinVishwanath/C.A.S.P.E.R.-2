/* test_casper_attitude.c — golden regression lock for App/nav/casper_attitude.c
 *
 * Replays pad static-init -> launch -> 2000 in-flight RK4 steps (with 10 Hz mag
 * correction, exercising the flight-phase ignition-gate code path) and asserts
 * the exact quaternion sequence produced by the ORIGINAL code. Used to prove
 * the gate-removal / dead-getter refactor is behaviour-preserving. */
#include "test.h"
#include "casper_attitude.h"

#define TOL 1e-5f

static void run(casper_attitude_t *att,
                float chk_init[4], float chk100[4], float chk500[4],
                float chk1000[4], float chkfinal[4], float *mission_out)
{
    casper_att_config_t cfg = {
        .Kp_grav = 1.0f, .Kp_mag_pad = 0.5f, .Kp_mag_flight = 0.2f,
        .Ki = 0.0f, .gyro_lpf_cutoff_hz = 50.0f, .mag_update_hz = 10.0f,
    };
    casper_att_init(att, &cfg);

    const float dt = 1.0f/833.0f;
    float accel_pad[3] = {0.15f, 9.70f, 0.60f};
    float mag[3]       = {22.0f, 5.0f, 41.0f};

    int iters = 0;
    while (!casper_att_static_init(att, accel_pad, mag) && iters < 2000) iters++;
    for (int k=0;k<4;k++) chk_init[k]=att->q[k];

    float gyro[3] = {0.05f, 0.12f, -0.03f};
    att->launched = true;   /* FSM sets this in flight */

    for (int i = 0; i < 2000; i++) {
        const float *m = (i % 83 == 0) ? mag : NULL;
        float ax = 40.0f + 0.002f*i;
        float accel[3] = {0.2f, ax, -0.1f};
        casper_att_update(att, gyro, accel, m, dt);
        if (i == 100)  for (int k=0;k<4;k++) chk100[k]=att->q[k];
        if (i == 500)  for (int k=0;k<4;k++) chk500[k]=att->q[k];
        if (i == 1000) for (int k=0;k<4;k++) chk1000[k]=att->q[k];
    }
    for (int k=0;k<4;k++) chkfinal[k]=att->q[k];
    *mission_out = att->mission_time;
}

TEST(attitude_golden_sequence) {
    casper_attitude_t att;
    float qi[4], q100[4], q500[4], q1000[4], qf[4], mt;
    run(&att, qi, q100, q500, q1000, qf, &mt);

    const float g_init [4] = {0.6228383f, 0.5909323f, 0.3463147f, 0.3780710f};
    const float g_100  [4] = {0.6192917f, 0.5894611f, 0.3528772f, 0.3801198f};
    const float g_500  [4] = {0.6044766f, 0.5831039f, 0.3793326f, 0.3882070f};
    const float g_1000 [4] = {0.5851001f, 0.5743247f, 0.4118299f, 0.3977503f};
    const float g_final[4] = {0.5436568f, 0.5540742f, 0.4746684f, 0.4148844f};

    for (int k=0;k<4;k++) {
        ASSERT_NEAR(qi[k],   g_init[k],  TOL);
        ASSERT_NEAR(q100[k], g_100[k],   TOL);
        ASSERT_NEAR(q500[k], g_500[k],   TOL);
        ASSERT_NEAR(q1000[k],g_1000[k],  TOL);
        ASSERT_NEAR(qf[k],   g_final[k], TOL);
    }
    ASSERT_NEAR(mt, 2.400915f, 1e-4f);

    /* quaternion stays unit-norm throughout */
    float n = qf[0]*qf[0]+qf[1]*qf[1]+qf[2]*qf[2]+qf[3]*qf[3];
    ASSERT_NEAR(n, 1.0f, 1e-4f);
}

int main(void){
    RUN(attitude_golden_sequence);
    return test_summary();
}
