/**
 * @file casper_gyro_int.c
 * @brief RK4 gyro quaternion integrator for C.A.S.P.E.R.-2 flight computer.
 */

#include "casper_gyro_int.h"
#include "casper_quat.h"
#include <math.h>
#include <string.h>

#define G_ACCEL   9.80665f
#define LAUNCH_G  3.0f          /* launch threshold in g */
#define PI_F      3.14159265f

/* qdot = 0.5 * q (x) [0; omega] */
static void quat_derivative(const float q[4], const float omega[3], float qdot[4])
{
    float omega_q[4] = {0.0f, omega[0], omega[1], omega[2]};
    float tmp[4];
    casper_quat_mult(q, omega_q, tmp);
    qdot[0] = 0.5f * tmp[0];
    qdot[1] = 0.5f * tmp[1];
    qdot[2] = 0.5f * tmp[2];
    qdot[3] = 0.5f * tmp[3];
}

void casper_gyro_int_init(casper_gyro_int_t *gi, const float accel_initial[3])
{
    memset(gi, 0, sizeof(*gi));

    casper_quat_from_accel(accel_initial, gi->q);

    gi->lpf_cutoff_hz = 50.0f;

    /* LSM6DSO32 characterization values */
    gi->gyro_arw[0] = 6.73e-05f;
    gi->gyro_arw[1] = 6.08e-05f;
    gi->gyro_arw[2] = 4.92e-05f;
}

void casper_gyro_int_update(casper_gyro_int_t *gi, const float gyro_raw[3],
                            const float accel_raw[3], float dt)
{
    int i;

    /* ── 1. First-order IIR low-pass filter on gyro ── */
    float alpha = dt / (dt + 1.0f / (2.0f * PI_F * gi->lpf_cutoff_hz));
    for (i = 0; i < 3; i++) {
        gi->gyro_filtered[i] = alpha * gyro_raw[i]
                             + (1.0f - alpha) * gi->gyro_filtered[i];
    }

    /* ── 2. Launch detection (|accel| > 3g) ── */
    if (!gi->launched) {
        float amag_sq = accel_raw[0]*accel_raw[0]
                      + accel_raw[1]*accel_raw[1]
                      + accel_raw[2]*accel_raw[2];
        float thresh = LAUNCH_G * G_ACCEL;
        if (amag_sq > thresh * thresh) {
            gi->launched = true;
            /* Bias is frozen — stop accumulating */
        }
    }

    /* ── 3. On-pad gyro bias estimation ── */
    if (!gi->launched) {
        for (i = 0; i < 3; i++)
            gi->bias_sum[i] += (double)gyro_raw[i];
        gi->bias_count++;
        for (i = 0; i < 3; i++)
            gi->gyro_bias[i] = (float)(gi->bias_sum[i] / (double)gi->bias_count);
    }

    /* ── 4. Subtract bias from filtered gyro ── */
    float omega[3];
    for (i = 0; i < 3; i++)
        omega[i] = gi->gyro_filtered[i] - gi->gyro_bias[i];

    /* ── 5. RK4 quaternion propagation ── */
    float *q  = gi->q;
    float *k1 = gi->k1;
    float *k2 = gi->k2;
    float *k3 = gi->k3;
    float *k4 = gi->k4;
    float *qt = gi->q_tmp;

    /* k1 = qdot(q, omega) */
    quat_derivative(q, omega, k1);

    /* k2 = qdot(q + k1*dt/2, omega) */
    for (i = 0; i < 4; i++)
        qt[i] = q[i] + k1[i] * dt * 0.5f;
    quat_derivative(qt, omega, k2);

    /* k3 = qdot(q + k2*dt/2, omega) */
    for (i = 0; i < 4; i++)
        qt[i] = q[i] + k2[i] * dt * 0.5f;
    quat_derivative(qt, omega, k3);

    /* k4 = qdot(q + k3*dt, omega) */
    for (i = 0; i < 4; i++)
        qt[i] = q[i] + k3[i] * dt;
    quat_derivative(qt, omega, k4);

    /* q_new = q + (dt/6)*(k1 + 2*k2 + 2*k3 + k4) */
    float dt6 = dt / 6.0f;
    for (i = 0; i < 4; i++)
        q[i] += dt6 * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);

    /* ── 6. Normalize quaternion ── */
    casper_quat_normalize(q);

    /* ── 7. Open-loop attitude uncertainty growth ── */
    for (i = 0; i < 3; i++) {
        float s2 = gi->att_sigma[i] * gi->att_sigma[i]
                  + gi->gyro_arw[i] * gi->gyro_arw[i] * dt;
        gi->att_sigma[i] = sqrtf(s2);
    }
}

void casper_gyro_int_set_lpf_cutoff(casper_gyro_int_t *gi, float cutoff_hz)
{
    gi->lpf_cutoff_hz = cutoff_hz;
}
