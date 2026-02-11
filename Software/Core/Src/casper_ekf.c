/**
 * @file casper_ekf.c
 * @brief 4-state vertical-channel Extended Kalman Filter for C.A.S.P.E.R.-2.
 */

#include "casper_ekf.h"
#include "casper_quat.h"
#include <math.h>
#include <string.h>

#define G_ACCEL  9.80665f

/* ═══════════════════════════════════════════════════════════════════
 *  EKF TUNING KNOBS — edit these to tune filter behavior
 * ═══════════════════════════════════════════════════════════════════ */

/* Predict rate */
#define EKF_DT              0.0012f     /* s  (833 Hz IMU rate)               */

/* Initial covariance P0 diagonal — how uncertain each state is at boot     */
#define P0_ALT              0.1f       /* m^2       (sigma = 0.1 m)          */
#define P0_VEL              0.001f     /* (m/s)^2   (sigma = 0.01 m/s)      */
#define P0_ACCEL_BIAS       0.025f     /* (m/s^2)^2 (sigma = 0.05 m/s^2)   */
#define P0_BARO_BIAS        0.75f        /* m^2       (sigma = 1.0 m)          */

/* Process noise Q source parameters (from MATLAB Allan variance)            */
#define ACCEL_VRW           2.162545e-03f   /* m/s/sqrt(s)   — velocity random walk  */
#define ACCEL_BI_SIGMA      1.953783e-04f   /* m/s^2/sqrt(s) — accel bias instability */
#define BARO_BI_SIGMA       1.000000e-03f   /* m/sqrt(s)     — baro bias instability  */

/* Measurement noise R — larger = less trust in sensor                       */
#define R_BARO              0.08f       /* m^2  (sigma ~ 0.32 m)              */

/* Transonic baro gating thresholds                                          */
#define MACH_GATE_ON        0.40f       /* gate baro above this Mach          */
#define MACH_GATE_OFF       0.35f       /* un-gate baro below this Mach       */

/* ═══════════════════════════════════════════════════════════════════ */

/* Enforce symmetry: P = 0.5*(P + P') */
static void symmetrize_4x4(float P[16])
{
    int r, c;
    for (r = 0; r < 4; r++) {
        for (c = r + 1; c < 4; c++) {
            float avg = 0.5f * (P[r*4 + c] + P[c*4 + r]);
            P[r*4 + c] = avg;
            P[c*4 + r] = avg;
        }
    }
}

void casper_ekf_init(casper_ekf_t *ekf)
{
    memset(ekf, 0, sizeof(*ekf));

    float dt = EKF_DT;
    ekf->dt = dt;

    /* ── Initial covariance P0 = diag(...) ── */
    ekf->P[0*4+0] = P0_ALT;
    ekf->P[1*4+1] = P0_VEL;
    ekf->P[2*4+2] = P0_ACCEL_BIAS;
    ekf->P[3*4+3] = P0_BARO_BIAS;

    /* ── State transition Phi (constant for fixed dt) ── */
    /*  [1   dt   -0.5*dt^2   0]                          */
    /*  [0   1    -dt         0]                          */
    /*  [0   0     1          0]                          */
    /*  [0   0     0          1]                          */
    float dt2_half = 0.5f * dt * dt;
    ekf->Phi[0*4+0] = 1.0f;
    ekf->Phi[0*4+1] = dt;
    ekf->Phi[0*4+2] = -dt2_half;
    ekf->Phi[1*4+1] = 1.0f;
    ekf->Phi[1*4+2] = -dt;
    ekf->Phi[2*4+2] = 1.0f;
    ekf->Phi[3*4+3] = 1.0f;

    /* Phi transpose */
    ekf->PhiT[0*4+0] = 1.0f;
    ekf->PhiT[1*4+0] = dt;
    ekf->PhiT[1*4+1] = 1.0f;
    ekf->PhiT[2*4+0] = -dt2_half;
    ekf->PhiT[2*4+1] = -dt;
    ekf->PhiT[2*4+2] = 1.0f;
    ekf->PhiT[3*4+3] = 1.0f;

    /* ── Process noise Q (constant for fixed dt) ── */
    float qa  = ACCEL_VRW * ACCEL_VRW;
    float qab = ACCEL_BI_SIGMA * ACCEL_BI_SIGMA;
    float qbb = BARO_BI_SIGMA * BARO_BI_SIGMA;

    ekf->Q[0*4+0] = qa * dt * dt * dt / 3.0f;
    ekf->Q[0*4+1] = qa * dt * dt / 2.0f;
    ekf->Q[1*4+0] = qa * dt * dt / 2.0f;
    ekf->Q[1*4+1] = qa * dt;
    ekf->Q[2*4+2] = qab * dt;
    ekf->Q[3*4+3] = qbb * dt;

    /* ── Initialize CMSIS-DSP matrix instances ── */
    arm_mat_init_f32(&ekf->P_mat,    4, 4, ekf->P);
    arm_mat_init_f32(&ekf->Phi_mat,  4, 4, ekf->Phi);
    arm_mat_init_f32(&ekf->PhiT_mat, 4, 4, ekf->PhiT);
    arm_mat_init_f32(&ekf->Q_mat,    4, 4, ekf->Q);
    arm_mat_init_f32(&ekf->tmp_a_mat, 4, 4, ekf->tmp_a);
    arm_mat_init_f32(&ekf->tmp_b_mat, 4, 4, ekf->tmp_b);

    /* ── Measurement noise ── */
    ekf->R_baro = R_BARO;
    ekf->baro_gated = false;
}

void casper_ekf_predict(casper_ekf_t *ekf, const casper_attitude_t *att,
                        const float accel_body[3], float dt)
{
    int i;

    /* ── 1. Rotate body-frame accel into NED ── */
    float R[9];
    casper_quat_to_rotmat(att->q, R);

    /* f_ned = R * accel_body  (3x3 * 3x1, manual — too small for CMSIS-DSP) */
    float f_ned[3];
    for (i = 0; i < 3; i++) {
        f_ned[i] = R[i*3+0] * accel_body[0]
                 + R[i*3+1] * accel_body[1]
                 + R[i*3+2] * accel_body[2];
    }

    /* f_ned[2] is positive-up per casper_quat_from_accel convention.
     * Subtract gravity and accel bias to get true vertical acceleration. */
    float a_up = f_ned[2] - G_ACCEL - ekf->x[2];

    /* ── 2. Propagate state ── */
    ekf->x[0] += ekf->x[1] * dt + 0.5f * a_up * dt * dt;
    ekf->x[1] += a_up * dt;
    /* x[2] (accel bias) and x[3] (baro bias) are random walks — no update */

    /* ── 3. Covariance propagation: P = Phi * P * Phi' + Q ── */
    arm_mat_mult_f32(&ekf->Phi_mat, &ekf->P_mat, &ekf->tmp_a_mat);
    arm_mat_mult_f32(&ekf->tmp_a_mat, &ekf->PhiT_mat, &ekf->tmp_b_mat);
    arm_mat_add_f32(&ekf->tmp_b_mat, &ekf->Q_mat, &ekf->P_mat);

    /* Enforce symmetry */
    symmetrize_4x4(ekf->P);

    /* ── 4. Transonic baro gating ── */
    float speed = fabsf(ekf->x[1]);
    float alt_m = ekf->x[0] > 0.0f ? ekf->x[0] : 0.0f;
    float T_K = 288.15f - 0.0065f * alt_m;
    float a_sound = sqrtf(1.4f * 287.058f * T_K);
    float mach = speed / a_sound;

    if (!ekf->baro_gated && mach > MACH_GATE_ON)
        ekf->baro_gated = true;
    if (ekf->baro_gated && mach < MACH_GATE_OFF)
        ekf->baro_gated = false;
}

void casper_ekf_update_baro(casper_ekf_t *ekf, float baro_alt_m)
{
    int i, j;

    /* Skip update during transonic flight */
    if (ekf->baro_gated)
        return;

    float *x = ekf->x;
    float *P = ekf->P;

    /* H = [1, 0, 0, 1] */
    /* innovation = z - H*x = z - (x[0] + x[3]) */
    float innovation = baro_alt_m - (x[0] + x[3]);

    /* S = H * P * H' + R  (scalar: P[0][0] + P[0][3] + P[3][0] + P[3][3] + R) */
    float S = P[0*4+0] + P[0*4+3] + P[3*4+0] + P[3*4+3] + ekf->R_baro;

    /* K = P * H' / S   (4x1 vector) */
    /* H' = [1; 0; 0; 1], so P*H' = col0(P) + col3(P) */
    float K[4];
    float S_inv = 1.0f / S;
    for (i = 0; i < 4; i++)
        K[i] = (P[i*4+0] + P[i*4+3]) * S_inv;

    /* State update: x = x + K * innovation */
    for (i = 0; i < 4; i++)
        x[i] += K[i] * innovation;

    /* ── Joseph form covariance update ──
     * P = (I - K*H) * P * (I - K*H)' + K * R * K'
     *
     * Build I_KH = I - K*H  (4x4)
     * H = [1, 0, 0, 1]
     */
    float I_KH[16];
    memset(I_KH, 0, sizeof(I_KH));
    for (i = 0; i < 4; i++)
        I_KH[i*4+i] = 1.0f;                /* I */
    for (i = 0; i < 4; i++) {
        I_KH[i*4+0] -= K[i] * 1.0f;       /* -K * H[0] */
        I_KH[i*4+3] -= K[i] * 1.0f;       /* -K * H[3] */
    }

    /* I_KH_T = (I - K*H)' */
    float I_KH_T[16];
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            I_KH_T[i*4+j] = I_KH[j*4+i];

    /* tmp_a = I_KH * P */
    arm_matrix_instance_f32 I_KH_mat, I_KH_T_mat;
    arm_mat_init_f32(&I_KH_mat, 4, 4, I_KH);
    arm_mat_init_f32(&I_KH_T_mat, 4, 4, I_KH_T);

    arm_mat_mult_f32(&I_KH_mat, &ekf->P_mat, &ekf->tmp_a_mat);

    /* tmp_b = tmp_a * I_KH' = (I-KH)*P*(I-KH)' */
    arm_mat_mult_f32(&ekf->tmp_a_mat, &I_KH_T_mat, &ekf->tmp_b_mat);

    /* Add K * R * K'  (rank-1 outer product) */
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            ekf->tmp_b[i*4+j] += K[i] * ekf->R_baro * K[j];

    /* Copy result back to P */
    memcpy(P, ekf->tmp_b, sizeof(ekf->P));

    /* Enforce symmetry */
    symmetrize_4x4(P);
}

void casper_ekf_update_gps_alt(casper_ekf_t *ekf, float gps_alt_m)
{
    /* H = [1, 0, 0, 0],  R = 100.0 m^2 (sigma = 10.0 m, u-blox MAX-M10M) */
    /* TODO: enable when GPS hardware integrated */
    (void)ekf;
    (void)gps_alt_m;
}

void casper_ekf_update_gps_vel(casper_ekf_t *ekf, float gps_vel_down)
{
    /* H = [0, 1, 0, 0],  R = 1.0 (m/s)^2 (sigma = 1.0 m/s) */
    /* TODO: enable when GPS hardware integrated */
    (void)ekf;
    (void)gps_vel_down;
}
