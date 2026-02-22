/**
 * @file casper_ekf.c
 * @brief 4-state vertical-channel Extended Kalman Filter for C.A.S.P.E.R.-2.
 *
 * Clean-sheet rewrite. Architecture: Teensy EKF + Mach gate + bias states.
 *
 * What this file does:
 *   - 4-state predict (alt, vel, accel_bias, baro_bias)
 *   - Baro altitude update (Joseph form)
 *   - ZUPT on pad / post-landing
 *   - Mach gate with hysteresis
 *   - Bias reset + P inflation at un-gate
 *   - Hard innovation reject (5-sigma)
 *
 * What this file does NOT do:
 *   - Baro-derived velocity (correlated with baro alt — let P cross-covariance handle it)
 *   - Adaptive gate (if baro is that wrong, reject it)
 *   - Multiple covariance floors (one on baro bias is enough)
 *   - Exponential R decay (step inflated R for N updates, then nominal)
 */

#include "casper_ekf.h"
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════
 *  TUNING KNOBS
 * ═══════════════════════════════════════════════════════════════════ */

#define G_ACCEL             9.80665f

/* Predict rate (416 Hz = every 2nd IMU sample) */
#define EKF_DT              0.0024f

/* Initial covariance P0 diagonal */
#define P0_ALT              0.1f        /* m²          */
#define P0_VEL              0.001f      /* (m/s)²      */
#define P0_ACCEL_BIAS       0.025f      /* (m/s²)²     */
#define P0_BARO_BIAS        0.75f       /* m²          */

/* Process noise source parameters (MATLAB Allan variance) */
#define ACCEL_VRW           2.162545e-03f   /* m/s/√s      */
#define ACCEL_BI_SIGMA      1.953783e-04f   /* m/s²/√s — Allan variance measured */
#define BARO_BI_SIGMA       1.0e-03f        /* m/√s        */

/* Baro measurement noise */
#define R_BARO              0.5f       /* m²  (sigma ≈ 0.7 m) */

/* ZUPT measurement noise */
#define R_ZUPT              6.15e-06f   /* (m/s)²      */

/* Innovation gate (5-sigma hard reject) */
#define BARO_GATE_K2        25.0f       /* 5² = 25     */

/* Baro bias covariance floor */
#define P_FLOOR_BARO_BIAS   0.01f       /* m²          */

/* Mach gate thresholds */
#define MACH_GATE_ON        0.40f
#define MACH_GATE_OFF       0.35f

/* Un-gate recovery: inflated R for N_UNGATE_STEPS, then nominal */
#define R_BARO_UNGATE       50.0f       /* m² — first N updates post-gate */
#define N_UNGATE_STEPS      10

/* Un-gate bias reset: P inflation values */
#define P_UNGATE_ACCEL_BIAS 1.0f        /* (m/s²)²     */
#define P_UNGATE_BARO_BIAS  10.0f       /* m²          */

/* ═══════════════════════════════════════════════════════════════════
 *  HELPERS
 * ═══════════════════════════════════════════════════════════════════ */

/** Enforce P = 0.5*(P + P') for a 4×4 row-major matrix. */
static void symmetrize(float P[16])
{
    for (int r = 0; r < 4; r++) {
        for (int c = r + 1; c < 4; c++) {
            float avg = 0.5f * (P[r * 4 + c] + P[c * 4 + r]);
            P[r * 4 + c] = avg;
            P[c * 4 + r] = avg;
        }
    }
}

/**
 * Joseph-form scalar measurement update.
 *
 * H is a 1×4 row vector passed as float[4].
 * Modifies x and P in place.
 */
/**
 * @param gate_k2  Chi-squared gate threshold. Pass INFINITY to disable gating (e.g. ZUPT).
 */
static void joseph_scalar_update(float x[4], float P[16],
                                 const float H[4], float z,
                                 float R, float gate_k2,
                                 float tmp_a[16], float tmp_b[16])
{
    /* innovation = z - H·x */
    float Hx = 0.0f;
    for (int i = 0; i < 4; i++)
        Hx += H[i] * x[i];
    float innovation = z - Hx;

    /* S = H·P·Hᵀ + R  (scalar) */
    float S = R;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            S += H[i] * P[i * 4 + j] * H[j];

    /* Innovation gate — NaN-safe: NaN comparisons return false, inverted logic rejects NaN */
    if (!(innovation * innovation <= gate_k2 * S))
        return;

    if (!isfinite(S) || S <= 0.0f)
        return;

    /* K = P·Hᵀ / S  (4×1 vector) */
    float K[4];
    float S_inv = 1.0f / S;
    for (int i = 0; i < 4; i++) {
        float PHt_i = 0.0f;
        for (int j = 0; j < 4; j++)
            PHt_i += P[i * 4 + j] * H[j];
        K[i] = PHt_i * S_inv;
    }

    /* State update: x += K · innovation */
    for (int i = 0; i < 4; i++)
        x[i] += K[i] * innovation;

    /* Joseph form: P = (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ
     *
     * Build I_KH in tmp_a, I_KH_T in tmp_b */
    memset(tmp_a, 0, 16 * sizeof(float));
    for (int i = 0; i < 4; i++)
        tmp_a[i * 4 + i] = 1.0f;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            tmp_a[i * 4 + j] -= K[i] * H[j];

    /* Transpose */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            tmp_b[i * 4 + j] = tmp_a[j * 4 + i];

    /* Use CMSIS-DSP for the two 4×4 multiplies */
    arm_matrix_instance_f32 IKH_mat, IKHT_mat, P_mat, res1_mat, res2_mat;
    float res1[16], res2[16];
    arm_mat_init_f32(&IKH_mat,  4, 4, tmp_a);
    arm_mat_init_f32(&IKHT_mat, 4, 4, tmp_b);
    arm_mat_init_f32(&P_mat,    4, 4, P);
    arm_mat_init_f32(&res1_mat, 4, 4, res1);
    arm_mat_init_f32(&res2_mat, 4, 4, res2);

    /* res1 = (I-KH) · P */
    arm_mat_mult_f32(&IKH_mat, &P_mat, &res1_mat);
    /* res2 = res1 · (I-KH)ᵀ */
    arm_mat_mult_f32(&res1_mat, &IKHT_mat, &res2_mat);

    /* Add K·R·Kᵀ (rank-1) */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            res2[i * 4 + j] += K[i] * R * K[j];

    memcpy(P, res2, sizeof(res2));
    symmetrize(P);
}

/* ═══════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

void casper_ekf_init(casper_ekf_t *ekf)
{
    memset(ekf, 0, sizeof(*ekf));

    float dt = EKF_DT;
    ekf->dt = dt;

    /* ── P0 ── */
    ekf->P[0 * 4 + 0] = P0_ALT;
    ekf->P[1 * 4 + 1] = P0_VEL;
    ekf->P[2 * 4 + 2] = P0_ACCEL_BIAS;
    ekf->P[3 * 4 + 3] = P0_BARO_BIAS;

    /* ── Phi ──
     *  [1   dt   -0.5·dt²   0]
     *  [0   1    -dt         0]
     *  [0   0     1          0]
     *  [0   0     0          1]
     */
    float dt2h = 0.5f * dt * dt;
    ekf->Phi[0 * 4 + 0] = 1.0f;
    ekf->Phi[0 * 4 + 1] = dt;
    ekf->Phi[0 * 4 + 2] = -dt2h;
    ekf->Phi[1 * 4 + 1] = 1.0f;
    ekf->Phi[1 * 4 + 2] = -dt;
    ekf->Phi[2 * 4 + 2] = 1.0f;
    ekf->Phi[3 * 4 + 3] = 1.0f;

    /* PhiT */
    ekf->PhiT[0 * 4 + 0] = 1.0f;
    ekf->PhiT[1 * 4 + 0] = dt;
    ekf->PhiT[1 * 4 + 1] = 1.0f;
    ekf->PhiT[2 * 4 + 0] = -dt2h;
    ekf->PhiT[2 * 4 + 1] = -dt;
    ekf->PhiT[2 * 4 + 2] = 1.0f;
    ekf->PhiT[3 * 4 + 3] = 1.0f;

    /* ── Q ── */
    float qa  = ACCEL_VRW * ACCEL_VRW;
    float qab = ACCEL_BI_SIGMA * ACCEL_BI_SIGMA;
    float qbb = BARO_BI_SIGMA * BARO_BI_SIGMA;

    ekf->Q[0 * 4 + 0] = qa * dt * dt * dt / 3.0f;
    ekf->Q[0 * 4 + 1] = qa * dt * dt / 2.0f;
    ekf->Q[1 * 4 + 0] = qa * dt * dt / 2.0f;
    ekf->Q[1 * 4 + 1] = qa * dt;
    ekf->Q[2 * 4 + 2] = qab * dt;
    ekf->Q[3 * 4 + 3] = qbb * dt;

    /* ── CMSIS-DSP descriptors ── */
    arm_mat_init_f32(&ekf->P_mat,     4, 4, ekf->P);
    arm_mat_init_f32(&ekf->Phi_mat,   4, 4, ekf->Phi);
    arm_mat_init_f32(&ekf->PhiT_mat,  4, 4, ekf->PhiT);
    arm_mat_init_f32(&ekf->Q_mat,     4, 4, ekf->Q);
    arm_mat_init_f32(&ekf->tmp_a_mat, 4, 4, ekf->tmp_a);
    arm_mat_init_f32(&ekf->tmp_b_mat, 4, 4, ekf->tmp_b);

    /* ── Measurement noise ── */
    ekf->R_baro = R_BARO;

    /* ── Gate state: not gated at boot ── */
    ekf->baro_gated = false;
    ekf->baro_prev_gated = false;
    ekf->ungate_count = N_UNGATE_STEPS;  /* no recovery needed on first boot */
}

void casper_ekf_predict(casper_ekf_t *ekf,
                        const float ned_accel_ms2[3],
                        float dt)
{
    /*
     * SIGN CONVENTION:
     *
     *   The attitude estimator uses a Z-UP local-level frame (NOT true NED).
     *   g_ned = {0, 0, +1} in casper_attitude.c — Z points UP.
     *   casper_quat_from_accel([0,0,+9.81]) → identity quaternion.
     *
     *   On pad (stationary): ned_accel_ms2[2] ≈ +9.81 (gravity in Z-up)
     *   Accelerating up:     ned_accel_ms2[2] > +9.81
     *
     *   a_up = ned_accel_ms2[2] - G - accel_bias
     *
     *   Stationary:   9.81 - 9.81 - 0 = 0     ✓
     *   Boost (200g): ~1972 - 9.81 - 0 ≈ 1962  ✓ (upward accel, positive)
     *
     *   DO NOT CHANGE THIS without checking casper_attitude.c g_ned definition.
     *   The frame convention propagates from the attitude estimator, not from
     *   any external standard.
     */
    float a_up = ned_accel_ms2[2] - G_ACCEL - ekf->x[2];

    /* State propagation */
    ekf->x[0] += ekf->x[1] * dt + 0.5f * a_up * dt * dt;
    ekf->x[1] += a_up * dt;
    /* x[2], x[3] unchanged (random walks) */

    /* Covariance: P = Φ·P·Φᵀ + Q */
    arm_mat_mult_f32(&ekf->Phi_mat, &ekf->P_mat, &ekf->tmp_a_mat);
    arm_mat_mult_f32(&ekf->tmp_a_mat, &ekf->PhiT_mat, &ekf->tmp_b_mat);
    arm_mat_add_f32(&ekf->tmp_b_mat, &ekf->Q_mat, &ekf->P_mat);
    symmetrize(ekf->P);

    /* Mach gate update */
    float alt_clamp = ekf->x[0] > 0.0f ? ekf->x[0] : 0.0f;
    float T_K = 288.15f - 0.0065f * alt_clamp;
    if (T_K < 216.65f) T_K = 216.65f;
    float a_sound = sqrtf(1.4f * 287.058f * T_K);
    float mach = fabsf(ekf->x[1]) / a_sound;

    if (!ekf->baro_gated && mach > MACH_GATE_ON)
        ekf->baro_gated = true;
    if (ekf->baro_gated && mach < MACH_GATE_OFF)
        ekf->baro_gated = false;
}

void casper_ekf_update_baro(casper_ekf_t *ekf, float baro_alt_m)
{
    /* ── Gated: record state and return ── */
    if (ekf->baro_gated) {
        ekf->baro_prev_gated = true;
        return;
    }

    /* ── NaN/Inf rejection ── */
    if (!isfinite(baro_alt_m))
        return;

    /* ── Gate-open transition: bias reset + P inflation ── */
    if (ekf->baro_prev_gated) {
        ekf->baro_prev_gated = false;
        ekf->ungate_count = 0;

        /* Reset bias states — no reliable estimate survived the gate */
        ekf->x[2] = 0.0f;  /* accel bias */
        ekf->x[3] = 0.0f;  /* baro bias  */

        /* Inflate P diagonals for bias states */
        ekf->P[2 * 4 + 2] = P_UNGATE_ACCEL_BIAS;
        ekf->P[3 * 4 + 3] = P_UNGATE_BARO_BIAS;

        /* Zero cross-covariance for rows/cols 2 and 3 */
        for (int j = 0; j < 4; j++) {
            if (j != 2) { ekf->P[2 * 4 + j] = 0.0f; ekf->P[j * 4 + 2] = 0.0f; }
            if (j != 3) { ekf->P[3 * 4 + j] = 0.0f; ekf->P[j * 4 + 3] = 0.0f; }
        }
    }

    /* ── R selection: inflated for N steps post-gate, then nominal ── */
    float R;
    if (ekf->ungate_count < N_UNGATE_STEPS) {
        R = R_BARO_UNGATE;
        ekf->ungate_count++;
    } else {
        R = ekf->R_baro;
    }

    /* ── Baro update: H = [1, 0, 0, 1], z = baro_alt_m ── */
    const float H[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    joseph_scalar_update(ekf->x, ekf->P, H, baro_alt_m, R, BARO_GATE_K2,
                         ekf->tmp_a, ekf->tmp_b);

    /* ── Baro bias covariance floor ── */
    if (ekf->P[3 * 4 + 3] < P_FLOOR_BARO_BIAS)
        ekf->P[3 * 4 + 3] = P_FLOOR_BARO_BIAS;
}

void casper_ekf_update_zupt(casper_ekf_t *ekf)
{
    /* H = [0, 1, 0, 0], z = 0.0 (velocity should be zero) */
    const float H[4] = {0.0f, 1.0f, 0.0f, 0.0f};
    /* ZUPT must NEVER be gated — pass INFINITY to disable innovation check */
    joseph_scalar_update(ekf->x, ekf->P, H, 0.0f, R_ZUPT, INFINITY,
                         ekf->tmp_a, ekf->tmp_b);
}

void casper_ekf_update_gps_alt(casper_ekf_t *ekf, float gps_alt_m)
{
    (void)ekf;
    (void)gps_alt_m;
    /* TODO: enable when GPS antenna populated */
}

void casper_ekf_update_gps_vel(casper_ekf_t *ekf, float gps_vel_down)
{
    (void)ekf;
    (void)gps_vel_down;
    /* TODO: enable when GPS antenna populated */
}