/**
 * @file casper_ekf.h
 * @brief 4-state vertical EKF for C.A.S.P.E.R.-2.
 *
 * States: [altitude (m, +up, AGL), velocity (m/s, +up),
 *          accel_bias (m/s²), baro_bias (m)]
 *
 * Clean-sheet rewrite. Teensy architecture + Mach gate + bias states.
 */

#ifndef CASPER_EKF_H
#define CASPER_EKF_H

#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"

typedef struct {
    /* ── State vector x[4] ── */
    float x[4];

    /* ── Covariance P (4×4 row-major) ── */
    float P[16];

    /* ── Pre-computed constant matrices ── */
    float Phi[16];          /* State transition            */
    float PhiT[16];         /* Phi transpose               */
    float Q[16];            /* Process noise               */

    /* ── CMSIS-DSP matrix descriptors ── */
    arm_matrix_instance_f32 P_mat;
    arm_matrix_instance_f32 Phi_mat;
    arm_matrix_instance_f32 PhiT_mat;
    arm_matrix_instance_f32 Q_mat;
    arm_matrix_instance_f32 tmp_a_mat;
    arm_matrix_instance_f32 tmp_b_mat;

    /* ── Working memory (avoids stack allocation) ── */
    float tmp_a[16];
    float tmp_b[16];

    /* ── Mach gate state ── */
    bool  baro_gated;       /* true = baro updates suppressed   */
    bool  baro_prev_gated;  /* previous cycle gate state        */
    uint8_t ungate_count;   /* counts 0..N post-gate, then stops */

    /* ── Timing ── */
    float dt;               /* predict timestep (s) */
    float R_baro;           /* baro measurement noise (m²) */

} casper_ekf_t;

/**
 * ZUPT stationarity threshold — caller checks accel magnitude before
 * calling casper_ekf_update_zupt(). Relaxed for ±32g noise floor.
 */
#define EKF_ZUPT_THRESHOLD  0.3f    /* m/s² */

/**
 * Initialise EKF: zero state, set P0, pre-compute Phi/Q.
 */
void casper_ekf_init(casper_ekf_t *ekf);

/**
 * Predict step. Call at EKF predict rate (416 Hz).
 *
 * @param ned_accel_ms2  Accelerometer rotated to local-level frame (Z-UP).
 *                       Despite the name, this is NOT true NED (which is Z-down).
 *                       The attitude estimator uses g_ned={0,0,+1}, so Z points UP.
 *                       Only [2] (vertical axis, positive up) is used.
 * @param dt             Time step (s). Use 2×dt_imu for 416 Hz decimation.
 */
void casper_ekf_predict(casper_ekf_t *ekf,
                        const float ned_accel_ms2[3],
                        float dt);

/**
 * Barometer altitude update. Call when ms5611_tick() delivers new data.
 * Automatically suppressed during Mach gate.
 *
 * @param baro_alt_m     Barometric altitude AGL (m), already referenced.
 */
void casper_ekf_update_baro(casper_ekf_t *ekf, float baro_alt_m);

/**
 * ZUPT: zero-velocity pseudo-measurement.
 * Call at predict rate while FSM is in PAD or LANDED.
 */
void casper_ekf_update_zupt(casper_ekf_t *ekf);

/**
 * GPS altitude update (stub — enable when antenna populated).
 */
void casper_ekf_update_gps_alt(casper_ekf_t *ekf, float gps_alt_m);

/**
 * GPS vertical velocity update (stub).
 */
void casper_ekf_update_gps_vel(casper_ekf_t *ekf, float gps_vel_down);

#endif /* CASPER_EKF_H */