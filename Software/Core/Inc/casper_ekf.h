/**
 * @file casper_ekf.h
 * @brief 4-state vertical-channel Extended Kalman Filter for C.A.S.P.E.R.-2.
 *
 * State vector:
 *   x[0] = altitude           (m, positive up, AGL)
 *   x[1] = vertical velocity  (m/s, positive up)
 *   x[2] = accel bias (vert)  (m/s^2)
 *   x[3] = baro bias          (m)
 */

#ifndef CASPER_EKF_H
#define CASPER_EKF_H

#include <stdbool.h>
#include "arm_math.h"
#include "casper_gyro_int.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* State vector [altitude, velocity, accel_bias, baro_bias] */
    float x[4];

    /* Error covariance (4x4, row-major) */
    float P[16];

    /* Pre-computed constant matrices for fixed dt */
    float Phi[16];          /* State transition matrix */
    float PhiT[16];         /* Phi transpose */
    float Q[16];            /* Process noise covariance */

    /* CMSIS-DSP matrix instances (point into arrays above) */
    arm_matrix_instance_f32 P_mat;
    arm_matrix_instance_f32 Phi_mat;
    arm_matrix_instance_f32 PhiT_mat;
    arm_matrix_instance_f32 Q_mat;

    /* Working memory for matrix operations */
    float tmp_a[16];
    float tmp_b[16];
    arm_matrix_instance_f32 tmp_a_mat;
    arm_matrix_instance_f32 tmp_b_mat;

    /* Sensor noise */
    float R_baro;           /* Barometer measurement noise (m^2) */

    /* Transonic baro gating */
    bool baro_gated;

    /* Fixed timestep */
    float dt;
} casper_ekf_t;

/**
 * Initialize EKF state, covariance, and pre-computed matrices.
 * Uses fixed dt = 0.002 s (500 Hz).
 */
void casper_ekf_init(casper_ekf_t *ekf);

/**
 * EKF predict step.  Runs at every IMU sample (500 Hz).
 * Rotates body-frame accel into NED using the gyro integrator's quaternion,
 * then propagates state and covariance.
 *
 * @param ekf         EKF state
 * @param gi          Gyro integrator (provides quaternion for rotation)
 * @param accel_body  Body-frame accelerometer [ax,ay,az] in m/s^2
 * @param dt          Time step in seconds (should match pre-computed Phi, Q)
 */
void casper_ekf_predict(casper_ekf_t *ekf, const casper_gyro_int_t *gi,
                        const float accel_body[3], float dt);

/**
 * EKF barometer measurement update.  Sequential scalar update with Joseph form.
 * Skipped when baro_gated is true (transonic flight).
 *
 * @param ekf         EKF state
 * @param baro_alt_m  Barometer altitude AGL in meters
 */
void casper_ekf_update_baro(casper_ekf_t *ekf, float baro_alt_m);

/**
 * GPS altitude measurement update (STUB — not yet implemented).
 * H = [1, 0, 0, 0],  R = 100.0 m^2
 */
void casper_ekf_update_gps_alt(casper_ekf_t *ekf, float gps_alt_m);

/**
 * GPS vertical velocity measurement update (STUB — not yet implemented).
 * H = [0, 1, 0, 0],  R = 1.0 (m/s)^2
 */
void casper_ekf_update_gps_vel(casper_ekf_t *ekf, float gps_vel_down);

#ifdef __cplusplus
}
#endif

#endif /* CASPER_EKF_H */
