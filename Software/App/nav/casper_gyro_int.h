/**
 * @file casper_gyro_int.h
 * @brief RK4 gyro quaternion integrator for C.A.S.P.E.R.-2 flight computer.
 *
 * Pure gyro dead-reckoning (no accel/mag correction).  Provides the body-to-NED
 * quaternion used by the EKF predict step to rotate accelerometer readings.
 */

#ifndef CASPER_GYRO_INT_H
#define CASPER_GYRO_INT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Quaternion state (body-to-NED, [w, x, y, z]) */
    float q[4];

    /* First-order IIR low-pass filter on gyro input */
    float gyro_filtered[3];     /* rad/s, filtered output */
    float lpf_cutoff_hz;        /* tuneable cutoff (default 50 Hz) */

    /* Gyro bias (estimated during timed calibration, then frozen) */
    float gyro_bias[3];         /* rad/s */
    double bias_sum[3];         /* accumulator (double for precision) */
    uint32_t bias_count;
    uint32_t cal_samples;       /* target sample count for calibration */

    /* Calibration state */
    bool launched;              /* true once calibration complete */

    /* Open-loop attitude uncertainty per-axis */
    float att_sigma[3];         /* rad */
    float gyro_arw[3];          /* rad/sqrt(s), per-axis noise parameter */

    /* RK4 working memory (avoids stack allocation in tight loop) */
    float k1[4], k2[4], k3[4], k4[4], q_tmp[4];
} casper_gyro_int_t;

/**
 * Initialize gyro integrator from accelerometer gravity vector on pad.
 * @param gi             Gyro integrator state (zeroed and initialized)
 * @param accel_initial  Accelerometer reading on pad [ax,ay,az] in m/s^2
 */
void casper_gyro_int_init(casper_gyro_int_t *gi, const float accel_initial[3]);

/**
 * Update gyro integrator with new IMU sample.
 * @param gi         Gyro integrator state
 * @param gyro_raw   Raw gyroscope [gx,gy,gz] in rad/s
 * @param accel_raw  Raw accelerometer [ax,ay,az] in m/s^2 (used ONLY for launch detection)
 * @param dt         Time step in seconds
 */
void casper_gyro_int_update(casper_gyro_int_t *gi, const float gyro_raw[3],
                            const float accel_raw[3], float dt);

/**
 * Change the gyro low-pass filter cutoff frequency.
 * @param gi          Gyro integrator state
 * @param cutoff_hz   New cutoff frequency in Hz (e.g. 50.0)
 */
void casper_gyro_int_set_lpf_cutoff(casper_gyro_int_t *gi, float cutoff_hz);

#ifdef __cplusplus
}
#endif

#endif /* CASPER_GYRO_INT_H */
