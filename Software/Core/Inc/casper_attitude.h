/**
 * @file casper_attitude.h
 * @brief Magnetometer-assisted attitude estimator for C.A.S.P.E.R.-2.
 *
 * Mahony complementary filter (accel + gyro + mag) on pad, RK4 gyro
 * propagation with 10 Hz tilt-compensated magnetometer roll correction
 * in flight.  Provides body-to-NED quaternion for the EKF predict step.
 */

#ifndef CASPER_ATTITUDE_H
#define CASPER_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ─────────────────────────────────────────────────────── */

typedef struct {
    float Kp_grav;              /* Gravity correction gain (pad)          */
    float Kp_mag_pad;           /* Mag correction gain (pad)              */
    float Kp_mag_flight;        /* Mag correction gain (flight)           */
    float Ki;                   /* Integral gain                          */
    float gyro_lpf_cutoff_hz;   /* Gyro LPF cutoff (default 50 Hz)       */
    float mag_update_hz;        /* Mag update rate in flight (default 10) */
    float launch_accel_g;       /* Launch detection threshold (default 3) */
} casper_att_config_t;

/* ── Ignition gate ─────────────────────────────────────────────────────── */

#define CASPER_ATT_MAX_GATES 4

typedef struct {
    float start_time_s;         /* Mission elapsed time to begin gating  */
    float duration_s;           /* Gate duration in seconds              */
} casper_mag_gate_t;

/* ── Main state ────────────────────────────────────────────────────────── */

typedef struct {
    /* Quaternion state (body-to-NED, [w, x, y, z]) */
    float q[4];

    /* Gyro low-pass filter state */
    float gyro_filtered[3];     /* rad/s, filtered output               */

    /* Gyro bias (running average on pad, frozen at launch) */
    float gyro_bias[3];         /* rad/s                                */
    double bias_sum[3];         /* accumulator (double for precision)   */
    uint32_t bias_count;

    /* Magnetometer reference field in NED frame (µT) */
    float m_ref_ned[3];

    /* Mahony integral error accumulator */
    float e_int[3];             /* rad/s                                */

    /* Open-loop attitude uncertainty */
    float att_sigma[3];         /* rad, per body axis (gyro ARW growth) */
    float heading_sigma;        /* rad, scalar heading uncertainty      */
    float gyro_arw[3];          /* rad/sqrt(s), per-axis noise param    */

    /* Configuration (copied at init) */
    casper_att_config_t config;

    /* Ignition gates */
    casper_mag_gate_t gates[CASPER_ATT_MAX_GATES];
    uint8_t num_gates;

    /* Phase flags */
    bool init_complete;         /* Static init averaging finished       */
    bool launched;              /* |accel| exceeded launch threshold    */
    bool mag_available;         /* Mag sensor responded during init     */

    /* Timers */
    float mag_update_timer;     /* Accumulates dt for flight decimation */
    float mission_time;         /* Elapsed time since init (s)          */

    /* Static-init accumulators */
    double accel_sum[3];
    uint32_t accel_count;
    double mag_sum[3];
    uint32_t mag_count;
    float init_elapsed;         /* seconds elapsed during static init   */

    /* RK4 working memory (avoids stack allocation in tight loop) */
    float k1[4], k2[4], k3[4], k4[4], q_tmp[4];
} casper_attitude_t;

/* ── Public API ────────────────────────────────────────────────────────── */

/**
 * Initialize attitude estimator.
 * @param att     Attitude state (zeroed and initialized)
 * @param config  Configuration parameters (copied into att)
 */
void casper_att_init(casper_attitude_t *att, const casper_att_config_t *config);

/**
 * Feed stationary data for initial quaternion + mag reference computation.
 * Call at IMU rate (833 Hz).  Pass mag_cal only when a new calibrated mag
 * reading is available (otherwise NULL).  Returns true once init is done
 * (500 mag samples collected, or 10 s timeout with gravity-only fallback).
 *
 * @param att       Attitude state
 * @param accel     Body-frame accel [ax,ay,az] in m/s²
 * @param mag_cal   Calibrated mag [mx,my,mz] in µT, or NULL if no new reading
 * @return true when initialization is complete
 */
bool casper_att_static_init(casper_attitude_t *att,
                            const float accel[3], const float *mag_cal);

/**
 * Main attitude update — call at IMU rate (833 Hz).
 * Runs complementary filter on pad, RK4 + 10 Hz mag correction in flight.
 *
 * @param att       Attitude state
 * @param gyro_raw  Raw gyroscope [gx,gy,gz] in rad/s
 * @param accel_raw Raw accelerometer [ax,ay,az] in m/s²
 * @param mag_cal   Calibrated mag in µT, or NULL if no new reading this cycle
 * @param dt        Time step in seconds
 */
void casper_att_update(casper_attitude_t *att,
                       const float gyro_raw[3],
                       const float accel_raw[3],
                       const float *mag_cal,
                       float dt);

/**
 * Add an ignition gate (max CASPER_ATT_MAX_GATES).
 * During the gate window, magnetometer corrections are disabled.
 */
void casper_att_add_gate(casper_attitude_t *att,
                         float start_s, float duration_s);

/* ── Getters ───────────────────────────────────────────────────────────── */

void casper_att_get_quaternion(const casper_attitude_t *att, float q_out[4]);
void casper_att_get_euler(const casper_attitude_t *att,
                          float *roll_deg, float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* CASPER_ATTITUDE_H */
