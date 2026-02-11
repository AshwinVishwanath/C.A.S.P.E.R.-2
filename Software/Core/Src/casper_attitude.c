/**
 * @file casper_attitude.c
 * @brief Magnetometer-assisted attitude estimator for C.A.S.P.E.R.-2.
 *
 * Pad:    Mahony complementary filter (accel + gyro + mag).
 * Flight: RK4 gyro propagation + 10 Hz tilt-compensated mag roll correction.
 */

#include "casper_attitude.h"
#include "casper_quat.h"
#include <math.h>
#include <string.h>

/* ── Constants ─────────────────────────────────────────────────────────── */

#define G_ACCEL     9.80665f
#define PI_F        3.14159265f
#define STATIC_INIT_MAG_SAMPLES   500
#define STATIC_INIT_TIMEOUT_S     10.0f
#define HEADING_SIGMA_FLOOR       0.01f   /* ~0.6° floor with mag correction */

/* ── Helpers ───────────────────────────────────────────────────────────── */

/* qdot = 0.5 * q (x) [0; omega] */
static void quat_derivative(const float q[4], const float omega[3],
                            float qdot[4])
{
    float omega_q[4] = {0.0f, omega[0], omega[1], omega[2]};
    float tmp[4];
    casper_quat_mult(q, omega_q, tmp);
    qdot[0] = 0.5f * tmp[0];
    qdot[1] = 0.5f * tmp[1];
    qdot[2] = 0.5f * tmp[2];
    qdot[3] = 0.5f * tmp[3];
}

static float vec3_norm(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void vec3_normalize(const float in[3], float out[3])
{
    float n = vec3_norm(in);
    if (n > 1.0e-10f) {
        float inv = 1.0f / n;
        out[0] = in[0] * inv;
        out[1] = in[1] * inv;
        out[2] = in[2] * inv;
    } else {
        out[0] = out[1] = out[2] = 0.0f;
    }
}

static void vec3_cross(const float a[3], const float b[3], float r[3])
{
    r[0] = a[1]*b[2] - a[2]*b[1];
    r[1] = a[2]*b[0] - a[0]*b[2];
    r[2] = a[0]*b[1] - a[1]*b[0];
}

/* Multiply R-transpose by a vector: out = R' * v  (NED-to-body)
 * R is row-major body-to-NED from casper_quat_to_rotmat. */
static void rotmat_transpose_mul(const float R[9], const float v[3],
                                 float out[3])
{
    out[0] = R[0]*v[0] + R[3]*v[1] + R[6]*v[2];
    out[1] = R[1]*v[0] + R[4]*v[1] + R[7]*v[2];
    out[2] = R[2]*v[0] + R[5]*v[1] + R[8]*v[2];
}

/* Multiply R by a vector: out = R * v  (body-to-NED) */
static void rotmat_mul(const float R[9], const float v[3], float out[3])
{
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

/* Check if mission_time falls within any ignition gate */
static bool mag_gated(const casper_attitude_t *att)
{
    for (uint8_t i = 0; i < att->num_gates; i++) {
        float t0 = att->gates[i].start_time_s;
        float t1 = t0 + att->gates[i].duration_s;
        if (att->mission_time >= t0 && att->mission_time < t1)
            return true;
    }
    return false;
}

/* ── Public API ────────────────────────────────────────────────────────── */

void casper_att_init(casper_attitude_t *att, const casper_att_config_t *config)
{
    memset(att, 0, sizeof(*att));
    att->config = *config;

    /* Identity quaternion */
    att->q[0] = 1.0f;

    /* LSM6DSO32 characterization values (rad/sqrt(s)) */
    att->gyro_arw[0] = 6.73e-05f;
    att->gyro_arw[1] = 6.08e-05f;
    att->gyro_arw[2] = 4.92e-05f;

    /* Assume mag is available until proven otherwise */
    att->mag_available = true;
}

bool casper_att_static_init(casper_attitude_t *att,
                            const float accel[3], const float *mag_cal)
{
    if (att->init_complete)
        return true;

    /* Always accumulate accel (833 Hz) */
    for (int i = 0; i < 3; i++)
        att->accel_sum[i] += (double)accel[i];
    att->accel_count++;

    /* Accumulate mag only when a new reading arrives (~93 Hz) */
    if (mag_cal) {
        for (int i = 0; i < 3; i++)
            att->mag_sum[i] += (double)mag_cal[i];
        att->mag_count++;
    }

    /* Track elapsed time (assume 1/833 s per call) */
    att->init_elapsed += (1.0f / 833.0f);

    /* Check completion: 500 mag samples or 10 s timeout */
    bool done_mag = (att->mag_count >= STATIC_INIT_MAG_SAMPLES);
    bool timeout  = (att->init_elapsed >= STATIC_INIT_TIMEOUT_S);

    if (!done_mag && !timeout)
        return false;

    /* ── Compute initial attitude ── */

    /* Average accel → gravity direction */
    float accel_avg[3];
    for (int i = 0; i < 3; i++)
        accel_avg[i] = (float)(att->accel_sum[i] / (double)att->accel_count);

    /* Roll and pitch from gravity */
    float ax = accel_avg[0], ay = accel_avg[1], az = accel_avg[2];
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float roll  = atan2f( ay, az);

    float yaw = 0.0f;

    if (done_mag && att->mag_count > 0) {
        /* Average calibrated mag */
        float mag_avg[3];
        for (int i = 0; i < 3; i++)
            mag_avg[i] = (float)(att->mag_sum[i] / (double)att->mag_count);

        /* Tilt-compensated heading */
        float cp = cosf(pitch), sp = sinf(pitch);
        float cr = cosf(roll),  sr = sinf(roll);
        float mx = mag_avg[0], my = mag_avg[1], mz = mag_avg[2];

        float mx_h = mx * cp + my * sr * sp + mz * cr * sp;
        float my_h = my * cr - mz * sr;
        yaw = atan2f(-my_h, mx_h);

        /* Build initial quaternion */
        casper_quat_from_euler(roll, pitch, yaw, att->q);

        /* Compute m_ref_ned = R_b2n * mag_avg_body */
        float R[9];
        casper_quat_to_rotmat(att->q, R);
        rotmat_mul(R, mag_avg, att->m_ref_ned);

        att->mag_available = true;
    } else {
        /* Timeout fallback: gravity-only init, no mag */
        casper_quat_from_euler(roll, pitch, 0.0f, att->q);
        att->mag_available = false;
    }

    att->init_complete = true;
    return true;
}

void casper_att_update(casper_attitude_t *att,
                       const float gyro_raw[3],
                       const float accel_raw[3],
                       const float *mag_cal,
                       float dt)
{
    int i;

    /* ── 1. First-order IIR low-pass filter on gyro ── */
    float alpha = dt / (dt + 1.0f / (2.0f * PI_F * att->config.gyro_lpf_cutoff_hz));
    for (i = 0; i < 3; i++) {
        att->gyro_filtered[i] = alpha * gyro_raw[i]
                              + (1.0f - alpha) * att->gyro_filtered[i];
    }

    /* ── 2. Gyro bias estimation (running average, frozen at launch) ── */
    if (!att->launched) {
        for (i = 0; i < 3; i++)
            att->bias_sum[i] += (double)gyro_raw[i];
        att->bias_count++;
        for (i = 0; i < 3; i++)
            att->gyro_bias[i] = (float)(att->bias_sum[i] / (double)att->bias_count);
    }

    /* ── 3. Subtract bias from filtered gyro ── */
    float omega[3];
    for (i = 0; i < 3; i++)
        omega[i] = att->gyro_filtered[i] - att->gyro_bias[i];

    /* ── 4. Compute correction terms ── */
    float e_grav[3] = {0.0f, 0.0f, 0.0f};
    float e_mag[3]  = {0.0f, 0.0f, 0.0f};

    float R[9];
    casper_quat_to_rotmat(att->q, R);

    if (!att->launched) {
        /* ── PAD PHASE ── */

        /* Launch detection */
        float accel_mag = vec3_norm(accel_raw);
        if (accel_mag > att->config.launch_accel_g * G_ACCEL) {
            att->launched = true;
            /* Reset integral to prevent windup from gain change */
            att->e_int[0] = att->e_int[1] = att->e_int[2] = 0.0f;
            att->mag_update_timer = 0.0f;
        }

        if (!att->launched) {
            /* Gravity correction: e_grav = cross(a_hat, R' * [0,0,1]) */
            float a_hat[3];
            vec3_normalize(accel_raw, a_hat);
            float g_ned[3] = {0.0f, 0.0f, 1.0f};
            float g_pred[3];
            rotmat_transpose_mul(R, g_ned, g_pred);
            vec3_cross(a_hat, g_pred, e_grav);

            /* Mag correction (if available) */
            if (mag_cal && att->mag_available) {
                float m_hat[3];
                vec3_normalize(mag_cal, m_hat);
                float m_pred[3];
                rotmat_transpose_mul(R, att->m_ref_ned, m_pred);
                float m_pred_hat[3];
                vec3_normalize(m_pred, m_pred_hat);
                vec3_cross(m_hat, m_pred_hat, e_mag);
            }

            /* Integral accumulates raw error */
            for (i = 0; i < 3; i++)
                att->e_int[i] += (e_grav[i] + e_mag[i]) * dt;

            /* Apply corrections */
            for (i = 0; i < 3; i++)
                omega[i] += att->config.Kp_grav * e_grav[i]
                          + att->config.Kp_mag_pad * e_mag[i]
                          + att->config.Ki * att->e_int[i];
        }
    } else {
        /* ── FLIGHT PHASE ── */

        /* Mag correction at 10 Hz (timer-based decimation) */
        att->mag_update_timer += dt;

        if (att->mag_update_timer >= 1.0f / att->config.mag_update_hz) {
            /* Timer has fired — attempt correction */
            bool gated = mag_gated(att);

            if (mag_cal && att->mag_available && !gated) {
                float m_hat[3];
                vec3_normalize(mag_cal, m_hat);
                float m_pred[3];
                rotmat_transpose_mul(R, att->m_ref_ned, m_pred);
                float m_pred_hat[3];
                vec3_normalize(m_pred, m_pred_hat);
                vec3_cross(m_hat, m_pred_hat, e_mag);

                /* Integral accumulates raw error */
                float dt_corr = att->mag_update_timer;
                for (i = 0; i < 3; i++)
                    att->e_int[i] += e_mag[i] * dt_corr;

                /* Apply correction */
                for (i = 0; i < 3; i++)
                    omega[i] += att->config.Kp_mag_flight * e_mag[i]
                              + att->config.Ki * att->e_int[i];

                /* Reduce heading uncertainty */
                float alpha_corr = att->config.Kp_mag_flight * dt_corr;
                if (alpha_corr > 1.0f) alpha_corr = 1.0f;
                att->heading_sigma = att->heading_sigma * (1.0f - alpha_corr)
                                   + HEADING_SIGMA_FLOOR * alpha_corr;

                /* Reset timer only on successful correction */
                att->mag_update_timer = 0.0f;
            }
            /* If mag_cal is NULL or gated, do NOT reset timer —
             * correction fires next tick with valid data. */
        }
    }

    /* ── 5. RK4 quaternion propagation ── */
    float *q  = att->q;
    float *k1 = att->k1;
    float *k2 = att->k2;
    float *k3 = att->k3;
    float *k4 = att->k4;
    float *qt = att->q_tmp;

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

    /* ── 7. Uncertainty tracking ── */
    for (i = 0; i < 3; i++) {
        float s2 = att->att_sigma[i] * att->att_sigma[i]
                  + att->gyro_arw[i] * att->gyro_arw[i] * dt;
        att->att_sigma[i] = sqrtf(s2);
    }
    /* Heading uncertainty grows with Z-axis ARW */
    {
        float s2 = att->heading_sigma * att->heading_sigma
                  + att->gyro_arw[2] * att->gyro_arw[2] * dt;
        att->heading_sigma = sqrtf(s2);
    }

    /* ── 8. Advance mission time ── */
    att->mission_time += dt;
}

void casper_att_add_gate(casper_attitude_t *att,
                         float start_s, float duration_s)
{
    if (att->num_gates < CASPER_ATT_MAX_GATES) {
        att->gates[att->num_gates].start_time_s = start_s;
        att->gates[att->num_gates].duration_s   = duration_s;
        att->num_gates++;
    }
}

/* ── Getters ───────────────────────────────────────────────────────────── */

void casper_att_get_quaternion(const casper_attitude_t *att, float q_out[4])
{
    q_out[0] = att->q[0];
    q_out[1] = att->q[1];
    q_out[2] = att->q[2];
    q_out[3] = att->q[3];
}

void casper_att_get_euler(const casper_attitude_t *att,
                          float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float euler[3];
    casper_quat_to_euler(att->q, euler);
    *roll_deg  = euler[0];
    *pitch_deg = euler[1];
    *yaw_deg   = euler[2];
}
