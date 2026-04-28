/* ============================================================
 *  TIER:     SAFETY-CRITICAL
 *  MODULE:   FSM Utilities
 *  SUMMARY:  FSM helpers, transition predicates, virtual-clock for HIL.
 * ============================================================ */
#include "fsm_util.h"
#include "casper_quat.h"
#include <math.h>

/* ── Virtual Clock (HIL_MODE only) ───────────────────────────────── */
#ifdef HIL_MODE
static uint32_t s_virtual_tick_ms = 0;

uint32_t fsm_get_tick(void)
{
    return s_virtual_tick_ms;
}

void fsm_set_tick(uint32_t ms)
{
    s_virtual_tick_ms = ms;
}
#endif

/* ── Quaternion Vector Rotation ──────────────────────────────────── */
void quat_rotate_vec(const float q[4], const float v[3], float out[3])
{
    /* v' = q * [0,v] * q*
     * q* = [w, -x, -y, -z]  (conjugate of unit quaternion)
     */
    float qv[4] = { 0.0f, v[0], v[1], v[2] };
    float q_conj[4] = { q[0], -q[1], -q[2], -q[3] };

    float tmp[4];
    casper_quat_mult(q, qv, tmp);

    float result[4];
    casper_quat_mult(tmp, q_conj, result);

    out[0] = result[1];
    out[1] = result[2];
    out[2] = result[3];
}

/* ── Vertical Acceleration (FSM_TRANSITION_SPEC.md §2.2) ────────── */
float compute_vert_accel(const float q[4], const float accel_body_ms2[3])
{
    /* Rotate body accel to local-level frame (Z-UP, not true NED).
     * On pad: a_nav[2] ≈ +9.81 (gravity reaction in +Z = up).
     * Subtract gravity so stationary = 0g, boost > 0g, free-fall ≈ -1g. */
    float a_nav[3];
    quat_rotate_vec(q, accel_body_ms2, a_nav);

    return (a_nav[2] - 9.80665f) / 9.80665f;
}

/* ── Antenna-Up Check (FSM_TRANSITION_SPEC.md §2.3) ─────────────── */
bool check_antenna_up(const float q[4])
{
    float p, y, t;
    return check_antenna_up_tilt(q, &p, &y, &t);
}

bool check_antenna_up_tilt(const float q[4],
                           float *pitch_deg_out,
                           float *yaw_deg_out,
                           float *tilt_deg_out)
{
    /* Body Y-axis = nose/thrust axis [0,1,0].
     * Nav frame is Z-UP (not true NED): vertical = [0, 0, +1].
     *
     * by_nav[0] = nav-X component (starboard tilt)
     * by_nav[1] = nav-Y component (forward tilt)
     * by_nav[2] = nav-Z component (how much nose points up)
     *
     * pitch = atan2(by_nav[0], by_nav[2])  — tilt in nav XZ plane
     * yaw   = atan2(by_nav[1], by_nav[2])  — tilt in nav YZ plane
     * tilt  = acos(by_nav[2])              — total cone angle      */
    float body_y[3] = { 0.0f, 1.0f, 0.0f };
    float by_nav[3];
    quat_rotate_vec(q, body_y, by_nav);

    *pitch_deg_out = atan2f(by_nav[0], by_nav[2]) * 57.2957795f;
    *yaw_deg_out   = atan2f(by_nav[1], by_nav[2]) * 57.2957795f;

    float dot = by_nav[2];
    if (dot >  1.0f) dot =  1.0f;
    if (dot < -1.0f) dot = -1.0f;
    *tilt_deg_out = acosf(dot) * 57.2957795f;

    return dot > 0.98481f;   /* < 10 degrees from vertical: cos(10°) */
}
