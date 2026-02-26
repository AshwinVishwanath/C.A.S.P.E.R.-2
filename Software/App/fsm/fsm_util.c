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
    /* Rotate body accel to NED frame */
    float a_ned[3];
    quat_rotate_vec(q, accel_body_ms2, a_ned);

    /* NED convention: Z-down is positive. Negate so upward accel = positive g. */
    /* Subtract gravity so stationary = 0g, not 1g.                            */
    return (-a_ned[2] - 9.80665f) / 9.80665f;
}

/* ── Antenna-Up Check (FSM_TRANSITION_SPEC.md §2.3) ─────────────── */
bool check_antenna_up(const float q[4])
{
    /* Body Z-axis [0,0,1] rotated to NED */
    float body_z[3] = { 0.0f, 0.0f, 1.0f };
    float bz_ned[3];
    quat_rotate_vec(q, body_z, bz_ned);

    /* NED up = [0, 0, -1]. Dot product with body Z in NED: */
    float dot = -bz_ned[2];  /* cos(angle between body-Z and NED-up) */

    return dot > 0.866f;     /* < 30 degrees from vertical */
}
