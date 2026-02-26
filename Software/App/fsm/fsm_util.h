#ifndef APP_FSM_FSM_UTIL_H
#define APP_FSM_FSM_UTIL_H

#include "fsm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Virtual Clock (FSM_TRANSITION_SPEC.md §3.1) ────────────────── */
#ifdef HIL_MODE
uint32_t fsm_get_tick(void);
void     fsm_set_tick(uint32_t ms);
#else
#include "stm32h7xx_hal.h"
#define  fsm_get_tick() HAL_GetTick()
#endif

/* ── Dwell Timer (FSM_TRANSITION_SPEC.md §3) ─────────────────────── */

/**
 * Sustained-condition check with dwell timer.
 * Returns true only if condition has been continuously true for
 * at least required_ms. Resets if condition drops out.
 */
static inline bool dwell_check(dwell_timer_t *t, bool condition, uint32_t required_ms)
{
    if (!condition) {
        t->active = false;
        return false;
    }
    if (!t->active) {
        t->start_ms = fsm_get_tick();
        t->active = true;
    }
    return (fsm_get_tick() - t->start_ms) >= required_ms;
}

/* ── Quaternion Vector Rotation ──────────────────────────────────── */

/**
 * Rotate a 3-vector by a unit quaternion: v' = q * [0,v] * q*
 * Uses Hamilton product from casper_quat_mult().
 *
 * @param q    Unit quaternion [w,x,y,z] (body-to-NED)
 * @param v    Input 3-vector (body frame)
 * @param out  Output 3-vector (NED frame)
 */
void quat_rotate_vec(const float q[4], const float v[3], float out[3]);

/* ── Vertical Acceleration (FSM_TRANSITION_SPEC.md §2.2) ────────── */

/**
 * Project body-frame accel onto NED vertical axis using attitude quat.
 * Returns vertical acceleration in g-units:
 *   stationary ~= 0g, boost > 0g, free fall ~= -1g
 */
float compute_vert_accel(const float q[4], const float accel_body_ms2[3]);

/* ── Antenna-Up Check (FSM_TRANSITION_SPEC.md §2.3) ─────────────── */

/**
 * Check if rocket is upright (body Z-axis within 30 deg of NED up).
 */
bool check_antenna_up(const float q[4]);

#ifdef __cplusplus
}
#endif

#endif /* APP_FSM_FSM_UTIL_H */
