/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Quaternion Math
 *  SUMMARY:  Hamilton-product quaternion ops; body-to-NED rotations.
 * ============================================================ */
/**
 * @file casper_quat.h
 * @brief Quaternion operations for C.A.S.P.E.R.-2 flight computer.
 *
 * Convention: q[4] = {w, x, y, z}, Hamilton product, scalar-first, body-to-NED.
 */

#ifndef CASPER_QUAT_H
#define CASPER_QUAT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Hamilton quaternion product: r = a (x) b
 * @param a  Input quaternion [w,x,y,z]
 * @param b  Input quaternion [w,x,y,z]
 * @param r  Output quaternion [w,x,y,z] (may NOT alias a or b)
 */
void casper_quat_mult(const float a[4], const float b[4], float r[4]);

/**
 * Normalize quaternion in-place: q = q / |q|
 */
void casper_quat_normalize(float q[4]);

/**
 * Convert quaternion to 3x3 rotation matrix (body-to-NED).
 * @param q  Input quaternion [w,x,y,z]
 * @param R  Output 3x3 matrix, row-major float[9]
 */
void casper_quat_to_rotmat(const float q[4], float R[9]);

/**
 * Build quaternion from ZYX Euler angles (all in radians).
 * @param roll_rad   Roll  (rotation about body X) in radians
 * @param pitch_rad  Pitch (rotation about body Y) in radians
 * @param yaw_rad    Yaw   (rotation about body Z) in radians
 * @param q          Output quaternion [w, x, y, z], body-to-NED
 */
void casper_quat_from_euler(float roll_rad, float pitch_rad, float yaw_rad,
                            float q[4]);

/**
 * Extract tilt-from-vertical Euler angles from a body-to-reference quaternion.
 *
 * Body frame: +Y = nose (up on pad), +X = starboard, +Z = toward operator.
 * Nominal nose-up attitude q0 = [sqrt2/2, -sqrt2/2, 0, 0].
 *
 * Internally computes deviation qd = conj(q0) x q, then decomposes with
 * intrinsic Z-X-Y order (Rz * Rx * Ry), so that a pure nose-spin (body Y)
 * changes ONLY roll and leaves pitch and yaw unchanged.
 *
 * @param q     Input quaternion [w,x,y,z] (body-to-NED, from Mahony filter)
 * @param euler Output angles in degrees:
 *              euler[0] = yaw   (body Z, side-tilt / heading),  atan2 full range
 *              euler[1] = roll  (body Y, nose spin),            atan2 full +/-180
 *              euler[2] = pitch (body X, fore/aft tilt),        asin, singular at +/-90
 */
void casper_quat_to_euler(const float q[4], float euler[3]);

#ifdef __cplusplus
}
#endif

#endif /* CASPER_QUAT_H */
