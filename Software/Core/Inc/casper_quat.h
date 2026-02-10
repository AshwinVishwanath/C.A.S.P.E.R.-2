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
 * Initialize quaternion from accelerometer gravity vector on pad.
 * Determines pitch and roll from gravity direction; yaw is set to zero.
 * @param accel  Accelerometer reading [ax,ay,az] in m/s^2 (body frame, measuring gravity)
 * @param q      Output quaternion [w,x,y,z] (body-to-NED)
 */
void casper_quat_from_accel(const float accel[3], float q[4]);

/**
 * Convert quaternion to Euler angles (ZYX convention).
 * @param q    Input quaternion [w,x,y,z] (body-to-NED)
 * @param euler Output [roll, pitch, yaw] in degrees
 */
void casper_quat_to_euler(const float q[4], float euler[3]);

#ifdef __cplusplus
}
#endif

#endif /* CASPER_QUAT_H */
