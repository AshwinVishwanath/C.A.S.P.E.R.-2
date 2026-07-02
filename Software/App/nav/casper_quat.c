/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Quaternion Math
 *  SUMMARY:  Hamilton-product quaternion ops; body-to-NED rotations.
 * ============================================================ */
/**
 * @file casper_quat.c
 * @brief Quaternion operations for C.A.S.P.E.R.-2 flight computer.
 */

#include "casper_quat.h"
#include <math.h>

void casper_quat_mult(const float a[4], const float b[4], float r[4])
{
    float aw = a[0], ax = a[1], ay = a[2], az = a[3];
    float bw = b[0], bx = b[1], by = b[2], bz = b[3];

    r[0] = aw*bw - ax*bx - ay*by - az*bz;
    r[1] = aw*bx + ax*bw + ay*bz - az*by;
    r[2] = aw*by - ax*bz + ay*bw + az*bx;
    r[3] = aw*bz + ax*by - ay*bx + az*bw;
}

void casper_quat_normalize(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1.0e-10f) {
        float inv = 1.0f / norm;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }
}

void casper_quat_to_rotmat(const float q[4], float R[9])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;

    /* Row-major 3x3: R[row*3 + col] */
    R[0] = 1.0f - 2.0f*(yy + zz);
    R[1] = 2.0f*(xy - wz);
    R[2] = 2.0f*(xz + wy);

    R[3] = 2.0f*(xy + wz);
    R[4] = 1.0f - 2.0f*(xx + zz);
    R[5] = 2.0f*(yz - wx);

    R[6] = 2.0f*(xz - wy);
    R[7] = 2.0f*(yz + wx);
    R[8] = 1.0f - 2.0f*(xx + yy);
}

void casper_quat_from_accel(const float accel[3], float q[4])
{
    float ax = accel[0], ay = accel[1], az = accel[2];

    /*
     * Standard ZYX gravity-to-Euler extraction.  Works for any body frame.
     *
     * Y-nose convention (body = sensor native):
     *   +Y = nose (up on pad), +X = starboard, +Z = toward operator.
     *   On pad: accel ≈ [0, +g, 0] → roll = π/2, pitch = 0.
     *   Quaternion maps body +Y to reference +Z ("up" axis).
     *
     * pitch = atan2(-ax, sqrt(ay² + az²))   (body Y rotation)
     * roll  = atan2( ay, az)                 (body X rotation)
     * yaw   = 0                              (no magnetometer)
     */
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float roll  = atan2f( ay, az);

    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll  * 0.5f);
    float sr = sinf(roll  * 0.5f);
    /* yaw = 0  →  cy = 1, sy = 0 */

    q[0] = cp * cr;           /* w */
    q[1] = cp * sr;           /* x */
    q[2] = sp * cr;           /* y */
    q[3] = -sp * sr;          /* z */

    casper_quat_normalize(q);
}

void casper_quat_from_euler(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);

    q[0] = cr*cp*cy + sr*sp*sy;   /* w */
    q[1] = sr*cp*cy - cr*sp*sy;   /* x */
    q[2] = cr*sp*cy + sr*cp*sy;   /* y */
    q[3] = cr*cp*sy - sr*sp*cy;   /* z */

    casper_quat_normalize(q);
}

void casper_quat_to_euler(const float q[4], float euler[3])
{
    /* Tilt-from-vertical convention for the Y-nose airframe (hardware-validated).
     * Nominal nose-up attitude q0 = [sqrt2/2, -sqrt2/2, 0, 0] (Rx(-90) body->ref).
     * Deviation qd = conj(q0) x q, then intrinsic Z-X-Y extraction:
     *   qd = Rz(yaw) * Rx(pitch) * Ry(roll)
     *   roll  (body Y, nose spin)      = INNER  = atan2 -> full +/-180 deg
     *   pitch (body X, fore/aft tilt)  = MIDDLE = asin  -> singularity at +/-90 (nose horizontal)
     *   yaw   (body Z, side tilt / heading) = OUTER = atan2
     * Key property: a pure spin about the nose (body Y) changes ONLY roll; pitch
     * and yaw report true tilt-from-vertical, invariant under nose-spin/heading.
     * Nose-up-no-spin -> (0,0,0).  Array layout UNCHANGED (callers unmodified):
     *   euler[0] = yaw   (heading / side tilt),  euler[1] = roll (nose spin),
     *   euler[2] = pitch (fore/aft tilt).   All in degrees. */

    static const float INV_SQRT2 = 0.70710678118f;
    static const float RAD2DEG   = 57.29577951f;

    /* Step 1: deviation qd = conj(q0) x q  (conj(q0) = [s, +s, 0, 0]) */
    float wd = INV_SQRT2 * (q[0] - q[1]);
    float xd = INV_SQRT2 * (q[0] + q[1]);
    float yd = INV_SQRT2 * (q[2] - q[3]);
    float zd = INV_SQRT2 * (q[2] + q[3]);

    /* Step 2: Z-X-Y decomposition of qd */
    float sinp = 2.0f * (yd*zd + wd*xd);          /* R[2][1] = sin(pitch) */
    if      (sinp >=  1.0f) sinp =  1.0f;
    else if (sinp <= -1.0f) sinp = -1.0f;
    euler[2] = asinf(sinp) * RAD2DEG;             /* pitch (body X) */

    euler[1] = atan2f(2.0f*(wd*yd - xd*zd),        /* roll (body Y), full +/-180 */
                      1.0f - 2.0f*(xd*xd + yd*yd)) * RAD2DEG;

    euler[0] = atan2f(2.0f*(wd*zd - xd*yd),        /* yaw (body Z) */
                      1.0f - 2.0f*(xd*xd + zd*zd)) * RAD2DEG;
}
