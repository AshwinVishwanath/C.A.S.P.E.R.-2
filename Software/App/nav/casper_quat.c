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
    /* Body frame: +Y = nose (thrust axis, up on pad), +X = starboard, +Z = toward operator.
     *
     * Nominal nose-up attitude: q0 = [sqrt2/2, -sqrt2/2, 0, 0]  (Rx(-90 deg) body->ref).
     * Step 1: compute deviation  qd = conj(q0) x q.
     *         conj(q0) = [sqrt2/2, +sqrt2/2, 0, 0].
     *         With s = sqrt2/2, the product simplifies to:
     *           wd = s*(q[0] - q[1])
     *           xd = s*(q[0] + q[1])
     *           yd = s*(q[2] - q[3])
     *           zd = s*(q[2] + q[3])
     *
     * Step 2: extract intrinsic Z-X-Y Euler angles from qd:
     *   qd = Rz(yaw) * Rx(pitch) * Ry(roll)
     *   roll  (body Y, nose spin)      = INNER  = atan2 => full +/-180 deg
     *   pitch (body X, fore/aft tilt)  = MIDDLE = asin  => singularity at +/-90 (nose horizontal)
     *   yaw   (body Z, side tilt / heading) = OUTER = atan2
     *
     * Key property: a pure spin about the nose (body Y) changes ONLY roll;
     * pitch and yaw are invariant under nose-spin.
     *
     * Output array layout (UNCHANGED — callers must not be modified):
     *   euler[0] = yaw   (body Z rotation, heading / side tilt)  degrees
     *   euler[1] = roll  (body Y rotation, nose spin)             degrees
     *   euler[2] = pitch (body X rotation, fore/aft tilt)         degrees   */

    static const float INV_SQRT2 = 0.70710678118f;
    static const float RAD2DEG   = 57.29577951f;   /* 180 / pi */

    /* --- Step 1: deviation qd = conj(q0) x q --- */
    float wd = INV_SQRT2 * (q[0] - q[1]);
    float xd = INV_SQRT2 * (q[0] + q[1]);
    float yd = INV_SQRT2 * (q[2] - q[3]);
    float zd = INV_SQRT2 * (q[2] + q[3]);

    /* --- Step 2: ZXY decomposition of qd --- */

    /* pitch (body X, middle rotation): R[2][1] = sin(pitch) = 2*(yd*zd + wd*xd) */
    float sinp = 2.0f * (yd*zd + wd*xd);
    if      (sinp >=  1.0f) sinp =  1.0f;
    else if (sinp <= -1.0f) sinp = -1.0f;
    float pitch_deg = asinf(sinp) * RAD2DEG;

    /* roll (body Y, inner rotation): atan2(-R[2][0], R[2][2])
     * R[2][0] = 2*(xd*zd - wd*yd),  R[2][2] = 1 - 2*(xd*xd + yd*yd) */
    float roll_deg  = atan2f(2.0f*(wd*yd - xd*zd),
                             1.0f - 2.0f*(xd*xd + yd*yd)) * RAD2DEG;

    /* yaw (body Z, outer rotation): atan2(-R[0][1], R[1][1])
     * R[0][1] = 2*(xd*yd - wd*zd),  R[1][1] = 1 - 2*(xd*xd + zd*zd) */
    float yaw_deg   = atan2f(2.0f*(wd*zd - xd*yd),
                             1.0f - 2.0f*(xd*xd + zd*zd)) * RAD2DEG;

    euler[0] = yaw_deg;
    euler[1] = roll_deg;
    euler[2] = pitch_deg;
}
