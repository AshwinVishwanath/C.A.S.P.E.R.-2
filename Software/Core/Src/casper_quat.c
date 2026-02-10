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
     * On the pad, the accelerometer measures +1g in the direction opposite to
     * gravity.  In NED frame, gravity points +Z (down).  We need the rotation
     * that maps the body-frame gravity measurement to NED [0, 0, +g].
     *
     * pitch = atan2(-ax, az)   (rotation about body Y)
     * roll  = atan2( ay, az)   (rotation about body X)
     * yaw   = 0                (no magnetometer)
     *
     * Euler (ZYX convention) to quaternion:
     */
    float pitch = atan2f(-ax, az);
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

void casper_quat_to_euler(const float q[4], float euler[3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    /* Roll (X) */
    float sinr = 2.0f * (w*x + y*z);
    float cosr = 1.0f - 2.0f * (x*x + y*y);
    euler[0] = atan2f(sinr, cosr) * (180.0f / 3.14159265f);

    /* Pitch (Y) — clamp to avoid NaN at gimbal lock */
    float sinp = 2.0f * (w*y - z*x);
    if (sinp >= 1.0f)       sinp = 1.0f;
    else if (sinp <= -1.0f) sinp = -1.0f;
    euler[1] = asinf(sinp) * (180.0f / 3.14159265f);

    /* Yaw (Z) */
    float siny = 2.0f * (w*z + x*y);
    float cosy = 1.0f - 2.0f * (y*y + z*z);
    euler[2] = atan2f(siny, cosy) * (180.0f / 3.14159265f);
}
