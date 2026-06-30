/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Quaternion Packer
 *  SUMMARY:  Smallest-three quaternion compression for telemetry (5B).
 * ============================================================ */
#include "quat_pack.h"
#include <math.h>

void quat_pack_smallest_three(uint8_t out[5], const float q[4])
{
    /* 1. Find index of largest-magnitude component */
    int drop = 0;
    float max_abs = fabsf(q[0]);
    for (int i = 1; i < 4; i++) {
        float a = fabsf(q[i]);
        if (a > max_abs) {
            drop = i;
            max_abs = a;
        }
    }

    /* 2. If dropped component is negative, negate all (q == -q for rotations) */
    float c[4];
    float sign = (q[drop] < 0.0f) ? -1.0f : 1.0f;
    for (int i = 0; i < 4; i++) {
        c[i] = q[i] * sign;
    }

    /* 3. Extract three remaining components in ascending index order */
    float rem[3];
    int ri = 0;
    for (int i = 0; i < 4; i++) {
        if (i != drop) {
            rem[ri++] = c[i];
        }
    }

    /* 4. Scale to int12: value * 4096, clamp to [-2048, 2047] */
    int16_t qa = (int16_t)roundf(rem[0] * 4096.0f);
    int16_t qb = (int16_t)roundf(rem[1] * 4096.0f);
    int16_t qc = (int16_t)roundf(rem[2] * 4096.0f);

    if (qa > 2047)  qa = 2047;
    if (qa < -2048) qa = -2048;
    if (qb > 2047)  qb = 2047;
    if (qb < -2048) qb = -2048;
    if (qc > 2047)  qc = 2047;
    if (qc < -2048) qc = -2048;

    /* Mask to 12 bits */
    uint16_t ua = (uint16_t)(qa & 0x0FFF);
    uint16_t ub = (uint16_t)(qb & 0x0FFF);
    uint16_t uc = (uint16_t)(qc & 0x0FFF);

    /* 5. Pack into 5 bytes per level-1.md bit layout */
    out[0] = (uint8_t)(uc & 0xFF);
    out[1] = (uint8_t)(((ub << 4) & 0xF0) | ((uc >> 8) & 0x0F));
    out[2] = (uint8_t)((ub >> 4) & 0xFF);
    out[3] = (uint8_t)(ua & 0xFF);
    out[4] = (uint8_t)(((uint8_t)drop << 6) | ((ua >> 8) & 0x0F));
}

void quat_unpack_smallest_three(const uint8_t in[5], float q[4])
{
    /* 1. Extract drop index from bits [7:6] of byte 4 */
    int drop = (in[4] >> 6) & 0x03;

    /* 2. Reconstruct three 12-bit unsigned words
     *    ua -> rem[0] (packed into in[3] and in[4][3:0])
     *    ub -> rem[1] (packed into in[2] and in[1][7:4])
     *    uc -> rem[2] (packed into in[0] and in[1][3:0])
     *
     *    Bit layout mirrors the packer:
     *      in[3]       = ua[7:0]
     *      in[4][3:0]  = ua[11:8]
     *      in[2]       = ub[11:4]
     *      in[1][7:4]  = ub[3:0]
     *      in[0]       = uc[7:0]
     *      in[1][3:0]  = uc[11:8]
     */
    uint16_t ua = (uint16_t)(((uint16_t)(in[4] & 0x0F) << 8) | in[3]);
    uint16_t ub = (uint16_t)(((uint16_t)in[2] << 4) | ((in[1] >> 4) & 0x0F));
    uint16_t uc = (uint16_t)(((uint16_t)(in[1] & 0x0F) << 8) | in[0]);

    /* 3. Sign-extend each 12-bit value to signed (two's complement) */
    int16_t sa = (ua & 0x800U) ? (int16_t)ua - 4096 : (int16_t)ua;
    int16_t sb = (ub & 0x800U) ? (int16_t)ub - 4096 : (int16_t)ub;
    int16_t sc = (uc & 0x800U) ? (int16_t)uc - 4096 : (int16_t)uc;

    /* 4. Scale back to float: rem = signed / 4096.0f */
    float rem[3];
    rem[0] = (float)sa / 4096.0f;
    rem[1] = (float)sb / 4096.0f;
    rem[2] = (float)sc / 4096.0f;

    /* 5. Place rem[0..2] into the three non-drop indices in ascending order */
    int ri = 0;
    for (int i = 0; i < 4; i++) {
        if (i != drop) {
            q[i] = rem[ri++];
        }
    }

    /* 6. Recover dropped component (always non-negative by pack convention) */
    float sumsq = rem[0]*rem[0] + rem[1]*rem[1] + rem[2]*rem[2];
    q[drop] = sqrtf(fmaxf(0.0f, 1.0f - sumsq));
}
