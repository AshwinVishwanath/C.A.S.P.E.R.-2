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
