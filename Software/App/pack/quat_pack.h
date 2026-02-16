#ifndef APP_PACK_QUAT_PACK_H
#define APP_PACK_QUAT_PACK_H

#include <stdint.h>

/**
 * Smallest-three quaternion packing into 5 bytes (40 bits).
 *
 * Bit layout (MSB first):
 *   [39:38] drop index (0=w, 1=x, 2=y, 3=z)
 *   [37:36] reserved (0)
 *   [35:24] qa (int12, signed)
 *   [23:12] qb (int12, signed)
 *   [11:0]  qc (int12, signed)
 *
 * Byte packing (little-endian on wire):
 *   packed[0] = qc[7:0]
 *   packed[1] = qb[3:0] | qc[11:8]
 *   packed[2] = qb[11:4]
 *   packed[3] = qa[7:0]
 *   packed[4] = drop[1:0] | rsvd[1:0] | qa[11:8]
 *
 * @param out  Output buffer, exactly 5 bytes
 * @param q    Unit quaternion [w, x, y, z]
 */
void quat_pack_smallest_three(uint8_t out[5], const float q[4]);

#endif /* APP_PACK_QUAT_PACK_H */
