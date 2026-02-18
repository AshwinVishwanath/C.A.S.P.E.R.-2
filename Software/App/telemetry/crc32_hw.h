#ifndef APP_TELEMETRY_CRC32_HW_H
#define APP_TELEMETRY_CRC32_HW_H

#include <stdint.h>

/**
 * Initialize CRC-32 wrapper.
 * Reconfigures the STM32 CRC peripheral for standard CRC-32
 * (reflected I/O, final XOR 0xFFFFFFFF).
 *
 * CRC-32 Configuration (MC must match exactly):
 *   Polynomial:      0x04C11DB7
 *   Initial value:   0xFFFFFFFF
 *   Input reflection: YES (bit-reversed bytes)
 *   Output reflection: YES
 *   Final XOR:       0xFFFFFFFF
 *
 *   Check value for ASCII "123456789" = 0xCBF43926
 *
 * Must be called after MX_CRC_Init().
 */
void crc32_hw_init(void);

/**
 * Compute CRC-32 over data[0..len-1].
 *
 * @param data  Input bytes
 * @param len   Number of bytes
 * @return      32-bit CRC value
 */
uint32_t crc32_hw_compute(const uint8_t *data, uint32_t len);

/**
 * Validate a received CRC-32.
 *
 * @param data         Payload bytes
 * @param payload_len  Number of payload bytes (CRC is computed over these)
 * @param received_crc CRC-32 value received in the packet
 * @return             1 if valid, 0 if mismatch
 */
int crc32_hw_validate(const uint8_t *data, uint32_t payload_len,
                      uint32_t received_crc);

/**
 * Continue CRC-32 computation from a previous value.
 * Used for streaming CRC over multiple data chunks.
 *
 * @param prev_crc  Previous CRC value (pass 0 to start fresh)
 * @param data      Input bytes for this chunk
 * @param len       Number of bytes
 * @return          Updated CRC-32 value
 */
uint32_t crc32_hw_compute_continue(uint32_t prev_crc, const uint8_t *data, uint32_t len);

#endif /* APP_TELEMETRY_CRC32_HW_H */
