/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Hamming SECDED
 *  SUMMARY:  Single-error-correct/double-error-detect for log records.
 * ============================================================ */
#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * Compute Hamming SECDED parity over data_len bytes.
 * For 60 bytes (480 bits): P=9 syndrome bits + 1 overall parity = 10 bits.
 *
 * @param data      Input data bytes
 * @param data_len  Number of bytes (typically 60 for hr_record_t)
 * @return          uint16_t with 10 parity bits in lower 10 bits
 */
uint16_t hamming_encode(const uint8_t *data, size_t data_len);

/**
 * Check and correct single-bit errors using Hamming SECDED.
 *
 * @param data      Data bytes (corrected in-place if single-bit error)
 * @param data_len  Number of data bytes
 * @param parity    10-bit parity value from hamming_encode()
 * @return          0 = no error, 1 = single-bit corrected, -1 = double-bit detected
 */
int hamming_decode(uint8_t *data, size_t data_len, uint16_t parity);

/**
 * CRC16-CCITT (polynomial 0x1021, init 0xFFFF).
 *
 * @param data  Input bytes
 * @param len   Number of bytes
 * @return      16-bit CRC
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);
