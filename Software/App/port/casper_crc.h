/**
 * @file  casper_crc.h
 * @brief Portable CRC-32 interface (hardware-accelerated on STM32).
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 *
 * On Casper 2 this wraps the STM32H7 CRC hardware unit (HAL_CRC_*).
 * The algorithm is standard CRC-32 (IEEE 802.3 / PKZIP polynomial),
 * with reflected input/output and a final XOR of 0xFFFFFFFF, matching
 * the existing crc32_hw.c behaviour.
 */

#ifndef CASPER_CRC_H
#define CASPER_CRC_H

#include <stdint.h>

/**
 * @brief Initialise the CRC peripheral.
 *
 * Must be called once at startup (after the CRC peripheral clock is
 * enabled by the generated init).  Configures the hardware unit for
 * standard CRC-32 with reflected I/O.
 *
 * On Casper 2 equivalent to casper_crc32_init() / crc32_hw_init().
 */
void casper_crc32_init(void);

/**
 * @brief Compute the CRC-32 of @p len bytes starting at @p data.
 *
 * Calculates the standard CRC-32 checksum (polynomial 0x04C11DB7,
 * reflected input and output, initial value 0xFFFFFFFF, final
 * XOR 0xFFFFFFFF).  This matches the zlib / PKZIP CRC-32.
 *
 * @param data  Pointer to input data (must not be NULL if @p len > 0).
 * @param len   Number of bytes to process.
 * @return      32-bit CRC value.
 */
uint32_t casper_crc32_compute(const uint8_t *data, uint32_t len);

#endif /* CASPER_CRC_H */
