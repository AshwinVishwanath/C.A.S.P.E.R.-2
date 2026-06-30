/**
 * @file  casper_i2c.h
 * @brief Portable blocking I2C interface.
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 *
 * -----------------------------------------------------------------------
 *  addr8 convention (CRITICAL — matches existing call sites)
 * -----------------------------------------------------------------------
 *  All address parameters are the 8-bit, left-shifted device address —
 *  i.e. the 7-bit I2C address shifted left by one bit:
 *
 *      addr8 = i2c_7bit_address << 1
 *
 *  Examples:
 *    MAX-M10M GPS  — 7-bit = 0x42  →  addr8 = 0x84
 *    MMC5983MA mag — 7-bit = 0x30  →  addr8 = 0x60
 *
 *  This matches the STM32 HAL_I2C_* convention used throughout the
 *  existing drivers and must NOT be changed by the board layer
 *  (it passes addr8 unchanged to the HAL).
 */

#ifndef CASPER_I2C_H
#define CASPER_I2C_H

#include "casper_types.h"

/**
 * @brief Read @p n bytes from a device register (I2C Mem Read).
 *
 * Equivalent to HAL_I2C_Mem_Read().
 *
 * @param bus      Opaque I2C bus handle.
 * @param addr8    8-bit device address (7-bit address << 1).
 * @param reg      Register address (sub-address / memory address).
 * @param reg_sz   Size of @p reg in bytes: 1 for 8-bit reg addr,
 *                 2 for 16-bit reg addr (I2C_MEMADD_SIZE_8BIT / _16BIT).
 * @param buf      Receive buffer, must be at least @p n bytes.
 * @param n        Number of bytes to read.
 * @param to_ms    Timeout in milliseconds.
 * @return         CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_i2c_mem_read(casper_i2c_t *bus,
                                    uint16_t addr8,
                                    uint16_t reg,
                                    uint16_t reg_sz,
                                    uint8_t *buf,
                                    uint16_t n,
                                    uint32_t to_ms);

/**
 * @brief Write @p n bytes to a device register (I2C Mem Write).
 *
 * Equivalent to HAL_I2C_Mem_Write().
 *
 * @param bus      Opaque I2C bus handle.
 * @param addr8    8-bit device address (7-bit address << 1).
 * @param reg      Register address.
 * @param reg_sz   Size of @p reg in bytes (1 or 2).
 * @param buf      Transmit buffer, must be at least @p n bytes.
 * @param n        Number of bytes to write.
 * @param to_ms    Timeout in milliseconds.
 * @return         CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_i2c_mem_write(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     uint16_t reg,
                                     uint16_t reg_sz,
                                     const uint8_t *buf,
                                     uint16_t n,
                                     uint32_t to_ms);

/**
 * @brief Transmit @p n bytes directly to a device (I2C Master Transmit).
 *
 * Equivalent to HAL_I2C_Master_Transmit().  Used when the protocol does
 * not use a separate register/sub-address phase (e.g. UBX config frames).
 *
 * @param bus      Opaque I2C bus handle.
 * @param addr8    8-bit device address (7-bit address << 1).
 * @param buf      Transmit buffer, must be at least @p n bytes.
 * @param n        Number of bytes to transmit.
 * @param to_ms    Timeout in milliseconds.
 * @return         CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_i2c_master_tx(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     const uint8_t *buf,
                                     uint16_t n,
                                     uint32_t to_ms);

/**
 * @brief Check whether a device is ready on the I2C bus.
 *
 * Equivalent to HAL_I2C_IsDeviceReady().  Sends a start + address + stop
 * and reports whether the device ACKed.
 *
 * @param bus      Opaque I2C bus handle.
 * @param addr8    8-bit device address (7-bit address << 1).
 * @param trials   Number of attempts before giving up.
 * @param to_ms    Per-trial timeout in milliseconds.
 * @return         CASPER_OK if device ACKed within the trials,
 *                 CASPER_TIMEOUT or CASPER_ERR otherwise.
 */
casper_status_t casper_i2c_dev_ready(casper_i2c_t *bus,
                                     uint16_t addr8,
                                     uint32_t trials,
                                     uint32_t to_ms);

#endif /* CASPER_I2C_H */
