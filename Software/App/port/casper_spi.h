/**
 * @file  casper_spi.h
 * @brief Portable blocking SPI interface.
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 * All functions are blocking and return when the transfer is complete
 * or when the timeout expires.
 *
 * CS assertion / de-assertion is the caller's responsibility (use
 * casper_gpio_write() with the device's cs casper_pin_t).
 */

#ifndef CASPER_SPI_H
#define CASPER_SPI_H

#include "casper_types.h"

/**
 * @brief Transmit @p n bytes from @p tx over SPI.
 *
 * Equivalent to HAL_SPI_Transmit().  Received bytes are discarded by the
 * board layer.
 *
 * @param bus      Opaque SPI bus handle.
 * @param tx       Pointer to transmit buffer (@p n bytes, must not be NULL).
 * @param n        Number of bytes to transmit.
 * @param to_ms    Timeout in milliseconds.
 * @return         CASPER_OK on success, CASPER_TIMEOUT if the peripheral did
 *                 not complete within @p to_ms ms, CASPER_ERR on HAL error.
 */
casper_status_t casper_spi_transmit(casper_spi_t *bus,
                                    const uint8_t *tx,
                                    uint16_t n,
                                    uint32_t to_ms);

/**
 * @brief Receive @p n bytes from SPI into @p rx.
 *
 * Equivalent to HAL_SPI_Receive().  The board layer transmits dummy bytes
 * (0xFF) to clock in data from the device.
 *
 * @param bus      Opaque SPI bus handle.
 * @param rx       Pointer to receive buffer (@p n bytes, must not be NULL).
 * @param n        Number of bytes to receive.
 * @param to_ms    Timeout in milliseconds.
 * @return         CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_spi_receive(casper_spi_t *bus,
                                   uint8_t *rx,
                                   uint16_t n,
                                   uint32_t to_ms);

/**
 * @brief Full-duplex SPI transfer: transmit @p tx and simultaneously receive
 *        into @p rx (@p n bytes each).
 *
 * Equivalent to HAL_SPI_TransmitReceive().  @p tx and @p rx must be distinct
 * non-overlapping buffers of at least @p n bytes.
 *
 * @param bus      Opaque SPI bus handle.
 * @param tx       Bytes to transmit (must not be NULL).
 * @param rx       Buffer for received bytes (must not be NULL).
 * @param n        Transfer length in bytes.
 * @param to_ms    Timeout in milliseconds.
 * @return         CASPER_OK, CASPER_TIMEOUT, or CASPER_ERR.
 */
casper_status_t casper_spi_transceive(casper_spi_t *bus,
                                      const uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t n,
                                      uint32_t to_ms);

#endif /* CASPER_SPI_H */
