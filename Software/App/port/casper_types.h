/**
 * @file  casper_types.h
 * @brief Portable type definitions for the CASPER port seam.
 *
 * Pure C11.  No HAL or CMSIS types may appear here.
 * All board-specific struct definitions live in the corresponding
 * board_<target>/board_<target>.c; these forward declarations keep
 * App/ code fully independent of the board implementation.
 */

#ifndef CASPER_TYPES_H
#define CASPER_TYPES_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 *  Status codes
 *  All casper_* functions that can fail return one of these.
 * --------------------------------------------------------------------- */
typedef enum {
    CASPER_OK      =  0,   /**< Operation completed successfully.              */
    CASPER_ERR     = -1,   /**< General / peripheral error (HAL_ERROR etc.).   */
    CASPER_TIMEOUT = -2    /**< Operation did not complete within timeout_ms.   */
} casper_status_t;

/* -----------------------------------------------------------------------
 *  Opaque bus-handle types
 *
 *  Each board .c file provides the concrete struct definition.
 *  App/ code only ever holds pointers to these types; it never
 *  dereferences the internals — maintaining a hard abstraction boundary.
 * --------------------------------------------------------------------- */

/** Opaque SPI bus handle (e.g. wraps SPI_HandleTypeDef* on Casper 2). */
typedef struct casper_spi_s  casper_spi_t;

/** Opaque I2C bus handle (e.g. wraps I2C_HandleTypeDef* on Casper 2). */
typedef struct casper_i2c_s  casper_i2c_t;

/** Opaque QSPI/OSPI flash-controller handle (wraps QSPI_HandleTypeDef* on Casper 2). */
typedef struct casper_qspi_s casper_qspi_t;

/** Opaque ADC channel handle (wraps HAL ADC handle + channel config on Casper 2). */
typedef struct casper_adc_s  casper_adc_t;

/** Opaque PWM output handle (wraps TIM handle + channel on Casper 2). */
typedef struct casper_pwm_s  casper_pwm_t;

/* -----------------------------------------------------------------------
 *  GPIO pin descriptor
 *
 *  Carries a (port, pin) pair without exposing GPIO_TypeDef.
 *  The board layer casts `port` back to the concrete GPIO type.
 *  `pin` carries the bit-mask value exactly as used in HAL
 *  (GPIO_PIN_n = 1u << n on STM32 HAL).
 * --------------------------------------------------------------------- */
typedef struct {
    void    *port;   /**< Board casts to GPIO_TypeDef* (STM32) or equivalent. */
    uint32_t pin;    /**< Bit-mask pin identifier (GPIO_PIN_n).               */
} casper_pin_t;

#endif /* CASPER_TYPES_H */
