/**
 * @file  casper_gpio.h
 * @brief Portable GPIO write / read / toggle interface.
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 * Uses casper_pin_t from casper_types.h so that App/ code never
 * references GPIO_TypeDef or any other HAL type directly.
 */

#ifndef CASPER_GPIO_H
#define CASPER_GPIO_H

#include "casper_types.h"

/* -----------------------------------------------------------------------
 *  Pin state
 * --------------------------------------------------------------------- */
typedef enum {
    CASPER_PIN_LOW  = 0,   /**< Drive / sample the pin low  (logic 0). */
    CASPER_PIN_HIGH = 1    /**< Drive / sample the pin high (logic 1). */
} casper_pin_state_t;

/* -----------------------------------------------------------------------
 *  API
 * --------------------------------------------------------------------- */

/**
 * @brief Drive a GPIO output pin to the given state.
 *
 * Equivalent to HAL_GPIO_WritePin().  The pin must already be configured
 * as a push-pull output by the board init (MX_GPIO_Init on STM32).
 *
 * @param pin  Pin descriptor (port + bit-mask).
 * @param s    Desired output state.
 */
void casper_gpio_write(casper_pin_t pin, casper_pin_state_t s);

/**
 * @brief Read the current level of a GPIO pin.
 *
 * Works for both input and output-configured pins.
 * Equivalent to HAL_GPIO_ReadPin().
 *
 * @param pin  Pin descriptor (port + bit-mask).
 * @return     CASPER_PIN_LOW or CASPER_PIN_HIGH.
 */
casper_pin_state_t casper_gpio_read(casper_pin_t pin);

/**
 * @brief Toggle a GPIO output pin.
 *
 * Equivalent to HAL_GPIO_TogglePin().  The pin must be configured as an
 * output.
 *
 * @param pin  Pin descriptor (port + bit-mask).
 */
void casper_gpio_toggle(casper_pin_t pin);

#endif /* CASPER_GPIO_H */
