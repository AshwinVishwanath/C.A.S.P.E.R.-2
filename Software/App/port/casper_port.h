/**
 * @file  casper_port.h
 * @brief Umbrella include for the CASPER portable seam.
 *
 * Every App/ file that currently includes "stm32h7xx_hal.h" or "main.h"
 * will instead include this single header after migration.  It pulls in
 * the full portable interface and <stdint.h> so that callers need no
 * other system headers for the port API.
 *
 * Include order is significant:
 *   1. <stdint.h>               — uint8_t / uint16_t / uint32_t etc.
 *   2. casper_types.h           — casper_status_t, opaque handles, casper_pin_t
 *   3. casper_time.h            — millis / micros / delay
 *   4. casper_gpio.h            — write / read / toggle
 *   5. casper_spi.h             — blocking SPI
 *   6. casper_i2c.h             — blocking I2C
 *   7. casper_adc.h             — ADC single-shot
 *   8. casper_qspi.h            — QSPI blocking + IT + handler
 *   9. casper_crc.h             — CRC-32
 *  10. casper_pwm.h             — PWM tone (buzzer)
 *
 * NO HAL or CMSIS types leak through this header.
 */

#ifndef CASPER_PORT_H
#define CASPER_PORT_H

#include <stdint.h>

#include "casper_types.h"
#include "casper_time.h"
#include "casper_gpio.h"
#include "casper_spi.h"
#include "casper_i2c.h"
#include "casper_adc.h"
#include "casper_qspi.h"
#include "casper_crc.h"
#include "casper_pwm.h"

#endif /* CASPER_PORT_H */
