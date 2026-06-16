/**
 * @file  board_casper2.h
 * @brief Casper 2 board singleton declarations and BSP_PIN_* macros.
 *
 * This header is the only file in App/ that may be included by code that
 * needs to reference a specific bus handle or GPIO pin by name.  All types
 * are from casper_port.h — no HAL or CMSIS types appear here.
 *
 * Include hierarchy:
 *   casper_port.h   (umbrella portable seam)
 *   board_casper2.h (board-specific instances — #include after casper_port.h)
 *
 * HARD RULE: Only board_casper2.c (and App/radio/sx1276.c by approved
 * exception) may #include "main.h" or "stm32h7xx_hal.h".  This header
 * deliberately contains no HAL types.
 */

#ifndef BOARD_CASPER2_H
#define BOARD_CASPER2_H

#include "casper_port.h"   /* casper_spi_t, casper_i2c_t, casper_pin_t … */

/* =========================================================================
 *  Bus singletons
 *
 *  Defined in board_casper2.c.  Include this header and pass a pointer to
 *  the appropriate singleton when initialising a driver.
 * ======================================================================= */

/** SPI2: LSM6DSO32 primary IMU (PC14/CS, PC15/INT2, SPI Mode 3, 5.25 MHz). */
extern casper_spi_t BSP_SPI_IMU;

/** SPI4: MS5611 barometer (PE11/CS, SPI Mode 0, 10.5 MHz). */
extern casper_spi_t BSP_SPI_BARO;

/** SPI3: ADXL372 high-g accelerometer (PA15/CS, PD2/INT, SPI Mode 3). */
extern casper_spi_t BSP_SPI_HIGHG;

/** I2C1: MAX-M10M GPS (addr8 = 0x84, PB8/SCL, PB9/SDA). */
extern casper_i2c_t BSP_I2C_GPS;

/** I2C3: MMC5983MA magnetometer (addr8 = 0x60, PA8/SCL, PC9/SDA). */
extern casper_i2c_t BSP_I2C_MAG;

/** QUADSPI: W25Q512JV 64 MB NOR flash (BK1, 4-byte address mode). */
extern casper_qspi_t BSP_QSPI_FLASH;

/** TIM4 CH3: Buzzer PWM (PD14, 216 MHz timer clock). */
extern casper_pwm_t BSP_PWM_BUZZ;

/**
 * @brief ADC continuity-sense pointer array (one casper_adc_t* per pyro channel).
 *
 * Each element is a pointer to an opaque casper_adc_t (defined only in
 * board_casper2.c).  App/ code passes this array directly to
 * casper_pyro_init() / pyro_mgr_init() without needing the concrete struct size.
 *
 * Mapping:
 *   [0] CH1 — ADC1 channel 4  (PC4,   CONT1)
 *   [1] CH2 — ADC1 channel 3  (PA6,   CONT_2)
 *   [2] CH3 — ADC3 channel 1  (PC3_C, CONT_3)
 *   [3] CH4 — ADC2 channel 10 (PC0,   CONT_4)
 */
extern casper_adc_t *BSP_ADC_CONT[4];  /**< Array of 4 opaque ADC channel pointers. */

/* =========================================================================
 *  GPIO pin descriptors — built from main.h defines
 *
 *  All BSP_PIN_* are casper_pin_t values (port, pin) that can be passed
 *  directly to casper_gpio_write() / casper_gpio_read() / casper_gpio_toggle().
 * ======================================================================= */

/*
 * Helper macro: construct a casper_pin_t literal from CubeMX pin defines.
 * Port is cast to void* so no HAL types appear in this header.
 * Example: BSP_MAKE_PIN(GPIOC, GPIO_PIN_14)
 *
 * Callers that use BSP_PIN_* macros do NOT need to include main.h themselves.
 * The concrete values are resolved inside board_casper2.c where main.h IS
 * included.  Here we only forward-declare the extern casper_pin_t variables
 * for the pin sets that App/ code needs to pass to drivers.
 */

/* ── IMU (LSM6DSO32) ── */
extern casper_pin_t BSP_PIN_IMU_CS;    /**< SPI2 CS — PC14. */
extern casper_pin_t BSP_PIN_IMU_INT;   /**< SPI2 INT2 — PC15 (data-ready EXTI). */

/* ── Barometer (MS5611) ── */
extern casper_pin_t BSP_PIN_BARO_CS;   /**< SPI4 CS — PE11. */

/* ── High-G accel (ADXL372) ── */
extern casper_pin_t BSP_PIN_HIGHG_CS;  /**< SPI3 CS — PA15. */
extern casper_pin_t BSP_PIN_HIGHG_INT; /**< SPI3 INT — PD2. */

/* ── Radio (SX1276) ── */
extern casper_pin_t BSP_PIN_RADIO_CS;  /**< SPI1 CS — PB0. */
extern casper_pin_t BSP_PIN_RADIO_NRST;/**< Radio NRST — PC13. */
extern casper_pin_t BSP_PIN_RADIO_DIO0;/**< Radio DIO0/INT — PB1. */
extern casper_pin_t BSP_PIN_RADIO_DIO1;/**< Radio DIO1 — PD7. */
extern casper_pin_t BSP_PIN_RADIO_DIO2;/**< Radio DIO2 — PD6. */
extern casper_pin_t BSP_PIN_RADIO_DIO3;/**< Radio DIO3 — PA4. */
extern casper_pin_t BSP_PIN_RADIO_DIO4;/**< Radio DIO4 — PB12. */
extern casper_pin_t BSP_PIN_RADIO_DIO5;/**< Radio DIO5 — PB13. */

/* ── GPS (MAX-M10M) ── */
extern casper_pin_t BSP_PIN_GPS_NRST;      /**< GPS NRST — PE15. */
extern casper_pin_t BSP_PIN_GPS_TIMEPULSE; /**< GPS time pulse — PD4. */
extern casper_pin_t BSP_PIN_GPS_INT;       /**< I2C1 INT — PE0. */

/* ── Mag (MMC5983MA) ── */
extern casper_pin_t BSP_PIN_MAG_INT;   /**< I2C3 INT — PC8. */

/* ── Pyro fire outputs (MOSFET gates) ── */
extern casper_pin_t BSP_PIN_PY1;  /**< PD10. */
extern casper_pin_t BSP_PIN_PY2;  /**< PD9.  */
extern casper_pin_t BSP_PIN_PY3;  /**< PD8.  */
extern casper_pin_t BSP_PIN_PY4;  /**< PB15. */

/**
 * @brief Array of pyro fire-output pins, indexed [0..3] = CH1..CH4.
 *
 * Convenience aggregate for passing to casper_pyro_init / pyro_mgr_init.
 * Same order as BSP_PIN_PY1..4.
 */
extern casper_pin_t BSP_PIN_PY[4];

/* ── Continuity-detect LEDs ── */
extern casper_pin_t BSP_PIN_CONT_YN_1;  /**< PA10. */
extern casper_pin_t BSP_PIN_CONT_YN_2;  /**< PB14. */
extern casper_pin_t BSP_PIN_CONT_YN_3;  /**< PE8.  */
extern casper_pin_t BSP_PIN_CONT_YN_4;  /**< PE7.  */

/**
 * @brief Array of continuity-detect LED pins, indexed [0..3] = CH1..CH4.
 *
 * Convenience aggregate for passing to casper_pyro_init / pyro_mgr_init.
 * Same order as BSP_PIN_CONT_YN_1..4.
 */
extern casper_pin_t BSP_PIN_CONT_LED[4];

#endif /* BOARD_CASPER2_H */
