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

/* =========================================================================
 *  Early board bring-up — moved from main.c USER CODE BEGIN Init / SysInit
 * ======================================================================= */

/**
 * @brief Early board bring-up: raw-register LED GPIO setup + DWT counter.
 *
 * Called from main() immediately after HAL_Init() (before SystemClock_Config)
 * for the LED GPIO raw-register setup, and again (conceptually — actually
 * inlined in main USER CODE SysInit) for DWT.  In the refactored flow main()
 * calls this once after HAL_Init() to cover both the GPIO milestone setup and
 * the DWT cycle-counter enable.
 *
 * Contains only register/direct-hardware operations that must happen before
 * MX_*_Init() runs.  No sensor or app logic here.
 */
void casper_board_early_init(void);

/* =========================================================================
 *  Board helpers called from app_main.c during app_init()
 *
 *  These wrap HAL or pin-level operations that app_main.c cannot do itself
 *  (it must not include main.h or stm32h7xx_hal.h).  Each helper is a thin
 *  wrapper around the exact HAL calls that existed in main.c USER CODE.
 * ======================================================================= */

/**
 * @brief Return a void* to hspi1 (SPI_HandleTypeDef*).
 *
 * Used by app_init() to pass to radio_manager_init() and ground_main_init(),
 * both of which accept the handle as void* to keep HAL types out of their
 * headers.  app_main.c calls this instead of referencing &hspi1 directly.
 */
void *casper_board_spi1_handle(void);

/**
 * @brief Close PC2 analog switch + reconfigure PC2 as AF5 (SPI2 MISO).
 *
 * HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE)
 * followed by HAL_GPIO_Init to AF5.  Must run after MS5611 init, before
 * LSM6DSO32 init.  Originally in main.c USER CODE BEGIN 2.
 */
void casper_board_fix_pc2_miso(void);

/**
 * @brief Reconfigure I2C_3_INT (PC8) from push-pull output to EXTI rising.
 *
 * CubeMX sets PC8 as an output; it must be an EXTI rising-edge input for the
 * MMC5983MA data-ready interrupt.  Originally in main.c USER CODE BEGIN 2
 * just before mmc5983ma_init().
 */
void casper_board_fix_i2c3_int(void);

/**
 * @brief Enable NVIC for EXTI15_10 (PC15 = LSM6DSO32 INT2), clear pending.
 *
 * Must be called after all sensor init completes to avoid an ISR flood from
 * INT2 already being asserted.  Originally in main.c USER CODE BEGIN 2 at
 * the end of the flight init block.
 */
void casper_board_exti_enable_imu_int2(void);

/**
 * @brief Enable EXTI1 NVIC line for SX1276 DIO0 (PB1).
 *
 * Ground-station build only: enables the interrupt line for radio DIO0.
 * Originally in main.c USER CODE BEGIN 2 (ground branch).
 */
void casper_board_exti_enable_dio0(void);

/**
 * @brief Enable EXTI9_5 NVIC line for SX1276 DIO1 (PD7).
 *
 * Ground-station build only: enables the interrupt line for radio DIO1.
 * Originally in main.c USER CODE BEGIN 2 (ground branch).
 */
void casper_board_exti_enable_dio1(void);

/**
 * @brief Reconfigure all four pyro output pins as floating inputs.
 *
 * Ground-station build safety measure: the GS has no pyros so the fire-output
 * MOSFET gates are put into high-Z INPUT/PULLDOWN to prevent any accidental
 * assertion.  Originally in main.c USER CODE BEGIN 2 (ground branch).
 */
void casper_board_pyro_safe_mode(void);

#endif /* BOARD_CASPER2_H */
