/**
 * @file  app_main.h
 * @brief Portable application entry points.
 *
 * Declares the two functions that main.c calls after board bring-up:
 *   app_init() — driver/subsystem initialisation (USER CODE BEGIN 2 content)
 *   app_tick() — superloop body (USER CODE BEGIN 3 content)
 *
 * Also pulls in app_globals.h so any translation unit that includes this
 * header can reach the extern driver-instance declarations (baro, imu, flash,
 * etc.) without a separate include.
 *
 * HARD RULE: This header must NOT include stm32h7xx_hal.h or main.h.
 * All board coupling lives in board_casper2.h / board_casper2.c.
 */

#ifndef APP_FLIGHT_APP_MAIN_H
#define APP_FLIGHT_APP_MAIN_H

#include "app_globals.h"   /* extern baro, imu, flash, ekf, att, gps, mag, logger */

/**
 * @brief Portable application initialisation.
 *
 * Called once from main() after all MX_*_Init() peripherals are ready and
 * casper_board_early_init() has run.  Initialises every driver and subsystem
 * via the casper_port seam (never touches HAL directly).
 *
 * Covers: USB enumeration wait, sensor init (MS5611/LSM6DSO32/ADXL372/
 * MAX-M10M/MMC5983MA/W25Q512JV), nav stack (EKF + attitude), telemetry,
 * FSM, pyro, buzzer, radio, logger, FATFS mount, calibration-mode dispatch
 * (MAG_CAL / MAG_VAL / GYRO_TEMP_CAL / GPS_TEST), EXTI enable.
 * Ground-station branch (BUILD_TARGET_GROUND) is also handled here.
 *
 * Preserves all #ifdef guards from the original USER CODE BEGIN 2 block.
 */
void app_init(void);

/**
 * @brief Portable superloop body.
 *
 * Called from while(1) in main() on every iteration.  Dispatches to the
 * correct runtime mode (flight loop, calibration tick, GPS test, MSC pyro
 * continuity, or ground station tick) via compile-time #ifdef guards.
 * Mirrors the original USER CODE BEGIN 3 block exactly.
 */
void app_tick(void);

#endif /* APP_FLIGHT_APP_MAIN_H */
