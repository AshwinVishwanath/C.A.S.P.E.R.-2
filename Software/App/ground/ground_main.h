/**
 * @file ground_main.h
 * @brief Ground station main application: init + tick loop.
 *
 * Manages radio RX, local sensor polling (MS5611 barometer, MAX-M10M GPS),
 * USB CDC forwarding, and GS status heartbeat.
 */

#ifndef APP_GROUND_GROUND_MAIN_H
#define APP_GROUND_GROUND_MAIN_H

#include "stm32h7xx_hal.h"
#include "ms5611.h"
#include "max_m10m.h"

/**
 * Initialize ground station subsystems.
 * @param hspi1  SPI1 handle for SX1276
 * @param baro   Pointer to MS5611 instance (already initialized)
 * @param gps    Pointer to MAX-M10M instance (already initialized)
 */
void ground_main_init(SPI_HandleTypeDef *hspi1, ms5611_t *baro, max_m10m_t *gps);

/**
 * Ground station main loop tick. Call from while(1).
 * Non-blocking: polls radio IRQ, sensors, CDC, outputs status.
 */
void ground_main_tick(void);

#endif /* APP_GROUND_GROUND_MAIN_H */
