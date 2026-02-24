/**
 * @file radio_manager.h
 * @brief Radio state machine: TX scheduler, RX window, profile switching.
 *
 * Manages SX1276 LoRa radio for 10 Hz downlink telemetry and uplink
 * command reception. Builds packets identically to tlm_manager.c using
 * the same encoding scales from tlm_types.h.
 */

#ifndef APP_RADIO_RADIO_MANAGER_H
#define APP_RADIO_RADIO_MANAGER_H

#include "stm32h7xx_hal.h"
#include "tlm_types.h"
#include "casper_ekf.h"
#include "radio_config.h"
#include <stdint.h>

/* Initialize radio: reset SX1276, configure LoRa, apply Profile A */
int radio_manager_init(SPI_HandleTypeDef *hspi);

/* Non-blocking tick — call every superloop iteration (~833 Hz).
 * Manages TX scheduling, RX windows, IRQ processing, profile switching. */
void radio_manager_tick(const casper_ekf_t *ekf,
                        const fc_telem_state_t *tstate,
                        const pyro_state_t *pstate,
                        fsm_state_t fsm);

/* Queue a GPS packet for next available TX slot */
void radio_send_gps(const fc_gps_state_t *gps_state);

/* Queue an event packet for next available TX slot */
void radio_queue_event(uint8_t type, uint16_t data);

/* Queue a response (ACK/NACK) for immediate TX (highest priority) */
int radio_send_response(const uint8_t *buf, uint8_t len);

/* Check if radio initialized OK */
int radio_is_active(void);

#endif /* APP_RADIO_RADIO_MANAGER_H */
