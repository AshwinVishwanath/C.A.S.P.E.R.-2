/**
 * @file ground_radio.h
 * @brief Ground station radio RX state machine: packet parsing,
 *        command relay, and profile switching.
 */

#ifndef APP_GROUND_GROUND_RADIO_H
#define APP_GROUND_GROUND_RADIO_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Profile switch FSM states */
typedef enum {
    GS_PROFILE_AWAITING_FIRST,   /* Before any packet — stay on A indefinitely */
    GS_PROFILE_A_ACTIVE,         /* Receiving packets on Profile A (SF7) */
    GS_PROFILE_B_ACTIVE          /* Switched to Profile B (SF8) — permanent */
} gs_profile_state_t;

/* Radio statistics */
typedef struct {
    uint16_t rx_pkt_count;       /* Total valid packets received */
    uint16_t rx_crc_fail;        /* CRC failures */
    int8_t   last_rssi;          /* dBm, from last valid packet */
    int8_t   last_snr;           /* dB, from last valid packet */
    uint8_t  current_profile;    /* 0=A, 1=B */
} gs_radio_stats_t;

/**
 * Initialize ground station radio: SX1276 in RX-continuous mode,
 * Profile A (SF7, BW250, CR4/5).
 * @param hspi  SPI1 handle
 * @return 0 on success, -1 on SX1276 init failure
 */
int ground_radio_init(SPI_HandleTypeDef *hspi);

/**
 * Handle a received packet from the SX1276 FIFO.
 * Called when DIO0 fires (RxDone). Reads FIFO, validates CRC,
 * parses message ID, outputs ASCII debug line via USB CDC.
 */
void ground_radio_on_rx(void);

/**
 * Profile switching tick. Call from main loop.
 * Tracks packet loss timer. Switches from Profile A to B
 * after 2 seconds of no valid packets (one-way, never switches back).
 * Only starts timing after first valid packet received.
 */
void ground_radio_profile_tick(void);

/**
 * Get current radio statistics (for GS status output).
 */
const gs_radio_stats_t *ground_radio_get_stats(void);

/**
 * Process an inbound command from USB CDC (MC -> GS -> FC).
 * Writes raw bytes to SX1276 TX FIFO and transmits.
 * Returns to RX-continuous after TX completes.
 * @param buf  Raw command packet bytes
 * @param len  Packet length
 * @return 0 on success, -1 if radio busy
 */
int ground_radio_send_cmd(const uint8_t *buf, uint8_t len);

/**
 * Check for TX completion and return to RX-continuous.
 * Called from ground_main_tick() each iteration.
 */
void ground_radio_check_tx_done(void);

#endif /* APP_GROUND_GROUND_RADIO_H */
