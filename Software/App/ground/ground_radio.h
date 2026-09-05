/* ============================================================
 *  TIER:     GROUND-STATION
 *  MODULE:   Ground Radio
 *  SUMMARY:  RX-continuous parser, ASCII output, command relay.
 * ============================================================ */
/**
 * @file ground_radio.h
 * @brief Ground station radio RX state machine: packet parsing,
 *        command relay, and profile switching.
 */

#ifndef APP_GROUND_GROUND_RADIO_H
#define APP_GROUND_GROUND_RADIO_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Profile switch FSM states. Bidirectional (see ground_radio_profile_tick):
 * a one-way A->B commit could not recover from an FC power-cycle without a
 * GS power-cycle, since a rebooted FC always comes back up on Profile A. */
typedef enum {
    GS_PROFILE_A_ACTIVE,         /* Listening on Profile A (SF7) */
    GS_PROFILE_B_ACTIVE          /* Listening on Profile B (SF8) */
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
 * Cycles between Profile A and B after 2 seconds of no valid packets on
 * whichever is currently active. Bidirectional: the FC's own profile
 * switch (altitude/velocity driven) is one-way, but the GS has no such
 * signal -- silence alone can't tell "FC switched to B for range" apart
 * from "FC power-cycled and came back up on A" -- so the GS keeps trying
 * both until one of them starts producing valid packets, then stays there
 * (the timer only fires on continued silence).
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

/**
 * @return non-zero while a TX is in flight (awaiting TxDone).
 * DIO0 is shared RxDone/TxDone; the main loop uses this to route a DIO0
 * edge to check_tx_done() rather than on_rx() while transmitting.
 */
int ground_radio_tx_pending(void);

/**
 * COBS-encode raw bytes and transmit via USB CDC.
 * Pattern mirrors tlm_manager.c: cobs_encode -> append 0x00 -> CDC_Transmit_FS.
 * Always compiled; reserved for callers that need to send binary GS packets
 * (e.g. ground_main.c 0x13 status heartbeat when GS_OUTPUT=COBS).
 * @param raw  Raw packet bytes (must not alias the internal encode buffer)
 * @param len  Number of raw bytes (max SIZE_GS_MSG_TELEM = 39)
 * @return 1 on success, 0 on COBS encode error or USB busy
 */
int ground_radio_cobs_send(const uint8_t *raw, int len);

#endif /* APP_GROUND_GROUND_RADIO_H */
