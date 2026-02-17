#ifndef APP_TELEMETRY_TLM_MANAGER_H
#define APP_TELEMETRY_TLM_MANAGER_H

#include <stdint.h>
#include "tlm_types.h"

/**
 * Initialize telemetry manager.
 * Must be called after crc32_hw_init().
 */
void tlm_init(void);

/**
 * Build and send FC_MSG_FAST (19 bytes) over USB CDC at 10 Hz.
 *
 * @param state  Current telemetry state snapshot
 * @param pyro   Current pyro state
 * @param fsm    Current FSM state
 * @return       1 if packet was sent, 0 if timer not elapsed or TX busy
 */
int tlm_tick(const fc_telem_state_t *state, const pyro_state_t *pyro,
             fsm_state_t fsm);

/**
 * Build and send FC_MSG_GPS (12 bytes) over USB CDC.
 *
 * @param gps_state  GPS state
 * @return           1 if sent, 0 if TX busy
 */
int tlm_send_gps(const fc_gps_state_t *gps_state);

/**
 * Queue and send FC_MSG_EVENT (9 bytes) over USB CDC.
 * Events are sent immediately (not rate-limited).
 *
 * @param type  Event type (FC_EVT_*)
 * @param data  Event data (context-dependent)
 * @return      1 if sent, 0 if TX busy
 */
int tlm_queue_event(uint8_t type, uint16_t data);

/**
 * Get current sequence counter value.
 */
uint8_t tlm_get_seq(void);

/**
 * Send a raw COBS-framed response packet over USB CDC.
 * Used by command handlers to send ACK/NACK/handshake responses.
 *
 * @param data  Raw packet data (before COBS encoding)
 * @param len   Length of raw data
 * @return      1 if sent, 0 if TX busy
 */
int tlm_send_response(const uint8_t *data, int len);

#endif /* APP_TELEMETRY_TLM_MANAGER_H */
