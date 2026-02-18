#ifndef FLIGHT_READOUT_H
#define FLIGHT_READOUT_H

#include <stdint.h>

/* Readout commands (single byte over CDC) */
#define READOUT_CMD_HR          0x01    /* Stream high-rate log */
#define READOUT_CMD_LR          0x02    /* Stream low-rate log */
#define READOUT_CMD_SUMMARY     0x03    /* Stream summary log */
#define READOUT_CMD_METADATA    0x04    /* Get log metadata */
#define READOUT_CMD_ERASE       0x05    /* Erase all logs */

/* Stream high-rate log data over USB CDC. Blocking. */
int flight_readout_stream_hr(void);

/* Stream low-rate log data over USB CDC. Blocking. */
int flight_readout_stream_lr(void);

/* Stream summary log over USB CDC. Blocking. */
int flight_readout_stream_summary(void);

/* Send metadata response (counts, addresses, CRC). */
int flight_readout_send_metadata(void);

/* Process a readout command byte. Returns 0 if handled, -1 if invalid. */
int flight_readout_handle_command(uint8_t cmd);

#endif /* FLIGHT_READOUT_H */
