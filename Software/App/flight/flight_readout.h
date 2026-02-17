#ifndef FLIGHT_READOUT_H
#define FLIGHT_READOUT_H

#include <stdint.h>

/**
 * Stream raw flight log entries over USB CDC.
 * Protocol:
 *   [magic "CASP":4][entry_count:4][entry_size:4][header_crc:4]
 *   [N * 32-byte entries]
 *   [data_crc:4]
 *
 * Reads entries from flash starting at FLIGHT_LOG_FLASH_BASE.
 * Entry count is determined by scanning for first entry with
 * timestamp_us == 0xFFFFFFFF.
 *
 * Blocking -- call only when system is in LANDED or PAD state
 * with USB connected.
 *
 * @return 0 on success, negative on error
 */
int flight_readout_stream_raw(void);

/**
 * Stream summary events over USB CDC.
 * Protocol:
 *   [magic "SUMM":4][total_size:4][data][crc:4]
 *
 * Reads from FLIGHT_LOG_SUMMARY_BASE until empty (0xFF).
 *
 * @return 0 on success, negative on error
 */
int flight_readout_stream_summary(void);

/**
 * Get number of valid log entries in flash.
 * Scans from FLIGHT_LOG_FLASH_BASE until first 0xFFFFFFFF timestamp.
 */
uint32_t flight_readout_get_entry_count(void);

#endif /* FLIGHT_READOUT_H */
