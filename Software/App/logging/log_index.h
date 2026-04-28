/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Log Index
 *  SUMMARY:  On-flash index of log records; supports flash dump protocol.
 * ============================================================ */
#pragma once

#include <stdbool.h>
#include "log_types.h"
#include "w25q512jv.h"

/**
 * Flight index manager.
 *
 * Reads/writes the flight index table at FLASH_INDEX_BASE.
 * Uses blocking flash operations (called during init and at landing only).
 */
typedef struct {
    w25q512jv_t *flash;
    uint16_t     flight_count;     /* total flights in index              */
    uint16_t     current_flight;   /* current flight ID (1-based)         */
    uint32_t     hr_next_addr;     /* next write address for HR pool      */
    uint32_t     lr_next_addr;     /* next write address for LR pool      */
    uint32_t     adxl_next_addr;   /* next write address for ADXL pool    */
} log_index_t;

/**
 * Initialize index: scan flash for existing flights, compute next addresses.
 * @return true on success
 */
bool log_index_init(log_index_t *idx, w25q512jv_t *flash);

/**
 * Start a new flight: write index entry with start addresses, dirty end fields.
 * @return true on success
 */
bool log_index_start_flight(log_index_t *idx, uint32_t start_tick);

/**
 * End current flight: update index entry with actual end addresses.
 * Uses NOR flash 1->0 trick to update without erasing.
 * @return true on success
 */
bool log_index_end_flight(log_index_t *idx, uint32_t end_tick,
                          uint32_t hr_end, uint32_t lr_end, uint32_t adxl_end);

/**
 * Get current write addresses for all three pools.
 */
void log_index_get_write_addrs(const log_index_t *idx,
                               uint32_t *hr, uint32_t *lr, uint32_t *adxl);
