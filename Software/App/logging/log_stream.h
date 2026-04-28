/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Log Stream
 *  SUMMARY:  Per-stream ring buffer + flash-write state machine.
 * ============================================================ */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "log_types.h"

/**
 * Generic ring-buffered log stream with flash drain.
 *
 * Uses absolute page counters (head_page, drain_page) for simple
 * comparison. Actual ring index = counter % ring_pages.
 *
 * States:
 *   LOG_IDLE  — not active, push() is a no-op
 *   LOG_PAD   — pre-launch: ring wraps freely, no flash writes
 *   LOG_DRAIN — post-launch: ring feeds flash writer, no wrap
 *   LOG_FINAL — finalized, no more pushes
 */
typedef struct {
    /* Ring buffer (allocated in AXI SRAM) */
    uint8_t    *ring_buf;          /* pointer to page-aligned buffer         */
    uint16_t    ring_pages;        /* total pages in ring                    */
    uint16_t    prelaunch_pages;   /* pages of pre-launch data to retain     */

    /* Producer (record writer) */
    uint32_t    head_page;         /* absolute page counter (producer)       */
    uint8_t     rec_idx;           /* slot within current page (0..rpp-1)    */
    uint8_t     recs_per_page;     /* records per page                       */
    uint8_t     rec_size;          /* bytes per record                       */

    /* Consumer (flash writer) */
    uint32_t    drain_page;        /* absolute page counter (consumer)       */

    /* Flash region */
    uint32_t    flash_base;        /* start of flash pool                    */
    uint32_t    flash_addr;        /* current flash write pointer            */
    uint32_t    flash_end;         /* end of flash pool                      */
    uint32_t    erased_up_to;      /* erase frontier (next addr to erase)    */

    /* State */
    log_state_t state;
    uint16_t    drop_count;        /* records lost to ring overrun           */
    uint16_t    err_count;         /* flash write errors                     */
    uint32_t    records_written;   /* total records pushed into ring         */
} log_stream_t;

/**
 * Initialize a log stream. Sets all fields, state = LOG_IDLE.
 */
void log_stream_init(log_stream_t *s, uint8_t *ring_mem,
                     uint16_t ring_pages, uint16_t prelaunch_pages,
                     uint8_t rec_size, uint8_t recs_per_page,
                     uint32_t flash_base, uint32_t flash_end);

/**
 * Transition IDLE -> PAD. Resets ring counters.
 */
void log_stream_start(log_stream_t *s);

/**
 * Push one record into the ring buffer.
 * No-op if state is IDLE or FINAL.
 */
void log_stream_push(log_stream_t *s, const void *record);

/**
 * Transition PAD -> DRAIN. Sets drain_page to preserve
 * the last prelaunch_pages of data.
 */
void log_stream_launch(log_stream_t *s);

/**
 * Check if there is a complete page ready for flash write.
 */
bool log_stream_has_page(const log_stream_t *s);

/**
 * Get pointer to the next page to write to flash.
 * Valid only when log_stream_has_page() returns true.
 */
const uint8_t *log_stream_peek_page(const log_stream_t *s);

/**
 * Get the flash address for the next page write.
 */
uint32_t log_stream_flash_addr(const log_stream_t *s);

/**
 * Acknowledge successful flash write. Advances drain pointer.
 */
void log_stream_page_done(log_stream_t *s);

/**
 * Acknowledge failed flash write. Advances drain pointer, increments err_count.
 */
void log_stream_page_fail(log_stream_t *s);

/**
 * Finalize stream: pad partial page with 0xFF, advance head, set LOG_FINAL.
 */
void log_stream_finalize(log_stream_t *s);
