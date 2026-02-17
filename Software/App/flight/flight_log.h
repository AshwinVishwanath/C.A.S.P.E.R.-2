#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_context.h"

/* ── Flash layout ───────────────────────────────────── */
#define FLIGHT_LOG_FLASH_BASE     0x00000000   /* start of flash for log data */
#define FLIGHT_LOG_FLASH_END      0x03F00000   /* 63 MB for log data */
#define FLIGHT_LOG_SUMMARY_BASE   0x03F00000   /* last 1 MB for summary */
#define FLIGHT_LOG_SUMMARY_END    0x04000000   /* end of 64 MB flash */

#define FLIGHT_LOG_SECTOR_SIZE    4096         /* W25Q512JV sector size */
#define FLIGHT_LOG_PAGE_SIZE      256          /* W25Q512JV page size */

/* Pre-erase during init: 32 sectors = 128KB, ~1.4s */
#define FLIGHT_LOG_PRE_ERASE_SECTORS  32

/* ── Log entry format (32 bytes, packed) ─────────────── */
/* [timestamp_us:4][fsm_state:1][flags:1][ax:2][ay:2][az:2]
   [gx:2][gy:2][gz:2][alt:2][vel:2][baro:2][tilt:2][batt:2][rsvd:4] = 32 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;        /* microsecond timestamp from DWT */
    uint8_t  fsm_state;           /* current FSM state */
    uint8_t  flags;               /* bit0: baro_updated, bit1: gps_valid, bit2-7: reserved */
    int16_t  accel_x_mg;          /* body accel X, milli-g */
    int16_t  accel_y_mg;          /* body accel Y, milli-g */
    int16_t  accel_z_mg;          /* body accel Z, milli-g */
    int16_t  gyro_x_mdps;        /* body gyro X, milli-dps (capped +/-32767) */
    int16_t  gyro_y_mdps;        /* body gyro Y, milli-dps */
    int16_t  gyro_z_mdps;        /* body gyro Z, milli-dps */
    int16_t  alt_dm;              /* EKF altitude, decimeters (alt_m * 10) */
    int16_t  vel_cmps;            /* EKF velocity, cm/s (vel_mps * 100) */
    int16_t  baro_alt_dm;         /* baro altitude AGL, decimeters */
    int16_t  tilt_cdeg;           /* tilt from vertical, centidegrees */
    uint16_t batt_mv;             /* battery voltage, millivolts */
    uint8_t  reserved[4];         /* alignment / future use */
} flight_log_entry_t;

_Static_assert(sizeof(flight_log_entry_t) == 32, "flight_log_entry_t must be 32 bytes");

/* ── Summary event entry ──────────────────────────────── */
#define FLIGHT_LOG_SUMMARY_MAX_LEN 128

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  type;                /* event type */
    uint8_t  len;                 /* string length */
    char     text[FLIGHT_LOG_SUMMARY_MAX_LEN]; /* null-terminated string */
} flight_log_summary_entry_t;

/* ── Log rate flags (bits 6-7 of flags field) ─────────── */
#define LOG_RATE_100HZ   0x00
#define LOG_RATE_50HZ    0x40
#define LOG_RATE_10HZ    0x80
#define LOG_RATE_STOPPED 0xC0

/* ── Ring buffer constants ──────────────────────────── */
#define FLIGHT_LOG_RING_SIZE      500    /* 5 seconds at 100 Hz */
#define FLIGHT_LOG_BUF_SIZE       128    /* entries per double-buffer half (one sector) */

/* ── API ─────────────────────────────────────────────── */

/**
 * Initialize flight log. Call during init after w25q512jv_init().
 * Pre-erases FLIGHT_LOG_PRE_ERASE_SECTORS sectors.
 * Enables DWT cycle counter for microsecond timestamps.
 */
void flight_log_init(void);

/**
 * Add an entry to the ring buffer (PAD state) or double buffer (flight).
 * Call at the logging rate (100 Hz after decimation).
 * In PAD state: writes to circular ring buffer.
 * In flight states: writes to double buffer, flushes to flash when full.
 * In LANDED state: ignored (logging stopped).
 */
void flight_log_write(const flight_log_entry_t *entry, fsm_state_t state);

/**
 * Commit ring buffer to flash. Call on launch detection.
 * Writes the last 5 seconds of PAD data to flash in chronological order.
 */
void flight_log_commit_ring_buffer(void);

/**
 * Flush any partial double buffer to flash. Call on landing.
 */
void flight_log_flush(void);

/**
 * Write a summary string event to the summary flash region.
 * Call on state transitions and key events.
 */
void flight_log_summary(uint32_t timestamp_ms, const char *msg);

/**
 * Erase all flight log data (both data and summary regions).
 * WARNING: Takes up to 400 seconds. Only call when USB-connected and idle.
 */
void flight_log_erase_all(void);

/**
 * Get current write address in flash.
 */
uint32_t flight_log_get_write_addr(void);

/**
 * Get total number of log entries written to flash.
 */
uint32_t flight_log_get_entry_count(void);

/**
 * Get number of dropped entries (buffer overflow).
 */
uint32_t flight_log_get_dropped(void);

/**
 * Check if logging is active (between commit and flush).
 */
bool flight_log_is_active(void);

/**
 * Get microsecond timestamp from DWT cycle counter.
 */
uint32_t flight_log_get_timestamp_us(void);

#endif /* FLIGHT_LOG_H */
