/**
 * flight_log.c - Flight data logging to QSPI flash
 *
 * Architecture:
 *   PAD phase:    Ring buffer in AXI SRAM (last 5s at 100Hz)
 *   Launch:       Commit ring buffer to flash in chronological order
 *   Flight:       Double-buffered writes to flash (erase-ahead)
 *   Landing:      Flush partial buffer, write summary
 *
 * Flash layout:
 *   0x00000000 - 0x03EFFFFF  (63 MB) flight data log
 *   0x03F00000 - 0x03FFFFFF  (1 MB)  summary events
 */

#ifndef HOST_TEST
#include "stm32h7xx_hal.h"
#include "w25q512jv.h"
#endif

#include "flight_log.h"
#include <string.h>

/* ── Flash device handle (defined in main.c) ────────── */
#ifndef HOST_TEST
extern w25q512jv_t flash;
#endif

/* ── CPU clock for DWT timestamp conversion ─────────── */
#define SYSCLK_HZ  432000000UL

/* ── Buffers in AXI SRAM ────────────────────────────── */
#ifndef HOST_TEST
__attribute__((section(".axi_sram")))
static flight_log_entry_t s_ring_buffer[FLIGHT_LOG_RING_SIZE];   /* 16 KB */

__attribute__((section(".axi_sram")))
static flight_log_entry_t s_buf_a[FLIGHT_LOG_BUF_SIZE];          /* 4 KB */

__attribute__((section(".axi_sram")))
static flight_log_entry_t s_buf_b[FLIGHT_LOG_BUF_SIZE];          /* 4 KB */
#else
static flight_log_entry_t s_ring_buffer[FLIGHT_LOG_RING_SIZE];
static flight_log_entry_t s_buf_a[FLIGHT_LOG_BUF_SIZE];
static flight_log_entry_t s_buf_b[FLIGHT_LOG_BUF_SIZE];
#endif

/* ── State variables ────────────────────────────────── */
static uint32_t s_ring_head;          /* next write index in ring buffer */
static uint32_t s_ring_count;         /* total entries ever written to ring */

static flight_log_entry_t *s_active_buf;  /* current write buffer (a or b) */
static flight_log_entry_t *s_flush_buf;   /* buffer being flushed to flash */
static uint32_t s_active_idx;         /* next write index in active buffer */
static bool     s_flush_pending;      /* flush_buf has data to write */

static uint32_t s_flash_write_addr;   /* current flash write address */
static uint32_t s_summary_write_addr; /* current summary write address */
static uint32_t s_entry_count;        /* total entries written to flash */
static uint32_t s_dropped;            /* entries dropped due to buffer full */
static bool     s_committed;          /* true after ring buffer committed (launch) */
static bool     s_flushed;            /* true after final flush (landing) */
static uint32_t s_next_erase_addr;    /* next sector address to erase ahead */

/* ── Flash write helpers ────────────────────────────── */

/**
 * Erase sectors ahead as needed so that the region
 * [s_flash_write_addr, s_flash_write_addr + bytes) is erased.
 */
static void erase_ahead_if_needed(uint32_t bytes)
{
#ifndef HOST_TEST
    while (s_flash_write_addr + bytes > s_next_erase_addr) {
        if (s_next_erase_addr < FLIGHT_LOG_FLASH_END) {
            w25q512jv_erase_sector(&flash, s_next_erase_addr);
        }
        s_next_erase_addr += FLIGHT_LOG_SECTOR_SIZE;
    }
#else
    (void)bytes;
#endif
}

/**
 * Write a buffer of log entries to flash at s_flash_write_addr.
 * Advances s_flash_write_addr and s_entry_count.
 */
static void flush_buffer_to_flash(const flight_log_entry_t *buf, uint32_t count)
{
    if (count == 0) {
        return;
    }

    uint32_t bytes = count * sizeof(flight_log_entry_t);

    /* Bounds check: don't write past the data region */
    if (s_flash_write_addr + bytes > FLIGHT_LOG_FLASH_END) {
        s_dropped += count;
        return;
    }

    /* Erase ahead if needed */
    erase_ahead_if_needed(bytes);

    /* Write to flash */
#ifndef HOST_TEST
    w25q512jv_write(&flash, s_flash_write_addr, (const uint8_t *)buf, bytes);
#endif

    s_flash_write_addr += bytes;
    s_entry_count += count;
}

/* ── Public API ─────────────────────────────────────── */

void flight_log_init(void)
{
#ifndef HOST_TEST
    /* Enable DWT cycle counter for microsecond timestamps */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Pre-erase sectors for fast initial writes */
    for (uint32_t i = 0; i < FLIGHT_LOG_PRE_ERASE_SECTORS; i++) {
        w25q512jv_erase_sector(&flash, i * FLIGHT_LOG_SECTOR_SIZE);
    }
#endif

    /* Initialize state */
    s_ring_head = 0;
    s_ring_count = 0;
    s_active_buf = s_buf_a;
    s_flush_buf = s_buf_b;
    s_active_idx = 0;
    s_flush_pending = false;
    s_flash_write_addr = FLIGHT_LOG_FLASH_BASE;
    s_summary_write_addr = FLIGHT_LOG_SUMMARY_BASE;
    s_entry_count = 0;
    s_dropped = 0;
    s_committed = false;
    s_flushed = false;
    s_next_erase_addr = FLIGHT_LOG_PRE_ERASE_SECTORS * FLIGHT_LOG_SECTOR_SIZE;

    memset(s_ring_buffer, 0, sizeof(s_ring_buffer));
    memset(s_buf_a, 0, sizeof(s_buf_a));
    memset(s_buf_b, 0, sizeof(s_buf_b));
}

void flight_log_write(const flight_log_entry_t *entry, fsm_state_t state)
{
    if (s_flushed) {
        return; /* logging stopped after landing */
    }

    if (state == FSM_STATE_LANDED) {
        return; /* don't log in LANDED state */
    }

    if (!s_committed) {
        /* PAD phase: write to ring buffer */
        s_ring_buffer[s_ring_head] = *entry;
        s_ring_head = (s_ring_head + 1) % FLIGHT_LOG_RING_SIZE;
        if (s_ring_count < FLIGHT_LOG_RING_SIZE) {
            s_ring_count++;
        }
        /* s_ring_count saturates at RING_SIZE to indicate buffer is full/wrapping */
        return;
    }

    /* Flight phase: write to double buffer */

    /* If flush is still pending, both buffers are in use.
     * We can still write to active_buf if it has room. */
    if (s_active_idx >= FLIGHT_LOG_BUF_SIZE) {
        /* Active buffer is full. We need to swap. */
        if (s_flush_pending) {
            /* Both buffers full, drop this entry */
            s_dropped++;
            return;
        }

        /* Swap buffers: active becomes flush target, flush becomes active */
        s_flush_pending = true;
        flight_log_entry_t *tmp = s_active_buf;
        s_active_buf = s_flush_buf;
        s_flush_buf = tmp;

        /* Write the full flush buffer to flash */
        flush_buffer_to_flash(s_flush_buf, FLIGHT_LOG_BUF_SIZE);
        s_flush_pending = false;

        s_active_idx = 0;
    }

    s_active_buf[s_active_idx] = *entry;
    s_active_idx++;
}

void flight_log_commit_ring_buffer(void)
{
    if (s_committed) {
        return; /* already committed */
    }

    s_committed = true;

    if (s_ring_count == 0) {
        return; /* nothing to commit */
    }

    /* Determine how many entries and where the oldest one is */
    uint32_t entries_to_write;
    uint32_t start_idx;

    if (s_ring_count < FLIGHT_LOG_RING_SIZE) {
        /* Ring never wrapped: entries are 0..ring_count-1 */
        entries_to_write = s_ring_count;
        start_idx = 0;
    } else {
        /* Ring wrapped: oldest entry is at s_ring_head */
        entries_to_write = FLIGHT_LOG_RING_SIZE;
        start_idx = s_ring_head;
    }

    /* Write entries in chronological order to flash.
     * We write in two segments if the ring has wrapped. */
    if (start_idx == 0) {
        /* Contiguous: just write everything */
        flush_buffer_to_flash(s_ring_buffer, entries_to_write);
    } else {
        /* Part 1: from start_idx to end of ring */
        uint32_t first_chunk = FLIGHT_LOG_RING_SIZE - start_idx;
        flush_buffer_to_flash(&s_ring_buffer[start_idx], first_chunk);

        /* Part 2: from 0 to start_idx (the newer entries before head) */
        if (start_idx > 0) {
            flush_buffer_to_flash(&s_ring_buffer[0], start_idx);
        }
    }

    /* Reset double buffer state for flight phase writes */
    s_active_buf = s_buf_a;
    s_flush_buf = s_buf_b;
    s_active_idx = 0;
    s_flush_pending = false;
}

void flight_log_flush(void)
{
    if (s_flushed) {
        return;
    }

    /* Flush any partial active buffer */
    if (s_active_idx > 0 && s_committed) {
        flush_buffer_to_flash(s_active_buf, s_active_idx);
        s_active_idx = 0;
    }

    /* If a flush was pending (shouldn't normally happen at landing), write it too */
    if (s_flush_pending) {
        flush_buffer_to_flash(s_flush_buf, FLIGHT_LOG_BUF_SIZE);
        s_flush_pending = false;
    }

    s_flushed = true;
}

void flight_log_summary(uint32_t timestamp_ms, const char *msg)
{
    if (msg == NULL) {
        return;
    }

    uint32_t msg_len = 0;
    while (msg[msg_len] != '\0' && msg_len < FLIGHT_LOG_SUMMARY_MAX_LEN - 1) {
        msg_len++;
    }

    /* Build the summary entry on the stack */
    flight_log_summary_entry_t entry;
    memset(&entry, 0xFF, sizeof(entry)); /* flash-friendly default */
    entry.timestamp_ms = timestamp_ms;
    entry.type = 0; /* generic event */
    entry.len = (uint8_t)msg_len;
    memset(entry.text, 0, sizeof(entry.text));
    memcpy(entry.text, msg, msg_len);
    entry.text[msg_len] = '\0';

    /* Actual bytes to write: header (6 bytes) + msg_len + 1 (null terminator) */
    uint32_t write_len = sizeof(entry); /* fixed-size structure */

    /* Bounds check */
    if (s_summary_write_addr + write_len > FLIGHT_LOG_SUMMARY_END) {
        return; /* summary region full */
    }

#ifndef HOST_TEST
    /* Erase sector if needed (summary region starts at 0x03F00000) */
    uint32_t current_sector = s_summary_write_addr & ~(FLIGHT_LOG_SECTOR_SIZE - 1U);
    uint32_t end_addr = s_summary_write_addr + write_len;
    uint32_t end_sector = (end_addr - 1) & ~(FLIGHT_LOG_SECTOR_SIZE - 1U);

    /* Erase any sectors we're about to write into if at sector boundary */
    if (s_summary_write_addr == current_sector) {
        w25q512jv_erase_sector(&flash, current_sector);
    }
    if (end_sector != current_sector && end_addr > end_sector) {
        w25q512jv_erase_sector(&flash, end_sector);
    }

    w25q512jv_write(&flash, s_summary_write_addr,
                    (const uint8_t *)&entry, write_len);
#endif

    s_summary_write_addr += write_len;
}

void flight_log_erase_all(void)
{
#ifndef HOST_TEST
    w25q512jv_erase_chip(&flash);
#endif
}

uint32_t flight_log_get_write_addr(void)
{
    return s_flash_write_addr;
}

uint32_t flight_log_get_entry_count(void)
{
    return s_entry_count;
}

uint32_t flight_log_get_dropped(void)
{
    return s_dropped;
}

bool flight_log_is_active(void)
{
    return s_committed && !s_flushed;
}

uint32_t flight_log_get_timestamp_us(void)
{
#ifndef HOST_TEST
    /* DWT->CYCCNT counts at SYSCLK (432 MHz).
     * us = cycles / (SYSCLK / 1000000) = cycles / 432 */
    return DWT->CYCCNT / (SYSCLK_HZ / 1000000UL);
#else
    return 0;
#endif
}
