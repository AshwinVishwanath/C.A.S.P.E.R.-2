/**
 * flight_log.c - Dual-rate flight data logging to QSPI flash
 *
 * Architecture:
 *   PAD phase:    Separate ring buffers for HR (5s @ 250Hz) and LR (10s @ 10Hz)
 *   Launch:       Commit both ring buffers to flash in chronological order
 *   Flight:       Double-buffered sector writes with erase-ahead
 *   Landing:      Flush partial buffers, write final summary
 *
 * Flash layout:
 *   0x00000000 - 0x037FFFFF  (56 MB) high-rate log
 *   0x03800000 - 0x03DFFFFF  (6 MB)  low-rate log
 *   0x03E00000 - 0x03EFFFFF  (1 MB)  summary events
 *   0x03F00000 - 0x03F00FFF  (4 KB)  config storage
 */

#ifndef HOST_TEST
#include "stm32h7xx_hal.h"
#include "w25q512jv.h"
#endif

#include "flight_log.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ── Flash device handle (defined in main.c) ────────────────────── */
#ifndef HOST_TEST
extern w25q512jv_t flash;
#endif

/* ── CPU clock for DWT timestamp conversion ──────────────────────── */
#define SYSCLK_HZ  432000000UL

/* ── Ring buffers in AXI SRAM ────────────────────────────────────── */
#ifndef HOST_TEST

__attribute__((section(".axi_sram")))
static highrate_entry_t hr_ring[HR_RING_ENTRIES];    /* 1250 * 64 = 80 KB */

__attribute__((section(".axi_sram")))
static lowrate_entry_t  lr_ring[LR_RING_ENTRIES];    /* 100 * 64 = 6.4 KB */

#else

static highrate_entry_t hr_ring[HR_RING_ENTRIES];
static lowrate_entry_t  lr_ring[LR_RING_ENTRIES];

#endif

static uint32_t hr_ring_head  = 0;
static uint32_t hr_ring_count = 0;

static uint32_t lr_ring_head  = 0;
static uint32_t lr_ring_count = 0;

/* ── Double buffers (one sector each = 64 entries = 4096 bytes) ─── */
static highrate_entry_t hr_buf_a[HR_ENTRIES_PER_SECTOR];
static highrate_entry_t hr_buf_b[HR_ENTRIES_PER_SECTOR];
static highrate_entry_t *hr_active = hr_buf_a;
static highrate_entry_t *hr_flush  = hr_buf_b;
static uint32_t hr_buf_idx     = 0;
static bool     hr_flush_pending = false;

static lowrate_entry_t lr_buf_a[LR_ENTRIES_PER_SECTOR];
static lowrate_entry_t lr_buf_b[LR_ENTRIES_PER_SECTOR];
static lowrate_entry_t *lr_active = lr_buf_a;
static lowrate_entry_t *lr_flush  = lr_buf_b;
static uint32_t lr_buf_idx     = 0;
static bool     lr_flush_pending = false;

/* ── Write pointers & counters ───────────────────────────────────── */
static uint32_t hr_write_addr      = FLASH_HR_START;
static uint32_t lr_write_addr      = FLASH_LR_START;
static uint32_t summary_write_addr = FLASH_SUMMARY_START;

static uint32_t hr_entry_count = 0;
static uint32_t lr_entry_count = 0;
static uint32_t hr_dropped     = 0;
static uint32_t lr_dropped     = 0;

static bool logging_active = false;

/* ── Erase-ahead tracking ────────────────────────────────────────── */
static uint32_t hr_erased_up_to      = FLASH_HR_START;
static uint32_t lr_erased_up_to      = FLASH_LR_START;
static uint32_t summary_erased_up_to = FLASH_SUMMARY_START;

/* ── Internal helpers ────────────────────────────────────────────── */

static void ensure_erased(uint32_t addr, uint32_t *erased_up_to)
{
#ifndef HOST_TEST
    while (*erased_up_to <= addr) {
        w25q512jv_erase_sector(&flash, *erased_up_to);
        *erased_up_to += FLOG_SECTOR_SIZE;
    }
#else
    (void)addr;
    (void)erased_up_to;
#endif
}

/* ── Public API ──────────────────────────────────────────────────── */

int flight_log_init(void)
{
#ifndef HOST_TEST
    /* Enable DWT cycle counter for microsecond timestamps */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
#endif

    /* Zero state */
    hr_ring_head  = 0;
    hr_ring_count = 0;
    lr_ring_head  = 0;
    lr_ring_count = 0;

    hr_active = hr_buf_a;
    hr_flush  = hr_buf_b;
    hr_buf_idx = 0;
    hr_flush_pending = false;

    lr_active = lr_buf_a;
    lr_flush  = lr_buf_b;
    lr_buf_idx = 0;
    lr_flush_pending = false;

    hr_write_addr      = FLASH_HR_START;
    lr_write_addr      = FLASH_LR_START;
    summary_write_addr = FLASH_SUMMARY_START;

    hr_entry_count = 0;
    lr_entry_count = 0;
    hr_dropped     = 0;
    lr_dropped     = 0;

    logging_active = false;

    hr_erased_up_to      = FLASH_HR_START;
    lr_erased_up_to      = FLASH_LR_START;
    summary_erased_up_to = FLASH_SUMMARY_START;

    memset(hr_ring, 0, sizeof(hr_ring));
    memset(lr_ring, 0, sizeof(lr_ring));
    memset(hr_buf_a, 0, sizeof(hr_buf_a));
    memset(hr_buf_b, 0, sizeof(hr_buf_b));
    memset(lr_buf_a, 0, sizeof(lr_buf_a));
    memset(lr_buf_b, 0, sizeof(lr_buf_b));

    return 0;
}

void flight_log_write_hr(const highrate_entry_t *entry, fsm_state_t state)
{
    if (state == FSM_STATE_LANDED) return;
    if (hr_write_addr >= FLASH_HR_END) return;

    if (!logging_active) {
        /* PAD phase: circular ring buffer */
        hr_ring[hr_ring_head] = *entry;
        hr_ring_head = (hr_ring_head + 1) % HR_RING_ENTRIES;
        if (hr_ring_count < HR_RING_ENTRIES) {
            hr_ring_count++;
        }
        return;
    }

    /* Flight phase: double-buffered sector writes */
    hr_active[hr_buf_idx] = *entry;
    hr_buf_idx++;

    if (hr_buf_idx >= HR_ENTRIES_PER_SECTOR) {
        /* Sector full — swap buffers */
        if (hr_flush_pending) {
            /* Previous flush not yet processed — drop the sector */
            hr_dropped += HR_ENTRIES_PER_SECTOR;
        }

        hr_flush_pending = true;

        highrate_entry_t *tmp = hr_active;
        hr_active = hr_flush;
        hr_flush  = tmp;
        hr_buf_idx = 0;
    }
}

void flight_log_write_lr(const lowrate_entry_t *entry, fsm_state_t state)
{
    if (state == FSM_STATE_LANDED) return;
    if (lr_write_addr >= FLASH_LR_END) return;

    if (!logging_active) {
        /* PAD phase: circular ring buffer */
        lr_ring[lr_ring_head] = *entry;
        lr_ring_head = (lr_ring_head + 1) % LR_RING_ENTRIES;
        if (lr_ring_count < LR_RING_ENTRIES) {
            lr_ring_count++;
        }
        return;
    }

    /* Flight phase: double-buffered sector writes */
    lr_active[lr_buf_idx] = *entry;
    lr_buf_idx++;

    if (lr_buf_idx >= LR_ENTRIES_PER_SECTOR) {
        if (lr_flush_pending) {
            lr_dropped += LR_ENTRIES_PER_SECTOR;
        }

        lr_flush_pending = true;

        lowrate_entry_t *tmp = lr_active;
        lr_active = lr_flush;
        lr_flush  = tmp;
        lr_buf_idx = 0;
    }
}

void flight_log_commit_ring_buffers(void)
{
    if (logging_active) return;

    /* ── Commit HR ring ─────────────────────────────────────────── */
    if (hr_ring_count > 0) {
        uint32_t entries = hr_ring_count;
        uint32_t start_idx;

        if (hr_ring_count < HR_RING_ENTRIES) {
            start_idx = 0;
        } else {
            entries = HR_RING_ENTRIES;
            start_idx = hr_ring_head; /* oldest entry */
        }

        /* Write page-by-page in chronological order.
         * Accumulate 4 entries per page, then write. */
        uint8_t page_buf[FLOG_PAGE_SIZE];
        uint32_t page_count = 0;
        uint32_t written = 0;

        for (uint32_t i = 0; i < entries; i++) {
            uint32_t idx = (start_idx + i) % HR_RING_ENTRIES;
            memcpy(page_buf + (page_count * sizeof(highrate_entry_t)),
                   &hr_ring[idx], sizeof(highrate_entry_t));
            page_count++;

            if (page_count == HR_ENTRIES_PER_PAGE) {
                ensure_erased(hr_write_addr, &hr_erased_up_to);
#ifndef HOST_TEST
                w25q512jv_write(&flash, hr_write_addr, page_buf, FLOG_PAGE_SIZE);
#endif
                hr_write_addr += FLOG_PAGE_SIZE;
                written += page_count;
                page_count = 0;
            }
        }

        /* Write any remaining partial page */
        if (page_count > 0) {
            /* Pad remainder with 0xFF */
            memset(page_buf + (page_count * sizeof(highrate_entry_t)), 0xFF,
                   FLOG_PAGE_SIZE - (page_count * sizeof(highrate_entry_t)));
            ensure_erased(hr_write_addr, &hr_erased_up_to);
#ifndef HOST_TEST
            w25q512jv_write(&flash, hr_write_addr, page_buf, FLOG_PAGE_SIZE);
#endif
            hr_write_addr += FLOG_PAGE_SIZE;
            written += page_count;
        }

        hr_entry_count += written;
    }

    /* ── Commit LR ring ─────────────────────────────────────────── */
    if (lr_ring_count > 0) {
        uint32_t entries = lr_ring_count;
        uint32_t start_idx;

        if (lr_ring_count < LR_RING_ENTRIES) {
            start_idx = 0;
        } else {
            entries = LR_RING_ENTRIES;
            start_idx = lr_ring_head;
        }

        uint8_t page_buf[FLOG_PAGE_SIZE];
        uint32_t page_count = 0;
        uint32_t written = 0;

        for (uint32_t i = 0; i < entries; i++) {
            uint32_t idx = (start_idx + i) % LR_RING_ENTRIES;
            memcpy(page_buf + (page_count * sizeof(lowrate_entry_t)),
                   &lr_ring[idx], sizeof(lowrate_entry_t));
            page_count++;

            if (page_count == LR_ENTRIES_PER_PAGE) {
                ensure_erased(lr_write_addr, &lr_erased_up_to);
#ifndef HOST_TEST
                w25q512jv_write(&flash, lr_write_addr, page_buf, FLOG_PAGE_SIZE);
#endif
                lr_write_addr += FLOG_PAGE_SIZE;
                written += page_count;
                page_count = 0;
            }
        }

        if (page_count > 0) {
            memset(page_buf + (page_count * sizeof(lowrate_entry_t)), 0xFF,
                   FLOG_PAGE_SIZE - (page_count * sizeof(lowrate_entry_t)));
            ensure_erased(lr_write_addr, &lr_erased_up_to);
#ifndef HOST_TEST
            w25q512jv_write(&flash, lr_write_addr, page_buf, FLOG_PAGE_SIZE);
#endif
            lr_write_addr += FLOG_PAGE_SIZE;
            written += page_count;
        }

        lr_entry_count += written;
    }

    /* Erase one sector ahead for both streams */
    ensure_erased(hr_write_addr, &hr_erased_up_to);
    ensure_erased(lr_write_addr, &lr_erased_up_to);

    logging_active = true;
}

int flight_log_process(void)
{
    int pages_written = 0;

    /* ── Flush HR double buffer ──────────────────────────────────── */
    if (hr_flush_pending) {
        if (hr_write_addr + FLOG_SECTOR_SIZE <= FLASH_HR_END) {
            ensure_erased(hr_write_addr, &hr_erased_up_to);

#ifndef HOST_TEST
            /* Write one sector = 16 pages */
            const uint8_t *src = (const uint8_t *)hr_flush;
            for (uint32_t p = 0; p < (FLOG_SECTOR_SIZE / FLOG_PAGE_SIZE); p++) {
                w25q512jv_write(&flash, hr_write_addr + (p * FLOG_PAGE_SIZE),
                                src + (p * FLOG_PAGE_SIZE), FLOG_PAGE_SIZE);
            }
#endif
            hr_entry_count += HR_ENTRIES_PER_SECTOR;
            hr_write_addr  += FLOG_SECTOR_SIZE;
            pages_written  += (FLOG_SECTOR_SIZE / FLOG_PAGE_SIZE);

            /* Erase next sector ahead */
            ensure_erased(hr_write_addr, &hr_erased_up_to);
        } else {
            hr_dropped += HR_ENTRIES_PER_SECTOR;
        }

        hr_flush_pending = false;
    }

    /* ── Flush LR double buffer ──────────────────────────────────── */
    if (lr_flush_pending) {
        if (lr_write_addr + FLOG_SECTOR_SIZE <= FLASH_LR_END) {
            ensure_erased(lr_write_addr, &lr_erased_up_to);

#ifndef HOST_TEST
            const uint8_t *src = (const uint8_t *)lr_flush;
            for (uint32_t p = 0; p < (FLOG_SECTOR_SIZE / FLOG_PAGE_SIZE); p++) {
                w25q512jv_write(&flash, lr_write_addr + (p * FLOG_PAGE_SIZE),
                                src + (p * FLOG_PAGE_SIZE), FLOG_PAGE_SIZE);
            }
#endif
            lr_entry_count += LR_ENTRIES_PER_SECTOR;
            lr_write_addr  += FLOG_SECTOR_SIZE;
            pages_written  += (FLOG_SECTOR_SIZE / FLOG_PAGE_SIZE);

            ensure_erased(lr_write_addr, &lr_erased_up_to);
        } else {
            lr_dropped += LR_ENTRIES_PER_SECTOR;
        }

        lr_flush_pending = false;
    }

    return pages_written;
}

void flight_log_summary(uint32_t timestamp_ms, const char *fmt, ...)
{
    if (fmt == NULL) return;
    if (summary_write_addr >= FLASH_SUMMARY_END) return;

    /* Format the message */
    char msg_buf[251];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(msg_buf, sizeof(msg_buf), fmt, ap);
    va_end(ap);

    if (len < 0) return;
    if (len > 250) len = 250;

    /* Build the entry: [timestamp_ms:4][msg_len:1][msg:N] */
    uint8_t hdr[5];
    hdr[0] = (uint8_t)(timestamp_ms & 0xFF);
    hdr[1] = (uint8_t)((timestamp_ms >> 8) & 0xFF);
    hdr[2] = (uint8_t)((timestamp_ms >> 16) & 0xFF);
    hdr[3] = (uint8_t)((timestamp_ms >> 24) & 0xFF);
    hdr[4] = (uint8_t)len;

    uint32_t total_bytes = 5 + (uint32_t)len;

    if (summary_write_addr + total_bytes > FLASH_SUMMARY_END) return;

    /* Write using page-sized chunks, handling page boundaries */
    ensure_erased(summary_write_addr, &summary_erased_up_to);

    /* If the entry spans a sector boundary, ensure the next sector too */
    uint32_t end_addr = summary_write_addr + total_bytes - 1;
    if ((end_addr / FLOG_SECTOR_SIZE) != (summary_write_addr / FLOG_SECTOR_SIZE)) {
        ensure_erased(end_addr, &summary_erased_up_to);
    }

#ifndef HOST_TEST
    /* Write header */
    w25q512jv_write(&flash, summary_write_addr, hdr, 5);
    /* Write message body */
    if (len > 0) {
        w25q512jv_write(&flash, summary_write_addr + 5,
                        (const uint8_t *)msg_buf, (uint32_t)len);
    }
#endif

    summary_write_addr += total_bytes;
}

void flight_log_flush(void)
{
    /* Flush any pending full-sector double buffer first */
    flight_log_process();

    /* ── Flush partial HR buffer ─────────────────────────────────── */
    if (hr_buf_idx > 0 && logging_active) {
        if (hr_write_addr + FLOG_SECTOR_SIZE <= FLASH_HR_END) {
            /* Pad remaining entries with 0xFF */
            memset(&hr_active[hr_buf_idx], 0xFF,
                   (HR_ENTRIES_PER_SECTOR - hr_buf_idx) * sizeof(highrate_entry_t));

            ensure_erased(hr_write_addr, &hr_erased_up_to);

#ifndef HOST_TEST
            const uint8_t *src = (const uint8_t *)hr_active;
            for (uint32_t p = 0; p < (FLOG_SECTOR_SIZE / FLOG_PAGE_SIZE); p++) {
                w25q512jv_write(&flash, hr_write_addr + (p * FLOG_PAGE_SIZE),
                                src + (p * FLOG_PAGE_SIZE), FLOG_PAGE_SIZE);
            }
#endif
            hr_entry_count += hr_buf_idx;
            hr_write_addr  += FLOG_SECTOR_SIZE;
        }
        hr_buf_idx = 0;
    }

    /* ── Flush partial LR buffer ─────────────────────────────────── */
    if (lr_buf_idx > 0 && logging_active) {
        if (lr_write_addr + FLOG_SECTOR_SIZE <= FLASH_LR_END) {
            memset(&lr_active[lr_buf_idx], 0xFF,
                   (LR_ENTRIES_PER_SECTOR - lr_buf_idx) * sizeof(lowrate_entry_t));

            ensure_erased(lr_write_addr, &lr_erased_up_to);

#ifndef HOST_TEST
            const uint8_t *src = (const uint8_t *)lr_active;
            for (uint32_t p = 0; p < (FLOG_SECTOR_SIZE / FLOG_PAGE_SIZE); p++) {
                w25q512jv_write(&flash, lr_write_addr + (p * FLOG_PAGE_SIZE),
                                src + (p * FLOG_PAGE_SIZE), FLOG_PAGE_SIZE);
            }
#endif
            lr_entry_count += lr_buf_idx;
            lr_write_addr  += FLOG_SECTOR_SIZE;
        }
        lr_buf_idx = 0;
    }
}

int flight_log_erase_all(void)
{
#ifndef HOST_TEST
    /* HR region: 56 MB / 64 KB = 896 blocks */
    for (uint32_t addr = FLASH_HR_START; addr < FLASH_HR_END; addr += FLOG_BLOCK64_SIZE) {
        int rc = w25q512jv_erase_block(&flash, addr);
        if (rc != 0) return rc;
    }

    /* LR region: 6 MB / 64 KB = 96 blocks */
    for (uint32_t addr = FLASH_LR_START; addr < FLASH_LR_END; addr += FLOG_BLOCK64_SIZE) {
        int rc = w25q512jv_erase_block(&flash, addr);
        if (rc != 0) return rc;
    }

    /* Summary region: 1 MB / 64 KB = 16 blocks */
    for (uint32_t addr = FLASH_SUMMARY_START; addr < FLASH_SUMMARY_END; addr += FLOG_BLOCK64_SIZE) {
        int rc = w25q512jv_erase_block(&flash, addr);
        if (rc != 0) return rc;
    }
#endif

    /* Reset all state */
    hr_write_addr      = FLASH_HR_START;
    lr_write_addr      = FLASH_LR_START;
    summary_write_addr = FLASH_SUMMARY_START;

    hr_entry_count = 0;
    lr_entry_count = 0;
    hr_dropped     = 0;
    lr_dropped     = 0;

    hr_erased_up_to      = FLASH_HR_START;
    lr_erased_up_to      = FLASH_LR_START;
    summary_erased_up_to = FLASH_SUMMARY_START;

    hr_ring_head  = 0;
    hr_ring_count = 0;
    lr_ring_head  = 0;
    lr_ring_count = 0;

    hr_buf_idx = 0;
    lr_buf_idx = 0;
    hr_flush_pending = false;
    lr_flush_pending = false;

    logging_active = false;

    return 0;
}

/* ── Getters ─────────────────────────────────────────────────────── */

uint32_t flight_log_get_hr_addr(void)      { return hr_write_addr; }
uint32_t flight_log_get_lr_addr(void)      { return lr_write_addr; }
uint32_t flight_log_get_summary_addr(void) { return summary_write_addr; }
uint32_t flight_log_get_hr_count(void)     { return hr_entry_count; }
uint32_t flight_log_get_lr_count(void)     { return lr_entry_count; }
uint32_t flight_log_get_dropped_hr(void)   { return hr_dropped; }
uint32_t flight_log_get_dropped_lr(void)   { return lr_dropped; }

uint32_t flight_log_get_timestamp_us(void)
{
#ifndef HOST_TEST
    return DWT->CYCCNT / (SYSCLK_HZ / 1000000UL);
#else
    return 0;
#endif
}
