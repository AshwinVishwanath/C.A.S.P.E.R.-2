/**
 * flight_readout.c - Post-flight log readout over USB CDC
 *
 * Streams raw binary flight log entries and summary events from QSPI
 * flash over USB CDC. Designed for post-flight retrieval when the
 * system is in LANDED or PAD state.
 *
 * Raw stream format:
 *   [4 bytes] Magic: 0x43 0x41 0x53 0x50  ("CASP")
 *   [4 bytes] Entry count (u32 LE)
 *   [4 bytes] Entry size (u32 LE, always 32)
 *   [4 bytes] CRC-32 of header (12 bytes above)
 *   [N x 32 bytes] Log entries, sequential
 *   [4 bytes] CRC-32 of all entry data
 *
 * Summary stream format:
 *   [4 bytes] Magic: 0x53 0x55 0x4D 0x4D  ("SUMM")
 *   [4 bytes] Total size in bytes (u32 LE)
 *   [N bytes] Summary entries (length-prefixed strings)
 *   [4 bytes] CRC-32 of all summary data
 */

#ifndef HOST_TEST
#include "stm32h7xx_hal.h"
#include "w25q512jv.h"
#include "usbd_cdc_if.h"
#include "crc32_hw.h"
#endif

#include "flight_readout.h"
#include "flight_log.h"
#include <string.h>

/* ── Flash device handle (defined in main.c) ────────── */
#ifndef HOST_TEST
extern w25q512jv_t flash;
#endif

/* ── Constants ───────────────────────────────────────── */
#define READOUT_CHUNK_ENTRIES  32          /* entries per flash read (32 * 32 = 1024 bytes) */
#define READOUT_CHUNK_BYTES   (READOUT_CHUNK_ENTRIES * sizeof(flight_log_entry_t))
#define ENTRY_SIZE            ((uint32_t)sizeof(flight_log_entry_t))  /* 32 */
#define ERASED_TIMESTAMP      0xFFFFFFFFu

/* Summary region read chunk size */
#define SUMMARY_CHUNK_SIZE    256

/* Magic bytes */
static const uint8_t MAGIC_CASP[4] = { 0x43, 0x41, 0x53, 0x50 }; /* "CASP" */
static const uint8_t MAGIC_SUMM[4] = { 0x53, 0x55, 0x4D, 0x4D }; /* "SUMM" */

/* ── Helpers ─────────────────────────────────────────── */

/**
 * Put a uint32_t into a buffer in little-endian order.
 */
static void put_le32(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val & 0xFFu);
    dst[1] = (uint8_t)((val >> 8) & 0xFFu);
    dst[2] = (uint8_t)((val >> 16) & 0xFFu);
    dst[3] = (uint8_t)((val >> 24) & 0xFFu);
}

#ifndef HOST_TEST
/**
 * Transmit a buffer over USB CDC with retry on USBD_BUSY.
 * Returns 0 on success, -1 on failure after retries.
 */
static int cdc_transmit_blocking(uint8_t *buf, uint16_t len)
{
    uint32_t retries = 0;
    const uint32_t max_retries = 100;

    while (retries < max_retries) {
        uint8_t result = CDC_Transmit_FS(buf, len);
        if (result == 0) {  /* USBD_OK */
            return 0;
        }
        /* USBD_BUSY or other — wait and retry */
        HAL_Delay(1);
        retries++;
    }
    return -1;
}
#endif

/* ── Public API ──────────────────────────────────────── */

uint32_t flight_readout_get_entry_count(void)
{
#ifndef HOST_TEST
    uint8_t chunk[READOUT_CHUNK_BYTES];
    uint32_t addr = FLIGHT_LOG_FLASH_BASE;
    uint32_t count = 0;

    while (addr < FLIGHT_LOG_FLASH_END) {
        /* Calculate how many bytes to read this iteration */
        uint32_t remaining = FLIGHT_LOG_FLASH_END - addr;
        uint32_t read_bytes = (remaining < READOUT_CHUNK_BYTES)
                                ? remaining : READOUT_CHUNK_BYTES;

        /* Round down to a whole number of entries */
        uint32_t entries_in_chunk = read_bytes / ENTRY_SIZE;
        if (entries_in_chunk == 0) {
            break;
        }
        read_bytes = entries_in_chunk * ENTRY_SIZE;

        if (w25q512jv_read(&flash, addr, chunk, read_bytes) != W25Q_OK) {
            break;
        }

        /* Scan each entry's timestamp */
        for (uint32_t i = 0; i < entries_in_chunk; i++) {
            uint32_t ts = 0;
            memcpy(&ts, &chunk[i * ENTRY_SIZE], sizeof(uint32_t));
            if (ts == ERASED_TIMESTAMP) {
                return count;
            }
            count++;
        }

        addr += read_bytes;
    }

    return count;
#else
    return 0;
#endif
}

int flight_readout_stream_raw(void)
{
#ifndef HOST_TEST
    /* Step 1: Count valid entries */
    uint32_t entry_count = flight_readout_get_entry_count();

    /* Step 2: Build and send header
     * [magic:4][count:4][entry_size:4][crc:4] = 16 bytes */
    uint8_t header[16];
    memcpy(&header[0], MAGIC_CASP, 4);
    put_le32(&header[4], entry_count);
    put_le32(&header[8], ENTRY_SIZE);

    /* CRC of first 12 bytes of header */
    uint32_t header_crc = crc32_hw_compute(header, 12);
    put_le32(&header[12], header_crc);

    if (cdc_transmit_blocking(header, 16) != 0) {
        return -1;
    }
    HAL_Delay(2);

    /* Step 3: Stream entries from flash, computing running CRC */
    if (entry_count == 0) {
        /* No data — send zero CRC for empty payload */
        uint8_t empty_crc_buf[4];
        uint32_t empty_crc = crc32_hw_compute(NULL, 0);
        put_le32(empty_crc_buf, empty_crc);
        if (cdc_transmit_blocking(empty_crc_buf, 4) != 0) {
            return -1;
        }
        return 0;
    }

    /*
     * We cannot compute a running CRC with the hardware CRC peripheral
     * because crc32_hw_compute() calls HAL_CRC_Calculate which resets state
     * each time.  Instead, we read all entries and compute CRC in chunks
     * by accumulating into a software CRC, or we do a two-pass approach.
     *
     * Approach: Read and send chunks, accumulate bytes for a final CRC
     * computation. Since the HW CRC resets each call, we'll compute the
     * CRC over the data as we stream it using a simple software CRC-32.
     *
     * Alternative: just use crc32_hw_compute at the end by reading flash
     * again. Two-pass is simple and safe for post-flight (non time-critical).
     */

    uint8_t chunk[READOUT_CHUNK_BYTES];
    uint32_t addr = FLIGHT_LOG_FLASH_BASE;
    uint32_t entries_remaining = entry_count;

    /* Pass 1: Stream data over CDC */
    while (entries_remaining > 0) {
        uint32_t batch = (entries_remaining > READOUT_CHUNK_ENTRIES)
                           ? READOUT_CHUNK_ENTRIES : entries_remaining;
        uint32_t batch_bytes = batch * ENTRY_SIZE;

        if (w25q512jv_read(&flash, addr, chunk, batch_bytes) != W25Q_OK) {
            return -2;
        }

        if (cdc_transmit_blocking(chunk, (uint16_t)batch_bytes) != 0) {
            return -3;
        }
        HAL_Delay(1);

        addr += batch_bytes;
        entries_remaining -= batch;
    }

    /* Pass 2: Re-read flash to compute data CRC
     * This is acceptable since readout is post-flight and not time-critical. */
    uint32_t total_data_bytes = entry_count * ENTRY_SIZE;

    /* For CRC, read everything into a large-ish buffer and compute.
     * If total_data_bytes > chunk size, we use a software CRC approach. */
    uint32_t crc = 0xFFFFFFFFu;

    addr = FLIGHT_LOG_FLASH_BASE;
    entries_remaining = entry_count;

    while (entries_remaining > 0) {
        uint32_t batch = (entries_remaining > READOUT_CHUNK_ENTRIES)
                           ? READOUT_CHUNK_ENTRIES : entries_remaining;
        uint32_t batch_bytes = batch * ENTRY_SIZE;

        if (w25q512jv_read(&flash, addr, chunk, batch_bytes) != W25Q_OK) {
            return -4;
        }

        /* Software CRC-32 (IEEE 802.3): update running value */
        for (uint32_t i = 0; i < batch_bytes; i++) {
            crc ^= chunk[i];
            for (int bit = 0; bit < 8; bit++) {
                if (crc & 1u) {
                    crc = (crc >> 1) ^ 0xEDB88320u;
                } else {
                    crc >>= 1;
                }
            }
        }

        addr += batch_bytes;
        entries_remaining -= batch;
    }

    crc ^= 0xFFFFFFFFu;

    (void)total_data_bytes;

    /* Send data CRC */
    uint8_t crc_buf[4];
    put_le32(crc_buf, crc);
    if (cdc_transmit_blocking(crc_buf, 4) != 0) {
        return -5;
    }

    return 0;
#else
    return 0;
#endif
}

int flight_readout_stream_summary(void)
{
#ifndef HOST_TEST
    /* First pass: determine total size of summary data.
     * Summary entries are written sequentially starting at
     * FLIGHT_LOG_SUMMARY_BASE.  An erased byte (0xFF) at the start
     * of an entry means no more data. */
    uint8_t chunk[SUMMARY_CHUNK_SIZE];
    uint32_t addr = FLIGHT_LOG_SUMMARY_BASE;
    uint32_t total_size = 0;

    while (addr < FLIGHT_LOG_SUMMARY_END) {
        uint32_t remaining = FLIGHT_LOG_SUMMARY_END - addr;
        uint32_t read_len = (remaining < SUMMARY_CHUNK_SIZE)
                              ? remaining : SUMMARY_CHUNK_SIZE;

        if (w25q512jv_read(&flash, addr, chunk, read_len) != W25Q_OK) {
            break;
        }

        /* Scan for first 0xFF byte — indicates end of summary data.
         * The first byte of a summary entry is the timestamp LSB.
         * An erased entry has all 0xFF, so checking the first byte
         * of where the next entry would start is sufficient.
         * However, summary entries are variable-length (fixed struct
         * size = sizeof(flight_log_summary_entry_t)).
         * We scan byte-by-byte for the first occurrence of an all-0xFF
         * region, but the simplest correct approach is to check if
         * the first 4 bytes (timestamp) at each entry offset are 0xFF. */

        /* Since summary entries are fixed-size structs, step through them */
        uint32_t entry_size = (uint32_t)sizeof(flight_log_summary_entry_t);
        uint32_t scan_addr = addr;

        /* Read one entry at a time */
        uint8_t entry_buf[sizeof(flight_log_summary_entry_t)];

        while (scan_addr + entry_size <= FLIGHT_LOG_SUMMARY_END) {
            if (w25q512jv_read(&flash, scan_addr, entry_buf, entry_size) != W25Q_OK) {
                goto scan_done;
            }

            /* Check if timestamp is erased (0xFFFFFFFF) */
            uint32_t ts = 0;
            memcpy(&ts, entry_buf, sizeof(uint32_t));
            if (ts == 0xFFFFFFFFu) {
                goto scan_done;
            }

            total_size += entry_size;
            scan_addr += entry_size;
        }
        break;  /* reached end of region in inner loop */
    }

scan_done:
    ;

    /* Build and send header: [magic:4][total_size:4] = 8 bytes */
    uint8_t header[8];
    memcpy(&header[0], MAGIC_SUMM, 4);
    put_le32(&header[4], total_size);

    if (cdc_transmit_blocking(header, 8) != 0) {
        return -1;
    }
    HAL_Delay(2);

    /* Stream summary data and compute CRC */
    uint32_t crc = 0xFFFFFFFFu;
    addr = FLIGHT_LOG_SUMMARY_BASE;
    uint32_t bytes_remaining = total_size;

    while (bytes_remaining > 0) {
        uint32_t read_len = (bytes_remaining < SUMMARY_CHUNK_SIZE)
                              ? bytes_remaining : SUMMARY_CHUNK_SIZE;

        if (w25q512jv_read(&flash, addr, chunk, read_len) != W25Q_OK) {
            return -2;
        }

        if (cdc_transmit_blocking(chunk, (uint16_t)read_len) != 0) {
            return -3;
        }

        /* Update software CRC-32 */
        for (uint32_t i = 0; i < read_len; i++) {
            crc ^= chunk[i];
            for (int bit = 0; bit < 8; bit++) {
                if (crc & 1u) {
                    crc = (crc >> 1) ^ 0xEDB88320u;
                } else {
                    crc >>= 1;
                }
            }
        }

        addr += read_len;
        bytes_remaining -= read_len;
        HAL_Delay(1);
    }

    crc ^= 0xFFFFFFFFu;

    /* Send data CRC */
    uint8_t crc_buf[4];
    put_le32(crc_buf, crc);
    if (cdc_transmit_blocking(crc_buf, 4) != 0) {
        return -4;
    }

    return 0;
#else
    return 0;
#endif
}
