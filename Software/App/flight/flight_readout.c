/**
 * flight_readout.c - Post-flight log readout over USB CDC
 *
 * Streams high-rate, low-rate, and summary flight log data from QSPI flash
 * over USB CDC.  Designed for post-flight retrieval when the system is in
 * LANDED or PAD state.
 *
 * High-rate / low-rate stream format:
 *   [CASP:4][stream_id:1][entry_size:1][reserved:2][count:4][header_crc:4] = 16
 *   [N * 64-byte entries]
 *   [data_crc:4]
 *
 * Summary stream format:
 *   [SUMM:4][payload_size:4][header_crc:4] = 12
 *   [payload bytes]
 *   [data_crc:4]
 *
 * Metadata response:
 *   [META:4][hr_count:4][lr_count:4][summary_bytes:4][hr_addr:4][lr_addr:4][crc:4] = 28
 */

#include "flight_readout.h"
#include "flight_log.h"
#include "crc32_hw.h"
#include "w25q512jv.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* ── Flash device handle (defined in main.c) ────────── */
extern w25q512jv_t flash;

/* ── Helpers ─────────────────────────────────────────── */

static void cdc_transmit_blocking(const uint8_t *data, uint32_t len)
{
    uint32_t offset = 0;
    while (offset < len) {
        uint32_t chunk = len - offset;
        if (chunk > 512) chunk = 512;
        while (CDC_Transmit_FS((uint8_t *)(data + offset), (uint16_t)chunk) == USBD_BUSY) {
            /* spin -- host drains buffer */
        }
        offset += chunk;
    }
}

/**
 * Stream a contiguous flash log region over CDC with CRC.
 *
 * @param start_addr  Flash start address of the log region
 * @param entry_count Number of entries written
 * @param entry_size  Size of each entry in bytes (64)
 * @param stream_id   Stream identifier (0x01=HR, 0x02=LR)
 * @return 0 on success
 */
static int stream_log_region(uint32_t start_addr, uint32_t entry_count,
                             uint8_t entry_size, uint8_t stream_id)
{
    uint8_t header[16];
    uint32_t crc;

    /* Build header: [CASP:4][stream_id:1][entry_size:1][reserved:2][count:4][CRC:4] = 16 */
    header[0] = 'C'; header[1] = 'A'; header[2] = 'S'; header[3] = 'P';
    header[4] = stream_id;
    header[5] = entry_size;
    header[6] = 0x00; header[7] = 0x00;
    memcpy(&header[8], &entry_count, 4);
    crc = crc32_hw_compute(header, 12);
    memcpy(&header[12], &crc, 4);
    cdc_transmit_blocking(header, 16);

    /* Stream entries page-by-page with running CRC */
    uint32_t data_crc = 0;  /* crc32_hw_compute_continue starts from 0 */
    uint32_t total_bytes = (uint32_t)entry_count * entry_size;
    uint8_t page_buf[FLOG_PAGE_SIZE];
    uint32_t addr = start_addr;

    while (total_bytes > 0) {
        uint32_t chunk = (total_bytes < 256) ? total_bytes : 256;
        w25q512jv_read(&flash, addr, page_buf, chunk);
        data_crc = crc32_hw_compute_continue(data_crc, page_buf, chunk);
        cdc_transmit_blocking(page_buf, chunk);
        addr += chunk;
        total_bytes -= chunk;
    }

    /* Send final data CRC */
    cdc_transmit_blocking((uint8_t *)&data_crc, 4);
    return 0;
}

/* ── Public API ──────────────────────────────────────── */

int flight_readout_stream_hr(void)
{
    return stream_log_region(FLASH_HR_START, flight_log_get_hr_count(), 64, 0x01);
}

int flight_readout_stream_lr(void)
{
    return stream_log_region(FLASH_LR_START, flight_log_get_lr_count(), 64, 0x02);
}

int flight_readout_stream_summary(void)
{
    uint32_t payload_size = flight_log_get_summary_addr() - FLASH_SUMMARY_START;
    uint8_t header[12];

    /* [SUMM:4][payload_size:4][CRC:4] = 12 */
    header[0] = 'S'; header[1] = 'U'; header[2] = 'M'; header[3] = 'M';
    memcpy(&header[4], &payload_size, 4);
    uint32_t crc = crc32_hw_compute(header, 8);
    memcpy(&header[8], &crc, 4);
    cdc_transmit_blocking(header, 12);

    /* Stream summary data */
    uint32_t data_crc = 0;
    uint32_t remaining = payload_size;
    uint32_t addr = FLASH_SUMMARY_START;
    uint8_t buf[256];

    while (remaining > 0) {
        uint32_t chunk = (remaining < 256) ? remaining : 256;
        w25q512jv_read(&flash, addr, buf, chunk);
        data_crc = crc32_hw_compute_continue(data_crc, buf, chunk);
        cdc_transmit_blocking(buf, chunk);
        addr += chunk;
        remaining -= chunk;
    }

    cdc_transmit_blocking((uint8_t *)&data_crc, 4);
    return 0;
}

int flight_readout_send_metadata(void)
{
    uint8_t resp[28];

    /* [META:4][hr_count:4][lr_count:4][summary_bytes:4][hr_addr:4][lr_addr:4][CRC:4] = 28 */
    resp[0] = 'M'; resp[1] = 'E'; resp[2] = 'T'; resp[3] = 'A';
    uint32_t hr_count  = flight_log_get_hr_count();
    uint32_t lr_count  = flight_log_get_lr_count();
    uint32_t sum_bytes = flight_log_get_summary_addr() - FLASH_SUMMARY_START;
    uint32_t hr_addr   = flight_log_get_hr_addr();
    uint32_t lr_addr   = flight_log_get_lr_addr();

    memcpy(&resp[4],  &hr_count,  4);
    memcpy(&resp[8],  &lr_count,  4);
    memcpy(&resp[12], &sum_bytes, 4);
    memcpy(&resp[16], &hr_addr,   4);
    memcpy(&resp[20], &lr_addr,   4);
    uint32_t crc = crc32_hw_compute(resp, 24);
    memcpy(&resp[24], &crc, 4);
    cdc_transmit_blocking(resp, 28);
    return 0;
}

int flight_readout_handle_command(uint8_t cmd)
{
    switch (cmd) {
        case READOUT_CMD_HR:       return flight_readout_stream_hr();
        case READOUT_CMD_LR:       return flight_readout_stream_lr();
        case READOUT_CMD_SUMMARY:  return flight_readout_stream_summary();
        case READOUT_CMD_METADATA: return flight_readout_send_metadata();
        case READOUT_CMD_ERASE:    return flight_log_erase_all();
        default:                   return -1;
    }
}
