#include "cfg_manager.h"
#include "flight_config.h"
#include "crc32_hw.h"
#include "tlm_manager.h"
#include "flight_readout.h"
#include "flight_log.h"
#include <string.h>

/* ── Private state ──────────────────────────────────────────────── */
static flight_config_t s_config;
static uint32_t        s_config_hash;
static bool            s_config_valid;

static flight_config_full_t s_full_config;

/* ── Public API ─────────────────────────────────────────────────── */

void cfg_manager_init(void)
{
    memset(&s_config, 0, sizeof(s_config));
    s_config_hash = 0;
    s_config_valid = false;

    flight_config_default(&s_full_config);

    /* TODO: Load config from QSPI flash via FATFS if present */
}

void cfg_handle_upload(const uint8_t *data, int len)
{
    /* Minimum: msg_id(1) + config payload + CRC-32(4) */
    if (len < (int)(1 + sizeof(flight_config_t) + 4)) {
        return;
    }

    /* Validate CRC-32 over everything before the last 4 bytes */
    int payload_len = len - 4;
    uint32_t received_crc = 0;
    memcpy(&received_crc, &data[payload_len], 4);
    if (crc32_hw_compute(data, (uint32_t)payload_len) != received_crc) {
        return;
    }

    /* Copy config from payload (starts at byte 1, after msg_id) */
    memcpy(&s_config, &data[1], sizeof(flight_config_t));

    /* Compute and store hash */
    s_config_hash = crc32_hw_compute((const uint8_t *)&s_config,
                                      sizeof(flight_config_t));
    s_config.config_hash = s_config_hash;
    s_config_valid = true;

    /* TODO: Write to QSPI flash via FATFS */

    /* Send ACK with config hash */
    uint8_t ack[10];
    ack[0] = MSG_ID_ACK_CFG;   /* 0xA3 */
    ack[1] = (uint8_t)(s_config_hash & 0xFF);
    ack[2] = (uint8_t)((s_config_hash >> 8) & 0xFF);
    ack[3] = (uint8_t)((s_config_hash >> 16) & 0xFF);
    ack[4] = (uint8_t)((s_config_hash >> 24) & 0xFF);
    ack[5] = PROTOCOL_VERSION;
    uint32_t crc = crc32_hw_compute(ack, 6);
    ack[6] = (uint8_t)(crc & 0xFF);
    ack[7] = (uint8_t)((crc >> 8) & 0xFF);
    ack[8] = (uint8_t)((crc >> 16) & 0xFF);
    ack[9] = (uint8_t)((crc >> 24) & 0xFF);
    tlm_send_response(ack, 10);
}

void cfg_handle_readlog(const uint8_t *data, int len)
{
    (void)data;
    (void)len;
    /* Stream high-rate flight log entries over USB CDC.
     * The readout protocol sends a header followed by all entries
     * and a trailing CRC.  See flight_readout.h for format details. */
    flight_readout_stream_hr();
}

void cfg_handle_eraselog(const uint8_t *data, int len)
{
    (void)data;
    (void)len;
    /* Erase all flight log data (data + summary regions).
     * WARNING: This triggers a full chip erase which takes up to 400s. */
    flight_log_erase_all();
}

void cfg_handle_readsummary(const uint8_t *data, int len)
{
    (void)data;
    (void)len;
    /* Stream summary events over USB CDC.
     * See flight_readout.h for format details. */
    flight_readout_stream_summary();
}

uint32_t cfg_get_active_hash(void)
{
    return s_config_hash;
}

const flight_config_t *cfg_get_active(void)
{
    if (!s_config_valid) {
        return (const flight_config_t *)0;
    }
    return &s_config;
}

const flight_config_full_t *cfg_get_full_config(void)
{
    return &s_full_config;
}
