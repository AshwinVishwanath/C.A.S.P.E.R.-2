/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Config Manager
 *  SUMMARY:  Persistent config blob (radio profile, deployments, etc.).
 * ============================================================ */
#include "cfg_manager.h"
#include "crc32_hw.h"
#include "tlm_manager.h"
#include "endian.h"
#include <string.h>

/* ── Private state ──────────────────────────────────────────────── */
static flight_config_t s_config;
static uint32_t        s_config_hash;
static bool            s_config_valid;

/* ── Public API ─────────────────────────────────────────────────── */

void cfg_manager_init(void)
{
    memset(&s_config, 0, sizeof(s_config));
    s_config_hash = 0;
    s_config_valid = false;

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
    put_le32(&ack[1], s_config_hash);
    ack[5] = PROTOCOL_VERSION;
    uint32_t crc = crc32_hw_compute(ack, 6);
    put_le32(&ack[6], crc);
    tlm_send_response(ack, 10);
}

void cfg_handle_readlog(const uint8_t *data, int len)
{
    (void)data;
    (void)len;
    /* TODO: Read flight log from QSPI flash and stream over CDC */
}

void cfg_handle_eraselog(const uint8_t *data, int len)
{
    (void)data;
    (void)len;
    /* TODO: Erase flight log sectors on QSPI flash */
}

uint32_t cfg_get_active_hash(void)
{
    return s_config_hash;
}
