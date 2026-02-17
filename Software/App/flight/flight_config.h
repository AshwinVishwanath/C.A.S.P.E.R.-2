#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "tlm_types.h"      /* for existing flight_config_t */
#include "pyro_logic.h"     /* for pyro_channel_config_t */

/* ── Full flight configuration ──────────────────────── */
typedef struct {
    flight_config_t basic;           /* existing 6-field struct, UNTOUCHED */
    pyro_channel_config_t channels[4];
    uint8_t motors_expected;         /* derived: 1 if no IGNITION channel, 2 if any */
    uint8_t num_channels;            /* 0-4 */
    uint8_t config_version;          /* protocol version */
    float   min_batt_v;              /* minimum battery voltage for arm */
    float   min_integrity_pct;       /* minimum self-test pass rate */
    uint32_t config_hash;            /* CRC-32 of config payload */
    bool    config_loaded;           /* true after successful parse */
} flight_config_full_t;

/* ── Config binary format ───────────────────────────── */
/* Total: 163 bytes (fixed, zero-padded)
 * [version:1][pad_lat:4][pad_lon:4][pad_alt:4][main_deploy:4][launch_accel:4]
 * [min_batt:4][min_integrity:4]
 * [num_channels:1]
 * For each channel (up to 4): [role:1][alt_source:1][deploy_alt:4][fire_delay:4]
 *                              [max_angle:4][backup_mode:1][backup_timer:4]
 *                              [backup_alt:4][primary_ch:1]
 *                              = 24 bytes per channel
 * [CRC-32:4] at the end
 *
 * Header: 1 + 7*4 + 1 = 30 bytes
 * Channels: up to 4 * 24 = 96 bytes
 * CRC: 4 bytes
 * Max used: 30 + 96 + 4 = 130 bytes, padded to 163.
 */
#define FLIGHT_CONFIG_PAYLOAD_SIZE  163
#define FLIGHT_CONFIG_HEADER_SIZE   30    /* version(1) + 7 floats(28) + num_channels(1) */
#define FLIGHT_CONFIG_CHANNEL_SIZE  24    /* per channel */

/* ── API ─────────────────────────────────────────────── */

/**
 * Parse a 163-byte config payload.
 * Returns true on success (valid CRC, valid fields).
 */
bool flight_config_parse(const uint8_t *payload, uint16_t len,
                         flight_config_full_t *out);

/**
 * Fill config with safe defaults (single-stage, no pyro channels).
 */
void flight_config_default(flight_config_full_t *out);

/**
 * Compute CRC-32 of a config payload (first len-4 bytes).
 */
uint32_t flight_config_compute_crc(const uint8_t *payload, uint16_t len);

#endif /* FLIGHT_CONFIG_H */
