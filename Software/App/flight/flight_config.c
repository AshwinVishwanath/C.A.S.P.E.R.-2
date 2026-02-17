/*
 * flight_config.c -- Flight configuration binary parser (Level 6)
 *
 * Parses a 163-byte config payload from Mission Control into
 * flight_config_full_t. Embeds the existing flight_config_t as .basic
 * for backward compatibility.
 */

#include "flight_config.h"
#include <string.h>

/* ── CRC-32 implementation ──────────────────────────────────────── */
#ifndef HOST_TEST
#include "crc32_hw.h"
#else
/*
 * Software CRC-32 (IEEE 802.3, polynomial 0xEDB88320 reflected).
 * Used for host-side unit tests where STM32 CRC peripheral is unavailable.
 *
 * Config: init=0xFFFFFFFF, input reflection=byte, output reflection=yes,
 *         final XOR=0xFFFFFFFF.  Check value for "123456789" = 0xCBF43926.
 */
static uint32_t crc32_sw_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFu;
}
#endif /* HOST_TEST */

/* ── Byte extraction helpers (little-endian) ────────────────────── */
static float read_f32_le(const uint8_t *p)
{
    float f;
    memcpy(&f, p, 4);
    return f;
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* ── CRC wrapper ────────────────────────────────────────────────── */
static uint32_t compute_crc(const uint8_t *data, uint32_t len)
{
#ifndef HOST_TEST
    return crc32_hw_compute(data, len);
#else
    return crc32_sw_compute(data, len);
#endif
}

/* ── Public API ─────────────────────────────────────────────────── */

uint32_t flight_config_compute_crc(const uint8_t *payload, uint16_t len)
{
    if (len <= 4) {
        return 0;
    }
    return compute_crc(payload, (uint32_t)(len - 4));
}

void flight_config_default(flight_config_full_t *out)
{
    memset(out, 0, sizeof(*out));

    /* Basic config -- safe defaults */
    out->basic.pad_lat_deg      = 0.0f;
    out->basic.pad_lon_deg      = 0.0f;
    out->basic.pad_alt_m        = 0.0f;
    out->basic.main_deploy_alt_m = 300.0f;
    out->basic.launch_accel_g   = 4.0f;
    out->basic.config_hash      = 0;

    /* Extended config */
    out->motors_expected    = 1;
    out->num_channels       = 0;    /* no pyro channels configured */
    out->config_version     = 0;
    out->min_batt_v         = 6.0f;
    out->min_integrity_pct  = 80.0f;
    out->config_hash        = 0;
    out->config_loaded      = false;
}

bool flight_config_parse(const uint8_t *payload, uint16_t len,
                         flight_config_full_t *out)
{
    if (!payload || !out) {
        return false;
    }

    /* Must be exactly 163 bytes */
    if (len != FLIGHT_CONFIG_PAYLOAD_SIZE) {
        return false;
    }

    /* ── CRC validation ────────────────────────────────────────── */
    uint32_t received_crc = read_u32_le(&payload[len - 4]);
    uint32_t computed_crc = compute_crc(payload, (uint32_t)(len - 4));
    if (received_crc != computed_crc) {
        return false;
    }

    /* Start with clean state */
    memset(out, 0, sizeof(*out));

    /* ── Header: [version:1][pad_lat:4][pad_lon:4][pad_alt:4]
     *            [main_deploy:4][launch_accel:4][min_batt:4][min_integrity:4]
     *            [num_channels:1]
     * = 1 + 7*4 + 1 = 30 bytes ──────────────────────────────── */
    uint16_t offset = 0;

    out->config_version = payload[offset];
    offset += 1;   /* byte 1 */

    out->basic.pad_lat_deg = read_f32_le(&payload[offset]);
    offset += 4;   /* byte 5 */

    out->basic.pad_lon_deg = read_f32_le(&payload[offset]);
    offset += 4;   /* byte 9 */

    out->basic.pad_alt_m = read_f32_le(&payload[offset]);
    offset += 4;   /* byte 13 */

    out->basic.main_deploy_alt_m = read_f32_le(&payload[offset]);
    offset += 4;   /* byte 17 */

    out->basic.launch_accel_g = read_f32_le(&payload[offset]);
    offset += 4;   /* byte 21 */

    out->min_batt_v = read_f32_le(&payload[offset]);
    offset += 4;   /* byte 25 */

    out->min_integrity_pct = read_f32_le(&payload[offset]);
    offset += 4;   /* byte 29 */

    uint8_t num_ch = payload[offset];
    offset += 1;   /* byte 30 */

    if (num_ch > 4) {
        return false;   /* invalid channel count */
    }
    out->num_channels = num_ch;

    /* ── Per-channel blocks: 24 bytes each ─────────────────────
     * [role:1][alt_source:1][deploy_alt:4][fire_delay:4][max_angle:4]
     * [backup_mode:1][backup_timer:4][backup_alt:4][primary_ch:1]
     * = 24 bytes */
    for (uint8_t i = 0; i < num_ch; i++) {
        uint16_t ch_base = offset + (uint16_t)(i * FLIGHT_CONFIG_CHANNEL_SIZE);

        /* Bounds check: ch_base + 24 must be <= len - 4 (CRC area) */
        if ((ch_base + FLIGHT_CONFIG_CHANNEL_SIZE) > (len - 4)) {
            return false;
        }

        pyro_channel_config_t *ch = &out->channels[i];

        ch->role        = (pyro_role_t)payload[ch_base + 0];
        ch->alt_source  = (alt_source_t)payload[ch_base + 1];
        ch->deploy_alt_m         = read_f32_le(&payload[ch_base + 2]);
        ch->fire_delay_s         = read_f32_le(&payload[ch_base + 6]);
        ch->max_flight_angle_deg = read_f32_le(&payload[ch_base + 10]);
        ch->backup_mode          = (backup_mode_t)payload[ch_base + 14];
        ch->backup_timer_s       = read_f32_le(&payload[ch_base + 15]);
        ch->backup_alt_m         = read_f32_le(&payload[ch_base + 19]);
        ch->primary_channel      = payload[ch_base + 23];
    }

    /* ── Derive motors_expected ─────────────────────────────── */
    out->motors_expected = 1;
    for (uint8_t i = 0; i < num_ch; i++) {
        if (out->channels[i].role == PYRO_ROLE_IGNITION) {
            out->motors_expected = 2;
            break;
        }
    }

    /* ── Store hash in both full config and basic config ────── */
    out->config_hash = computed_crc;
    out->basic.config_hash = computed_crc;

    out->config_loaded = true;
    return true;
}
