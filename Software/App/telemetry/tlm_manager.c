#include "tlm_manager.h"
#include "cobs.h"
#include "crc32_hw.h"
#include "quat_pack.h"
#include "status_pack.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <math.h>

/* ── Private state ──────────────────────────────────────────────── */
static uint8_t  s_seq;
static uint32_t s_last_fast_ms;

/* TX buffer: raw packet + COBS overhead + delimiter */
static uint8_t s_raw_buf[32];
static uint8_t s_cobs_buf[36];

/* ── Helpers ────────────────────────────────────────────────────── */
static void put_le16(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void put_le32(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
    dst[2] = (uint8_t)((val >> 16) & 0xFF);
    dst[3] = (uint8_t)((val >> 24) & 0xFF);
}

static int cobs_encode_and_send(const uint8_t *raw, int raw_len)
{
    int enc_len = cobs_encode(raw, raw_len, s_cobs_buf, (int)sizeof(s_cobs_buf) - 1);
    if (enc_len < 0) {
        return 0;
    }
    /* Append 0x00 delimiter */
    s_cobs_buf[enc_len] = 0x00;
    enc_len++;

    if (CDC_Transmit_FS(s_cobs_buf, (uint16_t)enc_len) != USBD_OK) {
        return 0;
    }
    return 1;
}

/* ── Public API ─────────────────────────────────────────────────── */

void tlm_init(void)
{
    s_seq = 0;
    s_last_fast_ms = 0;
}

int tlm_tick(const fc_telem_state_t *state, const pyro_state_t *pyro,
             fsm_state_t fsm)
{
    uint32_t now = HAL_GetTick();
    if (now - s_last_fast_ms < TLM_FAST_PERIOD_MS) {
        return 0;
    }
    s_last_fast_ms = now;

    /* Build FC_MSG_FAST (19 bytes) per PRD Section 3.1 */
    uint8_t *p = s_raw_buf;

    /* Bytes 0–1: FC_TLM_STATUS */
    status_pack_build(p, pyro, fsm, false);
    p += 2;

    /* Bytes 2–3: altitude in decametres, uint16 LE */
    float alt_dam = state->alt_m / ALT_SCALE_DAM;
    if (alt_dam < 0.0f) alt_dam = 0.0f;
    if (alt_dam > 65535.0f) alt_dam = 65535.0f;
    put_le16(p, (uint16_t)alt_dam);
    p += 2;

    /* Bytes 4–5: velocity in dm/s, int16 LE */
    float vel_dms = state->vel_mps / VEL_SCALE_DMS;
    if (vel_dms > 32767.0f) vel_dms = 32767.0f;
    if (vel_dms < -32768.0f) vel_dms = -32768.0f;
    put_le16(p, (uint16_t)(int16_t)vel_dms);
    p += 2;

    /* Bytes 6–10: quaternion packed (5 bytes) */
    quat_pack_smallest_three(p, state->quat);
    p += 5;

    /* Bytes 11–12: flight time in 0.1s ticks, uint16 LE */
    float time_ds = state->flight_time_s / TIME_SCALE_100MS;
    if (time_ds < 0.0f) time_ds = 0.0f;
    if (time_ds > 65535.0f) time_ds = 65535.0f;
    put_le16(p, (uint16_t)time_ds);
    p += 2;

    /* Byte 13: battery voltage, uint8 */
    float batt_encoded = (state->batt_v - BATT_OFFSET_V) / BATT_STEP_V;
    if (batt_encoded < 0.0f) batt_encoded = 0.0f;
    if (batt_encoded > 255.0f) batt_encoded = 255.0f;
    *p++ = (uint8_t)batt_encoded;

    /* Byte 14: sequence counter */
    *p++ = s_seq++;

    /* Bytes 15–18: CRC-32 over bytes 0–14 */
    uint32_t crc = crc32_hw_compute(s_raw_buf, 15);
    put_le32(p, crc);

    return cobs_encode_and_send(s_raw_buf, SIZE_FC_MSG_FAST);
}

int tlm_send_gps(const fc_gps_state_t *gps_state)
{
    uint8_t *p = s_raw_buf;

    /* Bytes 0–1: dlat_m, int16 LE */
    put_le16(p, (uint16_t)(int16_t)gps_state->dlat_m);
    p += 2;

    /* Bytes 2–3: dlon_m, int16 LE */
    put_le16(p, (uint16_t)(int16_t)gps_state->dlon_m);
    p += 2;

    /* Bytes 4–5: alt_msl in decametres, uint16 LE */
    float alt_dam = gps_state->alt_msl_m / ALT_SCALE_DAM;
    if (alt_dam < 0.0f) alt_dam = 0.0f;
    if (alt_dam > 65535.0f) alt_dam = 65535.0f;
    put_le16(p, (uint16_t)alt_dam);
    p += 2;

    /* Byte 6: fix_type [7:6] | sat_count [5:0] */
    *p++ = (uint8_t)((gps_state->fix_type << 6) | (gps_state->sat_count & 0x3F));

    /* Byte 7: PDOP * 10 */
    float pdop_enc = gps_state->pdop * 10.0f;
    if (pdop_enc > 255.0f) pdop_enc = 255.0f;
    if (pdop_enc < 0.0f) pdop_enc = 0.0f;
    *p++ = (uint8_t)pdop_enc;

    /* Bytes 8–11: CRC-32 over bytes 0–7 */
    uint32_t crc = crc32_hw_compute(s_raw_buf, 8);
    put_le32(p, crc);

    return cobs_encode_and_send(s_raw_buf, SIZE_FC_MSG_GPS);
}

int tlm_queue_event(uint8_t type, uint16_t data)
{
    uint8_t *p = s_raw_buf;

    /* Byte 0: event type */
    *p++ = type;

    /* Bytes 1–2: event data, uint16 LE */
    put_le16(p, data);
    p += 2;

    /* Bytes 3–4: event time (0.1s ticks), uint16 LE */
    extern float flight_fsm_get_time_s(void);
    float time_ds = flight_fsm_get_time_s() / TIME_SCALE_100MS;
    if (time_ds < 0.0f) time_ds = 0.0f;
    if (time_ds > 65535.0f) time_ds = 65535.0f;
    put_le16(p, (uint16_t)time_ds);
    p += 2;

    /* Bytes 5–8: CRC-32 over bytes 0–4 */
    uint32_t crc = crc32_hw_compute(s_raw_buf, 5);
    put_le32(p, crc);

    return cobs_encode_and_send(s_raw_buf, SIZE_FC_MSG_EVENT);
}

uint8_t tlm_get_seq(void)
{
    return s_seq;
}

int tlm_send_response(const uint8_t *data, int len)
{
    if (len <= 0 || len > (int)sizeof(s_raw_buf)) {
        return 0;
    }
    memcpy(s_raw_buf, data, (size_t)len);
    return cobs_encode_and_send(s_raw_buf, len);
}
