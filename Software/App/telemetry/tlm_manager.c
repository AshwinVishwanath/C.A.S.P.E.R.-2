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

    /* Build FC_MSG_FAST (20 bytes) per PRD Section 3.1 */
    uint8_t *p = s_raw_buf;

    /* Byte 0: message ID */
    *p++ = MSG_ID_FAST;

    /* Bytes 1–2: FC_TLM_STATUS */
    status_pack_build(p, pyro, fsm, false);
    p += 2;

    /* Bytes 3–4: altitude in decametres, uint16 LE */
    float alt_dam = state->alt_m / ALT_SCALE_M;
    if (alt_dam < 0.0f) alt_dam = 0.0f;
    if (alt_dam > 65535.0f) alt_dam = 65535.0f;
    put_le16(p, (uint16_t)alt_dam);
    p += 2;

    /* Bytes 5–6: velocity in dm/s, int16 LE */
    float vel_dms = state->vel_mps / VEL_SCALE_DMS;
    if (vel_dms > 32767.0f) vel_dms = 32767.0f;
    if (vel_dms < -32768.0f) vel_dms = -32768.0f;
    put_le16(p, (uint16_t)(int16_t)vel_dms);
    p += 2;

    /* Bytes 7–11: quaternion packed (5 bytes) */
    quat_pack_smallest_three(p, state->quat);
    p += 5;

    /* Bytes 12–13: flight time in 0.1s ticks, uint16 LE */
    float time_ds = state->flight_time_s / TIME_SCALE_100MS;
    if (time_ds < 0.0f) time_ds = 0.0f;
    if (time_ds > 65535.0f) time_ds = 65535.0f;
    put_le16(p, (uint16_t)time_ds);
    p += 2;

    /* Byte 14: battery voltage, uint8 */
    float batt_encoded = (state->batt_v - BATT_OFFSET_V) / BATT_STEP_V;
    if (batt_encoded < 0.0f) batt_encoded = 0.0f;
    if (batt_encoded > 255.0f) batt_encoded = 255.0f;
    *p++ = (uint8_t)batt_encoded;

    /* Byte 15: sequence counter */
    *p++ = s_seq++;

    /* Bytes 16–19: CRC-32 over bytes 0–15 */
    uint32_t crc = crc32_hw_compute(s_raw_buf, 16);
    put_le32(p, crc);

    return cobs_encode_and_send(s_raw_buf, SIZE_FC_MSG_FAST);
}

int tlm_send_gps(const fc_gps_state_t *gps_state)
{
    /* FC_MSG_GPS: 17 bytes per INTERFACE_SPEC §5.2
     * [0]=0x02 [1-4]=dlat_mm(i32) [5-8]=dlon_mm(i32)
     * [9-10]=alt_msl(u16) [11]=fix_type [12]=sat_count [13-16]=CRC */
    uint8_t *p = s_raw_buf;

    /* Byte 0: message ID */
    *p++ = MSG_ID_GPS;

    /* Bytes 1–4: dlat_mm, int32 LE (millimetres from pad) */
    put_le32(p, (uint32_t)gps_state->dlat_mm);
    p += 4;

    /* Bytes 5–8: dlon_mm, int32 LE (millimetres from pad) */
    put_le32(p, (uint32_t)gps_state->dlon_mm);
    p += 4;

    /* Bytes 9–10: alt_msl in decametres, uint16 LE */
    float alt_dam = gps_state->alt_msl_m / ALT_SCALE_M;
    if (alt_dam < 0.0f) alt_dam = 0.0f;
    if (alt_dam > 65535.0f) alt_dam = 65535.0f;
    put_le16(p, (uint16_t)alt_dam);
    p += 2;

    /* Byte 11: fix_type */
    *p++ = gps_state->fix_type;

    /* Byte 12: sat_count */
    *p++ = gps_state->sat_count;

    /* Bytes 13–16: CRC-32 over bytes 0–12 */
    uint32_t crc = crc32_hw_compute(s_raw_buf, 13);
    put_le32(p, crc);

    return cobs_encode_and_send(s_raw_buf, SIZE_FC_MSG_GPS);
}

int tlm_queue_event(uint8_t type, uint16_t data)
{
    /* FC_MSG_EVENT: 11 bytes per INTERFACE_SPEC §5.3
     * [0]=0x03 [1]=type [2-3]=data [4-5]=time [6]=rsvd [7-10]=CRC */
    uint8_t *p = s_raw_buf;

    /* Byte 0: message ID */
    *p++ = MSG_ID_EVENT;

    /* Byte 1: event type */
    *p++ = type;

    /* Bytes 2–3: event data, uint16 LE */
    put_le16(p, data);
    p += 2;

    /* Bytes 4–5: event time (0.1s ticks), uint16 LE */
    extern float flight_fsm_get_time_s(void);
    float time_ds = flight_fsm_get_time_s() / TIME_SCALE_100MS;
    if (time_ds < 0.0f) time_ds = 0.0f;
    if (time_ds > 65535.0f) time_ds = 65535.0f;
    put_le16(p, (uint16_t)time_ds);
    p += 2;

    /* Byte 6: reserved */
    *p++ = 0x00;

    /* Bytes 7–10: CRC-32 over bytes 0–6 */
    uint32_t crc = crc32_hw_compute(s_raw_buf, 7);
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
