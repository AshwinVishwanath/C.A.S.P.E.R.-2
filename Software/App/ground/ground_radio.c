/* ============================================================
 *  TIER:     GROUND-STATION
 *  MODULE:   Ground Radio
 *  SUMMARY:  RX-continuous parser, ASCII output, command relay.
 * ============================================================ */
/**
 * @file ground_radio.c
 * @brief Ground station radio: RX-continuous, packet parsing,
 *        ASCII debug output (GS_OUTPUT=ASCII, default) or binary COBS relay
 *        (GS_OUTPUT=COBS), profile switching, command relay.
 *
 * Output mode is selected at compile time via -DGS_OUTPUT_COBS:
 *   ASCII mode (default): human-readable ">..." lines via USB CDC
 *   COBS mode:            binary COBS-framed packets via USB CDC (no ASCII)
 *
 * CLAUDE.md rule #4 enforced: the two modes are mutually exclusive on
 * the shared USB CDC pipe.  All ASCII gs_cdc_print / snprintf->CDC paths
 * are suppressed when GS_OUTPUT_COBS is defined.
 */

#include "ground_radio.h"
#include "sx1276.h"
#include "radio_config.h"
#include "radio_irq.h"
#include "crc32_hw.h"
#include "cobs.h"          /* always compiled: needed by ground_radio_cobs_send() */
#include "tlm_types.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

#ifdef GS_OUTPUT_COBS
#include "quat_pack.h"
#include "casper_quat.h"
#include <math.h>
#endif /* GS_OUTPUT_COBS */

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

static gs_radio_stats_t   s_stats;
static gs_profile_state_t s_profile_state;
static uint32_t           s_last_valid_rx_ms;
static uint8_t            s_rx_buf[RADIO_MAX_PACKET_SIZE];
static uint8_t            s_tx_pending;  /* 1 if waiting for TX_DONE */

#define GS_PROFILE_LOSS_TIMEOUT_MS  2000

/* COBS relay: GS sequence counter + raw packet assembly buffer.
 * Only compiled when GS_OUTPUT=COBS to avoid unused-variable warnings
 * (CLAUDE.md rule #6). */
#ifdef GS_OUTPUT_COBS
static uint8_t s_gs_seq;
/* 0x10 packet assembly: SIZE_GS_MSG_TELEM = 39 bytes */
static uint8_t s_gs_raw_buf[SIZE_GS_MSG_TELEM];
#endif /* GS_OUTPUT_COBS */

/* ------------------------------------------------------------------ */
/*  Helpers — ASCII decode (suppressed in COBS mode to avoid warnings)*/
/* ------------------------------------------------------------------ */

#ifndef GS_OUTPUT_COBS

static const char *fsm_state_name(uint8_t st)
{
    static const char *names[] = {
        "PAD", "BOOST", "COAST", "COAST1",
        "SUSTAIN", "COAST2", "APOGEE", "DROGUE",
        "MAIN", "RECOVERY", "TUMBLE", "LANDED"
    };
    if (st < 12) return names[st];
    return "UNK";
}

static const char *event_name(uint8_t evt)
{
    switch (evt) {
    case FC_EVT_STATE:   return "STATE";
    case FC_EVT_PYRO:    return "PYRO";
    case FC_EVT_APOGEE:  return "APOGEE";
    case FC_EVT_ERROR:   return "ERROR";
    case FC_EVT_ORIGIN:  return "ORIGIN";
    case FC_EVT_BURNOUT: return "BURNOUT";
    case FC_EVT_STAGING: return "STAGING";
    case FC_EVT_ARM:     return "ARM";
    default:             return "UNK";
    }
}

static uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t get_le16_signed(const uint8_t *p)
{
    return (int16_t)get_le16(p);
}

static uint32_t get_le24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t get_le32_signed(const uint8_t *p)
{
    return (int32_t)get_le32(p);
}

static char s_cdc_buf[160];

static void gs_cdc_print(const char *str, int len)
{
    CDC_Transmit_FS((uint8_t *)str, (uint16_t)len);
}

#else /* GS_OUTPUT_COBS */

/* get_le32 is always needed in COBS mode for CRC validation */
static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#endif /* !GS_OUTPUT_COBS */

/* ------------------------------------------------------------------ */
/*  Shared COBS sender (always compiled; used by ground_main.c        */
/*  for any 0x13 status heartbeat when caller wants binary output)    */
/* ------------------------------------------------------------------ */

int ground_radio_cobs_send(const uint8_t *raw, int len)
{
    /* COBS overhead: at most 1 extra byte per 254 raw bytes + 1 delimiter.
     * Largest GS packet: SIZE_GS_MSG_TELEM = 39 bytes
     *   -> encoded max 40 bytes + 1 delimiter = 41 bytes.
     * Buffer is SIZE_GS_MSG_TELEM + 3 = 42 bytes for safety margin.
     * Mirrors pattern in tlm_manager.c: encode -> append 0x00 -> CDC_Transmit_FS. */
    static uint8_t cobs_buf[SIZE_GS_MSG_TELEM + 3];
    int enc_len = cobs_encode(raw, len, cobs_buf, (int)(sizeof(cobs_buf) - 1U));
    if (enc_len < 0) return 0;
    cobs_buf[enc_len] = 0x00;  /* COBS packet delimiter */
    enc_len++;
    return (CDC_Transmit_FS(cobs_buf, (uint16_t)enc_len) == USBD_OK) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */

int ground_radio_init(SPI_HandleTypeDef *hspi)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_profile_state    = GS_PROFILE_AWAITING_FIRST;
    s_last_valid_rx_ms = 0;
    s_tx_pending       = 0;
#ifdef GS_OUTPUT_COBS
    s_gs_seq           = 0;
#endif

    if (sx1276_init(hspi) != 0) return -1;

    sx1276_set_lora_mode();

    /* DIO mapping: DIO0=RxDone/TxDone, DIO1=RxTimeout */
    sx1276_write_reg(SX1276_REG_DIO_MAPPING_1, RADIO_DIO_MAPPING_1);
    sx1276_write_reg(SX1276_REG_DIO_MAPPING_2, RADIO_DIO_MAPPING_2);

    /* FIFO base addresses */
    sx1276_write_reg(SX1276_REG_FIFO_TX_BASE_ADDR, SX1276_FIFO_TX_BASE);
    sx1276_write_reg(SX1276_REG_FIFO_RX_BASE_ADDR, SX1276_FIFO_RX_BASE);

    /* Enable hardware CRC on RX */
    uint8_t mc2 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG_2);
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG_2, mc2 | SX1276_RX_CRC_ON);

    /* Apply Profile A (SF7, BW250, CR4/5) */
    sx1276_set_frequency(RADIO_PROFILE_A.freq_hz);
    sx1276_set_modulation(RADIO_PROFILE_A.sf, RADIO_PROFILE_A.bw_hz,
                          RADIO_PROFILE_A.cr);
    sx1276_set_sync_word(RADIO_PROFILE_A.sync_word);
    sx1276_set_preamble(RADIO_PROFILE_A.preamble);
    sx1276_set_tx_power(RADIO_PROFILE_A.tx_power_dbm);
    s_stats.current_profile = 0;

    radio_irq_clear_all();

    /* Enter RX-continuous mode */
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, SX1276_FIFO_RX_BASE);
    sx1276_clear_irq_flags(SX1276_IRQ_ALL);
    sx1276_set_mode(SX1276_MODE_RXCONTINUOUS);

    return 0;
}

/* ------------------------------------------------------------------ */
/*  RX handler                                                         */
/* ------------------------------------------------------------------ */

void ground_radio_on_rx(void)
{
    uint8_t irq = sx1276_get_irq_flags();
    sx1276_clear_irq_flags(SX1276_IRQ_ALL);
    g_radio_dio0_flag = 0;

    /* HW CRC error */
    if (irq & SX1276_IRQ_PAYLOAD_CRC_ERROR) {
        s_stats.rx_crc_fail++;
        /* Stay in RX-continuous — no mode change needed */
        return;
    }

    /* Read packet from FIFO */
    uint8_t nb   = sx1276_read_reg(SX1276_REG_RX_NB_BYTES);
    uint8_t addr = sx1276_read_reg(SX1276_REG_FIFO_RX_CURRENT_ADDR);
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, addr);

    if (nb > RADIO_MAX_PACKET_SIZE) nb = RADIO_MAX_PACKET_SIZE;
    sx1276_read_fifo(s_rx_buf, nb);

    /* Minimum: 1 ID + 4 CRC = 5 bytes */
    if (nb < 5) return;

    /* Read RSSI + SNR before any further SPI ops */
    int16_t rssi_raw = sx1276_get_packet_rssi();
    s_stats.last_rssi = (rssi_raw < -128) ? -128 : (int8_t)rssi_raw;
    s_stats.last_snr  = sx1276_get_packet_snr();

    /* ── Ping-pong responder (bench two-way link test with Casper-3) ─────
     * PING packet = [0xCA][0x53][0x01][seq_lo][seq_hi][sum8]; reply with a
     * PONG (type 0x02, same seq).  This bypasses the telemetry CRC-32 framing,
     * so it must be handled BEFORE the CRC-32 check below (and returns after).
     * The LoRa hardware CRC already guaranteed integrity of these bytes. */
    if (nb >= 6u &&
        s_rx_buf[0] == 0xCAu && s_rx_buf[1] == 0x53u && s_rx_buf[2] == 0x01u &&
        s_rx_buf[5] == (uint8_t)(s_rx_buf[0] + s_rx_buf[1] + s_rx_buf[2] +
                                 s_rx_buf[3] + s_rx_buf[4])) {
        uint16_t seq = (uint16_t)s_rx_buf[3] | ((uint16_t)s_rx_buf[4] << 8);
        uint8_t pong[6] = { 0xCAu, 0x53u, 0x02u, s_rx_buf[3], s_rx_buf[4], 0u };
        pong[5] = (uint8_t)(pong[0] + pong[1] + pong[2] + pong[3] + pong[4]);
        ground_radio_send_cmd(pong, 6u);   /* TX; check_tx_done() returns to RX */
        s_stats.rx_pkt_count++;
        s_last_valid_rx_ms = HAL_GetTick();
#ifndef GS_OUTPUT_COBS
        char l[80];
        int n = snprintf(l, sizeof(l), ">GS PING %u -> PONG  rssi %d snr %d\r\n",
                         (unsigned)seq, (int)s_stats.last_rssi, (int)s_stats.last_snr);
        if (n > 0) { gs_cdc_print(l, n); }
#endif
        return;                            /* handled — skip telemetry decode */
    }

    /* Validate CRC-32 */
    uint32_t payload_len = (uint32_t)(nb - 4);
    uint32_t computed    = crc32_hw_compute(s_rx_buf, payload_len);
    uint32_t received    = get_le32(&s_rx_buf[payload_len]);
    if (computed != received) {
        s_stats.rx_crc_fail++;
        return;
    }

    /* Valid packet — update stats and profile timer */
    s_stats.rx_pkt_count++;
    s_last_valid_rx_ms = HAL_GetTick();

    if (s_profile_state == GS_PROFILE_AWAITING_FIRST) {
        s_profile_state = GS_PROFILE_A_ACTIVE;
    }

    uint8_t msg_id = s_rx_buf[0];

#ifdef GS_OUTPUT_COBS
    /* ── COBS relay mode (GS_OUTPUT=COBS) ──────────────────────────
     * FC_MSG_FAST (0x01): build GS_MSG_TELEM (0x10, 39 bytes) and send.
     * All other IDs (GPS 0x02, EVENT 0x03, ACK/NACK/CONFIRM): relay the
     * raw received packet COBS-framed; MC parses those msg_ids natively. */

    if (msg_id == MSG_ID_FAST && nb >= SIZE_FC_MSG_FAST) {
        /* Build GS_MSG_TELEM (0x10) per canonical wire layout (LE, CRC last 4 B):
         * [ID:1][FC_RELAY:15][SEQ:1][RSSI:2][SNR:1][FREQ_ERR:2][DATA_AGE:2]
         * [RECOV:1][MACH:2][QBAR:2][ROLL:2][PITCH:2][YAW:2][CRC:4] = 39 bytes */
        uint8_t *p = s_gs_raw_buf;

        /* [0] msg_id = 0x10 */
        *p++ = MSG_ID_GS_TELEM;

        /* [1..15] FC relay — copy verbatim from received FC_MSG_FAST.
         *          Covers: status u16, alt u24, vel i16, quat 5B, time u16, batt u8. */
        memcpy(p, &s_rx_buf[1], 15);
        p += 15;

        /* [16] GS sequence — incremented once per relayed telem packet */
        *p++ = s_gs_seq++;

        /* [17..18] RSSI i16 LE, encoded x0.1 dBm
         *          raw = (int16)(last_rssi_dBm * 10) */
        {
            int16_t rssi_enc = (int16_t)((int32_t)s_stats.last_rssi * 10);
            p[0] = (uint8_t)((uint16_t)rssi_enc & 0xFFU);
            p[1] = (uint8_t)(((uint16_t)rssi_enc >> 8) & 0xFFU);
            p += 2;
        }

        /* [19] SNR i8, encoded x0.25 dB.
         *      sx1276_get_packet_snr() returns (reg_val / 4) i.e. whole dB.
         *      Multiply by 4 to restore 0.25 dB resolution; clamp to int8. */
        {
            int16_t snr_q   = (int16_t)s_stats.last_snr * 4;
            int8_t  snr_enc = (snr_q > 127)  ? (int8_t)127  :
                              (snr_q < -128) ? (int8_t)(-128) : (int8_t)snr_q;
            *p++ = (uint8_t)snr_enc;
        }

        /* [20..21] freq_err i16 LE = 0 (not populated yet) */
        p[0] = 0; p[1] = 0; p += 2;

        /* [22..23] data_age u16 LE ms = min(now - last_valid_rx_ms, 65535)
         *          HAL_GetTick() replaces casper_millis() (no port seam on this branch) */
        {
            uint32_t now_ms  = HAL_GetTick();
            uint32_t age_ms  = now_ms - s_last_valid_rx_ms;
            uint16_t age_u16 = (age_ms > 65535U) ? 65535U : (uint16_t)age_ms;
            p[0] = (uint8_t)(age_u16 & 0xFFU);
            p[1] = (uint8_t)((age_u16 >> 8) & 0xFFU);
            p += 2;
        }

        /* [24] recovery = 0 (not populated yet) */
        *p++ = 0;

        /* [25..26] mach u16 LE, [27..28] qbar u16 LE.
         * Derived locally from the FAST altitude and velocity already in s_rx_buf.
         * Approximation: ISA troposphere uses AGL altitude as MSL proxy;
         * vertical EKF velocity is used as speed proxy; qbar clamped to u16 max. */
        {
            /* Assemble raw fields directly from buffer bytes — avoids the
             * ASCII-only get_le* helpers that are #ifndef GS_OUTPUT_COBS-guarded. */
            uint32_t isa_alt_raw = (uint32_t)s_rx_buf[3]
                                 | ((uint32_t)s_rx_buf[4] << 8)
                                 | ((uint32_t)s_rx_buf[5] << 16);
            float isa_alt_m  = (float)isa_alt_raw * 0.01f;             /* u24 cm -> m (AGL) */
            int16_t isa_vraw = (int16_t)((uint16_t)s_rx_buf[6]
                                        | ((uint16_t)s_rx_buf[7] << 8));
            float isa_speed  = fabsf((float)isa_vraw * 0.1f);          /* i16 dm/s -> m/s */

            /* ISA standard atmosphere, troposphere (0..11 km) */
            const float T0    = 288.15f;    /* K   sea-level temp        */
            const float L     = 0.0065f;    /* K/m lapse rate            */
            const float P0    = 101325.0f;  /* Pa  sea-level pressure    */
            const float Rs    = 287.05f;    /* J/(kg*K) gas const for air*/
            const float GAMMA = 1.4f;
            float isa_h = isa_alt_m;
            if (isa_h < 0.0f)     isa_h = 0.0f;
            if (isa_h > 11000.0f) isa_h = 11000.0f;
            float isa_T   = T0 - L * isa_h;
            float isa_P   = P0 * powf(isa_T / T0, 5.25588f); /* exponent = g*M/(R*L) */
            float isa_rho = isa_P / (Rs * isa_T);
            float isa_a   = sqrtf(GAMMA * Rs * isa_T);       /* speed of sound */
            float qbar_f  = 0.5f * isa_rho * isa_speed * isa_speed; /* dynamic pressure Pa */
            float mach_f  = (isa_a > 0.0f) ? (isa_speed / isa_a) : 0.0f;

            long mach_long = lroundf(mach_f * 1000.0f);      /* x0.001 per LSB */
            long qbar_long = lroundf(qbar_f);                 /* Pa per LSB     */
            uint16_t mach_u16 = (mach_long > 65535L) ? 65535U
                              : (mach_long < 0L)     ? 0U
                              : (uint16_t)mach_long;
            uint16_t qbar_u16 = (qbar_long > 65535L) ? 65535U
                              : (qbar_long < 0L)     ? 0U
                              : (uint16_t)qbar_long;
            p[0] = (uint8_t)(mach_u16 & 0xFFU);
            p[1] = (uint8_t)((mach_u16 >> 8) & 0xFFU);
            p += 2;
            p[0] = (uint8_t)(qbar_u16 & 0xFFU);
            p[1] = (uint8_t)((qbar_u16 >> 8) & 0xFFU);
            p += 2;
        }

        /* [29..30] roll, [31..32] pitch, [33..34] yaw
         * Decode the packed quaternion (verbatim from s_rx_buf[8..12]) and
         * compute ZYX Euler using the FC-authoritative casper_quat_to_euler.
         * casper_quat_to_euler convention: e[0]=bodyZ(heading), e[1]=bodyY(roll),
         *   e[2]=bodyX(pitch) — matches the tilt-from-vertical convention on this branch.
         * Wire encoding: i16 LE x0.1 deg, raw = round(deg * 10), clamped to i16. */
        {
            float q_gs[4];
            quat_unpack_smallest_three(&s_rx_buf[8], q_gs);
            float e_gs[3];
            casper_quat_to_euler(q_gs, e_gs);
            long roll_raw  = lroundf(e_gs[1] * 10.0f);  /* bodyY */
            long pitch_raw = lroundf(e_gs[2] * 10.0f);  /* bodyX */
            long yaw_raw   = lroundf(e_gs[0] * 10.0f);  /* bodyZ */
            int16_t roll_enc  = (roll_raw  >  32767L) ? (int16_t) 32767 :
                                (roll_raw  < -32768L) ? (int16_t)(-32768) :
                                                        (int16_t)roll_raw;
            int16_t pitch_enc = (pitch_raw >  32767L) ? (int16_t) 32767 :
                                (pitch_raw < -32768L) ? (int16_t)(-32768) :
                                                        (int16_t)pitch_raw;
            int16_t yaw_enc   = (yaw_raw   >  32767L) ? (int16_t) 32767 :
                                (yaw_raw   < -32768L) ? (int16_t)(-32768) :
                                                        (int16_t)yaw_raw;
            p[0] = (uint8_t)((uint16_t)roll_enc & 0xFFU);
            p[1] = (uint8_t)(((uint16_t)roll_enc >> 8) & 0xFFU);
            p += 2;
            p[0] = (uint8_t)((uint16_t)pitch_enc & 0xFFU);
            p[1] = (uint8_t)(((uint16_t)pitch_enc >> 8) & 0xFFU);
            p += 2;
            p[0] = (uint8_t)((uint16_t)yaw_enc & 0xFFU);
            p[1] = (uint8_t)(((uint16_t)yaw_enc >> 8) & 0xFFU);
            p += 2;
        }

        /* [35..38] CRC-32 over bytes [0..34] (SIZE_GS_MSG_TELEM - 4 = 35 bytes).
         * CLAUDE.md rule #1: CRC range covers [0 .. N-5] for 4-byte CRC at end. */
        {
            uint32_t crc = crc32_hw_compute(s_gs_raw_buf, SIZE_GS_MSG_TELEM - 4);
            p[0] = (uint8_t)(crc & 0xFFU);
            p[1] = (uint8_t)((crc >> 8)  & 0xFFU);
            p[2] = (uint8_t)((crc >> 16) & 0xFFU);
            p[3] = (uint8_t)((crc >> 24) & 0xFFU);
        }

        ground_radio_cobs_send(s_gs_raw_buf, SIZE_GS_MSG_TELEM);

    } else {
        /* GPS (0x02), EVENT (0x03), ACK/NACK/CONFIRM, or unknown:
         * relay the raw received FC packet COBS-framed — MC parses natively. */
        ground_radio_cobs_send(s_rx_buf, (int)nb);
    }

#else /* GS_OUTPUT_COBS */

    /* ── ASCII relay mode (default, GS_OUTPUT=ASCII) ─────────────── */
    int len = 0;

    switch (msg_id) {
    case MSG_ID_FAST: {
        if (nb < SIZE_FC_MSG_FAST) break;
        /* Unpack status bitmap */
        uint8_t status_b1 = s_rx_buf[2];
        uint8_t fsm_st    = (status_b1 >> 4) & 0x0F;
        /* Unpack altitude (u24 cm) and velocity (dm/s) */
        uint32_t alt_raw = get_le24(&s_rx_buf[3]);
        int16_t  vel_raw = get_le16_signed(&s_rx_buf[6]);
        float alt_m   = (float)alt_raw * ALT_SCALE_M;
        float vel_mps = (float)vel_raw * VEL_SCALE_DMS;
        /* Unpack flight time (0.1s ticks) */
        uint16_t time_raw = get_le16(&s_rx_buf[13]);
        float time_s = (float)time_raw * TIME_SCALE_100MS;
        /* Battery */
        uint8_t batt_raw = s_rx_buf[15];
        float batt_v = BATT_OFFSET_V + (float)batt_raw * BATT_STEP_V;
        /* Sequence */
        uint8_t seq = s_rx_buf[16];

        len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">FAST ALT:%.2f VEL:%.1f ST:%s T:%.1f BATT:%.2f SEQ:%u RSSI:%d SNR:%d\r\n",
            alt_m, vel_mps, fsm_state_name(fsm_st), time_s, batt_v,
            seq, (int)s_stats.last_rssi, (int)s_stats.last_snr);
        break;
    }
    case MSG_ID_GPS: {
        if (nb < SIZE_FC_MSG_GPS) break;
        int32_t  dlat_mm = get_le32_signed(&s_rx_buf[1]);
        int32_t  dlon_mm = get_le32_signed(&s_rx_buf[5]);
        uint32_t alt_raw = get_le24(&s_rx_buf[9]);
        float    alt_m   = (float)alt_raw * ALT_SCALE_M;
        uint8_t  fix     = s_rx_buf[12];
        uint8_t  sats    = s_rx_buf[13];

        const char *fix_str = (fix == 3) ? "3D" :
                              (fix == 2) ? "2D" : "NONE";
        len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">GPS DLAT:%ld DLON:%ld ALT:%.2f FIX:%s SAT:%u RSSI:%d\r\n",
            (long)dlat_mm, (long)dlon_mm, alt_m, fix_str, sats,
            (int)s_stats.last_rssi);
        break;
    }
    case MSG_ID_EVENT: {
        if (nb < SIZE_FC_MSG_EVENT) break;
        uint8_t  etype = s_rx_buf[1];
        uint16_t edata = get_le16(&s_rx_buf[2]);
        uint16_t etime = get_le16(&s_rx_buf[4]);

        len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">EVT %s DATA:%u T:%u RSSI:%d\r\n",
            event_name(etype), edata, etime, (int)s_stats.last_rssi);
        break;
    }
    case MSG_ID_ACK_ARM:
    case MSG_ID_ACK_FIRE:
    case MSG_ID_NACK:
    case MSG_ID_CONFIRM: {
        len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">RESP ID:0x%02X LEN:%u RSSI:%d SNR:%d\r\n",
            msg_id, nb, (int)s_stats.last_rssi, (int)s_stats.last_snr);
        break;
    }
    default:
        len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">UNK ID:0x%02X LEN:%u RSSI:%d\r\n",
            msg_id, nb, (int)s_stats.last_rssi);
        break;
    }

    if (len > 0) {
        gs_cdc_print(s_cdc_buf, len);
    }

#endif /* GS_OUTPUT_COBS */
}

/* ------------------------------------------------------------------ */
/*  Profile switching tick                                             */
/* ------------------------------------------------------------------ */

void ground_radio_profile_tick(void)
{
    if (s_profile_state != GS_PROFILE_A_ACTIVE) return;

    uint32_t now = HAL_GetTick();
    if (now - s_last_valid_rx_ms >= GS_PROFILE_LOSS_TIMEOUT_MS) {
        /* Switch to Profile B (SF8) — one-way, never switch back */
        sx1276_set_mode(SX1276_MODE_STDBY);
        sx1276_set_modulation(RADIO_PROFILE_B.sf, RADIO_PROFILE_B.bw_hz,
                              RADIO_PROFILE_B.cr);

        /* Back to RX-continuous */
        sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, SX1276_FIFO_RX_BASE);
        sx1276_clear_irq_flags(SX1276_IRQ_ALL);
        sx1276_set_mode(SX1276_MODE_RXCONTINUOUS);

        s_stats.current_profile = 1;
        s_profile_state = GS_PROFILE_B_ACTIVE;

#ifndef GS_OUTPUT_COBS
        int len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">GS PROFILE_SWITCH A->B (loss timeout)\r\n");
        gs_cdc_print(s_cdc_buf, len);
#endif
    }
}

/* ------------------------------------------------------------------ */
/*  Command relay (CDC -> LoRa TX)                                     */
/* ------------------------------------------------------------------ */

int ground_radio_send_cmd(const uint8_t *buf, uint8_t len)
{
    if (s_tx_pending) return -1;
    if (len > RADIO_MAX_PACKET_SIZE) return -1;

    sx1276_set_mode(SX1276_MODE_STDBY);
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, SX1276_FIFO_TX_BASE);
    sx1276_write_fifo(buf, len);
    sx1276_write_reg(SX1276_REG_PAYLOAD_LENGTH, len);

    sx1276_clear_irq_flags(SX1276_IRQ_ALL);
    g_radio_dio0_flag = 0;
    sx1276_set_mode(SX1276_MODE_TX);

    s_tx_pending = 1;
    return 0;
}

void ground_radio_check_tx_done(void)
{
    if (!s_tx_pending) return;

    /* DIO0 is hard-mapped to RxDone (RegDioMapping1 bits[7:6]=00), so a TxDone
     * NEVER raises DIO0. Poll the SX1276 IRQ-flags register directly for the
     * TX_DONE bit instead — otherwise the GS never learns the reply finished,
     * stays parked in TX, and goes deaf after its first transmit. */
    uint8_t irq = sx1276_get_irq_flags();
    if (irq & SX1276_IRQ_TX_DONE) {
        sx1276_clear_irq_flags(SX1276_IRQ_ALL);
        g_radio_dio0_flag = 0;

        /* Return to RX-continuous */
        sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, SX1276_FIFO_RX_BASE);
        sx1276_clear_irq_flags(SX1276_IRQ_ALL);
        sx1276_set_mode(SX1276_MODE_RXCONTINUOUS);

        s_tx_pending = 0;

#ifndef GS_OUTPUT_COBS
        int len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">GS TX_DONE\r\n");
        gs_cdc_print(s_cdc_buf, len);
#endif
    }
}

/* ------------------------------------------------------------------ */
/*  Stats accessor                                                     */
/* ------------------------------------------------------------------ */

const gs_radio_stats_t *ground_radio_get_stats(void)
{
    return &s_stats;
}

/* ------------------------------------------------------------------ */
/*  TX-in-flight query                                                 */
/* ------------------------------------------------------------------ */
/* DIO0 is shared for RxDone and TxDone. The main loop uses this to route
 * a DIO0 edge to check_tx_done() while a TX is pending, instead of
 * misrouting the TxDone into on_rx() (which would clear the flag before
 * check_tx_done() could re-arm RX, leaving the GS stuck in TX and deaf). */
int ground_radio_tx_pending(void)
{
    return (int)s_tx_pending;
}
