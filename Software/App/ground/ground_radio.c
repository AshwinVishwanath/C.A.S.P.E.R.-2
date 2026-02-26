/**
 * @file ground_radio.c
 * @brief Ground station radio: RX-continuous, packet parsing,
 *        ASCII debug output, profile switching, command relay.
 */

#include "ground_radio.h"
#include "sx1276.h"
#include "radio_config.h"
#include "radio_irq.h"
#include "crc32_hw.h"
#include "tlm_types.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

static gs_radio_stats_t   s_stats;
static gs_profile_state_t s_profile_state;
static uint32_t           s_last_valid_rx_ms;
static uint8_t            s_rx_buf[RADIO_MAX_PACKET_SIZE];
static uint8_t            s_tx_pending;  /* 1 if waiting for TX_DONE */

#define GS_PROFILE_LOSS_TIMEOUT_MS  2000

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */

int ground_radio_init(SPI_HandleTypeDef *hspi)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_profile_state   = GS_PROFILE_AWAITING_FIRST;
    s_last_valid_rx_ms = 0;
    s_tx_pending       = 0;

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

    /* Parse and output ASCII based on message ID */
    uint8_t msg_id = s_rx_buf[0];
    int len = 0;

    switch (msg_id) {
    case MSG_ID_FAST: {
        if (nb < SIZE_FC_MSG_FAST) break;
        /* Unpack status bitmap */
        uint8_t status_b1 = s_rx_buf[2];
        uint8_t fsm_st    = (status_b1 >> 4) & 0x0F;
        /* Unpack altitude (decametres) and velocity (dm/s) */
        uint16_t alt_raw = get_le16(&s_rx_buf[3]);
        int16_t  vel_raw = get_le16_signed(&s_rx_buf[5]);
        float alt_m   = (float)alt_raw * ALT_SCALE_M;
        float vel_mps = (float)vel_raw * VEL_SCALE_DMS;
        /* Unpack flight time (0.1s ticks) */
        uint16_t time_raw = get_le16(&s_rx_buf[12]);
        float time_s = (float)time_raw * TIME_SCALE_100MS;
        /* Battery */
        uint8_t batt_raw = s_rx_buf[14];
        float batt_v = BATT_OFFSET_V + (float)batt_raw * BATT_STEP_V;
        /* Sequence */
        uint8_t seq = s_rx_buf[15];

        len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">FAST ALT:%.1f VEL:%.1f ST:%s T:%.1f BATT:%.2f SEQ:%u RSSI:%d SNR:%d\r\n",
            alt_m, vel_mps, fsm_state_name(fsm_st), time_s, batt_v,
            seq, (int)s_stats.last_rssi, (int)s_stats.last_snr);
        break;
    }
    case MSG_ID_GPS: {
        if (nb < SIZE_FC_MSG_GPS) break;
        int32_t  dlat_mm = get_le32_signed(&s_rx_buf[1]);
        int32_t  dlon_mm = get_le32_signed(&s_rx_buf[5]);
        uint16_t alt_raw = get_le16(&s_rx_buf[9]);
        uint8_t  fix     = s_rx_buf[11];
        uint8_t  sats    = s_rx_buf[12];

        const char *fix_str = (fix == 3) ? "3D" :
                              (fix == 2) ? "2D" : "NONE";
        len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">GPS DLAT:%ld DLON:%ld ALT:%u FIX:%s SAT:%u RSSI:%d\r\n",
            (long)dlat_mm, (long)dlon_mm, alt_raw, fix_str, sats,
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

        int len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">GS PROFILE_SWITCH A->B (loss timeout)\r\n");
        gs_cdc_print(s_cdc_buf, len);
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

    if (g_radio_dio0_flag) {
        sx1276_clear_irq_flags(SX1276_IRQ_ALL);
        g_radio_dio0_flag = 0;

        /* Return to RX-continuous */
        sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, SX1276_FIFO_RX_BASE);
        sx1276_clear_irq_flags(SX1276_IRQ_ALL);
        sx1276_set_mode(SX1276_MODE_RXCONTINUOUS);

        s_tx_pending = 0;

        int len = snprintf(s_cdc_buf, sizeof(s_cdc_buf),
            ">GS TX_DONE\r\n");
        gs_cdc_print(s_cdc_buf, len);
    }
}

/* ------------------------------------------------------------------ */
/*  Stats accessor                                                     */
/* ------------------------------------------------------------------ */

const gs_radio_stats_t *ground_radio_get_stats(void)
{
    return &s_stats;
}
