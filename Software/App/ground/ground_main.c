/**
 * @file ground_main.c
 * @brief Ground station main application: radio RX, sensor polling,
 *        USB CDC ASCII output (GS_OUTPUT=ASCII, default) or binary COBS
 *        output (GS_OUTPUT=COBS for Mission Control), 1Hz GS status heartbeat.
 *
 * COBS mode (GS_OUTPUT_COBS defined): ALL CDC output is binary-framed.
 *   No ASCII text is emitted on the CDC pipe (CLAUDE.md rule #4).
 *   1Hz heartbeat becomes a binary GS_MSG_STATUS (0x13, 24 bytes).
 * ASCII mode (default): behavior is identical to the original.
 */

#include "ground_main.h"
#include "ground_radio.h"
#include "radio_irq.h"
#include "casper_port.h"
#include "board_casper2.h"
#include "usbd_cdc_if.h"
#include "ms5611.h"
#include "max_m10m.h"
#include "buzzer.h"
#include "tlm_types.h"
#include "crc32_hw.h"
#include <stdio.h>
#include <string.h>

/* ── Module state ─────────────────────────────────────────────────── */

static ms5611_t   *s_baro;
static max_m10m_t *s_gps;

static uint32_t s_last_baro_ms;
static uint32_t s_last_gps_ms;
static uint32_t s_last_status_ms;

/* Latest sensor values */
static float s_ground_pressure_pa;
static float s_ground_lat_deg;
static float s_ground_lon_deg;

#ifndef GS_OUTPUT_COBS
/* CDC output buffer (ASCII mode only — unused in COBS mode) */
static char s_status_buf[200];
#endif

/* ── Init ─────────────────────────────────────────────────────────── */

void ground_main_init(void *hspi1_opaque, ms5611_t *baro, max_m10m_t *gps)
{
    s_baro = baro;
    s_gps  = gps;
    s_ground_pressure_pa = 0.0f;
    s_ground_lat_deg     = 0.0f;
    s_ground_lon_deg     = 0.0f;

    uint32_t now = casper_millis();
    s_last_baro_ms   = now;
    s_last_gps_ms    = now;
    s_last_status_ms = now;

    /* Init radio in RX-continuous mode */
    int rc = ground_radio_init(hspi1_opaque);

#ifndef GS_OUTPUT_COBS
    /* Report init result over CDC (ASCII mode only — COBS mode emits no ASCII) */
    int len = snprintf(s_status_buf, sizeof(s_status_buf),
        ">GS RADIO_INIT:%s\r\n", (rc == 0) ? "OK" : "FAIL");
    CDC_Transmit_FS((uint8_t *)s_status_buf, (uint16_t)len);
#else
    (void)rc;   /* init result not reported on CDC in COBS mode */
#endif

    /* Short beep to indicate GS is ready */
    buzzer_beep_n(30, 1, 100, 200);
}

/* ── Tick ──────────────────────────────────────────────────────────── */

void ground_main_tick(void)
{
    uint32_t now = casper_millis();

    /* ── 0. Poll DIO0/DIO1 GPIOs (EXTI disabled — polling mode) ── */
    if (casper_gpio_read(BSP_PIN_RADIO_DIO0) == CASPER_PIN_HIGH)
        g_radio_dio0_flag = 1;
    if (casper_gpio_read(BSP_PIN_RADIO_DIO1) == CASPER_PIN_HIGH)
        g_radio_dio1_flag = 1;

    /* ── 1. Radio RX: check for received packets ──────────────── */
    if (g_radio_dio0_flag) {
        ground_radio_on_rx();
        /* Note: g_radio_dio0_flag is cleared inside ground_radio_on_rx() */
    }

    /* ── 2. Radio TX completion check ─────────────────────────── */
    ground_radio_check_tx_done();

    /* ── 3. Profile switch tick ───────────────────────────────── */
    ground_radio_profile_tick();

    /* ── 4. MS5611 barometer: tick state machine + 10Hz read ──── */
    ms5611_tick(s_baro);
    if (now - s_last_baro_ms >= 100) {
        s_ground_pressure_pa = (float)s_baro->pressure;
        s_last_baro_ms = now;
    }

    /* ── 5. GPS: 10Hz poll ────────────────────────────────────── */
    if (now - s_last_gps_ms >= 100) {
        max_m10m_tick(s_gps);
        if (s_gps->fix_type >= 2) {
            s_ground_lat_deg = (float)s_gps->lat_deg;
            s_ground_lon_deg = (float)s_gps->lon_deg;
        }
        s_last_gps_ms = now;
    }

    /* ── 6. GS status heartbeat: 1Hz ─────────────────────────── */
    if (now - s_last_status_ms >= 1000) {
        const gs_radio_stats_t *stats = ground_radio_get_stats();

#ifdef GS_OUTPUT_COBS
        /* COBS mode: emit binary GS_MSG_STATUS (0x13, 24 bytes).
         * gs_msg_status_t is __attribute__((packed)); Cortex-M7 is
         * little-endian so direct field assignment produces the correct
         * wire byte order without manual put_le* calls.
         * CRC-32 covers bytes [0..19] — all fields before crc32. */
        gs_msg_status_t pkt;
        pkt.msg_id             = MSG_ID_GS_STATUS;
        pkt.radio_profile      = stats->current_profile;
        pkt.last_rssi          = stats->last_rssi;
        pkt.last_snr           = stats->last_snr;
        pkt.rx_pkt_count       = stats->rx_pkt_count;
        pkt.rx_crc_fail        = stats->rx_crc_fail;
        pkt.ground_pressure_pa = (uint32_t)s_ground_pressure_pa;
        pkt.ground_lat_1e7     = (int32_t)(s_ground_lat_deg * 1e7f);
        pkt.ground_lon_1e7     = (int32_t)(s_ground_lon_deg * 1e7f);
        /* CRC over [0..SIZE_GS_MSG_STATUS-5] = [0..19] (20 bytes) */
        pkt.crc32 = crc32_hw_compute((const uint8_t *)&pkt,
                                     SIZE_GS_MSG_STATUS - 4U);
        ground_radio_cobs_send((const uint8_t *)&pkt, SIZE_GS_MSG_STATUS);
#else
        /* ASCII mode: serial-plotter heartbeat line (unchanged) */
        int len = snprintf(s_status_buf, sizeof(s_status_buf),
            ">GS PROF:%c PKTS:%u FAIL:%u RSSI:%d SNR:%d "
            "GBARO:%u GLAT:%.7f GLON:%.7f GFIX:%u GSAT:%u\r\n",
            stats->current_profile ? 'B' : 'A',
            stats->rx_pkt_count,
            stats->rx_crc_fail,
            (int)stats->last_rssi,
            (int)stats->last_snr,
            (unsigned int)s_ground_pressure_pa,
            (double)s_ground_lat_deg,
            (double)s_ground_lon_deg,
            s_gps->fix_type,
            s_gps->num_sv);
        CDC_Transmit_FS((uint8_t *)s_status_buf, (uint16_t)len);
#endif /* GS_OUTPUT_COBS */

        s_last_status_ms = now;
    }

    /* ── 7. Buzzer tick (non-blocking pattern updates) ────────── */
    buzzer_tick();
}
