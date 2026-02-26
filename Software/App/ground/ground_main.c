/**
 * @file ground_main.c
 * @brief Ground station main application: radio RX, sensor polling,
 *        USB CDC ASCII output, 1Hz GS status heartbeat.
 */

#include "ground_main.h"
#include "ground_radio.h"
#include "radio_irq.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "ms5611.h"
#include "max_m10m.h"
#include "buzzer.h"
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

/* CDC output buffer */
static char s_status_buf[200];

/* ── Init ─────────────────────────────────────────────────────────── */

void ground_main_init(SPI_HandleTypeDef *hspi1, ms5611_t *baro, max_m10m_t *gps)
{
    s_baro = baro;
    s_gps  = gps;
    s_ground_pressure_pa = 0.0f;
    s_ground_lat_deg     = 0.0f;
    s_ground_lon_deg     = 0.0f;

    uint32_t now = HAL_GetTick();
    s_last_baro_ms   = now;
    s_last_gps_ms    = now;
    s_last_status_ms = now;

    /* Init radio in RX-continuous mode */
    int rc = ground_radio_init(hspi1);

    /* Report init result over CDC */
    int len = snprintf(s_status_buf, sizeof(s_status_buf),
        ">GS RADIO_INIT:%s\r\n", (rc == 0) ? "OK" : "FAIL");
    CDC_Transmit_FS((uint8_t *)s_status_buf, (uint16_t)len);

    /* Short beep to indicate GS is ready */
    buzzer_beep_n(30, 1, 100, 200);
}

/* ── Tick ──────────────────────────────────────────────────────────── */

void ground_main_tick(void)
{
    uint32_t now = HAL_GetTick();

    /* ── 0. Poll DIO0/DIO1 GPIOs (EXTI disabled — polling mode) ── */
    if (HAL_GPIO_ReadPin(SPI1_INT_GPIO_Port, SPI1_INT_Pin))
        g_radio_dio0_flag = 1;
    if (HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin))
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

    /* ── 6. GS status heartbeat: 1Hz ASCII output ────────────── */
    if (now - s_last_status_ms >= 1000) {
        const gs_radio_stats_t *stats = ground_radio_get_stats();

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

        s_last_status_ms = now;
    }

    /* ── 7. Buzzer tick (non-blocking pattern updates) ────────── */
    buzzer_tick();
}
