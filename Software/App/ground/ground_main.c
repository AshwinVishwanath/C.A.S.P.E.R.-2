/* ============================================================
 *  TIER:     GROUND-STATION
 *  MODULE:   Ground Main
 *  SUMMARY:  Ground board entry point: radio RX, local sensors, USB CDC.
 * ============================================================ */
/**
 * @file ground_main.c
 * @brief Ground station main application: radio RX, sensor polling,
 *        USB CDC ASCII output, 1Hz GS status heartbeat.
 */

#include "ground_main.h"
#include "ground_radio.h"
#include "radio_config.h"   /* RADIO_TX_PERIOD_MS (FEI beacon cadence) */
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

#ifdef GS_FEI_BEACON
/* FEI beacon (Casper-3 crystal-vs-TCXO bench): 10 Hz TX state. */
static uint32_t s_last_beacon_ms;
static uint16_t s_beacon_seq;
static uint32_t s_beacon_count;
#endif

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

    /* ── 1. Radio RX: DIO0 is mapped to RxDone. Parse inbound packets when
     *        it fires and we are not mid-transmit. ── */
    if (g_radio_dio0_flag && !ground_radio_tx_pending()) {
        ground_radio_on_rx();
        /* Note: g_radio_dio0_flag is cleared inside ground_radio_on_rx() */
    }

    /* ── 2. Radio TX completion. TxDone is NOT on DIO0 (hard-mapped to
     *        RxDone), so check_tx_done() polls the IRQ-flags register every
     *        loop and re-arms RX-continuous once the reply has gone out. ── */
    ground_radio_check_tx_done();

#ifdef GS_FEI_BEACON
    /* ── 2b. FEI beacon: 10 Hz TX toward the Casper-3 FEI bench (which is
     *        RX-only — see flight/app/radio_app.c in the C3 repo). Framing
     *        reuses the ping-pong header with its own type byte (0x03) so
     *        this GS's own responder ignores any echo; the FC bench never
     *        parses payloads anyway. check_tx_done() above re-arms
     *        RX-continuous after each burst. ── */
    if (!ground_radio_tx_pending() &&
        (now - s_last_beacon_ms >= RADIO_TX_PERIOD_MS)) {
        uint8_t b[6] = { 0xCAu, 0x53u, 0x03u,
                         (uint8_t)(s_beacon_seq & 0xFFu),
                         (uint8_t)(s_beacon_seq >> 8), 0u };
        b[5] = (uint8_t)(b[0] + b[1] + b[2] + b[3] + b[4]);
        if (ground_radio_send_cmd(b, 6u) == 0) {
            s_beacon_seq++;
            s_beacon_count++;
        }
        s_last_beacon_ms = now;
    }
#endif

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

#ifdef GS_FEI_BEACON
        int len = snprintf(s_status_buf, sizeof(s_status_buf),
            ">GS FEI_BEACON n=%lu seq=%u cfg=868.0MHz/SF9/BW125/CR5/+10dBm "
            "RX_PKTS:%u RX_FAIL:%u\r\n",
            (unsigned long)s_beacon_count, (unsigned)s_beacon_seq,
            stats->rx_pkt_count, stats->rx_crc_fail);
#else
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
#endif /* GS_FEI_BEACON */
        CDC_Transmit_FS((uint8_t *)s_status_buf, (uint16_t)len);

        s_last_status_ms = now;
    }

    /* ── 7. Buzzer tick (non-blocking pattern updates) ────────── */
    buzzer_tick();
}
