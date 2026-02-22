/**
 * @file    temp_cal.c
 * @brief   Multi-sensor temperature calibration data collection for C.A.S.P.E.R.-2
 *
 * Collects raw sensor data across a temperature sweep for offline regression
 * of gyro bias vs. die temperature coefficients.
 */
#include "temp_cal.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ── Buffer flush ───────────────────────────────────────────────────────── */

static void tcal_flush(temp_cal_t *tc)
{
    if (tc->wbuf_pos > 0 && tc->file_open) {
        UINT bw;
        f_write(&tc->file, tc->wbuf, tc->wbuf_pos, &bw);
        tc->wbuf_pos = 0;
    }
}

static void tcal_sync(temp_cal_t *tc)
{
    tcal_flush(tc);
    if (tc->file_open)
        f_sync(&tc->file);
}

/* ── Auto-increment filename ────────────────────────────────────────────── */

static bool tcal_find_filename(temp_cal_t *tc)
{
    FILINFO fno;
    for (int i = 1; i <= 999; i++) {
        snprintf(tc->filename, sizeof(tc->filename), "TCAL_%03d.CSV", i);
        if (f_stat(tc->filename, &fno) == FR_NO_FILE)
            return true;
    }
    return false;
}

/* ── CDC STOP command check ─────────────────────────────────────────────── */

static bool tcal_check_stop(temp_cal_t *tc)
{
    while (cdc_ring_available() > 0) {
        uint8_t c = cdc_ring_read_byte();

        if (tc->cmd_pos < 4) {
            tc->cmd_buf[tc->cmd_pos++] = (char)c;
        }

        if (tc->cmd_pos >= 4) {
            if (memcmp(tc->cmd_buf, "STOP", 4) == 0)
                return true;
            /* Slide window: keep last 3, wait for next byte */
            memmove(tc->cmd_buf, tc->cmd_buf + 1, 3);
            tc->cmd_pos = 3;
        }
    }
    return false;
}

/* ── CDC helper (send string) ───────────────────────────────────────────── */

static void tcal_cdc_print(const char *msg, int len)
{
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);
    HAL_Delay(50);
}

/* ── Init ───────────────────────────────────────────────────────────────── */

bool temp_cal_init(temp_cal_t *tc)
{
    memset(tc, 0, sizeof(*tc));

    /* Find next available TCAL_NNN.CSV */
    if (!tcal_find_filename(tc))
        return false;

    /* Open file */
    FRESULT fr = f_open(&tc->file, tc->filename,
                        FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
        return false;

    tc->file_open = true;

    /* Write comment line + CSV header */
    static const char comment[] =
        "# TEMP_CAL — board must be stationary. Do not move during collection.\r\n";
    static const char header[] =
        "timestamp_ms,lsm_temp_c,"
        "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
        "accel_x_g,accel_y_g,accel_z_g,"
        "ms5611_temp_c,pressure_pa,"
        "mag_x,mag_y,mag_z\r\n";
    UINT bw;
    f_write(&tc->file, comment, sizeof(comment) - 1, &bw);
    f_write(&tc->file, header, sizeof(header) - 1, &bw);
    f_sync(&tc->file);

    tc->state = TCAL_COLLECTING;
    tc->start_ms = HAL_GetTick();
    tc->last_heartbeat_ms = tc->start_ms;

    /* CDC startup banner */
    {
        char buf[80];
        int len;

        len = snprintf(buf, sizeof(buf),
            "# TEMP_CAL mode active\r\n");
        for (int i = 0; i < 5; i++) tcal_cdc_print(buf, len);

        len = snprintf(buf, sizeof(buf),
            "# Logging to %s\r\n", tc->filename);
        for (int i = 0; i < 5; i++) tcal_cdc_print(buf, len);

        len = snprintf(buf, sizeof(buf),
            "# Board must be stationary. Send STOP to end collection.\r\n");
        for (int i = 0; i < 5; i++) tcal_cdc_print(buf, len);
    }

    return true;
}

/* ── Tick ───────────────────────────────────────────────────────────────── */

void temp_cal_tick(temp_cal_t *tc, lsm6dso32_t *imu,
                   ms5611_t *baro, mmc5983ma_t *mag, uint32_t now_ms)
{
    if (tc->state != TCAL_COLLECTING)
        return;

    /* ── Check for STOP command ─────────────────────────────────────────── */
    if (tcal_check_stop(tc)) {
        tcal_flush(tc);
        f_sync(&tc->file);
        f_close(&tc->file);
        tc->file_open = false;
        tc->state = TCAL_STOPPED;

        char msg[80];
        int len = snprintf(msg, sizeof(msg),
            "# Collection stopped. File closed. %lu samples written to %s\r\n",
            (unsigned long)tc->sample_count, tc->filename);
        for (int i = 0; i < 5; i++) tcal_cdc_print(msg, len);
        return;
    }

    /* ── Sample all sensors synchronously ───────────────────────────────── */
    lsm6dso32_read(imu);               /* gyro + accel + die temp         */
    ms5611_read(baro);                  /* blocking D1+D2 at OSR_4096      */
    mmc5983ma_trigger_oneshot(mag);     /* single-shot trigger + read      */

    /* ── Build CSV line ─────────────────────────────────────────────────── */
    float ms5611_temp = ms5611_get_temperature(baro);

    int n = snprintf(tc->wbuf + tc->wbuf_pos,
                     TCAL_WBUF_SIZE - tc->wbuf_pos,
                     "%lu,"                /* timestamp_ms            */
                     "%.3f,"               /* lsm_temp_c              */
                     "%.4f,%.4f,%.4f,"     /* gyro x,y,z dps          */
                     "%.4f,%.4f,%.4f,"     /* accel x,y,z g           */
                     "%.3f,"               /* ms5611_temp_c            */
                     "%ld,"                /* pressure_pa              */
                     "%lu,%lu,%lu"         /* mag x,y,z raw 18-bit    */
                     "\r\n",
                     (unsigned long)now_ms,
                     (double)imu->temp_c,
                     (double)imu->gyro_dps[0],
                     (double)imu->gyro_dps[1],
                     (double)imu->gyro_dps[2],
                     (double)imu->accel_g[0],
                     (double)imu->accel_g[1],
                     (double)imu->accel_g[2],
                     (double)ms5611_temp,
                     (long)baro->pressure,
                     (unsigned long)mag->raw_mag[0],
                     (unsigned long)mag->raw_mag[1],
                     (unsigned long)mag->raw_mag[2]);
    if (n > 0 && (tc->wbuf_pos + n) < TCAL_WBUF_SIZE)
        tc->wbuf_pos += (uint16_t)n;

    tc->sample_count++;
    tc->samples_since_flush++;

    /* ── Flush when buffer is filling up ────────────────────────────────── */
    if (tc->wbuf_pos > 1500)
        tcal_flush(tc);

    /* ── f_sync every TCAL_FLUSH_INTERVAL samples (10s at 10Hz) ─────── */
    if (tc->samples_since_flush >= TCAL_FLUSH_INTERVAL) {
        tcal_sync(tc);
        tc->samples_since_flush = 0;
    }

    /* ── LED1 heartbeat (toggle at ~2 Hz) ───────────────────────────────── */
    {
        static uint32_t led_last = 0;
        if (now_ms - led_last >= 500) {
            HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
            led_last = now_ms;
        }
    }

    /* ── CDC heartbeat every 10 seconds ─────────────────────────────────── */
    if (now_ms - tc->last_heartbeat_ms >= TCAL_HEARTBEAT_MS) {
        uint32_t elapsed_s = (now_ms - tc->start_ms) / 1000;
        char hb[80];
        int hlen = snprintf(hb, sizeof(hb),
            "# T=%lus, N=%lu samples, lsm_temp=%.1f%cC\r\n",
            (unsigned long)elapsed_s,
            (unsigned long)tc->sample_count,
            (double)imu->temp_c,
            0xB0);  /* degree symbol */
        tcal_cdc_print(hb, hlen);
        tc->last_heartbeat_ms = now_ms;
    }
}

/* ── Stopped check ──────────────────────────────────────────────────────── */

bool temp_cal_is_stopped(const temp_cal_t *tc)
{
    return tc->state == TCAL_STOPPED;
}
