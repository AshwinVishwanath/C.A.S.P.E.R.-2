/**
 * @file    mag_cal.c
 * @brief   Magnetometer calibration data collection for C.A.S.P.E.R.-2
 */
#include "mag_cal.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define TWO_PI    6.2831853f
#define PI        3.1415927f
#define HALF_PI   1.5707963f

/* ── Buffer flush ────────────────────────────────────────────────────────── */

static void mag_cal_flush(mag_cal_t *cal)
{
    if (cal->wbuf_pos > 0 && cal->file_open) {
        UINT bw;
        f_write(&cal->file, cal->wbuf, cal->wbuf_pos, &bw);
        cal->wbuf_pos = 0;
    }
}

/* ── LED progress ────────────────────────────────────────────────────────── */

static void mag_cal_update_leds(uint8_t pct)
{
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin,
                      pct >= 25 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin,
                      pct >= 50 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin,
                      pct >= 75 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin,
                      pct >= 85 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── CDC progress report ─────────────────────────────────────────────────── */

static void mag_cal_report(mag_cal_t *cal)
{
    char msg[80];
    int len;
    if (cal->state == MAG_CAL_DRAINING) {
        len = snprintf(msg, sizeof(msg),
            ">mag_cal: %u%% (%u/%d bins, %lu samples) draining...\r\n",
            cal->coverage_pct, cal->bins_filled,
            MAG_CAL_TOTAL_BINS, cal->sample_count);
    } else {
        len = snprintf(msg, sizeof(msg),
            ">mag_cal: %u%% (%u/%d bins, %lu samples)\r\n",
            cal->coverage_pct, cal->bins_filled,
            MAG_CAL_TOTAL_BINS, cal->sample_count);
    }
    CDC_Transmit_FS((uint8_t *)msg, len);
}

/* ── Init ────────────────────────────────────────────────────────────────── */

bool mag_cal_init(mag_cal_t *cal)
{
    memset(cal, 0, sizeof(*cal));

    FRESULT fr = f_open(&cal->file, "MAG_CAL.CSV",
                        FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
        return false;

    cal->file_open = true;

    const char *hdr = "timestamp_ms,mag_x_ut,mag_y_ut,mag_z_ut\r\n";
    UINT bw;
    f_write(&cal->file, hdr, strlen(hdr), &bw);

    cal->state = MAG_CAL_COLLECTING;
    return true;
}

/* ── Tick ────────────────────────────────────────────────────────────────── */

void mag_cal_tick(mag_cal_t *cal, const mmc5983ma_t *mag, uint32_t now_ms)
{
    if (cal->state == MAG_CAL_DONE || cal->state == MAG_CAL_IDLE)
        return;

    /* Frame mapping: sensor frame → common frame (multiply each axis by -1) */
    float mx = -mag->mag_ut[0];
    float my = -mag->mag_ut[1];
    float mz = -mag->mag_ut[2];

    /* ── Append CSV line to write buffer ─────────────────────────────────── */
    int n = snprintf(cal->wbuf + cal->wbuf_pos,
                     sizeof(cal->wbuf) - cal->wbuf_pos,
                     "%lu,%.4f,%.4f,%.4f\r\n",
                     (unsigned long)now_ms, mx, my, mz);
    if (n > 0)
        cal->wbuf_pos += (uint16_t)n;

    cal->sample_count++;

    /* Flush when buffer is getting full */
    if (cal->wbuf_pos > 1500)
        mag_cal_flush(cal);

    /* ── Sphere coverage binning ─────────────────────────────────────────── */
    float norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm >= 1.0f) {
        float ux = mx / norm;
        float uy = my / norm;
        float uz = mz / norm;

        /* Clamp uz to [-1, 1] to protect asinf from NaN */
        if (uz > 1.0f) uz = 1.0f;
        if (uz < -1.0f) uz = -1.0f;

        float az = atan2f(uy, ux);            /* [-pi, pi] */
        if (az < 0.0f) az += TWO_PI;          /* [0, 2*pi) */
        int az_bin = (int)(az * (18.0f / TWO_PI));
        if (az_bin >= MAG_CAL_AZ_BINS) az_bin = MAG_CAL_AZ_BINS - 1;

        float el = asinf(uz);                 /* [-pi/2, pi/2] */
        int el_bin = (int)((el + HALF_PI) * (9.0f / PI));
        if (el_bin >= MAG_CAL_EL_BINS) el_bin = MAG_CAL_EL_BINS - 1;
        if (el_bin < 0) el_bin = 0;

        if (!cal->bins[az_bin][el_bin]) {
            cal->bins[az_bin][el_bin] = 1;
            cal->bins_filled++;
        }
    }

    /* ── Update coverage percentage ──────────────────────────────────────── */
    cal->coverage_pct = (uint8_t)(cal->bins_filled * 100 / MAG_CAL_TOTAL_BINS);

    /* ── LED feedback ────────────────────────────────────────────────────── */
    mag_cal_update_leds(cal->coverage_pct);

    /* ── State transitions ───────────────────────────────────────────────── */
    if (cal->state == MAG_CAL_COLLECTING &&
        cal->coverage_pct >= MAG_CAL_TARGET_PCT) {
        cal->state = MAG_CAL_DRAINING;
        cal->drain_start_ms = now_ms;
    }

    if (cal->state == MAG_CAL_DRAINING &&
        (now_ms - cal->drain_start_ms >= MAG_CAL_DRAIN_MS)) {
        mag_cal_flush(cal);
        f_close(&cal->file);
        cal->file_open = false;
        cal->state = MAG_CAL_DONE;
        return;
    }

    /* ── CDC progress: every 2s or at 5% milestones ──────────────────────── */
    uint8_t pct5 = cal->coverage_pct / 5;
    if (pct5 > cal->last_pct_printed) {
        cal->last_pct_printed = pct5;
        mag_cal_report(cal);
        cal->last_progress_ms = now_ms;
    } else if (now_ms - cal->last_progress_ms >= 2000) {
        mag_cal_report(cal);
        cal->last_progress_ms = now_ms;
    }
}

/* ── Done check ──────────────────────────────────────────────────────────── */

bool mag_cal_is_done(const mag_cal_t *cal)
{
    return cal->state == MAG_CAL_DONE;
}

/* ── Reusable calibration correction ─────────────────────────────────────── */

static const float mag_hard_iron[3] = {
    -9.082933f, -23.520703f, -18.099732f
};

static const float mag_soft_iron[3][3] = {
    { 0.777972f, -0.017063f, -0.010798f},
    {-0.017063f,  0.806570f,  0.014623f},
    {-0.010798f,  0.014623f,  0.784949f}
};

void mag_cal_apply(const float raw_ut[3], float cal_ut[3])
{
    float c[3];
    for (int i = 0; i < 3; i++)
        c[i] = raw_ut[i] - mag_hard_iron[i];

    for (int i = 0; i < 3; i++)
        cal_ut[i] = mag_soft_iron[i][0] * c[0]
                   + mag_soft_iron[i][1] * c[1]
                   + mag_soft_iron[i][2] * c[2];
}
