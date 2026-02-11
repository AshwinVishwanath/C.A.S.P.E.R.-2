/**
 * @file    mag_val.c
 * @brief   Magnetometer calibration validation — 30s stationary collection
 */
#include "mag_val.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── Buffer flush ────────────────────────────────────────────────────────── */

static void mag_val_flush(mag_val_t *val)
{
    if (val->wbuf_pos > 0 && val->file_open) {
        UINT bw;
        f_write(&val->file, val->wbuf, val->wbuf_pos, &bw);
        val->wbuf_pos = 0;
    }
}

/* ── LED progress (4 LEDs = 25/50/75/100% of 30s) ───────────────────────── */

static void mag_val_update_leds(uint8_t pct)
{
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin,
                      pct >= 25 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin,
                      pct >= 50 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin,
                      pct >= 75 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin,
                      pct >= 100 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ── Init ────────────────────────────────────────────────────────────────── */

bool mag_val_init(mag_val_t *val)
{
    memset(val, 0, sizeof(*val));

    FRESULT fr = f_open(&val->file, "MAG_VAL.CSV",
                        FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
        return false;

    val->file_open = true;

    const char *hdr = "timestamp_ms,raw_x_ut,raw_y_ut,raw_z_ut,"
                      "cal_x_ut,cal_y_ut,cal_z_ut\r\n";
    UINT bw;
    f_write(&val->file, hdr, strlen(hdr), &bw);

    val->state = MAG_VAL_COLLECTING;
    return true;
}

/* ── Tick ────────────────────────────────────────────────────────────────── */

void mag_val_tick(mag_val_t *val, const mmc5983ma_t *mag, uint32_t now_ms)
{
    if (val->state != MAG_VAL_COLLECTING)
        return;

    /* Record start time on first sample */
    if (val->sample_count == 0)
        val->start_ms = now_ms;

    /* Frame mapping: sensor → common frame (×-1) */
    float raw[3] = {
        -mag->mag_ut[0],
        -mag->mag_ut[1],
        -mag->mag_ut[2]
    };

    /* Apply calibration correction */
    float cal[3];
    mag_cal_apply(raw, cal);

    /* Append CSV line */
    int n = snprintf(val->wbuf + val->wbuf_pos,
                     sizeof(val->wbuf) - val->wbuf_pos,
                     "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                     (unsigned long)now_ms,
                     raw[0], raw[1], raw[2],
                     cal[0], cal[1], cal[2]);
    if (n > 0)
        val->wbuf_pos += (uint16_t)n;

    val->sample_count++;

    if (val->wbuf_pos > 1500)
        mag_val_flush(val);

    /* Progress as percentage of 30s */
    uint32_t elapsed = now_ms - val->start_ms;
    uint8_t pct = (uint8_t)(elapsed * 100 / MAG_VAL_DURATION_MS);
    if (pct > 100) pct = 100;

    mag_val_update_leds(pct);

    /* CDC progress every 2s or at 10% milestones */
    uint8_t pct10 = pct / 10;
    if (pct10 > val->last_pct_printed) {
        val->last_pct_printed = pct10;
        char msg[64];
        int len = snprintf(msg, sizeof(msg),
            ">mag_val: %u%% (%lu samples)\r\n", pct, val->sample_count);
        CDC_Transmit_FS((uint8_t *)msg, len);
        val->last_progress_ms = now_ms;
    } else if (now_ms - val->last_progress_ms >= 2000) {
        char msg[64];
        int len = snprintf(msg, sizeof(msg),
            ">mag_val: %u%% (%lu samples)\r\n", pct, val->sample_count);
        CDC_Transmit_FS((uint8_t *)msg, len);
        val->last_progress_ms = now_ms;
    }

    /* Done after 30s */
    if (elapsed >= MAG_VAL_DURATION_MS) {
        mag_val_flush(val);
        f_close(&val->file);
        val->file_open = false;
        val->state = MAG_VAL_DONE;
    }
}

/* ── Done check ──────────────────────────────────────────────────────────── */

bool mag_val_is_done(const mag_val_t *val)
{
    return val->state == MAG_VAL_DONE;
}
