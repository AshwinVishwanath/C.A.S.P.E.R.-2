/**
 * @file    mag_noise.c
 * @brief   Magnetometer noise characterization logger for C.A.S.P.E.R.-2
 */
#include "mag_noise.h"
#include "mag_cal.h"
#include "casper_quat.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define DEG2RAD       0.017453292f
#define G_TO_MPS2     9.80665f

/* Run-profile name for the CSV header (compile-time selected). */
#if defined(MAG_NOISE_RUN_A)
  #define MAG_NOISE_RUN_NAME "A"
#elif defined(MAG_NOISE_RUN_B)
  #define MAG_NOISE_RUN_NAME "B"
#elif defined(MAG_NOISE_RUN_C)
  #define MAG_NOISE_RUN_NAME "C"
#elif defined(MAG_NOISE_RUN_D)
  #define MAG_NOISE_RUN_NAME "D"
#endif

/* ── Buffer flush ────────────────────────────────────────────────────────── */

static void mag_noise_flush(mag_noise_t *cap)
{
    if (cap->wbuf_pos > 0 && cap->file_open) {
        UINT bw;
        f_write(&cap->file, cap->wbuf, cap->wbuf_pos, &bw);
        cap->wbuf_pos = 0;
    }
}

/* ── LED progress: quartile bar across the 4 continuity LEDs ─────────────── */

static void mag_noise_update_leds(uint8_t pct)
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

/* ── CDC progress report ─────────────────────────────────────────────────── */

static void mag_noise_report(mag_noise_t *cap, uint8_t pct)
{
    char msg[96];
    int len = snprintf(msg, sizeof(msg),
        ">mag_noise[%s]: %u%% (%lu samples, %lu ms)\r\n",
        MAG_NOISE_RUN_NAME, pct,
        cap->sample_count,
        (unsigned long)(HAL_GetTick() - cap->start_ms));
    CDC_Transmit_FS((uint8_t *)msg, len);
}

/* ── Init ────────────────────────────────────────────────────────────────── */

bool mag_noise_init(mag_noise_t *cap)
{
    memset(cap, 0, sizeof(*cap));

    FRESULT fr = f_open(&cap->file, "MAG_NOISE.CSV",
                        FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
        return false;

    cap->file_open = true;

    /* Header: comment line documents the run profile, then CSV column names. */
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
        "# C.A.S.P.E.R.-2 mag noise capture, run=%s, duration_ms=%lu\r\n"
        "t_ms,sample_idx,"
        "mx_raw,my_raw,mz_raw,"
        "mx_ut,my_ut,mz_ut,"
        "ax_mps2,ay_mps2,az_mps2,"
        "gx_rps,gy_rps,gz_rps,"
        "qw,qx,qy,qz,"
        "radio_tx,qspi_busy,tx_count\r\n",
        MAG_NOISE_RUN_NAME,
        (unsigned long)MAG_NOISE_DURATION_MS);
    UINT bw;
    f_write(&cap->file, hdr, (UINT)n, &bw);

    cap->state    = MAG_NOISE_COLLECTING;
    cap->start_ms = HAL_GetTick();
    return true;
}

/* ── Tick ────────────────────────────────────────────────────────────────── */

void mag_noise_tick(mag_noise_t *cap,
                    const mmc5983ma_t *mag,
                    const lsm6dso32_t *imu,
                    const casper_attitude_t *att,
                    bool radio_tx_active,
                    bool qspi_busy,
                    uint16_t radio_tx_count,
                    uint32_t now_ms)
{
    if (cap->state != MAG_NOISE_COLLECTING)
        return;

    /* ── Done check: stop on duration elapsed ───────────────────────────── */
    if ((uint32_t)(now_ms - cap->start_ms) >= MAG_NOISE_DURATION_MS) {
        mag_noise_flush(cap);
        f_close(&cap->file);
        cap->file_open = false;
        cap->state = MAG_NOISE_DONE;
        return;
    }

    /* ── Frame mapping: sensor frame → common frame (negate, matches mag_cal) */
    float mx_ut = -mag->mag_ut[0];
    float my_ut = -mag->mag_ut[1];
    float mz_ut = -mag->mag_ut[2];

    /* ── IMU unit conversion: g → m/s², dps → rad/s ─────────────────────── */
    float ax = imu->accel_g[0]  * G_TO_MPS2;
    float ay = imu->accel_g[1]  * G_TO_MPS2;
    float az = imu->accel_g[2]  * G_TO_MPS2;
    float gx = imu->gyro_dps[0] * DEG2RAD;
    float gy = imu->gyro_dps[1] * DEG2RAD;
    float gz = imu->gyro_dps[2] * DEG2RAD;

    /* ── Append CSV row ─────────────────────────────────────────────────── */
    int n = snprintf(cap->wbuf + cap->wbuf_pos,
                     sizeof(cap->wbuf) - cap->wbuf_pos,
                     "%lu,%lu,"
                     "%lu,%lu,%lu,"
                     "%.4f,%.4f,%.4f,"
                     "%.4f,%.4f,%.4f,"
                     "%.6f,%.6f,%.6f,"
                     "%.6f,%.6f,%.6f,%.6f,"
                     "%u,%u,%u\r\n",
                     (unsigned long)(now_ms - cap->start_ms),
                     (unsigned long)cap->sample_count,
                     (unsigned long)mag->raw_mag[0],
                     (unsigned long)mag->raw_mag[1],
                     (unsigned long)mag->raw_mag[2],
                     mx_ut, my_ut, mz_ut,
                     ax, ay, az,
                     gx, gy, gz,
                     att->q[0], att->q[1], att->q[2], att->q[3],
                     (unsigned)(radio_tx_active ? 1 : 0),
                     (unsigned)(qspi_busy ? 1 : 0),
                     (unsigned)radio_tx_count);
    if (n > 0)
        cap->wbuf_pos += (uint16_t)n;

    cap->sample_count++;

    /* Flush when buffer is getting full */
    if (cap->wbuf_pos > 1800)
        mag_noise_flush(cap);

    /* ── Progress: LEDs + CDC ───────────────────────────────────────────── */
    uint32_t elapsed = now_ms - cap->start_ms;
    uint8_t pct = (uint8_t)((uint64_t)elapsed * 100 / MAG_NOISE_DURATION_MS);
    if (pct > 100) pct = 100;
    mag_noise_update_leds(pct);

    uint8_t pct10 = pct / 10;
    if (pct10 > cap->last_pct_printed) {
        cap->last_pct_printed = pct10;
        mag_noise_report(cap, pct);
        cap->last_progress_ms = now_ms;
    } else if (now_ms - cap->last_progress_ms >= 5000) {
        mag_noise_report(cap, pct);
        cap->last_progress_ms = now_ms;
    }
}

/* ── Forced flush (Run D burst behaviour) ────────────────────────────────── */

void mag_noise_force_flush(mag_noise_t *cap, uint32_t now_ms)
{
    if (cap->state != MAG_NOISE_COLLECTING)
        return;
    if (now_ms - cap->last_burst_flush_ms < MAG_NOISE_BURST_PERIOD_MS)
        return;
    mag_noise_flush(cap);
    cap->last_burst_flush_ms = now_ms;
}

/* ── Done check ──────────────────────────────────────────────────────────── */

bool mag_noise_is_done(const mag_noise_t *cap)
{
    return cap->state == MAG_NOISE_DONE;
}
