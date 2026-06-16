/**
 * @file    mag_noise.c
 * @brief   Magnetometer noise + EMI characterization sequencer
 */
#include "mag_noise.h"
#include "mag_cal.h"
#include "casper_quat.h"
#include "radio_manager.h"
#include "buzzer.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define DEG2RAD       0.017453292f
#define G_TO_MPS2     9.80665f

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static const char *run_label(mag_noise_run_t r)
{
    switch (r) {
    case MAG_NOISE_RUN_A: return "A";
    case MAG_NOISE_RUN_B: return "B";
    case MAG_NOISE_RUN_C: return "C";
    case MAG_NOISE_RUN_D: return "D";
    default: return "?";
    }
}

static bool run_uses_radio(mag_noise_run_t r)
{
    return r == MAG_NOISE_RUN_B || r == MAG_NOISE_RUN_C;
}

static void mag_buf_flush(mag_noise_t *cap)
{
    if (cap->wbuf_pos > 0 && cap->file_open) {
        UINT bw;
        f_write(&cap->file, cap->wbuf, cap->wbuf_pos, &bw);
        cap->wbuf_pos = 0;
    }
}

static void evt_buf_flush(mag_noise_t *cap)
{
    if (cap->evt_wbuf_pos > 0 && cap->evt_file_open) {
        UINT bw;
        f_write(&cap->evt_file, cap->evt_wbuf, cap->evt_wbuf_pos, &bw);
        cap->evt_wbuf_pos = 0;
    }
}

static void leds_run_pattern(mag_noise_run_t r)
{
    /* Per-run LED pattern: progress bar shifts left so the user can tell
     * at a glance which run is active without watching the clock.
     *   A: LED1
     *   B: LED1+LED2
     *   C: LED1+LED2+LED3
     *   D: LED1+LED2+LED3+LED4 (all on)
     * (Quartile progress within the run is overlaid in mag_noise_tick.) */
    HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin,
                      (r >= MAG_NOISE_RUN_B) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin,
                      (r >= MAG_NOISE_RUN_C) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin,
                      (r >= MAG_NOISE_RUN_D) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void leds_done_solid(void)
{
    HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
}

static void cdc_msg(const char *fmt, ...)
{
    char buf[100];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
}

/* ── Open / close run files ─────────────────────────────────────────────── */

static bool open_run_files(mag_noise_t *cap, mag_noise_run_t r)
{
    char name[16];
    snprintf(name, sizeof(name), "MAG_%s.CSV", run_label(r));

    FRESULT fr = f_open(&cap->file, name, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) return false;
    cap->file_open = true;
    cap->wbuf_pos  = 0;
    cap->sample_count = 0;

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
        run_label(r), (unsigned long)MAG_NOISE_RUN_DURATION_MS);
    UINT bw;
    f_write(&cap->file, hdr, (UINT)n, &bw);

    /* Open EVT file too if the run uses radio */
    cap->evt_file_open = false;
    cap->evt_wbuf_pos  = 0;
    cap->evt_count     = 0;
    if (run_uses_radio(r)) {
        snprintf(name, sizeof(name), "EVT_%s.CSV", run_label(r));
        fr = f_open(&cap->evt_file, name, FA_CREATE_ALWAYS | FA_WRITE);
        if (fr == FR_OK) {
            cap->evt_file_open = true;
            const char *ehdr =
                "# C.A.S.P.E.R.-2 radio TX events (matched to MAG_<run>.CSV)\r\n"
                "evt_idx,tx_index,start_ms,end_ms,duration_ms,ok\r\n";
            f_write(&cap->evt_file, ehdr, (UINT)strlen(ehdr), &bw);
        }
    }

    /* Drain any stale TX events from before this run started */
    uint32_t st, en; uint16_t tix; bool ok;
    while (radio_drain_tx_event(&st, &en, &tix, &ok)) {
        (void)st; (void)en; (void)tix; (void)ok;
    }
    return true;
}

static void close_run_files(mag_noise_t *cap)
{
    mag_buf_flush(cap);
    if (cap->file_open) {
        f_close(&cap->file);
        cap->file_open = false;
    }
    evt_buf_flush(cap);
    if (cap->evt_file_open) {
        f_close(&cap->evt_file);
        cap->evt_file_open = false;
    }
}

/* ── Sequencer transitions ──────────────────────────────────────────────── */

static uint8_t beeps_for_finished_run(mag_noise_run_t r)
{
    return (uint8_t)(r + 1);  /* A→1, B→2, C→3, D→4 */
}

static void enter_run(mag_noise_t *cap, mag_noise_run_t r, uint32_t now_ms)
{
    cap->phase             = MN_PHASE_RUN;
    cap->current_run       = r;
    cap->run_start_ms      = now_ms;
    cap->last_progress_ms  = now_ms;
    cap->last_pct_printed  = 0;
    cap->last_burst_flush_ms = now_ms;
    leds_run_pattern(r);
    cdc_msg(">mag_noise: run %s START\r\n", run_label(r));
    if (!open_run_files(cap, r)) {
        /* If we can't open the file, halt with all LEDs blinking */
        cap->phase = MN_PHASE_DONE;
        cdc_msg(">mag_noise: ERROR opening files for run %s\r\n", run_label(r));
        return;
    }
}

static void finish_current_run(mag_noise_t *cap, uint32_t now_ms)
{
    /* current_run stays as the just-finished run for the sequencer to use
     * when picking next-run / beep-count. The public getter masks it to
     * MAG_NOISE_RUN_NONE while we're not actively in MN_PHASE_RUN. */
    mag_noise_run_t r = cap->current_run;
    close_run_files(cap);
    cdc_msg(">mag_noise: run %s END (%lu samples)\r\n",
            run_label(r), cap->sample_count);

    cap->phase           = (r == MAG_NOISE_RUN_D) ? MN_PHASE_BEEP_FINAL
                                                  : MN_PHASE_BEEP_GAP;
    cap->phase_start_ms  = now_ms;
    cap->beeps_started   = false;
}

/* ── Public: init ───────────────────────────────────────────────────────── */

bool mag_noise_init(mag_noise_t *cap)
{
    memset(cap, 0, sizeof(*cap));
    cap->current_run = MAG_NOISE_RUN_NONE;
    enter_run(cap, MAG_NOISE_RUN_A, HAL_GetTick());
    return cap->phase == MN_PHASE_RUN;
}

/* ── Public: sample tick ────────────────────────────────────────────────── */

void mag_noise_tick(mag_noise_t *cap,
                    const mmc5983ma_t *mag,
                    const lsm6dso32_t *imu,
                    const casper_attitude_t *att,
                    bool radio_tx_active,
                    bool qspi_busy,
                    uint16_t radio_tx_count,
                    uint32_t now_ms)
{
    if (cap->phase != MN_PHASE_RUN || !cap->file_open) return;

    /* Frame mapping matches mag_cal */
    float mx_ut = -mag->mag_ut[0];
    float my_ut = -mag->mag_ut[1];
    float mz_ut = -mag->mag_ut[2];

    float ax = imu->accel_g[0]  * G_TO_MPS2;
    float ay = imu->accel_g[1]  * G_TO_MPS2;
    float az = imu->accel_g[2]  * G_TO_MPS2;
    float gx = imu->gyro_dps[0] * DEG2RAD;
    float gy = imu->gyro_dps[1] * DEG2RAD;
    float gz = imu->gyro_dps[2] * DEG2RAD;

    int n = snprintf(cap->wbuf + cap->wbuf_pos,
                     sizeof(cap->wbuf) - cap->wbuf_pos,
                     "%lu,%lu,"
                     "%lu,%lu,%lu,"
                     "%.4f,%.4f,%.4f,"
                     "%.4f,%.4f,%.4f,"
                     "%.6f,%.6f,%.6f,"
                     "%.6f,%.6f,%.6f,%.6f,"
                     "%u,%u,%u\r\n",
                     (unsigned long)(now_ms - cap->run_start_ms),
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
    if (n > 0) cap->wbuf_pos += (uint16_t)n;
    cap->sample_count++;

    if (cap->wbuf_pos > 1800)
        mag_buf_flush(cap);

    /* CDC progress every 10% of the run */
    uint32_t elapsed = now_ms - cap->run_start_ms;
    uint8_t pct = (uint8_t)((uint64_t)elapsed * 100 / MAG_NOISE_RUN_DURATION_MS);
    if (pct > 100) pct = 100;
    uint8_t pct10 = pct / 10;
    if (pct10 > cap->last_pct_printed) {
        cap->last_pct_printed = pct10;
        cdc_msg(">mag_noise[%s]: %u%% (%lu samples)\r\n",
                run_label(cap->current_run), pct, cap->sample_count);
        cap->last_progress_ms = now_ms;
    }
}

/* ── Public: sequencer tick ─────────────────────────────────────────────── */

void mag_noise_sequencer_tick(mag_noise_t *cap, uint32_t now_ms)
{
    switch (cap->phase) {
    case MN_PHASE_RUN:
        if ((uint32_t)(now_ms - cap->run_start_ms) >= MAG_NOISE_RUN_DURATION_MS) {
            finish_current_run(cap, now_ms);
        }
        break;

    case MN_PHASE_BEEP_GAP:
        if (!cap->beeps_started) {
            buzzer_beep_n(MAG_NOISE_BEEP_DUTY_PCT,
                          beeps_for_finished_run(cap->current_run),
                          MAG_NOISE_BEEP_ON_MS, MAG_NOISE_BEEP_PERIOD_MS);
            cap->beeps_started  = true;
            cap->phase_start_ms = now_ms;
        }
        if (!buzzer_is_busy() &&
            (uint32_t)(now_ms - cap->phase_start_ms) >= MAG_NOISE_GAP_MS) {
            enter_run(cap, (mag_noise_run_t)(cap->current_run + 1), now_ms);
        }
        break;

    case MN_PHASE_BEEP_FINAL:
        if (!cap->beeps_started) {
            buzzer_beep_n(MAG_NOISE_BEEP_DUTY_PCT, 4,
                          MAG_NOISE_BEEP_ON_MS, MAG_NOISE_BEEP_PERIOD_MS);
            cap->beeps_started  = true;
            cap->phase_start_ms = now_ms;
        }
        if (!buzzer_is_busy() &&
            (uint32_t)(now_ms - cap->phase_start_ms) >= 500UL) {
            cap->phase          = MN_PHASE_BEEP_LONG;
            cap->beeps_started  = false;
        }
        break;

    case MN_PHASE_BEEP_LONG:
        if (!cap->beeps_started) {
            buzzer_beep_n(MAG_NOISE_BEEP_DUTY_PCT, 1,
                          MAG_NOISE_FINAL_LONG_MS, MAG_NOISE_FINAL_LONG_MS);
            cap->beeps_started  = true;
            cap->phase_start_ms = now_ms;
        }
        if (!buzzer_is_busy()) {
            leds_done_solid();
            cap->phase = MN_PHASE_DONE;
            cdc_msg(">mag_noise: ALL RUNS COMPLETE\r\n");
        }
        break;

    case MN_PHASE_DONE:
    default:
        break;
    }
}

/* ── Public: misc ───────────────────────────────────────────────────────── */

void mag_noise_force_flush(mag_noise_t *cap, uint32_t now_ms)
{
    if (cap->phase != MN_PHASE_RUN || cap->current_run != MAG_NOISE_RUN_D)
        return;
    if (now_ms - cap->last_burst_flush_ms < MAG_NOISE_BURST_PERIOD_MS)
        return;
    mag_buf_flush(cap);
    cap->last_burst_flush_ms = now_ms;
}

void mag_noise_drain_tx_events(mag_noise_t *cap)
{
    if (cap->phase != MN_PHASE_RUN || !cap->evt_file_open)
        return;

    uint32_t st_abs, en_abs;
    uint16_t tix;
    bool     ok;
    while (radio_drain_tx_event(&st_abs, &en_abs, &tix, &ok)) {
        uint32_t st = st_abs - cap->run_start_ms;
        uint32_t en = en_abs - cap->run_start_ms;
        int n = snprintf(cap->evt_wbuf + cap->evt_wbuf_pos,
                         sizeof(cap->evt_wbuf) - cap->evt_wbuf_pos,
                         "%lu,%u,%lu,%lu,%lu,%u\r\n",
                         (unsigned long)cap->evt_count,
                         (unsigned)tix,
                         (unsigned long)st,
                         (unsigned long)en,
                         (unsigned long)(en_abs - st_abs),
                         (unsigned)(ok ? 1 : 0));
        if (n > 0) cap->evt_wbuf_pos += (uint16_t)n;
        cap->evt_count++;

        if (cap->evt_wbuf_pos > (uint16_t)(sizeof(cap->evt_wbuf) - 80))
            evt_buf_flush(cap);
    }
}

mag_noise_run_t mag_noise_current_run(const mag_noise_t *cap)
{
    return (cap->phase == MN_PHASE_RUN) ? cap->current_run : MAG_NOISE_RUN_NONE;
}

bool mag_noise_is_done(const mag_noise_t *cap)
{
    return cap->phase == MN_PHASE_DONE;
}
