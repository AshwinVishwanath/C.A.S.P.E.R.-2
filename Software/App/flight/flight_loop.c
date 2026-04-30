/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Flight Loop
 *  SUMMARY:  Top-level superloop: schedules sensor reads, EKF, FSM, telemetry.
 * ============================================================ */
#include "flight_loop.h"
#include "app_globals.h"

#include "stm32h7xx_hal.h"
#include "main.h"
#include "casper_quat.h"
#include "casper_gyro_int.h"
#include "mag_cal.h"
#include "pyro_manager.h"
#include "tlm_manager.h"
#include "tlm_types.h"
#include "flight_fsm.h"
#include "cmd_router.h"
#include "cac_handler.h"
#include "radio_manager.h"
#include "buzzer.h"
#include "usbd_cdc_if.h"
#include "fsm_util.h"
/* cycle_probe.h provides no-op DIAG_PROBE_BEGIN/END when LOGGER_SANITY is off */
#include "cycle_probe.h"
#ifdef LOGGER_SANITY
#include "logger_sanity.h"

/* Probes — names match what gets printed in [CYC] lines. */
diag_probe_t probe_superloop;
diag_probe_t probe_imu_svc;
diag_probe_t probe_imu_read;
diag_probe_t probe_att_upd;
diag_probe_t probe_ekf_pred;
diag_probe_t probe_hr_push;
diag_probe_t probe_radio;
diag_probe_t probe_mag_rd;
diag_probe_t probe_pyro_rd;
diag_probe_t probe_log_tick;
#endif
#ifdef HIL_MODE
#include "hil_raw_handler.h"
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef TEST_MODE
#define TEST_MODE 1
#endif

/* ── Gyro temperature compensation (slope-only linear model) ── */
#ifdef GYRO_TEMP_COMP
#include "temp_cal_coeffs.h"
#endif

#if TEST_MODE != 2
/* ── Module-level state (was local to main before while-loop) ── */
static float   mag_cal_ut[3];
static bool    mag_new        = false;
static uint32_t last_mag_tick = 0;

static const float EKF_DT   = 0.0012f;
static const float G_CONST  = 9.80665f;
#ifndef HIL_MODE
static const float DEG2RAD  = 0.0174532925f;
#endif

static float   baro_ref       = 0.0f;
static bool    init_done      = false;
static bool    cal_done       = false;
static uint32_t loop_start_tick = 0;
static uint32_t last_imu_tick  = 0;

/* Baro ground-level calibration accumulator */
static double   baro_cal_sum   = 0.0;
static uint32_t baro_cal_count = 0;

/* NED accumulation for 416 Hz EKF predict (Item 11) */
static float    ned_accel_accum[3];
static uint8_t  imu_subsample_count = 0;
static float    dt_predict_accum = 0.0f;  /* sum of dt over the 2 subsamples between predicts */

/* Last computed accel magnitude for FSM launch detection */
static float    last_accel_mag_g = 0.0f;

/* Last body-frame accel (m/s^2) — used by FSM section in both HIL and normal */
static float    last_body_accel_ms2[3] = {0};

/* Last baro altitude for FSM landing detection */
static float    last_baro_alt_agl = 0.0f;

/* Diagnostics for EKF tuning (TEST_MODE=2) */
static float    diag_ned_z = 0.0f;       /* latest NED-Z accel for diagnostics */
static bool     diag_zupt_fired = false;  /* did ZUPT fire this predict cycle  */
#endif /* TEST_MODE != 2 */

/* ── FSM flight configuration defaults ── */
flight_cfg_t g_flight_cfg = {
    .main_deploy_alt   = 250.0f,
    .drogue_fail_vel   = 50.0f,
    .drogue_fail_time  = 3.0f,
    .apogee_pyro_ch    = 0,
    .main_pyro_ch      = 1,
    .apogee_fire_dur   = 1000,
    .main_fire_dur     = 1000,
};

/* ── Bench test mode: CDC text command parser (TEST_MODE 2 only) ── */
#if TEST_MODE == 2
static char    bench_cmd_buf[32];
static uint8_t bench_cmd_idx = 0;

static void bench_send(const char *msg)
{
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
}

static void bench_dispatch(const char *cmd)
{
    typedef struct { const char *name; fsm_state_t state; } cmd_entry_t;
    static const cmd_entry_t state_cmds[] = {
        { "pad",     FSM_STATE_PAD     },
        { "boost",   FSM_STATE_BOOST   },
        { "coast",   FSM_STATE_COAST   },
        { "coast1",  FSM_STATE_COAST_1 },
        { "sustain", FSM_STATE_SUSTAIN },
        { "coast2",  FSM_STATE_COAST_2 },
        { "apogee",  FSM_STATE_APOGEE  },
        { "drogue",  FSM_STATE_DROGUE  },
        { "main",    FSM_STATE_MAIN    },
        { "recovery",FSM_STATE_RECOVERY},
        { "tumble",  FSM_STATE_TUMBLE  },
        { "landed",  FSM_STATE_LANDED  },
    };

    /* Toggle bench mode */
    if (strcmp(cmd, "bench") == 0) {
        bool on = !flight_fsm_bench_active();
        flight_fsm_set_bench_mode(on);
        bench_send(on ? "[BENCH] ON\r\n" : "[BENCH] OFF\r\n");
        return;
    }

#if TEST_MODE != 2
    /* Skip calibration period */
    if (strcmp(cmd, "skip") == 0) {
        if (cal_done) {
            bench_send("[BENCH] cal already done\r\n");
        } else {
            cal_done = true;
            if (baro_cal_count > 0)
                baro_ref = (float)(baro_cal_sum / (double)baro_cal_count);
            else
                baro_ref = ms5611_get_altitude(&baro, 1013.25f);
            bench_send("[BENCH] cal skipped\r\n");
        }
        return;
    }
#endif

    /* State commands — auto-enable bench mode */
    for (int i = 0; i < (int)(sizeof(state_cmds) / sizeof(state_cmds[0])); i++) {
        if (strcmp(cmd, state_cmds[i].name) == 0) {
            flight_fsm_set_bench_mode(true);
            flight_fsm_force_state(state_cmds[i].state);
            char ack[48];
            snprintf(ack, sizeof(ack), "[BENCH] FSM -> %s\r\n", state_cmds[i].name);
            bench_send(ack);
            return;
        }
    }

    /* Buzzer test: numeric 0-100 = volume %, beep 3 times */
    {
        int val = -1;
        const char *p = cmd;
        bool is_num = (*p != '\0');
        while (*p) {
            if (*p < '0' || *p > '9') { is_num = false; break; }
            p++;
        }
        if (is_num) val = atoi(cmd);
        if (val >= 0 && val <= 100) {
            buzzer_beep_n((uint8_t)val, 3, 200, 1000);
            char ack[48];
            snprintf(ack, sizeof(ack), "[BUZZ] %d%%\r\n", val);
            bench_send(ack);
            return;
        }
    }

    bench_send("[BENCH] unknown cmd\r\n");
}

static void bench_process_cdc(void)
{
    while (cdc_ring_available() > 0) {
        uint8_t c = cdc_ring_read_byte();

        if (c == '\r' || c == '\n') {
            if (bench_cmd_idx > 0) {
                bench_cmd_buf[bench_cmd_idx] = '\0';
                bench_dispatch(bench_cmd_buf);
                bench_cmd_idx = 0;
            }
        } else if (c == 0x00) {
            /* COBS delimiter — not a text command, discard accumulated text */
            bench_cmd_idx = 0;
        } else if (c >= 0x20 && c <= 0x7E) {
            if (bench_cmd_idx < sizeof(bench_cmd_buf) - 1) {
                bench_cmd_buf[bench_cmd_idx++] = (char)c;
            } else {
                bench_cmd_idx = 0; /* overflow — discard */
            }
        }
        /* ignore other control chars */
    }
}
#endif

/* ─────────────────────────────────────────────────────────────── */

bool flight_loop_is_cal_done(void)
{
    return cal_done;
}

void flight_loop_init(void)
{
#if TEST_MODE != 2
    last_mag_tick    = 0;
    baro_ref         = 0.0f;
    init_done        = false;
    cal_done         = false;
    mag_new          = false;
    last_imu_tick    = 0;
    baro_cal_sum     = 0.0;
    baro_cal_count   = 0;
    loop_start_tick  = HAL_GetTick();
    imu_subsample_count = 0;
    ned_accel_accum[0] = ned_accel_accum[1] = ned_accel_accum[2] = 0.0f;
    last_accel_mag_g = 0.0f;
    last_baro_alt_agl = 0.0f;
#endif
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Flash dump over CDC — blocking, called post-flight
 *
 *  Protocol:
 *    FW → PC:  "CDMP" (4B magic)
 *    FW → PC:  num_regions (1B)
 *    For each region:
 *      FW → PC:  region_id (1B) + flash_offset (4B LE) + size (4B LE)
 *      For each 4KB chunk:
 *        FW → PC:  4096 bytes (or remainder)
 *        PC → FW:  ACK 0x06 (1 byte)
 *    FW → PC:  "DONE" (4B)
 * ═══════════════════════════════════════════════════════════════════════ */
#if TEST_MODE == 1 && !defined(HIL_MODE)

#define DUMP_CHUNK  4096
#define DUMP_ACK    0x06
#define DUMP_TIMEOUT_MS  5000

static void cdc_send_blocking(const uint8_t *buf, uint16_t len)
{
    while (CDC_Transmit_FS((uint8_t *)buf, len) == USBD_BUSY) {}
    /* Small delay for USB IN transfer to complete */
    HAL_Delay(2);
}

static bool wait_for_ack(void)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < DUMP_TIMEOUT_MS) {
        if (cdc_ring_available() > 0) {
            uint8_t b = cdc_ring_read_byte();
            if (b == DUMP_ACK) return true;
        }
    }
    return false;  /* timeout */
}

static void flash_dump_over_cdc(w25q512jv_t *fl)
{
    typedef struct { uint8_t id; uint32_t offset; uint32_t size; } region_t;
    static const region_t regions[] = {
        { 0, FLASH_INDEX_BASE,   FLASH_INDEX_SIZE   },
        { 1, FLASH_SUMMARY_BASE, FLASH_SUMMARY_SIZE },
        { 2, FLASH_LR_BASE,     FLASH_LR_SIZE      },
        { 3, FLASH_ADXL_BASE,   FLASH_ADXL_SIZE    },
        { 4, FLASH_HR_BASE,     FLASH_HR_SIZE       },
    };
    static const uint8_t num_regions = sizeof(regions) / sizeof(regions[0]);

    /* Let any in-flight CDC TX drain, then flush RX */
    HAL_Delay(50);
    while (cdc_ring_available() > 0) cdc_ring_read_byte();

    /* Header: magic + region count */
    uint8_t hdr[5] = { 'C', 'D', 'M', 'P', num_regions };
    cdc_send_blocking(hdr, sizeof(hdr));

    /* Region descriptors */
    for (int r = 0; r < num_regions; r++) {
        uint8_t desc[9];
        desc[0] = regions[r].id;
        memcpy(&desc[1], &regions[r].offset, 4);
        memcpy(&desc[5], &regions[r].size, 4);
        cdc_send_blocking(desc, sizeof(desc));
    }

    /* Wait for initial ACK (PC is ready) */
    if (!wait_for_ack()) return;

    /* Stream each region */
    static uint8_t chunk_buf[DUMP_CHUNK];
    for (int r = 0; r < num_regions; r++) {
        uint32_t addr = regions[r].offset;
        uint32_t remain = regions[r].size;

        while (remain > 0) {
            uint32_t n = (remain > DUMP_CHUNK) ? DUMP_CHUNK : remain;
            if (w25q512jv_read(fl, addr, chunk_buf, n) != W25Q_OK) {
                /* Fill with 0xFF on read error */
                memset(chunk_buf, 0xFF, n);
            }
            cdc_send_blocking(chunk_buf, (uint16_t)n);
            if (!wait_for_ack()) return;  /* PC timed out */
            addr   += n;
            remain -= n;
        }
    }

    /* End marker */
    uint8_t end[4] = { 'D', 'O', 'N', 'E' };
    cdc_send_blocking(end, sizeof(end));
}

#endif /* TEST_MODE == 1 && !HIL_MODE */

void flight_loop_tick(void)
{
    DIAG_PROBE_BEGIN(probe_superloop);
    /* ── Buzzer state machine ── */
    buzzer_tick();

#if TEST_MODE != 2

#ifdef HIL_MODE
    /* ═══════════════════════════════════════════════════════════════
     *  HIL RAW: event-driven — one full pipeline iteration per packet
     * ═══════════════════════════════════════════════════════════════ */
    cmd_router_process();

    if (!g_hil_raw.pending) return;
    g_hil_raw.pending = false;

    /* Advance virtual clock */
    static uint32_t hil_prev_tick = 0;
    fsm_set_tick(g_hil_raw.tick_ms);
    uint32_t now = g_hil_raw.tick_ms;

    /* Compute dt from consecutive packets */
    float hil_dt = (hil_prev_tick == 0) ? EKF_DT
                 : (float)(now - hil_prev_tick) / 1000.0f;
    if (hil_dt <= 0.0f || hil_dt > 0.1f) hil_dt = EKF_DT;
    hil_prev_tick = now;

    /* Copy injected sensor data */
    float accel_ms2[3], gyro_rads[3];
    memcpy(accel_ms2, g_hil_raw.accel_ms2, sizeof(accel_ms2));
    memcpy(gyro_rads, g_hil_raw.gyro_rads, sizeof(gyro_rads));

    if (g_hil_raw.mag_valid) {
      memcpy(mag_cal_ut, g_hil_raw.mag_ut, sizeof(mag_cal_ut));
      mag_new = true;
    }

    /* Skip calibration if flag set */
    if (g_hil_raw.skip_cal && !cal_done) {
      if (!init_done) {
        const float *mp = g_hil_raw.mag_valid ? mag_cal_ut : NULL;
        if (casper_att_static_init(&att, accel_ms2, mp))
          init_done = true;
      }
      if (init_done) {
        cal_done = true;
        /* Use first baro reading as ground reference */
        if (g_hil_raw.baro_valid && g_hil_raw.baro_pa > 0.0f) {
          float p_hPa = g_hil_raw.baro_pa * 0.01f;
          baro_ref = 44307.694f * (1.0f - powf(p_hPa / 1013.25f, 0.190284f));
        }
      }
    }

    const float *mag_ptr = mag_new ? mag_cal_ut : NULL;

    /* Normal cal / init path (when not skip_cal) */
    if (!cal_done) {
      if (!init_done) {
        if (casper_att_static_init(&att, accel_ms2, mag_ptr))
          init_done = true;
      } else {
        casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, hil_dt);
      }
      if (init_done && (now - loop_start_tick) >= 30000) {
        cal_done = true;
        if (g_hil_raw.baro_valid && g_hil_raw.baro_pa > 0.0f) {
          float p_hPa = g_hil_raw.baro_pa * 0.01f;
          baro_ref = 44307.694f * (1.0f - powf(p_hPa / 1013.25f, 0.190284f));
        }
      }
      mag_new = false;
      return;  /* Still calibrating — no EKF/FSM yet */
    }

    /* ── Attitude update ── */
    casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, hil_dt);
    mag_new = false;

    last_accel_mag_g = sqrtf(accel_ms2[0]*accel_ms2[0]
                           + accel_ms2[1]*accel_ms2[1]
                           + accel_ms2[2]*accel_ms2[2]) / G_CONST;

    /* ── Rotate accel to NED, trapezoidal EKF predict ── */
    {
      float R_mat[9];
      casper_quat_to_rotmat(att.q, R_mat);
      float ned_accel[3];
      ned_accel[0] = R_mat[0]*accel_ms2[0] + R_mat[1]*accel_ms2[1] + R_mat[2]*accel_ms2[2];
      ned_accel[1] = R_mat[3]*accel_ms2[0] + R_mat[4]*accel_ms2[1] + R_mat[5]*accel_ms2[2];
      ned_accel[2] = R_mat[6]*accel_ms2[0] + R_mat[7]*accel_ms2[1] + R_mat[8]*accel_ms2[2];
      diag_ned_z = ned_accel[2];

      if (imu_subsample_count == 0) {
        ned_accel_accum[0] = ned_accel[0];
        ned_accel_accum[1] = ned_accel[1];
        ned_accel_accum[2] = ned_accel[2];
        imu_subsample_count = 1;
      } else {
        float ned_avg[3];
        ned_avg[0] = 0.5f * (ned_accel_accum[0] + ned_accel[0]);
        ned_avg[1] = 0.5f * (ned_accel_accum[1] + ned_accel[1]);
        ned_avg[2] = 0.5f * (ned_accel_accum[2] + ned_accel[2]);

        float dt_predict = 2.0f * hil_dt;
        casper_ekf_predict(&ekf, ned_avg, dt_predict);

        diag_zupt_fired = false;
        {
          fsm_state_t fsm_now = flight_fsm_get_state();
          if (fsm_now == FSM_STATE_PAD || fsm_now == FSM_STATE_LANDED) {
            float amag = last_accel_mag_g * G_CONST;
            if (fabsf(amag - G_CONST) < EKF_ZUPT_THRESHOLD) {
              casper_ekf_update_zupt(&ekf);
              diag_zupt_fired = true;
            }
          }
        }
        imu_subsample_count = 0;
      }
    }

    /* ── Baro update from injected data ── */
    static bool s_baro_fed_ekf = false;
    if (g_hil_raw.baro_valid && g_hil_raw.baro_pa > 0.0f) {
      float p_hPa = g_hil_raw.baro_pa * 0.01f;
      float altitude = 44307.694f * (1.0f - powf(p_hPa / 1013.25f, 0.190284f)) - baro_ref;
      last_baro_alt_agl = altitude;
      casper_ekf_update_baro(&ekf, altitude);
      s_baro_fed_ekf = true;
    }

    (void)s_baro_fed_ekf; /* used by LED code in non-HIL builds */

    /* Store body accel for FSM section */
    last_body_accel_ms2[0] = accel_ms2[0];
    last_body_accel_ms2[1] = accel_ms2[1];
    last_body_accel_ms2[2] = accel_ms2[2];

    /* Pyro manager still needs ticking in HIL */
    pyro_mgr_tick();

#else /* !HIL_MODE — normal sensor-driven path */

    uint32_t now = HAL_GetTick();
    /* ── Mag: polled read at 100 Hz, frame-map + calibrate ── */
    if (now - last_mag_tick >= 10) {
      DIAG_PROBE_BEGIN(probe_mag_rd);
      mmc5983ma_read(&mag);
      DIAG_PROBE_END(probe_mag_rd);
      last_mag_tick = now;
      float mag_raw[3] = {-mag.mag_ut[0], -mag.mag_ut[1], -mag.mag_ut[2]};
      mag_cal_apply(mag_raw, mag_cal_ut);
      mag_new = true;
    }

#ifdef LOGGER_SANITY
    /* IMU watchdog diagnostic counters (used in the watchdog branch below).
     * g_imu_exti_fires is also defined in flight_logger.c but only the
     * EXTI ISR in main.c writes it — flight_loop.c doesn't need it here. */
    extern volatile uint32_t g_imu_wd_polls;
    extern volatile uint32_t g_imu_wd_hits;
#endif

    /* ── IMU servicing: polled at SysTick granularity ─────────────────────
     * EXTI on PC15 is now the primary path (sets data_ready in ISR).
     * The 1 ms watchdog STATUS poll stays as belt-and-braces — fires only
     * when EXTI hasn't latched a sample. */
    if (!imu.data_ready && (now - last_imu_tick) >= 1) {
#ifdef LOGGER_SANITY
      g_imu_wd_polls++;
#endif
      uint8_t status = lsm6dso32_read_reg_ext(&imu, LSM6DSO32_STATUS_REG);
      if (status & 0x01) {  /* XLDA = accel data available */
        imu.data_ready = true;
#ifdef LOGGER_SANITY
        g_imu_wd_hits++;
#endif
      }
    }

    /* ── 833 Hz: IMU read + attitude estimator (interrupt-driven) ── */
    if (imu.data_ready) {
      DIAG_PROBE_BEGIN(probe_imu_svc);
      last_imu_tick = now;
      DIAG_PROBE_BEGIN(probe_imu_read);
      lsm6dso32_read(&imu);
      DIAG_PROBE_END(probe_imu_read);

      /* ── Adaptive dt: measure actual elapsed time since the previous
       * IMU service via the DWT cycle counter. The estimators must be
       * fed real elapsed time, not a constant 1/833 = EKF_DT, otherwise
       * gyro integration and EKF state propagation are systematically
       * scaled by the rate-mismatch factor. With our ~736 Hz observed
       * service rate vs. the 833 Hz the constant assumes, that was a
       * 13% under-integration — visible as cumulative attitude drift
       * and EKF position bias.
       *
       * SYSCLK = 432 MHz so each cycle = 1/432e6 s. DWT->CYCCNT is a
       * 32-bit free-running counter (rolls over every ~10 s), so
       * unsigned subtraction handles wrap correctly. On the very first
       * call we have no history, so fall back to EKF_DT.
       *
       * Also clamp the upper bound: if a long-running call (e.g. radio
       * TX, USB stall) blocks us for >5 ms, don't feed that gigantic
       * dt to the filter — it would destabilise. Better to drop a
       * step's accuracy than blow up the estimator. */
      float dt_actual;
      {
          static uint32_t s_prev_imu_cyc = 0;
          uint32_t cyc = DWT->CYCCNT;
          if (s_prev_imu_cyc == 0u) {
              dt_actual = EKF_DT;
          } else {
              uint32_t delta = cyc - s_prev_imu_cyc;  /* unsigned wrap-safe */
              dt_actual = (float)delta * (1.0f / 432000000.0f);
              if (dt_actual > 0.005f) dt_actual = 0.005f;
              if (dt_actual < 0.0001f) dt_actual = 0.0001f;
          }
          s_prev_imu_cyc = cyc;
      }

      float gyro_sensor[3] = {
        imu.gyro_dps[0] * DEG2RAD,
        imu.gyro_dps[1] * DEG2RAD,
        imu.gyro_dps[2] * DEG2RAD
      };
#ifdef GYRO_TEMP_COMP
      {
        float dT = imu.temp_c - GYRO_TC_T0;
        gyro_sensor[0] -= GYRO_TC_SLOPE_X * dT;
        gyro_sensor[1] -= GYRO_TC_SLOPE_Y * dT;
        gyro_sensor[2] -= GYRO_TC_SLOPE_Z * dT;
      }
#endif

      float accel_ms2[3] = {
        imu.accel_g[0] * G_CONST,   /* body X = sensor X (starboard)       */
        imu.accel_g[1] * G_CONST,   /* body Y = sensor Y (nose, up on pad) */
        imu.accel_g[2] * G_CONST    /* body Z = sensor Z (toward operator) */
      };
      float gyro_rads[3] = {
        gyro_sensor[0],              /* body X = sensor X (starboard)       */
        gyro_sensor[1],              /* body Y = sensor Y (nose)            */
        gyro_sensor[2]               /* body Z = sensor Z (toward operator) */
      };

      const float *mag_ptr = mag_new ? mag_cal_ut : NULL;

      if (!cal_done) {
        if (!init_done) {
          if (casper_att_static_init(&att, accel_ms2, mag_ptr)) {
            init_done = true;
            HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
          } else {
            uint32_t mc = att.mag_count;
            if (mc >= 125)
              HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
            if (mc >= 250)
              HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
            if (mc >= 375)
              HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
            if (mc >= 500)
              HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
          }
        } else {
          casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, dt_actual);
        }

        if (init_done && (now - loop_start_tick) >= 30000) {
          /* att.launched stays false — Mahony gravity correction remains
           * active until FSM transitions to BOOST (PAD→BOOST). */
          cal_done = true;
          if (baro_cal_count > 0)
            baro_ref = (float)(baro_cal_sum / (double)baro_cal_count);
          else
            baro_ref = ms5611_get_altitude(&baro, 1013.25f);
          HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
        }
      } else {
        /* Attitude update at actual sample rate (see adaptive-dt block above) */
        DIAG_PROBE_BEGIN(probe_att_upd);
        casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, dt_actual);
        DIAG_PROBE_END(probe_att_upd);

        /* Compute accel magnitude for launch detection */
        last_accel_mag_g = sqrtf(accel_ms2[0]*accel_ms2[0]
                               + accel_ms2[1]*accel_ms2[1]
                               + accel_ms2[2]*accel_ms2[2]) / G_CONST;

        /* Store body accel for FSM section */
        last_body_accel_ms2[0] = accel_ms2[0];
        last_body_accel_ms2[1] = accel_ms2[1];
        last_body_accel_ms2[2] = accel_ms2[2];

        /* 833 Hz: rotate accel to NED using current quaternion */
        float R_mat[9];
        casper_quat_to_rotmat(att.q, R_mat);
        float ned_accel[3];
        ned_accel[0] = R_mat[0]*accel_ms2[0] + R_mat[1]*accel_ms2[1] + R_mat[2]*accel_ms2[2];
        ned_accel[1] = R_mat[3]*accel_ms2[0] + R_mat[4]*accel_ms2[1] + R_mat[5]*accel_ms2[2];
        ned_accel[2] = R_mat[6]*accel_ms2[0] + R_mat[7]*accel_ms2[1] + R_mat[8]*accel_ms2[2];
        diag_ned_z = ned_accel[2];

        /* Trapezoidal NED-accel accumulation -> EKF predict every 2nd sample.
         * dt_predict accumulates the two adaptive sample intervals so the
         * predict step gets the true elapsed time, not 2*EKF_DT. */
        if (imu_subsample_count == 0) {
          ned_accel_accum[0] = ned_accel[0];
          ned_accel_accum[1] = ned_accel[1];
          ned_accel_accum[2] = ned_accel[2];
          dt_predict_accum   = dt_actual;
          imu_subsample_count = 1;
        } else {
          /* Trapezoidal average of 2 NED-frame samples */
          float ned_avg[3];
          ned_avg[0] = 0.5f * (ned_accel_accum[0] + ned_accel[0]);
          ned_avg[1] = 0.5f * (ned_accel_accum[1] + ned_accel[1]);
          ned_avg[2] = 0.5f * (ned_accel_accum[2] + ned_accel[2]);

          float dt_predict = dt_predict_accum + dt_actual;
          DIAG_PROBE_BEGIN(probe_ekf_pred);
          casper_ekf_predict(&ekf, ned_avg, dt_predict);
          DIAG_PROBE_END(probe_ekf_pred);

          /* ZUPT at predict rate (416 Hz): clamp velocity when stationary */
          diag_zupt_fired = false;
          {
            fsm_state_t fsm_now = flight_fsm_get_state();
            if (fsm_now == FSM_STATE_PAD || fsm_now == FSM_STATE_LANDED) {
              float accel_mag = last_accel_mag_g * G_CONST;
              if (fabsf(accel_mag - G_CONST) < EKF_ZUPT_THRESHOLD) {
                casper_ekf_update_zupt(&ekf);
                diag_zupt_fired = true;
              }
            }
          }

          imu_subsample_count = 0;
        }

#if TEST_MODE == 1
        /* ── Logger: HR record + peak tracking (833 Hz, internal divider) ── */
        {
            uint8_t fsm_now = (uint8_t)flight_fsm_get_state();
            float tilt_deg = acosf(R_mat[8]) * 57.2957795f;
            float roll_dps = fabsf(imu.gyro_dps[2]);

            uint8_t flags = 0;
            if (gps.fix_type >= 2)                    flags |= 0x02;
            if (adxl372_activity_detected(&high_g))   flags |= 0x04;
            if (diag_zupt_fired)                      flags |= 0x08;
            if (ekf.baro_gated)                       flags |= 0x10;
            if (mag_new)                              flags |= 0x20;
            if (pyro_mgr_get_arm_bitmap())            flags |= 0x40;

            DIAG_PROBE_BEGIN(probe_hr_push);
            flight_logger_push_hr(&logger, &imu, &baro, &high_g, &mag,
                                  &ekf, &att, fsm_now, flags, 0);
            DIAG_PROBE_END(probe_hr_push);

            flight_logger_summary_imu(&logger, last_accel_mag_g, tilt_deg,
                                      roll_dps, fsm_now);

            float temp_k = imu.temp_c + 273.15f;
            flight_logger_summary_ekf(&logger, ekf.x[1], ekf.x[0],
                                      baro.pressure, temp_k);
        }
#endif
      }

      mag_new = false;
      DIAG_PROBE_END(probe_imu_svc);
    }

    /* ── Async baro: accumulate during cal, feed EKF after cal_done ── */
    static bool s_baro_fed_ekf = false;
    static float last_baro_alt = 0.0f; (void)last_baro_alt;
    if (ms5611_tick(&baro)) {
      if (cal_done) {
        float altitude = ms5611_get_altitude(&baro, 1013.25f) - baro_ref;
        last_baro_alt = altitude;
        last_baro_alt_agl = altitude;
        casper_ekf_update_baro(&ekf, altitude);

        s_baro_fed_ekf = true;
#ifdef LOGGER_SANITY
        (void)s_baro_fed_ekf;  /* LED reader gated out under LOGGER_SANITY */
#endif
      } else if (init_done) {
        /* Accumulate baro altitude for ground-level reference */
        baro_cal_sum += (double)ms5611_get_altitude(&baro, 1013.25f);
        baro_cal_count++;
      }
    }

    /* ── GPS: non-blocking poll (10 Hz NAV-PVT) ── */
    if (max_m10m_tick(&gps)) {
      if (cal_done && max_m10m_has_3d_fix(&gps)) {
        casper_ekf_update_gps_alt(&ekf, gps.alt_msl_m);
        casper_ekf_update_gps_vel(&ekf, gps.vel_d_m_s);
      }
#if TEST_MODE == 1
      flight_logger_summary_gps(&logger, &gps);
#endif
    }

    /* ── Pyro manager tick (wraps casper_pyro_tick) ── */
    pyro_mgr_tick();

#endif /* HIL_MODE */

#endif /* TEST_MODE != 2 — end of nav stack gate */

#if TEST_MODE != 2
#ifndef HIL_MODE
#ifndef LOGGER_SANITY
    /* ── EKF diagnostic LEDs (override pyro LEDs after init) ── */
    if (cal_done) {
      static uint32_t ekf_led_tick = 0;
      if (now - ekf_led_tick >= 500) {
        HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
        ekf_led_tick = now;
      }
      if (s_baro_fed_ekf) {
        HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
      }
    }
#endif
#endif

    /* ── Build telemetry state + FSM tick (all modes) ── */
    {
      fc_telem_state_t tstate;
      tstate.alt_m        = ekf.x[0];
      tstate.vel_mps      = ekf.x[1];
      tstate.quat[0]      = att.q[0];
      tstate.quat[1]      = att.q[1];
      tstate.quat[2]      = att.q[2];
      tstate.quat[3]      = att.q[3];
      tstate.batt_v       = 7.4f;
      tstate.flight_time_s = flight_fsm_get_time_s();
      tstate.accel_mag_g   = last_accel_mag_g;
      tstate.baro_alt_m    = last_baro_alt_agl;
#ifdef HIL_MODE
      tstate.adxl_activity = false;
      tstate.adxl_available = false;
      (void)tstate; /* used by radio_manager in non-HIL builds */
#else
      tstate.adxl_activity = adxl372_activity_detected(&high_g);
      tstate.adxl_available = (high_g.device_id == ADXL372_DEVID_VAL);
#endif

#ifndef HIL_MODE
      if (flight_fsm_sim_active()) {
        flight_fsm_sim_get_state(&tstate);
        tstate.batt_v = 7.4f;
      }
#endif

      /* Build fsm_input_t for sensor-driven transitions */
      fsm_input_t fsm_in = {0};
      fsm_in.alt_m               = ekf.x[0];
      fsm_in.vel_mps             = ekf.x[1];
      fsm_in.vert_accel_g        = compute_vert_accel(att.q, last_body_accel_ms2);
      static float diag_pitch_deg = 0.0f, diag_yaw_deg = 0.0f, diag_tilt_deg = 0.0f;
      fsm_in.antenna_up          = check_antenna_up_tilt(att.q, &diag_pitch_deg,
                                     &diag_yaw_deg, &diag_tilt_deg);
      fsm_in.flight_time_s       = flight_fsm_get_time_s();
      fsm_in.main_deploy_alt_m   = g_flight_cfg.main_deploy_alt;
      fsm_in.drogue_fail_vel_mps = g_flight_cfg.drogue_fail_vel;
      fsm_in.drogue_fail_time_s  = g_flight_cfg.drogue_fail_time;
      fsm_in.apogee_pyro_ch      = g_flight_cfg.apogee_pyro_ch;
      fsm_in.main_pyro_ch        = g_flight_cfg.main_pyro_ch;
      fsm_in.apogee_fire_dur_ms  = g_flight_cfg.apogee_fire_dur;
      fsm_in.main_fire_dur_ms    = g_flight_cfg.main_fire_dur;

      fsm_state_t fsm = flight_fsm_tick(&fsm_in);

      /* On state transitions: ADXL mode, logger lifecycle */
      {
        static fsm_state_t prev_fsm = FSM_STATE_PAD;
        if (prev_fsm != fsm) {
#if TEST_MODE == 1
          flight_logger_summary_event(&logger, (uint8_t)fsm, (uint8_t)prev_fsm);
          flight_logger_set_rate(&logger, (uint8_t)fsm);
#endif
        }

        if (prev_fsm == FSM_STATE_PAD && fsm == FSM_STATE_BOOST) {
          /* Switch attitude to flight mode: freeze gyro bias,
           * drop gravity correction, keep 10Hz mag only. */
          att.launched = true;
          att.e_int[0] = att.e_int[1] = att.e_int[2] = 0.0f;
#ifndef HIL_MODE
          adxl372_enter_measurement(&high_g);
#endif
#if TEST_MODE == 1
#ifndef LOGGER_SANITY
          flight_logger_launch(&logger);
          buzzer_beep_n(30, 2, 100, 200);  /* 2 short beeps = launch */
#endif
#endif
        }

#if TEST_MODE == 1
        /* Apogee: snapshot EKF/baro/GPS altitude, write partial summary */
        if (prev_fsm != FSM_STATE_APOGEE && fsm == FSM_STATE_APOGEE) {
          logger.summary.apogee_ekf_m    = ekf.x[0];
          logger.summary.apogee_baro_m   = last_baro_alt_agl;
          logger.summary.apogee_gps_msl_m = gps.alt_msl_m;
          flight_logger_write_partial_summary(&logger);
          buzzer_beep_n(30, 3, 100, 200);  /* 3 short beeps = apogee */
        }

        /* Main deploy: stop ADXL stream */
        if (prev_fsm != FSM_STATE_MAIN && fsm == FSM_STATE_MAIN)
          log_stream_finalize(&logger.adxl);

        /* Landed: record 5 more seconds, then finalize */
        static uint32_t landed_at = 0;
        if (prev_fsm != FSM_STATE_LANDED && fsm == FSM_STATE_LANDED) {
          landed_at = now;
        }
        if (fsm == FSM_STATE_LANDED && landed_at != 0
            && (now - landed_at >= 5000) && !logger.finalized) {
#ifndef LOGGER_SANITY
          flight_logger_finalize(&logger);
          buzzer_beep_n(30, 5, 200, 300);  /* 5 long beeps = finalized */
#endif
          landed_at = 0;
        }
#endif

        prev_fsm = fsm;
      }

#if TEST_MODE == 1
#ifdef FLIGHT_ASCII_PLOT
      /* ── Serial plotter telemetry ── */
#ifdef HIL_MODE
      /* HIL: output every packet (PC controls rate), all FSM states */
      {
        char buf[200];
        int len = snprintf(buf, sizeof(buf),
            ">alt:%.2f,vel:%.2f,pitch:%.2f,yaw:%.2f,tilt:%.2f,"
            "ant_up:%d,vaccel:%.2f,fsm:%u,amag:%.2f,"
            "baro:%.2f,ab:%.4f,bb:%.4f\r\n",
            (double)ekf.x[0],
            (double)ekf.x[1],
            (double)diag_pitch_deg,
            (double)diag_yaw_deg,
            (double)diag_tilt_deg,
            (int)fsm_in.antenna_up,
            (double)fsm_in.vert_accel_g,
            (unsigned)fsm,
            (double)last_accel_mag_g,
            (double)last_baro_alt_agl,
            (double)ekf.x[2],
            (double)ekf.x[3]);
        if (len > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
      }
#else
      /* Normal: 10 Hz output, suppressed during flash dump */
      if (!cmd_router_dump_requested()) {
        static uint32_t ascii_last = 0;
        if (now - ascii_last >= 100) {
          ascii_last = now;
          char buf[200];
          int len;
          if (fsm == FSM_STATE_PAD) {
            /* Serial plotter: >name:value pairs */
            float euler[3];
            casper_quat_to_euler(att.q, euler);
            float heading_deg = euler[2] * 57.2957795f;
            float mag_norm = sqrtf(mag_cal_ut[0]*mag_cal_ut[0]
                                 + mag_cal_ut[1]*mag_cal_ut[1]
                                 + mag_cal_ut[2]*mag_cal_ut[2]);
            len = snprintf(buf, sizeof(buf),
                ">pitch:%.2f,yaw:%.2f,tilt:%.2f,ant_up:%d,vaccel:%.2f,fsm:%u,heading:%.1f,mag:%.1f\r\n",
                (double)diag_pitch_deg,
                (double)diag_yaw_deg,
                (double)diag_tilt_deg,
                (int)fsm_in.antenna_up,
                (double)fsm_in.vert_accel_g,
                (unsigned)fsm,
                (double)heading_deg,
                (double)mag_norm);
          } else {
            len = snprintf(buf, sizeof(buf),
                ">alt:%.1f,vel:%.1f,vaccel:%.2f,fsm:%u,t:%.1f\r\n",
                (double)tstate.alt_m, (double)tstate.vel_mps,
                (double)fsm_in.vert_accel_g, (unsigned)fsm,
                (double)tstate.flight_time_s);
          }
          if (len > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
        }
      }
#endif /* HIL_MODE */
#endif /* FLIGHT_ASCII_PLOT */

#ifndef HIL_MODE
        /* ── Radio TX + RX window ── */
        {
          pyro_state_t pstate = {0};
          uint8_t arm_bm  = pyro_mgr_get_arm_bitmap();
          uint8_t cont_bm = pyro_mgr_get_cont_bitmap();
          pstate.armed[0] = (arm_bm & 0x01) != 0;
          pstate.armed[1] = (arm_bm & 0x02) != 0;
          pstate.armed[2] = (arm_bm & 0x04) != 0;
          pstate.armed[3] = (arm_bm & 0x08) != 0;
          pstate.continuity[0] = (cont_bm & 0x01) != 0;
          pstate.continuity[1] = (cont_bm & 0x02) != 0;
          pstate.continuity[2] = (cont_bm & 0x04) != 0;
          pstate.continuity[3] = (cont_bm & 0x08) != 0;
          pstate.fired = pyro_mgr_is_firing();
          DIAG_PROBE_BEGIN(probe_radio);
          radio_manager_tick(&ekf, &tstate, &pstate, fsm);
          DIAG_PROBE_END(probe_radio);
        }

        /* ── Logger: LR record (pyro + radio + GPS) ── */
        {
          uint16_t pyro_cont[4];
          DIAG_PROBE_BEGIN(probe_pyro_rd);
          pyro_mgr_get_cont_adc(pyro_cont);
          uint8_t pyro_st = (uint8_t)((pyro_mgr_get_cont_bitmap() << 4)
                                     | pyro_mgr_get_arm_bitmap());
          DIAG_PROBE_END(probe_pyro_rd);
          int8_t rssi, snr;
          uint16_t tx_cnt, rx_cnt, fail_cnt;
          radio_get_stats(&rssi, &snr, &tx_cnt, &rx_cnt, &fail_cnt);
          flight_logger_push_lr(&logger, pyro_st, pyro_cont,
                                rssi, snr, tx_cnt, rx_cnt, fail_cnt, &gps);
        }
#endif /* !HIL_MODE */
#endif /* TEST_MODE == 1 */
    }
#endif /* TEST_MODE != 2 */

#if TEST_MODE == 1
#ifndef HIL_MODE
    /* ── Process incoming CDC commands ── */
    if (cdc_ring_available() > 0) {
      cmd_router_process();
    }

    /* ── Flash dump over CDC (blocking, post-flight only) ── */
    if (cmd_router_dump_requested()) {
      cmd_router_dump_clear();
      flash_dump_over_cdc(&flash);
    }

    /* ── CAC confirm timeout check ── */
    cac_tick();

    /* ── Flight logger QSPI dispatch (erase-ahead on PAD, page writes in flight) ── */
    DIAG_PROBE_BEGIN(probe_log_tick);
    flight_logger_tick(&logger);
    DIAG_PROBE_END(probe_log_tick);
#ifdef LOGGER_SANITY
    logger_sanity_tick(HAL_GetTick());
#endif

#ifndef LOGGER_SANITY
    /* ── Logger status LEDs ── */
    {
      /* LED2 (PB14): SOLID = launched (DRAIN), FAST BLINK = finalized */
      if (logger.launched && !logger.finalized) {
        HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
      } else if (logger.finalized) {
        static uint32_t fin_blink = 0;
        if (now - fin_blink >= 200) {
          HAL_GPIO_TogglePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin);
          fin_blink = now;
        }
      }
      /* LED3 (PE8): ON when QSPI busy, OFF when idle */
      HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin,
          logger.qspi_state != QSPI_IDLE ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
#endif
#endif /* !HIL_MODE */
#elif TEST_MODE == 2
    /* ── Bench test: process text commands from CDC ── */
    bench_process_cdc();
#endif
    DIAG_PROBE_END(probe_superloop);
}
