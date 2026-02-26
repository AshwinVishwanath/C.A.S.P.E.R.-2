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
static const float DEG2RAD  = 0.0174532925f;

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

/* Last computed accel magnitude for FSM launch detection */
static float    last_accel_mag_g = 0.0f;

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

/* ── Bench test mode: CDC text command parser ── */
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
            if (init_done) {
                att.launched = true;
                att.e_int[0] = att.e_int[1] = att.e_int[2] = 0.0f;
            }
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

/* ─────────────────────────────────────────────────────────────── */

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

void flight_loop_tick(void)
{
    /* ── Buzzer state machine ── */
    buzzer_tick();

#if TEST_MODE != 2
    uint32_t now = HAL_GetTick();
    /* ── Mag: polled read at 100 Hz, frame-map + calibrate ── */
    if (now - last_mag_tick >= 10) {
      mmc5983ma_read(&mag);
      last_mag_tick = now;
      float mag_raw[3] = {-mag.mag_ut[0], -mag.mag_ut[1], -mag.mag_ut[2]};
      mag_cal_apply(mag_raw, mag_cal_ut);
      mag_new = true;
    }

    /* ── IMU interrupt watchdog: poll STATUS if EXTI hasn't fired in 5ms ── */
    if (!imu.data_ready && (now - last_imu_tick) > 5) {
      uint8_t status = lsm6dso32_read_reg_ext(&imu, LSM6DSO32_STATUS_REG);
      if (status & 0x01)  /* XLDA = accel data available */
        imu.data_ready = true;
    }

    /* ── 833 Hz: IMU read + attitude estimator (interrupt-driven) ── */
    if (imu.data_ready) {
      last_imu_tick = now;
      lsm6dso32_read(&imu);

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
          casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, EKF_DT);
        }

        if (init_done && (now - loop_start_tick) >= 30000) {
          att.launched = true;
          att.e_int[0] = att.e_int[1] = att.e_int[2] = 0.0f;
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
        /* 833 Hz: attitude update (unchanged rate) */
        casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, EKF_DT);

        /* Compute accel magnitude for launch detection */
        last_accel_mag_g = sqrtf(accel_ms2[0]*accel_ms2[0]
                               + accel_ms2[1]*accel_ms2[1]
                               + accel_ms2[2]*accel_ms2[2]) / G_CONST;

        /* 833 Hz: rotate accel to NED using current quaternion */
        float R_mat[9];
        casper_quat_to_rotmat(att.q, R_mat);
        float ned_accel[3];
        ned_accel[0] = R_mat[0]*accel_ms2[0] + R_mat[1]*accel_ms2[1] + R_mat[2]*accel_ms2[2];
        ned_accel[1] = R_mat[3]*accel_ms2[0] + R_mat[4]*accel_ms2[1] + R_mat[5]*accel_ms2[2];
        ned_accel[2] = R_mat[6]*accel_ms2[0] + R_mat[7]*accel_ms2[1] + R_mat[8]*accel_ms2[2];
        diag_ned_z = ned_accel[2];

        /* 416 Hz: trapezoidal accumulation -> EKF predict every 2nd sample */
        if (imu_subsample_count == 0) {
          ned_accel_accum[0] = ned_accel[0];
          ned_accel_accum[1] = ned_accel[1];
          ned_accel_accum[2] = ned_accel[2];
          imu_subsample_count = 1;
        } else {
          /* Trapezoidal average of 2 NED-frame samples */
          float ned_avg[3];
          ned_avg[0] = 0.5f * (ned_accel_accum[0] + ned_accel[0]);
          ned_avg[1] = 0.5f * (ned_accel_accum[1] + ned_accel[1]);
          ned_avg[2] = 0.5f * (ned_accel_accum[2] + ned_accel[2]);

          float dt_predict = 2.0f * EKF_DT;
          casper_ekf_predict(&ekf, ned_avg, dt_predict);

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
      }

      mag_new = false;
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
      } else if (init_done) {
        /* Accumulate baro altitude for ground-level reference */
        baro_cal_sum += (double)ms5611_get_altitude(&baro, 1013.25f);
        baro_cal_count++;
      }
    }

    /* ── GPS: non-blocking poll (10 Hz NAV-PVT) ── */
#if TEST_MODE == 1
    static bool gps_new_fix = false;
    gps_new_fix = false;
#endif
    if (max_m10m_tick(&gps)) {
#if TEST_MODE == 1
      gps_new_fix = true;
#endif
      if (cal_done && max_m10m_has_3d_fix(&gps)) {
        casper_ekf_update_gps_alt(&ekf, gps.alt_msl_m);
        casper_ekf_update_gps_vel(&ekf, gps.vel_d_m_s);
      }
    }

    /* ── Pyro manager tick (wraps casper_pyro_tick) ── */
    pyro_mgr_tick();

#endif /* TEST_MODE != 2 — end of nav stack gate */

#if TEST_MODE != 2
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
      tstate.adxl_activity = adxl372_activity_detected(&high_g);
      tstate.adxl_available = (high_g.device_id == ADXL372_DEVID_VAL);

      if (flight_fsm_sim_active()) {
        flight_fsm_sim_get_state(&tstate);
        tstate.batt_v = 7.4f;
      }

      /* Build fsm_input_t for sensor-driven transitions */
      fsm_input_t fsm_in = {0};
      fsm_in.alt_m               = ekf.x[0];
      fsm_in.vel_mps             = ekf.x[1];
      fsm_in.vert_accel_g        = compute_vert_accel(att.q, (float[]){
                                     imu.accel_g[0] * G_CONST,
                                     imu.accel_g[1] * G_CONST,
                                     imu.accel_g[2] * G_CONST });
      fsm_in.antenna_up          = check_antenna_up(att.q);
      fsm_in.flight_time_s       = flight_fsm_get_time_s();
      fsm_in.main_deploy_alt_m   = g_flight_cfg.main_deploy_alt;
      fsm_in.drogue_fail_vel_mps = g_flight_cfg.drogue_fail_vel;
      fsm_in.drogue_fail_time_s  = g_flight_cfg.drogue_fail_time;
      fsm_in.apogee_pyro_ch      = g_flight_cfg.apogee_pyro_ch;
      fsm_in.main_pyro_ch        = g_flight_cfg.main_pyro_ch;
      fsm_in.apogee_fire_dur_ms  = g_flight_cfg.apogee_fire_dur;
      fsm_in.main_fire_dur_ms    = g_flight_cfg.main_fire_dur;

      fsm_state_t fsm = flight_fsm_tick(&fsm_in);

      /* On launch: transition ADXL372 from wake-up to measurement mode */
      {
        static fsm_state_t prev_fsm = FSM_STATE_PAD;
        if (prev_fsm == FSM_STATE_PAD && fsm == FSM_STATE_BOOST) {
          adxl372_enter_measurement(&high_g);
        }
        prev_fsm = fsm;
      }

#if TEST_MODE == 1
      /* ── COBS binary telemetry (10 Hz via tlm_tick) ── */
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
        tlm_tick(&tstate, &pstate, fsm);

        /* ── GPS telemetry on new fix ── */
        if (gps_new_fix) {
          fc_gps_state_t gstate;
          gstate.dlat_mm   = 0;
          gstate.dlon_mm   = 0;
          gstate.alt_msl_m = gps.alt_msl_m;
          gstate.fix_type  = gps.fix_type;
          gstate.sat_count = gps.num_sv;
          tlm_send_gps(&gstate);
        }

        /* ── Radio telemetry TX + RX window ── */
        radio_manager_tick(&ekf, &tstate, &pstate, fsm);
      }
#endif /* TEST_MODE == 1 */
    }
#endif /* TEST_MODE != 2 */

#if TEST_MODE == 1
    /* ── Process incoming CDC commands ── */
    if (cdc_ring_available() > 0) {
      cmd_router_process();
    }

    /* ── CAC confirm timeout check ── */
    cac_tick();
#elif TEST_MODE == 2
    /* ── Bench test: process text commands from CDC ── */
    bench_process_cdc();
#endif
}
