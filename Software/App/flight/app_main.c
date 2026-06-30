/**
 * @file  app_main.c
 * @brief Portable application entry point — init and superloop body.
 *
 * Contains exactly the logic that previously lived in main.c USER CODE
 * regions, relocated here verbatim (pure behaviour-preserving move):
 *
 *   USER CODE BEGIN Includes  → #include list below
 *   USER CODE BEGIN PV        → global driver-instance definitions
 *   USER CODE BEGIN 2         → app_init()
 *   USER CODE BEGIN 3         → app_tick()
 *
 * HARD RULE: This file must NOT include stm32h7xx_hal.h or main.h.
 * All hardware coupling goes through casper_port.h / board_casper2.h.
 * HAL_GPIO_WritePin/TogglePin → casper_gpio_write/toggle.
 * HAL_Delay/HAL_GetTick       → casper_delay_ms/casper_millis.
 * Board-specific pin fixups (PC2, I2C3_INT, EXTI enables) are behind
 * casper_board_* helpers declared in board_casper2.h.
 *
 * Approved non-HAL middleware includes: usbd_cdc_if.h, fatfs.h (both
 * come from ST USB / FatFs middleware, not HAL, and have no substitute
 * in the port seam).
 */

/* ---- standard library --------------------------------------------------- */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ---- casper port seam --------------------------------------------------- */
#include "casper_port.h"
#include "board_casper2.h"

/* ---- USB CDC middleware (not HAL) --------------------------------------- */
#if (USB_MODE != 2)
#include "usbd_cdc_if.h"
#endif

/* ---- FatFS middleware (not HAL) ----------------------------------------- */
#ifndef BUILD_TARGET_GROUND
#include "fatfs.h"
#endif

/* ---- sensor drivers ----------------------------------------------------- */
#include "ms5611.h"
#ifndef BUILD_TARGET_GROUND
#include "lsm6dso32.h"
#include "adxl372.h"
#include "w25q512jv.h"
#endif
#include "max_m10m.h"
#ifndef BUILD_TARGET_GROUND
#include "mmc5983ma.h"
#include "mag_cal.h"
#ifdef MAG_VAL
#include "mag_val.h"
#endif
#ifdef GYRO_TEMP_CAL
#include "temp_cal.h"
#endif
#endif /* !BUILD_TARGET_GROUND */

/* ---- navigation / estimation -------------------------------------------- */
#ifndef BUILD_TARGET_GROUND
#include "casper_ekf.h"
#include "casper_attitude.h"
#include "casper_quat.h"
#endif

/* ---- telemetry / command / FSM / pyro / radio / logger ------------------ */
#ifndef BUILD_TARGET_GROUND
#include "tlm_manager.h"
#include "cmd_router.h"
#include "cac_handler.h"
#include "cfg_manager.h"
#include "flight_fsm.h"
#include "pyro_manager.h"
#include "self_test.h"
#include "flight_loop.h"
#include "radio_manager.h"
#include "flight_logger.h"
#ifdef LOGGER_SANITY
#include "logger_sanity.h"
#include "cycle_probe.h"
#endif
#endif /* !BUILD_TARGET_GROUND */

/* ---- shared (both builds) ----------------------------------------------- */
#include "crc32_hw.h"
#include "radio_irq.h"
#include "buzzer.h"

/* ---- ground station ----------------------------------------------------- */
#ifdef BUILD_TARGET_GROUND
#include "ground_main.h"
#endif

/* ---- compile-time sanity ------------------------------------------------ */
#if defined(MAG_CAL) + defined(MAG_VAL) + defined(GYRO_TEMP_CAL) + defined(GPS_TEST) > 1
#error "Only one calibration/test mode may be defined at a time"
#endif

/* =========================================================================
 *  Driver-instance definitions  (extern declarations live in app_globals.h)
 *
 *  Moved here from main.c USER CODE BEGIN PV.  The application owns these;
 *  other translation units reach them via the extern declarations in
 *  app_globals.h (included through app_main.h).
 * ======================================================================= */

ms5611_t baro;
#ifndef BUILD_TARGET_GROUND
lsm6dso32_t     imu;
adxl372_t       high_g;
w25q512jv_t     flash;
casper_ekf_t    ekf;
casper_attitude_t att;
#endif
max_m10m_t gps;
#ifndef BUILD_TARGET_GROUND
mmc5983ma_t      mag;
flight_logger_t  logger;
#endif

/* =========================================================================
 *  Calibration / test mode state objects.
 *
 *  Declared at file scope so both app_init() (which initialises them) and
 *  app_tick() (which calls their tick functions) can reach them without
 *  dynamic allocation or passing pointers across the init/tick boundary.
 *
 *  Each block is guarded by the same #ifdef as the code that uses it;
 *  the object is zero-size / absent when the mode is not compiled in.
 * ======================================================================= */
#if defined(MAG_CAL) && !defined(BUILD_TARGET_GROUND)
static mag_cal_t  s_mcal;
#endif
#if defined(MAG_VAL) && !defined(BUILD_TARGET_GROUND)
static mag_val_t  s_mval;
#endif
#if defined(GYRO_TEMP_CAL) && !defined(BUILD_TARGET_GROUND)
static temp_cal_t s_tcal;
#endif

/* =========================================================================
 *  app_init()
 *  ── Moved verbatim from main.c USER CODE BEGIN 2 .. END 2 ──────────────
 *
 *  All HAL_GPIO_WritePin → casper_gpio_write,
 *      HAL_GPIO_TogglePin → casper_gpio_toggle,
 *      HAL_Delay          → casper_delay_ms.
 *  Board-specific fixups (PC2 MISO, I2C3_INT, EXTI enables, pyro safe-mode,
 *  GS SPI1 handle retrieval) delegated to casper_board_* helpers so this
 *  file stays HAL-free.
 * ======================================================================= */
void app_init(void)
{
  /* ── 5-second USB enumeration window ──
   * Wait here so the user can open a serial terminal before any
   * sensor init runs.  All CDC debug prints after this point will
   * be visible. */
  casper_delay_ms(5000);

#ifdef BUILD_TARGET_GROUND
  /* ── Ground station init ──────────────────────────────────────── */
  {
    char _dbg[80]; int _len;
    #define DBG_PRINT(msg) do { \
      _len = snprintf(_dbg, sizeof(_dbg), msg); \
      CDC_Transmit_FS((uint8_t *)_dbg, (uint16_t)_len); \
      casper_delay_ms(20); \
    } while(0)

    DBG_PRINT("[INIT] Ground station mode\r\n");
    DBG_PRINT("[INIT] MS5611...\r\n");
    if (!ms5611_init(&baro, &BSP_SPI_BARO, BSP_PIN_BARO_CS)) {
      for (int i = 0; i < 6; i++) {
        casper_gpio_toggle(BSP_PIN_CONT_YN_3);
        casper_gpio_toggle(BSP_PIN_CONT_YN_4);
        casper_delay_ms(200);
      }
    }
    ms5611_set_oversampling(&baro, MS5611_OSR_2048);

    DBG_PRINT("[INIT] GPS...\r\n");
    if (!max_m10m_init(&gps, &BSP_I2C_GPS, BSP_PIN_GPS_NRST)) {
      for (int i = 0; i < 6; i++) {
        casper_gpio_toggle(BSP_PIN_CONT_YN_3);
        casper_gpio_toggle(BSP_PIN_CONT_YN_4);
        casper_delay_ms(200);
      }
    }

    DBG_PRINT("[INIT] CRC + radio...\r\n");
    crc32_hw_init();
    buzzer_init(&BSP_PWM_BUZZ);

    DBG_PRINT("[INIT] ground_main_init...\r\n");
    /* ground_main_init() and radio_manager_init() both accept a void* that
     * is cast back to SPI_HandleTypeDef* inside their respective .c files
     * (which are allowed to include HAL).  We obtain the raw pointer from
     * the board helper casper_board_spi1_handle() so this file never needs
     * to include main.h or reference hspi1 directly. */
    ground_main_init(casper_board_spi1_handle(), &baro, &gps);

    DBG_PRINT("[INIT] Ground station init complete\r\n");
  }

  /* Enable EXTI1 for SX1276 DIO0 (PB1) */
  casper_board_exti_enable_dio0();

  /* Enable EXTI9_5 for DIO1 (PD7) */
  casper_board_exti_enable_dio1();

  /* Reset pyro pins to input mode (safety — GS has no pyros) */
  casper_board_pyro_safe_mode();

#else /* FLIGHT build */

  {
    char _dbg[80]; int _len;
#if (USB_MODE == 2)
    #define DBG_PRINT(msg) do { (void)_dbg; (void)_len; } while(0)
#else
    #define DBG_PRINT(msg) do { \
      _len = snprintf(_dbg, sizeof(_dbg), msg); \
      CDC_Transmit_FS((uint8_t *)_dbg, (uint16_t)_len); \
      casper_delay_ms(20); \
    } while(0)
#endif

    DBG_PRINT("[INIT] USB up, starting sensor init\r\n");

    // M1: MS5611
    DBG_PRINT("[INIT] MS5611...\r\n");

  // Init MS5611 barometer on SPI4 via port seam
  if (!ms5611_init(&baro, &BSP_SPI_BARO, BSP_PIN_BARO_CS)) {
    // PROM read failed — blink LED3+LED4 as warning but continue
    for (int i = 0; i < 6; i++) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(200);
    }
  }
#ifdef GYRO_TEMP_CAL
  ms5611_set_oversampling(&baro, MS5611_OSR_4096);
#else
  ms5611_set_oversampling(&baro, MS5611_OSR_2048);
#endif

    DBG_PRINT("[INIT] MS5611 done\r\n");

  /* Explicitly close PC2 analog switch for SPI2 MISO (PC2_C) and fix
   * PC2 MODER back to AF5.  Board-specific HAL calls delegated to the
   * board layer helper so we stay HAL-free here. */
  casper_board_fix_pc2_miso();

    DBG_PRINT("[INIT] LSM6DSO32...\r\n");

  // Init LSM6DSO32 IMU on SPI2
  lsm6dso32_init(&imu, &BSP_SPI_IMU, BSP_PIN_IMU_CS);
  if (imu.device_id != LSM6DSO32_WHO_AM_I_VAL) {
    // WHO_AM_I mismatch — blink LED3+LED4 as warning but continue
    for (int i = 0; i < 6; i++) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(200);
    }
  }

    DBG_PRINT("[INIT] LSM6DSO32 done\r\n");

  // NOTE: EXTI15 for LSM6DSO32 INT2 deferred to after all init completes
  // (enabling it here caused immediate ISR hang — INT2 already asserted)

    DBG_PRINT("[INIT] EKF+attitude...\r\n");
  {
    casper_att_config_t att_cfg = {
      .Kp_grav           = 10.0f,
      .Kp_mag_pad        = 0.0f,
      .Kp_mag_flight     = 0.0f,
      .Ki                = 0.1f,
      .gyro_lpf_cutoff_hz = 50.0f,
      .mag_update_hz     = 10.0f,
    };
    casper_att_init(&att, &att_cfg);
    casper_ekf_init(&ekf);
  }

    DBG_PRINT("[INIT] ADXL372...\r\n");
  adxl372_init(&high_g, &BSP_SPI_HIGHG, BSP_PIN_HIGHG_CS);
  if (high_g.device_id != ADXL372_DEVID_VAL) {
    for (int i = 0; i < 6; i++) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(200);
    }
  }
  if (high_g.device_id == ADXL372_DEVID_VAL) {
    adxl372_wakeup_init(&high_g, ADXL_LAUNCH_G, 6);
  }

    DBG_PRINT("[INIT] GPS...\r\n");
#ifndef GYRO_TEMP_CAL
#ifdef GPS_TEST
  /* Minimal init — no UBX config, just reset + I2C check.
   * Module outputs default NMEA which we passthrough to CDC. */
  if (!max_m10m_init_minimal(&gps, &BSP_I2C_GPS, BSP_PIN_GPS_NRST)) {
    for (int i = 0; i < 6; i++) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(200);
    }
  }
  if (gps.alive) {
    bool cfg_ok = max_m10m_configure_gps_test(&gps);
    bool rf_ok  = max_m10m_poll_mon_rf(&gps);
    {
      char _msg[160]; int _ml;
      static const char *ant_str[] = {"INIT","DONTKNOW","OK","SHORT","OPEN"};
      static const char *pwr_str[] = {"OFF","ON","DONTKNOW"};
      if (rf_ok) {
        _ml = snprintf(_msg, sizeof(_msg),
          "[INIT] GPS cfg=%s ant=%s pwr=%s noise=%u agc=%u jam=%u\r\n",
          cfg_ok ? "OK" : "FAIL",
          gps.ant_status <= 4 ? ant_str[gps.ant_status] : "?",
          gps.ant_power <= 2 ? pwr_str[gps.ant_power] : "?",
          gps.rf_noise_per_ms, gps.rf_agc_cnt, gps.rf_jam_ind);
      } else {
        _ml = snprintf(_msg, sizeof(_msg),
          "[INIT] GPS cfg=%s (MON-RF timeout)\r\n",
          cfg_ok ? "OK" : "FAIL");
      }
      CDC_Transmit_FS((uint8_t *)_msg, (uint16_t)_ml);
      casper_delay_ms(20);
    }
  }
#else
  if (!max_m10m_init(&gps, &BSP_I2C_GPS, BSP_PIN_GPS_NRST)) {
    for (int i = 0; i < 6; i++) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(200);
    }
  }
#endif
#endif

  /* Reconfigure I2C_3_INT (PC8) from output to EXTI rising edge.
   * Board-layer helper wraps HAL_GPIO_Init so we stay HAL-free here. */
  casper_board_fix_i2c3_int();

    DBG_PRINT("[INIT] MMC5983MA...\r\n");
#ifdef GYRO_TEMP_CAL
  mmc5983ma_init_oneshot(&mag, &BSP_I2C_MAG);
#else
  mmc5983ma_init(&mag, &BSP_I2C_MAG);
#endif
  if (mag.product_id != MMC5983MA_PROD_ID_VAL) {
    for (int i = 0; i < 6; i++) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(200);
    }
  }

    DBG_PRINT("[INIT] telemetry+FSM+pyro+buzzer...\r\n");
  crc32_hw_init();
  tlm_init();
  cmd_router_init();
  cac_init();
  cfg_manager_init();
  flight_fsm_init();
  /* ADC calibration must run before pyro init (STM32H7 requirement). */
  casper_adc_init_all();
  pyro_mgr_init(BSP_ADC_CONT, BSP_PIN_PY, BSP_PIN_CONT_LED);
#ifndef GPS_TEST
  buzzer_init(&BSP_PWM_BUZZ);
#endif

    DBG_PRINT("[INIT] radio...\r\n");
  int radio_init_ok = 0;
  {
    /* radio_manager_init() accepts void* and casts it to SPI_HandleTypeDef*
     * internally (radio_manager.c includes HAL).  Obtain the raw pointer
     * via the board helper to avoid including main.h here. */
    int rc = radio_manager_init(casper_board_spi1_handle());
    radio_init_ok = (rc == 0);
#if (USB_MODE != 2)
    _len = snprintf(_dbg, sizeof(_dbg), "[INIT] radio returned %d\r\n", rc);
    CDC_Transmit_FS((uint8_t *)_dbg, (uint16_t)_len);
    casper_delay_ms(20);
#endif

    if (rc != 0) {
      for (int i = 0; i < 10; i++) {
        casper_gpio_toggle(BSP_PIN_CONT_YN_3);
        casper_gpio_toggle(BSP_PIN_CONT_YN_4);
        casper_delay_ms(100);
      }
      casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_LOW);
      casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_LOW);
    }
  }

  /* Flight data logger init (reads flight index, sets up ring buffers).
   * flash was initialised in main() before USB device enumeration. */
  flight_logger_init(&logger, &flash);

    DBG_PRINT("[INIT] all init complete\r\n");

#ifndef GPS_TEST
  /* Startup beep: 3 short = radio OK, 5 long = radio FAIL */
  if (radio_init_ok) {
    buzzer_beep_n(50, 3, 100, 150);
  } else {
    buzzer_beep_n(50, 5, 300, 100);
  }
#endif

  } /* end DBG_PRINT scope */

  /* EXTI15_10 (PC15 = LSM6DSO32 INT2):
   *   Originally disabled because SX1276 DIO4 (PB12) / DIO5 (PB13) were
   *   configured as IT_RISING by CubeMX and toggled constantly,
   *   producing stale-pending interrupts that flooded EXTI15_10.
   *   Fix applied in MX_GPIO_Init_2 USER block: PB12/PB13 are now plain
   *   inputs, so EXTI15_10 only ever fires for PC15.  Enable the NVIC
   *   line and clear any pending bit before enabling. */
  casper_board_exti_enable_imu_int2();

  // Check flash init result (already initialized before USB, in main())
  bool flash_ok = false;
  if (flash.jedec_id[0] != W25Q512JV_MANUFACTURER_ID) {
    for (int i = 0; i < 6; i++) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(200);
    }
  } else {
    flash_ok = true;
  }
  (void)flash_ok;  // suppress warning when FATFS block is compiled out

#if (USB_MODE != 2)
  bool fatfs_ok = false;
  bool file_test_ok = false;
#if (TEST_MODE != 1)
  // Mount FATFS on flash (CDC mode only — MSC lets the PC manage the filesystem)
  if (flash_ok) {
    FRESULT fres = f_mount(&USERFatFS, USERPath, 1);
    if (fres == FR_NO_FILESYSTEM) {
      // First boot — format the flash with FAT
      BYTE work[4096];
      fres = f_mkfs(USERPath, FM_FAT, 0, work, sizeof(work));
      if (fres == FR_OK)
        fres = f_mount(&USERFatFS, USERPath, 1);
    }
    fatfs_ok = (fres == FR_OK);
  }

  // File I/O test: write a string, read it back, compare
  if (fatfs_ok) {
    const char test_str[] = "CASPER-2 FATFS OK";
    char read_buf[32] = {0};
    UINT bw = 0, br = 0;
    FRESULT fr;

    // Write test file
    fr = f_open(&USERFile, "TEST.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
      fr = f_write(&USERFile, test_str, sizeof(test_str) - 1, &bw);
      f_close(&USERFile);
    }

    // Read it back
    if (fr == FR_OK && bw == sizeof(test_str) - 1) {
      fr = f_open(&USERFile, "TEST.TXT", FA_READ);
      if (fr == FR_OK) {
        fr = f_read(&USERFile, read_buf, sizeof(test_str) - 1, &br);
        f_close(&USERFile);
      }
    }

    // Verify
    if (fr == FR_OK && br == sizeof(test_str) - 1 &&
        memcmp(test_str, read_buf, br) == 0) {
      file_test_ok = true;
    }
  }
#endif /* TEST_MODE != 1 */
#endif /* USB_MODE != 2 */

  /* DEBUG: LED4 ON = FATFS done */
  casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_HIGH);

  // M5: init complete — all LEDs off, then blink all 3 times
  casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_LOW);
  for (int i = 0; i < 3; i++) {
    casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_HIGH);
    casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_HIGH);
    casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_HIGH);
    casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_HIGH);
    casper_delay_ms(150);
    casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_LOW);
    casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_LOW);
    casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_LOW);
    casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_LOW);
    casper_delay_ms(150);
  }
  casper_delay_ms(500);  // USB enumeration time

#if (USB_MODE != 2)
  // Print status on first few lines so terminal can capture it
  {
    char info_buf[160];
    int info_len = snprintf(info_buf, sizeof(info_buf),
        ">who:0x%02X,adxl:0x%02X,mag:0x%02X,flash_id:0x%02X%02X%02X,fatfs:%s,file_test:%s,gps:%s\r\n",
        imu.device_id, high_g.device_id, mag.product_id,
        flash.jedec_id[0], flash.jedec_id[1], flash.jedec_id[2],
        fatfs_ok ? "OK" : "ERR",
        file_test_ok ? "PASS" : "FAIL",
        gps.alive ? "OK" : "NO_ACK");
    for (int i = 0; i < 5; i++) {
      CDC_Transmit_FS((uint8_t *)info_buf, info_len);
      casper_delay_ms(200);
    }
  }
  // Print MS5611 raw PROM calibration coefficients
  {
    char prom_buf[100];
    int prom_len = snprintf(prom_buf, sizeof(prom_buf),
        ">PROM:C0=%u,C1=%u,C2=%u,C3=%u,C4=%u,C5=%u,C6=%u\r\n",
        baro.prom[0], baro.prom[1], baro.prom[2],
        baro.prom[3], baro.prom[4], baro.prom[5], baro.prom[6]);
    for (int i = 0; i < 3; i++) {
      CDC_Transmit_FS((uint8_t *)prom_buf, prom_len);
      casper_delay_ms(200);
    }
  }
#endif /* USB_MODE != 2 */

#if (USB_MODE == 2)
  /* MSC mode: LEDs now driven by pyro_tick for continuity — all off initially */
  casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_LOW);
#else
  /* Clear all LEDs before calibration sequence */
  casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_LOW);
  casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_LOW);
#endif

#ifdef MAG_CAL
  if (!mag_cal_init(&s_mcal)) {
    /* File open failed — rapid blink all LEDs */
    while (1) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_1);
      casper_gpio_toggle(BSP_PIN_CONT_YN_2);
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(100);
    }
  }
#endif
#ifdef MAG_VAL
  if (!mag_val_init(&s_mval)) {
    while (1) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_1);
      casper_gpio_toggle(BSP_PIN_CONT_YN_2);
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(100);
    }
  }
#elif defined(GYRO_TEMP_CAL)
  if (!temp_cal_init(&s_tcal)) {
    /* File open failed — rapid blink all LEDs */
    while (1) {
      casper_gpio_toggle(BSP_PIN_CONT_YN_1);
      casper_gpio_toggle(BSP_PIN_CONT_YN_2);
      casper_gpio_toggle(BSP_PIN_CONT_YN_3);
      casper_gpio_toggle(BSP_PIN_CONT_YN_4);
      casper_delay_ms(100);
    }
  }
#else
  flight_loop_init();
  flight_logger_start(&logger);  /* Begin PAD-state ring filling + erase-ahead */
#ifdef LOGGER_SANITY
  logger_sanity_init(&logger);
#endif
#endif

#endif /* BUILD_TARGET_GROUND */
}

/* =========================================================================
 *  app_tick()
 *  ── Moved verbatim from main.c USER CODE BEGIN 3 .. END 3 ─────────────
 *
 *  Called from while(1) in main() on every iteration.
 *  HAL_GetTick() → casper_millis().
 *  HAL_GPIO_*    → casper_gpio_*.
 *  HAL_Delay     → casper_delay_ms.
 * ======================================================================= */
void app_tick(void)
{
#if defined(BUILD_TARGET_GROUND)
    ground_main_tick();
#elif (USB_MODE == 2)
    buzzer_tick();
    // MSC mode: pyro continuity LEDs at 10 Hz (replaces old ping-pong)
    {
      static uint32_t last_pyro_tick = 0;
      if (casper_millis() - last_pyro_tick >= 100) {
        pyro_mgr_tick();
        last_pyro_tick = casper_millis();
      }
    }
#elif defined(MAG_CAL)
    {
      static uint32_t last_mag_tick = 0;
      uint32_t now = casper_millis();

      /* 100 Hz mag read + calibration tick */
      if (now - last_mag_tick >= 10) {
        mmc5983ma_read(&mag);
        mag_cal_tick(&s_mcal, &mag, now);
        last_mag_tick = now;
      }

      /* Done: all LEDs solid, print final message, idle forever */
      if (mag_cal_is_done(&s_mcal)) {
        casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_HIGH);
        char done_buf[64];
        int done_len = snprintf(done_buf, sizeof(done_buf),
            ">mag_cal: DONE (%lu samples)\r\n", s_mcal.sample_count);
        for (int i = 0; i < 5; i++) {
          CDC_Transmit_FS((uint8_t *)done_buf, done_len);
          casper_delay_ms(200);
        }
        while (1) { casper_delay_ms(1000); }
      }
    }
#elif defined(MAG_VAL)
    {
      static uint32_t last_mag_tick = 0;
      uint32_t now = casper_millis();

      /* 100 Hz mag read + validation tick */
      if (now - last_mag_tick >= 10) {
        mmc5983ma_read(&mag);
        mag_val_tick(&s_mval, &mag, now);
        last_mag_tick = now;
      }

      /* Done: all LEDs solid, print final message, idle forever */
      if (mag_val_is_done(&s_mval)) {
        casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_HIGH);
        char done_buf[64];
        int done_len = snprintf(done_buf, sizeof(done_buf),
            ">mag_val: DONE (%lu samples)\r\n", s_mval.sample_count);
        for (int i = 0; i < 5; i++) {
          CDC_Transmit_FS((uint8_t *)done_buf, done_len);
          casper_delay_ms(200);
        }
        while (1) { casper_delay_ms(1000); }
      }
    }
#elif defined(GYRO_TEMP_CAL)
    {
      static uint32_t last_tcal_tick = 0;
      uint32_t now = casper_millis();

      /* 10 Hz: sample all sensors + log to CSV */
      if (now - last_tcal_tick >= 100) {
        temp_cal_tick(&s_tcal, &imu, &baro, &mag, now);
        last_tcal_tick = now;
      }

      /* Stopped: all LEDs solid, idle forever */
      if (temp_cal_is_stopped(&s_tcal)) {
        casper_gpio_write(BSP_PIN_CONT_YN_1, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_2, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_3, CASPER_PIN_HIGH);
        casper_gpio_write(BSP_PIN_CONT_YN_4, CASPER_PIN_HIGH);
        while (1) { casper_delay_ms(1000); }
      }
    }
#elif defined(GPS_TEST)
    {
      static uint32_t last_status = 0;
      uint32_t now = casper_millis();

      /* NMEA passthrough: read I2C, forward complete lines to CDC */
      max_m10m_tick_nmea(&gps);

      if (gps.nmea_line_ready) {
        gps.nmea_line_ready = false;
        uint8_t len = (uint8_t)strlen(gps.nmea_line);
        /* Add \r\n for serial monitor display */
        if (len < sizeof(gps.nmea_line) - 2) {
          gps.nmea_line[len]   = '\r';
          gps.nmea_line[len+1] = '\n';
          CDC_Transmit_FS((uint8_t *)gps.nmea_line, len + 2);
          casper_delay_ms(5);
        }
      }

      /* Status line every 5s with RF diagnostics */
      if (now - last_status >= 5000) {
        last_status = now;
        char sbuf[180];
        int slen;

        static const char *ant_str[] = {"INIT","DONTKNOW","OK","SHORT","OPEN"};
        static const char *pwr_str[] = {"OFF","ON","DONTKNOW"};

        if (gps.mon_rf_valid) {
          slen = snprintf(sbuf, sizeof(sbuf),
            ">-- ant=%s pwr=%s agc=%u noise=%u jam=%u | polls=%lu avZ=%lu i2cE=%lu bytes=%lu --\r\n",
            gps.ant_status <= 4 ? ant_str[gps.ant_status] : "?",
            gps.ant_power <= 2 ? pwr_str[gps.ant_power] : "?",
            gps.rf_agc_cnt, gps.rf_noise_per_ms, gps.rf_jam_ind,
            (unsigned long)gps.dbg_polls,
            (unsigned long)gps.dbg_avail_zero,
            (unsigned long)gps.dbg_i2c_err,
            (unsigned long)gps.dbg_bytes_read);
        } else {
          slen = snprintf(sbuf, sizeof(sbuf),
            ">-- ant=? (no MON-RF) | polls=%lu avZ=%lu i2cE=%lu bytes=%lu --\r\n",
            (unsigned long)gps.dbg_polls,
            (unsigned long)gps.dbg_avail_zero,
            (unsigned long)gps.dbg_i2c_err,
            (unsigned long)gps.dbg_bytes_read);
        }
        CDC_Transmit_FS((uint8_t *)sbuf, (uint16_t)slen);
        casper_gpio_toggle(BSP_PIN_CONT_YN_1);
      }

      /* Re-poll MON-RF every 30s for fresh RF diagnostics */
      {
        static uint32_t last_rf_poll = 0;
        if (now - last_rf_poll >= 30000) {
          last_rf_poll = now;
          /* Non-blocking: send poll, response parsed in tick_nmea's UBX parser */
          max_m10m_poll_mon_rf(&gps);
        }
      }
    }
#else
    flight_loop_tick();
#endif
}
