/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#if (USB_MODE != 2)
#include "usbd_cdc_if.h"
#endif
#include "ms5611.h"
#include "lsm6dso32.h"
#include "adxl372.h"
#include "w25q512jv.h"
#include "casper_ekf.h"
#include "casper_attitude.h"
#include "casper_quat.h"
#include "max_m10m.h"
#include "mmc5983ma.h"
#include "mag_cal.h"
#include "casper_pyro.h"
#ifdef MAG_VAL
#include "mag_val.h"
#endif
/* ── MC Testing / Telemetry modules ── */
#include "crc32_hw.h"
#include "tlm_manager.h"
#include "cmd_router.h"
#include "cac_handler.h"
#include "cfg_manager.h"
#include "flight_fsm.h"
#include "pyro_manager.h"
#include "self_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ── Serial output toggles (uncomment to enable) ───────────────── */
#ifdef CDC_OUTPUT
  #define OUT_ATTITUDE     /* roll, pitch, yaw, heading_sigma       */
  //#define OUT_EKF          /* altitude, velocity, biases             */
  //#define OUT_GPS          /* lat, lon, alt, vel_d, fix, sats        */
  // #define OUT_MAG       /* calibrated magnetometer field (uT)     */
  // #define OUT_IMU_RAW   /* raw accel (g) + gyro (dps)             */
  // #define OUT_BARO      /* barometric altitude AGL (m)            */
  #define OUT_IMU_TEMP  /* IMU die temperature (°C)               */
#endif

/* ── Gyro temperature compensation (slope-only linear model) ── */
#ifdef GYRO_TEMP_COMP
#define GYRO_TC_T0       32.485f       /* Reference temperature (°C)    */
#define GYRO_TC_SLOPE_X  9.3745e-04f   /* rad/s per °C, X axis          */
#define GYRO_TC_SLOPE_Y  2.3098e-04f   /* rad/s per °C, Y axis          */
#define GYRO_TC_SLOPE_Z  2.8388e-04f   /* rad/s per °C, Z axis          */
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
ms5611_t baro;
lsm6dso32_t imu;
adxl372_t high_g;
w25q512jv_t flash;
casper_ekf_t ekf;
casper_attitude_t att;
max_m10m_t gps;
mmc5983ma_t mag;
static float mag_cal_ut[3];
static bool mag_new = false;
casper_pyro_t pyro;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Enable GPIO clocks for all 4 LEDs
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOEEN;
  (void)RCC->AHB4ENR;

  // Configure all 4 LED pins as GP output (MODER = 01)
  GPIOA->MODER = (GPIOA->MODER & ~(3UL << (10*2))) | (1UL << (10*2)); // PA10
  GPIOB->MODER = (GPIOB->MODER & ~(3UL << (14*2))) | (1UL << (14*2)); // PB14
  GPIOE->MODER = (GPIOE->MODER & ~(3UL << (8*2)))  | (1UL << (8*2));  // PE8
  GPIOE->MODER = (GPIOE->MODER & ~(3UL << (7*2)))  | (1UL << (7*2));  // PE7

  // Milestone 1: HAL_Init passed
  GPIOA->BSRR = (1UL << 10);  // LED1 ON
  HAL_Delay(500);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();

  // Init flash BEFORE USB so MSC storage callbacks can respond during enumeration
  w25q512jv_init(&flash, &hqspi);

  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  // Milestone 2: GPIO configured — turn on CONT_YN_2 (PB14)
  HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
  HAL_Delay(500);

  // Milestone 3: USB init done — turn on CONT_YN_3 (PE8)
  HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
  HAL_Delay(500);

  // Init MS5611 barometer on SPI4
  if (!ms5611_init(&baro, &hspi4, SPI4_CS_GPIO_Port, SPI4_CS_Pin)) {
    // PROM read failed — blink LED3+LED4 as warning but continue
    for (int i = 0; i < 6; i++) {
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(200);
    }
  }
  ms5611_set_oversampling(&baro, MS5611_OSR_1024);

  // Explicitly close PC2 analog switch for SPI2 MISO (PC2_C)
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  // Fix PC2 MODER: something during init sets PC2 to analog mode instead of AF.
  // AFR is correctly set to AF5 by SPI2 MspInit, but MODER gets clobbered.
  // Force PC2 back to AF mode for SPI2 MISO.
  {
    GPIO_InitTypeDef gpio_fix = {0};
    gpio_fix.Pin = GPIO_PIN_2;
    gpio_fix.Mode = GPIO_MODE_AF_PP;
    gpio_fix.Pull = GPIO_NOPULL;
    gpio_fix.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_fix.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &gpio_fix);
  }

  // Init LSM6DSO32 IMU on SPI2
  lsm6dso32_init(&imu, &hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin);
  if (imu.device_id != LSM6DSO32_WHO_AM_I_VAL) {
    // WHO_AM_I mismatch — blink LED3+LED4 as warning but continue
    for (int i = 0; i < 6; i++) {
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(200);
    }
  }

  // Enable EXTI15 interrupt for LSM6DSO32 INT2 (PC15)
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  // Init attitude estimator and EKF
  {
    casper_att_config_t att_cfg = {
      .Kp_grav           = 1.0f,
      .Kp_mag_pad        = 0.5f,
      .Kp_mag_flight     = 0.3f,
      .Ki                = 0.05f,
      .gyro_lpf_cutoff_hz = 50.0f,
      .mag_update_hz     = 10.0f,
      .launch_accel_g    = 3.0f,
    };
    casper_att_init(&att, &att_cfg);
    casper_ekf_init(&ekf);
  }

  // Init ADXL372 high-G accelerometer on SPI3
  adxl372_init(&high_g, &hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin);
  if (high_g.device_id != ADXL372_DEVID_VAL) {
    // DEVID mismatch — blink LED3+LED4 as warning but continue
    for (int i = 0; i < 6; i++) {
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(200);
    }
  }

  // Init MAX-M10M GPS on I2C1
  if (!max_m10m_init(&gps, &hi2c1, NRST_GPS_GPIO_Port, NRST_GPS_Pin)) {
    // I2C NAK — GPS module not responding — blink LED3+LED4 warning, continue
    for (int i = 0; i < 6; i++) {
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(200);
    }
  }

  // Reconfigure I2C_3_INT (PC8) from output to EXTI rising edge
  {
    GPIO_InitTypeDef gpio_fix = {0};
    gpio_fix.Pin = I2C_3_INT_Pin;
    gpio_fix.Mode = GPIO_MODE_IT_RISING;
    gpio_fix.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I2C_3_INT_GPIO_Port, &gpio_fix);
  }

  // Init MMC5983MA magnetometer on I2C3
  mmc5983ma_init(&mag, &hi2c3);
  if (mag.product_id != MMC5983MA_PROD_ID_VAL) {
    // Product ID mismatch — blink LED3+LED4 as warning but continue
    for (int i = 0; i < 6; i++) {
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(200);
    }
  }

  // EXTI9_5 for MMC5983MA DRDY (PC8) — disabled, using polled reads instead
  // HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  // HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  // Init pyro channels: ADC calibration, force all MOSFETs off
  casper_pyro_init(&pyro, &hadc1, &hadc2, &hadc3);

  // Init MC telemetry + command subsystems
  crc32_hw_init();
  tlm_init();
  cmd_router_init();
  cac_init();
  cfg_manager_init();
  flight_fsm_init();
  pyro_mgr_init();

  // Check flash init result (already initialized before USB)
  bool flash_ok = false;
  if (flash.jedec_id[0] != W25Q512JV_MANUFACTURER_ID) {
    for (int i = 0; i < 6; i++) {
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(200);
    }
  } else {
    flash_ok = true;
  }
  (void)flash_ok;  // suppress warning when FATFS block is compiled out

#if (USB_MODE != 2)
  // Mount FATFS on flash (CDC mode only — MSC lets the PC manage the filesystem)
  bool fatfs_ok = false;
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
  bool file_test_ok = false;
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
#endif /* USB_MODE != 2 */

  // Milestone 4: all done — turn on CONT_YN_4 (PE7)
  HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);  // USB enumeration time

  // Turn all OFF before entering main loop
  HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);

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
      HAL_Delay(200);
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
      HAL_Delay(200);
    }
  }
#endif /* USB_MODE != 2 */

#if (USB_MODE == 2)
  /* MSC mode: LEDs now driven by pyro_tick for continuity — all off initially */
  HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
#else
  /* Clear all LEDs before calibration sequence */
  HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
#endif

  uint32_t last_output_tick = 0;
  uint32_t last_mag_tick = 0;
  const float EKF_DT = 0.0012f;              /* 833 Hz (IMU data-ready driven) */
  const float G_CONST = 9.80665f;
  const float DEG2RAD = 0.0174532925f;        /* pi / 180 */
  float baro_ref = 0.0f;
  bool init_done = false;       /* static init (accel+mag averaging) complete */
  bool cal_done = false;        /* 30s init period complete, filter running */
  uint8_t cal_pct_printed = 0;  /* last 10% milestone printed (0-10) */
  uint32_t loop_start_tick = 0; /* HAL_GetTick() at main loop entry */

#ifdef MAG_CAL
  mag_cal_t mcal;
  if (!mag_cal_init(&mcal)) {
    /* File open failed — rapid blink all LEDs */
    while (1) {
      HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
      HAL_GPIO_TogglePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin);
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(100);
    }
  }
#endif
#ifdef MAG_VAL
  mag_val_t mval;
  if (!mag_val_init(&mval)) {
    while (1) {
      HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
      HAL_GPIO_TogglePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin);
      HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
      HAL_GPIO_TogglePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin);
      HAL_Delay(100);
    }
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  loop_start_tick = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (USB_MODE == 2)
    // MSC mode: pyro continuity LEDs at 10 Hz (replaces old ping-pong)
    {
      static uint32_t last_pyro_tick = 0;
      if (HAL_GetTick() - last_pyro_tick >= 100) {
        casper_pyro_tick(&pyro);
        last_pyro_tick = HAL_GetTick();
      }
    }
#elif defined(MAG_CAL)
    {
      uint32_t now = HAL_GetTick();

      /* 100 Hz mag read + calibration tick */
      if (now - last_mag_tick >= 10) {
        mmc5983ma_read(&mag);
        mag_cal_tick(&mcal, &mag, now);
        last_mag_tick = now;
      }

      /* Done: all LEDs solid, print final message, idle forever */
      if (mag_cal_is_done(&mcal)) {
        HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
        char done_buf[64];
        int done_len = snprintf(done_buf, sizeof(done_buf),
            ">mag_cal: DONE (%lu samples)\r\n", mcal.sample_count);
        for (int i = 0; i < 5; i++) {
          CDC_Transmit_FS((uint8_t *)done_buf, done_len);
          HAL_Delay(200);
        }
        while (1) { HAL_Delay(1000); }
      }
    }
#elif defined(MAG_VAL)
    {
      uint32_t now = HAL_GetTick();

      /* 100 Hz mag read + validation tick */
      if (now - last_mag_tick >= 10) {
        mmc5983ma_read(&mag);
        mag_val_tick(&mval, &mag, now);
        last_mag_tick = now;
      }

      /* Done: all LEDs solid, print final message, idle forever */
      if (mag_val_is_done(&mval)) {
        HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
        char done_buf[64];
        int done_len = snprintf(done_buf, sizeof(done_buf),
            ">mag_val: DONE (%lu samples)\r\n", mval.sample_count);
        for (int i = 0; i < 5; i++) {
          CDC_Transmit_FS((uint8_t *)done_buf, done_len);
          HAL_Delay(200);
        }
        while (1) { HAL_Delay(1000); }
      }
    }
#else
    {
      uint32_t now = HAL_GetTick();

      /* ── Mag: polled read at 100 Hz, frame-map + calibrate ── */
      if (now - last_mag_tick >= 10) {
        mmc5983ma_read(&mag);
        last_mag_tick = now;
        /* Frame mapping (sensor → common frame) + hard/soft-iron cal */
        float mag_raw[3] = {-mag.mag_ut[0], -mag.mag_ut[1], -mag.mag_ut[2]};
        mag_cal_apply(mag_raw, mag_cal_ut);
        mag_new = true;
      }

      /* ── 833 Hz: IMU read + attitude estimator (interrupt-driven) ── */
      if (imu.data_ready) {
        lsm6dso32_read(&imu);

        /* ── Temp comp in sensor frame (slopes characterized per-sensor-axis) ── */
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

        /* ── Remap sensor → vehicle body frame (Xv=Zs, Yv=Xs, Zv=Ys) ── */
        float accel_ms2[3] = {
          imu.accel_g[2] * G_CONST,    /* Vehicle X = Sensor Z */
          imu.accel_g[0] * G_CONST,    /* Vehicle Y = Sensor X */
          imu.accel_g[1] * G_CONST     /* Vehicle Z = Sensor Y */
        };
        float gyro_rads[3] = {
          gyro_sensor[2],               /* Vehicle X = Sensor Z */
          gyro_sensor[0],               /* Vehicle Y = Sensor X */
          gyro_sensor[1]                /* Vehicle Z = Sensor Y */
        };

        const float *mag_ptr = mag_new ? mag_cal_ut : NULL;

        if (!cal_done) {
          /* ── Phase 1: 30s init (static init + complementary filter + gyro bias) ── */
          if (!init_done) {
            /* Sub-phase A: accumulate 500 mag samples for initial quaternion */
            if (casper_att_static_init(&att, accel_ms2, mag_ptr)) {
              init_done = true;
              /* Turn off init progress LEDs */
              HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
            } else {
              /* LED loading bar based on mag sample count */
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
            /* Sub-phase B: full complementary filter (accel+gyro+mag) + gyro bias */
            casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, EKF_DT);
          }

          /* Check 30s timer for transition */
          if (init_done && (now - loop_start_tick) >= 30000) {
            att.launched = true;
            att.e_int[0] = att.e_int[1] = att.e_int[2] = 0.0f;
            cal_done = true;
            HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
          }
        } else {
          /* ── Phase 2: Running — gyro+mag filter (no gravity correction) ── */
          casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, EKF_DT);
        }

        mag_new = false;
      }

      /* ── Async baro: always tick (warms up during init), feed EKF after launch ── */
      if (ms5611_tick(&baro) && cal_done) {
        float altitude = ms5611_get_altitude(&baro, 1013.25f) - baro_ref;
        casper_ekf_update_baro(&ekf, altitude);
      }

      /* ── GPS: non-blocking poll (10 Hz NAV-PVT) ── */
      static bool gps_new_fix = false;
      gps_new_fix = false;
      if (max_m10m_tick(&gps)) {
        gps_new_fix = true;
        if (cal_done && max_m10m_has_3d_fix(&gps)) {
          casper_ekf_update_gps_alt(&ekf, gps.alt_msl_m);
          casper_ekf_update_gps_vel(&ekf, gps.vel_d_m_s);
        }
      }

#ifdef CDC_OUTPUT
      /* ── 50 Hz: USB serial output ── */
      if (now - last_output_tick >= 20) {
        if (!cal_done) {
          if (!init_done) {
            /* Print static init progress */
            uint8_t pct = (uint8_t)(att.mag_count * 100 / 500);
            if (pct > cal_pct_printed * 10) {
              cal_pct_printed = pct / 10;
              char cal_buf[40];
              int cal_len = snprintf(cal_buf, sizeof(cal_buf),
                  ">init %d%%\r\n", pct);
              CDC_Transmit_FS((uint8_t *)cal_buf, cal_len);
            }
          } else {
            /* Pad: complementary filter converging */
            char buf[128];
            float euler[3];
            casper_quat_to_euler(att.q, euler);
            uint32_t remaining = 0;
            if ((now - loop_start_tick) < 30000)
              remaining = (30000 - (now - loop_start_tick)) / 1000;
            int len = snprintf(buf, sizeof(buf),
                ">pad roll:%.1f,pitch:%.1f,yaw:%.1f,t:%lu\r\n",
                euler[0], euler[1], euler[2], remaining);
            CDC_Transmit_FS((uint8_t *)buf, len);
          }
        } else {
          /* Running: toggle-gated telemetry output */
          char buf[256];
          int pos = 0;
          bool sep = false;     /* comma separator between fields */
          pos += snprintf(buf + pos, sizeof(buf) - pos, ">");

#ifdef OUT_ATTITUDE
          {
            float euler[3];
            casper_quat_to_euler(att.q, euler);
            if (sep) buf[pos++] = ',';
            sep = true;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "roll:%.1f,pitch:%.1f,yaw:%.1f,hdg_sig:%.4f",
                euler[0], euler[1], euler[2], att.heading_sigma);
          }
#endif
#ifdef OUT_EKF
          if (sep) buf[pos++] = ',';
          sep = true;
          pos += snprintf(buf + pos, sizeof(buf) - pos,
              "ekf:%.2f,%.2f,%.3f,%.3f",
              ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3]);
#endif
#ifdef OUT_GPS
          if (sep) buf[pos++] = ',';
          sep = true;
          pos += snprintf(buf + pos, sizeof(buf) - pos,
              "gps:%.6f,%.6f,%.1f,%.2f,%u,%u",
              gps.lat_deg, gps.lon_deg, gps.alt_msl_m,
              gps.vel_d_m_s, gps.fix_type, gps.num_sv);
#endif
#ifdef OUT_MAG
          if (sep) buf[pos++] = ',';
          sep = true;
          pos += snprintf(buf + pos, sizeof(buf) - pos,
              "mag:%.2f,%.2f,%.2f",
              mag_cal_ut[0], mag_cal_ut[1], mag_cal_ut[2]);
#endif
#ifdef OUT_IMU_RAW
          if (sep) buf[pos++] = ',';
          sep = true;
          pos += snprintf(buf + pos, sizeof(buf) - pos,
              "imu:%.3f,%.3f,%.3f,%.1f,%.1f,%.1f",
              imu.accel_g[0], imu.accel_g[1], imu.accel_g[2],
              imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2]);
#endif
#ifdef OUT_BARO
          if (sep) buf[pos++] = ',';
          sep = true;
          pos += snprintf(buf + pos, sizeof(buf) - pos,
              "bar:%.2f",
              ms5611_get_altitude(&baro, 1013.25f) - baro_ref);
#endif
#ifdef OUT_IMU_TEMP
          if (sep) buf[pos++] = ',';
          sep = true;
          pos += snprintf(buf + pos, sizeof(buf) - pos,
              "imu_temp:%.2f", imu.temp_c);
#endif

          pos += snprintf(buf + pos, sizeof(buf) - pos, "\r\n");
          CDC_Transmit_FS((uint8_t *)buf, pos);
          HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
        }
        last_output_tick = now;
      }
#endif /* CDC_OUTPUT */

      /* ── Keep all pyro gate pins high ── */
      HAL_GPIO_WritePin(PY1_GPIO_Port, PY1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PY2_GPIO_Port, PY2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PY3_GPIO_Port, PY3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PY4_GPIO_Port, PY4_Pin, GPIO_PIN_SET);

      /* ── MC Telemetry: populate state + 10 Hz binary COBS packets ── */
      {
        /* Build telemetry state from current sensor values */
        fc_telem_state_t tstate;
        tstate.alt_m        = ekf.x[0];
        tstate.vel_mps      = ekf.x[1];
        tstate.quat[0]      = att.q[0];
        tstate.quat[1]      = att.q[1];
        tstate.quat[2]      = att.q[2];
        tstate.quat[3]      = att.q[3];
        tstate.batt_v       = 7.4f;  /* TODO: ADC battery sensing */
        tstate.flight_time_s = flight_fsm_get_time_s();

        /* Build pyro state */
        pyro_state_t pstate;
        pstate.armed[0]     = (pyro_mgr_get_arm_bitmap() & 0x01) != 0;
        pstate.armed[1]     = (pyro_mgr_get_arm_bitmap() & 0x02) != 0;
        pstate.armed[2]     = (pyro_mgr_get_arm_bitmap() & 0x04) != 0;
        pstate.armed[3]     = (pyro_mgr_get_arm_bitmap() & 0x08) != 0;
        pstate.continuity[0] = pyro.continuity[0];
        pstate.continuity[1] = pyro.continuity[1];
        pstate.continuity[2] = pyro.continuity[2];
        pstate.continuity[3] = pyro.continuity[3];
        pstate.fired        = pyro_mgr_is_firing();

        /* If sim flight active, override alt/vel/quat with sim values */
        if (flight_fsm_sim_active()) {
          flight_fsm_sim_get_state(&tstate);
          tstate.batt_v = 7.4f;
        }

        fsm_state_t fsm = flight_fsm_tick(&tstate);
        tlm_tick(&tstate, &pstate, fsm);
      }

      /* ── GPS telemetry on new fix ── */
      if (gps_new_fix) {
        fc_gps_state_t gstate;
        /* TODO: compute delta lat/lon from pad position */
        gstate.dlat_m    = 0;
        gstate.dlon_m    = 0;
        gstate.alt_msl_m = gps.alt_msl_m;
        gstate.fix_type  = gps.fix_type;
        gstate.sat_count = gps.num_sv;
        gstate.pdop      = (float)gps.pDOP / 100.0f;
        gstate.new_fix   = true;
        tlm_send_gps(&gstate);
      }

      /* ── Process incoming CDC commands ── */
      if (cdc_ring_available() > 0) {
        cmd_router_process();
      }

      /* ── Pyro manager tick (wraps casper_pyro_tick) ── */
      pyro_mgr_tick();

      /* ── CAC confirm timeout check ── */
      cac_tick();
    }
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  // WORKAROUND: HSE crystal dead — use HSI (64MHz) as PLL source
  // HSI/4 = 16MHz input, x54 = 864MHz VCO, /2 = 432MHz SYSCLK, /18 = 48MHz USB
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 54;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 18;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_UART4;
  // WORKAROUND: PLL source is HSI (64MHz) — recalculate PLL2 for same 168MHz VCO
  // HSI/8 = 8MHz input, x21 = 168MHz VCO, /2 = 84MHz output (same as HSE config)
  PeriphClkInitStruct.PLL2.PLL2M = 8;
  PeriphClkInitStruct.PLL2.PLL2N = 21;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10707DBC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10707DBC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  // CubeMX defaults are wrong for W25Q512JV 64MB flash.
  // ClockPrescaler=255 (0.8MHz) → 7 (27MHz), FlashSize=1 → 25 (64MB),
  // SampleShifting → HALFCYCLE, ChipSelectHighTime → 2 cycles.
  hqspi.Init.ClockPrescaler = 7;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 25;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
  HAL_QSPI_Init(&hqspi);
  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  // LSM6DSO32 requires SPI Mode 3 (CPOL=1, CPHA=1).
  // MasterKeepIOState=ENABLE keeps SCK HIGH (CPOL=1 idle) between transactions,
  // preventing a spurious rising edge when HAL_SPI_TransmitReceive enables the peripheral.
  // CubeMX also reverts BaudRatePrescaler to 2 (42MHz) — LSM6DSO32 max is 10MHz.
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;   // CPOL=1
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;        // CPHA=1
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // PLL2P=84MHz /16 = 5.25MHz
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(&hspi2);
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  // CubeMX defaults DataSize to 4-bit and BaudRatePrescaler to 2 (42MHz).
  // ADXL372 needs 8-bit data, max 10MHz SPI clock.
  // PLL2P=84MHz / 16 = 5.25MHz — safe for ADXL372.
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  HAL_SPI_Init(&hspi3);
  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */
  // CubeMX reverts DataSize to 4-bit and BaudRatePrescaler to 2 (42MHz).
  // MS5611 needs 8-bit data, max 20MHz SPI clock.
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  // PLL2P=84MHz /8 = 10.5MHz
  HAL_SPI_Init(&hspi4);
  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RADIO_NRST_Pin|SPI2_CS_Pin|I2C_3_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Radio_CS_Pin|CONT_YN_2_Pin|PY4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CONT_YN_4_Pin|CONT_YN_3_Pin|SPI4_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRST_GPS_GPIO_Port, NRST_GPS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PY3_Pin|PY2_Pin|PY1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CONT_YN_1_Pin|SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RADIO_NRST_Pin SPI2_CS_Pin I2C_3_INT_Pin */
  GPIO_InitStruct.Pin = RADIO_NRST_Pin|SPI2_CS_Pin|I2C_3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_INT_Pin */
  GPIO_InitStruct.Pin = SPI2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO3_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Radio_CS_Pin CONT_YN_2_Pin PY4_Pin */
  GPIO_InitStruct.Pin = Radio_CS_Pin|CONT_YN_2_Pin|PY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_INT_Pin RADIO_DIO4_Pin RADIO_DIO5_Pin */
  GPIO_InitStruct.Pin = SPI1_INT_Pin|RADIO_DIO4_Pin|RADIO_DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CONT_YN_4_Pin CONT_YN_3_Pin SPI4_CS_Pin */
  GPIO_InitStruct.Pin = CONT_YN_4_Pin|CONT_YN_3_Pin|SPI4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : NRST_GPS_Pin */
  GPIO_InitStruct.Pin = NRST_GPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRST_GPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PY3_Pin PY2_Pin PY1_Pin */
  GPIO_InitStruct.Pin = PY3_Pin|PY2_Pin|PY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CONT_YN_1_Pin SPI3_CS_Pin */
  GPIO_InitStruct.Pin = CONT_YN_1_Pin|SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_INT_Pin RADIO_DIO2_Pin RADIO_DIO1_Pin */
  GPIO_InitStruct.Pin = SPI3_INT_Pin|RADIO_DIO2_Pin|RADIO_DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_TIMEPULSE_Pin */
  GPIO_InitStruct.Pin = GPS_TIMEPULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_TIMEPULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C1_INT_Pin */
  GPIO_InitStruct.Pin = I2C1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I2C1_INT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // CubeMX groups SPI CS pins with other outputs and sets them LOW.
  // All SPI CS pins must idle HIGH so sensors are deselected.
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SPI2_INT_Pin) {  /* PC15 = LSM6DSO32 INT2 */
    lsm6dso32_irq_handler(&imu);
  }
  if (GPIO_Pin == I2C1_INT_Pin) {  /* PE0 = GPS data ready (NVIC not enabled yet) */
    max_m10m_irq_handler(&gps);
  }
  if (GPIO_Pin == I2C_3_INT_Pin) {  /* PC8 = MMC5983MA DRDY */
    mmc5983ma_irq_handler(&mag);
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  // DIAGNOSTIC: Rapid blink all LEDs to indicate error
  // Enable GPIO clocks (might already be enabled, but safe to call again)
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure all 4 LED pins as GP output (MODER = 01)
  GPIOA->MODER = (GPIOA->MODER & ~(3UL << (10*2))) | (1UL << (10*2)); // PA10
  GPIOB->MODER = (GPIOB->MODER & ~(3UL << (14*2))) | (1UL << (14*2)); // PB14
  GPIOE->MODER = (GPIOE->MODER & ~(3UL << (8*2)))  | (1UL << (8*2));  // PE8
  GPIOE->MODER = (GPIOE->MODER & ~(3UL << (7*2)))  | (1UL << (7*2));  // PE7

  while (1)
  {
      // Very fast blink all LEDs (100ms) = ERROR indication
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);  // YN4
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // YN3
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // YN2
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // YN1
      for(volatile uint32_t i = 0; i < 500000; i++);  // Software delay

      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
      for(volatile uint32_t i = 0; i < 500000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
