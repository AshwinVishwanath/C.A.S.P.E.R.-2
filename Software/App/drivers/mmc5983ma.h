/**
 * @file    mmc5983ma.h
 * @brief   MMC5983MA 3-axis magnetometer driver (I2C) for C.A.S.P.E.R.-2
 *
 * MEMSIC MMC5983MA: ±8 Gauss, 18-bit, auto SET/RESET degauss.
 * Connected via I2C3 (PA8=SCL, PC9=SDA), DRDY interrupt on PC8.
 */
#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ── I2C address (7-bit) ─────────────────────────────────────────────────── */
#define MMC5983MA_I2C_ADDR        0x30
#define MMC5983MA_I2C_ADDR_WRITE  (MMC5983MA_I2C_ADDR << 1)       /* 0x60 */
#define MMC5983MA_I2C_ADDR_READ   ((MMC5983MA_I2C_ADDR << 1) | 1) /* 0x61 */

/* ── Register map ────────────────────────────────────────────────────────── */
#define MMC5983MA_REG_X_OUT_0     0x00
#define MMC5983MA_REG_X_OUT_1     0x01
#define MMC5983MA_REG_Y_OUT_0     0x02
#define MMC5983MA_REG_Y_OUT_1     0x03
#define MMC5983MA_REG_Z_OUT_0     0x04
#define MMC5983MA_REG_Z_OUT_1     0x05
#define MMC5983MA_REG_XYZ_OUT_2   0x06  /* bits [1:0] of each axis */
#define MMC5983MA_REG_T_OUT       0x07
#define MMC5983MA_REG_STATUS      0x08
#define MMC5983MA_REG_CTRL0       0x09
#define MMC5983MA_REG_CTRL1       0x0A
#define MMC5983MA_REG_CTRL2       0x0B
#define MMC5983MA_REG_CTRL3       0x0C
#define MMC5983MA_REG_PROD_ID     0x2F

/* ── Status register (0x08) ──────────────────────────────────────────────── */
#define MMC5983MA_STATUS_MEAS_M_DONE   (1 << 0)
#define MMC5983MA_STATUS_MEAS_T_DONE   (1 << 1)
#define MMC5983MA_STATUS_OTP_RD_DONE   (1 << 4)

/* ── Internal Control 0 (0x09) ───────────────────────────────────────────── */
#define MMC5983MA_CTRL0_TM_M              (1 << 0)  /* take magnetic measurement */
#define MMC5983MA_CTRL0_TM_T              (1 << 1)  /* take temperature measurement */
#define MMC5983MA_CTRL0_INT_MEAS_DONE_EN  (1 << 2)  /* DRDY interrupt enable */
#define MMC5983MA_CTRL0_SET               (1 << 3)  /* SET pulse */
#define MMC5983MA_CTRL0_RESET             (1 << 4)  /* RESET pulse */
#define MMC5983MA_CTRL0_AUTO_SR_EN        (1 << 5)  /* automatic SET/RESET */

/* ── Internal Control 1 (0x0A) ───────────────────────────────────────────── */
#define MMC5983MA_CTRL1_BW0               (1 << 0)  /* bandwidth bit 0 */
#define MMC5983MA_CTRL1_BW1               (1 << 1)  /* bandwidth bit 1 */
#define MMC5983MA_CTRL1_X_INHIBIT         (1 << 2)
#define MMC5983MA_CTRL1_Y_INHIBIT         (1 << 3)
#define MMC5983MA_CTRL1_Z_INHIBIT         (1 << 4)
#define MMC5983MA_CTRL1_SW_RST            (1 << 7)  /* software reset */

/* Bandwidth settings (BW1:BW0) → measurement time */
#define MMC5983MA_BW_100HZ  0x00  /* 8 ms  */
#define MMC5983MA_BW_200HZ  0x01  /* 4 ms  */
#define MMC5983MA_BW_400HZ  0x02  /* 2 ms  */
#define MMC5983MA_BW_800HZ  0x03  /* 0.5 ms */

/* ── Internal Control 2 (0x0B) ───────────────────────────────────────────── */
#define MMC5983MA_CTRL2_CM_FREQ_0         (1 << 0)
#define MMC5983MA_CTRL2_CM_FREQ_1         (1 << 1)
#define MMC5983MA_CTRL2_CM_FREQ_2         (1 << 2)
#define MMC5983MA_CTRL2_CMM_EN            (1 << 3)  /* continuous mode enable */
#define MMC5983MA_CTRL2_PRD_SET_0         (1 << 4)
#define MMC5983MA_CTRL2_PRD_SET_1         (1 << 5)
#define MMC5983MA_CTRL2_PRD_SET_2         (1 << 6)
#define MMC5983MA_CTRL2_EN_PRD_SET        (1 << 7)  /* periodic SET enable */

/* Continuous mode frequency (CM_FREQ[2:0]) */
#define MMC5983MA_CM_OFF      0x00  /* continuous mode off */
#define MMC5983MA_CM_1HZ      0x01
#define MMC5983MA_CM_10HZ     0x02
#define MMC5983MA_CM_20HZ     0x03
#define MMC5983MA_CM_50HZ     0x04
#define MMC5983MA_CM_100HZ    0x05
#define MMC5983MA_CM_200HZ    0x06
#define MMC5983MA_CM_1000HZ   0x07

/* ── Product ID ──────────────────────────────────────────────────────────── */
#define MMC5983MA_PROD_ID_VAL  0x30

/* ── Conversion constants ────────────────────────────────────────────────── */
#define MMC5983MA_18BIT_OFFSET  131072.0f   /* 2^17: midpoint = 0 Gauss */
#define MMC5983MA_18BIT_SCALE   16384.0f    /* counts per Gauss (2^18 / 16G) */

/* ── Return codes ────────────────────────────────────────────────────────── */
#define MMC5983MA_OK            0
#define MMC5983MA_ERR_I2C      -1
#define MMC5983MA_ERR_ID       -2

/* ── Device handle ───────────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t           addr;           /* 8-bit shifted address for HAL */
    float              mag_gauss[3];   /* X, Y, Z in Gauss */
    float              mag_ut[3];      /* X, Y, Z in microtesla */
    uint32_t           raw_mag[3];     /* 18-bit unsigned raw */
    int16_t            raw_temp;
    volatile bool      data_ready;     /* set by EXTI ISR */
    uint8_t            product_id;
} mmc5983ma_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the MMC5983MA: software reset, verify product ID,
 *         enable auto SET/RESET, set BW=800Hz, continuous mode at 100Hz,
 *         enable DRDY interrupt.
 * @return true if product ID matches 0x30
 */
bool mmc5983ma_init(mmc5983ma_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read 18-bit magnetic field data, convert to Gauss and uT.
 * @return MMC5983MA_OK on success, negative on I2C error.
 */
int mmc5983ma_read(mmc5983ma_t *dev);

/**
 * @brief  Read temperature register.
 *         Result: temperature_C = -75 + (raw_temp * 200.0 / 255.0)
 * @return MMC5983MA_OK on success, negative on I2C error.
 */
int mmc5983ma_read_temp(mmc5983ma_t *dev);

/**
 * @brief  Initialise MMC5983MA in single-shot mode (no continuous measurement).
 *         Same as mmc5983ma_init() but skips CTRL2 continuous mode setup.
 *         Use with mmc5983ma_trigger_oneshot() for on-demand measurements.
 * @return true if product ID matches 0x30
 */
bool mmc5983ma_init_oneshot(mmc5983ma_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Trigger a single magnetic measurement, poll for completion,
 *         then read the result into dev->raw_mag[] and dev->mag_ut[].
 *         Blocking: up to ~5 ms at BW=800Hz.
 * @return MMC5983MA_OK on success, negative on I2C error or timeout.
 */
int mmc5983ma_trigger_oneshot(mmc5983ma_t *dev);

/**
 * @brief  Call from EXTI callback when I2C_3_INT_Pin fires.
 */
void mmc5983ma_irq_handler(mmc5983ma_t *dev);

#endif /* MMC5983MA_H */
