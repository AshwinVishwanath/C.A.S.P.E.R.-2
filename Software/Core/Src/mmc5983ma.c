/**
 * @file    mmc5983ma.c
 * @brief   MMC5983MA 3-axis magnetometer driver (I2C) for C.A.S.P.E.R.-2
 */
#include "mmc5983ma.h"

#define I2C_TIMEOUT  50  /* ms */

/* ── I2C helpers ─────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef mmc5983ma_write_reg(mmc5983ma_t *dev,
                                              uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef mmc5983ma_read_reg(mmc5983ma_t *dev,
                                             uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef mmc5983ma_read_burst(mmc5983ma_t *dev,
                                               uint8_t reg,
                                               uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT);
}

/* ── Initialisation ──────────────────────────────────────────────────────── */

bool mmc5983ma_init(mmc5983ma_t *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;
    dev->addr = MMC5983MA_I2C_ADDR_WRITE;  /* HAL expects left-shifted addr */
    dev->data_ready = false;
    dev->product_id = 0x00;

    for (int i = 0; i < 3; i++) {
        dev->mag_gauss[i] = 0.0f;
        dev->mag_ut[i]    = 0.0f;
        dev->raw_mag[i]   = 0;
    }
    dev->raw_temp = 0;

    /* Software reset */
    mmc5983ma_write_reg(dev, MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
    HAL_Delay(15);  /* datasheet: 10ms typical after SW_RST */

    /* Read and verify product ID */
    uint8_t id = 0;
    if (mmc5983ma_read_reg(dev, MMC5983MA_REG_PROD_ID, &id) != HAL_OK)
        return false;
    dev->product_id = id;

    if (id != MMC5983MA_PROD_ID_VAL)
        return false;

    /* Enable auto SET/RESET (no DRDY interrupt — using polled reads) */
    mmc5983ma_write_reg(dev, MMC5983MA_REG_CTRL0,
                        MMC5983MA_CTRL0_AUTO_SR_EN);

    /* Bandwidth = 800 Hz (0.5 ms measurement time) */
    mmc5983ma_write_reg(dev, MMC5983MA_REG_CTRL1, MMC5983MA_BW_800HZ);

    /* Continuous mode at 100 Hz */
    mmc5983ma_write_reg(dev, MMC5983MA_REG_CTRL2,
                        MMC5983MA_CTRL2_CMM_EN | MMC5983MA_CM_100HZ);

    return true;
}

/* ── Magnetic field read ─────────────────────────────────────────────────── */

int mmc5983ma_read(mmc5983ma_t *dev)
{
    uint8_t buf[7];

    if (mmc5983ma_read_burst(dev, MMC5983MA_REG_X_OUT_0, buf, 7) != HAL_OK)
        return MMC5983MA_ERR_I2C;

    /* Assemble 18-bit unsigned values from three registers per axis */
    dev->raw_mag[0] = ((uint32_t)buf[0] << 10) |
                      ((uint32_t)buf[1] << 2)  |
                      ((buf[6] >> 6) & 0x03);

    dev->raw_mag[1] = ((uint32_t)buf[2] << 10) |
                      ((uint32_t)buf[3] << 2)  |
                      ((buf[6] >> 4) & 0x03);

    dev->raw_mag[2] = ((uint32_t)buf[4] << 10) |
                      ((uint32_t)buf[5] << 2)  |
                      ((buf[6] >> 2) & 0x03);

    /* Convert offset-binary 18-bit to Gauss and microtesla */
    for (int i = 0; i < 3; i++) {
        dev->mag_gauss[i] = ((float)dev->raw_mag[i] - MMC5983MA_18BIT_OFFSET)
                            / MMC5983MA_18BIT_SCALE;
        dev->mag_ut[i] = dev->mag_gauss[i] * 100.0f;  /* 1 Gauss = 100 uT */
    }

    dev->data_ready = false;
    return MMC5983MA_OK;
}

/* ── Temperature read ────────────────────────────────────────────────────── */

int mmc5983ma_read_temp(mmc5983ma_t *dev)
{
    /* Request temperature measurement */
    mmc5983ma_write_reg(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_T);

    /* Wait for measurement complete (up to 5 ms) */
    uint8_t status = 0;
    uint32_t start = HAL_GetTick();
    while (!(status & MMC5983MA_STATUS_MEAS_T_DONE)) {
        if (HAL_GetTick() - start > 10)
            return MMC5983MA_ERR_I2C;
        mmc5983ma_read_reg(dev, MMC5983MA_REG_STATUS, &status);
    }

    uint8_t raw = 0;
    if (mmc5983ma_read_reg(dev, MMC5983MA_REG_T_OUT, &raw) != HAL_OK)
        return MMC5983MA_ERR_I2C;

    dev->raw_temp = (int16_t)raw;
    /* Temperature = -75 + raw * (200.0 / 255.0)  degrees C */

    /* Re-enable auto SET/RESET + DRDY (CTRL0 is self-clearing for TM_T) */
    mmc5983ma_write_reg(dev, MMC5983MA_REG_CTRL0,
                        MMC5983MA_CTRL0_AUTO_SR_EN |
                        MMC5983MA_CTRL0_INT_MEAS_DONE_EN);

    return MMC5983MA_OK;
}

/* ── EXTI interrupt handler ──────────────────────────────────────────────── */

void mmc5983ma_irq_handler(mmc5983ma_t *dev)
{
    dev->data_ready = true;
}
