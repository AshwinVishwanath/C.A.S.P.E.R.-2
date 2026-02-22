/**
 * ADXL372 SPI high-G accelerometer driver — bare-metal C for STM32 HAL.
 * Reference: Analog Devices ADXL372 datasheet Rev B
 *
 * Configured for: +-200g (fixed range), 400 Hz ODR, 200 Hz bandwidth,
 * full bandwidth measurement mode, direct register polling (no FIFO).
 *
 * SPI protocol note: the ADXL372 address byte is (reg_addr << 1) | RNW,
 * where RNW=1 for read, RNW=0 for write.  This differs from LSM6DSO32
 * which uses bit 7 as the R/W flag.
 */

#include "adxl372.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  SPI helpers (using TransmitReceive for STM32H7 FIFO robustness)   */
/* ------------------------------------------------------------------ */

static inline void cs_low(const adxl372_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(const adxl372_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void adxl372_write_reg(adxl372_t *dev, uint8_t reg, uint8_t val)
{
    /* Address byte: (reg << 1) | 0 (write) */
    uint8_t tx[2] = {(uint8_t)(reg << 1), val};
    uint8_t rx[2];
    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100);
    cs_high(dev);
}

static uint8_t adxl372_read_reg(adxl372_t *dev, uint8_t reg)
{
    /* Address byte: (reg << 1) | 1 (read) */
    uint8_t tx[2] = {(uint8_t)((reg << 1) | 1u), 0x00};
    uint8_t rx[2] = {0};
    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100);
    cs_high(dev);
    return rx[1];
}

static void adxl372_read_burst(adxl372_t *dev, uint8_t reg,
                                uint8_t *buf, uint16_t len)
{
    /* Max burst: 1 addr + 6 data bytes (XYZ) */
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};
    tx[0] = (uint8_t)((reg << 1) | 1u);
    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len + 1, 100);
    cs_high(dev);
    memcpy(buf, &rx[1], len);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

bool adxl372_init(adxl372_t *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi       = hspi;
    dev->cs_port    = cs_port;
    dev->cs_pin     = cs_pin;
    dev->data_ready = false;
    dev->device_id  = 0;

    for (int i = 0; i < 3; i++) {
        dev->accel_g[i] = 0.0f;
    }

    /* CS idle high */
    cs_high(dev);

    /* Soft reset (write 0x52 to SRESET register) */
    adxl372_write_reg(dev, ADXL372_SRESET, ADXL372_RESET_CODE);
    HAL_Delay(10);  /* Wait for reset to complete */

    /* Read device ID (PARTID, register 0x02) for diagnostics */
    dev->device_id = adxl372_read_reg(dev, ADXL372_DEVID);

    /* MEASURE (0x3E): Low noise ON, BW = 200 Hz, no autosleep */
    adxl372_write_reg(dev, ADXL372_MEASURE,
                      ADXL372_LOW_NOISE_EN | ADXL372_BW_200HZ);

    /* TIMING (0x3D): ODR = 400 Hz (bits [7:5] = 000) — lowest available */
    adxl372_write_reg(dev, ADXL372_TIMING, ADXL372_ODR_400HZ);

    /* POWER_CTL (0x3F): Full BW measurement, LPF on (4-pole AA), HPF off */
    adxl372_write_reg(dev, ADXL372_POWER_CTL,
                      ADXL372_HPF_DISABLE | ADXL372_OP_FULL_BW_MEASUREMENT);

    return true;
}

int adxl372_read(adxl372_t *dev)
{
    uint8_t buf[6];

    /* Burst-read 6 bytes: X_DATA_H (0x08) through Z_DATA_L (0x0D) */
    adxl372_read_burst(dev, ADXL372_X_DATA_H, buf, 6);

    /*
     * Data format: 12-bit, left-justified in 16 bits.
     *   DATA_H = bits [11:4], DATA_L = bits [3:0] in D7:D4 (lower nibble zero).
     *   Combine as (H << 8 | L), then arithmetic right-shift by 4 to get
     *   signed 12-bit value.
     * Sensitivity: 100 mg/LSB.
     */
    int16_t raw_x = (int16_t)((uint16_t)buf[0] << 8 | buf[1]) >> 4;
    int16_t raw_y = (int16_t)((uint16_t)buf[2] << 8 | buf[3]) >> 4;
    int16_t raw_z = (int16_t)((uint16_t)buf[4] << 8 | buf[5]) >> 4;

    dev->accel_g[0] = (float)raw_x * 0.1f ;  /* 100 mg/LSB → g */
    dev->accel_g[1] = (float)raw_y * 0.1f ;
    dev->accel_g[2] = (float)raw_z * 0.1f;

    dev->data_ready = false;
    return ADXL372_READ_OK;
}

void adxl372_fifo_init(adxl372_t *dev, uint8_t odr_bits)
{
    /* Go to standby before reconfiguring */
    adxl372_write_reg(dev, ADXL372_POWER_CTL, ADXL372_OP_STANDBY);
    HAL_Delay(1);

    /* Set ODR and matching BW (BW = ODR/2) */
    adxl372_write_reg(dev, ADXL372_TIMING, odr_bits);

    /* BW = ODR/2: 800Hz ODR → 400Hz BW */
    uint8_t bw;
    switch (odr_bits) {
    case ADXL372_ODR_800HZ:  bw = ADXL372_BW_400HZ;  break;
    case ADXL372_ODR_1600HZ: bw = ADXL372_BW_800HZ;  break;
    case ADXL372_ODR_3200HZ: bw = ADXL372_BW_1600HZ; break;
    default:                 bw = ADXL372_BW_200HZ;   break;
    }
    adxl372_write_reg(dev, ADXL372_MEASURE, ADXL372_LOW_NOISE_EN | bw);

    /* FIFO: stream mode, XYZ format, 512 samples (max) */
    adxl372_write_reg(dev, ADXL372_FIFO_SAMPLES, 0xFF); /* lower 8 bits of watermark */
    adxl372_write_reg(dev, ADXL372_FIFO_CTL,
                      ADXL372_FIFO_STREAM | ADXL372_FIFO_FORMAT_XYZ);

    /* Full BW measurement, HPF off, LPF on */
    adxl372_write_reg(dev, ADXL372_POWER_CTL,
                      ADXL372_HPF_DISABLE | ADXL372_OP_FULL_BW_MEASUREMENT);
}

uint16_t adxl372_fifo_entries(adxl372_t *dev)
{
    uint8_t hi = adxl372_read_reg(dev, ADXL372_FIFO_ENTRIES_2);
    uint8_t lo = adxl372_read_reg(dev, ADXL372_FIFO_ENTRIES_1);
    return ((uint16_t)(hi & 0x03) << 8) | lo;
}

int adxl372_fifo_read(adxl372_t *dev)
{
    uint16_t entries = adxl372_fifo_entries(dev);
    if (entries < 3) return 0;  /* Need at least 3 entries for one XYZ triplet */

    uint8_t buf[6];
    adxl372_read_burst(dev, ADXL372_FIFO_DATA, buf, 6);

    /* 12-bit left-justified in 16 bits; shift right 4 to get signed 12-bit
       value (same conversion as adxl372_read).  FIFO marker bits in [3:0]
       are discarded by the shift. */
    dev->raw_accel[0] = (int16_t)((uint16_t)buf[0] << 8 | buf[1]) >> 4;
    dev->raw_accel[1] = (int16_t)((uint16_t)buf[2] << 8 | buf[3]) >> 4;
    dev->raw_accel[2] = (int16_t)((uint16_t)buf[4] << 8 | buf[5]) >> 4;

    return 1;
}

uint8_t adxl372_read_reg_ext(adxl372_t *dev, uint8_t reg)
{
    return adxl372_read_reg(dev, reg);
}

void adxl372_irq_handler(adxl372_t *dev)
{
    dev->data_ready = true;
}

void adxl372_wakeup_init(adxl372_t *dev, float threshold_g, uint8_t time_act)
{
    /* Go to standby before reconfiguring */
    adxl372_write_reg(dev, ADXL372_POWER_CTL, ADXL372_OP_STANDBY);
    HAL_Delay(1);

    /* Activity threshold: 11-bit, 100 mg/LSB
     * thresh_raw = threshold_g / 0.1
     * THRESH_ACT_H = bits [10:3], THRESH_ACT_L = bits [2:0] << 5 */
    uint16_t thresh_raw = (uint16_t)(threshold_g / 0.1f);
    adxl372_write_reg(dev, ADXL372_THRESH_ACT_H, (uint8_t)((thresh_raw >> 3) & 0xFF));
    adxl372_write_reg(dev, ADXL372_THRESH_ACT_L, (uint8_t)((thresh_raw & 0x07) << 5));

    /* Activity time: consecutive samples above threshold */
    adxl372_write_reg(dev, ADXL372_TIME_ACT, time_act);

    /* Activity enable, referenced mode */
    adxl372_write_reg(dev, ADXL372_ACT_INACT_CTL, 0x01);

    /* Map activity to INT1 (PD2) */
    adxl372_write_reg(dev, ADXL372_INT1_MAP, ADXL372_INT1_ACT);

    /* Enter wake-up mode */
    adxl372_write_reg(dev, ADXL372_POWER_CTL, ADXL372_OP_WAKE_UP);
}

bool adxl372_activity_detected(adxl372_t *dev)
{
    uint8_t status = adxl372_read_reg(dev, ADXL372_STATUS_2);
    return (status & ADXL372_STATUS2_ACT) != 0;
}

void adxl372_enter_measurement(adxl372_t *dev)
{
    /* Transition back to full bandwidth measurement mode.
     * Same configuration as adxl372_init(): low noise, 200Hz BW, 400Hz ODR */
    adxl372_write_reg(dev, ADXL372_MEASURE,
                      ADXL372_LOW_NOISE_EN | ADXL372_BW_200HZ);
    adxl372_write_reg(dev, ADXL372_TIMING, ADXL372_ODR_400HZ);
    adxl372_write_reg(dev, ADXL372_POWER_CTL,
                      ADXL372_HPF_DISABLE | ADXL372_OP_FULL_BW_MEASUREMENT);
}
