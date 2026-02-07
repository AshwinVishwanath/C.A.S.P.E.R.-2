/**
 * LSM6DSO32 SPI IMU driver — bare-metal C for STM32 HAL.
 * Reference: STMicroelectronics/lsm6dso32-pid
 *
 * Configured for: ±32g accel, ±2000 dps gyro, 104 Hz high-performance,
 * accelerometer data-ready interrupt on INT2 (INT1 is NC on this PCB).
 */

#include "lsm6dso32.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  SPI helpers (using TransmitReceive for STM32H7 FIFO robustness)   */
/* ------------------------------------------------------------------ */

static inline void cs_low(const lsm6dso32_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(const lsm6dso32_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void lsm6dso32_write_reg(lsm6dso32_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val};
    uint8_t rx[2];
    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100);
    cs_high(dev);
}

static uint8_t lsm6dso32_read_reg(lsm6dso32_t *dev, uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};
    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, 100);
    cs_high(dev);
    return rx[1];
}

static void lsm6dso32_read_burst(lsm6dso32_t *dev, uint8_t reg,
                                  uint8_t *buf, uint16_t len)
{
    uint8_t tx[13] = {0};  /* max: 1 addr + 12 data */
    uint8_t rx[13] = {0};
    /* SPI multi-byte read: set READ (bit7) + auto-increment (bit6). */
    tx[0] = reg | 0xC0;
    cs_low(dev);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, len + 1, 100);
    cs_high(dev);
    memcpy(buf, &rx[1], len);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

bool lsm6dso32_init(lsm6dso32_t *dev, SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi       = hspi;
    dev->cs_port    = cs_port;
    dev->cs_pin     = cs_pin;
    dev->data_ready = false;
    dev->device_id  = 0;

    for (int i = 0; i < 3; i++) {
        dev->accel_g[i]  = 0.0f;
        dev->gyro_dps[i] = 0.0f;
    }

    /* CS idle high */
    cs_high(dev);

    /* Software reset */
    lsm6dso32_write_reg(dev, LSM6DSO32_CTRL3_C, 0x01);
    HAL_Delay(20);  /* Boot time ~15ms per datasheet */

    /* Read WHO_AM_I (stored in dev->device_id for diagnostics) */
    dev->device_id = lsm6dso32_read_reg(dev, LSM6DSO32_WHO_AM_I);

    /* CTRL3_C: BDU=1, IF_INC=1 (auto-increment for burst reads) */
    lsm6dso32_write_reg(dev, LSM6DSO32_CTRL3_C, 0x44);

    /* CTRL1_XL: ODR=104Hz high-perf (0100b), FS=±32g (01b) = 0x41 */
    lsm6dso32_write_reg(dev, LSM6DSO32_CTRL1_XL, 0x41);

    /* CTRL2_G: ODR=104Hz high-perf (0100b), FS=±2000dps (110b) = 0x4C */
    lsm6dso32_write_reg(dev, LSM6DSO32_CTRL2_G, 0x4C);

    /* INT2_CTRL: route accelerometer data-ready to INT2 (INT1 is NC on this PCB) */
    lsm6dso32_write_reg(dev, LSM6DSO32_INT2_CTRL, 0x01);

    return true;
}

int lsm6dso32_read(lsm6dso32_t *dev)
{
    uint8_t buf[12];

    /* Burst-read 12 bytes: OUTX_L_G (0x22) through OUTZ_H_A (0x2D) */
    lsm6dso32_read_burst(dev, LSM6DSO32_OUTX_L_G, buf, 12);

    /* Gyroscope: bytes 0-5 (little-endian int16) */
    int16_t gx = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    int16_t gy = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    int16_t gz = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);

    /* Accelerometer: bytes 6-11 (little-endian int16) */
    int16_t ax = (int16_t)((uint16_t)buf[7]  << 8 | buf[6]);
    int16_t ay = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    int16_t az = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);

    /* Convert to physical units */
    /* ±32g: sensitivity = 0.976 mg/LSB → g = raw * 0.000976 */
    dev->accel_g[0] = (float)ax * 0.000976f;
    dev->accel_g[1] = (float)ay * 0.000976f;
    dev->accel_g[2] = (float)az * 0.000976f;

    /* ±2000 dps: sensitivity = 70.0 mdps/LSB → dps = raw * 0.070 */
    dev->gyro_dps[0] = (float)gx * 0.070f;
    dev->gyro_dps[1] = (float)gy * 0.070f;
    dev->gyro_dps[2] = (float)gz * 0.070f;

    dev->data_ready = false;
    return LSM6DSO32_READ_OK;
}

void lsm6dso32_irq_handler(lsm6dso32_t *dev)
{
    dev->data_ready = true;
}
