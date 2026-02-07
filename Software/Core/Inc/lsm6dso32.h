#ifndef LSM6DSO32_H
#define LSM6DSO32_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define LSM6DSO32_READ_OK   0
#define LSM6DSO32_ERROR    -1

/* Device identification */
#define LSM6DSO32_WHO_AM_I_VAL  0x6C

/* Register addresses */
#define LSM6DSO32_INT1_CTRL     0x0D
#define LSM6DSO32_INT2_CTRL     0x0E
#define LSM6DSO32_WHO_AM_I      0x0F
#define LSM6DSO32_CTRL1_XL      0x10
#define LSM6DSO32_CTRL2_G       0x11
#define LSM6DSO32_CTRL3_C       0x12
#define LSM6DSO32_STATUS_REG    0x1E
#define LSM6DSO32_OUTX_L_G      0x22
#define LSM6DSO32_OUTX_L_A      0x28

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    float              accel_g[3];    /* Acceleration in g */
    float              gyro_dps[3];   /* Angular rate in degrees/sec */
    volatile bool      data_ready;    /* Set by EXTI ISR */
    uint8_t            device_id;
} lsm6dso32_t;

/* Initialise: reset sensor, verify WHO_AM_I, configure accel/gyro/interrupt.
 * Returns true on success. */
bool lsm6dso32_init(lsm6dso32_t *dev, SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Read accelerometer + gyroscope data. Returns LSM6DSO32_READ_OK on success. */
int lsm6dso32_read(lsm6dso32_t *dev);

/* Call from HAL_GPIO_EXTI_Callback when INT1 pin fires. */
void lsm6dso32_irq_handler(lsm6dso32_t *dev);

#endif /* LSM6DSO32_H */
