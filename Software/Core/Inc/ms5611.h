#ifndef MS5611_H
#define MS5611_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define MS5611_READ_OK   0
#define MS5611_ERROR    -1

/* SPI commands */
#define MS5611_CMD_RESET      0x1E
#define MS5611_CMD_READ_ADC   0x00
#define MS5611_CMD_READ_PROM  0xA0
#define MS5611_CMD_CONVERT_D1 0x40
#define MS5611_CMD_CONVERT_D2 0x50

typedef enum {
    MS5611_OSR_256  = 0,  /*  0.6 ms */
    MS5611_OSR_512  = 1,  /*  1.2 ms */
    MS5611_OSR_1024 = 2,  /*  2.3 ms */
    MS5611_OSR_2048 = 3,  /*  4.6 ms */
    MS5611_OSR_4096 = 4   /*  9.1 ms */
} ms5611_osr_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    float              C[7];          /* Pre-scaled calibration constants */
    int32_t            temperature;   /* 0.01 deg C */
    int32_t            pressure;      /* Pascal     */
    ms5611_osr_t       osr;
    uint32_t           device_id;
    uint32_t           last_read;
    int                last_result;
    bool               compensation;
} ms5611_t;

/* Initialise: reset sensor, read PROM calibration. Returns true on success. */
bool  ms5611_init(ms5611_t *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Blocking read of temperature + pressure. Returns MS5611_READ_OK on success. */
int   ms5611_read(ms5611_t *dev);

/* Temperature in degrees Celsius. */
float ms5611_get_temperature(const ms5611_t *dev);

/* Pressure in millibar (hPa). */
float ms5611_get_pressure(const ms5611_t *dev);

/* Altitude in metres (barometric formula). sea_level_hPa = e.g. 1013.25 */
float ms5611_get_altitude(const ms5611_t *dev, float sea_level_hPa);

/* Set oversampling rate (default: MS5611_OSR_4096). */
void  ms5611_set_oversampling(ms5611_t *dev, ms5611_osr_t osr);

#endif /* MS5611_H */
