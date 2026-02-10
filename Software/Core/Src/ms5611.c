/**
 * MS5611 SPI barometer driver — bare-metal C for STM32 HAL.
 * Converted from RobTillaart/MS5611_SPI (Arduino C++ library).
 * https://github.com/RobTillaart/MS5611_SPI
 */

#include "ms5611.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static inline void cs_low(const ms5611_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(const ms5611_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void ms5611_command(ms5611_t *dev, uint8_t cmd)
{
    cs_low(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    cs_high(dev);
}

static uint16_t ms5611_read_prom(ms5611_t *dev, uint8_t reg)
{
    if (reg > 7) return 0;

    uint8_t cmd = MS5611_CMD_READ_PROM + reg * 2;
    uint8_t buf[2] = {0};

    cs_low(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, buf, 2, 100);
    cs_high(dev);

    return ((uint16_t)buf[0] << 8) | buf[1];
}

static uint32_t ms5611_read_adc(ms5611_t *dev)
{
    uint8_t cmd = MS5611_CMD_READ_ADC;
    uint8_t buf[3] = {0};

    cs_low(dev);
    HAL_SPI_Transmit(dev->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(dev->hspi, buf, 3, 100);
    cs_high(dev);

    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

/* Conversion delays in microseconds for OSR 256..4096 (rounded up) */
static const uint16_t conv_delay_us[5] = {600, 1200, 2300, 4600, 9100};

static void ms5611_convert(ms5611_t *dev, uint8_t base_cmd)
{
    uint8_t idx = (uint8_t)dev->osr;
    if (idx > 4) idx = 4;

    uint8_t cmd = base_cmd + idx * 2;
    ms5611_command(dev, cmd);

    /* Wait for conversion — HAL_Delay works in milliseconds, round up */
    uint32_t delay_ms = (conv_delay_us[idx] + 999) / 1000;
    HAL_Delay(delay_ms);
}

/* Compute temperature & pressure from raw D1, D2 ADC values */
static void ms5611_compute(ms5611_t *dev, uint32_t D1, uint32_t D2)
{
    float dT = (float)D2 - dev->C[5];
    dev->temperature = (int32_t)(2000.0f + dT * dev->C[6]);

    float offset = dev->C[2] + dT * dev->C[4];
    float sens   = dev->C[1] + dT * dev->C[3];

    if (dev->compensation) {
        if (dev->temperature < 2000) {
            float T2 = dT * dT * 4.6566128731E-10f;
            float t  = (float)(dev->temperature - 2000) *
                       (float)(dev->temperature - 2000);
            float offset2 = 2.5f * t;
            float sens2   = 1.25f * t;

            if (dev->temperature < -1500) {
                t = (float)(dev->temperature + 1500) *
                    (float)(dev->temperature + 1500);
                offset2 += 7.0f * t;
                sens2   += 5.5f * t;
            }
            dev->temperature -= (int32_t)T2;
            offset -= offset2;
            sens   -= sens2;
        }
    }

    dev->pressure = (int32_t)(((float)D1 * sens * 4.76837158205E-7f - offset)
                    * 3.051757813E-5f);
}

static void ms5611_init_constants(float C[7])
{
    /* Pre-scaling factors from MS5611 datasheet page 7-8 (mathMode 0).
     * These get multiplied by the PROM values during reset(). */
    C[0] = 1;
    C[1] = 32768.0f;          /* SENSt1   = C1 * 2^15 */
    C[2] = 65536.0f;          /* OFFt1    = C2 * 2^16 */
    C[3] = 3.90625E-3f;       /* TCS      = C3 / 2^8  */
    C[4] = 7.8125E-3f;        /* TCO      = C4 / 2^7  */
    C[5] = 256.0f;            /* Tref     = C5 * 2^8  */
    C[6] = 1.1920928955E-7f;  /* TEMPSENS = C6 / 2^23 */
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

bool ms5611_init(ms5611_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi        = hspi;
    dev->cs_port     = cs_port;
    dev->cs_pin      = cs_pin;
    dev->osr         = MS5611_OSR_4096;
    dev->temperature = 0;
    dev->pressure    = 0;
    dev->last_read   = 0;
    dev->last_result = MS5611_ERROR;
    dev->device_id   = 0;
    dev->compensation = true;

    /* CS idle high */
    cs_high(dev);

    /* Reset */
    ms5611_command(dev, MS5611_CMD_RESET);
    HAL_Delay(4); /* Datasheet: 2.8 ms reload, use 4 ms margin */

    /* Initialise scaling constants */
    ms5611_init_constants(dev->C);

    /* Read PROM calibration registers */
    bool rom_ok = true;
    for (uint8_t reg = 0; reg < 7; reg++) {
        uint16_t val = ms5611_read_prom(dev, reg);
        dev->prom[reg] = val;
        dev->C[reg] *= (float)val;

        dev->device_id <<= 4;
        dev->device_id ^= val;

        if (reg > 0 && val == 0) {
            rom_ok = false;
        }
    }

    return rom_ok;
}

int ms5611_read(ms5611_t *dev)
{
    /* --- D1: pressure (blocking) --- */
    ms5611_convert(dev, MS5611_CMD_CONVERT_D1);
    uint32_t D1 = ms5611_read_adc(dev);

    /* --- D2: temperature (blocking) --- */
    ms5611_convert(dev, MS5611_CMD_CONVERT_D2);
    uint32_t D2 = ms5611_read_adc(dev);

    ms5611_compute(dev, D1, D2);

    dev->last_read   = HAL_GetTick();
    dev->last_result = MS5611_READ_OK;
    return MS5611_READ_OK;
}

float ms5611_get_temperature(const ms5611_t *dev)
{
    return (float)dev->temperature * 0.01f;
}

float ms5611_get_pressure(const ms5611_t *dev)
{
    return (float)dev->pressure * 0.01f;
}

float ms5611_get_altitude(const ms5611_t *dev, float sea_level_hPa)
{
    float pressure_hPa = (float)dev->pressure * 0.01f;
    float ratio = pressure_hPa / sea_level_hPa;
    return 44307.694f * (1.0f - powf(ratio, 0.190284f));
}

int ms5611_read_raw(ms5611_t *dev)
{
    /* D1: pressure */
    ms5611_convert(dev, MS5611_CMD_CONVERT_D1);
    dev->raw_pressure = ms5611_read_adc(dev);

    /* D2: temperature */
    ms5611_convert(dev, MS5611_CMD_CONVERT_D2);
    dev->raw_temperature = ms5611_read_adc(dev);

    return MS5611_READ_OK;
}

void ms5611_set_oversampling(ms5611_t *dev, ms5611_osr_t osr)
{
    dev->osr = osr;

    /* Pre-compute delay for non-blocking tick (round up us → ms) */
    uint8_t idx = (uint8_t)osr;
    if (idx > 4) idx = 4;
    dev->nb_delay_ms = (conv_delay_us[idx] + 999) / 1000;
}

int ms5611_tick(ms5611_t *dev)
{
    uint32_t now = HAL_GetTick();

    switch (dev->nb_state) {
    case MS5611_NB_IDLE:
        /* Start D1 (pressure) conversion */
        {
            uint8_t idx = (uint8_t)dev->osr;
            if (idx > 4) idx = 4;
            ms5611_command(dev, MS5611_CMD_CONVERT_D1 + idx * 2);
        }
        dev->nb_convert_start = now;
        dev->nb_state = MS5611_NB_CONVERTING_D1;
        return 0;

    case MS5611_NB_CONVERTING_D1:
        if (now - dev->nb_convert_start >= dev->nb_delay_ms) {
            dev->raw_pressure = ms5611_read_adc(dev);
            /* Start D2 (temperature) conversion */
            uint8_t idx = (uint8_t)dev->osr;
            if (idx > 4) idx = 4;
            ms5611_command(dev, MS5611_CMD_CONVERT_D2 + idx * 2);
            dev->nb_convert_start = now;
            dev->nb_state = MS5611_NB_CONVERTING_D2;
        }
        return 0;

    case MS5611_NB_CONVERTING_D2:
        if (now - dev->nb_convert_start >= dev->nb_delay_ms) {
            dev->raw_temperature = ms5611_read_adc(dev);
            ms5611_compute(dev, dev->raw_pressure, dev->raw_temperature);
            dev->last_read   = now;
            dev->last_result = MS5611_READ_OK;
            dev->nb_state = MS5611_NB_IDLE;
            return 1;  /* New data ready */
        }
        return 0;
    }

    return 0;
}
