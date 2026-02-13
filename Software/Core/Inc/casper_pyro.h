#ifndef CASPER_PYRO_H
#define CASPER_PYRO_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define PYRO_NUM_CHANNELS          4
#define PYRO_CONTINUITY_THRESHOLD  8000   /* 16-bit ADC; tune empirically */
#define PYRO_DEFAULT_FIRE_MS       1000   /* 1 second default fire pulse  */

typedef struct {
    /* State */
    bool     continuity[PYRO_NUM_CHANNELS];
    uint16_t adc_raw[PYRO_NUM_CHANNELS];
    bool     firing[PYRO_NUM_CHANNELS];
    uint32_t fire_start_ms[PYRO_NUM_CHANNELS];
    uint32_t fire_duration_ms[PYRO_NUM_CHANNELS];

    /* Hardware handles (set during init) */
    ADC_HandleTypeDef *hadc1;   /* channels 1 & 2 (ADC1 CH4/CH3) */
    ADC_HandleTypeDef *hadc2;   /* channel 4      (ADC2 CH10)    */
    ADC_HandleTypeDef *hadc3;   /* channel 3      (ADC3 CH1)     */
} casper_pyro_t;

/* Init: store ADC handles, calibrate ADCs, force all pyros OFF */
void casper_pyro_init(casper_pyro_t *p,
                      ADC_HandleTypeDef *hadc1,
                      ADC_HandleTypeDef *hadc2,
                      ADC_HandleTypeDef *hadc3);

/* Fire channel ch (0-3) for duration_ms.  Returns false if ch invalid. */
bool casper_pyro_fire(casper_pyro_t *p, uint8_t ch, uint32_t duration_ms);

/* Immediately stop one channel */
void casper_pyro_stop(casper_pyro_t *p, uint8_t ch);

/* Stop all channels */
void casper_pyro_stop_all(casper_pyro_t *p);

/* Periodic tick (~10 Hz): read ADCs, update continuity + LEDs, auto-stop expired fires */
void casper_pyro_tick(casper_pyro_t *p);

#endif /* CASPER_PYRO_H */
