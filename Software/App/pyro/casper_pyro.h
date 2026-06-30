/* ============================================================
 *  TIER:     SAFETY-CRITICAL
 *  MODULE:   Pyro HAL
 *  SUMMARY:  Low-level pyro pin/ADC access. Driven only by Pyro Manager.
 * ============================================================ */
#ifndef CASPER_PYRO_H
#define CASPER_PYRO_H

#include "casper_port.h"   /* casper_adc_t, casper_pin_t, casper_gpio_write, casper_adc_read */
#include <stdint.h>
#include <stdbool.h>

#define PYRO_NUM_CHANNELS          4
#define PYRO_CONTINUITY_THRESHOLD  8000   /* 16-bit ADC; tune empirically */

typedef struct {
    /* State */
    bool     continuity[PYRO_NUM_CHANNELS];
    uint16_t adc_raw[PYRO_NUM_CHANNELS];
    bool     firing[PYRO_NUM_CHANNELS];
    uint32_t fire_start_ms[PYRO_NUM_CHANNELS];
    uint32_t fire_duration_ms[PYRO_NUM_CHANNELS];

    /* Board-provided hardware tables (set during init, never NULL) */
    casper_adc_t *cont[PYRO_NUM_CHANNELS];   /* one ADC channel per pyro continuity input */
    casper_pin_t  fire[PYRO_NUM_CHANNELS];   /* fire output pins (MOSFET gates)            */
    casper_pin_t  led[PYRO_NUM_CHANNELS];    /* continuity indicator LED pins               */
} casper_pyro_t;

/**
 * Init: store board-provided ADC + GPIO tables, force all fire pins LOW,
 * zero all state.
 *
 * @param p     Driver context to initialise.
 * @param adc   Pointer array of PYRO_NUM_CHANNELS casper_adc_t* (one per ch).
 *              Typically BSP_ADC_CONT (pointer to the board's ADC array).
 * @param fire  Array of PYRO_NUM_CHANNELS casper_pin_t for fire outputs.
 * @param led   Array of PYRO_NUM_CHANNELS casper_pin_t for continuity LEDs.
 *
 * NOTE: casper_adc_init_all() (HAL calibration) must be called BEFORE this.
 */
void casper_pyro_init(casper_pyro_t *p,
                      casper_adc_t  *adc[PYRO_NUM_CHANNELS],
                      casper_pin_t   fire[PYRO_NUM_CHANNELS],
                      casper_pin_t   led[PYRO_NUM_CHANNELS]);

/* Fire channel ch (0-3) for duration_ms.  Returns false if ch invalid. */
bool casper_pyro_fire(casper_pyro_t *p, uint8_t ch, uint32_t duration_ms);

/* Immediately stop one channel */
void casper_pyro_stop(casper_pyro_t *p, uint8_t ch);

/* Stop all channels */
void casper_pyro_stop_all(casper_pyro_t *p);

/* Periodic tick (~10 Hz): read ADCs, update continuity + LEDs, auto-stop expired fires */
void casper_pyro_tick(casper_pyro_t *p);

#endif /* CASPER_PYRO_H */
