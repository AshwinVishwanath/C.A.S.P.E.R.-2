/**
 * @file  casper_adc.h
 * @brief Portable ADC interface (pyro-channel continuity sensing).
 *
 * Pure C11.  Implemented in the board layer (board_<target>.c).
 *
 * Each logical ADC channel is represented by an opaque casper_adc_t.
 * On Casper 2 the board layer creates one per pyro continuity input,
 * bundling the HAL ADC handle, channel number, and any per-channel
 * configuration needed for ADCEx calibration.
 *
 * Design rationale:
 *   The STM32H7 ADC requires per-channel ChannelConfTypeDef setup and
 *   HAL_ADCEx_Calibration_Start() — both are intrinsically board-wired.
 *   casper_adc_init_all() performs those steps in the board layer so
 *   App/ code never needs to reference ADC_ChannelConfTypeDef or HAL
 *   calibration functions.
 */

#ifndef CASPER_ADC_H
#define CASPER_ADC_H

#include "casper_types.h"

/**
 * @brief Initialise all ADC channels (calibration, channel config).
 *
 * Must be called once at startup, after MX_ADC*_Init() has been called
 * by the generated board init.  On STM32H7 this runs
 * HAL_ADCEx_Calibration_Start() for each ADC peripheral and configures
 * per-channel rank/sampling-time settings.
 *
 * App/ code calls this once and then uses casper_adc_read() per channel.
 */
void casper_adc_init_all(void);

/**
 * @brief Perform a single-shot ADC conversion and return the raw result.
 *
 * Blocking.  Configures the channel (if not already active), starts the
 * conversion, polls for completion, and returns the raw 16-bit result.
 *
 * On Casper 2: equivalent to HAL_ADC_Start() + HAL_ADC_PollForConversion()
 * + HAL_ADC_GetValue() + HAL_ADC_Stop() for the channel wrapped by @p ch.
 *
 * @param ch       Opaque ADC channel handle (one per logical continuity input).
 * @param out_raw  Pointer to receive the raw ADC value (12-bit result in
 *                 a uint16_t on STM32H7, range 0–4095).
 * @return         CASPER_OK on success, CASPER_TIMEOUT if the conversion did
 *                 not complete, CASPER_ERR on HAL error.
 */
casper_status_t casper_adc_read(casper_adc_t *ch, uint16_t *out_raw);

#endif /* CASPER_ADC_H */
