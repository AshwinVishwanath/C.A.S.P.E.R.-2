/* Mock ADC — returns configurable ADC values */
#ifndef MOCK_ADC_H
#define MOCK_ADC_H

#include "stm32h7xx_hal.h"

/* Set what HAL_ADC_GetValue returns for a given ADC handle */
void mock_adc_set_value(ADC_HandleTypeDef *hadc, uint32_t value);

/* Reset all mock ADC state */
void mock_adc_reset(void);

#endif /* MOCK_ADC_H */
