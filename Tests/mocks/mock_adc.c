/* Mock ADC implementation */
#include "mock_adc.h"

#define MAX_ADC_HANDLES 4

typedef struct {
    ADC_HandleTypeDef *handle;
    uint32_t value;
} mock_adc_entry_t;

static mock_adc_entry_t s_entries[MAX_ADC_HANDLES];
static int s_count = 0;

void mock_adc_set_value(ADC_HandleTypeDef *hadc, uint32_t value)
{
    for (int i = 0; i < s_count; i++) {
        if (s_entries[i].handle == hadc) {
            s_entries[i].value = value;
            return;
        }
    }
    if (s_count < MAX_ADC_HANDLES) {
        s_entries[s_count].handle = hadc;
        s_entries[s_count].value = value;
        s_count++;
    }
}

void mock_adc_reset(void)
{
    s_count = 0;
}

/* Override weak stubs */
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{
    (void)hadc; (void)Timeout;
    return HAL_OK;
}

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
{
    for (int i = 0; i < s_count; i++) {
        if (s_entries[i].handle == hadc) {
            return s_entries[i].value;
        }
    }
    return 0;
}

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc,
                                         ADC_ChannelConfTypeDef *sConfig)
{
    (void)hadc; (void)sConfig;
    return HAL_OK;
}
