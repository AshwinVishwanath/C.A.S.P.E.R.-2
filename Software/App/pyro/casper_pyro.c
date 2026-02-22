#include "casper_pyro.h"
#include "main.h"
#include <string.h>

/* ── Hardware lookup table ── */
static const struct {
    GPIO_TypeDef *fire_port;  uint16_t fire_pin;
    GPIO_TypeDef *led_port;   uint16_t led_pin;
    uint32_t      adc_channel;
    uint8_t       adc_idx;     /* 0 = hadc1, 1 = hadc2, 2 = hadc3 */
} hw[PYRO_NUM_CHANNELS] = {
    /* CH 1 */ { PY1_GPIO_Port, PY1_Pin, CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, ADC_CHANNEL_4,  0 },
    /* CH 2 */ { PY2_GPIO_Port, PY2_Pin, CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, ADC_CHANNEL_3,  0 },
    /* CH 3 */ { PY3_GPIO_Port, PY3_Pin, CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, ADC_CHANNEL_1,  2 },
    /* CH 4 */ { PY4_GPIO_Port, PY4_Pin, CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, ADC_CHANNEL_10, 1 },
};

/* ── Helpers ── */

static uint16_t read_adc(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;   /* longer sample for analog sense */
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;
    HAL_ADC_ConfigChannel(hadc, &cfg);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 2);              /* 2 ms timeout — plenty */
    uint16_t val = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return val;
}

static ADC_HandleTypeDef *get_adc(const casper_pyro_t *p, uint8_t idx)
{
    switch (idx) {
        case 0: return p->hadc1;
        case 1: return p->hadc2;
        case 2: return p->hadc3;
        default: return p->hadc1;
    }
}

/* ── Public API ── */

void casper_pyro_init(casper_pyro_t *p,
                      ADC_HandleTypeDef *hadc1,
                      ADC_HandleTypeDef *hadc2,
                      ADC_HandleTypeDef *hadc3)
{
    memset(p, 0, sizeof(*p));
    p->hadc1 = hadc1;
    p->hadc2 = hadc2;
    p->hadc3 = hadc3;

    /* Force all pyro outputs LOW */
    for (int i = 0; i < PYRO_NUM_CHANNELS; i++)
        HAL_GPIO_WritePin(hw[i].fire_port, hw[i].fire_pin, GPIO_PIN_RESET);

    /* Calibrate ADCs (required on STM32H7 before first conversion) */
    HAL_ADCEx_Calibration_Start(hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

bool casper_pyro_fire(casper_pyro_t *p, uint8_t ch, uint32_t duration_ms)
{
    if (ch >= PYRO_NUM_CHANNELS)
        return false;

    p->firing[ch]          = true;
    p->fire_start_ms[ch]   = HAL_GetTick();
    p->fire_duration_ms[ch] = duration_ms;

    HAL_GPIO_WritePin(hw[ch].fire_port, hw[ch].fire_pin, GPIO_PIN_SET);
    return true;
}

void casper_pyro_stop(casper_pyro_t *p, uint8_t ch)
{
    if (ch >= PYRO_NUM_CHANNELS)
        return;

    HAL_GPIO_WritePin(hw[ch].fire_port, hw[ch].fire_pin, GPIO_PIN_RESET);
    p->firing[ch] = false;
}

void casper_pyro_stop_all(casper_pyro_t *p)
{
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++)
        casper_pyro_stop(p, i);
}

void casper_pyro_tick(casper_pyro_t *p)
{
    /* ── 1. Read all continuity ADCs ── */
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        ADC_HandleTypeDef *adc = get_adc(p, hw[i].adc_idx);
        p->adc_raw[i] = read_adc(adc, hw[i].adc_channel);
        p->continuity[i] = (p->adc_raw[i] > PYRO_CONTINUITY_THRESHOLD);
    }

    /* ── 2. Update continuity LEDs ── */
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        HAL_GPIO_WritePin(hw[i].led_port, hw[i].led_pin,
                          p->continuity[i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /* ── 3. Auto-stop expired fires ── */
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < PYRO_NUM_CHANNELS; i++) {
        if (p->firing[i] && (now - p->fire_start_ms[i] >= p->fire_duration_ms[i]))
            casper_pyro_stop(p, i);
    }
}
