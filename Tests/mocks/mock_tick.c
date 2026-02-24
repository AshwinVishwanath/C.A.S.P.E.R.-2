/* Mock tick implementation — overrides weak HAL_GetTick/HAL_Delay */
#include "mock_tick.h"
#include "stm32h7xx_hal.h"

static uint32_t s_mock_tick = 0;

void mock_tick_set(uint32_t tick) { s_mock_tick = tick; }
void mock_tick_advance(uint32_t delta_ms) { s_mock_tick += delta_ms; }
uint32_t mock_tick_get(void) { return s_mock_tick; }
void mock_tick_reset(void) { s_mock_tick = 0; }

/* Override weak stubs */
uint32_t HAL_GetTick(void) { return s_mock_tick; }
void HAL_Delay(uint32_t ms) { s_mock_tick += ms; }
