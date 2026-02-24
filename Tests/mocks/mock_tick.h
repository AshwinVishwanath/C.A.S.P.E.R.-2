/* Mock tick — deterministic HAL_GetTick() for state machine testing */
#ifndef MOCK_TICK_H
#define MOCK_TICK_H

#include <stdint.h>

/* Set the current tick value */
void mock_tick_set(uint32_t tick);

/* Advance tick by delta ms */
void mock_tick_advance(uint32_t delta_ms);

/* Get current mock tick */
uint32_t mock_tick_get(void);

/* Reset to 0 */
void mock_tick_reset(void);

#endif /* MOCK_TICK_H */
