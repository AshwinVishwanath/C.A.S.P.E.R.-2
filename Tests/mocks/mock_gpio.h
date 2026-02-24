/* Mock GPIO — records WritePin calls and returns configurable ReadPin values */
#ifndef MOCK_GPIO_H
#define MOCK_GPIO_H

#include "stm32h7xx_hal.h"

#define MOCK_GPIO_LOG_SIZE 64

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState state;
} mock_gpio_log_entry_t;

/* Reset log and pin states */
void mock_gpio_reset(void);

/* Get write log */
int mock_gpio_get_log_count(void);
const mock_gpio_log_entry_t *mock_gpio_get_log(int index);

/* Set what ReadPin returns for a given port/pin */
void mock_gpio_set_read(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_PinState state);

/* Get the last state written to a port/pin */
GPIO_PinState mock_gpio_get_written(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif /* MOCK_GPIO_H */
