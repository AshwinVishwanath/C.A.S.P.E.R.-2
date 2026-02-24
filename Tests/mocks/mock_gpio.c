/* Mock GPIO implementation */
#include "mock_gpio.h"
#include <string.h>

static mock_gpio_log_entry_t s_log[MOCK_GPIO_LOG_SIZE];
static int s_log_count = 0;

/* Pin read return values — simple array */
#define MAX_PINS 16
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState state;
} mock_pin_state_t;

static mock_pin_state_t s_read_states[MAX_PINS * 5]; /* 5 ports * 16 pins */
static int s_read_count = 0;
static mock_pin_state_t s_write_states[MAX_PINS * 5];
static int s_write_count = 0;

void mock_gpio_reset(void)
{
    s_log_count = 0;
    s_read_count = 0;
    s_write_count = 0;
    memset(s_log, 0, sizeof(s_log));
    memset(s_read_states, 0, sizeof(s_read_states));
    memset(s_write_states, 0, sizeof(s_write_states));
}

int mock_gpio_get_log_count(void) { return s_log_count; }

const mock_gpio_log_entry_t *mock_gpio_get_log(int index)
{
    if (index < 0 || index >= s_log_count) return NULL;
    return &s_log[index];
}

void mock_gpio_set_read(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_PinState state)
{
    for (int i = 0; i < s_read_count; i++) {
        if (s_read_states[i].port == GPIOx && s_read_states[i].pin == pin) {
            s_read_states[i].state = state;
            return;
        }
    }
    if (s_read_count < MAX_PINS * 5) {
        s_read_states[s_read_count].port = GPIOx;
        s_read_states[s_read_count].pin = pin;
        s_read_states[s_read_count].state = state;
        s_read_count++;
    }
}

GPIO_PinState mock_gpio_get_written(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    for (int i = s_write_count - 1; i >= 0; i--) {
        if (s_write_states[i].port == GPIOx && s_write_states[i].pin == pin) {
            return s_write_states[i].state;
        }
    }
    return GPIO_PIN_RESET;
}

/* Override weak stubs */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if (s_log_count < MOCK_GPIO_LOG_SIZE) {
        s_log[s_log_count].port = GPIOx;
        s_log[s_log_count].pin = GPIO_Pin;
        s_log[s_log_count].state = PinState;
        s_log_count++;
    }
    /* Update write state */
    for (int i = 0; i < s_write_count; i++) {
        if (s_write_states[i].port == GPIOx && s_write_states[i].pin == GPIO_Pin) {
            s_write_states[i].state = PinState;
            return;
        }
    }
    if (s_write_count < MAX_PINS * 5) {
        s_write_states[s_write_count].port = GPIOx;
        s_write_states[s_write_count].pin = GPIO_Pin;
        s_write_states[s_write_count].state = PinState;
        s_write_count++;
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    for (int i = 0; i < s_read_count; i++) {
        if (s_read_states[i].port == GPIOx && s_read_states[i].pin == GPIO_Pin) {
            return s_read_states[i].state;
        }
    }
    return GPIO_PIN_RESET;
}
