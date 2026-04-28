/**
 * @file logger_sanity.c
 * @brief CDC-triggered bench harness that runs the flight logger for ~5 s.
 */
#include "logger_sanity.h"

#ifdef LOGGER_SANITY

#include <stdio.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "buzzer.h"
#include "flight_logger.h"

#define BOOT_HOLD_MS    3000U
#define LOG_DURATION_MS 5000U
#define EMIT_PERIOD_MS  250U

typedef enum {
    SANITY_BOOT_HOLD = 0,
    SANITY_WAIT_GO,
    SANITY_LOGGING,
    SANITY_FINALIZE,
    SANITY_DONE
} sanity_state_t;

static struct {
    flight_logger_t *log;
    sanity_state_t   state;
    uint32_t         t_init_ms;
    uint32_t         t_launch_ms;
    uint32_t         t_last_emit_ms;
    bool             waitgo_banner_sent;
} S;

static void leds_set(bool l1, bool l2, bool l3, bool l4)
{
    HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, l1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, l2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, l3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, l4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void emit(const char *s)
{
    uint16_t len = 0;
    while (s[len] != '\0') len++;
    CDC_Transmit_FS((uint8_t *)s, len);
}

void logger_sanity_init(flight_logger_t *log)
{
    S.log = log;
    S.state = SANITY_BOOT_HOLD;
    S.t_init_ms = HAL_GetTick();
    S.t_launch_ms = 0;
    S.t_last_emit_ms = 0;
    S.waitgo_banner_sent = false;
}

void logger_sanity_tick(uint32_t now_ms)
{
    if (S.log == NULL) return;

    switch (S.state) {
    case SANITY_BOOT_HOLD: {
        bool on = ((now_ms / 500U) & 1U) != 0U;
        leds_set(on, false, false, false);
        if ((now_ms - S.t_init_ms) >= BOOT_HOLD_MS) {
            S.state = SANITY_WAIT_GO;
        }
        break;
    }

    case SANITY_WAIT_GO: {
        if (!S.waitgo_banner_sent) {
            emit("[SANITY] init complete - type 'go<Enter>' to start logging\r\n");
            S.waitgo_banner_sent = true;
        }
        bool on = ((now_ms / 125U) & 1U) != 0U;
        leds_set(on, false, false, false);
        if (cdc_sanity_take_go()) {
            flight_logger_launch(S.log);
            S.t_launch_ms = now_ms;
            S.t_last_emit_ms = now_ms;
            emit("[SANITY] GO received - logger launched\r\n");
            buzzer_beep_n(30, 1, 300, 500);
            leds_set(true, true, true, false);
            S.state = SANITY_LOGGING;
        }
        break;
    }

    case SANITY_LOGGING: {
        leds_set(true, true, true, false);
        if ((now_ms - S.t_last_emit_ms) >= EMIT_PERIOD_MS) {
            S.t_last_emit_ms = now_ms;
            float t = (float)(now_ms - S.t_launch_ms) / 1000.0f;
            unsigned long hr = (unsigned long)flight_logger_hr_records(S.log);
            unsigned long lr = (unsigned long)flight_logger_lr_records(S.log);
            char buf[80];
            int len = snprintf(buf, sizeof(buf),
                               ">t:%.1f,hr:%lu,lr:%lu\r\n",
                               (double)t, hr, lr);
            if (len > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
        }
        if ((now_ms - S.t_launch_ms) >= LOG_DURATION_MS) {
            S.state = SANITY_FINALIZE;
        }
        break;
    }

    case SANITY_FINALIZE: {
        flight_logger_finalize(S.log);
        emit("[SANITY] FINALIZE\r\n");
        HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
        HAL_GPIO_TogglePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin);
        HAL_GPIO_TogglePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin);
        emit("[SANITY] DONE - power-cycle and decode flash\r\n");
        buzzer_beep_n(30, 3, 50, 200);
        S.state = SANITY_DONE;
        break;
    }

    case SANITY_DONE:
    default: {
        bool on = ((now_ms / 500U) & 1U) != 0U;
        leds_set(on, on, on, on);
        break;
    }
    }
}

#else  /* !LOGGER_SANITY */

void logger_sanity_init(flight_logger_t *log)  { (void)log; }
void logger_sanity_tick(uint32_t now_ms)       { (void)now_ms; }

#endif /* LOGGER_SANITY */
