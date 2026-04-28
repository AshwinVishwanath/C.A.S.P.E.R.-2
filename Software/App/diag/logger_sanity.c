/**
 * @file logger_sanity.c
 * @brief CDC-triggered bench harness that runs the flight logger for ~5 s.
 */
#include "logger_sanity.h"

#ifdef LOGGER_SANITY

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "buzzer.h"
#include "flight_logger.h"
#include "w25q512jv.h"

#define BOOT_HOLD_MS    3000U
#define LOG_DURATION_MS 5000U
#define EMIT_PERIOD_MS  250U

typedef enum {
    SANITY_BOOT_HOLD = 0,
    SANITY_WAIT_GO,
    SANITY_LOGGING,
    SANITY_FINALIZE,
    SANITY_DONE,
    SANITY_FAIL_DRIVER,    /* w25q512jv_write returned non-OK */
    SANITY_FAIL_SILENT,    /* write OK but readback unchanged — likely BP/WPS */
    SANITY_FAIL_CORRUPT    /* readback differs from both pre and pattern */
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

static void emit_hex16(const char *prefix, const uint8_t *buf)
{
    char line[96];
    int n = snprintf(line, sizeof(line),
        "%s %02x %02x %02x %02x %02x %02x %02x %02x  "
        "%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
        prefix,
        buf[0], buf[1], buf[2],  buf[3],  buf[4],  buf[5],  buf[6],  buf[7],
        buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
    if (n > 0) CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
}

/* Sector-0 write probe.
 *
 * Reads 16 bytes at flash 0x00000000, writes a known pattern, reads back,
 * compares. Reports verdict via CDC + sets the sanity state to either
 * BOOT_HOLD (probe passed) or one of the FAIL states (probe failed). Uses
 * HAL_Delay between prints so the human watching a serial terminal sees
 * each step. Runs once during init.
 */
static sanity_state_t do_sector0_probe(flight_logger_t *log)
{
    static const uint8_t pattern[16] = {
        0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A,
        0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A
    };
    uint8_t pre[16]  = {0};
    uint8_t post[16] = {0};
    char line[96];
    int n;

    HAL_Delay(200);
    emit("\r\n[PROBE] sector-0 write probe starting\r\n");
    HAL_Delay(200);

    int rc = w25q512jv_read(log->index.flash, 0x00000000U, pre, 16);
    if (rc != W25Q_OK) {
        n = snprintf(line, sizeof(line),
            "[PROBE] FAIL: pre-read driver error %d\r\n", rc);
        if (n > 0) CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
        return SANITY_FAIL_DRIVER;
    }
    emit_hex16("[PROBE] pre: ", pre);
    HAL_Delay(200);

    bool pre_all_ff = true;
    for (int i = 0; i < 16; i++) {
        if (pre[i] != 0xFF) { pre_all_ff = false; break; }
    }
    if (!pre_all_ff) {
        emit("[PROBE] WARNING: sector 0 not erased; verdict will be inconclusive\r\n");
        HAL_Delay(200);
    }

    emit("[PROBE] writing pattern A5 5A x8 at 0x00000000\r\n");
    HAL_Delay(200);
    rc = w25q512jv_write(log->index.flash, 0x00000000U, pattern, 16);
    n = snprintf(line, sizeof(line),
        "[PROBE] w25q512jv_write returned: %d (W25Q_OK=%d)\r\n",
        rc, W25Q_OK);
    if (n > 0) CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
    HAL_Delay(200);

    if (rc != W25Q_OK) {
        emit("[PROBE] verdict: FAIL — driver error\r\n");
        HAL_Delay(300);
        return SANITY_FAIL_DRIVER;
    }

    int rrc = w25q512jv_read(log->index.flash, 0x00000000U, post, 16);
    if (rrc != W25Q_OK) {
        n = snprintf(line, sizeof(line),
            "[PROBE] post-read driver error %d\r\n", rrc);
        if (n > 0) CDC_Transmit_FS((uint8_t *)line, (uint16_t)n);
        return SANITY_FAIL_DRIVER;
    }
    emit_hex16("[PROBE] post:", post);
    HAL_Delay(200);

    if (memcmp(post, pattern, 16) == 0) {
        emit("[PROBE] verdict: PASS — sector 0 write works\r\n");
        emit("[PROBE]          bug is in log_index_start_flight() or its caller\r\n");
        HAL_Delay(500);
        return SANITY_BOOT_HOLD;
    }
    if (memcmp(post, pre, 16) == 0) {
        emit("[PROBE] verdict: FAIL — silent write (likely BP/WPS bits)\r\n");
        HAL_Delay(500);
        return SANITY_FAIL_SILENT;
    }
    emit("[PROBE] verdict: FAIL — corruption (post != pattern AND post != pre)\r\n");
    HAL_Delay(500);
    return SANITY_FAIL_CORRUPT;
}

void logger_sanity_init(flight_logger_t *log)
{
    S.log = log;
    S.t_launch_ms = 0;
    S.t_last_emit_ms = 0;
    S.waitgo_banner_sent = false;

    S.state = do_sector0_probe(log);
    S.t_init_ms = HAL_GetTick();   /* anchor BOOT_HOLD after probe completes */
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
            emit("[SANITY] GO detected - calling flight_logger_launch...\r\n");
            HAL_Delay(50);
            flight_logger_launch(S.log);
            S.t_launch_ms = HAL_GetTick();
            S.t_last_emit_ms = S.t_launch_ms;
            emit("[SANITY] launch returned - logging for 5s\r\n");
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

    case SANITY_DONE: {
        bool on = ((now_ms / 500U) & 1U) != 0U;
        leds_set(on, on, on, on);
        break;
    }

    case SANITY_FAIL_DRIVER: {
        /* LED2+LED3 alternating 4 Hz forever */
        bool on = ((now_ms / 125U) & 1U) != 0U;
        leds_set(false, on, !on, false);
        break;
    }

    case SANITY_FAIL_SILENT: {
        /* LED1+LED4 alternating 4 Hz forever */
        bool on = ((now_ms / 125U) & 1U) != 0U;
        leds_set(on, false, false, !on);
        break;
    }

    case SANITY_FAIL_CORRUPT:
    default: {
        /* All 4 LEDs alternating 4 Hz forever */
        bool on = ((now_ms / 125U) & 1U) != 0U;
        leds_set(on, on, on, on);
        break;
    }
    }
}

#else  /* !LOGGER_SANITY */

void logger_sanity_init(flight_logger_t *log)  { (void)log; }
void logger_sanity_tick(uint32_t now_ms)       { (void)now_ms; }

#endif /* LOGGER_SANITY */
