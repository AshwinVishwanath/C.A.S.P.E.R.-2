/**
 * hse_test.c — HSE Crystal Test Campaign for C.A.S.P.E.R.-2
 *
 * 8 sequential test phases to determine if the 24 MHz HSE crystal
 * is truly dead, marginal, or potentially recoverable.
 *
 * SAFETY: SYSCLK stays on HSI+PLL throughout. HSE is only enabled
 * as an independent oscillator for observation — never used as
 * PLL source or SYSCLK.
 */

#include "hse_test.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "buzzer.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ── Globals ─────────────────────────────────────────────────── */

/* CSS NMI flag — set by NMI_Handler in stm32h7xx_it.c */
volatile uint8_t g_hse_css_nmi_fired = 0;

/* Track if HSE ever started across all phases */
static bool s_hse_ever_started = false;

/* Per-phase results */
static hse_test_result_t s_results[HSE_TEST_NUM_PHASES];

/* ── CDC helper ──────────────────────────────────────────────── */

static char s_cdc_buf[256];

static void hse_cdc_print(const char *msg)
{
    uint16_t len = (uint16_t)strlen(msg);
    /* Retry if USB busy */
    for (int i = 0; i < 10; i++) {
        if (CDC_Transmit_FS((uint8_t *)msg, len) != 1) break; /* 1 = USBD_BUSY */
        HAL_Delay(5);
    }
    HAL_Delay(3);
}

static void hse_cdc_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_cdc_buf, sizeof(s_cdc_buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        hse_cdc_print(s_cdc_buf);
    }
}

/* ── LED helpers ─────────────────────────────────────────────── */

static void leds_all_off(void)
{
    HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
}

static void leds_all_on(void)
{
    HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
}

/* Show phase number in binary on LED4(MSB)-LED3-LED2, LED1 blinks */
static void leds_show_phase(uint8_t phase)
{
    /* phase 1-8 mapped to 3-bit binary on LED4/LED3/LED2 */
    uint8_t bits = phase & 0x07;
    HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin,
                      (bits & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin,
                      (bits & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin,
                      (bits & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Show result pattern for 2 seconds */
static void leds_show_result(hse_result_status_t status)
{
    uint32_t end = HAL_GetTick() + 2000;

    switch (status) {
    case HSE_RESULT_PASS:
        leds_all_on();
        while (HAL_GetTick() < end) {}
        break;

    case HSE_RESULT_FAIL:
    case HSE_RESULT_TIMEOUT:
        /* Rapid blink all 4 for fail, checkerboard for timeout */
        while (HAL_GetTick() < end) {
            if (status == HSE_RESULT_FAIL) {
                leds_all_on();
                HAL_Delay(100);
                leds_all_off();
                HAL_Delay(100);
            } else {
                /* checkerboard: 1+3 ON, 2+4 OFF */
                HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_RESET);
                HAL_Delay(200);
                HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);
                HAL_Delay(200);
            }
        }
        break;

    case HSE_RESULT_SKIPPED:
        leds_all_off();
        HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
        while (HAL_GetTick() < end) {}
        break;

    case HSE_RESULT_INFO:
        /* Slow pulse all LEDs */
        while (HAL_GetTick() < end) {
            leds_all_on();
            HAL_Delay(500);
            leds_all_off();
            HAL_Delay(500);
        }
        break;
    }

    leds_all_off();
}

/* ── Buzzer helpers (blocking — acceptable for test firmware) ── */

extern TIM_HandleTypeDef htim4;

static void buzzer_blocking_tone(uint16_t freq_hz, uint16_t duration_ms)
{
    if (freq_hz == 0) return;
    uint32_t arr = (216000000UL / 2) / freq_hz - 1; /* TIM4 on APB1 = SYSCLK/2 */
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, arr / 2); /* 50% duty */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_Delay(duration_ms);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

static void buzzer_pass_sound(void)
{
    buzzer_blocking_tone(1000, 80);
    HAL_Delay(40);
    buzzer_blocking_tone(1500, 80);
}

static void buzzer_fail_sound(void)
{
    buzzer_blocking_tone(300, 200);
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 1: Basic HSE Enable (100 ms timeout)
 * ═══════════════════════════════════════════════════════════════ */
static void phase1_basic_hse(hse_test_result_t *r)
{
    r->phase = 1;

    uint32_t cr_before = RCC->CR;
    hse_cdc_printf("[HSE] RCC->CR before: 0x%08lX\r\n", cr_before);

    /* Clean slate: HSE off */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
    HAL_Delay(10);

    /* Set HSEON */
    hse_cdc_printf("[HSE] Setting HSEON, polling HSERDY for 100 ms...\r\n");
    RCC->CR |= RCC_CR_HSEON;

    uint32_t start = HAL_GetTick();
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        if ((HAL_GetTick() - start) > 100) break;
    }

    uint32_t cr_after = RCC->CR;
    bool hserdy = (cr_after & RCC_CR_HSERDY) != 0;
    uint32_t elapsed = HAL_GetTick() - start;

    hse_cdc_printf("[HSE] RCC->CR after:  0x%08lX\r\n", cr_after);
    hse_cdc_printf("[HSE] HSERDY: %d  (elapsed %lu ms)\r\n", hserdy ? 1 : 0, elapsed);

    /* Turn HSE back off */
    RCC->CR &= ~RCC_CR_HSEON;
    HAL_Delay(10);

    if (hserdy) s_hse_ever_started = true;

    r->passed = hserdy;
    r->status = hserdy ? HSE_RESULT_PASS : HSE_RESULT_FAIL;
    r->detail = cr_after;
    r->startup_ms = hserdy ? elapsed : 0;
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 2: Extended HSE Enable (2000 ms timeout)
 * ═══════════════════════════════════════════════════════════════ */
static void phase2_extended_hse(hse_test_result_t *r)
{
    r->phase = 2;

    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
    HAL_Delay(10);

    hse_cdc_printf("[HSE] Setting HSEON, polling HSERDY for 2000 ms...\r\n");
    RCC->CR |= RCC_CR_HSEON;

    bool started = false;
    for (int i = 0; i < 20; i++) {
        HAL_Delay(100);
        uint32_t cr = RCC->CR;
        bool rdy = (cr & RCC_CR_HSERDY) != 0;
        hse_cdc_printf("[HSE]   t=%4d ms  HSERDY=%d  CR=0x%08lX\r\n",
                       (i + 1) * 100, rdy ? 1 : 0, cr);
        if (rdy) {
            started = true;
            r->startup_ms = (uint32_t)(i + 1) * 100;
            break;
        }
    }

    RCC->CR &= ~RCC_CR_HSEON;
    HAL_Delay(10);

    if (started) s_hse_ever_started = true;

    r->passed = started;
    r->status = started ? HSE_RESULT_PASS : HSE_RESULT_TIMEOUT;
    r->detail = started ? r->startup_ms : 0;
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 3: Repeated Startup Attempts (10 cycles, 500 ms each)
 * ═══════════════════════════════════════════════════════════════ */
static void phase3_repeated(hse_test_result_t *r)
{
    r->phase = 3;

    int successes = 0;
    for (int attempt = 0; attempt < 10; attempt++) {
        RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
        HAL_Delay(50);

        RCC->CR |= RCC_CR_HSEON;

        uint32_t start = HAL_GetTick();
        bool started = false;
        while ((HAL_GetTick() - start) < 500) {
            if (RCC->CR & RCC_CR_HSERDY) {
                started = true;
                break;
            }
        }

        uint32_t elapsed = HAL_GetTick() - start;
        hse_cdc_printf("[HSE]   Attempt %2d/10: %s (%lu ms)\r\n",
                       attempt + 1, started ? "STARTED" : "TIMEOUT", elapsed);

        if (started) {
            successes++;
            s_hse_ever_started = true;
        }

        RCC->CR &= ~RCC_CR_HSEON;
        HAL_Delay(50);

        /* Blink LED1 for activity */
        HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
    }

    hse_cdc_printf("[HSE] Success rate: %d/10\r\n", successes);

    r->passed = (successes > 0);
    r->status = (successes > 0) ? HSE_RESULT_PASS : HSE_RESULT_FAIL;
    r->detail = (uint32_t)successes;
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 4: CSS (Clock Security System) NMI Detection
 * ═══════════════════════════════════════════════════════════════ */
static void phase4_css(hse_test_result_t *r)
{
    r->phase = 4;

    g_hse_css_nmi_fired = 0;

    /* Enable HSE (it won't start if crystal is dead) */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSHSEON);
    HAL_Delay(10);
    RCC->CR |= RCC_CR_HSEON;
    HAL_Delay(200);

    bool hse_running = (RCC->CR & RCC_CR_HSERDY) != 0;
    hse_cdc_printf("[HSE] HSE state before CSS: HSERDY=%d\r\n", hse_running ? 1 : 0);

    /* Enable CSS — if HSE is not running, CSS should fire NMI */
    hse_cdc_printf("[HSE] Enabling CSS (CSSHSEON)...\r\n");
    RCC->CR |= RCC_CR_CSSHSEON;

    /* Wait up to 1 second for NMI to fire */
    uint32_t start = HAL_GetTick();
    while (!g_hse_css_nmi_fired && (HAL_GetTick() - start) < 1000) {
        /* spin */
    }

    hse_cdc_printf("[HSE] CSS NMI fired:  %s\r\n", g_hse_css_nmi_fired ? "YES" : "NO");
    hse_cdc_printf("[HSE] HSE was running: %s\r\n", hse_running ? "YES" : "NO");

    /* Cleanup */
    RCC->CR &= ~(RCC_CR_CSSHSEON | RCC_CR_HSEON);
    HAL_Delay(10);

    /*
     * CSS test passes if:
     *  - NMI fired (CSS correctly detected missing HSE clock), OR
     *  - HSE was actually running (CSS sees valid clock, no NMI needed)
     *
     * Note: on some STM32H7 revisions, CSS only fires when HSE *was*
     * running and then stops, not when it never started. So "neither"
     * is a valid outcome that doesn't indicate a CSS hardware fault.
     */
    if (g_hse_css_nmi_fired) {
        hse_cdc_printf("[HSE] RESULT: PASS (CSS detected HSE failure)\r\n");
        r->passed = true;
        r->status = HSE_RESULT_PASS;
        r->detail = 1;
    } else if (hse_running) {
        hse_cdc_printf("[HSE] RESULT: PASS (HSE running, CSS sees valid clock)\r\n");
        r->passed = true;
        r->status = HSE_RESULT_PASS;
        r->detail = 2;
        s_hse_ever_started = true;
    } else {
        hse_cdc_printf("[HSE] RESULT: INFO (CSS did not fire — known behavior when HSE never starts)\r\n");
        r->passed = false;
        r->status = HSE_RESULT_INFO;
        r->detail = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 5: PH0/PH1 GPIO Loopback (PCB trace test)
 * ═══════════════════════════════════════════════════════════════ */
static void phase5_gpio_loopback(hse_test_result_t *r)
{
    r->phase = 5;

    /* Ensure HSE fully off */
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSHSEON);
    HAL_Delay(20);

    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* --- Test A: PH0 output, read back --- */
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &gpio);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(1);
    bool ph0_high = (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_0) == GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(1);
    bool ph0_low = (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_0) == GPIO_PIN_RESET);

    /* --- Test B: PH1 output, read back --- */
    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOH, &gpio);

    /* Revert PH0 to input for crosstalk test */
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOH, &gpio);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1);
    bool ph1_high = (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_1) == GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(1);
    bool ph1_low = (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_1) == GPIO_PIN_RESET);

    /* --- Test C: Crosstalk — PH0 output high, read PH1 --- */
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &gpio);

    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOH, &gpio);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(1);
    bool crosstalk_01 = (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_1) == GPIO_PIN_SET);

    /* --- Test D: Reverse crosstalk — PH1 output high, read PH0 --- */
    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &gpio);

    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOH, &gpio);

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(1);
    bool crosstalk_10 = (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_0) == GPIO_PIN_SET);

    /* Restore PH0/PH1 to analog (safe for future HSE use) */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &gpio);

    hse_cdc_printf("[HSE] PH0 drive high read-back: %s\r\n", ph0_high ? "OK" : "FAIL");
    hse_cdc_printf("[HSE] PH0 drive low read-back:  %s\r\n", ph0_low  ? "OK" : "FAIL");
    hse_cdc_printf("[HSE] PH1 drive high read-back: %s\r\n", ph1_high ? "OK" : "FAIL");
    hse_cdc_printf("[HSE] PH1 drive low read-back:  %s\r\n", ph1_low  ? "OK" : "FAIL");
    hse_cdc_printf("[HSE] PH0->PH1 crosstalk:       %s\r\n",
                   crosstalk_01 ? "YES (DC coupling through crystal)" : "NO (crystal is open-circuit)");
    hse_cdc_printf("[HSE] PH1->PH0 crosstalk:       %s\r\n",
                   crosstalk_10 ? "YES" : "NO");

    bool pins_ok = ph0_high && ph0_low && ph1_high && ph1_low;
    r->passed = pins_ok;
    r->status = pins_ok ? HSE_RESULT_PASS : HSE_RESULT_FAIL;
    r->detail = (uint32_t)((ph0_high << 0) | (ph0_low << 1) |
                           (ph1_high << 2) | (ph1_low << 3) |
                           (crosstalk_01 << 4) | (crosstalk_10 << 5));
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 6: Full RCC Register Dump
 * ═══════════════════════════════════════════════════════════════ */
static void phase6_rcc_dump(hse_test_result_t *r)
{
    r->phase = 6;

    /* Briefly enable HSE for register capture */
    RCC->CR |= RCC_CR_HSEON;
    HAL_Delay(200);

    hse_cdc_printf("[HSE] --- RCC Register Dump ---\r\n");
    hse_cdc_printf("[HSE] RCC->CR         = 0x%08lX\r\n", RCC->CR);
    hse_cdc_printf("[HSE]   HSION=%lu HSIRDY=%lu HSIDIV=%lu\r\n",
                   (RCC->CR >> 0) & 1, (RCC->CR >> 2) & 1, (RCC->CR >> 3) & 3);
    hse_cdc_printf("[HSE]   CSION=%lu CSIRDY=%lu\r\n",
                   (RCC->CR >> 7) & 1, (RCC->CR >> 8) & 1);
    hse_cdc_printf("[HSE]   HSEON=%lu HSERDY=%lu HSEBYP=%lu CSSHSEON=%lu\r\n",
                   (RCC->CR >> 16) & 1, (RCC->CR >> 17) & 1,
                   (RCC->CR >> 18) & 1, (RCC->CR >> 19) & 1);
    hse_cdc_printf("[HSE]   PLL1ON=%lu PLL1RDY=%lu\r\n",
                   (RCC->CR >> 24) & 1, (RCC->CR >> 25) & 1);

    hse_cdc_printf("[HSE] RCC->CFGR       = 0x%08lX\r\n", RCC->CFGR);
    hse_cdc_printf("[HSE]   SWS=%lu (0=HSI,1=CSI,2=HSE,3=PLL1)\r\n",
                   (RCC->CFGR >> 3) & 7);

    hse_cdc_printf("[HSE] RCC->PLLCKSELR  = 0x%08lX\r\n", RCC->PLLCKSELR);
    hse_cdc_printf("[HSE]   PLLSRC=%lu (0=HSI,1=CSI,2=HSE,3=none)\r\n",
                   RCC->PLLCKSELR & 3);

    hse_cdc_printf("[HSE] RCC->CIFR       = 0x%08lX\r\n", RCC->CIFR);
    hse_cdc_printf("[HSE]   HSERDYF=%lu CSSF=%lu\r\n",
                   (RCC->CIFR >> 3) & 1, (RCC->CIFR >> 8) & 1);

    hse_cdc_printf("[HSE] RCC->CIER       = 0x%08lX\r\n", RCC->CIER);

    hse_cdc_printf("[HSE] RCC->BDCR       = 0x%08lX\r\n", RCC->BDCR);
    hse_cdc_printf("[HSE]   LSEON=%lu LSERDY=%lu\r\n",
                   (RCC->BDCR >> 0) & 1, (RCC->BDCR >> 1) & 1);

    /* Chip revision and device ID */
    uint32_t dbgmcu = DBGMCU->IDCODE;
    hse_cdc_printf("[HSE] DBGMCU->IDCODE  = 0x%08lX (DEV=0x%03lX REV=0x%04lX)\r\n",
                   dbgmcu, dbgmcu & 0xFFF, (dbgmcu >> 16) & 0xFFFF);

    /* HAL clock frequencies */
    hse_cdc_printf("[HSE] SYSCLK = %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
    hse_cdc_printf("[HSE] HCLK   = %lu Hz\r\n", HAL_RCC_GetHCLKFreq());

    /* Disable HSE */
    RCC->CR &= ~RCC_CR_HSEON;

    r->passed = true;
    r->status = HSE_RESULT_INFO;
    r->detail = RCC->CR;
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 7: MCO + Timer Input Capture Frequency Measurement
 *
 *  Requires physical jumper wire: PA8 (MCO1) -> PA0 (TIM5_CH1)
 *  Only runs if HSE started in any earlier phase.
 * ═══════════════════════════════════════════════════════════════ */
static void phase7_mco_measure(hse_test_result_t *r)
{
    r->phase = 7;

    if (!s_hse_ever_started) {
        hse_cdc_printf("[HSE] SKIPPED: HSE never started in any prior phase\r\n");
        hse_cdc_printf("[HSE] (No frequency to measure)\r\n");
        r->passed = false;
        r->skipped = true;
        r->status = HSE_RESULT_SKIPPED;
        return;
    }

    hse_cdc_printf("[HSE] NOTE: Requires jumper wire PA8 -> PA0\r\n");
    hse_cdc_printf("[HSE] If no jumper is installed, this test will timeout.\r\n");

    /* 1. Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    HAL_Delay(500);
    if (!(RCC->CR & RCC_CR_HSERDY)) {
        hse_cdc_printf("[HSE] HSE failed to start for measurement\r\n");
        r->passed = false;
        r->status = HSE_RESULT_FAIL;
        RCC->CR &= ~RCC_CR_HSEON;
        return;
    }

    /* 2. Route HSE to MCO1 (PA8), /4 = 6 MHz output */
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_4);

    /* 3. Configure TIM5 CH1 (PA0, AF2) for input capture */
    __HAL_RCC_TIM5_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* TIM5: 32-bit, APB1 timer clock = 216 MHz, no prescaler */
    TIM5->PSC = 0;
    TIM5->ARR = 0xFFFFFFFF;
    TIM5->CCMR1 = TIM_CCMR1_CC1S_0;  /* CC1 mapped to TI1 */
    TIM5->CCER = TIM_CCER_CC1E;       /* Capture on rising edge */
    TIM5->SR = 0;
    TIM5->CR1 = TIM_CR1_CEN;          /* Start timer */

    /* 4. Capture first edge */
    uint32_t t0 = HAL_GetTick();
    while (!(TIM5->SR & TIM_SR_CC1IF)) {
        if ((HAL_GetTick() - t0) > 2000) {
            hse_cdc_printf("[HSE] No edges captured (no jumper wire?)\r\n");
            r->passed = false;
            r->status = HSE_RESULT_TIMEOUT;
            goto cleanup_phase7;
        }
    }
    uint32_t first_cap = TIM5->CCR1;
    TIM5->SR = ~TIM_SR_CC1IF;

    /* 5. Capture 100 more edges */
    uint32_t last_cap = first_cap;
    int edge_count = 1;
    for (int i = 0; i < 100; i++) {
        t0 = HAL_GetTick();
        while (!(TIM5->SR & TIM_SR_CC1IF)) {
            if ((HAL_GetTick() - t0) > 100) goto done_capture;
        }
        last_cap = TIM5->CCR1;
        TIM5->SR = ~TIM_SR_CC1IF;
        edge_count++;
    }

done_capture:
    if (edge_count < 2) {
        hse_cdc_printf("[HSE] Only %d edge captured, cannot measure\r\n", edge_count);
        r->passed = false;
        r->status = HSE_RESULT_FAIL;
        goto cleanup_phase7;
    }

    {
        /* Timer clock = 216 MHz */
        uint32_t ticks = last_cap - first_cap;
        uint32_t periods = (uint32_t)(edge_count - 1);
        float freq_mco = (216000000.0f * (float)periods) / (float)ticks;
        float freq_hse = freq_mco * 4.0f; /* MCO divider was /4 */
        float ppm_error = ((freq_hse - 24000000.0f) / 24000000.0f) * 1000000.0f;

        hse_cdc_printf("[HSE] Captured %d edges over %lu timer ticks\r\n", edge_count, ticks);
        hse_cdc_printf("[HSE] MCO frequency:  %.1f Hz\r\n", (double)freq_mco);
        hse_cdc_printf("[HSE] HSE frequency:  %.1f Hz (expected 24000000)\r\n", (double)freq_hse);
        hse_cdc_printf("[HSE] Error:          %.1f ppm\r\n", (double)ppm_error);

        bool freq_ok = (freq_hse > 23976000.0f) && (freq_hse < 24024000.0f);
        r->passed = freq_ok;
        r->status = freq_ok ? HSE_RESULT_PASS : HSE_RESULT_FAIL;
        r->freq_hz = freq_hse;
    }

cleanup_phase7:
    TIM5->CR1 = 0;
    __HAL_RCC_TIM5_CLK_DISABLE();
    RCC->CR &= ~RCC_CR_HSEON;

    /* Restore PA0 to TIM2_CH1 (AF1) for servo PWM */
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Restore PA8 to analog */
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* ═══════════════════════════════════════════════════════════════
 *  PHASE 8: Thermal Retry (60 seconds)
 * ═══════════════════════════════════════════════════════════════ */
static void phase8_thermal(hse_test_result_t *r)
{
    r->phase = 8;

    hse_cdc_printf("[HSE] Running 60-second thermal retry test...\r\n");
    hse_cdc_printf("[HSE] Board warms from MCU power dissipation at 432 MHz\r\n");

    int total_attempts = 0;
    int successes = 0;
    uint32_t test_start = HAL_GetTick();

    while ((HAL_GetTick() - test_start) < 60000) {
        RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
        HAL_Delay(20);

        RCC->CR |= RCC_CR_HSEON;

        uint32_t poll_start = HAL_GetTick();
        bool started = false;
        while ((HAL_GetTick() - poll_start) < 200) {
            if (RCC->CR & RCC_CR_HSERDY) {
                started = true;
                break;
            }
        }

        total_attempts++;
        if (started) {
            successes++;
            s_hse_ever_started = true;
        }

        RCC->CR &= ~RCC_CR_HSEON;
        HAL_Delay(20);

        /* Progress print every 10 seconds */
        uint32_t elapsed_sec = (HAL_GetTick() - test_start) / 1000;
        if (total_attempts % 20 == 0) {
            hse_cdc_printf("[HSE]   t=%2lus: %d/%d started so far\r\n",
                           elapsed_sec, successes, total_attempts);
        }

        /* Blink LED1 for activity */
        HAL_GPIO_TogglePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin);
    }

    hse_cdc_printf("[HSE] Thermal retry: %d/%d started (%.1f%%)\r\n",
                   successes, total_attempts,
                   total_attempts > 0 ? ((float)successes / (float)total_attempts * 100.0f) : 0.0f);

    r->passed = (successes > 0);
    r->status = (successes > 0) ? HSE_RESULT_PASS : HSE_RESULT_FAIL;
    r->detail = (uint32_t)successes;
}

/* ═══════════════════════════════════════════════════════════════
 *  Campaign Sequencer
 * ═══════════════════════════════════════════════════════════════ */

static const char *phase_names[HSE_TEST_NUM_PHASES] = {
    "Basic HSE enable (100ms)",
    "Extended timeout (2000ms)",
    "Repeated attempts (10x)",
    "CSS NMI detection",
    "PH0/PH1 GPIO loopback",
    "RCC register dump",
    "MCO frequency measure",
    "Thermal retry (60s)",
};

static const char *status_str(hse_result_status_t s)
{
    switch (s) {
    case HSE_RESULT_PASS:    return "PASS";
    case HSE_RESULT_FAIL:    return "FAIL";
    case HSE_RESULT_TIMEOUT: return "TIMEOUT";
    case HSE_RESULT_SKIPPED: return "SKIP";
    case HSE_RESULT_INFO:    return "INFO";
    default:                 return "???";
    }
}

typedef void (*phase_fn_t)(hse_test_result_t *);

void hse_test_run_all(void)
{
    /* Campaign start — 1 long tone */
    buzzer_blocking_tone(800, 500);
    HAL_Delay(200);

    hse_cdc_printf("[HSE] ============================================\r\n");
    hse_cdc_printf("[HSE] C.A.S.P.E.R.-2 HSE Crystal Test Campaign\r\n");
    hse_cdc_printf("[HSE] Build: %s %s\r\n", __DATE__, __TIME__);
    hse_cdc_printf("[HSE] SYSCLK: HSI+PLL @ %lu Hz (known-good)\r\n",
                   HAL_RCC_GetSysClockFreq());
    hse_cdc_printf("[HSE] HSE_VALUE: %lu Hz\r\n", (uint32_t)HSE_VALUE);
    hse_cdc_printf("[HSE] ============================================\r\n");

    static const phase_fn_t phases[HSE_TEST_NUM_PHASES] = {
        phase1_basic_hse,
        phase2_extended_hse,
        phase3_repeated,
        phase4_css,
        phase5_gpio_loopback,
        phase6_rcc_dump,
        phase7_mco_measure,
        phase8_thermal,
    };

    memset(s_results, 0, sizeof(s_results));

    for (int i = 0; i < HSE_TEST_NUM_PHASES; i++) {
        hse_cdc_printf("\r\n[HSE] --- Phase %d: %s ---\r\n", i + 1, phase_names[i]);

        leds_show_phase((uint8_t)(i + 1));

        /* Short beep to mark phase start */
        buzzer_blocking_tone(1200, 50);
        HAL_Delay(50);

        /* Run phase */
        phases[i](&s_results[i]);

        hse_cdc_printf("[HSE] RESULT: %s\r\n", status_str(s_results[i].status));

        /* Audio feedback */
        if (s_results[i].passed) {
            buzzer_pass_sound();
        } else if (!s_results[i].skipped) {
            buzzer_fail_sound();
        }

        /* LED result display */
        leds_show_result(s_results[i].status);
        leds_all_off();
        HAL_Delay(500);
    }

    /* ── Summary ────────────────────────────────────────────── */
    int pass_count = 0;
    hse_cdc_printf("\r\n[HSE] ============================================\r\n");
    hse_cdc_printf("[HSE] SUMMARY\r\n");
    hse_cdc_printf("[HSE] ============================================\r\n");
    for (int i = 0; i < HSE_TEST_NUM_PHASES; i++) {
        hse_cdc_printf("[HSE]   Phase %d (%s): %s\r\n",
                       i + 1, phase_names[i], status_str(s_results[i].status));
        if (s_results[i].passed) pass_count++;
    }
    hse_cdc_printf("[HSE] ============================================\r\n");
    hse_cdc_printf("[HSE] Total: %d/%d passed\r\n", pass_count, HSE_TEST_NUM_PHASES);

    /* Verdict */
    if (s_hse_ever_started) {
        /* Check if phases 1-3 all passed reliably */
        bool reliable = s_results[0].passed && s_results[1].passed &&
                        (s_results[2].detail >= 8); /* 8+/10 attempts */
        if (reliable) {
            hse_cdc_printf("[HSE] VERDICT: HSE CRYSTAL IS ALIVE AND RELIABLE\r\n");
            hse_cdc_printf("[HSE] Recommendation: Migrate to HSE+PLL for better accuracy\r\n");
            hse_cdc_printf("[HSE] Suggested PLL: PLLM=3, PLLN=108, PLLP=2, PLLQ=18\r\n");
        } else {
            hse_cdc_printf("[HSE] VERDICT: HSE CRYSTAL IS MARGINAL\r\n");
            hse_cdc_printf("[HSE] Recommendation: Crystal starts but is unreliable.\r\n");
            hse_cdc_printf("[HSE] Inspect solder joints, check load capacitors,\r\n");
            hse_cdc_printf("[HSE] or replace crystal. Continue using HSI+PLL for now.\r\n");
        }
    } else {
        bool pcb_ok = s_results[4].passed; /* Phase 5 GPIO test */
        if (pcb_ok) {
            hse_cdc_printf("[HSE] VERDICT: HSE CRYSTAL APPEARS DEAD\r\n");
            hse_cdc_printf("[HSE] PCB traces to crystal pads are OK (Phase 5 passed).\r\n");
            hse_cdc_printf("[HSE] The crystal itself is likely defective.\r\n");
            hse_cdc_printf("[HSE] Recommendation: Replace crystal or continue using HSI+PLL.\r\n");
        } else {
            hse_cdc_printf("[HSE] VERDICT: POSSIBLE PCB DEFECT\r\n");
            hse_cdc_printf("[HSE] GPIO test on PH0/PH1 FAILED — trace may be broken.\r\n");
            hse_cdc_printf("[HSE] Inspect PH0/PH1 solder joints under magnification.\r\n");
        }
    }
    hse_cdc_printf("[HSE] ============================================\r\n");

    /* Final LED: binary count of passes on LED4-LED1 */
    leds_all_off();
    if (pass_count & 0x01)
        HAL_GPIO_WritePin(CONT_YN_1_GPIO_Port, CONT_YN_1_Pin, GPIO_PIN_SET);
    if (pass_count & 0x02)
        HAL_GPIO_WritePin(CONT_YN_2_GPIO_Port, CONT_YN_2_Pin, GPIO_PIN_SET);
    if (pass_count & 0x04)
        HAL_GPIO_WritePin(CONT_YN_3_GPIO_Port, CONT_YN_3_Pin, GPIO_PIN_SET);
    if (pass_count & 0x08)
        HAL_GPIO_WritePin(CONT_YN_4_GPIO_Port, CONT_YN_4_Pin, GPIO_PIN_SET);

    /* Final buzzer */
    HAL_Delay(500);
    if (s_hse_ever_started) {
        /* Happy melody — 3 ascending tones */
        buzzer_blocking_tone(800, 150);
        HAL_Delay(50);
        buzzer_blocking_tone(1200, 150);
        HAL_Delay(50);
        buzzer_blocking_tone(1600, 300);
    } else {
        /* Sad melody — 3 descending tones */
        buzzer_blocking_tone(800, 150);
        HAL_Delay(50);
        buzzer_blocking_tone(500, 150);
        HAL_Delay(50);
        buzzer_blocking_tone(300, 300);
    }

    /* Idle loop */
    hse_cdc_printf("[HSE] Test campaign complete. Board idle.\r\n");
    while (1) {
        HAL_Delay(1000);
    }
}
