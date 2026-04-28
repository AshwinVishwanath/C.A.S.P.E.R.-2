#ifndef APP_HSE_TEST_HSE_TEST_H
#define APP_HSE_TEST_HSE_TEST_H

#include <stdint.h>
#include <stdbool.h>

#define HSE_TEST_NUM_PHASES  8

typedef enum {
    HSE_RESULT_FAIL    = 0,
    HSE_RESULT_PASS    = 1,
    HSE_RESULT_TIMEOUT = 2,
    HSE_RESULT_SKIPPED = 3,
    HSE_RESULT_INFO    = 4,
} hse_result_status_t;

typedef struct {
    uint8_t              phase;
    hse_result_status_t  status;
    bool                 passed;
    bool                 skipped;
    uint32_t             detail;
    uint32_t             startup_ms;
    float                freq_hz;
} hse_test_result_t;

/**
 * Run the complete HSE crystal test campaign.
 *
 * Boots on HSI+PLL (known-good), runs 8 sequential test phases,
 * prints diagnostics via USB CDC, shows status on LEDs,
 * plays buzzer feedback.
 *
 * Does NOT return — enters idle loop after completion.
 */
void hse_test_run_all(void);

#endif /* APP_HSE_TEST_HSE_TEST_H */
