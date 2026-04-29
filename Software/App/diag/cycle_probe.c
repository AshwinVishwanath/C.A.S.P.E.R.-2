/**
 * @file cycle_probe.c
 * @brief DWT cycle-counter probe implementation (LOGGER_SANITY only).
 */
#include "cycle_probe.h"

#ifdef LOGGER_SANITY

#include <stdio.h>
#include "usbd_cdc_if.h"

void diag_probe_emit(const char *name, diag_probe_t *p, uint32_t elapsed_ms)
{
    char buf[160];
    if (p->count == 0 || elapsed_ms == 0) {
        int n = snprintf(buf, sizeof(buf),
            "[CYC] %-9s n=0\r\n", name);
        if (n > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
    } else {
        uint32_t avg = p->sum / p->count;
        uint32_t max_us = p->max / DIAG_CYC_PER_US;
        uint32_t avg_us = avg / DIAG_CYC_PER_US;
        /* total active time in micro-seconds across the window: */
        uint32_t total_us = p->sum / DIAG_CYC_PER_US;
        /* Elapsed window in micro-seconds. */
        uint32_t window_us = elapsed_ms * 1000U;
        /* CPU % to one decimal place: total_us / window_us * 100, ×10 then /10. */
        uint32_t pct_x10 = (window_us > 0)
            ? (uint32_t)(((uint64_t)total_us * 1000U) / window_us)
            : 0U;
        int n = snprintf(buf, sizeof(buf),
            "[CYC] %-9s n=%lu max=%lu (%luus) avg=%lu (%luus) "
            "total=%lu.%lums = %lu.%lu%%\r\n",
            name,
            (unsigned long)p->count,
            (unsigned long)p->max,  (unsigned long)max_us,
            (unsigned long)avg,     (unsigned long)avg_us,
            (unsigned long)(total_us / 1000U),
            (unsigned long)((total_us % 1000U) / 100U),
            (unsigned long)(pct_x10 / 10U),
            (unsigned long)(pct_x10 % 10U));
        if (n > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
    }
    /* Reset for next window. */
    p->count = 0;
    p->sum   = 0;
    p->max   = 0;
}

#endif
