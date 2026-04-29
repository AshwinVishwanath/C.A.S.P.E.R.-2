/**
 * @file cycle_probe.h
 * @brief DWT-cycle-counter timing probes for performance diagnostics.
 *
 * Each probe accumulates count, sum, and max cycle counts over a window.
 * Call DIAG_PROBE_BEGIN/END around the code under test, then once per
 * window call diag_probe_emit() to print and reset.
 *
 * Gated on LOGGER_SANITY: zero overhead in flight builds.
 */
#ifndef APP_DIAG_CYCLE_PROBE_H
#define APP_DIAG_CYCLE_PROBE_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef LOGGER_SANITY

typedef struct {
    uint32_t count;
    uint32_t sum;
    uint32_t max;
} diag_probe_t;

/* SYSCLK = 432 MHz, so cycles / 432 ≈ µs. Update if clock changes. */
#define DIAG_CPU_HZ        432000000UL
#define DIAG_CYC_PER_US    432UL

/* One-time DWT enable. Call after SystemClock_Config in main(). */
static inline void diag_probe_init_dwt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Cortex-M7 lock register — write magic to enable CYCCNT writes. */
    DWT->LAR = 0xC5ACCE55u;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

#define DIAG_PROBE_BEGIN(P)  uint32_t _t_##P = DWT->CYCCNT
#define DIAG_PROBE_END(P)    do {                                   \
    uint32_t _d = DWT->CYCCNT - _t_##P;                              \
    (P).count++;                                                     \
    (P).sum += _d;                                                   \
    if (_d > (P).max) (P).max = _d;                                  \
} while (0)

/* Print one probe line and reset its counters.
 *
 *   [CYC] <name>: n=<count> max=<cyc> (<us>) avg=<cyc> (<us>) total=<ms>/s = <pct>% CPU
 *
 * Caller passes elapsed_ms = HAL_GetTick delta since last print so the
 * "ms/s" field can be computed correctly even if the print isn't exactly
 * 1 second apart. */
void diag_probe_emit(const char *name, diag_probe_t *p, uint32_t elapsed_ms);

#else  /* !LOGGER_SANITY */

#define DIAG_PROBE_BEGIN(P)   ((void)0)
#define DIAG_PROBE_END(P)     ((void)0)

#endif

#endif /* APP_DIAG_CYCLE_PROBE_H */
