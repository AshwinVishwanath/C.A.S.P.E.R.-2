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
#include "casper_port.h"

#ifdef LOGGER_SANITY

typedef struct {
    uint32_t count;
    uint32_t sum;   /* accumulated µs */
    uint32_t max;   /* peak µs        */
} diag_probe_t;

/* Probes now record microseconds via casper_micros() so the board layer
 * owns the SYSCLK constant — no hardcoded 432 MHz here. */
#define DIAG_PROBE_BEGIN(P)  uint32_t _t_##P = casper_micros()
#define DIAG_PROBE_END(P)    do {                                   \
    uint32_t _d = casper_micros() - _t_##P;                          \
    (P).count++;                                                     \
    (P).sum += _d;                                                   \
    if (_d > (P).max) (P).max = _d;                                  \
} while (0)

/* Print one probe line and reset its counters.
 *
 *   [CYC] <name>: n=<count> max=<cyc> (<us>) avg=<cyc> (<us>) total=<ms>/s = <pct>% CPU
 *
 * Caller passes elapsed_ms = casper_millis() delta since last print so the
 * "ms/s" field can be computed correctly even if the print isn't exactly
 * 1 second apart. */
void diag_probe_emit(const char *name, diag_probe_t *p, uint32_t elapsed_ms);

#else  /* !LOGGER_SANITY */

#define DIAG_PROBE_BEGIN(P)   ((void)0)
#define DIAG_PROBE_END(P)     ((void)0)

#endif

#endif /* APP_DIAG_CYCLE_PROBE_H */
