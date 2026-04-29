/**
 * @file logger_sanity.h
 * @brief CDC-triggered bench harness that runs the flight logger for ~5 s.
 */
#ifndef APP_DIAG_LOGGER_SANITY_H
#define APP_DIAG_LOGGER_SANITY_H

#include <stdint.h>
#include "flight_logger.h"

void logger_sanity_init(flight_logger_t *log);
void logger_sanity_tick(uint32_t now_ms);

#endif /* APP_DIAG_LOGGER_SANITY_H */
