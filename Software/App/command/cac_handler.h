#ifndef APP_COMMAND_CAC_HANDLER_H
#define APP_COMMAND_CAC_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the CAC (Command-Acknowledge-Confirm) handler.
 */
void cac_init(void);

/**
 * Periodic tick — check confirm timeout (call every superloop iteration).
 */
void cac_tick(void);

/* ── Command handlers (called by cmd_router) ──────────────────── */

void cac_handle_arm(const uint8_t *data, int len);
void cac_handle_fire(const uint8_t *data, int len);
void cac_handle_testmode(const uint8_t *data, int len);
void cac_handle_config_poll(const uint8_t *data, int len);
void cac_handle_confirm(const uint8_t *data, int len);
void cac_handle_abort(const uint8_t *data, int len);

/* ── Test mode API ────────────────────────────────────────────── */

bool cac_test_mode_active(void);
uint32_t cac_test_mode_remaining_ms(void);

#endif /* APP_COMMAND_CAC_HANDLER_H */
