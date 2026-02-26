#ifndef APP_COMMAND_HIL_HANDLER_H
#define APP_COMMAND_HIL_HANDLER_H

#include <stdint.h>

#ifdef HIL_MODE
/**
 * Handle HIL inject command (0xD1).
 * Deserializes fsm_input_t + virtual tick from the wire,
 * sets the virtual clock, and calls flight_fsm_tick().
 */
void hil_handle_inject(const uint8_t *data, int len);
#endif

#endif /* APP_COMMAND_HIL_HANDLER_H */
