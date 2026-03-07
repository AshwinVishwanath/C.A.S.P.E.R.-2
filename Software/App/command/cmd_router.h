#ifndef APP_COMMAND_CMD_ROUTER_H
#define APP_COMMAND_CMD_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize the command router.
 */
void cmd_router_init(void);

/**
 * Process incoming CDC bytes. Call every superloop iteration.
 * Drains the CDC ring buffer, accumulates COBS frames,
 * and dispatches complete frames to the appropriate handler.
 */
void cmd_router_process(void);

/**
 * Flash dump request flag (set by 0xD2 command, cleared by flight_loop).
 */
bool cmd_router_dump_requested(void);
void cmd_router_dump_clear(void);

#endif /* APP_COMMAND_CMD_ROUTER_H */
