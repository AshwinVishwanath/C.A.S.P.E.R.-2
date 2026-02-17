#ifndef APP_COMMAND_CMD_ROUTER_H
#define APP_COMMAND_CMD_ROUTER_H

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

#endif /* APP_COMMAND_CMD_ROUTER_H */
