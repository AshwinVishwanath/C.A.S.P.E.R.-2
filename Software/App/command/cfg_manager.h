#ifndef APP_COMMAND_CFG_MANAGER_H
#define APP_COMMAND_CFG_MANAGER_H

#include <stdint.h>
#include "tlm_types.h"

/**
 * Initialize the config manager. Loads config from flash if present.
 */
void cfg_manager_init(void);

/**
 * Handle config upload from MC (0xC1).
 */
void cfg_handle_upload(const uint8_t *data, int len);

/**
 * Handle log read request from MC (0xC3).
 */
void cfg_handle_readlog(const uint8_t *data, int len);

/**
 * Handle log erase request from MC (0xC4).
 */
void cfg_handle_eraselog(const uint8_t *data, int len);

/**
 * Get hash of active config. Returns 0 if no config loaded.
 */
uint32_t cfg_get_active_hash(void);

/**
 * Get pointer to active flight config.
 */
const flight_config_t *cfg_get_active(void);

#endif /* APP_COMMAND_CFG_MANAGER_H */
