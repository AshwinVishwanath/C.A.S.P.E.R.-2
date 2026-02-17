#ifndef APP_DIAG_SELF_TEST_H
#define APP_DIAG_SELF_TEST_H

#include <stdint.h>
#include "tlm_types.h"

/**
 * Run all self-tests and fill results array.
 *
 * @param results  Array of diag_result_t to fill (must hold at least 7 entries)
 * @return         Number of tests executed
 */
int self_test_run_all(diag_result_t *results);

/**
 * Run self-tests and send diagnostic response (0xC2) over USB CDC.
 *
 * @return  1 if response sent, 0 if TX busy
 */
int self_test_run_and_send(void);

#endif /* APP_DIAG_SELF_TEST_H */
