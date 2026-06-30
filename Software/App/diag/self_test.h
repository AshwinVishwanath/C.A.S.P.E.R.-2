/* ============================================================
 *  TIER:     SAFETY-CRITICAL
 *  MODULE:   Self Test
 *  SUMMARY:  Power-on integrity checks; gates entry to ARMED.
 * ============================================================ */
#ifndef APP_DIAG_SELF_TEST_H
#define APP_DIAG_SELF_TEST_H

#include <stdint.h>
#include "tlm_types.h"

/**
 * Run self-tests and send diagnostic response (0xC2) over USB CDC.
 *
 * @return  1 if response sent, 0 if TX busy
 */
int self_test_run_and_send(void);

#endif /* APP_DIAG_SELF_TEST_H */
