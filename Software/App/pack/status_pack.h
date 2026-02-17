#ifndef APP_PACK_STATUS_PACK_H
#define APP_PACK_STATUS_PACK_H

#include <stdint.h>
#include <stdbool.h>
#include "tlm_types.h"

/**
 * Build the 2-byte FC_TLM_STATUS bitmap.
 *
 * Byte 0: [ARM4..ARM1][CNT4..CNT1]
 *   b7: ARM4    b3: CNT4
 *   b6: ARM3    b2: CNT3
 *   b5: ARM2    b1: CNT2
 *   b4: ARM1    b0: CNT1
 *
 * Byte 1: [ST3..ST0][FIRED][ERROR][RSVD][RSVD]
 *   b7-b4: FSM state (4-bit)
 *   b3:    FIRED (any pyro fired this tick)
 *   b2:    ERROR (system error active)
 *   b1-b0: Reserved (0)
 *
 * @param out       Output buffer, exactly 2 bytes
 * @param pyro      Pyro state (arm + continuity)
 * @param fsm_state Current FSM state (0x0–0xB)
 * @param sys_error System error flag
 */
void status_pack_build(uint8_t out[2], const pyro_state_t *pyro,
                       fsm_state_t fsm_state, bool sys_error);

#endif /* APP_PACK_STATUS_PACK_H */
