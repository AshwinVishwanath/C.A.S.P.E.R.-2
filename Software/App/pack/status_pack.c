#include "status_pack.h"

void status_pack_build(uint8_t out[2], const pyro_state_t *pyro,
                       fsm_state_t fsm_state, bool sys_error)
{
    uint8_t byte0 = 0;
    uint8_t byte1 = 0;

    /* Byte 0: [ARM4..ARM1][CNT4..CNT1] */
    for (int i = 0; i < PYRO_MGR_NUM_CHANNELS; i++) {
        if (pyro->armed[i]) {
            byte0 |= (uint8_t)(1u << (4 + i));
        }
        if (pyro->continuity[i]) {
            byte0 |= (uint8_t)(1u << i);
        }
    }

    /* Byte 1: [ST3..ST0][FIRED][ERROR][0][0] */
    byte1 = (uint8_t)((fsm_state & 0x0F) << 4);
    if (pyro->fired) {
        byte1 |= (1u << 3);
    }
    if (sys_error) {
        byte1 |= (1u << 2);
    }

    out[0] = byte0;
    out[1] = byte1;
}
