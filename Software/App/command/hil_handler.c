#ifdef HIL_MODE

#include "hil_handler.h"
#include "flight_fsm.h"
#include "fsm_util.h"
#include "fsm_types.h"
#include "tlm_types.h"
#include "crc32_hw.h"
#include "endian.h"
#include <string.h>

/* Forward declaration */
extern int tlm_queue_event(uint8_t type, uint16_t data);

void hil_handle_inject(const uint8_t *data, int len)
{
    if (len < SIZE_HIL_INJECT) return;

    /* Verify CRC (bytes 0..39, CRC at bytes 40..43) */
    uint32_t crc_received = get_le32(&data[40]);
    uint32_t crc_computed = crc32_hw_compute(data, 40);
    if (crc_received != crc_computed) return;

    /* Deserialize */
    uint32_t tick_ms = get_le32(&data[1]);
    fsm_input_t in = {0};

    memcpy(&in.alt_m,               &data[5],  4);
    memcpy(&in.vel_mps,             &data[9],  4);
    memcpy(&in.vert_accel_g,        &data[13], 4);
    in.antenna_up = data[17] != 0;
    memcpy(&in.flight_time_s,       &data[18], 4);
    memcpy(&in.main_deploy_alt_m,   &data[22], 4);
    memcpy(&in.drogue_fail_vel_mps, &data[26], 4);
    memcpy(&in.drogue_fail_time_s,  &data[30], 4);
    in.apogee_pyro_ch    = data[34];
    in.main_pyro_ch      = data[35];
    in.apogee_fire_dur_ms = get_le16(&data[36]);
    in.main_fire_dur_ms   = get_le16(&data[38]);

    /* Set virtual clock and tick FSM */
    fsm_set_tick(tick_ms);
    flight_fsm_tick(&in);
}

#endif /* HIL_MODE */
