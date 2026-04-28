/* ============================================================
 *  TIER:     TEST-HARNESS
 *  MODULE:   HIL Handler (raw)
 *  SUMMARY:  HIL injection of raw IMU/baro/mag bytes pre-driver scaling.
 * ============================================================ */
#ifdef HIL_MODE

#include "hil_raw_handler.h"
#include "tlm_types.h"
#include "crc32_hw.h"
#include "endian.h"
#include <string.h>

/* Single global instance — no dynamic allocation */
hil_raw_data_t g_hil_raw;

void hil_raw_handle_inject(const uint8_t *data, int len)
{
    if (len < SIZE_HIL_RAW_INJECT) return;

    /* Verify CRC (bytes 0..45, CRC at bytes 46..49) */
    uint32_t crc_received = get_le32(&data[46]);
    uint32_t crc_computed = crc32_hw_compute(data, 46);
    if (crc_received != crc_computed) return;

    /* Deserialize into global struct */
    g_hil_raw.tick_ms = get_le32(&data[1]);

    memcpy(&g_hil_raw.accel_ms2[0], &data[5],  4);
    memcpy(&g_hil_raw.accel_ms2[1], &data[9],  4);
    memcpy(&g_hil_raw.accel_ms2[2], &data[13], 4);

    memcpy(&g_hil_raw.gyro_rads[0], &data[17], 4);
    memcpy(&g_hil_raw.gyro_rads[1], &data[21], 4);
    memcpy(&g_hil_raw.gyro_rads[2], &data[25], 4);

    memcpy(&g_hil_raw.baro_pa,      &data[29], 4);

    memcpy(&g_hil_raw.mag_ut[0],    &data[33], 4);
    memcpy(&g_hil_raw.mag_ut[1],    &data[37], 4);
    memcpy(&g_hil_raw.mag_ut[2],    &data[41], 4);

    uint8_t flags = data[45];
    g_hil_raw.baro_valid = (flags & 0x01) != 0;
    g_hil_raw.mag_valid  = (flags & 0x02) != 0;
    g_hil_raw.skip_cal   = (flags & 0x04) != 0;

    /* Signal flight_loop that new data is ready */
    g_hil_raw.pending = true;
}

#endif /* HIL_MODE */
