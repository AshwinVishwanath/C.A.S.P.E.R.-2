/* ============================================================
 *  TIER:     TEST-HARNESS
 *  MODULE:   HIL Aux Handler
 *  SUMMARY:  Deserialises 0xD4 packets into g_hil_aux. HIL_MODE only.
 * ============================================================ */
#ifdef HIL_MODE

#include "hil_aux_handler.h"
#include "tlm_types.h"
#include "crc32_hw.h"
#include "endian.h"
#include <string.h>

/* Defaults are chosen so a host that never sends 0xD4 still gets a
 * sensible-looking flight: all four pyro channels show continuity and
 * the battery reads ~7.4 V. Hosts that DO send 0xD4 override these. */
hil_aux_data_t g_hil_aux = {
    .cont_bitmap = 0x0Fu,
    .batt_v_x100 = 740u,
};

void hil_aux_handle_inject(const uint8_t *data, int len)
{
    if (len < SIZE_HIL_AUX) return;

    /* CRC over bytes 0..28; CRC stored at bytes 29..32. */
    uint32_t crc_received = get_le32(&data[29]);
    uint32_t crc_computed = crc32_hw_compute(data, 29);
    if (crc_received != crc_computed) return;

    g_hil_aux.tick_ms = get_le32(&data[1]);

    int32_t dlat = (int32_t)get_le32(&data[5]);
    int32_t dlon = (int32_t)get_le32(&data[9]);
    g_hil_aux.gps_dlat_mm = dlat;
    g_hil_aux.gps_dlon_mm = dlon;

    memcpy(&g_hil_aux.gps_alt_msl_m, &data[13], 4);
    memcpy(&g_hil_aux.gps_vel_d_mps, &data[17], 4);

    g_hil_aux.gps_fix       = data[21];
    g_hil_aux.gps_sat       = data[22];
    g_hil_aux.cont_bitmap   = data[23];
    g_hil_aux.batt_v_x100   = get_le16(&data[24]);
    g_hil_aux.adxl_activity = data[26];
    g_hil_aux.flags         = get_le16(&data[27]);

    g_hil_aux.pending = true;
}

#endif /* HIL_MODE */
