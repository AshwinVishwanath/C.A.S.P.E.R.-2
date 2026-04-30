/* ============================================================
 *  TIER:     TEST-HARNESS
 *  MODULE:   HIL ADXL Handler
 *  SUMMARY:  Deserialises 0xD5 packets into g_hil_adxl. HIL_MODE only.
 * ============================================================ */
#ifdef HIL_MODE

#include "hil_adxl_handler.h"
#include "tlm_types.h"
#include "crc32_hw.h"
#include "endian.h"

hil_adxl_sample_t g_hil_adxl;

void hil_adxl_handle_inject(const uint8_t *data, int len)
{
    if (len < SIZE_HIL_ADXL) return;

    /* CRC over bytes 0..11; CRC stored at bytes 12..15. */
    uint32_t crc_received = get_le32(&data[12]);
    uint32_t crc_computed = crc32_hw_compute(data, 12);
    if (crc_received != crc_computed) return;

    g_hil_adxl.tick_ms = get_le32(&data[1]);
    g_hil_adxl.raw_ax  = (int16_t)get_le16(&data[5]);
    g_hil_adxl.raw_ay  = (int16_t)get_le16(&data[7]);
    g_hil_adxl.raw_az  = (int16_t)get_le16(&data[9]);
    g_hil_adxl.flags   = data[11];

    g_hil_adxl.pending = true;
}

#endif /* HIL_MODE */
