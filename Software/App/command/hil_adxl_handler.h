/* ============================================================
 *  TIER:     TEST-HARNESS
 *  MODULE:   HIL ADXL Handler
 *  SUMMARY:  Receives 0xD5 packets carrying ADXL372 high-G
 *            samples (one per packet). HIL_MODE only.
 * ============================================================ */
#ifndef APP_COMMAND_HIL_ADXL_HANDLER_H
#define APP_COMMAND_HIL_ADXL_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef HIL_MODE

/* One ADXL sample. raw_xyz are int16 left-justified 12-bit
 * (matches what `adxl372_read()` produces on real hardware). */
typedef struct {
    uint32_t tick_ms;
    int16_t  raw_ax;
    int16_t  raw_ay;
    int16_t  raw_az;
    uint8_t  flags;
    volatile bool pending;
} hil_adxl_sample_t;

extern hil_adxl_sample_t g_hil_adxl;

/**
 * Deserialise a 0xD5 packet into g_hil_adxl.
 * Validates length and CRC; silently drops malformed frames.
 */
void hil_adxl_handle_inject(const uint8_t *data, int len);

#endif /* HIL_MODE */

#endif /* APP_COMMAND_HIL_ADXL_HANDLER_H */
