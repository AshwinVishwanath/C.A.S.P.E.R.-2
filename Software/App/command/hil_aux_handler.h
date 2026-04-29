/* ============================================================
 *  TIER:     TEST-HARNESS
 *  MODULE:   HIL Aux Handler
 *  SUMMARY:  Receives 0xD4 packets carrying low-rate signals
 *            (GPS, pyro continuity, battery, ADXL activity).
 *            HIL_MODE only.
 * ============================================================ */
#ifndef APP_COMMAND_HIL_AUX_HANDLER_H
#define APP_COMMAND_HIL_AUX_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef HIL_MODE

/* Bit positions in the FLAGS field of the wire packet. */
#define HIL_AUX_FLAG_GPS_VALID  0x0001u

/* Latest aux signals injected from the host. Updated atomically
 * once per 0xD4 packet; consumers read whatever is most recent. */
typedef struct {
    uint32_t tick_ms;          /* virtual timestamp                          */
    int32_t  gps_dlat_mm;      /* latitude offset from pad, millimetres      */
    int32_t  gps_dlon_mm;      /* longitude offset from pad, millimetres     */
    float    gps_alt_msl_m;    /* MSL altitude, metres                       */
    float    gps_vel_d_mps;    /* NED-down velocity, m/s (positive = down)   */
    uint8_t  gps_fix;          /* 0 = no fix, 2 = 2D, 3 = 3D                 */
    uint8_t  gps_sat;          /* satellites used                            */
    uint8_t  cont_bitmap;      /* bits 0..3 = pyro continuity ch1..4         */
    uint16_t batt_v_x100;      /* battery voltage × 100                      */
    uint8_t  adxl_activity;    /* ADXL372 wake-up / activity flag            */
    uint16_t flags;            /* HIL_AUX_FLAG_*                             */
    volatile bool pending;     /* set by handler, cleared by flight_loop     */
} hil_aux_data_t;

extern hil_aux_data_t g_hil_aux;

/* Returns battery voltage as float (0..655.35 V). */
static inline float hil_aux_get_batt_v(void)
{
    return (float)g_hil_aux.batt_v_x100 * 0.01f;
}

/* Returns continuity for channel 0..3 (1-indexed channels: ch1 = bit 0). */
static inline bool hil_aux_get_continuity(uint8_t ch_zero_idx)
{
    if (ch_zero_idx >= 4) return false;
    return (g_hil_aux.cont_bitmap & (1u << ch_zero_idx)) != 0u;
}

static inline bool hil_aux_gps_valid(void)
{
    return (g_hil_aux.flags & HIL_AUX_FLAG_GPS_VALID) != 0u;
}

/**
 * Deserialise a 0xD4 packet (HIL aux) into g_hil_aux.
 * Validates length and CRC; silently drops malformed frames.
 */
void hil_aux_handle_inject(const uint8_t *data, int len);

#endif /* HIL_MODE */

#endif /* APP_COMMAND_HIL_AUX_HANDLER_H */
