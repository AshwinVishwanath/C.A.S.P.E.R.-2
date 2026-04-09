#ifndef APP_COMMAND_HIL_RAW_HANDLER_H
#define APP_COMMAND_HIL_RAW_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef HIL_MODE

/* Injected raw sensor data (written by HIL handler, read by flight_loop) */
typedef struct {
    float    accel_ms2[3];   /* body frame, m/s^2          */
    float    gyro_rads[3];   /* body frame, rad/s           */
    float    baro_pa;        /* pressure in Pa              */
    float    mag_ut[3];      /* calibrated mag, body, uT    */
    uint32_t tick_ms;        /* virtual timestamp           */
    bool     baro_valid;     /* baro reading present        */
    bool     mag_valid;      /* mag reading present         */
    bool     skip_cal;       /* skip 30s calibration        */
    volatile bool pending;   /* new data available          */
} hil_raw_data_t;

extern hil_raw_data_t g_hil_raw;

/**
 * Handle raw sensor HIL inject command (0xD3).
 * Deserializes raw sensor data from the wire into g_hil_raw.
 */
void hil_raw_handle_inject(const uint8_t *data, int len);

#endif /* HIL_MODE */

#endif /* APP_COMMAND_HIL_RAW_HANDLER_H */
