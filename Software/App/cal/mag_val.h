/**
 * @file    mag_val.h
 * @brief   Magnetometer calibration validation data collection for C.A.S.P.E.R.-2
 *
 * Logs both raw and calibration-corrected MMC5983MA readings to MAG_VAL.CSV
 * on QSPI flash for a fixed 30-second window (stationary board).
 * Data is post-processed in MATLAB to compare raw vs corrected magnitude consistency.
 */
#ifndef MAG_VAL_H
#define MAG_VAL_H

#include "stm32h7xx_hal.h"
#include "mmc5983ma.h"
#include "mag_cal.h"
#include "ff.h"
#include <stdbool.h>
#include <stdint.h>

#define MAG_VAL_DURATION_MS  30000  /* 30 seconds of stationary collection */

typedef enum {
    MAG_VAL_IDLE,
    MAG_VAL_COLLECTING,
    MAG_VAL_DONE
} mag_val_state_t;

typedef struct {
    mag_val_state_t state;

    /* FATFS */
    FIL      file;
    bool     file_open;

    /* Write buffer */
    char     wbuf[2048];
    uint16_t wbuf_pos;

    /* Timing */
    uint32_t start_ms;
    uint32_t last_progress_ms;
    uint32_t sample_count;
    uint8_t  last_pct_printed;
} mag_val_t;

bool mag_val_init(mag_val_t *val);
void mag_val_tick(mag_val_t *val, const mmc5983ma_t *mag, uint32_t now_ms);
bool mag_val_is_done(const mag_val_t *val);

#endif /* MAG_VAL_H */
