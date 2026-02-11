/**
 * @file    mag_cal.h
 * @brief   Magnetometer calibration data collection for C.A.S.P.E.R.-2
 *
 * Logs MMC5983MA readings to MAG_CAL.CSV on QSPI flash while tracking
 * sphere coverage to auto-detect when enough orientations have been sampled.
 * Data is post-processed in MATLAB for ellipsoid-fitting hard/soft-iron cal.
 */
#ifndef MAG_CAL_H
#define MAG_CAL_H

#include "stm32h7xx_hal.h"
#include "mmc5983ma.h"
#include "ff.h"
#include <stdbool.h>
#include <stdint.h>

/* Sphere coverage grid: 20-degree bins */
#define MAG_CAL_AZ_BINS    18   /* 360 / 20 */
#define MAG_CAL_EL_BINS     9   /* 180 / 20 */
#define MAG_CAL_TOTAL_BINS (MAG_CAL_AZ_BINS * MAG_CAL_EL_BINS)  /* 162 */
#define MAG_CAL_TARGET_PCT  85
#define MAG_CAL_DRAIN_MS    5000  /* continue logging 5s after target reached */

typedef enum {
    MAG_CAL_IDLE,
    MAG_CAL_COLLECTING,
    MAG_CAL_DRAINING,
    MAG_CAL_DONE
} mag_cal_state_t;

typedef struct {
    /* State machine */
    mag_cal_state_t state;

    /* Sphere coverage tracking */
    uint8_t  bins[MAG_CAL_AZ_BINS][MAG_CAL_EL_BINS];
    uint16_t bins_filled;
    uint8_t  coverage_pct;

    /* FATFS */
    FIL      file;
    bool     file_open;

    /* Write buffer (batch writes to flash) */
    char     wbuf[2048];
    uint16_t wbuf_pos;

    /* Timing */
    uint32_t drain_start_ms;
    uint32_t last_progress_ms;
    uint32_t sample_count;
    uint8_t  last_pct_printed;
} mag_cal_t;

/**
 * @brief  Open MAG_CAL.CSV, write header, begin collecting.
 * @return true if file opened successfully
 */
bool mag_cal_init(mag_cal_t *cal);

/**
 * @brief  Process one mag sample: log to CSV, update sphere coverage,
 *         report progress via CDC + LEDs.
 *         Call at 100 Hz after mmc5983ma_read().
 */
void mag_cal_tick(mag_cal_t *cal, const mmc5983ma_t *mag, uint32_t now_ms);

/**
 * @brief  Check if calibration collection is finished.
 */
bool mag_cal_is_done(const mag_cal_t *cal);

/* ── Reusable calibration correction (for flight software) ───────────────── */

/**
 * @brief  Apply hard-iron and soft-iron calibration to raw mag readings.
 *         cal = W * (raw - b)
 *         where W is the 3x3 soft-iron matrix and b is the hard-iron vector.
 * @param  raw_ut   Input:  frame-mapped raw field in µT  [3]
 * @param  cal_ut   Output: calibrated field in µT        [3]
 */
void mag_cal_apply(const float raw_ut[3], float cal_ut[3]);

/** Expected field magnitude after calibration (µT) */
#define MAG_CAL_EXPECTED_MAG  40.1800f

#endif /* MAG_CAL_H */
