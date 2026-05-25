/**
 * @file    mag_noise.h
 * @brief   Magnetometer noise characterization logger for C.A.S.P.E.R.-2
 *
 * Logs MMC5983MA + LSM6DSO32 + accel-gyro-only reference quaternion to
 * MAG_NOISE.CSV on QSPI flash for ~10 minutes, with simultaneous radio TX
 * and flash-write interference markers. Output feeds MATLAB Allan-variance
 * + EMI-delta analysis to populate the Simulink mag noise / radio
 * interference models.
 *
 * Run profile is selected at compile time by defining exactly one of:
 *   -DMAG_NOISE_RUN_A  Radio OFF, normal FATFS flushes        (noise floor)
 *   -DMAG_NOISE_RUN_B  Radio Profile A (SF7),   10 Hz TX      (typical EMI)
 *   -DMAG_NOISE_RUN_C  Radio Profile B (SF8 hi-power), 10 Hz  (worst-case)
 *   -DMAG_NOISE_RUN_D  Radio OFF, forced FATFS flush 50 ms    (flash EMI)
 *
 * The Makefile exposes a shortcut: make MAG_NOISE_RUN=A|B|C|D
 */
#ifndef MAG_NOISE_H
#define MAG_NOISE_H

#include "stm32h7xx_hal.h"
#include "mmc5983ma.h"
#include "lsm6dso32.h"
#include "casper_attitude.h"
#include "ff.h"
#include <stdbool.h>
#include <stdint.h>

/* Compile-time sanity: exactly one run must be selected. */
#if defined(MAG_NOISE_RUN_A) + defined(MAG_NOISE_RUN_B) + \
    defined(MAG_NOISE_RUN_C) + defined(MAG_NOISE_RUN_D) != 1
#error "MAG_NOISE requires exactly one of MAG_NOISE_RUN_A..D"
#endif

/* Default capture duration: 10 minutes */
#ifndef MAG_NOISE_DURATION_MS
#define MAG_NOISE_DURATION_MS  600000UL
#endif

/* Forced flash-flush period for Run D (ms) */
#ifndef MAG_NOISE_BURST_PERIOD_MS
#define MAG_NOISE_BURST_PERIOD_MS  50UL
#endif

typedef enum {
    MAG_NOISE_IDLE,
    MAG_NOISE_COLLECTING,
    MAG_NOISE_DONE
} mag_noise_state_t;

typedef struct {
    mag_noise_state_t state;

    /* FATFS */
    FIL      file;
    bool     file_open;

    /* Write buffer (sized so a single line never straddles the flush) */
    char     wbuf[2048];
    uint16_t wbuf_pos;

    /* Timing */
    uint32_t start_ms;
    uint32_t last_progress_ms;
    uint32_t last_burst_flush_ms;
    uint32_t sample_count;
    uint8_t  last_pct_printed;
} mag_noise_t;

/**
 * @brief  Open MAG_NOISE.CSV, write header, set start time.
 * @return true if file opened successfully
 */
bool mag_noise_init(mag_noise_t *cap);

/**
 * @brief  Log one sample row. Call at MMC5983MA DRDY rate (100 Hz).
 *         @p radio_tx_active and @p qspi_busy are sampled at log time.
 */
void mag_noise_tick(mag_noise_t *cap,
                    const mmc5983ma_t *mag,
                    const lsm6dso32_t *imu,
                    const casper_attitude_t *att,
                    bool radio_tx_active,
                    bool qspi_busy,
                    uint16_t radio_tx_count,
                    uint32_t now_ms);

/**
 * @brief  Force a partial flush of the write buffer to flash. Used by
 *         Run D to drive realistic flash-write EMI cadence.
 */
void mag_noise_force_flush(mag_noise_t *cap, uint32_t now_ms);

/**
 * @brief  True once the capture window has elapsed and the file is closed.
 */
bool mag_noise_is_done(const mag_noise_t *cap);

#endif /* MAG_NOISE_H */
