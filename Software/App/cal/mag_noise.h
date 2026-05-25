/**
 * @file    mag_noise.h
 * @brief   Magnetometer noise + radio/flash EMI characterization sequencer
 *
 * Single-flash autonomous capture: build with -DMAG_NOISE, flash, leave the
 * rocket on the bench. The firmware walks all four runs back-to-back:
 *
 *   Run A  (10 min)  Radio OFF, normal FATFS flushes      -> MAG_A.CSV
 *   1 beep
 *   Run B  (10 min)  Radio Profile A (SF7), 10 Hz TX      -> MAG_B.CSV + EVT_B.CSV
 *   2 beeps
 *   Run C  (10 min)  Radio Profile B (SF8 hi-pwr), 10 Hz  -> MAG_C.CSV + EVT_C.CSV
 *   3 beeps
 *   Run D  (10 min)  Radio OFF, forced 50 ms flash bursts -> MAG_D.CSV
 *   4 short beeps + 1 long beep
 *   All 4 LEDs solid, idle forever
 *
 * After completion, reflash USB_MODE=2, mount as USB MSC, pull the 6 CSVs.
 *
 * The mag CSV row matches MAG_NOISE.CSV from the earlier single-run mode.
 * EVT_*.CSV has one row per radio TX burst, sub-ms accurate start/end.
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

/* Per-run duration (ms). 10 min default — change here if you want longer. */
#ifndef MAG_NOISE_RUN_DURATION_MS
#define MAG_NOISE_RUN_DURATION_MS  600000UL
#endif

/* Forced flash-flush period for Run D (ms) */
#ifndef MAG_NOISE_BURST_PERIOD_MS
#define MAG_NOISE_BURST_PERIOD_MS  50UL
#endif

/* Settling gap inserted between the last beep and the next run start. */
#ifndef MAG_NOISE_GAP_MS
#define MAG_NOISE_GAP_MS  1000UL
#endif

/* Buzzer beep timing */
#define MAG_NOISE_BEEP_DUTY_PCT       50U
#define MAG_NOISE_BEEP_ON_MS          200U
#define MAG_NOISE_BEEP_PERIOD_MS      400U   /* 200 on + 200 off */
#define MAG_NOISE_FINAL_LONG_MS       2000U

/* Which run is currently active or just finished. */
typedef enum {
    MAG_NOISE_RUN_A = 0,
    MAG_NOISE_RUN_B = 1,
    MAG_NOISE_RUN_C = 2,
    MAG_NOISE_RUN_D = 3,
    MAG_NOISE_RUN_NONE = 4
} mag_noise_run_t;

/* Sequencer phase */
typedef enum {
    MN_PHASE_RUN,           /* actively collecting samples for current_run */
    MN_PHASE_BEEP_GAP,      /* run finished, playing N beeps then GAP_MS   */
    MN_PHASE_BEEP_FINAL,    /* 4 short beeps after Run D                   */
    MN_PHASE_BEEP_LONG,     /* the final 2-second tone                     */
    MN_PHASE_DONE           /* idle forever, LEDs solid                    */
} mn_phase_t;

typedef struct {
    /* Sequencer */
    mn_phase_t      phase;
    mag_noise_run_t current_run;     /* meaningful in MN_PHASE_RUN          */
    uint32_t        run_start_ms;    /* HAL_GetTick at start of current run */
    uint32_t        phase_start_ms;  /* for inter-phase settle timing       */
    bool            beeps_started;   /* did we already queue beeps?         */

    /* FATFS — mag samples (one file per run, opened/closed per phase) */
    FIL      file;
    bool     file_open;
    char     wbuf[2048];
    uint16_t wbuf_pos;
    uint32_t sample_count;

    /* FATFS — radio TX events (only for runs B & C) */
    FIL      evt_file;
    bool     evt_file_open;
    char     evt_wbuf[512];
    uint16_t evt_wbuf_pos;
    uint32_t evt_count;

    /* Run D burst-flush timer */
    uint32_t last_burst_flush_ms;

    /* Progress reporting */
    uint32_t last_progress_ms;
    uint8_t  last_pct_printed;
} mag_noise_t;

/**
 * @brief  Initialize the sequencer and open Run A's CSV. Call once after
 *         all sensor inits. Returns false if MAG_A.CSV cannot be opened.
 */
bool mag_noise_init(mag_noise_t *cap);

/**
 * @brief  Log one mag sample row. Call at MMC5983MA DRDY rate (100 Hz).
 *         No-op when the sequencer is between runs or done.
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
 * @brief  Advance the sequencer state machine. Call every superloop
 *         iteration. Handles run-end transitions, beeps, file rollover.
 */
void mag_noise_sequencer_tick(mag_noise_t *cap, uint32_t now_ms);

/**
 * @brief  Force a partial flush of the mag write buffer to flash. Used by
 *         Run D to drive realistic flash-write EMI cadence.
 */
void mag_noise_force_flush(mag_noise_t *cap, uint32_t now_ms);

/**
 * @brief  Drain pending radio TX events and append them to EVT_<run>.CSV.
 *         Call every superloop iteration. Cheap when the ring is empty.
 */
void mag_noise_drain_tx_events(mag_noise_t *cap);

/** Which run is the sequencer currently collecting? (MAG_NOISE_RUN_NONE
 *  if we're between runs or finished.) */
mag_noise_run_t mag_noise_current_run(const mag_noise_t *cap);

/** True once all 4 runs have completed and final beeps played out. */
bool mag_noise_is_done(const mag_noise_t *cap);

#endif /* MAG_NOISE_H */
