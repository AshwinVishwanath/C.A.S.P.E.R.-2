/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Flight Logger
 *  SUMMARY:  Multi-stream record logger to QSPI flash; Hamming-protected.
 * ============================================================ */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "log_types.h"
#include "log_stream.h"
#include "log_index.h"
#include "lsm6dso32.h"
#include "ms5611.h"
#include "adxl372.h"
#include "mmc5983ma.h"
#include "max_m10m.h"
#include "casper_ekf.h"
#include "casper_attitude.h"

/* ═══════════════════════════════════════════════════════════════════════
 *  QSPI writer state machine
 * ═══════════════════════════════════════════════════════════════════════ */

typedef enum {
    QSPI_IDLE = 0,
    QSPI_ERASING,
    QSPI_WRITING,
    QSPI_POLLING
} qspi_state_t;

/* ═══════════════════════════════════════════════════════════════════════
 *  Top-level flight logger
 * ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    log_stream_t  hr;
    log_stream_t  lr;
    log_stream_t  adxl;
    log_index_t   index;
    flight_summary_t summary;

    qspi_state_t  qspi_state;
    log_stream_t  *active_stream;  /* stream being written/erased        */
    uint8_t        erase_pool;     /* which pool is being erased (0-2)   */

    uint16_t hr_seq;               /* HR record sequence counter         */
    uint16_t hr_tick_div;          /* HR rate divider                    */
    uint16_t lr_tick_div;          /* LR rate divider                    */
    uint16_t hr_tick_count;        /* HR divider counter                 */
    uint16_t lr_tick_count;        /* LR divider counter                 */

    bool     launched;
    bool     finalized;
} flight_logger_t;

/* ── Lifecycle ────────────────────────────────────────────────────────── */

bool flight_logger_init(flight_logger_t *log, w25q512jv_t *flash);
void flight_logger_start(flight_logger_t *log);
void flight_logger_launch(flight_logger_t *log);
void flight_logger_tick(flight_logger_t *log);
void flight_logger_finalize(flight_logger_t *log);

/* ── Rate management ──────────────────────────────────────────────────── */

void flight_logger_set_rate(flight_logger_t *log, uint8_t fsm_state);

/* ── Record packing + push ────────────────────────────────────────────── */

void flight_logger_push_hr(flight_logger_t *log,
                           const lsm6dso32_t *imu, const ms5611_t *baro,
                           const adxl372_t *adxl, const mmc5983ma_t *mag,
                           const casper_ekf_t *ekf, const casper_attitude_t *att,
                           uint8_t fsm_state, uint8_t flags,
                           uint16_t sustain_ms);

void flight_logger_push_lr(flight_logger_t *log,
                           uint8_t pyro_state, const uint16_t pyro_cont[4],
                           int8_t rssi, int8_t snr,
                           uint16_t tx_count, uint16_t rx_count, uint16_t fail_count,
                           const max_m10m_t *gps);

void flight_logger_push_adxl(flight_logger_t *log, const int16_t samples[][3],
                             uint16_t count);

/* ── Summary updates ──────────────────────────────────────────────────── */

void flight_logger_summary_imu(flight_logger_t *log, float accel_mag_g,
                               float tilt_deg, float roll_rate_dps,
                               uint8_t fsm_state);

void flight_logger_summary_ekf(flight_logger_t *log, float vel_mps,
                               float alt_m, float baro_pa, float temp_k);

void flight_logger_summary_event(flight_logger_t *log, uint8_t fsm_state,
                                 uint8_t prev_state);

void flight_logger_summary_gps(flight_logger_t *log, const max_m10m_t *gps);

void flight_logger_write_partial_summary(flight_logger_t *log);
void flight_logger_write_final_summary(flight_logger_t *log);

/* ── Status queries ──────────────────────────────────────────────────── */

static inline bool flight_logger_is_launched(const flight_logger_t *log)
    { return log->launched; }
static inline bool flight_logger_is_finalized(const flight_logger_t *log)
    { return log->finalized; }
static inline qspi_state_t flight_logger_qspi_get_state(const flight_logger_t *log)
    { return log->qspi_state; }
static inline uint32_t flight_logger_hr_records(const flight_logger_t *log)
    { return log->hr.records_written; }
static inline uint32_t flight_logger_lr_records(const flight_logger_t *log)
    { return log->lr.records_written; }
static inline uint16_t flight_logger_drop_count(const flight_logger_t *log)
    { return (uint16_t)(log->hr.drop_count + log->lr.drop_count + log->adxl.drop_count); }
static inline uint16_t flight_logger_err_count(const flight_logger_t *log)
    { return (uint16_t)(log->hr.err_count + log->lr.err_count + log->adxl.err_count); }

/* ── QSPI callbacks (ISR context) ─────────────────────────────────────── */

void flight_logger_qspi_complete(flight_logger_t *log);
void flight_logger_qspi_error(flight_logger_t *log);

#ifdef LOGGER_SANITY
/* Diagnostic call-rate counters (LOGGER_SANITY only).
 * "calls" = entry to push_hr/_lr (every invocation).
 * "pushes" = post-divider records actually pushed into the stream. */
uint32_t flight_logger_diag_hr_calls(void);
uint32_t flight_logger_diag_hr_pushes(void);
uint32_t flight_logger_diag_lr_calls(void);
uint32_t flight_logger_diag_lr_pushes(void);
#endif
