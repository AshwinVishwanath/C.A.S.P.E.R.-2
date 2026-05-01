/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Log Types
 *  SUMMARY:  Stream IDs, record headers, on-flash log layout constants.
 * ============================================================ */
#pragma once

#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════
 *  Flash Layout (W25Q512JV — 64 MB total)
 *
 *  The flash is partitioned into two non-overlapping regions:
 *
 *    0x00000000 .. 0x03C00000   Flight log (60 MB) — raw NOR, custom layout
 *    0x03C00000 .. 0x04000000   FATFS    ( 4 MB) — used by USB MSC + cal CSV
 *
 *  Keeping FATFS off the flight log address range is safety-critical:
 *  f_mkfs and FATFS writes hit sector 0 of FATFS = 0x03C00000 (NOT 0), so
 *  formatting the filesystem can never corrupt the flight index.
 * ═══════════════════════════════════════════════════════════════════════ */

#define LOG_FLASH_END          0x03C00000u   /* 60 MB — top of flight log  */

#define FLASH_INDEX_BASE       0x00000000u   /* 4 KB   — flight index      */
#define FLASH_INDEX_SIZE       0x00001000u
#define FLASH_INDEX_END        (FLASH_INDEX_BASE + FLASH_INDEX_SIZE)

#define FLASH_SUMMARY_BASE     0x00001000u   /* 64 KB  — flight summaries  */
#define FLASH_SUMMARY_SIZE     0x00010000u
#define FLASH_SUMMARY_END      (FLASH_SUMMARY_BASE + FLASH_SUMMARY_SIZE)

#define FLASH_LR_BASE          0x00011000u   /* 512 KB — low-rate records  */
#define FLASH_LR_SIZE          0x00080000u
#define FLASH_LR_END           (FLASH_LR_BASE + FLASH_LR_SIZE)

#define FLASH_ADXL_BASE        0x00091000u   /* 16 MB  — ADXL records      */
#define FLASH_ADXL_SIZE        0x01000000u
#define FLASH_ADXL_END         (FLASH_ADXL_BASE + FLASH_ADXL_SIZE)

#define FLASH_HR_BASE          0x01091000u   /* ~43.4 MB high-rate records */
#define FLASH_HR_SIZE          (LOG_FLASH_END - FLASH_HR_BASE)
#define FLASH_HR_END           LOG_FLASH_END

/* FATFS region — separate from the flight log. user_diskio.c and
 * usbd_msc_storage_if.c offset all sector accesses by FATFS_FLASH_BASE so
 * sector 0 of the FATFS / MSC view maps to physical flash 0x03C00000. */
#define FATFS_FLASH_BASE       0x03C00000u   /* 4 MB FATFS partition       */
#define FATFS_FLASH_SIZE       0x00400000u
#define FATFS_FLASH_END        0x04000000u
#define FATFS_SECTOR_COUNT     (FATFS_FLASH_SIZE / 0x1000u)  /* 1024 × 4 KB */

/* ═══════════════════════════════════════════════════════════════════════
 *  Constants
 * ═══════════════════════════════════════════════════════════════════════ */

#define LOG_PAGE_SIZE        256
#define HR_REC_SIZE          64
#define LR_REC_SIZE          32
#define ADXL_REC_SIZE        10
#define HR_RECS_PER_PAGE     4      /* 256 / 64 */
#define LR_RECS_PER_PAGE     8      /* 256 / 32 */
#define ADXL_RECS_PER_PAGE   25     /* 250 / 10, 6 bytes pad per page */
#define SUMMARY_MAGIC        0x43535052u  /* "CSPR" */
#define SUMMARY_VERSION      1
#define SUMMARY_PARTIAL      0x01
#define SUMMARY_FINAL        0x02
#define MAX_FLIGHTS          113   /* floor(4096 / 36) — index entry grew to 36 B */
#define INDEX_DIRTY          0xFFFFFFFFu

/* ═══════════════════════════════════════════════════════════════════════
 *  HR flags bits
 * ═══════════════════════════════════════════════════════════════════════ */

#define HR_FLAG_BARO_VALID    (1u << 0)
#define HR_FLAG_GPS_VALID     (1u << 1)
#define HR_FLAG_ADXL_ACTIVE   (1u << 2)
#define HR_FLAG_ZUPT_FIRED    (1u << 3)
#define HR_FLAG_MACH_GATED    (1u << 4)
#define HR_FLAG_MAG_VALID     (1u << 5)
#define HR_FLAG_PYRO_ARMED    (1u << 6)

/* ═══════════════════════════════════════════════════════════════════════
 *  Logger state machine
 * ═══════════════════════════════════════════════════════════════════════ */

typedef enum {
    LOG_IDLE  = 0,
    LOG_PAD   = 1,
    LOG_DRAIN = 2,
    LOG_FINAL = 3
} log_state_t;

/* ═══════════════════════════════════════════════════════════════════════
 *  High-rate record (64 bytes, packed)
 * ═══════════════════════════════════════════════════════════════════════ */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;          /*  0: HAL_GetTick()                  */
    uint16_t baro_pressure;         /*  4: 2 Pa/LSB                       */
    int16_t  lsm6_accel[3];        /*  6: raw LSB                        */
    int16_t  lsm6_gyro[3];         /* 12: raw LSB                        */
    int16_t  adxl372[3];           /* 18: 100 mg/LSB                     */
    uint16_t mmc[3];               /* 24: raw 18-bit right-shifted to 16 */
    float    ekf_alt_m;            /* 30: EKF altitude                   */
    float    ekf_vel_mps;          /* 34: EKF velocity                   */
    uint8_t  quat_packed[5];       /* 38: smallest-three quaternion      */
    uint8_t  fsm_state;            /* 43: flight FSM state               */
    uint8_t  flags;                /* 44: HR_FLAG_* bits                 */
    int16_t  ekf_accel_bias;       /* 45: 0.001 m/s^2 / LSB             */
    int16_t  ekf_baro_bias;        /* 47: 0.01 m / LSB                  */
    int16_t  imu_temp;             /* 49: 0.01 degC / LSB               */
    int16_t  baro_temp;            /* 51: 0.01 degC / LSB               */
    uint16_t seq_num;              /* 53: monotonic counter              */
    uint16_t sustain_counter_ms;   /* 55: sustain dwell counter          */
    uint8_t  reserved[3];          /* 57: pad to 60                      */
    uint16_t hamming_secded;       /* 60: SECDED over [0:59]             */
    uint16_t crc16;                /* 62: CRC16-CCITT over [0:61]        */
} hr_record_t;                     /* 64 bytes total                     */

_Static_assert(sizeof(hr_record_t) == 64, "hr_record_t must be 64 bytes");

/* ═══════════════════════════════════════════════════════════════════════
 *  Low-rate record (32 bytes, packed)
 * ═══════════════════════════════════════════════════════════════════════ */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;          /*  0: HAL_GetTick()                 */
    uint8_t  pyro_state;            /*  4: bits [3:0] = fire per ch      */
    uint16_t pyro_cont_adc[4];      /*  5: raw ADC (one ch = battery)    */
    int8_t   radio_rssi;            /* 13: dBm                           */
    int8_t   radio_snr;             /* 14: dB                            */
    uint16_t radio_tx_count;        /* 15: total TX packets              */
    uint16_t radio_rx_count;        /* 17: total RX packets              */
    uint16_t radio_fail_count;      /* 19: total failures                */
    int32_t  gps_dlat_mm;           /* 21: delta lat from pad (mm)       */
    int32_t  gps_dlon_mm;           /* 25: delta lon from pad (mm)       */
    int16_t  gps_alt_msl_m;        /* 29: GPS alt MSL (m)               */
    uint8_t  gps_fix_sats;         /* 31: upper nibble fix, lower sats  */
} lr_record_t;                     /* 32 bytes total                    */

_Static_assert(sizeof(lr_record_t) == 32, "lr_record_t must be 32 bytes");

/* ═══════════════════════════════════════════════════════════════════════
 *  ADXL high-g record (10 bytes, packed)
 * ═══════════════════════════════════════════════════════════════════════ */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;          /* 0: HAL_GetTick()                  */
    int16_t  accel_x;              /* 4: 100 mg/LSB                     */
    int16_t  accel_y;              /* 6: 100 mg/LSB                     */
    int16_t  accel_z;              /* 8: 100 mg/LSB                     */
} adxl_record_t;                   /* 10 bytes total                    */

_Static_assert(sizeof(adxl_record_t) == 10, "adxl_record_t must be 10 bytes");

/* ═══════════════════════════════════════════════════════════════════════
 *  Flight summary (256 bytes, packed)
 *
 *  HEADER (8) + TIMING (28) + DERIVED TIMES (12) + PEAK DYNAMICS (24)
 *  + APOGEE (12) + ATTITUDE (16) + TEMPERATURE (8) + PYRO (24)
 *  + GPS (20) + DIAGNOSTICS (20) + RESERVED (80) + INTEGRITY (4)
 *  = 256 bytes
 * ═══════════════════════════════════════════════════════════════════════ */

typedef struct __attribute__((packed)) {
    /* ── HEADER (8 B) ────────────────────────────────────────────────── */
    uint32_t magic;                 /*  0: SUMMARY_MAGIC (0x43535052)    */
    uint8_t  format_version;        /*  4: SUMMARY_VERSION               */
    uint8_t  summary_type;          /*  5: SUMMARY_PARTIAL / FINAL       */
    uint16_t flags;                 /*  6: reserved flags                */

    /* ── TIMING (28 B) ───────────────────────────────────────────────── */
    uint32_t launch_tick;           /*  8: HAL_GetTick at launch         */
    uint32_t burnout1_tick;         /* 12: first burnout                 */
    uint32_t burnout2_tick;         /* 16: second burnout (0 if N/A)     */
    uint32_t apogee_tick;           /* 20: apogee detection              */
    uint32_t main_deploy_tick;      /* 24: main chute deploy             */
    uint32_t landing_tick;          /* 28: landing detection             */
    uint32_t total_flight_tick;     /* 32: total flight duration (ms)    */

    /* ── DERIVED TIMES (12 B) ────────────────────────────────────────── */
    uint32_t time_to_apogee;        /* 36: launch-to-apogee (ms)         */
    uint32_t peak_vel_tick;         /* 40: tick at peak velocity          */
    uint32_t max_q_tick;            /* 44: tick at max dynamic pressure   */

    /* ── PEAK DYNAMICS (24 B) ────────────────────────────────────────── */
    float    peak_accel_boost_g;    /* 48: peak accel during boost (g)   */
    float    peak_accel_sustain_g;  /* 52: peak accel during sustain (g) */
    float    peak_velocity_mps;     /* 56: peak velocity (m/s)           */
    float    peak_mach;             /* 60: peak Mach number              */
    float    max_q_pa;              /* 64: max dynamic pressure (Pa)     */
    float    max_q_alt_m;           /* 68: altitude at max-Q (m)         */

    /* ── APOGEE (12 B) ───────────────────────────────────────────────── */
    float    apogee_ekf_m;          /* 72: EKF altitude at apogee (m)    */
    float    apogee_baro_m;         /* 76: baro altitude at apogee (m)   */
    float    apogee_gps_msl_m;      /* 80: GPS MSL alt at apogee (m)     */

    /* ── ATTITUDE (16 B) ─────────────────────────────────────────────── */
    float    pad_tilt_deg;          /* 84: tilt angle on pad (deg)       */
    float    max_tilt_deg;          /* 88: max tilt during flight (deg)  */
    float    max_roll_rate_dps;     /* 92: max roll rate (deg/s)         */
    float    max_descent_rate_mps;  /* 96: max descent rate (m/s, +down) */

    /* ── TEMPERATURE (8 B) ───────────────────────────────────────────── */
    float    peak_temp_avg_c;       /* 100: peak average temp (C)        */
    float    min_temp_avg_c;        /* 104: min average temp (C)         */

    /* ── PYRO (24 B) ─────────────────────────────────────────────────── */
    uint32_t pyro_fire_tick[4];     /* 108: fire tick per channel         */
    uint16_t pyro_cont_at_launch[4];/* 124: continuity ADC at launch     */

    /* ── GPS (20 B) ──────────────────────────────────────────────────── */
    int32_t  launch_lat_1e7;        /* 132: launch latitude (deg*1e7)    */
    int32_t  launch_lon_1e7;        /* 136: launch longitude (deg*1e7)   */
    int32_t  landing_lat_1e7;       /* 140: landing latitude (deg*1e7)   */
    int32_t  landing_lon_1e7;       /* 144: landing longitude (deg*1e7)  */
    float    gps_max_alt_msl_m;     /* 148: max GPS altitude MSL (m)     */

    /* ── DIAGNOSTICS (20 B) ─────────────────────────────────────────── */
    uint32_t hr_records_written;    /* 152: total HR records written     */
    uint32_t lr_records_written;    /* 156: total LR records written     */
    uint32_t adxl_records_written;  /* 160: total ADXL records written   */
    uint16_t total_drop_count;      /* 164: total ring overrun drops     */
    uint16_t flash_err_count;       /* 166: total flash write errors     */
    uint16_t num_fsm_transitions;   /* 168: number of FSM transitions    */
    uint16_t diag_reserved;         /* 170: reserved                     */

    /* ── RESERVED (80 B) ─────────────────────────────────────────────── */
    uint8_t  _reserved[80];         /* 172: pad to 252                   */

    /* ── INTEGRITY (4 B) ─────────────────────────────────────────────── */
    uint32_t crc32;                 /* 252: CRC32 over [0:251]           */
} flight_summary_t;                 /* 256 bytes total                   */

_Static_assert(sizeof(flight_summary_t) == 256, "flight_summary_t must be 256 bytes");

/* ═══════════════════════════════════════════════════════════════════════
 *  Flight index entry (32 bytes, packed)
 * ═══════════════════════════════════════════════════════════════════════ */

/* [ID:2][start_tick:4][end_tick:4][hr_start:4][hr_end:4][lr_start:4][lr_end:4]
 * [adxl_start:4][adxl_end:4][flags:2] = 38 bytes
 * adxl_end_addr added at end; index sector is rewritten each flight cycle so
 * no backward compatibility concern with existing flash contents. */
typedef struct __attribute__((packed)) {
    uint16_t flight_id;             /*  0: monotonic flight number       */
    uint32_t start_tick_ms;         /*  2: launch tick                   */
    uint32_t end_tick_ms;           /*  6: landing tick (INDEX_DIRTY)    */
    uint32_t hr_start_addr;         /* 10: first HR flash address        */
    uint32_t hr_end_addr;           /* 14: last HR flash addr (DIRTY)    */
    uint32_t lr_start_addr;         /* 18: first LR flash address        */
    uint32_t lr_end_addr;           /* 22: last LR flash addr (DIRTY)    */
    uint32_t adxl_start_addr;       /* 26: first ADXL flash address      */
    uint32_t adxl_end_addr;         /* 30: last ADXL flash addr (DIRTY)  */
    uint16_t flags;                 /* 34: bit0: clean_shutdown          */
} flight_index_entry_t;            /* 36 bytes total                    */

_Static_assert(sizeof(flight_index_entry_t) == 36, "flight_index_entry_t must be 36 bytes");
