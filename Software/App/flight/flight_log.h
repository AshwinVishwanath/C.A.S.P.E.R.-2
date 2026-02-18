#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "tlm_types.h"   /* fsm_state_t, FSM_STATE_* */

/* ── Flash layout ──────────────────────────────────────────────────── */
#define FLASH_HR_START          0x00000000
#define FLASH_HR_END            0x03800000   /* 56 MB high-rate region */
#define FLASH_LR_START          0x03800000
#define FLASH_LR_END            0x03E00000   /* 6 MB low-rate region  */
#define FLASH_SUMMARY_START     0x03E00000
#define FLASH_SUMMARY_END       0x03F00000   /* 1 MB summary region   */
#define FLASH_CONFIG_START      0x03F00000
#define FLASH_CONFIG_END        0x03F01000   /* 4 KB config storage   */
#define FLOG_PAGE_SIZE          256
#define FLOG_SECTOR_SIZE        4096
#define FLOG_BLOCK64_SIZE       65536

/* ── High-rate entry (64 bytes, packed) ─────────────────────────────
 * [timestamp_us:4][fresh:1][fsm_state:1][accel_mg:6][gyro_raw:6]
 * [imu_temp_c100:2][highg_10mg:6][baro_pa:4][baro_temp_c100:2]
 * [ekf_alt_cm:4][ekf_vel_cmps:4][ekf_abias_mmps2:2][ekf_bbias_cm:2]
 * [quat_packed:6][tilt_cdeg:2][flags:1][reserved:11] = 64           */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;          /* DWT us since boot                    */
    uint8_t  fresh;                 /* b0=imu, b1=highg, b2=baro            */
    uint8_t  fsm_state;             /* current FSM state                    */
    int16_t  accel_mg[3];           /* body accel, milligees                */
    int16_t  gyro_raw[3];           /* raw register LSBs                    */
    int16_t  imu_temp_c100;         /* die temp C * 100                     */
    int16_t  highg_10mg[3];         /* 10 mg units                          */
    uint32_t baro_pa;               /* compensated Pascals                  */
    int16_t  baro_temp_c100;        /* die temp C * 100                     */
    int32_t  ekf_alt_cm;            /* altitude AGL, cm                     */
    int32_t  ekf_vel_cmps;          /* velocity cm/s, +up                   */
    int16_t  ekf_abias_mmps2;       /* accel bias mm/s^2                    */
    int16_t  ekf_bbias_cm;          /* baro bias cm                         */
    int16_t  quat_packed[3];        /* smallest-three Q14                   */
    int16_t  tilt_cdeg;             /* centidegrees                         */
    uint8_t  flags;                 /* b0=baro_gated b1=launched b2=mag_valid */
    uint8_t  reserved[11];
} highrate_entry_t;

_Static_assert(sizeof(highrate_entry_t) == 64, "highrate_entry_t must be 64 bytes");

/* ── Low-rate entry (64 bytes, packed) ──────────────────────────────
 * [timestamp_us:4][fsm_state:1][flags:1][mag_raw:6][mag_temp_c100:2]
 * [batt_mv:2][batt_ma:2][cont_scaled:4][gps_lat_deg7:4][gps_lon_deg7:4]
 * [gps_alt_dm:2][gps_vel_d_cmps:2][gps_sats:1][gps_fix:1][gps_pdop:1]
 * [gps_fresh:1][radio_tx_seq:1][radio_rx_good:1][radio_rx_bad:1]
 * [radio_rssi:1][radio_snr:1][pyro_arm_cont:1][reserved:20] = 64    */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;          /* DWT us since boot                    */
    uint8_t  fsm_state;             /* current FSM state                    */
    uint8_t  flags;                 /* b0=firing b1=test_mode b2=sim_active */
    int16_t  mag_raw[3];            /* raw mag sensor reading               */
    int16_t  mag_temp_c100;         /* mag temp C*100, 0x7FFF if unavail    */
    uint16_t batt_mv;               /* battery voltage mV                   */
    int16_t  batt_ma;               /* battery current mA                   */
    uint8_t  cont_scaled[4];        /* (raw_adc >> 8) per channel           */
    int32_t  gps_lat_deg7;          /* latitude deg * 1e7                   */
    int32_t  gps_lon_deg7;          /* longitude deg * 1e7                  */
    int16_t  gps_alt_dm;            /* GPS altitude decimeters              */
    int16_t  gps_vel_d_cmps;        /* GPS down velocity cm/s               */
    uint8_t  gps_sats;              /* satellites used                      */
    uint8_t  gps_fix;               /* fix type                             */
    uint8_t  gps_pdop;              /* PDOP scaled                          */
    uint8_t  gps_fresh;             /* 1 if new GPS this tick               */
    uint8_t  radio_tx_seq;          /* last TX sequence number              */
    uint8_t  radio_rx_good;         /* good packets received                */
    uint8_t  radio_rx_bad;          /* bad packets received                 */
    int8_t   radio_rssi;            /* RSSI dBm                             */
    int8_t   radio_snr;             /* SNR dB                               */
    uint8_t  pyro_arm_cont;         /* high nibble=arm, low=continuity      */
    uint8_t  reserved[20];
} lowrate_entry_t;

_Static_assert(sizeof(lowrate_entry_t) == 64, "lowrate_entry_t must be 64 bytes");

/* ── Summary entry (variable length in flash) ──────────────────────
 * [timestamp_ms:4][msg_len:1][msg:N] — NOT null-terminated in flash */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  msg_len;               /* max 250 */
    char     msg[];                  /* flexible array member */
} flight_log_summary_entry_t;

/* ── Ring buffer & rate constants ──────────────────────────────────── */
#define HR_ENTRIES_PER_PAGE     (FLOG_PAGE_SIZE / sizeof(highrate_entry_t))    /* 4  */
#define HR_ENTRIES_PER_SECTOR   (FLOG_SECTOR_SIZE / sizeof(highrate_entry_t))  /* 64 */
#define LR_ENTRIES_PER_PAGE     (FLOG_PAGE_SIZE / sizeof(lowrate_entry_t))     /* 4  */
#define LR_ENTRIES_PER_SECTOR   (FLOG_SECTOR_SIZE / sizeof(lowrate_entry_t))   /* 64 */

#define HR_LOG_RATE_HZ          250
#define LR_LOG_RATE_HZ          10

#define HR_RING_SECONDS         5
#define HR_RING_ENTRIES         (HR_RING_SECONDS * HR_LOG_RATE_HZ)   /* 1250 */
#define LR_RING_SECONDS         10
#define LR_RING_ENTRIES         (LR_RING_SECONDS * LR_LOG_RATE_HZ)  /* 100  */

/* ── Fresh-data bitfield values ────────────────────────────────────── */
#define FRESH_IMU               0x01
#define FRESH_HIGHG             0x02
#define FRESH_BARO              0x04

/* ── API ───────────────────────────────────────────────────────────── */

/** Initialize flight log. Enables DWT cycle counter. Returns 0 on success. */
int  flight_log_init(void);

/** Write a high-rate entry. Ring-buffers in PAD, double-buffers in flight. */
void flight_log_write_hr(const highrate_entry_t *entry, fsm_state_t state);

/** Write a low-rate entry. Ring-buffers in PAD, double-buffers in flight. */
void flight_log_write_lr(const lowrate_entry_t *entry, fsm_state_t state);

/** Commit ring buffers to flash on launch detection. Sets logging_active. */
void flight_log_commit_ring_buffers(void);

/** Write a printf-formatted summary string to the summary region. */
void flight_log_summary(uint32_t timestamp_ms, const char *fmt, ...);

/** Flush partial double buffers to flash. Call on landing. */
void flight_log_flush(void);

/** Get current high-rate flash write address. */
uint32_t flight_log_get_hr_addr(void);

/** Get current low-rate flash write address. */
uint32_t flight_log_get_lr_addr(void);

/** Get current summary flash write address. */
uint32_t flight_log_get_summary_addr(void);

/** Get total high-rate entries written to flash. */
uint32_t flight_log_get_hr_count(void);

/** Get total low-rate entries written to flash. */
uint32_t flight_log_get_lr_count(void);

/** Erase all flight log regions using block64 erases. Returns 0 on success. */
int  flight_log_erase_all(void);

/** Process pending flash writes (double-buffer flush). Returns pages written. */
int  flight_log_process(void);

/** Get number of dropped high-rate entries. */
uint32_t flight_log_get_dropped_hr(void);

/** Get number of dropped low-rate entries. */
uint32_t flight_log_get_dropped_lr(void);

/** Get microsecond timestamp from DWT cycle counter. */
uint32_t flight_log_get_timestamp_us(void);

#endif /* FLIGHT_LOG_H */
