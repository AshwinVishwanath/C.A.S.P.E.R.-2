#ifndef FLIGHT_LOG_HELPERS_H
#define FLIGHT_LOG_HELPERS_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_log.h"

/* ── Snapshot structs ────────────────────────────────────────────── */
/* main.c populates these each tick; populate functions pack them    */
/* into highrate_entry_t / lowrate_entry_t for flash logging.       */

typedef struct {
    float   accel_ms2[3];   /* body-frame accel in m/s^2 */
    int16_t gyro_raw[3];    /* raw gyro register values   */
    float   imu_temp_c;     /* die temp in C              */
    bool    updated;
} imu_snapshot_t;

typedef struct {
    float   accel_g[3];     /* high-g accel in g          */
    bool    updated;
} highg_snapshot_t;

typedef struct {
    uint32_t pressure_pa;   /* compensated pressure Pa    */
    float    temp_c;        /* baro die temp in C         */
    bool     updated;
} baro_snapshot_t;

typedef struct {
    float   alt_m;          /* EKF altitude AGL           */
    float   vel_mps;        /* EKF velocity m/s +up       */
    float   accel_bias;     /* EKF accel bias m/s^2       */
    float   baro_bias;      /* EKF baro bias m            */
} ekf_snapshot_t;

typedef struct {
    float   q[4];           /* quaternion [w,x,y,z]       */
    float   tilt_deg;       /* tilt from vertical          */
} attitude_snapshot_t;

typedef struct {
    int16_t raw[3];         /* 16-bit mag values           */
    float   temp_c;         /* mag temperature C           */
    bool    updated;
} mag_snapshot_t;

typedef struct {
    int32_t lat_deg7;       /* latitude deg * 1e-7         */
    int32_t lon_deg7;       /* longitude deg * 1e-7        */
    int16_t alt_dm;         /* altitude decimetres         */
    int16_t vel_d_cmps;     /* down velocity cm/s          */
    uint8_t sats;           /* satellites used             */
    uint8_t fix;            /* fix type                    */
    uint8_t pdop;           /* PDOP scaled (/10)           */
    bool    fresh;          /* new fix this tick           */
} gps_snapshot_t;

typedef struct {
    uint16_t batt_mv;       /* battery voltage mV          */
    int16_t  batt_ma;       /* battery current mA          */
    uint16_t cont_raw[4];   /* continuity ADC raw          */
} power_snapshot_t;

typedef struct {
    uint8_t tx_seq;         /* last TX sequence number     */
    uint8_t rx_good;        /* good packets received       */
    uint8_t rx_bad;         /* bad packets received        */
    int8_t  rssi;           /* RSSI dBm                    */
    int8_t  snr;            /* SNR dB                      */
} radio_snapshot_t;

/* ── Populate functions ──────────────────────────────────────────── */

/**
 * Pack snapshot data into a highrate_entry_t.
 * @param entry       Output entry to populate
 * @param ts_us       DWT microsecond timestamp
 * @param state       Current FSM state
 * @param imu         IMU snapshot
 * @param highg       High-G accel snapshot
 * @param baro        Baro snapshot
 * @param ekf         EKF snapshot
 * @param att         Attitude snapshot
 * @param baro_gated  True if EKF baro gating is active
 */
void flight_log_populate_hr(highrate_entry_t *entry,
                            uint32_t ts_us,
                            fsm_state_t state,
                            const imu_snapshot_t *imu,
                            const highg_snapshot_t *highg,
                            const baro_snapshot_t *baro,
                            const ekf_snapshot_t *ekf,
                            const attitude_snapshot_t *att,
                            bool baro_gated);

/**
 * Pack snapshot data into a lowrate_entry_t.
 * @param entry       Output entry to populate
 * @param ts_us       DWT microsecond timestamp
 * @param state       Current FSM state
 * @param mag         Magnetometer snapshot
 * @param gps         GPS snapshot
 * @param pwr         Power snapshot
 * @param radio       Radio snapshot
 * @param arm_bm      Pyro arm bitmap (bit per channel)
 * @param cont_bm     Pyro continuity bitmap
 * @param firing      True if any pyro currently firing
 * @param test_mode   True if test mode active
 * @param sim_active  True if sim flight active
 */
void flight_log_populate_lr(lowrate_entry_t *entry,
                            uint32_t ts_us,
                            fsm_state_t state,
                            const mag_snapshot_t *mag,
                            const gps_snapshot_t *gps,
                            const power_snapshot_t *pwr,
                            const radio_snapshot_t *radio,
                            uint8_t arm_bm,
                            uint8_t cont_bm,
                            bool firing,
                            bool test_mode,
                            bool sim_active);

#endif /* FLIGHT_LOG_HELPERS_H */
