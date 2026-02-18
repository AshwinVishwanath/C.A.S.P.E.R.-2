#include "flight_log_helpers.h"
#include <string.h>

/* ── High-rate populate ────────────────────────────────────────── */
void flight_log_populate_hr(highrate_entry_t *entry,
                            uint32_t ts_us,
                            fsm_state_t state,
                            const imu_snapshot_t *imu,
                            const highg_snapshot_t *highg,
                            const baro_snapshot_t *baro,
                            const ekf_snapshot_t *ekf,
                            const attitude_snapshot_t *att,
                            bool baro_gated)
{
    memset(entry, 0, sizeof(*entry));
    entry->timestamp_us = ts_us;
    entry->fsm_state    = state;

    /* Fresh-data bitfield */
    uint8_t fresh = 0;
    if (imu->updated)   fresh |= FRESH_IMU;
    if (highg->updated)  fresh |= FRESH_HIGHG;
    if (baro->updated)   fresh |= FRESH_BARO;
    entry->fresh = fresh;

    /* IMU: accel in milligees, gyro raw, temp */
    entry->accel_mg[0]    = (int16_t)(imu->accel_ms2[0] * (1000.0f / 9.80665f));
    entry->accel_mg[1]    = (int16_t)(imu->accel_ms2[1] * (1000.0f / 9.80665f));
    entry->accel_mg[2]    = (int16_t)(imu->accel_ms2[2] * (1000.0f / 9.80665f));
    entry->gyro_raw[0]    = imu->gyro_raw[0];
    entry->gyro_raw[1]    = imu->gyro_raw[1];
    entry->gyro_raw[2]    = imu->gyro_raw[2];
    entry->imu_temp_c100  = (int16_t)(imu->imu_temp_c * 100.0f);

    /* High-G: 10 mg units (accel_g * 100) */
    entry->highg_10mg[0]  = (int16_t)(highg->accel_g[0] * 100.0f);
    entry->highg_10mg[1]  = (int16_t)(highg->accel_g[1] * 100.0f);
    entry->highg_10mg[2]  = (int16_t)(highg->accel_g[2] * 100.0f);

    /* Baro */
    entry->baro_pa        = baro->pressure_pa;
    entry->baro_temp_c100 = (int16_t)(baro->temp_c * 100.0f);

    /* EKF: centimetres, cm/s, mm/s^2, cm */
    entry->ekf_alt_cm       = (int32_t)(ekf->alt_m * 100.0f);
    entry->ekf_vel_cmps     = (int32_t)(ekf->vel_mps * 100.0f);
    entry->ekf_abias_mmps2  = (int16_t)(ekf->accel_bias * 1000.0f);
    entry->ekf_bbias_cm     = (int16_t)(ekf->baro_bias * 100.0f);

    /* Attitude: smallest-three Q14 packing of quaternion */
    /* Simple approach: store x,y,z scaled by 16384 (Q14), assuming w > 0 */
    float qw = att->q[0], qx = att->q[1], qy = att->q[2], qz = att->q[3];
    if (qw < 0.0f) { qw = -qw; qx = -qx; qy = -qy; qz = -qz; }
    entry->quat_packed[0] = (int16_t)(qx * 16384.0f);
    entry->quat_packed[1] = (int16_t)(qy * 16384.0f);
    entry->quat_packed[2] = (int16_t)(qz * 16384.0f);
    entry->tilt_cdeg       = (int16_t)(att->tilt_deg * 100.0f);

    /* Flags */
    entry->flags = 0;
    if (baro_gated) entry->flags |= 0x01;
}

/* ── Low-rate populate ─────────────────────────────────────────── */
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
                            bool sim_active)
{
    memset(entry, 0, sizeof(*entry));
    entry->timestamp_us = ts_us;
    entry->fsm_state    = state;

    /* Flags: b0=firing, b1=test_mode, b2=sim_active */
    entry->flags = 0;
    if (firing)     entry->flags |= 0x01;
    if (test_mode)  entry->flags |= 0x02;
    if (sim_active) entry->flags |= 0x04;

    /* Magnetometer */
    entry->mag_raw[0]     = mag->raw[0];
    entry->mag_raw[1]     = mag->raw[1];
    entry->mag_raw[2]     = mag->raw[2];
    entry->mag_temp_c100  = mag->updated ? (int16_t)(mag->temp_c * 100.0f) : 0x7FFF;

    /* Power */
    entry->batt_mv        = pwr->batt_mv;
    entry->batt_ma        = pwr->batt_ma;
    entry->cont_scaled[0] = (uint8_t)(pwr->cont_raw[0] >> 8);
    entry->cont_scaled[1] = (uint8_t)(pwr->cont_raw[1] >> 8);
    entry->cont_scaled[2] = (uint8_t)(pwr->cont_raw[2] >> 8);
    entry->cont_scaled[3] = (uint8_t)(pwr->cont_raw[3] >> 8);

    /* GPS */
    entry->gps_lat_deg7   = gps->lat_deg7;
    entry->gps_lon_deg7   = gps->lon_deg7;
    entry->gps_alt_dm     = gps->alt_dm;
    entry->gps_vel_d_cmps = gps->vel_d_cmps;
    entry->gps_sats       = gps->sats;
    entry->gps_fix        = gps->fix;
    entry->gps_pdop       = gps->pdop;
    entry->gps_fresh      = gps->fresh ? 1 : 0;

    /* Radio */
    entry->radio_tx_seq   = radio->tx_seq;
    entry->radio_rx_good  = radio->rx_good;
    entry->radio_rx_bad   = radio->rx_bad;
    entry->radio_rssi     = radio->rssi;
    entry->radio_snr      = radio->snr;

    /* Pyro: high nibble = arm, low = continuity */
    entry->pyro_arm_cont  = (uint8_t)((arm_bm << 4) | (cont_bm & 0x0F));
}
