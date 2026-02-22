/**
 * @file    temp_cal.h
 * @brief   Multi-sensor temperature calibration data collection for C.A.S.P.E.R.-2
 *
 * Logs LSM6DSO32 (gyro/accel/die temp), MS5611 (temp/pressure), and MMC5983MA
 * (mag field raw counts) at 10 Hz to TCAL_NNN.CSV on QSPI flash.
 * Terminated by "STOP" command over USB CDC.
 */
#ifndef TEMP_CAL_H
#define TEMP_CAL_H

#include "stm32h7xx_hal.h"
#include "lsm6dso32.h"
#include "ms5611.h"
#include "mmc5983ma.h"
#include "ff.h"
#include <stdbool.h>
#include <stdint.h>

#define TCAL_FLUSH_INTERVAL   30     /* samples between forced f_sync()    */
#define TCAL_HEARTBEAT_MS     10000  /* CDC heartbeat interval             */
#define TCAL_WBUF_SIZE        2048   /* write buffer size                  */

typedef enum {
    TCAL_IDLE,
    TCAL_COLLECTING,
    TCAL_STOPPED
} temp_cal_state_t;

typedef struct {
    temp_cal_state_t state;

    /* FATFS */
    FIL      file;
    bool     file_open;
    char     filename[16];       /* "TCAL_NNN.CSV" */

    /* Write buffer */
    char     wbuf[TCAL_WBUF_SIZE];
    uint16_t wbuf_pos;

    /* Timing & counters */
    uint32_t sample_count;
    uint32_t samples_since_flush;
    uint32_t start_ms;
    uint32_t last_heartbeat_ms;

    /* CDC STOP command accumulator */
    char     cmd_buf[4];
    uint8_t  cmd_pos;
} temp_cal_t;

/**
 * @brief  Find next available TCAL_NNN.CSV, open it, write comment + header.
 *         Sends startup banner over CDC.
 * @return true if file opened successfully
 */
bool temp_cal_init(temp_cal_t *tc);

/**
 * @brief  Sample all three sensors, append CSV line, check for STOP command,
 *         send heartbeat.  Call at 10 Hz from the main loop.
 * @param  tc     Module state
 * @param  imu    Pointer to LSM6DSO32 handle (read inside)
 * @param  baro   Pointer to MS5611 handle (blocking read inside)
 * @param  mag    Pointer to MMC5983MA handle (oneshot trigger+read inside)
 * @param  now_ms Current HAL_GetTick() value
 */
void temp_cal_tick(temp_cal_t *tc, lsm6dso32_t *imu,
                   ms5611_t *baro, mmc5983ma_t *mag, uint32_t now_ms);

/**
 * @brief  Check if collection has been stopped by the user.
 */
bool temp_cal_is_stopped(const temp_cal_t *tc);

#endif /* TEMP_CAL_H */
