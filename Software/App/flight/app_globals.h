#ifndef APP_FLIGHT_APP_GLOBALS_H
#define APP_FLIGHT_APP_GLOBALS_H

#include "ms5611.h"
#include "lsm6dso32.h"
#include "adxl372.h"
#include "w25q512jv.h"
#include "casper_ekf.h"
#include "casper_attitude.h"
#include "max_m10m.h"
#include "mmc5983ma.h"

extern ms5611_t baro;
extern lsm6dso32_t imu;
extern adxl372_t high_g;
extern w25q512jv_t flash;
extern casper_ekf_t ekf;
extern casper_attitude_t att;
extern max_m10m_t gps;
extern mmc5983ma_t mag;

#endif
