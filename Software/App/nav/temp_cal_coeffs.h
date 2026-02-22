/**
 * @file    temp_cal_coeffs.h
 * @brief   Gyro temperature compensation coefficients for C.A.S.P.E.R.-2.
 *
 * Source: casper_temp_cal_v2.m (v3), freezer-to-desk ramp (T_CAL_COLD.csv)
 * Primary fit: COLD warming segment.  Cross-validated against COLD cooling.
 *
 * Usage: #include from flight_loop.c, guarded by GYRO_TEMP_COMP define.
 * Applied before axis remapping: gyro_sensor[i] -= SLOPE[i] * (T - T0)
 */
#ifndef TEMP_CAL_COEFFS_H
#define TEMP_CAL_COEFFS_H

#define GYRO_TC_T0       15.250f          /* Reference temperature (deg C)         */
#define GYRO_TC_SLOPE_X  (-5.2144e-04f)   /* rad/s per deg C, X (R^2=0.970)       */
#define GYRO_TC_SLOPE_Y  (+2.2923e-04f)   /* rad/s per deg C, Y (R^2=0.987)       */
#define GYRO_TC_SLOPE_Z  0.0f             /* ZEROED: cross-validation failed       */

#endif /* TEMP_CAL_COEFFS_H */
