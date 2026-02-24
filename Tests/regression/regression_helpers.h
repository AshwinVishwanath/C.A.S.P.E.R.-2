#ifndef REGRESSION_HELPERS_H
#define REGRESSION_HELPERS_H

#include "csv_loader.h"

/* Convert feet to meters */
float ft_to_m(float ft);

/* Convert feet/s to m/s */
float fps_to_mps(float fps);

/* Find liftoff row in RawDataFile by detecting |accel| > threshold.
 * Accel columns are ax, ay, az in m/s^2 (old FC body frame).
 * Returns row index, or -1 if not found. */
int find_liftoff_row(const csv_table_t *table, float accel_threshold_mps2);

/* Compute flight time from RawDataFile absolute microsecond timestamps.
 * Returns (time_us - liftoff_time_us) / 1e6  in seconds. */
float raw_to_flight_time(float time_us, float liftoff_time_us);

/* Linear interpolation in a time series.
 * times[] must be monotonically increasing. Returns interpolated value
 * at time t. Clamps to first/last value if t is outside range. */
float interp_at_time(const float *times, const float *values, int count, float t);

#endif
