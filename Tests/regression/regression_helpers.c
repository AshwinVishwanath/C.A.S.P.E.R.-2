/**
 * @file regression_helpers.c
 * @brief Helper functions for regression tests.
 */

#include "regression_helpers.h"
#include <math.h>

float ft_to_m(float ft)
{
    return ft * 0.3048f;
}

float fps_to_mps(float fps)
{
    return fps * 0.3048f;
}

int find_liftoff_row(const csv_table_t *table, float accel_threshold_mps2)
{
    int col_ax = csv_find_col(table, "ax");
    int col_ay = csv_find_col(table, "ay");
    int col_az = csv_find_col(table, "az");

    if (col_ax < 0 || col_ay < 0 || col_az < 0)
        return -1;

    for (int i = 0; i < table->row_count; i++) {
        float ax = table->data[i][col_ax];
        float ay = table->data[i][col_ay];
        float az = table->data[i][col_az];
        float mag = sqrtf(ax * ax + ay * ay + az * az);
        if (mag > accel_threshold_mps2)
            return i;
    }
    return -1;
}

float raw_to_flight_time(float time_us, float liftoff_time_us)
{
    return (time_us - liftoff_time_us) / 1.0e6f;
}

float interp_at_time(const float *times, const float *values, int count, float t)
{
    if (count <= 0)
        return 0.0f;

    /* Clamp to bounds */
    if (t <= times[0])
        return values[0];
    if (t >= times[count - 1])
        return values[count - 1];

    /* Binary search for the interval [times[lo], times[lo+1]] containing t */
    int lo = 0, hi = count - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (times[mid] <= t)
            lo = mid;
        else
            hi = mid;
    }

    /* Linear interpolation */
    float dt = times[hi] - times[lo];
    if (dt <= 0.0f)
        return values[lo];

    float alpha = (t - times[lo]) / dt;
    return values[lo] + alpha * (values[hi] - values[lo]);
}
