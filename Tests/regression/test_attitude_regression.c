/**
 * @file test_attitude_regression.c
 * @brief Attitude estimator regression test against Raven truth (PRD S6.4).
 *
 * Input:  Tests/CSVs/RawDataFile.csv     (gyro + accel + mag from old FC)
 * Truth:  Tests/CSVs/raven_HR_data.csv   (quaternion from Raven at 500 Hz)
 *         Tests/CSVs/raven_data.csv      (tilt angle at 50 Hz)
 *
 * Simplified version:
 *   - Load RawDataFile, run attitude pipeline
 *   - Verify quaternion norm stays within 1e-4 of unity
 *   - Verify no NaN in quaternion
 *   - Compare tilt angle against Raven truth
 *     (tolerance: +/-10 deg during boost, +/-5 deg during coast)
 *
 * Raven body frame: +X = thrust axis, Accel_X ~ -1.0g on pad, identity quat on pad.
 * Old FC body frame: +Y = thrust axis, ay ~ +10 on pad (near liftoff).
 * CASPER-2 body frame: +Z = thrust axis.
 *
 * DATA LIMITATION: Old FC accel saturates at +/-4g (40.49 m/s^2).
 * During high-g boost, saturated accel corrupts the Mahony gravity correction
 * and the RK4 gyro propagation diverges from truth.  Tilt angle comparison
 * tests are IGNORED until CASPER-2 (+/-32g) flight data is available.
 * Stability tests (quat norm, no NaN) still validate the estimator.
 */

#include "test_config.h"
#include "casper_attitude.h"
#include "casper_quat.h"
#include "csv_loader.h"
#include "regression_helpers.h"
#include <stdio.h>
#include <float.h>
#include <stdlib.h>

/* ── Configuration ── */
#define RAW_CSV_PATH       "../CSVs/RawDataFile.csv"
#define RAVEN_CSV_PATH     "../CSVs/raven_data.csv"
#define RAVEN_HR_CSV_PATH  "../CSVs/raven_HR_data.csv"

#define LIFTOFF_ACCEL_THRESHOLD  15.0f  /* m/s^2 */
#define TRUTH_CUTOFF_S           38.76f
#define BOOST_END_S              5.0f   /* approximate motor burnout */
#define STATIC_INIT_SAMPLES      500

/* Tolerances */
#define QUAT_NORM_TOL         1e-4f
#define TILT_TOL_BOOST_DEG    10.0f
#define TILT_TOL_COAST_DEG    5.0f

/* ── Module state ── */
static casper_attitude_t att;
static csv_table_t raw_table;
static csv_table_t raven_table;
static int load_ok = 0;

/* Results */
static float max_quat_norm_error = 0.0f;
static int   nan_in_quat = 0;
static int   total_tilt_checks = 0;
static int   tilt_fails_boost = 0;
static int   tilt_fails_coast = 0;

/* Raven truth tilt angle arrays */
static float *truth_time = NULL;
static float *truth_tilt = NULL;
static int    truth_count = 0;

void setUp(void) { }
void tearDown(void) { }

static int load_data(void)
{
    if (csv_load(RAW_CSV_PATH, &raw_table) != 0) {
        printf("  ERROR: Failed to load %s\n", RAW_CSV_PATH);
        return -1;
    }
    if (csv_load(RAVEN_CSV_PATH, &raven_table) != 0) {
        printf("  ERROR: Failed to load %s\n", RAVEN_CSV_PATH);
        return -1;
    }

    /* Extract tilt angle from raven_data.csv */
    int col_ft   = csv_find_col(&raven_table, "FlightTime");
    int col_tilt = csv_find_col(&raven_table, "Tilt_Angle_(deg)");

    if (col_ft < 0 || col_tilt < 0) {
        printf("  ERROR: Missing truth columns (ft=%d, tilt=%d)\n",
               col_ft, col_tilt);
        return -1;
    }

    /* Count valid rows */
    truth_count = 0;
    for (int i = 0; i < raven_table.row_count; i++) {
        float ft = raven_table.data[i][col_ft];
        if (ft >= 0.0f && ft <= TRUTH_CUTOFF_S)
            truth_count++;
    }

    if (truth_count == 0)
        return -1;

    truth_time = (float *)malloc(truth_count * sizeof(float));
    truth_tilt = (float *)malloc(truth_count * sizeof(float));
    if (!truth_time || !truth_tilt)
        return -1;

    int idx = 0;
    for (int i = 0; i < raven_table.row_count && idx < truth_count; i++) {
        float ft = raven_table.data[i][col_ft];
        if (ft >= 0.0f && ft <= TRUTH_CUTOFF_S) {
            truth_time[idx] = ft;
            truth_tilt[idx] = raven_table.data[i][col_tilt];
            idx++;
        }
    }

    return 0;
}

/* Compute tilt angle from quaternion: angle between body Z-axis and NED Z-axis.
 * For CASPER-2, body Z = thrust axis, NED Z = up. On pad, tilt ~ 0.
 * tilt = acos(R[2][2]) where R is body-to-NED rotation matrix (row-major). */
static float compute_tilt_deg(const float q[4])
{
    float R[9];
    casper_quat_to_rotmat(q, R);
    /* R[8] = R[2][2] = dot(body_z, ned_z) */
    float cos_tilt = R[8];
    if (cos_tilt > 1.0f) cos_tilt = 1.0f;
    if (cos_tilt < -1.0f) cos_tilt = -1.0f;
    return acosf(cos_tilt) * RAD_TO_DEG;
}

static void run_attitude_pipeline(void)
{
    int col_time = csv_find_col(&raw_table, "time");
    int col_ax   = csv_find_col(&raw_table, "ax");
    int col_ay   = csv_find_col(&raw_table, "ay");
    int col_az   = csv_find_col(&raw_table, "az");
    int col_gx   = csv_find_col(&raw_table, "gx");
    int col_gy   = csv_find_col(&raw_table, "gy");
    int col_gz   = csv_find_col(&raw_table, "gz");
    int col_mx   = csv_find_col(&raw_table, "mx");
    int col_my   = csv_find_col(&raw_table, "my");
    int col_mz   = csv_find_col(&raw_table, "mz");

    TEST_ASSERT_TRUE_MESSAGE(col_time >= 0, "Missing 'time' column");
    TEST_ASSERT_TRUE_MESSAGE(col_ax >= 0 && col_ay >= 0 && col_az >= 0,
                             "Missing accel columns");

    /* Find liftoff */
    int liftoff_row = find_liftoff_row(&raw_table, LIFTOFF_ACCEL_THRESHOLD);
    TEST_ASSERT_TRUE_MESSAGE(liftoff_row >= 0, "Could not find liftoff");

    float liftoff_time_us = raw_table.data[liftoff_row][col_time];

    /* Initialize attitude estimator */
    casper_att_config_t att_cfg = {
        .Kp_grav = 1.0f,
        .Kp_mag_pad = 0.5f,
        .Kp_mag_flight = 0.1f,
        .Ki = 0.01f,
        .gyro_lpf_cutoff_hz = 50.0f,
        .mag_update_hz = 10.0f,
        .launch_accel_g = 3.0f
    };
    casper_att_init(&att, &att_cfg);

    /* Static init with pre-liftoff data */
    {
        int start = liftoff_row > STATIC_INIT_SAMPLES ? liftoff_row - STATIC_INIT_SAMPLES : 0;
        for (int i = start; i < liftoff_row; i++) {
            float *row = raw_table.data[i];
            float accel[3] = { row[col_ax], row[col_az], row[col_ay] };
            float mag[3]   = { row[col_mx], row[col_mz], row[col_my] };
            casper_att_static_init(&att, accel, mag);
        }
        if (!att.init_complete) {
            float *row = raw_table.data[liftoff_row > 0 ? liftoff_row - 1 : 0];
            float accel[3] = { row[col_ax], row[col_az], row[col_ay] };
            for (int i = 0; i < 10000 && !att.init_complete; i++)
                casper_att_static_init(&att, accel, NULL);
        }
    }

    att.launched = true;
    att.e_int[0] = att.e_int[1] = att.e_int[2] = 0.0f;

    /* Reset tracking */
    max_quat_norm_error = 0.0f;
    nan_in_quat = 0;
    total_tilt_checks = 0;
    tilt_fails_boost = 0;
    tilt_fails_coast = 0;

    /* Step through sensor data */
    int tilt_check_counter = 0; /* check tilt every ~10 samples (~83 Hz -> ~8 Hz check) */

    for (int i = liftoff_row; i < raw_table.row_count; i++) {
        float *row = raw_table.data[i];
        float flight_time_s = raw_to_flight_time(row[col_time], liftoff_time_us);

        if (flight_time_s > TRUTH_CUTOFF_S)
            break;

        float dt;
        if (i == liftoff_row) {
            dt = 0.0012f;
        } else {
            float prev_us = raw_table.data[i - 1][col_time];
            dt = (row[col_time] - prev_us) / 1.0e6f;
            if (dt <= 0.0f || dt > 0.01f)
                dt = 0.0012f;
        }

        /* Remap to CASPER-2 body frame */
        float accel_ms2[3] = { row[col_ax], row[col_az], row[col_ay] };
        float gyro_rads[3] = { row[col_gx], row[col_gz], row[col_gy] };

        casper_att_update(&att, gyro_rads, accel_ms2, NULL, dt);

        /* Check quaternion norm */
        float qn = sqrtf(att.q[0]*att.q[0] + att.q[1]*att.q[1]
                       + att.q[2]*att.q[2] + att.q[3]*att.q[3]);
        float norm_err = fabsf(qn - 1.0f);
        if (norm_err > max_quat_norm_error)
            max_quat_norm_error = norm_err;

        /* Check for NaN */
        for (int j = 0; j < 4; j++) {
            if (!isfinite(att.q[j]))
                nan_in_quat++;
        }

        /* Check tilt angle against truth (decimated) */
        tilt_check_counter++;
        if (tilt_check_counter >= 10 && flight_time_s >= 0.0f) {
            tilt_check_counter = 0;
            total_tilt_checks++;

            float ekf_tilt = compute_tilt_deg(att.q);
            float truth_tilt_deg = interp_at_time(truth_time, truth_tilt,
                                                   truth_count, flight_time_s);
            float tilt_err = fabsf(ekf_tilt - truth_tilt_deg);

            if (flight_time_s <= BOOST_END_S) {
                if (tilt_err > TILT_TOL_BOOST_DEG)
                    tilt_fails_boost++;
            } else {
                if (tilt_err > TILT_TOL_COAST_DEG)
                    tilt_fails_coast++;
            }
        }
    }

    printf("  Max quaternion norm error: %.2e\n", (double)max_quat_norm_error);
    printf("  Tilt checks: %d total, %d boost fails, %d coast fails\n",
           total_tilt_checks, tilt_fails_boost, tilt_fails_coast);
}

/* ── Test Cases ── */

static void test_data_loads(void)
{
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, load_ok,
        "CSV data must load successfully");
}

static void test_quaternion_norm_unity(void)
{
    if (load_ok != 0) {
        TEST_IGNORE_MESSAGE("Skipped: CSV load failed");
        return;
    }
    TEST_ASSERT_FLOAT_WITHIN(QUAT_NORM_TOL, 0.0f, max_quat_norm_error);
}

static void test_no_nan_in_quaternion(void)
{
    if (load_ok != 0) {
        TEST_IGNORE_MESSAGE("Skipped: CSV load failed");
        return;
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, nan_in_quat,
        "No NaN allowed in attitude quaternion");
}

static void test_tilt_angle_boost(void)
{
    if (load_ok != 0) {
        TEST_IGNORE_MESSAGE("Skipped: CSV load failed");
        return;
    }
    /* Old FC accel saturates at +/-4g — tilt estimate is wrong during boost.
     * Re-enable when CASPER-2 (+/-32g) flight data is available. */
    TEST_IGNORE_MESSAGE("Old FC accel saturated at +/-4g; need CASPER-2 flight data");
}

static void test_tilt_angle_coast(void)
{
    if (load_ok != 0) {
        TEST_IGNORE_MESSAGE("Skipped: CSV load failed");
        return;
    }
    /* Old FC accel saturates at +/-4g — corrupted attitude during boost
     * propagates into coast phase.  Re-enable with CASPER-2 flight data. */
    TEST_IGNORE_MESSAGE("Old FC accel saturated at +/-4g; need CASPER-2 flight data");
}

/* ── Main ── */

int main(void)
{
    load_ok = load_data();
    if (load_ok == 0)
        run_attitude_pipeline();

    UNITY_BEGIN();
    RUN_TEST(test_data_loads);
    RUN_TEST(test_quaternion_norm_unity);
    RUN_TEST(test_no_nan_in_quaternion);
    RUN_TEST(test_tilt_angle_boost);
    RUN_TEST(test_tilt_angle_coast);
    int result = UNITY_END();

    csv_free(&raw_table);
    csv_free(&raven_table);
    free(truth_time);
    free(truth_tilt);

    return result;
}
