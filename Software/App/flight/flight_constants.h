#ifndef FLIGHT_CONSTANTS_H
#define FLIGHT_CONSTANTS_H

/* ── Launch detection ──────────────────────────────────────────── */
#define LAUNCH_ACCEL_THRESHOLD_MS2     39.2f    /* 4g */
#define LAUNCH_DEBOUNCE_MS             500

/* ── Burnout detection ─────────────────────────────────────────── */
#define BURNOUT_ACCEL_THRESHOLD_MS2    14.7f    /* 1.5g */
#define BURNOUT_DEBOUNCE_MS            200
#define BURNOUT_MIN_BOOST_TIME_MS      500
#define BURNOUT_MIN_VELOCITY_MPS       10.0f

/* ── Staging detection ─────────────────────────────────────────── */
#define STAGING_ACCEL_THRESHOLD_MS2    29.4f    /* 3g */
#define STAGING_DEBOUNCE_MS            200

/* ── Apogee detection ──────────────────────────────────────────── */
#define APOGEE_VEL_THRESHOLD_MPS       (-1.0f)
#define APOGEE_ALT_MARGIN_M            2.0f
#define APOGEE_DEBOUNCE_MS             300
#define APOGEE_MIN_COAST_TIME_MS       2000
#define APOGEE_MIN_ALTITUDE_M          20.0f

/* ── Landing detection ─────────────────────────────────────────── */
#define LANDING_VEL_THRESHOLD_MPS      2.0f
#define LANDING_ALT_STABLE_M           1.0f
#define LANDING_DEBOUNCE_MS            5000
#define LANDING_MIN_TIME_AFTER_APOGEE_MS 10000

/* ── Main deploy safety ────────────────────────────────────────── */
#define MAIN_DEPLOY_MIN_TIME_AFTER_APOGEE_MS 3000

/* ── Logging ───────────────────────────────────────────────────── */
#define LOG_HIGH_VEL_THRESHOLD_MPS     15.0f

#endif /* FLIGHT_CONSTANTS_H */
