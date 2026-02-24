/* test_config.h — Shared configuration for all C.A.S.P.E.R.-2 test files */

#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define UNITY_INCLUDE_FLOAT
#define UNITY_INCLUDE_DOUBLE
#define UNITY_FLOAT_PRECISION 1e-6f
#define UNITY_DOUBLE_PRECISION 1e-10

#include "unity.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* ---- Custom assertion: float array within tolerance ---- */
#define TEST_ASSERT_FLOAT_ARRAY_WITHIN(delta, expected, actual, len) \
    do { for (int _i = 0; _i < (int)(len); _i++) { \
        TEST_ASSERT_FLOAT_WITHIN((delta), (expected)[_i], (actual)[_i]); \
    } } while(0)

/* ---- Custom assertion: quaternion comparison (handles q == -q ambiguity) ---- */
#define TEST_ASSERT_QUAT_EQUAL(expected, actual, tol) \
    do { \
        float _dot = (expected)[0]*(actual)[0] + (expected)[1]*(actual)[1] + \
                     (expected)[2]*(actual)[2] + (expected)[3]*(actual)[3]; \
        TEST_ASSERT_FLOAT_WITHIN((tol), 1.0f, fabsf(_dot)); \
    } while(0)

/* ---- Assert no NaN or Inf in array ---- */
#define TEST_ASSERT_ALL_FINITE(arr, len) \
    do { for (int _i = 0; _i < (int)(len); _i++) { \
        TEST_ASSERT_TRUE_MESSAGE(isfinite((arr)[_i]), "NaN/Inf detected"); \
    } } while(0)

/* ---- Assert matrix symmetry ---- */
#define TEST_ASSERT_MATRIX_SYMMETRIC(mat, n, tol) \
    do { for (int _r = 0; _r < (int)(n); _r++) { \
        for (int _c = _r + 1; _c < (int)(n); _c++) { \
            TEST_ASSERT_FLOAT_WITHIN((tol), (mat)[_r*(n)+_c], (mat)[_c*(n)+_r]); \
        } \
    } } while(0)

/* ---- Assert all diagonal elements positive ---- */
#define TEST_ASSERT_DIAG_POSITIVE(mat, n) \
    do { for (int _i = 0; _i < (int)(n); _i++) { \
        TEST_ASSERT_TRUE_MESSAGE((mat)[_i*(n)+_i] > 0.0f, "Negative diagonal in covariance"); \
    } } while(0)

/* ---- Tolerance constants (from PRD §9.2) ---- */
#define TOL_QUAT_MATH       1e-7f
#define TOL_EULER_DEG        0.01f
#define TOL_EKF_STATIC_M     0.001f
#define TOL_EKF_FLIGHT_PCT   0.05f   /* 5% of apogee */
#define TOL_TIMING_S         0.5f

/* ---- Conversion constants ---- */
#define FT_TO_M              0.3048f
#define FPS_TO_MPS            0.3048f
#define DEG_TO_RAD           (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG           (180.0f / 3.14159265358979323846f)

/* ---- Pi constant ---- */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PIf
#define M_PIf 3.14159265f
#endif

#endif /* TEST_CONFIG_H */
