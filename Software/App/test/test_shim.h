#ifndef TEST_SHIM_H
#define TEST_SHIM_H

/*
 * Test shim for host-PC compilation of flight firmware test harnesses.
 * Provides stubs for HAL functions and telemetry functions that are
 * not available outside the STM32 target.
 *
 * Usage: Include this header BEFORE any firmware headers in test files
 * when compiling on a host PC (not on the STM32 target).
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* ── Controllable tick counter ─────────────────────────────────── */
static uint32_t s_test_tick = 0;

static inline uint32_t HAL_GetTick(void)
{
    return s_test_tick;
}

static inline void test_set_tick(uint32_t ms)
{
    s_test_tick = ms;
}

static inline void test_advance_tick(uint32_t delta_ms)
{
    s_test_tick += delta_ms;
}

/* ── Telemetry event stub ──────────────────────────────────────── */
#define TEST_MAX_EVENTS 64

static uint8_t  s_event_types[TEST_MAX_EVENTS];
static uint16_t s_event_data[TEST_MAX_EVENTS];
static int      s_event_count = 0;

static inline int tlm_queue_event(uint8_t type, uint16_t data)
{
    if (s_event_count < TEST_MAX_EVENTS) {
        s_event_types[s_event_count] = type;
        s_event_data[s_event_count] = data;
        s_event_count++;
    }
    return 0;
}

static inline void test_clear_events(void)
{
    s_event_count = 0;
}

/* ── Test assertion macros ─────────────────────────────────────── */
static int s_test_failures = 0;
static int s_test_passes = 0;

#define ASSERT_EQ(actual, expected, msg) do { \
    if ((actual) != (expected)) { \
        printf("  FAIL: %s (got %d, expected %d)\n", msg, (int)(actual), (int)(expected)); \
        s_test_failures++; \
    } else { \
        s_test_passes++; \
    } \
} while (0)

#define ASSERT_TRUE(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s\n", msg); \
        s_test_failures++; \
    } else { \
        s_test_passes++; \
    } \
} while (0)

#define ASSERT_FLOAT_NEAR(actual, expected, tol, msg) do { \
    float _diff = (actual) - (expected); \
    if (_diff < 0) _diff = -_diff; \
    if (_diff > (tol)) { \
        printf("  FAIL: %s (got %.4f, expected %.4f, tol %.4f)\n", msg, \
               (double)(actual), (double)(expected), (double)(tol)); \
        s_test_failures++; \
    } else { \
        s_test_passes++; \
    } \
} while (0)

#define TEST_BEGIN(name) do { \
    printf("TEST: %s\n", name); \
} while (0)

#define TEST_END(name) do { \
    if (s_test_failures == 0) { \
        printf("  PASS: %s\n", name); \
    } \
} while (0)

#define TEST_SUMMARY() do { \
    printf("\n=== RESULTS: %d passed, %d failed ===\n", s_test_passes, s_test_failures); \
    if (s_test_failures > 0) { \
        printf("SOME TESTS FAILED\n"); \
        return 1; \
    } else { \
        printf("ALL TESTS PASSED\n"); \
        return 0; \
    } \
} while (0)

#endif /* TEST_SHIM_H */
