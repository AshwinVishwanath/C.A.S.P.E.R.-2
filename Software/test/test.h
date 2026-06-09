/*
 * test.h — tiny zero-dependency unit-test harness for host-side (gcc) testing
 * of pure-logic flight-firmware modules.
 *
 * These tests are CHARACTERIZATION / GOLDEN tests: they pin the CURRENT
 * observable behaviour of a module so that dead-code removal / de-duplication
 * refactors can be proven behaviour-preserving. They are compiled and run with
 * the host gcc (NOT the arm toolchain) and never touch HAL/CMSIS.
 *
 * Usage:
 *   #include "test.h"
 *   TEST(name) { ASSERT_EQ_INT(actual, expected); }
 *   int main(void) { RUN(name); return test_summary(); }
 */
#ifndef CASPER_TEST_H
#define CASPER_TEST_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

static int g_tests = 0;
static int g_failed = 0;
static int g_cur_fail = 0;

#define TEST(name) static void test_##name(void)

#define RUN(name) do {                                                  \
    g_cur_fail = 0;                                                     \
    g_tests++;                                                         \
    test_##name();                                                     \
    if (g_cur_fail) { g_failed++; printf("[FAIL] %s\n", #name); }       \
    else            { printf("[ OK ] %s\n", #name); }                   \
} while (0)

#define FAILMSG(...) do {                                               \
    g_cur_fail = 1;                                                    \
    printf("       %s:%d: ", __FILE__, __LINE__);                      \
    printf(__VA_ARGS__);                                               \
    printf("\n");                                                      \
} while (0)

#define ASSERT_TRUE(cond) do {                                         \
    if (!(cond)) FAILMSG("ASSERT_TRUE(%s) failed", #cond);             \
} while (0)

#define ASSERT_EQ_INT(a, b) do {                                       \
    long _a = (long)(a), _b = (long)(b);                              \
    if (_a != _b) FAILMSG("ASSERT_EQ_INT(%s,%s): %ld != %ld",          \
                          #a, #b, _a, _b);                            \
} while (0)

#define ASSERT_EQ_U(a, b) do {                                         \
    unsigned long _a = (unsigned long)(a), _b = (unsigned long)(b);    \
    if (_a != _b) FAILMSG("ASSERT_EQ_U(%s,%s): %lu != %lu",            \
                          #a, #b, _a, _b);                            \
} while (0)

/* compare two byte buffers */
#define ASSERT_EQ_MEM(a, b, n) do {                                    \
    if (memcmp((a), (b), (n)) != 0) {                                 \
        FAILMSG("ASSERT_EQ_MEM(%s,%s,%s) differ", #a, #b, #n);         \
        for (int _i = 0; _i < (int)(n); _i++)                         \
            printf("         [%d] %02X vs %02X%s\n", _i,               \
                   ((const uint8_t*)(a))[_i], ((const uint8_t*)(b))[_i],\
                   ((const uint8_t*)(a))[_i]==((const uint8_t*)(b))[_i] \
                   ? "" : "  <--");                                    \
    }                                                                 \
} while (0)

/* float compare within tolerance */
#define ASSERT_NEAR(a, b, tol) do {                                    \
    double _a = (double)(a), _b = (double)(b);                        \
    if (fabs(_a - _b) > (tol))                                        \
        FAILMSG("ASSERT_NEAR(%s,%s,%g): %.9g vs %.9g (d=%.3g)",        \
                #a, #b, (double)(tol), _a, _b, fabs(_a-_b));          \
} while (0)

static int test_summary(void)
{
    printf("----------------------------------------\n");
    printf("%d tests, %d failed\n", g_tests, g_failed);
    return g_failed ? 1 : 0;
}

#endif /* CASPER_TEST_H */
