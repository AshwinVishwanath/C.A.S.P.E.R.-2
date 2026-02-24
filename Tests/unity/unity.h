/* Unity Test Framework - Public API (Minimal vendored version)
 * Based on ThrowTheSwitch Unity (MIT License) */

#ifndef UNITY_FRAMEWORK_H
#define UNITY_FRAMEWORK_H

#include "unity_internals.h"

/* ---- Test lifecycle ---- */
#define UNITY_BEGIN()    UnityBegin(__FILE__)
#define UNITY_END()      UnityEnd()

#define RUN_TEST(func)   UnityDefaultTestRun(func, #func, __LINE__)

/* setUp/tearDown are called by UnityDefaultTestRun if defined */
void setUp(void);
void tearDown(void);

/* ---- Boolean assertions ---- */
#define TEST_ASSERT(cond)               do { if (!(cond)) { UnityFail("Expected TRUE", __LINE__); } } while(0)
#define TEST_ASSERT_TRUE(cond)          TEST_ASSERT(cond)
#define TEST_ASSERT_FALSE(cond)         do { if (cond) { UnityFail("Expected FALSE", __LINE__); } } while(0)

#define TEST_ASSERT_TRUE_MESSAGE(cond, msg) \
    do { if (!(cond)) { UnityFail(msg, __LINE__); } } while(0)
#define TEST_ASSERT_FALSE_MESSAGE(cond, msg) \
    do { if (cond) { UnityFail(msg, __LINE__); } } while(0)

/* ---- Integer assertions ---- */
#define TEST_ASSERT_EQUAL_INT(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(exp), (UNITY_INT)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT)
#define TEST_ASSERT_EQUAL_INT32(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(exp), (UNITY_INT)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT)
#define TEST_ASSERT_EQUAL_UINT(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(exp), (UNITY_INT)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT)
#define TEST_ASSERT_EQUAL_UINT8(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(uint8_t)(exp), (UNITY_INT)(uint8_t)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT)
#define TEST_ASSERT_EQUAL_UINT16(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(uint16_t)(exp), (UNITY_INT)(uint16_t)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT)
#define TEST_ASSERT_EQUAL_UINT32(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(uint32_t)(exp), (UNITY_INT)(uint32_t)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT)

/* ---- Hex assertions ---- */
#define TEST_ASSERT_EQUAL_HEX8(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(uint8_t)(exp), (UNITY_INT)(uint8_t)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_HEX8)
#define TEST_ASSERT_EQUAL_HEX16(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(uint16_t)(exp), (UNITY_INT)(uint16_t)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_HEX16)
#define TEST_ASSERT_EQUAL_HEX32(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(uint32_t)(exp), (UNITY_INT)(uint32_t)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_HEX32)

/* ---- Pointer assertions ---- */
#define TEST_ASSERT_NULL(ptr)           UnityAssertNull((const void *)(ptr), NULL, __LINE__)
#define TEST_ASSERT_NOT_NULL(ptr)       UnityAssertNotNull((const void *)(ptr), NULL, __LINE__)

/* ---- String assertions ---- */
#define TEST_ASSERT_EQUAL_STRING(exp, act) \
    UnityAssertEqualString((exp), (act), NULL, __LINE__)

/* ---- Float assertions ---- */
#define TEST_ASSERT_FLOAT_WITHIN(delta, exp, act) \
    UnityAssertFloatWithin((UNITY_FLOAT)(delta), (UNITY_FLOAT)(exp), (UNITY_FLOAT)(act), NULL, __LINE__)
#define TEST_ASSERT_FLOAT_WITHIN_MESSAGE(delta, exp, act, msg) \
    UnityAssertFloatWithin((UNITY_FLOAT)(delta), (UNITY_FLOAT)(exp), (UNITY_FLOAT)(act), msg, __LINE__)
#define TEST_ASSERT_EQUAL_FLOAT(exp, act) \
    TEST_ASSERT_FLOAT_WITHIN(UNITY_FLOAT_PRECISION, (exp), (act))

/* ---- Double assertions ---- */
#define TEST_ASSERT_DOUBLE_WITHIN(delta, exp, act) \
    UnityAssertDoubleWithin((UNITY_DOUBLE)(delta), (UNITY_DOUBLE)(exp), (UNITY_DOUBLE)(act), NULL, __LINE__)
#define TEST_ASSERT_EQUAL_DOUBLE(exp, act) \
    TEST_ASSERT_DOUBLE_WITHIN(UNITY_DOUBLE_PRECISION, (exp), (act))

/* ---- Not-equal assertions ---- */
#define TEST_ASSERT_NOT_EQUAL(exp, act) \
    TEST_ASSERT_TRUE_MESSAGE((UNITY_INT)(exp) != (UNITY_INT)(act), "Expected not equal")
#define TEST_ASSERT_NOT_EQUAL_MESSAGE(exp, act, msg) \
    TEST_ASSERT_TRUE_MESSAGE((UNITY_INT)(exp) != (UNITY_INT)(act), msg)

/* ---- Generic equal (alias for EQUAL_INT) ---- */
#define TEST_ASSERT_EQUAL(exp, act) \
    UnityAssertEqualNumber((UNITY_INT)(exp), (UNITY_INT)(act), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT)

/* ---- Message variants ---- */
#define TEST_ASSERT_EQUAL_INT_MESSAGE(exp, act, msg) \
    UnityAssertEqualNumber((UNITY_INT)(exp), (UNITY_INT)(act), msg, __LINE__, UNITY_DISPLAY_STYLE_INT)
#define TEST_ASSERT_EQUAL_UINT_MESSAGE(exp, act, msg) \
    UnityAssertEqualNumber((UNITY_INT)(exp), (UNITY_INT)(act), msg, __LINE__, UNITY_DISPLAY_STYLE_UINT)
#define TEST_ASSERT_EQUAL_UINT8_MESSAGE(exp, act, msg) \
    UnityAssertEqualNumber((UNITY_INT)(uint8_t)(exp), (UNITY_INT)(uint8_t)(act), msg, __LINE__, UNITY_DISPLAY_STYLE_UINT)

/* ---- Greater/less with message ---- */
#define TEST_ASSERT_GREATER_THAN_MESSAGE(threshold, actual, msg) \
    TEST_ASSERT_TRUE_MESSAGE((actual) > (threshold), msg)
#define TEST_ASSERT_LESS_THAN_MESSAGE(threshold, actual, msg) \
    TEST_ASSERT_TRUE_MESSAGE((actual) < (threshold), msg)
#define TEST_ASSERT_GREATER_OR_EQUAL(threshold, actual) \
    TEST_ASSERT_TRUE_MESSAGE((actual) >= (threshold), "Expected >= threshold")

/* ---- Failure ---- */
#define TEST_FAIL()                 UnityFail(NULL, __LINE__)
#define TEST_FAIL_MESSAGE(msg)      UnityFail(msg, __LINE__)

/* ---- Ignore ---- */
#define TEST_IGNORE()               do { Unity.CurrentTestIgnored = 1; return; } while(0)
#define TEST_IGNORE_MESSAGE(msg)    do { (void)(msg); Unity.CurrentTestIgnored = 1; return; } while(0)

/* ---- Comparison helpers (not assertions, return bool) ---- */
#define TEST_ASSERT_GREATER_THAN(threshold, actual) \
    TEST_ASSERT_TRUE_MESSAGE((actual) > (threshold), "Expected greater than threshold")
#define TEST_ASSERT_LESS_THAN(threshold, actual) \
    TEST_ASSERT_TRUE_MESSAGE((actual) < (threshold), "Expected less than threshold")

/* ---- Memory comparison ---- */
#define TEST_ASSERT_EQUAL_MEMORY(exp, act, len) \
    do { \
        const uint8_t *_e = (const uint8_t *)(exp); \
        const uint8_t *_a = (const uint8_t *)(act); \
        for (size_t _i = 0; _i < (size_t)(len); _i++) { \
            if (_e[_i] != _a[_i]) { UnityFail("Memory mismatch", __LINE__); break; } \
        } \
    } while(0)

#endif /* UNITY_FRAMEWORK_H */
