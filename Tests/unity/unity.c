/* Unity Test Framework - Implementation (Minimal vendored version)
 * Based on ThrowTheSwitch Unity (MIT License) */

#include "unity.h"
#include <stdio.h>
#include <string.h>
#include <setjmp.h>

struct UNITY_STORAGE_T Unity;

static jmp_buf unity_abort_jmp;

void UnityBegin(const char *filename)
{
    Unity.TestFile = filename;
    Unity.CurrentTestName = NULL;
    Unity.CurrentTestLineNumber = 0;
    Unity.NumberOfTests = 0;
    Unity.TestFailures = 0;
    Unity.TestIgnores = 0;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
}

int UnityEnd(void)
{
    printf("\n-----------------------\n");
    printf("%u Tests %u Failures %u Ignored\n",
           (unsigned)Unity.NumberOfTests,
           (unsigned)Unity.TestFailures,
           (unsigned)Unity.TestIgnores);
    if (Unity.TestFailures == 0) {
        printf("OK\n");
    } else {
        printf("FAIL\n");
    }
    return (int)Unity.TestFailures;
}

void UnityDefaultTestRun(void (*func)(void), const char *name, int line)
{
    Unity.CurrentTestName = name;
    Unity.CurrentTestLineNumber = (UNITY_UINT)line;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
    Unity.NumberOfTests++;

    setUp();

    if (setjmp(unity_abort_jmp) == 0) {
        func();
    }

    tearDown();

    if (Unity.CurrentTestFailed) {
        Unity.TestFailures++;
        printf("FAIL\n");
    } else if (Unity.CurrentTestIgnored) {
        Unity.TestIgnores++;
        printf("IGNORED\n");
    } else {
        printf("%s:%d:%s:PASS\n", Unity.TestFile, line, name);
    }
}

void UnityFail(const char *msg, UNITY_UINT line)
{
    printf("%s:%u:%s:FAIL",
           Unity.TestFile,
           (unsigned)line,
           Unity.CurrentTestName ? Unity.CurrentTestName : "");
    if (msg) {
        printf(": %s", msg);
    }
    printf("\n");
    Unity.CurrentTestFailed = 1;
    longjmp(unity_abort_jmp, 1);
}

void UnityAssertEqualNumber(UNITY_INT expected, UNITY_INT actual,
                            const char *msg, UNITY_UINT line,
                            UNITY_DISPLAY_STYLE_T style)
{
    if (expected != actual) {
        char buf[128];
        switch (style) {
        case UNITY_DISPLAY_STYLE_HEX8:
            snprintf(buf, sizeof(buf), "Expected 0x%02X Was 0x%02X",
                     (unsigned)(expected & 0xFF), (unsigned)(actual & 0xFF));
            break;
        case UNITY_DISPLAY_STYLE_HEX16:
            snprintf(buf, sizeof(buf), "Expected 0x%04X Was 0x%04X",
                     (unsigned)(expected & 0xFFFF), (unsigned)(actual & 0xFFFF));
            break;
        case UNITY_DISPLAY_STYLE_HEX32:
            snprintf(buf, sizeof(buf), "Expected 0x%08X Was 0x%08X",
                     (unsigned)expected, (unsigned)actual);
            break;
        case UNITY_DISPLAY_STYLE_UINT:
            snprintf(buf, sizeof(buf), "Expected %u Was %u",
                     (unsigned)expected, (unsigned)actual);
            break;
        default:
            snprintf(buf, sizeof(buf), "Expected %d Was %d",
                     (int)expected, (int)actual);
            break;
        }
        if (msg) {
            char full[256];
            snprintf(full, sizeof(full), "%s %s", buf, msg);
            UnityFail(full, line);
        } else {
            UnityFail(buf, line);
        }
    }
}

void UnityAssertBits(UNITY_INT mask, UNITY_INT expected, UNITY_INT actual,
                     const char *msg, UNITY_UINT line)
{
    if ((expected & mask) != (actual & mask)) {
        UnityFail(msg ? msg : "Bit mismatch", line);
    }
}

void UnityAssertNull(const void *pointer, const char *msg, UNITY_UINT line)
{
    if (pointer != NULL) {
        UnityFail(msg ? msg : "Expected NULL", line);
    }
}

void UnityAssertNotNull(const void *pointer, const char *msg, UNITY_UINT line)
{
    if (pointer == NULL) {
        UnityFail(msg ? msg : "Expected Not NULL", line);
    }
}

void UnityAssertEqualString(const char *expected, const char *actual,
                            const char *msg, UNITY_UINT line)
{
    if (expected == NULL && actual == NULL) return;
    if (expected == NULL || actual == NULL || strcmp(expected, actual) != 0) {
        char buf[256];
        snprintf(buf, sizeof(buf), "Expected '%s' Was '%s'",
                 expected ? expected : "(null)",
                 actual ? actual : "(null)");
        if (msg) {
            char full[512];
            snprintf(full, sizeof(full), "%s %s", buf, msg);
            UnityFail(full, line);
        } else {
            UnityFail(buf, line);
        }
    }
}

void UnityAssertFloatWithin(UNITY_FLOAT delta, UNITY_FLOAT expected,
                            UNITY_FLOAT actual, const char *msg,
                            UNITY_UINT line)
{
    float diff = expected - actual;
    if (diff < 0.0f) diff = -diff;
    if (delta < 0.0f) delta = -delta;

    if (!(diff <= delta)) {  /* NaN-safe: NaN comparison yields false */
        char buf[128];
        snprintf(buf, sizeof(buf), "Expected %.9g +/- %.9g Was %.9g",
                 (double)expected, (double)delta, (double)actual);
        if (msg) {
            char full[256];
            snprintf(full, sizeof(full), "%s %s", buf, msg);
            UnityFail(full, line);
        } else {
            UnityFail(buf, line);
        }
    }
}

void UnityAssertDoubleWithin(UNITY_DOUBLE delta, UNITY_DOUBLE expected,
                             UNITY_DOUBLE actual, const char *msg,
                             UNITY_UINT line)
{
    double diff = expected - actual;
    if (diff < 0.0) diff = -diff;
    if (delta < 0.0) delta = -delta;

    if (!(diff <= delta)) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Expected %.15g +/- %.15g Was %.15g",
                 expected, delta, actual);
        if (msg) {
            char full[256];
            snprintf(full, sizeof(full), "%s %s", buf, msg);
            UnityFail(full, line);
        } else {
            UnityFail(buf, line);
        }
    }
}
