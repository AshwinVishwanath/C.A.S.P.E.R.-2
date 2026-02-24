/* Unity Test Framework - Internals (Minimal vendored version)
 * Based on ThrowTheSwitch Unity (MIT License)
 * Stripped down for C.A.S.P.E.R.-2 host-side testing */

#ifndef UNITY_INTERNALS_H
#define UNITY_INTERNALS_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

/* ---- Configuration ---- */
#ifndef UNITY_INCLUDE_FLOAT
#define UNITY_INCLUDE_FLOAT
#endif
#ifndef UNITY_INCLUDE_DOUBLE
#define UNITY_INCLUDE_DOUBLE
#endif

typedef float  UNITY_FLOAT;
typedef double UNITY_DOUBLE;

#ifndef UNITY_FLOAT_PRECISION
#define UNITY_FLOAT_PRECISION 1e-6f
#endif
#ifndef UNITY_DOUBLE_PRECISION
#define UNITY_DOUBLE_PRECISION 1e-10
#endif

/* ---- Internal types ---- */
typedef int32_t  UNITY_INT;
typedef uint32_t UNITY_UINT;
typedef int64_t  UNITY_INT64;
typedef uint64_t UNITY_UINT64;

typedef enum {
    UNITY_DISPLAY_STYLE_INT     = 0,
    UNITY_DISPLAY_STYLE_UINT    = 1,
    UNITY_DISPLAY_STYLE_HEX8   = 2,
    UNITY_DISPLAY_STYLE_HEX16  = 3,
    UNITY_DISPLAY_STYLE_HEX32  = 4,
} UNITY_DISPLAY_STYLE_T;

/* ---- Internal state ---- */
struct UNITY_STORAGE_T {
    const char *TestFile;
    const char *CurrentTestName;
    UNITY_UINT CurrentTestLineNumber;
    UNITY_UINT NumberOfTests;
    UNITY_UINT TestFailures;
    UNITY_UINT TestIgnores;
    UNITY_UINT CurrentTestFailed;
    UNITY_UINT CurrentTestIgnored;
};

extern struct UNITY_STORAGE_T Unity;

/* ---- Internal functions ---- */
void UnityBegin(const char *filename);
int  UnityEnd(void);
void UnityDefaultTestRun(void (*func)(void), const char *name, int line);

void UnityAssertEqualNumber(UNITY_INT expected, UNITY_INT actual,
                            const char *msg, UNITY_UINT line,
                            UNITY_DISPLAY_STYLE_T style);
void UnityAssertBits(UNITY_INT mask, UNITY_INT expected, UNITY_INT actual,
                     const char *msg, UNITY_UINT line);
void UnityAssertNull(const void *pointer, const char *msg, UNITY_UINT line);
void UnityAssertNotNull(const void *pointer, const char *msg, UNITY_UINT line);
void UnityAssertEqualString(const char *expected, const char *actual,
                            const char *msg, UNITY_UINT line);
void UnityAssertFloatWithin(UNITY_FLOAT delta, UNITY_FLOAT expected,
                            UNITY_FLOAT actual, const char *msg,
                            UNITY_UINT line);
void UnityAssertDoubleWithin(UNITY_DOUBLE delta, UNITY_DOUBLE expected,
                             UNITY_DOUBLE actual, const char *msg,
                             UNITY_UINT line);
void UnityFail(const char *msg, UNITY_UINT line);

#endif /* UNITY_INTERNALS_H */
