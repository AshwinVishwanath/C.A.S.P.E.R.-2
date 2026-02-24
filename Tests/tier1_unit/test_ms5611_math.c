/**
 * @file test_ms5611_math.c
 * @brief Unit tests for MS5611 math/conversion functions (PRD S3.7).
 *
 * ms5611_compute is static, so we test indirectly through the public accessors
 * by directly setting the internal struct fields (temperature, pressure).
 *
 * Tested functions:
 *   ms5611_get_temperature: (float)dev->temperature * 0.01f
 *   ms5611_get_pressure:    (float)dev->pressure * 0.01f
 *   ms5611_get_altitude:    44307.694 * (1 - pow(P/P0, 0.190284))
 */

#include "test_config.h"
#include "ms5611.h"

/* ------------------------------------------------------------------ */
static ms5611_t dev;

void setUp(void)
{
    memset(&dev, 0, sizeof(dev));
}

void tearDown(void) { }

/* ================================================================== */
/*  Temperature conversion                                             */
/* ================================================================== */

void test_ms5611_temperature_25C(void)
{
    /* temperature field in 0.01 degC units: 2500 => 25.00 C */
    dev.temperature = 2500;
    float t = ms5611_get_temperature(&dev);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.0f, t);
}

void test_ms5611_temperature_0C(void)
{
    dev.temperature = 0;
    float t = ms5611_get_temperature(&dev);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, t);
}

void test_ms5611_temperature_negative(void)
{
    /* -10.50 C = -1050 */
    dev.temperature = -1050;
    float t = ms5611_get_temperature(&dev);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.5f, t);
}

/* ================================================================== */
/*  Pressure conversion                                                */
/* ================================================================== */

void test_ms5611_pressure_standard(void)
{
    /* pressure field in Pascal: 101325 Pa => 1013.25 hPa */
    dev.pressure = 101325;
    float p = ms5611_get_pressure(&dev);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1013.25f, p);
}

void test_ms5611_pressure_low(void)
{
    /* 90000 Pa => 900.00 hPa */
    dev.pressure = 90000;
    float p = ms5611_get_pressure(&dev);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 900.0f, p);
}

/* ================================================================== */
/*  Altitude computation                                               */
/* ================================================================== */

void test_ms5611_altitude_sea_level(void)
{
    /* At sea level: pressure = 1013.25 hPa, sea_level = 1013.25 hPa => alt ~ 0m
     * pressure stored in Pascal: 101325 */
    dev.pressure = 101325;
    float alt = ms5611_get_altitude(&dev, 1013.25f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, alt);
}

void test_ms5611_altitude_900hPa(void)
{
    /* Pressure 900 hPa at sea_level 1013.25 hPa => ~988m
     * Using hypsometric formula: 44307.694 * (1 - (900/1013.25)^0.190284)
     * = 44307.694 * (1 - 0.97731...) => roughly 988 m
     * Compute exact: ratio = 900/1013.25 = 0.888235
     *   pow(0.888235, 0.190284) = exp(0.190284 * ln(0.888235))
     *   ln(0.888235) ~ -0.11843, * 0.190284 ~ -0.02253
     *   exp(-0.02253) ~ 0.97771
     *   alt = 44307.694 * (1 - 0.97771) ~ 44307.694 * 0.02229 ~ 987.6 m */
    dev.pressure = 90000;
    float alt = ms5611_get_altitude(&dev, 1013.25f);

    /* Use generous tolerance for float precision of hypsometric formula */
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 988.0f, alt);
}

void test_ms5611_altitude_high(void)
{
    /* ~5500m: pressure ~ 505 hPa => 50500 Pa */
    dev.pressure = 50500;
    float alt = ms5611_get_altitude(&dev, 1013.25f);
    /* Should be approximately 5500m, use wide tolerance */
    TEST_ASSERT_TRUE(alt > 5000.0f);
    TEST_ASSERT_TRUE(alt < 6000.0f);
}

/* ================================================================== */
/*  Edge cases                                                         */
/* ================================================================== */

void test_ms5611_altitude_zero_pressure(void)
{
    /* Zero pressure: should be clamped to 0.01 hPa, no NaN */
    dev.pressure = 0;
    float alt = ms5611_get_altitude(&dev, 1013.25f);
    TEST_ASSERT_TRUE(isfinite(alt));
    /* Should produce a very high altitude */
    TEST_ASSERT_TRUE(alt > 0.0f);
}

void test_ms5611_altitude_negative_pressure(void)
{
    /* Negative pressure (error case): clamped to 0.01, no NaN */
    dev.pressure = -100;
    float alt = ms5611_get_altitude(&dev, 1013.25f);
    TEST_ASSERT_TRUE(isfinite(alt));
}

/* ================================================================== */
/*  main()                                                             */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Temperature */
    RUN_TEST(test_ms5611_temperature_25C);
    RUN_TEST(test_ms5611_temperature_0C);
    RUN_TEST(test_ms5611_temperature_negative);

    /* Pressure */
    RUN_TEST(test_ms5611_pressure_standard);
    RUN_TEST(test_ms5611_pressure_low);

    /* Altitude */
    RUN_TEST(test_ms5611_altitude_sea_level);
    RUN_TEST(test_ms5611_altitude_900hPa);
    RUN_TEST(test_ms5611_altitude_high);

    /* Edge cases */
    RUN_TEST(test_ms5611_altitude_zero_pressure);
    RUN_TEST(test_ms5611_altitude_negative_pressure);

    return UNITY_END();
}
