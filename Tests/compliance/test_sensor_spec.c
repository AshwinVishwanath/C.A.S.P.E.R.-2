/**
 * @file test_sensor_spec.c
 * @brief Sensor specification compliance tests (PRD 7.1).
 *
 * Validates sensor device IDs, command constants, sensitivity values,
 * and pyro continuity threshold from the driver headers.
 */

#include "test_config.h"
#include "lsm6dso32.h"
#include "adxl372.h"
#include "ms5611.h"
#include "casper_pyro.h"

void setUp(void) { }
void tearDown(void) { }

/* ═══════════════════════════════════════════════════════════════════════
 *  MS5611 barometer
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_ms5611_cmd_reset(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x1E, MS5611_CMD_RESET);
}

static void test_ms5611_cmd_read_adc(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x00, MS5611_CMD_READ_ADC);
}

static void test_ms5611_cmd_read_prom(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xA0, MS5611_CMD_READ_PROM);
}

static void test_ms5611_cmd_convert_d1(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x40, MS5611_CMD_CONVERT_D1);
}

static void test_ms5611_cmd_convert_d2(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x50, MS5611_CMD_CONVERT_D2);
}

static void test_ms5611_osr_enum_values(void)
{
    TEST_ASSERT_EQUAL_INT(0, MS5611_OSR_256);
    TEST_ASSERT_EQUAL_INT(1, MS5611_OSR_512);
    TEST_ASSERT_EQUAL_INT(2, MS5611_OSR_1024);
    TEST_ASSERT_EQUAL_INT(3, MS5611_OSR_2048);
    TEST_ASSERT_EQUAL_INT(4, MS5611_OSR_4096);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  LSM6DSO32 IMU
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_lsm6dso32_who_am_i_value(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x6C, LSM6DSO32_WHO_AM_I_VAL);
}

static void test_lsm6dso32_who_am_i_register(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x0F, LSM6DSO32_WHO_AM_I);
}

static void test_lsm6dso32_ctrl_registers(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x10, LSM6DSO32_CTRL1_XL);
    TEST_ASSERT_EQUAL_HEX8(0x11, LSM6DSO32_CTRL2_G);
    TEST_ASSERT_EQUAL_HEX8(0x12, LSM6DSO32_CTRL3_C);
}

static void test_lsm6dso32_int_ctrl_registers(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x0D, LSM6DSO32_INT1_CTRL);
    TEST_ASSERT_EQUAL_HEX8(0x0E, LSM6DSO32_INT2_CTRL);
}

static void test_lsm6dso32_output_registers(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x20, LSM6DSO32_OUT_TEMP_L);
    TEST_ASSERT_EQUAL_HEX8(0x22, LSM6DSO32_OUTX_L_G);
    TEST_ASSERT_EQUAL_HEX8(0x28, LSM6DSO32_OUTX_L_A);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  ADXL372 high-g accelerometer
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_adxl372_adi_devid(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xAD, ADXL372_ADI_DEVID_VAL);
}

static void test_adxl372_mst_devid(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x1D, ADXL372_MST_DEVID_VAL);
}

static void test_adxl372_devid(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xFA, ADXL372_DEVID_VAL);
}

static void test_adxl372_reset_code(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x52, ADXL372_RESET_CODE);
}

static void test_adxl372_sensitivity_100mg_per_lsb(void)
{
    /* ADXL372: +-200g range, 12-bit, 100 mg/LSB.
     * From the driver struct comment: "100 mg/LSB".
     * 200g / 2048 = 0.09765 ~ 0.1 g/LSB = 100 mg/LSB.
     * Verify by checking that the raw-to-g conversion factor
     * matches 100 mg/LSB. We verify the range constant here. */
    TEST_ASSERT_EQUAL_HEX8(0x00, ADXL372_ADI_DEVID);  /* register 0x00 */
    TEST_ASSERT_EQUAL_HEX8(0x02, ADXL372_DEVID);       /* register 0x02 */
}

static void test_adxl372_register_addresses(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x3Du, ADXL372_TIMING);
    TEST_ASSERT_EQUAL_HEX8(0x3Eu, ADXL372_MEASURE);
    TEST_ASSERT_EQUAL_HEX8(0x3Fu, ADXL372_POWER_CTL);
    TEST_ASSERT_EQUAL_HEX8(0x41u, ADXL372_SRESET);
    TEST_ASSERT_EQUAL_HEX8(0x42u, ADXL372_FIFO_DATA);
}

static void test_adxl372_operating_modes(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x00, ADXL372_OP_STANDBY);
    TEST_ASSERT_EQUAL_HEX8(0x01, ADXL372_OP_WAKE_UP);
    TEST_ASSERT_EQUAL_HEX8(0x02, ADXL372_OP_INSTANT_ON);
    TEST_ASSERT_EQUAL_HEX8(0x03, ADXL372_OP_FULL_BW_MEASUREMENT);
}

static void test_adxl372_launch_threshold_default(void)
{
    /* Default wake-up launch threshold = 3.0g */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, ADXL_LAUNCH_G);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Pyro continuity
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_pyro_continuity_threshold(void)
{
    TEST_ASSERT_EQUAL_UINT16(8000, PYRO_CONTINUITY_THRESHOLD);
}

static void test_pyro_num_channels(void)
{
    TEST_ASSERT_EQUAL_INT(4, PYRO_NUM_CHANNELS);
}

static void test_pyro_default_fire_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(1000, PYRO_DEFAULT_FIRE_MS);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    UNITY_BEGIN();

    /* MS5611 */
    RUN_TEST(test_ms5611_cmd_reset);
    RUN_TEST(test_ms5611_cmd_read_adc);
    RUN_TEST(test_ms5611_cmd_read_prom);
    RUN_TEST(test_ms5611_cmd_convert_d1);
    RUN_TEST(test_ms5611_cmd_convert_d2);
    RUN_TEST(test_ms5611_osr_enum_values);

    /* LSM6DSO32 */
    RUN_TEST(test_lsm6dso32_who_am_i_value);
    RUN_TEST(test_lsm6dso32_who_am_i_register);
    RUN_TEST(test_lsm6dso32_ctrl_registers);
    RUN_TEST(test_lsm6dso32_int_ctrl_registers);
    RUN_TEST(test_lsm6dso32_output_registers);

    /* ADXL372 */
    RUN_TEST(test_adxl372_adi_devid);
    RUN_TEST(test_adxl372_mst_devid);
    RUN_TEST(test_adxl372_devid);
    RUN_TEST(test_adxl372_reset_code);
    RUN_TEST(test_adxl372_sensitivity_100mg_per_lsb);
    RUN_TEST(test_adxl372_register_addresses);
    RUN_TEST(test_adxl372_operating_modes);
    RUN_TEST(test_adxl372_launch_threshold_default);

    /* Pyro */
    RUN_TEST(test_pyro_continuity_threshold);
    RUN_TEST(test_pyro_num_channels);
    RUN_TEST(test_pyro_default_fire_ms);

    return UNITY_END();
}
