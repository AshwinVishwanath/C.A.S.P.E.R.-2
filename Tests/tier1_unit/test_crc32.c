/**
 * @file test_crc32.c
 * @brief Unit tests for CRC-32 hardware wrapper (PRD S3.5).
 *
 * Uses software CRC-32 stub (reflected, poly 0xEDB88320, init 0xFFFFFFFF).
 * crc32_hw_compute does final XOR: raw ^ 0xFFFFFFFF.
 */

#include "test_config.h"
#include "crc32_hw.h"

/* ------------------------------------------------------------------ */
void setUp(void)
{
    crc32_hw_init();
}

void tearDown(void) { }

/* ================================================================== */
/*  Standard test vectors                                              */
/* ================================================================== */

void test_crc32_standard_123456789(void)
{
    /* The canonical CRC-32 check value for ASCII "123456789" is 0xCBF43926. */
    const uint8_t data[] = "123456789";
    uint32_t crc = crc32_hw_compute(data, 9);
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926, crc);
}

void test_crc32_empty_input(void)
{
    /* Empty input: CRC-32 of 0 bytes.
     * init=0xFFFFFFFF, no data processed, final XOR = 0xFFFFFFFF ^ 0xFFFFFFFF = 0x00000000.
     * But actually: the stub computes CRC over 0 bytes, crc stays 0xFFFFFFFF,
     * then crc32_hw_compute does raw ^ 0xFFFFFFFF = 0x00000000. */
    uint8_t dummy = 0;
    uint32_t crc = crc32_hw_compute(&dummy, 0);
    /* CRC-32 of empty is 0x00000000 with final XOR */
    TEST_ASSERT_EQUAL_HEX32(0x00000000, crc);
}

void test_crc32_single_byte_zero(void)
{
    /* CRC-32 of single byte 0x00: known reference value 0xD202EF8D */
    uint8_t data[] = {0x00};
    uint32_t crc = crc32_hw_compute(data, 1);
    TEST_ASSERT_EQUAL_HEX32(0xD202EF8D, crc);
}

void test_crc32_all_ff_256(void)
{
    /* CRC-32 of 256 bytes of 0xFF.
     * Computed via reflected CRC-32 (init 0xFFFFFFFF, poly 0xEDB88320,
     * final XOR 0xFFFFFFFF) = 0xFEA8A821. */
    uint8_t data[256];
    memset(data, 0xFF, 256);
    uint32_t crc = crc32_hw_compute(data, 256);

    TEST_ASSERT_EQUAL_HEX32(0xFEA8A821, crc);
}

/* ================================================================== */
/*  Validate function                                                  */
/* ================================================================== */

void test_crc32_validate_correct(void)
{
    const uint8_t data[] = "123456789";
    uint32_t expected_crc = 0xCBF43926;
    int result = crc32_hw_validate(data, 9, expected_crc);
    TEST_ASSERT_EQUAL_INT(1, result);
}

void test_crc32_validate_incorrect(void)
{
    const uint8_t data[] = "123456789";
    uint32_t wrong_crc = 0xDEADBEEF;
    int result = crc32_hw_validate(data, 9, wrong_crc);
    TEST_ASSERT_EQUAL_INT(0, result);
}

/* ================================================================== */
/*  Consistency tests                                                  */
/* ================================================================== */

void test_crc32_deterministic(void)
{
    /* Same input always produces same CRC */
    const uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint32_t crc1 = crc32_hw_compute(data, 4);
    uint32_t crc2 = crc32_hw_compute(data, 4);
    TEST_ASSERT_EQUAL_HEX32(crc1, crc2);
}

void test_crc32_different_data(void)
{
    /* Different input should (almost certainly) produce different CRC */
    const uint8_t data1[] = {0x01, 0x02, 0x03};
    const uint8_t data2[] = {0x01, 0x02, 0x04};
    uint32_t crc1 = crc32_hw_compute(data1, 3);
    uint32_t crc2 = crc32_hw_compute(data2, 3);
    TEST_ASSERT_NOT_EQUAL(crc1, crc2);
}

/* ================================================================== */
/*  main()                                                             */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Standard vectors */
    RUN_TEST(test_crc32_standard_123456789);
    RUN_TEST(test_crc32_empty_input);
    RUN_TEST(test_crc32_single_byte_zero);
    RUN_TEST(test_crc32_all_ff_256);

    /* Validate */
    RUN_TEST(test_crc32_validate_correct);
    RUN_TEST(test_crc32_validate_incorrect);

    /* Consistency */
    RUN_TEST(test_crc32_deterministic);
    RUN_TEST(test_crc32_different_data);

    return UNITY_END();
}
