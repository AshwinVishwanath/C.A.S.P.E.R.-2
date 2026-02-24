/**
 * @file test_cobs.c
 * @brief Unit tests for COBS encode/decode (PRD S3.4).
 *
 * cobs_encode does NOT append 0x00 delimiter.
 * cobs_decode input should NOT include trailing 0x00.
 */

#include "test_config.h"
#include "cobs.h"

/* ------------------------------------------------------------------ */
void setUp(void) { }
void tearDown(void) { }

/* ================================================================== */
/*  Encode tests                                                       */
/* ================================================================== */

void test_cobs_encode_empty(void)
{
    /* Empty input: [] -> encoded length 1 (just overhead byte 0x01) */
    uint8_t out[8];
    int len = cobs_encode(NULL, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(1, len);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
}

void test_cobs_encode_no_zeros(void)
{
    /* [0x01, 0x02, 0x03] -> [0x04, 0x01, 0x02, 0x03], length 4 */
    uint8_t input[] = {0x01, 0x02, 0x03};
    uint8_t out[8];
    int len = cobs_encode(input, 3, out, sizeof(out));

    TEST_ASSERT_EQUAL_INT(4, len);
    TEST_ASSERT_EQUAL_HEX8(0x04, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[3]);
}

void test_cobs_encode_single_zero(void)
{
    /* [0x00] -> [0x01, 0x01], length 2 */
    uint8_t input[] = {0x00};
    uint8_t out[8];
    int len = cobs_encode(input, 1, out, sizeof(out));

    TEST_ASSERT_EQUAL_INT(2, len);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[1]);
}

void test_cobs_encode_leading_zero(void)
{
    /* [0x00, 0x01] -> [0x01, 0x02, 0x01], length 3 */
    uint8_t input[] = {0x00, 0x01};
    uint8_t out[8];
    int len = cobs_encode(input, 2, out, sizeof(out));

    TEST_ASSERT_EQUAL_INT(3, len);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[2]);
}

void test_cobs_encode_254_nonzero(void)
{
    /* 254 non-zero bytes [0x01]*254 -> [0xFF, 0x01*254, 0x01], length 256.
     * When the code byte reaches 0xFF after 254 data bytes, the encoder
     * emits the 0xFF code, then opens a new block with a trailing overhead
     * byte (code=0x01, meaning zero additional data bytes follow). */
    uint8_t input[254];
    memset(input, 0x01, 254);

    uint8_t out[260];
    int len = cobs_encode(input, 254, out, sizeof(out));

    TEST_ASSERT_EQUAL_INT(256, len);
    TEST_ASSERT_EQUAL_HEX8(0xFF, out[0]);
    for (int i = 1; i <= 254; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x01, out[i]);
    }
    TEST_ASSERT_EQUAL_HEX8(0x01, out[255]);
}

void test_cobs_encode_255_nonzero(void)
{
    /* 255 non-zero bytes [0x01]*255 ->
     * [0xFF, 0x01*254, 0x02, 0x01], length 258 */
    uint8_t input[255];
    memset(input, 0x01, 255);

    uint8_t out[264];
    int len = cobs_encode(input, 255, out, sizeof(out));

    TEST_ASSERT_EQUAL_INT(257, len);
    TEST_ASSERT_EQUAL_HEX8(0xFF, out[0]);
    for (int i = 1; i <= 254; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x01, out[i]);
    }
    TEST_ASSERT_EQUAL_HEX8(0x02, out[255]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[256]);
}

void test_cobs_encode_no_zero_in_output(void)
{
    /* Verify that encoded output never contains 0x00 */
    uint8_t input[] = {0x00, 0x01, 0x00, 0x02, 0x03, 0x00};
    uint8_t out[16];
    int len = cobs_encode(input, sizeof(input), out, sizeof(out));

    TEST_ASSERT_TRUE(len > 0);
    for (int i = 0; i < len; i++) {
        TEST_ASSERT_NOT_EQUAL(0x00, out[i]);
    }
}

/* ================================================================== */
/*  Decode tests                                                       */
/* ================================================================== */

void test_cobs_decode_empty_encoded(void)
{
    /* Encoded [0x01] -> decoded empty (0 bytes) */
    uint8_t encoded[] = {0x01};
    uint8_t out[8];
    int len = cobs_decode(encoded, 1, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(0, len);
}

void test_cobs_decode_no_zeros(void)
{
    /* [0x04, 0x01, 0x02, 0x03] -> [0x01, 0x02, 0x03] */
    uint8_t encoded[] = {0x04, 0x01, 0x02, 0x03};
    uint8_t out[8];
    int len = cobs_decode(encoded, 4, out, sizeof(out));

    TEST_ASSERT_EQUAL_INT(3, len);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x02, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0x03, out[2]);
}

void test_cobs_decode_malformed_zero_in_stream(void)
{
    /* Unexpected zero byte in encoded stream should return -1 */
    uint8_t encoded[] = {0x03, 0x01, 0x00, 0x02};
    uint8_t out[8];
    int len = cobs_decode(encoded, 4, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(-1, len);
}

/* ================================================================== */
/*  Round-trip tests                                                   */
/* ================================================================== */

static void round_trip_helper(const uint8_t *data, int data_len)
{
    uint8_t encoded[512];
    uint8_t decoded[512];

    int enc_len = cobs_encode(data, data_len, encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(enc_len > 0);

    int dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));
    TEST_ASSERT_EQUAL_INT(data_len, dec_len);

    if (data_len > 0) {
        TEST_ASSERT_EQUAL_MEMORY(data, decoded, data_len);
    }
}

void test_cobs_roundtrip_simple(void)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    round_trip_helper(data, sizeof(data));
}

void test_cobs_roundtrip_with_zeros(void)
{
    uint8_t data[] = {0x00, 0x01, 0x00, 0x02, 0x00};
    round_trip_helper(data, sizeof(data));
}

void test_cobs_roundtrip_all_zeros(void)
{
    uint8_t data[10];
    memset(data, 0x00, sizeof(data));
    round_trip_helper(data, sizeof(data));
}

void test_cobs_roundtrip_empty(void)
{
    round_trip_helper(NULL, 0);
}

void test_cobs_roundtrip_large(void)
{
    /* 300 bytes with mixed content */
    uint8_t data[300];
    for (int i = 0; i < 300; i++) {
        data[i] = (uint8_t)(i % 256);
    }
    round_trip_helper(data, sizeof(data));
}

/* ================================================================== */
/*  Error handling                                                     */
/* ================================================================== */

void test_cobs_encode_negative_length(void)
{
    uint8_t out[8];
    int len = cobs_encode(NULL, -1, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(-1, len);
}

void test_cobs_encode_output_too_small(void)
{
    uint8_t input[] = {0x01, 0x02, 0x03};
    uint8_t out[2]; /* too small for 4 bytes */
    int len = cobs_encode(input, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(-1, len);
}

void test_cobs_decode_empty_input(void)
{
    uint8_t out[8];
    int len = cobs_decode(NULL, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_INT(-1, len);
}

/* ================================================================== */
/*  main()                                                             */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Encode */
    RUN_TEST(test_cobs_encode_empty);
    RUN_TEST(test_cobs_encode_no_zeros);
    RUN_TEST(test_cobs_encode_single_zero);
    RUN_TEST(test_cobs_encode_leading_zero);
    RUN_TEST(test_cobs_encode_254_nonzero);
    RUN_TEST(test_cobs_encode_255_nonzero);
    RUN_TEST(test_cobs_encode_no_zero_in_output);

    /* Decode */
    RUN_TEST(test_cobs_decode_empty_encoded);
    RUN_TEST(test_cobs_decode_no_zeros);
    RUN_TEST(test_cobs_decode_malformed_zero_in_stream);

    /* Round-trip */
    RUN_TEST(test_cobs_roundtrip_simple);
    RUN_TEST(test_cobs_roundtrip_with_zeros);
    RUN_TEST(test_cobs_roundtrip_all_zeros);
    RUN_TEST(test_cobs_roundtrip_empty);
    RUN_TEST(test_cobs_roundtrip_large);

    /* Error handling */
    RUN_TEST(test_cobs_encode_negative_length);
    RUN_TEST(test_cobs_encode_output_too_small);
    RUN_TEST(test_cobs_decode_empty_input);

    return UNITY_END();
}
