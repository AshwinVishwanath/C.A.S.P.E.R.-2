/**
 * @file test_telemetry_framing.c
 * @brief Tier-2 tests for end-to-end COBS framing of telemetry packets.
 *
 * Tests:
 *   - Build FC_MSG_FAST (20 bytes), COBS-encode, verify no 0x00 in encoded data
 *   - Build FC_MSG_EVENT (11 bytes), COBS-encode, verify round-trip decode matches original
 *   - Build FC_MSG_GPS (17 bytes), COBS-encode, verify round-trip
 *   - Verify CRC-32 is last 4 bytes in each packet type
 *   - COBS round-trip with zero-heavy payload
 */

#include "test_config.h"
#include "cobs.h"
#include "crc32_hw.h"
#include "tlm_types.h"
#include "endian.h"

/* ================================================================== */
/*  setUp / tearDown                                                    */
/* ================================================================== */

void setUp(void)
{
    crc32_hw_init();
}

void tearDown(void) { }

/* ================================================================== */
/*  Helpers                                                             */
/* ================================================================== */

/* Build a synthetic FC_MSG_FAST packet (20 bytes) */
static void build_fc_msg_fast(uint8_t *buf)
{
    /* [ID:1][STATUS:2][ALT:2][VEL:2][QUAT:5][TIME:2][BATT:1][SEQ:1][CRC:4] = 20 */
    memset(buf, 0, SIZE_FC_MSG_FAST);
    buf[0] = MSG_ID_FAST;            /* 0x01 */

    /* STATUS: state=BOOST(1), arm=0x00 */
    put_le16(&buf[1], 0x0100);

    /* ALT: 1500m (scaled 1:1 -> 0x05DC) */
    put_le16(&buf[3], 1500);

    /* VEL: 250 dm/s (25 m/s * 10) */
    put_le16(&buf[5], 250);

    /* QUAT: 5-byte packed (smallest-three) — use dummy */
    buf[7] = 0x00; buf[8] = 0x7F; buf[9] = 0x00;
    buf[10] = 0x00; buf[11] = 0x00;

    /* TIME: 100 * 100ms = 10.0 s */
    put_le16(&buf[12], 100);

    /* BATT: (12.6 - 6.0) / 0.012 = 550 -> clamped to uint8 = 255 */
    buf[14] = 200;

    /* SEQ */
    buf[15] = 42;

    /* CRC-32 over bytes 0..15 */
    uint32_t crc = crc32_hw_compute(buf, 16);
    put_le32(&buf[16], crc);
}

/* Build a synthetic FC_MSG_EVENT packet (11 bytes) */
static void build_fc_msg_event(uint8_t *buf, uint8_t evt_type, uint16_t evt_data)
{
    /* [ID:1][TYPE:1][DATA:2][TIME:2][RSVD:1][CRC:4] = 11 */
    memset(buf, 0, SIZE_FC_MSG_EVENT);
    buf[0] = MSG_ID_EVENT;
    buf[1] = evt_type;
    put_le16(&buf[2], evt_data);
    put_le16(&buf[4], 500);   /* flight time: 50.0s */
    buf[6] = 0x00;            /* reserved */

    uint32_t crc = crc32_hw_compute(buf, 7);
    put_le32(&buf[7], crc);
}

/* Build a synthetic FC_MSG_GPS packet (17 bytes) */
static void build_fc_msg_gps(uint8_t *buf)
{
    /* [ID:1][DLAT:4][DLON:4][ALT:2][FIX:1][SAT:1][CRC:4] = 17 */
    memset(buf, 0, SIZE_FC_MSG_GPS);
    buf[0] = MSG_ID_GPS;
    put_le32(&buf[1], 1234567);     /* dlat_mm */
    put_le32(&buf[5], -7654321);    /* dlon_mm */
    put_le16(&buf[9], 456);         /* alt */
    buf[11] = 3;                     /* fix type */
    buf[12] = 12;                    /* sat count */

    uint32_t crc = crc32_hw_compute(buf, 13);
    put_le32(&buf[13], crc);
}

/* ================================================================== */
/*  Tests                                                               */
/* ================================================================== */

void test_fc_msg_fast_cobs_no_zeros(void)
{
    uint8_t raw[SIZE_FC_MSG_FAST];
    build_fc_msg_fast(raw);

    uint8_t encoded[SIZE_FC_MSG_FAST + 10];
    int enc_len = cobs_encode(raw, SIZE_FC_MSG_FAST, encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(enc_len > 0);

    /* Verify no 0x00 bytes in encoded data */
    for (int i = 0; i < enc_len; i++) {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0x00, encoded[i],
            "COBS-encoded data must not contain 0x00");
    }
}

void test_fc_msg_fast_crc_position(void)
{
    uint8_t raw[SIZE_FC_MSG_FAST];
    build_fc_msg_fast(raw);

    /* CRC should be at bytes 16-19 (last 4 bytes) */
    uint32_t stored_crc = get_le32(&raw[16]);
    uint32_t computed_crc = crc32_hw_compute(raw, 16);
    TEST_ASSERT_EQUAL_HEX32(computed_crc, stored_crc);
}

void test_fc_msg_event_cobs_round_trip(void)
{
    uint8_t raw[SIZE_FC_MSG_EVENT];
    build_fc_msg_event(raw, FC_EVT_APOGEE, 0x1234);

    /* Encode */
    uint8_t encoded[SIZE_FC_MSG_EVENT + 10];
    int enc_len = cobs_encode(raw, SIZE_FC_MSG_EVENT, encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(enc_len > 0);

    /* Decode */
    uint8_t decoded[SIZE_FC_MSG_EVENT + 10];
    int dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));
    TEST_ASSERT_EQUAL_INT(SIZE_FC_MSG_EVENT, dec_len);

    /* Should match original */
    TEST_ASSERT_EQUAL_MEMORY(raw, decoded, SIZE_FC_MSG_EVENT);
}

void test_fc_msg_event_crc_position(void)
{
    uint8_t raw[SIZE_FC_MSG_EVENT];
    build_fc_msg_event(raw, FC_EVT_STATE, FSM_STATE_BOOST);

    /* CRC at bytes 7-10 (last 4 bytes of 11-byte packet) */
    uint32_t stored_crc = get_le32(&raw[7]);
    uint32_t computed_crc = crc32_hw_compute(raw, 7);
    TEST_ASSERT_EQUAL_HEX32(computed_crc, stored_crc);
}

void test_fc_msg_gps_cobs_round_trip(void)
{
    uint8_t raw[SIZE_FC_MSG_GPS];
    build_fc_msg_gps(raw);

    uint8_t encoded[SIZE_FC_MSG_GPS + 10];
    int enc_len = cobs_encode(raw, SIZE_FC_MSG_GPS, encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(enc_len > 0);

    uint8_t decoded[SIZE_FC_MSG_GPS + 10];
    int dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));
    TEST_ASSERT_EQUAL_INT(SIZE_FC_MSG_GPS, dec_len);
    TEST_ASSERT_EQUAL_MEMORY(raw, decoded, SIZE_FC_MSG_GPS);
}

void test_fc_msg_gps_crc_position(void)
{
    uint8_t raw[SIZE_FC_MSG_GPS];
    build_fc_msg_gps(raw);

    /* CRC at bytes 13-16 (last 4 bytes of 17-byte packet) */
    uint32_t stored_crc = get_le32(&raw[13]);
    uint32_t computed_crc = crc32_hw_compute(raw, 13);
    TEST_ASSERT_EQUAL_HEX32(computed_crc, stored_crc);
}

void test_cobs_round_trip_zero_heavy_payload(void)
{
    /* Test with a payload that contains many 0x00 bytes */
    uint8_t raw[20];
    memset(raw, 0, sizeof(raw));  /* All zeros */
    raw[0] = 0x01;   /* msg_id */
    raw[10] = 0x42;  /* one non-zero byte in the middle */

    uint8_t encoded[32];
    int enc_len = cobs_encode(raw, sizeof(raw), encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(enc_len > 0);

    /* No zeros in encoded */
    for (int i = 0; i < enc_len; i++) {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0x00, encoded[i],
            "COBS-encoded data must not contain 0x00");
    }

    /* Decode must match original */
    uint8_t decoded[32];
    int dec_len = cobs_decode(encoded, enc_len, decoded, sizeof(decoded));
    TEST_ASSERT_EQUAL_INT((int)sizeof(raw), dec_len);
    TEST_ASSERT_EQUAL_MEMORY(raw, decoded, sizeof(raw));
}

void test_cobs_encode_decode_preserves_length(void)
{
    /* For various payload sizes, encode+decode should preserve length */
    for (int sz = 1; sz <= 30; sz++) {
        uint8_t raw[30];
        for (int i = 0; i < sz; i++) {
            raw[i] = (uint8_t)(i * 7 + 3);  /* arbitrary non-zero */
        }

        uint8_t enc[64];
        int enc_len = cobs_encode(raw, sz, enc, sizeof(enc));
        TEST_ASSERT_TRUE(enc_len > 0);

        uint8_t dec[64];
        int dec_len = cobs_decode(enc, enc_len, dec, sizeof(dec));
        TEST_ASSERT_EQUAL_INT(sz, dec_len);
        TEST_ASSERT_EQUAL_MEMORY(raw, dec, (size_t)sz);
    }
}

void test_cobs_single_byte_payload(void)
{
    /* Edge case: 1-byte payload */
    uint8_t raw[1] = {0x42};
    uint8_t enc[4];
    int enc_len = cobs_encode(raw, 1, enc, sizeof(enc));
    TEST_ASSERT_TRUE(enc_len > 0);

    uint8_t dec[4];
    int dec_len = cobs_decode(enc, enc_len, dec, sizeof(dec));
    TEST_ASSERT_EQUAL_INT(1, dec_len);
    TEST_ASSERT_EQUAL_HEX8(0x42, dec[0]);
}

void test_cobs_single_zero_byte(void)
{
    /* Edge case: payload is a single 0x00 */
    uint8_t raw[1] = {0x00};
    uint8_t enc[4];
    int enc_len = cobs_encode(raw, 1, enc, sizeof(enc));
    TEST_ASSERT_TRUE(enc_len > 0);

    /* No zeros in encoded */
    for (int i = 0; i < enc_len; i++) {
        TEST_ASSERT_NOT_EQUAL(0x00, enc[i]);
    }

    uint8_t dec[4];
    int dec_len = cobs_decode(enc, enc_len, dec, sizeof(dec));
    TEST_ASSERT_EQUAL_INT(1, dec_len);
    TEST_ASSERT_EQUAL_HEX8(0x00, dec[0]);
}

/* ================================================================== */
/*  main()                                                              */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* FC_MSG_FAST */
    RUN_TEST(test_fc_msg_fast_cobs_no_zeros);
    RUN_TEST(test_fc_msg_fast_crc_position);

    /* FC_MSG_EVENT */
    RUN_TEST(test_fc_msg_event_cobs_round_trip);
    RUN_TEST(test_fc_msg_event_crc_position);

    /* FC_MSG_GPS */
    RUN_TEST(test_fc_msg_gps_cobs_round_trip);
    RUN_TEST(test_fc_msg_gps_crc_position);

    /* COBS edge cases */
    RUN_TEST(test_cobs_round_trip_zero_heavy_payload);
    RUN_TEST(test_cobs_encode_decode_preserves_length);
    RUN_TEST(test_cobs_single_byte_payload);
    RUN_TEST(test_cobs_single_zero_byte);

    return UNITY_END();
}
