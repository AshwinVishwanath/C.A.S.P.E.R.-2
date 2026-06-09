/* test_cobs.c — characterization + property tests for App/telemetry/cobs.c
 * Golden vectors captured from the ORIGINAL implementation. */
#include "test.h"
#include "cobs.h"

/* Golden: {0x11,0x22,0x00,0x33} encodes to {03 11 22 02 33} */
TEST(cobs_golden_with_zero) {
    const uint8_t in[]  = {0x11, 0x22, 0x00, 0x33};
    const uint8_t gold[] = {0x03, 0x11, 0x22, 0x02, 0x33};
    uint8_t out[16];
    int n = cobs_encode(in, sizeof in, out, sizeof out);
    ASSERT_EQ_INT(n, (int)sizeof gold);
    ASSERT_EQ_MEM(out, gold, sizeof gold);
}

/* Golden: {0x00,0x00,0x00} encodes to {01 01 01 01} */
TEST(cobs_golden_all_zero) {
    const uint8_t in[]  = {0x00, 0x00, 0x00};
    const uint8_t gold[] = {0x01, 0x01, 0x01, 0x01};
    uint8_t out[16];
    int n = cobs_encode(in, sizeof in, out, sizeof out);
    ASSERT_EQ_INT(n, (int)sizeof gold);
    ASSERT_EQ_MEM(out, gold, sizeof gold);
}

/* Round-trip property over many payloads incl. a 254-run boundary */
TEST(cobs_roundtrip) {
    uint8_t in[600], enc[700], dec[600];
    for (int len = 1; len <= 600; len += 37) {
        for (int i = 0; i < len; i++) in[i] = (uint8_t)((i * 31 + 7) & 0xFF);
        int e = cobs_encode(in, len, enc, sizeof enc);
        ASSERT_TRUE(e > 0);
        int d = cobs_decode(enc, e, dec, sizeof dec);
        ASSERT_EQ_INT(d, len);
        ASSERT_EQ_MEM(dec, in, len);
    }
}

/* Guard: undersized output buffer must return -1, not overflow */
TEST(cobs_out_too_small) {
    uint8_t in[300]; for (int i = 0; i < 300; i++) in[i] = (uint8_t)(i + 1);
    uint8_t out[64];
    int n = cobs_encode(in, sizeof in, out, sizeof out);
    ASSERT_EQ_INT(n, -1);
}

int main(void) {
    RUN(cobs_golden_with_zero);
    RUN(cobs_golden_all_zero);
    RUN(cobs_roundtrip);
    RUN(cobs_out_too_small);
    return test_summary();
}
