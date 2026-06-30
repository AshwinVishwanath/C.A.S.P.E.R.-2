/* test_hamming.c — characterization tests for App/logging/hamming.c
 * Golden parity/CRC captured from the ORIGINAL implementation. */
#include "test.h"
#include "hamming.h"

static void fill(uint8_t *d) { for (int i = 0; i < 60; i++) d[i] = (uint8_t)(i * 7 + 3); }

TEST(hamming_golden_parity) {
    uint8_t d[60]; fill(d);
    ASSERT_EQ_U(hamming_encode(d, 60), 0x028Fu);
}

TEST(crc16_golden) {
    uint8_t d[60]; fill(d);
    ASSERT_EQ_U(crc16_ccitt(d, 60), 0xF92Bu);
}

TEST(hamming_clean_no_error) {
    uint8_t d[60]; fill(d);
    uint16_t par = hamming_encode(d, 60);
    ASSERT_EQ_INT(hamming_decode(d, 60, par), 0);
}

/* Single-bit error in every bit position must be corrected (r==1, data restored) */
TEST(hamming_single_bit_correction) {
    uint8_t d[60]; fill(d);
    uint16_t par = hamming_encode(d, 60);
    for (int byte = 0; byte < 60; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t c[60]; for (int i = 0; i < 60; i++) c[i] = d[i];
            c[byte] ^= (uint8_t)(1u << bit);
            int r = hamming_decode(c, 60, par);
            ASSERT_EQ_INT(r, 1);
            ASSERT_EQ_MEM(c, d, 60);
        }
    }
}

int main(void) {
    RUN(hamming_golden_parity);
    RUN(crc16_golden);
    RUN(hamming_clean_no_error);
    RUN(hamming_single_bit_correction);
    return test_summary();
}
