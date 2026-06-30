/* test_endian.c — known-answer tests for App/util/endian.h serialization
 * helpers, including the newly added put_le24 used to de-duplicate the three
 * copy-pasted helper sets in tlm_manager.c / radio_manager.c / self_test.c. */
#include "test.h"
#include "endian.h"

TEST(endian_put_le16) {
    uint8_t b[2]; put_le16(b, 0xBEEF);
    ASSERT_EQ_U(b[0], 0xEF); ASSERT_EQ_U(b[1], 0xBE);
    ASSERT_EQ_U(get_le16(b), 0xBEEFu);
}

TEST(endian_put_le24) {
    uint8_t b[3]; put_le24(b, 0x123456);
    ASSERT_EQ_U(b[0], 0x56); ASSERT_EQ_U(b[1], 0x34); ASSERT_EQ_U(b[2], 0x12);
    /* upper byte of a 32-bit value must be dropped */
    put_le24(b, 0xAABBCCDD);
    ASSERT_EQ_U(b[0], 0xDD); ASSERT_EQ_U(b[1], 0xCC); ASSERT_EQ_U(b[2], 0xBB);
}

TEST(endian_put_le32) {
    uint8_t b[4]; put_le32(b, 0xDEADBEEF);
    ASSERT_EQ_U(b[0], 0xEF); ASSERT_EQ_U(b[1], 0xBE);
    ASSERT_EQ_U(b[2], 0xAD); ASSERT_EQ_U(b[3], 0xDE);
    ASSERT_EQ_U(get_le32(b), 0xDEADBEEFu);
}

int main(void){
    RUN(endian_put_le16);
    RUN(endian_put_le24);
    RUN(endian_put_le32);
    return test_summary();
}
