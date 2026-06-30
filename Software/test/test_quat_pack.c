/* test_quat_pack.c — characterization tests for App/pack/quat_pack.c
 * Golden 5-byte packings captured from the ORIGINAL implementation. */
#include "test.h"
#include "quat_pack.h"

TEST(quat_pack_golden) {
    uint8_t out[5];

    float q1[4] = {0.7071068f, 0.7071068f, 0.0f, 0.0f};
    const uint8_t g1[5] = {0x00, 0x00, 0x00, 0xFF, 0x07};
    quat_pack_smallest_three(out, q1);
    ASSERT_EQ_MEM(out, g1, 5);

    float q2[4] = {0.5f, -0.5f, 0.5f, -0.5f};
    const uint8_t g2[5] = {0x00, 0xF8, 0x7F, 0x00, 0x08};
    quat_pack_smallest_three(out, q2);
    ASSERT_EQ_MEM(out, g2, 5);

    float q3[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const uint8_t g3[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    quat_pack_smallest_three(out, q3);
    ASSERT_EQ_MEM(out, g3, 5);
}

/* Sign-flip invariance: q and -q represent the same rotation; the smallest-three
 * packer should canonicalize and produce identical bytes. */
TEST(quat_pack_sign_invariant) {
    uint8_t a[5], b[5];
    float q[4]  = {0.3f, 0.4f, -0.5f, 0.7071f};
    float qn[4] = {-0.3f, -0.4f, 0.5f, -0.7071f};
    quat_pack_smallest_three(a, q);
    quat_pack_smallest_three(b, qn);
    ASSERT_EQ_MEM(a, b, 5);
}

int main(void) {
    RUN(quat_pack_golden);
    RUN(quat_pack_sign_invariant);
    return test_summary();
}
