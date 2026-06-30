/* test_status_pack.c — known-answer + spec-conformance tests for
 * App/pack/status_pack.c. Expected bytes derived from the documented bit layout. */
#include "test.h"
#include "status_pack.h"

TEST(status_pack_known_answer) {
    pyro_state_t p = {0};
    p.armed[0]=1; p.armed[2]=1;            /* ARM1, ARM3 -> byte0 bits 4,6 */
    p.continuity[1]=1; p.continuity[3]=1;  /* CNT2, CNT4 -> byte0 bits 1,3 */
    p.fired = true;                        /* byte1 bit3 */
    uint8_t out[2];
    status_pack_build(out, &p, (fsm_state_t)FSM_STATE_APOGEE /*0x6*/, true /*error*/);
    /* byte0 = 0x50 | 0x0A = 0x5A ; byte1 = (0x6<<4)|0x08|0x04 = 0x6C */
    ASSERT_EQ_U(out[0], 0x5Au);
    ASSERT_EQ_U(out[1], 0x6Cu);
}

TEST(status_pack_all_clear) {
    pyro_state_t p = {0};
    uint8_t out[2];
    status_pack_build(out, &p, (fsm_state_t)FSM_STATE_PAD /*0x0*/, false);
    ASSERT_EQ_U(out[0], 0x00u);
    ASSERT_EQ_U(out[1], 0x00u);
}

TEST(status_pack_fsm_nibble_masked) {
    pyro_state_t p = {0};
    uint8_t out[2];
    /* state value with high bits set must be masked to 4 bits */
    status_pack_build(out, &p, (fsm_state_t)0xAB, false);
    ASSERT_EQ_U(out[1], 0xB0u);  /* (0xAB & 0x0F) << 4 */
}

int main(void){
    RUN(status_pack_known_answer);
    RUN(status_pack_all_clear);
    RUN(status_pack_fsm_nibble_masked);
    return test_summary();
}
