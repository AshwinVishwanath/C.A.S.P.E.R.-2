/**
 * @file test_status_pack.c
 * @brief Unit tests for status_pack_build (PRD S3.6).
 *
 * Bit layout:
 *   Byte 0: [ARM4..ARM1 bits 7:4][CNT4..CNT1 bits 3:0]
 *   Byte 1: [ST3..ST0 bits 7:4][FIRED bit 3][ERROR bit 2][RSVD bits 1:0 = 0]
 */

#include "test_config.h"
#include "status_pack.h"

/* ------------------------------------------------------------------ */
void setUp(void) { }
void tearDown(void) { }

/* ---- Helper to build a clean pyro_state_t ---- */
static pyro_state_t make_pyro(bool a1, bool a2, bool a3, bool a4,
                              bool c1, bool c2, bool c3, bool c4,
                              bool fired)
{
    pyro_state_t p;
    memset(&p, 0, sizeof(p));
    p.armed[0] = a1; p.armed[1] = a2; p.armed[2] = a3; p.armed[3] = a4;
    p.continuity[0] = c1; p.continuity[1] = c2; p.continuity[2] = c3; p.continuity[3] = c4;
    p.fired = fired;
    return p;
}

/* ================================================================== */
/*  Spec example                                                       */
/* ================================================================== */

void test_status_pack_spec_example(void)
{
    /* [0x15, 0x68]:
     * Byte 0 = 0x15 = 0b0001_0101
     *   ARM bits [7:4] = 0001 => ARM1=1, ARM2=0, ARM3=0, ARM4=0
     *   CNT bits [3:0] = 0101 => CNT1=1, CNT2=0, CNT3=1, CNT4=0
     *
     * Byte 1 = 0x68 = 0b0110_1000
     *   ST [7:4] = 0110 = 0x6 => APOGEE
     *   FIRED [3] = 1
     *   ERROR [2] = 0
     *   RSVD [1:0] = 00
     */
    pyro_state_t pyro = make_pyro(
        true, false, false, false,   /* armed: CH1 */
        true, false, true, false,    /* continuity: CH1, CH3 */
        true                         /* fired */
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_APOGEE, false);

    TEST_ASSERT_EQUAL_HEX8(0x15, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x68, out[1]);
}

/* ================================================================== */
/*  All zeros                                                          */
/* ================================================================== */

void test_status_pack_all_zeros(void)
{
    pyro_state_t pyro = make_pyro(
        false, false, false, false,
        false, false, false, false,
        false
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);

    TEST_ASSERT_EQUAL_HEX8(0x00, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1]);
}

/* ================================================================== */
/*  All arm + continuity bits set                                      */
/* ================================================================== */

void test_status_pack_all_arm_cont(void)
{
    pyro_state_t pyro = make_pyro(
        true, true, true, true,
        true, true, true, true,
        false
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);

    /* Byte 0: ARM[7:4]=1111=0xF0, CNT[3:0]=1111=0x0F => 0xFF */
    TEST_ASSERT_EQUAL_HEX8(0xFF, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1]);
}

/* ================================================================== */
/*  FSM state encoding                                                 */
/* ================================================================== */

void test_status_pack_fsm_pad(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_PAD << 4), out[1]);
}

void test_status_pack_fsm_boost(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_BOOST, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_BOOST << 4), out[1]);
}

void test_status_pack_fsm_coast(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_COAST, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_COAST << 4), out[1]);
}

void test_status_pack_fsm_coast_1(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_COAST_1, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_COAST_1 << 4), out[1]);
}

void test_status_pack_fsm_sustain(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_SUSTAIN, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_SUSTAIN << 4), out[1]);
}

void test_status_pack_fsm_coast_2(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_COAST_2, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_COAST_2 << 4), out[1]);
}

void test_status_pack_fsm_apogee(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_APOGEE, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_APOGEE << 4), out[1]);
}

void test_status_pack_fsm_drogue(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_DROGUE, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_DROGUE << 4), out[1]);
}

void test_status_pack_fsm_main(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_MAIN, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_MAIN << 4), out[1]);
}

void test_status_pack_fsm_recovery(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_RECOVERY, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_RECOVERY << 4), out[1]);
}

void test_status_pack_fsm_tumble(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_TUMBLE, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_TUMBLE << 4), out[1]);
}

void test_status_pack_fsm_landed(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false,false,false,false,false,false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_LANDED, false);
    TEST_ASSERT_EQUAL_HEX8((FSM_STATE_LANDED << 4), out[1]);
}

/* ================================================================== */
/*  FIRED flag                                                         */
/* ================================================================== */

void test_status_pack_fired_set(void)
{
    pyro_state_t pyro = make_pyro(
        false, false, false, false,
        false, false, false, false,
        true  /* fired */
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);

    /* Byte 1: [PAD=0x0 << 4][FIRED=1 << 3] = 0x08 */
    TEST_ASSERT_EQUAL_HEX8(0x08, out[1]);
}

void test_status_pack_fired_clear(void)
{
    pyro_state_t pyro = make_pyro(
        false, false, false, false,
        false, false, false, false,
        false
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1] & 0x08);
}

/* ================================================================== */
/*  ERROR flag                                                         */
/* ================================================================== */

void test_status_pack_error_set(void)
{
    pyro_state_t pyro = make_pyro(
        false, false, false, false,
        false, false, false, false,
        false
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, true /* sys_error */);

    /* Byte 1: [PAD=0x0 << 4][FIRED=0][ERROR=1 << 2] = 0x04 */
    TEST_ASSERT_EQUAL_HEX8(0x04, out[1]);
}

void test_status_pack_error_clear(void)
{
    pyro_state_t pyro = make_pyro(
        false, false, false, false,
        false, false, false, false,
        false
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1] & 0x04);
}

/* ================================================================== */
/*  Arm/continuity individual channels                                 */
/* ================================================================== */

void test_status_pack_arm_ch1_only(void)
{
    pyro_state_t pyro = make_pyro(true,false,false,false, false,false,false,false, false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    /* ARM1 = bit 4 = 0x10 */
    TEST_ASSERT_EQUAL_HEX8(0x10, out[0]);
}

void test_status_pack_arm_ch2_only(void)
{
    pyro_state_t pyro = make_pyro(false,true,false,false, false,false,false,false, false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    /* ARM2 = bit 5 = 0x20 */
    TEST_ASSERT_EQUAL_HEX8(0x20, out[0]);
}

void test_status_pack_arm_ch3_only(void)
{
    pyro_state_t pyro = make_pyro(false,false,true,false, false,false,false,false, false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    /* ARM3 = bit 6 = 0x40 */
    TEST_ASSERT_EQUAL_HEX8(0x40, out[0]);
}

void test_status_pack_arm_ch4_only(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,true, false,false,false,false, false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    /* ARM4 = bit 7 = 0x80 */
    TEST_ASSERT_EQUAL_HEX8(0x80, out[0]);
}

void test_status_pack_cont_ch1_only(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false, true,false,false,false, false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    /* CNT1 = bit 0 = 0x01 */
    TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
}

void test_status_pack_cont_ch4_only(void)
{
    pyro_state_t pyro = make_pyro(false,false,false,false, false,false,false,true, false);
    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);
    /* CNT4 = bit 3 = 0x08 */
    TEST_ASSERT_EQUAL_HEX8(0x08, out[0]);
}

/* ================================================================== */
/*  Combined flags                                                     */
/* ================================================================== */

void test_status_pack_combined(void)
{
    /* All armed, all continuity, LANDED, fired + error */
    pyro_state_t pyro = make_pyro(
        true, true, true, true,
        true, true, true, true,
        true
    );

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_LANDED, true);

    TEST_ASSERT_EQUAL_HEX8(0xFF, out[0]);
    /* Byte 1: [LANDED=0xB << 4][FIRED=1 << 3][ERROR=1 << 2] = 0xB0 | 0x08 | 0x04 = 0xBC */
    TEST_ASSERT_EQUAL_HEX8(0xBC, out[1]);
}

/* ================================================================== */
/*  main()                                                             */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Spec example */
    RUN_TEST(test_status_pack_spec_example);
    RUN_TEST(test_status_pack_all_zeros);
    RUN_TEST(test_status_pack_all_arm_cont);

    /* FSM states */
    RUN_TEST(test_status_pack_fsm_pad);
    RUN_TEST(test_status_pack_fsm_boost);
    RUN_TEST(test_status_pack_fsm_coast);
    RUN_TEST(test_status_pack_fsm_coast_1);
    RUN_TEST(test_status_pack_fsm_sustain);
    RUN_TEST(test_status_pack_fsm_coast_2);
    RUN_TEST(test_status_pack_fsm_apogee);
    RUN_TEST(test_status_pack_fsm_drogue);
    RUN_TEST(test_status_pack_fsm_main);
    RUN_TEST(test_status_pack_fsm_recovery);
    RUN_TEST(test_status_pack_fsm_tumble);
    RUN_TEST(test_status_pack_fsm_landed);

    /* Flags */
    RUN_TEST(test_status_pack_fired_set);
    RUN_TEST(test_status_pack_fired_clear);
    RUN_TEST(test_status_pack_error_set);
    RUN_TEST(test_status_pack_error_clear);

    /* Individual arm/continuity */
    RUN_TEST(test_status_pack_arm_ch1_only);
    RUN_TEST(test_status_pack_arm_ch2_only);
    RUN_TEST(test_status_pack_arm_ch3_only);
    RUN_TEST(test_status_pack_arm_ch4_only);
    RUN_TEST(test_status_pack_cont_ch1_only);
    RUN_TEST(test_status_pack_cont_ch4_only);

    /* Combined */
    RUN_TEST(test_status_pack_combined);

    return UNITY_END();
}
