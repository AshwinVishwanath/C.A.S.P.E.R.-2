/**
 * @file test_interface_spec.c
 * @brief Interface specification compliance tests (PRD 7.1).
 *
 * Validates message IDs, packet sizes, CAC constants, FSM states,
 * NACK error codes, and status bitmap layout from tlm_types.h.
 */

#include "test_config.h"
#include "tlm_types.h"

void setUp(void) { }
void tearDown(void) { }

/* ═══════════════════════════════════════════════════════════════════════
 *  Message IDs
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_msg_id_fast(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x01, MSG_ID_FAST);
}

static void test_msg_id_gps(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x02, MSG_ID_GPS);
}

static void test_msg_id_event(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x03, MSG_ID_EVENT);
}

static void test_msg_id_cmd_arm(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x80, MSG_ID_CMD_ARM);
}

static void test_msg_id_cmd_fire(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x81, MSG_ID_CMD_FIRE);
}

static void test_msg_id_confirm(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xF0, MSG_ID_CONFIRM);
}

static void test_msg_id_abort(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xF1, MSG_ID_ABORT);
}

static void test_msg_id_nack(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xE0, MSG_ID_NACK);
}

static void test_msg_id_handshake(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xC0, MSG_ID_HANDSHAKE);
}

static void test_msg_id_cmd_testmode(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x82, MSG_ID_CMD_TESTMODE);
}

static void test_msg_id_cmd_poll(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x83, MSG_ID_CMD_POLL);
}

static void test_msg_id_ack_arm(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xA0, MSG_ID_ACK_ARM);
}

static void test_msg_id_ack_fire(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xA1, MSG_ID_ACK_FIRE);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Packet sizes (byte-counted per INTERFACE_SPEC.md)
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_size_fc_msg_fast(void)
{
    /* [ID:1][STATUS:2][ALT:2][VEL:2][QUAT:5][TIME:2][BATT:1][SEQ:1][CRC:4] = 20 */
    TEST_ASSERT_EQUAL_INT(20, SIZE_FC_MSG_FAST);
}

static void test_size_fc_msg_gps(void)
{
    /* [ID:1][DLAT:4][DLON:4][ALT:2][FIX:1][SAT:1][CRC:4] = 17 */
    TEST_ASSERT_EQUAL_INT(17, SIZE_FC_MSG_GPS);
}

static void test_size_fc_msg_event(void)
{
    /* [ID:1][TYPE:1][DATA:2][TIME:2][RSVD:1][CRC:4] = 11 */
    TEST_ASSERT_EQUAL_INT(11, SIZE_FC_MSG_EVENT);
}

static void test_size_cmd_arm(void)
{
    /* [ID:1][MAG:2][NONCE:2][CH:1][ACT:1][~CH:1][CRC:4] = 12 */
    TEST_ASSERT_EQUAL_INT(12, SIZE_CMD_ARM);
}

static void test_size_cmd_fire(void)
{
    /* [ID:1][MAG:2][NONCE:2][CH:1][DUR:1][~CH:1][~DUR:1][CRC:4] = 13 */
    TEST_ASSERT_EQUAL_INT(13, SIZE_CMD_FIRE);
}

static void test_size_confirm(void)
{
    /* [ID:1][MAG:2][NONCE:2][CRC:4] = 9 */
    TEST_ASSERT_EQUAL_INT(9, SIZE_CONFIRM);
}

static void test_size_nack(void)
{
    /* [ID:1][NONCE:2][ERR:1][RSVD:2][CRC:4] = 10 */
    TEST_ASSERT_EQUAL_INT(10, SIZE_NACK);
}

static void test_size_ack_arm(void)
{
    /* [ID:1][NONCE:2][CH:1][ACT:1][ARM:1][CONT:1][RSVD:1][CRC:4] = 12 */
    TEST_ASSERT_EQUAL_INT(12, SIZE_ACK_ARM);
}

static void test_size_ack_fire(void)
{
    /* [ID:1][NONCE:2][CH:1][DUR:1][FLAGS:1][CONT:1][RSVD:2][CRC:4] = 13 */
    TEST_ASSERT_EQUAL_INT(13, SIZE_ACK_FIRE);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  CAC constants
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_cac_magic_1(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xCA, CAC_MAGIC_1);
}

static void test_cac_magic_2(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x5A, CAC_MAGIC_2);
}

static void test_cac_confirm_timeout(void)
{
    TEST_ASSERT_EQUAL_UINT32(5000, CAC_CONFIRM_TIMEOUT_MS);
}

static void test_cac_action_arm(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x01, CAC_ACTION_ARM);
}

static void test_cac_action_disarm(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x00, CAC_ACTION_DISARM);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Status bitmap layout assertions
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_status_bitmap_fits_2_bytes(void)
{
    /* STATUS field is 2 bytes in FC_MSG_FAST.
     * FSM state uses lower 4 bits (0x0 - 0xB = 12 states, fits in nibble).
     * Upper bits: armed flags, test mode, etc. */
    TEST_ASSERT_TRUE(FSM_STATE_LANDED <= 0x0F);
}

static void test_fsm_state_ordering(void)
{
    /* FSM states must be sequential 0x0 through 0xB */
    TEST_ASSERT_EQUAL_HEX8(0x0, FSM_STATE_PAD);
    TEST_ASSERT_EQUAL_HEX8(0x1, FSM_STATE_BOOST);
    TEST_ASSERT_EQUAL_HEX8(0x2, FSM_STATE_COAST);
    TEST_ASSERT_EQUAL_HEX8(0x3, FSM_STATE_COAST_1);
    TEST_ASSERT_EQUAL_HEX8(0x4, FSM_STATE_SUSTAIN);
    TEST_ASSERT_EQUAL_HEX8(0x5, FSM_STATE_COAST_2);
    TEST_ASSERT_EQUAL_HEX8(0x6, FSM_STATE_APOGEE);
    TEST_ASSERT_EQUAL_HEX8(0x7, FSM_STATE_DROGUE);
    TEST_ASSERT_EQUAL_HEX8(0x8, FSM_STATE_MAIN);
    TEST_ASSERT_EQUAL_HEX8(0x9, FSM_STATE_RECOVERY);
    TEST_ASSERT_EQUAL_HEX8(0xA, FSM_STATE_TUMBLE);
    TEST_ASSERT_EQUAL_HEX8(0xB, FSM_STATE_LANDED);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Protocol version and timing
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_protocol_version(void)
{
    TEST_ASSERT_EQUAL_INT(5, PROTOCOL_VERSION);
}

static void test_telemetry_fast_period(void)
{
    /* 10 Hz => 100 ms period */
    TEST_ASSERT_EQUAL_UINT32(100, TLM_FAST_PERIOD_MS);
}

static void test_test_mode_timeout(void)
{
    /* Test mode expires after 60 seconds */
    TEST_ASSERT_EQUAL_UINT32(60000, TEST_MODE_TIMEOUT_MS);
}

static void test_pyro_max_fire_ms(void)
{
    TEST_ASSERT_EQUAL_UINT32(2000, PYRO_MAX_FIRE_MS);
}

static void test_pyro_mgr_num_channels(void)
{
    TEST_ASSERT_EQUAL_INT(4, PYRO_MGR_NUM_CHANNELS);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  COBS / TX buffer size
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_cobs_max_overhead(void)
{
    TEST_ASSERT_EQUAL_INT(2, COBS_MAX_OVERHEAD);
}

static void test_tlm_tx_buf_size(void)
{
    /* SIZE_FC_MSG_FAST + COBS_MAX_OVERHEAD + 1 = 20 + 2 + 1 = 23 */
    TEST_ASSERT_EQUAL_INT(SIZE_FC_MSG_FAST + COBS_MAX_OVERHEAD + 1,
                           TLM_TX_BUF_SIZE);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    UNITY_BEGIN();

    /* Message IDs */
    RUN_TEST(test_msg_id_fast);
    RUN_TEST(test_msg_id_gps);
    RUN_TEST(test_msg_id_event);
    RUN_TEST(test_msg_id_cmd_arm);
    RUN_TEST(test_msg_id_cmd_fire);
    RUN_TEST(test_msg_id_confirm);
    RUN_TEST(test_msg_id_abort);
    RUN_TEST(test_msg_id_nack);
    RUN_TEST(test_msg_id_handshake);
    RUN_TEST(test_msg_id_cmd_testmode);
    RUN_TEST(test_msg_id_cmd_poll);
    RUN_TEST(test_msg_id_ack_arm);
    RUN_TEST(test_msg_id_ack_fire);

    /* Packet sizes */
    RUN_TEST(test_size_fc_msg_fast);
    RUN_TEST(test_size_fc_msg_gps);
    RUN_TEST(test_size_fc_msg_event);
    RUN_TEST(test_size_cmd_arm);
    RUN_TEST(test_size_cmd_fire);
    RUN_TEST(test_size_confirm);
    RUN_TEST(test_size_nack);
    RUN_TEST(test_size_ack_arm);
    RUN_TEST(test_size_ack_fire);

    /* CAC constants */
    RUN_TEST(test_cac_magic_1);
    RUN_TEST(test_cac_magic_2);
    RUN_TEST(test_cac_confirm_timeout);
    RUN_TEST(test_cac_action_arm);
    RUN_TEST(test_cac_action_disarm);

    /* Status bitmap */
    RUN_TEST(test_status_bitmap_fits_2_bytes);
    RUN_TEST(test_fsm_state_ordering);

    /* Protocol / timing */
    RUN_TEST(test_protocol_version);
    RUN_TEST(test_telemetry_fast_period);
    RUN_TEST(test_test_mode_timeout);
    RUN_TEST(test_pyro_max_fire_ms);
    RUN_TEST(test_pyro_mgr_num_channels);

    /* COBS / buffer */
    RUN_TEST(test_cobs_max_overhead);
    RUN_TEST(test_tlm_tx_buf_size);

    return UNITY_END();
}
