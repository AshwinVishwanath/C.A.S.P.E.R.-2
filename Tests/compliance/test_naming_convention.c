/**
 * @file test_naming_convention.c
 * @brief Naming convention compliance tests (PRD 7.1).
 *
 * Verifies that all protocol constants in tlm_types.h follow the
 * [Source]_[Category]_[Field] naming pattern. Since these are
 * compile-time defines, the test validates that each required
 * prefix family exists and compiles correctly.
 */

#include "test_config.h"
#include "tlm_types.h"
#include <stdio.h>

void setUp(void) { }
void tearDown(void) { }

/* ═══════════════════════════════════════════════════════════════════════
 *  MSG_ID_ prefix family
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_msg_id_prefix_exists(void)
{
    /* All MSG_ID_ defines must compile and be distinct uint8 values */
    uint8_t ids[] = {
        MSG_ID_FAST,
        MSG_ID_GPS,
        MSG_ID_EVENT,
        MSG_ID_CMD_ARM,
        MSG_ID_CMD_FIRE,
        MSG_ID_CMD_TESTMODE,
        MSG_ID_CMD_POLL,
        MSG_ID_ACK_ARM,
        MSG_ID_ACK_FIRE,
        MSG_ID_ACK_CFG,
        MSG_ID_HANDSHAKE,
        MSG_ID_UPLOAD,
        MSG_ID_DIAG,
        MSG_ID_READLOG,
        MSG_ID_ERASELOG,
        MSG_ID_CONFIRM,
        MSG_ID_ABORT,
        MSG_ID_NACK,
        MSG_ID_SIM_FLIGHT,
    };
    int count = (int)(sizeof(ids) / sizeof(ids[0]));

    /* Each ID must be unique */
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (ids[i] == ids[j]) {
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "Duplicate MSG_ID: index %d and %d both = 0x%02X",
                         i, j, ids[i]);
                TEST_FAIL_MESSAGE(msg);
            }
        }
    }
    TEST_ASSERT_TRUE(count >= 15); /* at minimum 15 message types */
}

/* ═══════════════════════════════════════════════════════════════════════
 *  FC_EVT_ prefix family
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_fc_evt_prefix_exists(void)
{
    uint8_t evts[] = {
        FC_EVT_STATE,
        FC_EVT_PYRO,
        FC_EVT_APOGEE,
        FC_EVT_ERROR,
        FC_EVT_ORIGIN,
        FC_EVT_BURNOUT,
        FC_EVT_STAGING,
        FC_EVT_ARM,
    };
    int count = (int)(sizeof(evts) / sizeof(evts[0]));

    /* Each event type must be unique */
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (evts[i] == evts[j]) {
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "Duplicate FC_EVT: index %d and %d both = 0x%02X",
                         i, j, evts[i]);
                TEST_FAIL_MESSAGE(msg);
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(8, count);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  FSM_STATE_ prefix family
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_fsm_state_prefix_exists(void)
{
    uint8_t states[] = {
        FSM_STATE_PAD,
        FSM_STATE_BOOST,
        FSM_STATE_COAST,
        FSM_STATE_COAST_1,
        FSM_STATE_SUSTAIN,
        FSM_STATE_COAST_2,
        FSM_STATE_APOGEE,
        FSM_STATE_DROGUE,
        FSM_STATE_MAIN,
        FSM_STATE_RECOVERY,
        FSM_STATE_TUMBLE,
        FSM_STATE_LANDED,
    };
    int count = (int)(sizeof(states) / sizeof(states[0]));

    /* 12 FSM states */
    TEST_ASSERT_EQUAL_INT(12, count);

    /* Each state must be unique */
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (states[i] == states[j]) {
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "Duplicate FSM_STATE: index %d and %d both = 0x%02X",
                         i, j, states[i]);
                TEST_FAIL_MESSAGE(msg);
            }
        }
    }
}

static void test_fsm_state_sequential(void)
{
    /* FSM states should be sequential 0x0..0xB */
    TEST_ASSERT_EQUAL_HEX8(0x0, FSM_STATE_PAD);
    TEST_ASSERT_EQUAL_HEX8(0xB, FSM_STATE_LANDED);

    /* Each state = previous + 1 */
    uint8_t states[] = {
        FSM_STATE_PAD,      FSM_STATE_BOOST,   FSM_STATE_COAST,
        FSM_STATE_COAST_1,  FSM_STATE_SUSTAIN, FSM_STATE_COAST_2,
        FSM_STATE_APOGEE,   FSM_STATE_DROGUE,  FSM_STATE_MAIN,
        FSM_STATE_RECOVERY, FSM_STATE_TUMBLE,  FSM_STATE_LANDED,
    };
    for (int i = 0; i < 12; i++) {
        TEST_ASSERT_EQUAL_HEX8((uint8_t)i, states[i]);
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 *  NACK_ERR_ prefix family
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_nack_err_prefix_exists(void)
{
    uint8_t errs[] = {
        NACK_ERR_CRC_FAIL,
        NACK_ERR_BAD_STATE,
        NACK_ERR_NOT_ARMED,
        NACK_ERR_NO_TESTMODE,
        NACK_ERR_NONCE_REUSE,
        NACK_ERR_NO_CONTINUITY,
        NACK_ERR_LOW_BATTERY,
        NACK_ERR_SELF_TEST,
        NACK_ERR_CFG_TOO_LARGE,
        NACK_ERR_FLASH_FAIL,
    };
    int count = (int)(sizeof(errs) / sizeof(errs[0]));

    /* All error codes must be unique */
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (errs[i] == errs[j]) {
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "Duplicate NACK_ERR: index %d and %d both = 0x%02X",
                         i, j, errs[i]);
                TEST_FAIL_MESSAGE(msg);
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(10, count);
}

static void test_nack_err_sequential(void)
{
    /* NACK error codes should be sequential 0x01..0x0A */
    TEST_ASSERT_EQUAL_HEX8(0x01, NACK_ERR_CRC_FAIL);
    TEST_ASSERT_EQUAL_HEX8(0x02, NACK_ERR_BAD_STATE);
    TEST_ASSERT_EQUAL_HEX8(0x03, NACK_ERR_NOT_ARMED);
    TEST_ASSERT_EQUAL_HEX8(0x04, NACK_ERR_NO_TESTMODE);
    TEST_ASSERT_EQUAL_HEX8(0x05, NACK_ERR_NONCE_REUSE);
    TEST_ASSERT_EQUAL_HEX8(0x06, NACK_ERR_NO_CONTINUITY);
    TEST_ASSERT_EQUAL_HEX8(0x07, NACK_ERR_LOW_BATTERY);
    TEST_ASSERT_EQUAL_HEX8(0x08, NACK_ERR_SELF_TEST);
    TEST_ASSERT_EQUAL_HEX8(0x09, NACK_ERR_CFG_TOO_LARGE);
    TEST_ASSERT_EQUAL_HEX8(0x0A, NACK_ERR_FLASH_FAIL);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  CAC_MAGIC_ and CAC_ACTION_ prefix families
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_cac_magic_prefix_exists(void)
{
    /* CAC_MAGIC_ defines must compile */
    TEST_ASSERT_EQUAL_HEX8(0xCA, CAC_MAGIC_1);
    TEST_ASSERT_EQUAL_HEX8(0x5A, CAC_MAGIC_2);

    /* Magic bytes must be distinct */
    TEST_ASSERT_NOT_EQUAL(CAC_MAGIC_1, CAC_MAGIC_2);
}

static void test_cac_action_prefix_exists(void)
{
    /* CAC_ACTION_ defines must compile and be distinct */
    TEST_ASSERT_NOT_EQUAL(CAC_ACTION_ARM, CAC_ACTION_DISARM);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  SIZE_ prefix consistency
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_size_prefix_all_positive(void)
{
    /* All SIZE_ constants must be > 0 and reasonable (< 256) */
    TEST_ASSERT_TRUE(SIZE_FC_MSG_FAST > 0 && SIZE_FC_MSG_FAST < 256);
    TEST_ASSERT_TRUE(SIZE_FC_MSG_GPS  > 0 && SIZE_FC_MSG_GPS  < 256);
    TEST_ASSERT_TRUE(SIZE_FC_MSG_EVENT > 0 && SIZE_FC_MSG_EVENT < 256);
    TEST_ASSERT_TRUE(SIZE_CMD_ARM     > 0 && SIZE_CMD_ARM     < 256);
    TEST_ASSERT_TRUE(SIZE_CMD_FIRE    > 0 && SIZE_CMD_FIRE    < 256);
    TEST_ASSERT_TRUE(SIZE_CONFIRM     > 0 && SIZE_CONFIRM     < 256);
    TEST_ASSERT_TRUE(SIZE_NACK        > 0 && SIZE_NACK        < 256);
    TEST_ASSERT_TRUE(SIZE_ACK_ARM     > 0 && SIZE_ACK_ARM     < 256);
    TEST_ASSERT_TRUE(SIZE_ACK_FIRE    > 0 && SIZE_ACK_FIRE    < 256);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Message ID range convention
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_msg_id_range_convention(void)
{
    /* Downlink (FC->MC): 0x01-0x0F */
    TEST_ASSERT_TRUE(MSG_ID_FAST  >= 0x01 && MSG_ID_FAST  <= 0x0F);
    TEST_ASSERT_TRUE(MSG_ID_GPS   >= 0x01 && MSG_ID_GPS   <= 0x0F);
    TEST_ASSERT_TRUE(MSG_ID_EVENT >= 0x01 && MSG_ID_EVENT <= 0x0F);

    /* Uplink commands: 0x80-0x8F */
    TEST_ASSERT_TRUE(MSG_ID_CMD_ARM      >= 0x80 && MSG_ID_CMD_ARM      <= 0x8F);
    TEST_ASSERT_TRUE(MSG_ID_CMD_FIRE     >= 0x80 && MSG_ID_CMD_FIRE     <= 0x8F);
    TEST_ASSERT_TRUE(MSG_ID_CMD_TESTMODE >= 0x80 && MSG_ID_CMD_TESTMODE <= 0x8F);

    /* ACKs: 0xA0-0xAF */
    TEST_ASSERT_TRUE(MSG_ID_ACK_ARM  >= 0xA0 && MSG_ID_ACK_ARM  <= 0xAF);
    TEST_ASSERT_TRUE(MSG_ID_ACK_FIRE >= 0xA0 && MSG_ID_ACK_FIRE <= 0xAF);

    /* Control: 0xC0+ */
    TEST_ASSERT_TRUE(MSG_ID_HANDSHAKE >= 0xC0);

    /* Protocol control: 0xE0+ and 0xF0+ */
    TEST_ASSERT_TRUE(MSG_ID_NACK    >= 0xE0);
    TEST_ASSERT_TRUE(MSG_ID_CONFIRM >= 0xF0);
    TEST_ASSERT_TRUE(MSG_ID_ABORT   >= 0xF0);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    UNITY_BEGIN();

    /* Prefix families */
    RUN_TEST(test_msg_id_prefix_exists);
    RUN_TEST(test_fc_evt_prefix_exists);
    RUN_TEST(test_fsm_state_prefix_exists);
    RUN_TEST(test_fsm_state_sequential);
    RUN_TEST(test_nack_err_prefix_exists);
    RUN_TEST(test_nack_err_sequential);
    RUN_TEST(test_cac_magic_prefix_exists);
    RUN_TEST(test_cac_action_prefix_exists);

    /* SIZE_ prefix */
    RUN_TEST(test_size_prefix_all_positive);

    /* ID range convention */
    RUN_TEST(test_msg_id_range_convention);

    return UNITY_END();
}
