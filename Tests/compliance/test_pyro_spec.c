/**
 * @file test_pyro_spec.c
 * @brief Pyro system specification compliance tests (PRD 7.1).
 *
 * Validates CAC protocol constants, pyro channel configuration,
 * fire pin assignments, and safety-critical numeric values.
 */

#include "test_config.h"
#include "tlm_types.h"
#include "casper_pyro.h"

void setUp(void) { }
void tearDown(void) { }

/* ═══════════════════════════════════════════════════════════════════════
 *  CAC command message IDs
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_cmd_arm_id(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x80, MSG_ID_CMD_ARM);
}

static void test_cmd_fire_id(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x81, MSG_ID_CMD_FIRE);
}

static void test_cmd_testmode_id(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x82, MSG_ID_CMD_TESTMODE);
}

static void test_confirm_id(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xF0, MSG_ID_CONFIRM);
}

static void test_abort_id(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xF1, MSG_ID_ABORT);
}

static void test_nack_id(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xE0, MSG_ID_NACK);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  CAC magic bytes and timeout
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_cac_magic_bytes(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xCA, CAC_MAGIC_1);
    TEST_ASSERT_EQUAL_HEX8(0x5A, CAC_MAGIC_2);
}

static void test_cac_magic_combined(void)
{
    /* Magic bytes form 0xCA5A when combined big-endian */
    uint16_t magic = ((uint16_t)CAC_MAGIC_1 << 8) | CAC_MAGIC_2;
    TEST_ASSERT_EQUAL_HEX16(0xCA5A, magic);
}

static void test_cac_confirm_timeout_5s(void)
{
    TEST_ASSERT_EQUAL_UINT32(5000, CAC_CONFIRM_TIMEOUT_MS);
}

static void test_cac_action_values(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x01, CAC_ACTION_ARM);
    TEST_ASSERT_EQUAL_HEX8(0x00, CAC_ACTION_DISARM);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Pyro channel configuration
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_pyro_num_channels_4(void)
{
    TEST_ASSERT_EQUAL_INT(4, PYRO_NUM_CHANNELS);
}

static void test_pyro_mgr_num_channels_matches(void)
{
    /* tlm_types.h and casper_pyro.h must agree on channel count */
    TEST_ASSERT_EQUAL_INT(PYRO_NUM_CHANNELS, PYRO_MGR_NUM_CHANNELS);
}

static void test_pyro_continuity_threshold_value(void)
{
    TEST_ASSERT_EQUAL_UINT16(8000, PYRO_CONTINUITY_THRESHOLD);
}

static void test_pyro_default_fire_duration(void)
{
    TEST_ASSERT_EQUAL_UINT32(1000, PYRO_DEFAULT_FIRE_MS);
}

static void test_pyro_max_fire_duration(void)
{
    TEST_ASSERT_EQUAL_UINT32(2000, PYRO_MAX_FIRE_MS);
}

static void test_pyro_max_fire_exceeds_default(void)
{
    /* Safety: max fire must be >= default fire */
    TEST_ASSERT_TRUE(PYRO_MAX_FIRE_MS >= PYRO_DEFAULT_FIRE_MS);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Pyro state struct layout
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_pyro_state_array_sizes(void)
{
    /* Verify pyro_state_t arrays match PYRO_MGR_NUM_CHANNELS */
    pyro_state_t ps;
    (void)ps;
    TEST_ASSERT_EQUAL_INT(PYRO_MGR_NUM_CHANNELS,
                           (int)(sizeof(ps.armed) / sizeof(ps.armed[0])));
    TEST_ASSERT_EQUAL_INT(PYRO_MGR_NUM_CHANNELS,
                           (int)(sizeof(ps.continuity) / sizeof(ps.continuity[0])));
    TEST_ASSERT_EQUAL_INT(PYRO_MGR_NUM_CHANNELS,
                           (int)(sizeof(ps.cont_v) / sizeof(ps.cont_v[0])));
}

static void test_casper_pyro_array_sizes(void)
{
    /* Verify casper_pyro_t internal arrays match PYRO_NUM_CHANNELS */
    casper_pyro_t cp;
    (void)cp;
    TEST_ASSERT_EQUAL_INT(PYRO_NUM_CHANNELS,
                           (int)(sizeof(cp.continuity) / sizeof(cp.continuity[0])));
    TEST_ASSERT_EQUAL_INT(PYRO_NUM_CHANNELS,
                           (int)(sizeof(cp.adc_raw) / sizeof(cp.adc_raw[0])));
    TEST_ASSERT_EQUAL_INT(PYRO_NUM_CHANNELS,
                           (int)(sizeof(cp.firing) / sizeof(cp.firing[0])));
    TEST_ASSERT_EQUAL_INT(PYRO_NUM_CHANNELS,
                           (int)(sizeof(cp.fire_start_ms) / sizeof(cp.fire_start_ms[0])));
    TEST_ASSERT_EQUAL_INT(PYRO_NUM_CHANNELS,
                           (int)(sizeof(cp.fire_duration_ms) / sizeof(cp.fire_duration_ms[0])));
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Packet sizes for pyro-related messages
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_cmd_arm_packet_size(void)
{
    TEST_ASSERT_EQUAL_INT(12, SIZE_CMD_ARM);
}

static void test_cmd_fire_packet_size(void)
{
    TEST_ASSERT_EQUAL_INT(13, SIZE_CMD_FIRE);
}

static void test_ack_arm_packet_size(void)
{
    TEST_ASSERT_EQUAL_INT(12, SIZE_ACK_ARM);
}

static void test_ack_fire_packet_size(void)
{
    TEST_ASSERT_EQUAL_INT(13, SIZE_ACK_FIRE);
}

static void test_confirm_packet_size(void)
{
    TEST_ASSERT_EQUAL_INT(9, SIZE_CONFIRM);
}

static void test_nack_packet_size(void)
{
    TEST_ASSERT_EQUAL_INT(10, SIZE_NACK);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  NACK error codes relevant to pyro
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_nack_not_armed(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x03, NACK_ERR_NOT_ARMED);
}

static void test_nack_no_testmode(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x04, NACK_ERR_NO_TESTMODE);
}

static void test_nack_no_continuity(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x06, NACK_ERR_NO_CONTINUITY);
}

static void test_nack_nonce_reuse(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x05, NACK_ERR_NONCE_REUSE);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Test mode timeout
 * ═══════════════════════════════════════════════════════════════════════ */

static void test_test_mode_timeout_60s(void)
{
    TEST_ASSERT_EQUAL_UINT32(60000, TEST_MODE_TIMEOUT_MS);
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    UNITY_BEGIN();

    /* CAC command IDs */
    RUN_TEST(test_cmd_arm_id);
    RUN_TEST(test_cmd_fire_id);
    RUN_TEST(test_cmd_testmode_id);
    RUN_TEST(test_confirm_id);
    RUN_TEST(test_abort_id);
    RUN_TEST(test_nack_id);

    /* CAC magic / timeout */
    RUN_TEST(test_cac_magic_bytes);
    RUN_TEST(test_cac_magic_combined);
    RUN_TEST(test_cac_confirm_timeout_5s);
    RUN_TEST(test_cac_action_values);

    /* Pyro channel config */
    RUN_TEST(test_pyro_num_channels_4);
    RUN_TEST(test_pyro_mgr_num_channels_matches);
    RUN_TEST(test_pyro_continuity_threshold_value);
    RUN_TEST(test_pyro_default_fire_duration);
    RUN_TEST(test_pyro_max_fire_duration);
    RUN_TEST(test_pyro_max_fire_exceeds_default);

    /* Struct layouts */
    RUN_TEST(test_pyro_state_array_sizes);
    RUN_TEST(test_casper_pyro_array_sizes);

    /* Packet sizes */
    RUN_TEST(test_cmd_arm_packet_size);
    RUN_TEST(test_cmd_fire_packet_size);
    RUN_TEST(test_ack_arm_packet_size);
    RUN_TEST(test_ack_fire_packet_size);
    RUN_TEST(test_confirm_packet_size);
    RUN_TEST(test_nack_packet_size);

    /* NACK error codes */
    RUN_TEST(test_nack_not_armed);
    RUN_TEST(test_nack_no_testmode);
    RUN_TEST(test_nack_no_continuity);
    RUN_TEST(test_nack_nonce_reuse);

    /* Test mode */
    RUN_TEST(test_test_mode_timeout_60s);

    return UNITY_END();
}
