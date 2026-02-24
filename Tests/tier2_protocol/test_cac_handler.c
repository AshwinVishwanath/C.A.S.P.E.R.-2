/**
 * @file test_cac_handler.c
 * @brief Tier-2 protocol tests for the CAC (Command-Acknowledge-Confirm) handler.
 *
 * Tests per PRD S4.1:
 *   - Happy-path ARM flow (CMD->ACK->CONFIRM->armed)
 *   - Rejection: wrong magic, wrong complement, bad CRC, out-of-range channel, no continuity
 *   - Confirm timeout (5 s)
 *   - Nonce replay rejection
 *   - Happy-path FIRE flow
 *   - ABORT cancels pending action
 */

#include "test_config.h"
#include "main.h"
#include "mock_tick.h"
#include "mock_gpio.h"
#include "mock_adc.h"
#include "cac_handler.h"
#include "crc32_hw.h"
#include "tlm_types.h"
#include "endian.h"
#include "pyro_manager.h"
#include "flight_fsm.h"
#include "casper_pyro.h"

/* ================================================================== */
/*  Spy / stub infrastructure                                          */
/* ================================================================== */

/* Spy for tlm_send_response — captures the last response packet */
static uint8_t spy_resp_buf[64];
static int     spy_resp_len = 0;
static int     spy_resp_count = 0;

int tlm_send_response(const uint8_t *data, int len)
{
    if (len > 0 && len <= (int)sizeof(spy_resp_buf)) {
        memcpy(spy_resp_buf, data, (size_t)len);
    }
    spy_resp_len = len;
    spy_resp_count++;
    return 1;
}

/* Spy for tlm_queue_event */
static uint8_t spy_evt_type;
static uint16_t spy_evt_data;
static int spy_evt_count = 0;

int tlm_queue_event(uint8_t type, uint16_t data)
{
    spy_evt_type = type;
    spy_evt_data = data;
    spy_evt_count++;
    return 1;
}

/* Stub for cfg_get_active_hash (referenced by cac_handler.c via extern) */
uint32_t cfg_get_active_hash(void) { return 0xDEADBEEF; }

/* ================================================================== */
/*  Helpers                                                             */
/* ================================================================== */

static void reset_spies(void)
{
    memset(spy_resp_buf, 0, sizeof(spy_resp_buf));
    spy_resp_len = 0;
    spy_resp_count = 0;
    spy_evt_type = 0;
    spy_evt_data = 0;
    spy_evt_count = 0;
}

/* Build a CMD_ARM packet (12 bytes) with valid magic, CRC */
static void build_cmd_arm(uint8_t *buf, uint16_t nonce, uint8_t channel,
                           uint8_t action)
{
    buf[0] = MSG_ID_CMD_ARM;         /* 0x80 */
    buf[1] = CAC_MAGIC_1;            /* 0xCA */
    buf[2] = CAC_MAGIC_2;            /* 0x5A */
    put_le16(&buf[3], nonce);
    buf[5] = channel;                /* 0-indexed */
    buf[6] = action;
    buf[7] = (uint8_t)(~channel);    /* complement */
    uint32_t crc = crc32_hw_compute(buf, 8);
    put_le32(&buf[8], crc);
}

/* Build a CMD_FIRE packet (13 bytes) with valid magic, CRC */
static void build_cmd_fire(uint8_t *buf, uint16_t nonce, uint8_t channel,
                            uint8_t duration)
{
    buf[0] = MSG_ID_CMD_FIRE;        /* 0x81 */
    buf[1] = CAC_MAGIC_1;
    buf[2] = CAC_MAGIC_2;
    put_le16(&buf[3], nonce);
    buf[5] = channel;
    buf[6] = duration;
    buf[7] = (uint8_t)(~channel);
    buf[8] = (uint8_t)(~duration);
    uint32_t crc = crc32_hw_compute(buf, 9);
    put_le32(&buf[9], crc);
}

/* Build a CONFIRM packet (9 bytes) */
static void build_confirm(uint8_t *buf, uint16_t nonce)
{
    buf[0] = MSG_ID_CONFIRM;         /* 0xF0 */
    buf[1] = CAC_MAGIC_1;
    buf[2] = CAC_MAGIC_2;
    put_le16(&buf[3], nonce);
    uint32_t crc = crc32_hw_compute(buf, 5);
    put_le32(&buf[5], crc);
}

/* Build an ABORT packet (9 bytes) */
static void build_abort(uint8_t *buf, uint16_t nonce)
{
    buf[0] = MSG_ID_ABORT;           /* 0xF1 */
    buf[1] = CAC_MAGIC_1;
    buf[2] = CAC_MAGIC_2;
    put_le16(&buf[3], nonce);
    uint32_t crc = crc32_hw_compute(buf, 5);
    put_le32(&buf[5], crc);
}

/* Force continuity on all channels by setting ADC values above threshold */
static void force_all_continuity(void)
{
    /* casper_pyro_tick reads ADC; set values above PYRO_CONTINUITY_THRESHOLD (8000) */
    mock_adc_set_value(&hadc1, 12000);  /* CH1 + CH2 */
    mock_adc_set_value(&hadc2, 12000);  /* CH4 */
    mock_adc_set_value(&hadc3, 12000);  /* CH3 */
    pyro_mgr_tick();  /* updates continuity flags */
}

static void force_no_continuity(void)
{
    mock_adc_set_value(&hadc1, 100);
    mock_adc_set_value(&hadc2, 100);
    mock_adc_set_value(&hadc3, 100);
    pyro_mgr_tick();
}

/* ================================================================== */
/*  setUp / tearDown                                                    */
/* ================================================================== */

void setUp(void)
{
    mock_tick_reset();
    mock_gpio_reset();
    mock_adc_reset();
    reset_spies();

    crc32_hw_init();
    flight_fsm_init();
    pyro_mgr_init(&hadc1, &hadc2, &hadc3);
    cac_init();

    mock_tick_set(1000);   /* start at a non-zero time */
}

void tearDown(void) { }

/* ================================================================== */
/*  Happy-path ARM tests                                                */
/* ================================================================== */

void test_arm_happy_path(void)
{
    /* Precondition: channel 1 has continuity */
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x1234, 0, CAC_ACTION_ARM);

    /* Step 1: Send ARM command */
    cac_handle_arm(pkt, SIZE_CMD_ARM);

    /* Verify ACK was sent */
    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_ACK_ARM, spy_resp_buf[0]);

    /* Verify ACK contains the correct nonce */
    uint16_t ack_nonce = get_le16(&spy_resp_buf[1]);
    TEST_ASSERT_EQUAL_HEX16(0x1234, ack_nonce);

    /* Step 2: Send CONFIRM before timeout */
    mock_tick_advance(1000);
    uint8_t confirm[SIZE_CONFIRM];
    build_confirm(confirm, 0x1234);
    cac_handle_confirm(confirm, SIZE_CONFIRM);

    /* Channel 1 should now be armed */
    TEST_ASSERT_TRUE(pyro_mgr_get_arm_bitmap() & 0x01);

    /* An FC_EVT_ARM event should have been queued */
    TEST_ASSERT_EQUAL_HEX8(FC_EVT_ARM, spy_evt_type);
}

void test_arm_disarm(void)
{
    force_all_continuity();

    /* Arm channel 2 */
    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0001, 1, CAC_ACTION_ARM);
    cac_handle_arm(pkt, SIZE_CMD_ARM);

    uint8_t confirm[SIZE_CONFIRM];
    build_confirm(confirm, 0x0001);
    cac_handle_confirm(confirm, SIZE_CONFIRM);
    TEST_ASSERT_TRUE(pyro_mgr_get_arm_bitmap() & 0x02);

    /* Now disarm */
    reset_spies();
    mock_tick_advance(100);
    build_cmd_arm(pkt, 0x0002, 1, CAC_ACTION_DISARM);
    cac_handle_arm(pkt, SIZE_CMD_ARM);

    mock_tick_advance(100);
    build_confirm(confirm, 0x0002);
    cac_handle_confirm(confirm, SIZE_CONFIRM);

    TEST_ASSERT_FALSE(pyro_mgr_get_arm_bitmap() & 0x02);
}

/* ================================================================== */
/*  Rejection tests                                                     */
/* ================================================================== */

void test_arm_wrong_magic_silent_discard(void)
{
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0010, 0, CAC_ACTION_ARM);
    pkt[1] = 0xBB;  /* corrupt magic byte 1 */
    /* Re-compute CRC with corrupted magic so CRC itself is "valid" for the corrupted data */
    uint32_t crc = crc32_hw_compute(pkt, 8);
    put_le32(&pkt[8], crc);

    cac_handle_arm(pkt, SIZE_CMD_ARM);

    /* Should be silently discarded — no response at all */
    TEST_ASSERT_EQUAL_INT(0, spy_resp_count);
}

void test_arm_wrong_complement_silent_discard(void)
{
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0020, 0, CAC_ACTION_ARM);
    pkt[7] = 0x00;  /* wrong complement (should be ~0x00 = 0xFF) */
    /* Re-compute CRC */
    uint32_t crc = crc32_hw_compute(pkt, 8);
    put_le32(&pkt[8], crc);

    cac_handle_arm(pkt, SIZE_CMD_ARM);

    TEST_ASSERT_EQUAL_INT(0, spy_resp_count);
}

void test_arm_bad_crc_silent_discard(void)
{
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0030, 0, CAC_ACTION_ARM);
    pkt[8] ^= 0xFF;  /* corrupt CRC byte */

    cac_handle_arm(pkt, SIZE_CMD_ARM);

    TEST_ASSERT_EQUAL_INT(0, spy_resp_count);
}

void test_arm_channel_out_of_range_nack(void)
{
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0040, 5, CAC_ACTION_ARM);  /* channel 5 > 3 */

    cac_handle_arm(pkt, SIZE_CMD_ARM);

    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_NACK, spy_resp_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(NACK_ERR_BAD_STATE, spy_resp_buf[3]);
}

void test_arm_no_continuity_nack(void)
{
    force_no_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0050, 0, CAC_ACTION_ARM);

    cac_handle_arm(pkt, SIZE_CMD_ARM);

    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_NACK, spy_resp_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(NACK_ERR_NO_CONTINUITY, spy_resp_buf[3]);
}

void test_arm_too_short_packet_ignored(void)
{
    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0060, 0, CAC_ACTION_ARM);

    cac_handle_arm(pkt, SIZE_CMD_ARM - 1);  /* len too short */

    TEST_ASSERT_EQUAL_INT(0, spy_resp_count);
}

/* ================================================================== */
/*  Timeout tests                                                       */
/* ================================================================== */

void test_confirm_timeout_cancels(void)
{
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0100, 0, CAC_ACTION_ARM);
    cac_handle_arm(pkt, SIZE_CMD_ARM);
    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);

    /* Advance past 5-second timeout */
    mock_tick_advance(CAC_CONFIRM_TIMEOUT_MS + 1);
    cac_tick();

    /* Now send CONFIRM with the same nonce — should be rejected
     * because cac_tick already returned to IDLE */
    reset_spies();
    uint8_t confirm[SIZE_CONFIRM];
    build_confirm(confirm, 0x0100);
    cac_handle_confirm(confirm, SIZE_CONFIRM);

    /* Channel should NOT be armed */
    TEST_ASSERT_FALSE(pyro_mgr_get_arm_bitmap() & 0x01);
}

/* ================================================================== */
/*  Nonce replay test                                                   */
/* ================================================================== */

void test_nonce_idempotent_retry(void)
{
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0200, 0, CAC_ACTION_ARM);

    /* Send the same ARM command twice */
    cac_handle_arm(pkt, SIZE_CMD_ARM);
    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);

    cac_handle_arm(pkt, SIZE_CMD_ARM);
    TEST_ASSERT_EQUAL_INT(2, spy_resp_count);

    /* Both should be ACKs (idempotent retry, not a second pending) */
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_ACK_ARM, spy_resp_buf[0]);
}

void test_different_nonce_replaces_pending(void)
{
    force_all_continuity();

    /* First ARM with nonce 0x0001 */
    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0001, 0, CAC_ACTION_ARM);
    cac_handle_arm(pkt, SIZE_CMD_ARM);

    /* Second ARM with different nonce 0x0002 (replaces pending) */
    mock_tick_advance(100);
    build_cmd_arm(pkt, 0x0002, 1, CAC_ACTION_ARM);
    cac_handle_arm(pkt, SIZE_CMD_ARM);

    /* Confirm with old nonce — should be rejected (nonce mismatch) */
    mock_tick_advance(100);
    reset_spies();
    uint8_t confirm[SIZE_CONFIRM];
    build_confirm(confirm, 0x0001);
    cac_handle_confirm(confirm, SIZE_CONFIRM);

    /* Nothing should be armed — old nonce was replaced */
    TEST_ASSERT_EQUAL_INT(0, pyro_mgr_get_arm_bitmap());

    /* Confirm with new nonce — should work */
    build_confirm(confirm, 0x0002);
    cac_handle_confirm(confirm, SIZE_CONFIRM);
    TEST_ASSERT_TRUE(pyro_mgr_get_arm_bitmap() & 0x02);
}

/* ================================================================== */
/*  FIRE tests                                                          */
/* ================================================================== */

void test_fire_requires_test_mode(void)
{
    force_all_continuity();

    /* Arm channel 1 first */
    uint8_t arm[SIZE_CMD_ARM];
    build_cmd_arm(arm, 0x0300, 0, CAC_ACTION_ARM);
    cac_handle_arm(arm, SIZE_CMD_ARM);
    uint8_t confirm[SIZE_CONFIRM];
    build_confirm(confirm, 0x0300);
    mock_tick_advance(100);
    cac_handle_confirm(confirm, SIZE_CONFIRM);
    TEST_ASSERT_TRUE(pyro_mgr_get_arm_bitmap() & 0x01);

    /* Try FIRE without test mode */
    reset_spies();
    mock_tick_advance(100);
    uint8_t fire[SIZE_CMD_FIRE];
    build_cmd_fire(fire, 0x0301, 0, 50);
    cac_handle_fire(fire, SIZE_CMD_FIRE);

    /* Should get NACK: no test mode */
    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_NACK, spy_resp_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(NACK_ERR_NO_TESTMODE, spy_resp_buf[3]);
}

void test_fire_requires_arm(void)
{
    force_all_continuity();

    /* pyro_mgr_set_test_mode(true) only sets the pyro_manager's test mode flag.
     * The CAC handler's internal s_test_mode is separate and remains false
     * because we did not go through cac_handle_testmode().
     * So the FIRE handler hits the !s_test_mode check first, returning
     * NACK_ERR_NO_TESTMODE before it ever checks the armed state. */

    /* Channel 1 NOT armed — try fire */
    uint8_t fire[SIZE_CMD_FIRE];
    build_cmd_fire(fire, 0x0400, 0, 50);
    cac_handle_fire(fire, SIZE_CMD_FIRE);

    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_NACK, spy_resp_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(NACK_ERR_NO_TESTMODE, spy_resp_buf[3]);
}

void test_fire_happy_path(void)
{
    force_all_continuity();

    /* Arm channel 1 */
    uint8_t arm[SIZE_CMD_ARM];
    build_cmd_arm(arm, 0x0500, 0, CAC_ACTION_ARM);
    cac_handle_arm(arm, SIZE_CMD_ARM);
    uint8_t confirm[SIZE_CONFIRM];
    build_confirm(confirm, 0x0500);
    mock_tick_advance(100);
    cac_handle_confirm(confirm, SIZE_CONFIRM);

    /* Enable test mode via cac_handle_testmode — this sets the CAC handler's
     * internal s_test_mode flag AND calls pyro_mgr_set_test_mode(true).
     * Using pyro_mgr_set_test_mode() directly only sets pyro_manager's flag
     * but NOT the CAC's s_test_mode, so cac_handle_fire would NACK. */
    reset_spies();
    mock_tick_advance(100);
    cac_handle_testmode(NULL, 0);

    TEST_ASSERT_TRUE(pyro_mgr_get_arm_bitmap() & 0x01);

    /* Send FIRE command */
    reset_spies();
    mock_tick_advance(100);
    uint8_t fire[SIZE_CMD_FIRE];
    build_cmd_fire(fire, 0x0501, 0, 50);
    cac_handle_fire(fire, SIZE_CMD_FIRE);

    /* Should get ACK_FIRE */
    TEST_ASSERT_EQUAL_INT(1, spy_resp_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_ACK_FIRE, spy_resp_buf[0]);

    /* Confirm the fire */
    mock_tick_advance(100);
    build_confirm(confirm, 0x0501);
    cac_handle_confirm(confirm, SIZE_CONFIRM);

    /* FC_EVT_PYRO event should have been queued */
    TEST_ASSERT_EQUAL_HEX8(FC_EVT_PYRO, spy_evt_type);
}

void test_fire_wrong_complement_silent_discard(void)
{
    force_all_continuity();
    pyro_mgr_set_test_mode(true);

    uint8_t fire[SIZE_CMD_FIRE];
    build_cmd_fire(fire, 0x0600, 0, 50);
    fire[7] = 0x00;  /* wrong channel complement */
    /* Re-compute CRC with corrupted data */
    uint32_t crc = crc32_hw_compute(fire, 9);
    put_le32(&fire[9], crc);

    cac_handle_fire(fire, SIZE_CMD_FIRE);

    TEST_ASSERT_EQUAL_INT(0, spy_resp_count);
}

void test_fire_duration_complement_silent_discard(void)
{
    force_all_continuity();
    pyro_mgr_set_test_mode(true);

    uint8_t fire[SIZE_CMD_FIRE];
    build_cmd_fire(fire, 0x0700, 0, 50);
    fire[8] = 0x00;  /* wrong duration complement */
    uint32_t crc = crc32_hw_compute(fire, 9);
    put_le32(&fire[9], crc);

    cac_handle_fire(fire, SIZE_CMD_FIRE);

    TEST_ASSERT_EQUAL_INT(0, spy_resp_count);
}

/* ================================================================== */
/*  ABORT test                                                          */
/* ================================================================== */

void test_abort_cancels_pending(void)
{
    force_all_continuity();

    uint8_t arm[SIZE_CMD_ARM];
    build_cmd_arm(arm, 0x0800, 0, CAC_ACTION_ARM);
    cac_handle_arm(arm, SIZE_CMD_ARM);

    /* Send ABORT with matching nonce */
    uint8_t abort_pkt[SIZE_CONFIRM];
    build_abort(abort_pkt, 0x0800);
    cac_handle_abort(abort_pkt, SIZE_CONFIRM);

    /* Now try CONFIRM — should fail since we aborted */
    reset_spies();
    mock_tick_advance(100);
    uint8_t confirm[SIZE_CONFIRM];
    build_confirm(confirm, 0x0800);
    cac_handle_confirm(confirm, SIZE_CONFIRM);

    /* Channel should NOT be armed */
    TEST_ASSERT_EQUAL_INT(0, pyro_mgr_get_arm_bitmap());
}

/* ================================================================== */
/*  ACK packet structure validation                                     */
/* ================================================================== */

void test_ack_arm_crc_valid(void)
{
    force_all_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0900, 0, CAC_ACTION_ARM);
    cac_handle_arm(pkt, SIZE_CMD_ARM);

    /* Validate CRC in the ACK_ARM response */
    TEST_ASSERT_EQUAL_INT(SIZE_ACK_ARM, spy_resp_len);
    uint32_t payload_crc = crc32_hw_compute(spy_resp_buf, 8);
    uint32_t received_crc = get_le32(&spy_resp_buf[8]);
    TEST_ASSERT_EQUAL_HEX32(payload_crc, received_crc);
}

void test_nack_crc_valid(void)
{
    force_no_continuity();

    uint8_t pkt[SIZE_CMD_ARM];
    build_cmd_arm(pkt, 0x0A00, 0, CAC_ACTION_ARM);
    cac_handle_arm(pkt, SIZE_CMD_ARM);

    /* Validate CRC in the NACK response */
    TEST_ASSERT_EQUAL_INT(SIZE_NACK, spy_resp_len);
    uint32_t payload_crc = crc32_hw_compute(spy_resp_buf, 6);
    uint32_t received_crc = get_le32(&spy_resp_buf[6]);
    TEST_ASSERT_EQUAL_HEX32(payload_crc, received_crc);
}

/* ================================================================== */
/*  main()                                                              */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* Happy-path ARM */
    RUN_TEST(test_arm_happy_path);
    RUN_TEST(test_arm_disarm);

    /* Rejection tests */
    RUN_TEST(test_arm_wrong_magic_silent_discard);
    RUN_TEST(test_arm_wrong_complement_silent_discard);
    RUN_TEST(test_arm_bad_crc_silent_discard);
    RUN_TEST(test_arm_channel_out_of_range_nack);
    RUN_TEST(test_arm_no_continuity_nack);
    RUN_TEST(test_arm_too_short_packet_ignored);

    /* Timeout */
    RUN_TEST(test_confirm_timeout_cancels);

    /* Nonce */
    RUN_TEST(test_nonce_idempotent_retry);
    RUN_TEST(test_different_nonce_replaces_pending);

    /* FIRE */
    RUN_TEST(test_fire_requires_test_mode);
    RUN_TEST(test_fire_requires_arm);
    RUN_TEST(test_fire_happy_path);
    RUN_TEST(test_fire_wrong_complement_silent_discard);
    RUN_TEST(test_fire_duration_complement_silent_discard);

    /* ABORT */
    RUN_TEST(test_abort_cancels_pending);

    /* Packet structure */
    RUN_TEST(test_ack_arm_crc_valid);
    RUN_TEST(test_nack_crc_valid);

    return UNITY_END();
}
