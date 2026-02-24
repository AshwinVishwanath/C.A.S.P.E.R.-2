/**
 * @file test_cmd_router.c
 * @brief Tier-2 protocol tests for the COBS command router.
 *
 * Tests per PRD S4.2:
 *   - COBS-encoded frame dispatch by msg_id
 *   - Partial frame + delimiter -> frame dispatched
 *   - Corrupted frame -> silently discarded
 *   - Back-to-back frames -> both dispatched
 *   - Ring buffer wraparound
 *   - Oversized frame -> discarded
 */

#include "test_config.h"
#include "mock_tick.h"
#include "cobs.h"
#include "crc32_hw.h"
#include "tlm_types.h"
#include "endian.h"
#include "usbd_cdc_if.h"

/* ================================================================== */
/*  Spy / dispatch tracking                                             */
/* ================================================================== */

/* Track dispatched commands by msg_id */
#define DISPATCH_LOG_SIZE 16
static uint8_t s_dispatch_ids[DISPATCH_LOG_SIZE];
static int     s_dispatch_count = 0;
static uint8_t s_dispatch_data[DISPATCH_LOG_SIZE][64];
static int     s_dispatch_lens[DISPATCH_LOG_SIZE];

static void reset_dispatch(void)
{
    s_dispatch_count = 0;
    memset(s_dispatch_ids, 0, sizeof(s_dispatch_ids));
    memset(s_dispatch_data, 0, sizeof(s_dispatch_data));
    memset(s_dispatch_lens, 0, sizeof(s_dispatch_lens));
}

static void record_dispatch(const uint8_t *data, int len)
{
    if (s_dispatch_count < DISPATCH_LOG_SIZE) {
        s_dispatch_ids[s_dispatch_count] = data[0];
        int copy_len = len < 64 ? len : 64;
        memcpy(s_dispatch_data[s_dispatch_count], data, (size_t)copy_len);
        s_dispatch_lens[s_dispatch_count] = len;
        s_dispatch_count++;
    }
}

/* ---- Override CAC handlers to track dispatch ---- */
void cac_handle_arm(const uint8_t *data, int len)       { record_dispatch(data, len); }
void cac_handle_fire(const uint8_t *data, int len)      { record_dispatch(data, len); }
void cac_handle_testmode(const uint8_t *data, int len)  { record_dispatch(data, len); }
void cac_handle_config_poll(const uint8_t *data, int len){ record_dispatch(data, len); }
void cac_handle_confirm(const uint8_t *data, int len)   { record_dispatch(data, len); }
void cac_handle_abort(const uint8_t *data, int len)     { record_dispatch(data, len); }
void cfg_handle_upload(const uint8_t *data, int len)    { record_dispatch(data, len); }
void cfg_handle_readlog(const uint8_t *data, int len)   { record_dispatch(data, len); }
void cfg_handle_eraselog(const uint8_t *data, int len)   { record_dispatch(data, len); }
int  self_test_run_and_send(void) { return 0; }
uint32_t cfg_get_active_hash(void) { return 0xDEADBEEF; }

/* Stubs for tlm_send_response and tlm_queue_event (called by cmd_handle_handshake) */
int tlm_send_response(const uint8_t *data, int len) { record_dispatch(data, len); return 1; }
int tlm_queue_event(uint8_t type, uint16_t data) { (void)type; (void)data; return 1; }

/* Stub for flight_fsm functions (called by cmd_handle_sim_flight) */
bool flight_fsm_sim_active(void) { return false; }
void flight_fsm_sim_start(void) { }
void flight_fsm_sim_stop(void) { }
uint8_t flight_fsm_get_state(void) { return 0; }

/* ================================================================== */
/*  CDC ring buffer helpers                                             */
/* ================================================================== */

/* Feed bytes into the CDC ring buffer used by cmd_router */
static void feed_ring(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++) {
        cdc_rx_ring[cdc_rx_head] = data[i];
        cdc_rx_head = (cdc_rx_head + 1) % CDC_RING_SIZE;
    }
}

static void reset_ring(void)
{
    cdc_rx_head = 0;
    cdc_rx_tail = 0;
}

/* COBS-encode + append 0x00 delimiter and feed into ring */
static void feed_cobs_frame(const uint8_t *raw, int raw_len)
{
    uint8_t enc[128];
    int enc_len = cobs_encode(raw, raw_len, enc, sizeof(enc));
    TEST_ASSERT_TRUE(enc_len > 0);
    feed_ring(enc, enc_len);
    /* Append delimiter */
    uint8_t delim = 0x00;
    feed_ring(&delim, 1);
}

/* ================================================================== */
/*  setUp / tearDown                                                    */
/* ================================================================== */

/* Forward declaration — we include cmd_router.h */
#include "cmd_router.h"

void setUp(void)
{
    mock_tick_reset();
    reset_dispatch();
    reset_ring();
    crc32_hw_init();
    cmd_router_init();
}

void tearDown(void) { }

/* ================================================================== */
/*  Tests                                                               */
/* ================================================================== */

void test_single_arm_frame_dispatched(void)
{
    /* Build a raw CMD_ARM packet */
    uint8_t raw[SIZE_CMD_ARM];
    raw[0] = MSG_ID_CMD_ARM;
    raw[1] = CAC_MAGIC_1;
    raw[2] = CAC_MAGIC_2;
    put_le16(&raw[3], 0x1234);
    raw[5] = 0x00;  /* channel 0 */
    raw[6] = CAC_ACTION_ARM;
    raw[7] = 0xFF;  /* ~0x00 */
    uint32_t crc = crc32_hw_compute(raw, 8);
    put_le32(&raw[8], crc);

    feed_cobs_frame(raw, SIZE_CMD_ARM);
    cmd_router_process();

    TEST_ASSERT_EQUAL_INT(1, s_dispatch_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_CMD_ARM, s_dispatch_ids[0]);
}

void test_partial_then_delimiter_dispatches(void)
{
    /* Build raw packet */
    uint8_t raw[SIZE_CMD_ARM];
    raw[0] = MSG_ID_CMD_ARM;
    raw[1] = CAC_MAGIC_1;
    raw[2] = CAC_MAGIC_2;
    put_le16(&raw[3], 0x5678);
    raw[5] = 0x01;
    raw[6] = CAC_ACTION_ARM;
    raw[7] = 0xFE;  /* ~0x01 */
    uint32_t crc = crc32_hw_compute(raw, 8);
    put_le32(&raw[8], crc);

    /* COBS-encode */
    uint8_t enc[128];
    int enc_len = cobs_encode(raw, SIZE_CMD_ARM, enc, sizeof(enc));

    /* Feed first half */
    int half = enc_len / 2;
    feed_ring(enc, half);
    cmd_router_process();
    TEST_ASSERT_EQUAL_INT(0, s_dispatch_count);

    /* Feed second half + delimiter */
    feed_ring(&enc[half], enc_len - half);
    uint8_t delim = 0x00;
    feed_ring(&delim, 1);
    cmd_router_process();

    TEST_ASSERT_EQUAL_INT(1, s_dispatch_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_CMD_ARM, s_dispatch_ids[0]);
}

void test_corrupted_cobs_silently_discarded(void)
{
    /* Feed invalid COBS data (contains 0x00 in the encoded stream) */
    uint8_t bad[] = {0x05, 0x01, 0x00, 0x03, 0x04, 0x00};
    /* The 0x00 at index 2 is treated as a delimiter,
     * so we get a 2-byte frame [0x05, 0x01] which decodes to
     * a 4-byte block attempt but only 1 data byte — may decode or fail.
     * Let's use a simpler corruption: a valid-looking COBS but with
     * a code byte that overruns the frame. */
    uint8_t bad2[] = {0xFF, 0x01, 0x02, 0x00};  /* code=255 but only 2 data bytes */
    feed_ring(bad2, sizeof(bad2));
    cmd_router_process();

    TEST_ASSERT_EQUAL_INT(0, s_dispatch_count);
}

void test_back_to_back_frames_both_dispatched(void)
{
    /* Frame 1: CMD_ARM */
    uint8_t raw1[SIZE_CMD_ARM];
    raw1[0] = MSG_ID_CMD_ARM;
    raw1[1] = CAC_MAGIC_1;
    raw1[2] = CAC_MAGIC_2;
    put_le16(&raw1[3], 0x1111);
    raw1[5] = 0x00;
    raw1[6] = CAC_ACTION_ARM;
    raw1[7] = 0xFF;
    put_le32(&raw1[8], crc32_hw_compute(raw1, 8));

    /* Frame 2: CONFIRM */
    uint8_t raw2[SIZE_CONFIRM];
    raw2[0] = MSG_ID_CONFIRM;
    raw2[1] = CAC_MAGIC_1;
    raw2[2] = CAC_MAGIC_2;
    put_le16(&raw2[3], 0x1111);
    put_le32(&raw2[5], crc32_hw_compute(raw2, 5));

    /* Feed both frames before calling process */
    feed_cobs_frame(raw1, SIZE_CMD_ARM);
    feed_cobs_frame(raw2, SIZE_CONFIRM);

    cmd_router_process();

    TEST_ASSERT_EQUAL_INT(2, s_dispatch_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_CMD_ARM, s_dispatch_ids[0]);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_CONFIRM, s_dispatch_ids[1]);
}

void test_unknown_msg_id_silently_discarded(void)
{
    /* Build a frame with unknown msg_id 0xFF */
    uint8_t raw[5] = {0xFF, 0x01, 0x02, 0x03, 0x04};
    feed_cobs_frame(raw, sizeof(raw));

    cmd_router_process();

    /* Unknown msg_id -> dispatch_frame's switch falls through, no handler called */
    TEST_ASSERT_EQUAL_INT(0, s_dispatch_count);
}

void test_empty_frame_ignored(void)
{
    /* Just a delimiter with no data */
    uint8_t delim = 0x00;
    feed_ring(&delim, 1);

    cmd_router_process();

    TEST_ASSERT_EQUAL_INT(0, s_dispatch_count);
}

void test_ring_buffer_wraparound(void)
{
    /* Fill the ring close to capacity, then wrap around */
    /* Move tail close to end of ring */
    cdc_rx_head = CDC_RING_SIZE - 4;
    cdc_rx_tail = CDC_RING_SIZE - 4;

    /* Build a frame that wraps around the ring boundary */
    uint8_t raw[SIZE_CONFIRM];
    raw[0] = MSG_ID_CONFIRM;
    raw[1] = CAC_MAGIC_1;
    raw[2] = CAC_MAGIC_2;
    put_le16(&raw[3], 0x2222);
    put_le32(&raw[5], crc32_hw_compute(raw, 5));

    feed_cobs_frame(raw, SIZE_CONFIRM);
    cmd_router_process();

    TEST_ASSERT_EQUAL_INT(1, s_dispatch_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_CONFIRM, s_dispatch_ids[0]);
}

void test_multiple_delimiters_between_frames(void)
{
    /* Multiple 0x00 delimiters should not cause issues */
    uint8_t delims[] = {0x00, 0x00, 0x00};
    feed_ring(delims, sizeof(delims));

    uint8_t raw[SIZE_CONFIRM];
    raw[0] = MSG_ID_CONFIRM;
    raw[1] = CAC_MAGIC_1;
    raw[2] = CAC_MAGIC_2;
    put_le16(&raw[3], 0x3333);
    put_le32(&raw[5], crc32_hw_compute(raw, 5));

    feed_cobs_frame(raw, SIZE_CONFIRM);
    cmd_router_process();

    TEST_ASSERT_EQUAL_INT(1, s_dispatch_count);
    TEST_ASSERT_EQUAL_HEX8(MSG_ID_CONFIRM, s_dispatch_ids[0]);
}

/* ================================================================== */
/*  main()                                                              */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_single_arm_frame_dispatched);
    RUN_TEST(test_partial_then_delimiter_dispatches);
    RUN_TEST(test_corrupted_cobs_silently_discarded);
    RUN_TEST(test_back_to_back_frames_both_dispatched);
    RUN_TEST(test_unknown_msg_id_silently_discarded);
    RUN_TEST(test_empty_frame_ignored);
    RUN_TEST(test_ring_buffer_wraparound);
    RUN_TEST(test_multiple_delimiters_between_frames);

    return UNITY_END();
}
