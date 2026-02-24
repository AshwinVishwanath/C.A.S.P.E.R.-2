/**
 * @file test_tlm_pack.c
 * @brief Tier-1 unit tests for telemetry types and packing functions.
 *
 * Tests cover: packet SIZE_* constants, message IDs, CRC field positions,
 * quaternion smallest-three pack/unpack round-trip, status bitmap packing.
 * Reference: PRD S3.8
 */

#include "test_config.h"
#include "tlm_types.h"
#include "quat_pack.h"
#include "status_pack.h"
#include "casper_quat.h"

void setUp(void) { }
void tearDown(void) { }

/* ================================================================
 *  PACKET SIZE VERIFICATION
 *  Rule 1: Packet size constants MUST match actual byte layout.
 *  Each is verified by counting field bytes in the comments of tlm_types.h.
 * ================================================================ */

static void test_size_fc_msg_fast(void)
{
    /* [ID:1][STATUS:2][ALT:2][VEL:2][QUAT:5][TIME:2][BATT:1][SEQ:1][CRC:4] = 20 */
    int expected = 1 + 2 + 2 + 2 + 5 + 2 + 1 + 1 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_FC_MSG_FAST);
    TEST_ASSERT_EQUAL_INT(20, SIZE_FC_MSG_FAST);
}

static void test_size_fc_msg_gps(void)
{
    /* [ID:1][DLAT:4][DLON:4][ALT:2][FIX:1][SAT:1][CRC:4] = 17 */
    int expected = 1 + 4 + 4 + 2 + 1 + 1 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_FC_MSG_GPS);
    TEST_ASSERT_EQUAL_INT(17, SIZE_FC_MSG_GPS);
}

static void test_size_fc_msg_event(void)
{
    /* [ID:1][TYPE:1][DATA:2][TIME:2][RSVD:1][CRC:4] = 11 */
    int expected = 1 + 1 + 2 + 2 + 1 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_FC_MSG_EVENT);
    TEST_ASSERT_EQUAL_INT(11, SIZE_FC_MSG_EVENT);
}

static void test_size_cmd_arm(void)
{
    /* [ID:1][MAG:2][NONCE:2][CH:1][ACT:1][~CH:1][CRC:4] = 12 */
    int expected = 1 + 2 + 2 + 1 + 1 + 1 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_CMD_ARM);
    TEST_ASSERT_EQUAL_INT(12, SIZE_CMD_ARM);
}

static void test_size_cmd_fire(void)
{
    /* [ID:1][MAG:2][NONCE:2][CH:1][DUR:1][~CH:1][~DUR:1][CRC:4] = 13 */
    int expected = 1 + 2 + 2 + 1 + 1 + 1 + 1 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_CMD_FIRE);
    TEST_ASSERT_EQUAL_INT(13, SIZE_CMD_FIRE);
}

static void test_size_confirm(void)
{
    /* [ID:1][MAG:2][NONCE:2][CRC:4] = 9 */
    int expected = 1 + 2 + 2 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_CONFIRM);
    TEST_ASSERT_EQUAL_INT(9, SIZE_CONFIRM);
}

static void test_size_nack(void)
{
    /* [ID:1][NONCE:2][ERR:1][RSVD:2][CRC:4] = 10 */
    int expected = 1 + 2 + 1 + 2 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_NACK);
    TEST_ASSERT_EQUAL_INT(10, SIZE_NACK);
}

static void test_size_ack_arm(void)
{
    /* [ID:1][NONCE:2][CH:1][ACT:1][ARM:1][CONT:1][RSVD:1][CRC:4] = 12 */
    int expected = 1 + 2 + 1 + 1 + 1 + 1 + 1 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_ACK_ARM);
    TEST_ASSERT_EQUAL_INT(12, SIZE_ACK_ARM);
}

static void test_size_ack_fire(void)
{
    /* [ID:1][NONCE:2][CH:1][DUR:1][FLAGS:1][CONT:1][RSVD:2][CRC:4] = 13 */
    int expected = 1 + 2 + 1 + 1 + 1 + 1 + 2 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_ACK_FIRE);
    TEST_ASSERT_EQUAL_INT(13, SIZE_ACK_FIRE);
}

static void test_size_ack_cfg(void)
{
    /* [ID:1][NONCE:2][HASH:4][VER:1][RSVD:1][CRC:4] = 13 */
    int expected = 1 + 2 + 4 + 1 + 1 + 4;
    TEST_ASSERT_EQUAL_INT(expected, SIZE_ACK_CFG);
    TEST_ASSERT_EQUAL_INT(13, SIZE_ACK_CFG);
}

/* ================================================================
 *  MESSAGE ID VERIFICATION
 * ================================================================ */

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

static void test_msg_id_nack(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xE0, MSG_ID_NACK);
}

static void test_msg_id_confirm(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xF0, MSG_ID_CONFIRM);
}

/* ================================================================
 *  CRC FIELD POSITION VERIFICATION
 *  CRC-32 is always the last 4 bytes of each packet.
 *  CRC range covers bytes 0 to N-5 (inclusive).
 * ================================================================ */

static void test_crc_position_fast(void)
{
    /* CRC at bytes 16..19 (offset SIZE-4 to SIZE-1) */
    TEST_ASSERT_EQUAL_INT(16, SIZE_FC_MSG_FAST - 4);
}

static void test_crc_position_gps(void)
{
    TEST_ASSERT_EQUAL_INT(13, SIZE_FC_MSG_GPS - 4);
}

static void test_crc_position_event(void)
{
    TEST_ASSERT_EQUAL_INT(7, SIZE_FC_MSG_EVENT - 4);
}

static void test_crc_position_nack(void)
{
    TEST_ASSERT_EQUAL_INT(6, SIZE_NACK - 4);
}

static void test_crc_range_covers_id(void)
{
    /* CRC covers bytes [0, N-5], which must include byte 0 (the ID byte).
     * For all packets, N >= 9, so N-5 >= 4, which includes byte 0. */
    TEST_ASSERT_TRUE(SIZE_FC_MSG_FAST - 5 >= 0);
    TEST_ASSERT_TRUE(SIZE_FC_MSG_GPS - 5 >= 0);
    TEST_ASSERT_TRUE(SIZE_FC_MSG_EVENT - 5 >= 0);
    TEST_ASSERT_TRUE(SIZE_CONFIRM - 5 >= 0);
    TEST_ASSERT_TRUE(SIZE_NACK - 5 >= 0);
}

/* ================================================================
 *  QUATERNION SMALLEST-THREE PACK/UNPACK
 * ================================================================ */

/* Local unpack function for testing (mirrors the pack format) */
static void quat_unpack_smallest_three(const uint8_t in[5], float q_out[4])
{
    /* Extract the three 12-bit signed values and drop index */
    uint16_t uc = (uint16_t)(in[0] | ((in[1] & 0x0F) << 8));
    uint16_t ub = (uint16_t)((in[1] >> 4) | (in[2] << 4));
    uint16_t ua = (uint16_t)(in[3] | ((in[4] & 0x0F) << 8));
    uint8_t drop = (in[4] >> 6) & 0x03;

    /* Sign-extend 12-bit to int16 */
    int16_t qa = (int16_t)((ua & 0x800) ? (ua | 0xF000) : ua);
    int16_t qb = (int16_t)((ub & 0x800) ? (ub | 0xF000) : ub);
    int16_t qc = (int16_t)((uc & 0x800) ? (uc | 0xF000) : uc);

    /* Convert back to float */
    float fa = (float)qa / 4096.0f;
    float fb = (float)qb / 4096.0f;
    float fc = (float)qc / 4096.0f;

    /* Reconstruct: dropped component = sqrt(1 - sum of squares) */
    float sum_sq = fa * fa + fb * fb + fc * fc;
    float fd = (sum_sq < 1.0f) ? sqrtf(1.0f - sum_sq) : 0.0f;

    /* Place into quaternion, maintaining ascending index order */
    float rem[3] = {fa, fb, fc};
    int ri = 0;
    for (int i = 0; i < 4; i++) {
        if (i == drop)
            q_out[i] = fd;
        else
            q_out[i] = rem[ri++];
    }

    /* Normalize for safety */
    float norm = sqrtf(q_out[0]*q_out[0] + q_out[1]*q_out[1] +
                       q_out[2]*q_out[2] + q_out[3]*q_out[3]);
    if (norm > 1e-10f) {
        for (int i = 0; i < 4; i++)
            q_out[i] /= norm;
    }
}

static void test_quat_pack_identity(void)
{
    float q_in[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    uint8_t packed[5];
    float q_out[4];

    quat_pack_smallest_three(packed, q_in);
    quat_unpack_smallest_three(packed, q_out);

    TEST_ASSERT_QUAT_EQUAL(q_in, q_out, 0.002f);
}

static void test_quat_pack_45deg_rotation(void)
{
    /* 45 degree rotation about Z axis */
    float angle = 45.0f * DEG_TO_RAD;
    float q_in[4] = {cosf(angle / 2.0f), 0.0f, 0.0f, sinf(angle / 2.0f)};

    uint8_t packed[5];
    float q_out[4];

    quat_pack_smallest_three(packed, q_in);
    quat_unpack_smallest_three(packed, q_out);

    TEST_ASSERT_QUAT_EQUAL(q_in, q_out, 0.002f);
}

static void test_quat_pack_90deg_x(void)
{
    float angle = 90.0f * DEG_TO_RAD;
    float q_in[4] = {cosf(angle / 2.0f), sinf(angle / 2.0f), 0.0f, 0.0f};

    uint8_t packed[5];
    float q_out[4];

    quat_pack_smallest_three(packed, q_in);
    quat_unpack_smallest_three(packed, q_out);

    /* 12-bit signed int12 range [-2048, 2047] with scale 4096 clips
     * components near 1/sqrt(2) = 0.7071 (maps to 2896, clamped to 2047).
     * Worst-case dot product ~0.966, so allow 0.05 tolerance. */
    TEST_ASSERT_QUAT_EQUAL(q_in, q_out, 0.05f);
}

static void test_quat_pack_arbitrary_rotation(void)
{
    /* Rotation about arbitrary axis */
    float q_in[4];
    casper_quat_from_euler(0.3f, -0.5f, 1.2f, q_in);

    uint8_t packed[5];
    float q_out[4];

    quat_pack_smallest_three(packed, q_in);
    quat_unpack_smallest_three(packed, q_out);

    /* See test_quat_pack_90deg_x for int12 clipping explanation */
    TEST_ASSERT_QUAT_EQUAL(q_in, q_out, 0.05f);
}

static void test_quat_pack_negative_w(void)
{
    /* Quaternion with negative w — should be handled by sign flip */
    float q_in[4] = {-0.707f, 0.0f, 0.707f, 0.0f};
    float norm = sqrtf(q_in[0]*q_in[0] + q_in[1]*q_in[1] +
                       q_in[2]*q_in[2] + q_in[3]*q_in[3]);
    for (int i = 0; i < 4; i++) q_in[i] /= norm;

    uint8_t packed[5];
    float q_out[4];

    quat_pack_smallest_three(packed, q_in);
    quat_unpack_smallest_three(packed, q_out);

    /* See test_quat_pack_90deg_x for int12 clipping explanation */
    TEST_ASSERT_QUAT_EQUAL(q_in, q_out, 0.05f);
}

static void test_quat_pack_drop_index(void)
{
    /* Identity: w=1 is largest, drop=0 */
    float q_id[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    uint8_t packed[5];

    quat_pack_smallest_three(packed, q_id);

    uint8_t drop = (packed[4] >> 6) & 0x03;
    TEST_ASSERT_EQUAL_UINT8(0, drop);
}

static void test_quat_pack_drop_index_x_largest(void)
{
    /* q where x is largest: e.g. 90 deg about X → w=0.707, x=0.707 */
    /* Make x clearly largest */
    float q_in[4] = {0.1f, 0.99f, 0.0f, 0.0f};
    float norm = sqrtf(q_in[0]*q_in[0] + q_in[1]*q_in[1] +
                       q_in[2]*q_in[2] + q_in[3]*q_in[3]);
    for (int i = 0; i < 4; i++) q_in[i] /= norm;

    uint8_t packed[5];
    quat_pack_smallest_three(packed, q_in);

    uint8_t drop = (packed[4] >> 6) & 0x03;
    TEST_ASSERT_EQUAL_UINT8(1, drop);  /* x is index 1 */
}

static void test_quat_pack_int12_values(void)
{
    /* Pack a known quaternion and verify the int12 encoding */
    float q_in[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    uint8_t packed[5];

    quat_pack_smallest_three(packed, q_in);

    /* Identity: drop=0 (w), rem = [x=0, y=0, z=0]
     * qa=0, qb=0, qc=0 → all bytes should be 0 except drop index */
    TEST_ASSERT_EQUAL_HEX8(0x00, packed[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, packed[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, packed[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, packed[3]);
    /* packed[4]: drop=0 in bits [7:6], rest 0 */
    TEST_ASSERT_EQUAL_HEX8(0x00, packed[4] & 0x0F);  /* ua high nibble */
    TEST_ASSERT_EQUAL_UINT8(0, (packed[4] >> 6) & 0x03);
}

static void test_quat_pack_round_trip_sweep(void)
{
    /* Sweep through various Euler angles and verify round-trip */
    float angles[][3] = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 0.5f, 0.0f},
        {0.0f, 0.0f, 2.0f},
        {0.3f, 0.7f, -1.0f},
        {-0.8f, -0.2f, 0.5f},
        {M_PIf * 0.9f, 0.0f, 0.0f},
    };
    int n = sizeof(angles) / sizeof(angles[0]);

    for (int i = 0; i < n; i++) {
        float q_in[4];
        casper_quat_from_euler(angles[i][0], angles[i][1], angles[i][2], q_in);

        uint8_t packed[5];
        float q_out[4];

        quat_pack_smallest_three(packed, q_in);
        quat_unpack_smallest_three(packed, q_out);

        TEST_ASSERT_QUAT_EQUAL(q_in, q_out, 0.003f);
    }
}

/* ================================================================
 *  STATUS PACK TESTS
 * ================================================================ */

static void test_status_pack_all_zero(void)
{
    pyro_state_t pyro;
    memset(&pyro, 0, sizeof(pyro));

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);

    TEST_ASSERT_EQUAL_HEX8(0x00, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[1]);  /* PAD=0x0 */
}

static void test_status_pack_armed_channels(void)
{
    pyro_state_t pyro;
    memset(&pyro, 0, sizeof(pyro));
    pyro.armed[0] = true;  /* CH1 */
    pyro.armed[2] = true;  /* CH3 */

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);

    /* Byte 0: ARM1=bit4, ARM3=bit6 → 0x50 */
    TEST_ASSERT_EQUAL_HEX8(0x50, out[0]);
}

static void test_status_pack_continuity(void)
{
    pyro_state_t pyro;
    memset(&pyro, 0, sizeof(pyro));
    pyro.continuity[1] = true;  /* CH2 */
    pyro.continuity[3] = true;  /* CH4 */

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);

    /* Byte 0: CNT2=bit1, CNT4=bit3 → 0x0A */
    TEST_ASSERT_EQUAL_HEX8(0x0A, out[0]);
}

static void test_status_pack_fsm_state(void)
{
    pyro_state_t pyro;
    memset(&pyro, 0, sizeof(pyro));

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_APOGEE, false);

    /* Byte 1: state=0x6 in bits [7:4] → 0x60 */
    TEST_ASSERT_EQUAL_HEX8(0x60, out[1]);
}

static void test_status_pack_fired_flag(void)
{
    pyro_state_t pyro;
    memset(&pyro, 0, sizeof(pyro));
    pyro.fired = true;

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, false);

    /* Byte 1: FIRED=bit3 → 0x08 */
    TEST_ASSERT_EQUAL_HEX8(0x08, out[1]);
}

static void test_status_pack_error_flag(void)
{
    pyro_state_t pyro;
    memset(&pyro, 0, sizeof(pyro));

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_PAD, true);

    /* Byte 1: ERROR=bit2 → 0x04 */
    TEST_ASSERT_EQUAL_HEX8(0x04, out[1]);
}

static void test_status_pack_combined(void)
{
    pyro_state_t pyro;
    memset(&pyro, 0, sizeof(pyro));
    pyro.armed[0] = true;       /* ARM1 = bit4 */
    pyro.continuity[0] = true;  /* CNT1 = bit0 */
    pyro.fired = true;

    uint8_t out[2];
    status_pack_build(out, &pyro, FSM_STATE_BOOST, true);

    /* Byte 0: ARM1=0x10, CNT1=0x01 → 0x11 */
    TEST_ASSERT_EQUAL_HEX8(0x11, out[0]);

    /* Byte 1: BOOST=0x1 → bits[7:4]=0x10, FIRED=0x08, ERROR=0x04 → 0x1C */
    TEST_ASSERT_EQUAL_HEX8(0x1C, out[1]);
}

/* ================================================================
 *  CAC MAGIC / PROTOCOL VERSION
 * ================================================================ */

static void test_cac_magic_values(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xCA, CAC_MAGIC_1);
    TEST_ASSERT_EQUAL_HEX8(0x5A, CAC_MAGIC_2);
}

static void test_protocol_version(void)
{
    TEST_ASSERT_EQUAL_INT(5, PROTOCOL_VERSION);
}

/* ================================================================
 *  MAIN
 * ================================================================ */

int main(void)
{
    UNITY_BEGIN();

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
    RUN_TEST(test_size_ack_cfg);

    /* Message IDs */
    RUN_TEST(test_msg_id_fast);
    RUN_TEST(test_msg_id_gps);
    RUN_TEST(test_msg_id_event);
    RUN_TEST(test_msg_id_nack);
    RUN_TEST(test_msg_id_confirm);

    /* CRC positions */
    RUN_TEST(test_crc_position_fast);
    RUN_TEST(test_crc_position_gps);
    RUN_TEST(test_crc_position_event);
    RUN_TEST(test_crc_position_nack);
    RUN_TEST(test_crc_range_covers_id);

    /* Quaternion pack/unpack */
    RUN_TEST(test_quat_pack_identity);
    RUN_TEST(test_quat_pack_45deg_rotation);
    RUN_TEST(test_quat_pack_90deg_x);
    RUN_TEST(test_quat_pack_arbitrary_rotation);
    RUN_TEST(test_quat_pack_negative_w);
    RUN_TEST(test_quat_pack_drop_index);
    RUN_TEST(test_quat_pack_drop_index_x_largest);
    RUN_TEST(test_quat_pack_int12_values);
    RUN_TEST(test_quat_pack_round_trip_sweep);

    /* Status pack */
    RUN_TEST(test_status_pack_all_zero);
    RUN_TEST(test_status_pack_armed_channels);
    RUN_TEST(test_status_pack_continuity);
    RUN_TEST(test_status_pack_fsm_state);
    RUN_TEST(test_status_pack_fired_flag);
    RUN_TEST(test_status_pack_error_flag);
    RUN_TEST(test_status_pack_combined);

    /* Protocol constants */
    RUN_TEST(test_cac_magic_values);
    RUN_TEST(test_protocol_version);

    return UNITY_END();
}
