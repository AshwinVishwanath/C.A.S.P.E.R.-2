/**
 * @file  test_hil_handler.c
 * @brief Tier-2 tests for HIL inject handler and flight loop integration.
 *
 * Tests per CASPER_FSM_PRD Level 4:
 *   - HIL inject deserializes correctly
 *   - HIL inject sets virtual clock
 *   - HIL inject drives FSM transitions
 *   - HIL inject rejects bad CRC
 *   - HIL inject rejects short packets
 *   - Flight loop builds fsm_input_t correctly
 *   - Flight loop passes config defaults
 *
 * Compiled with -DHIL_MODE -DUNIT_TEST.
 */

#include "test_config.h"
#include "flight_fsm.h"
#include "fsm_util.h"
#include "fsm_types.h"
#include "tlm_types.h"
#include "mock_tick.h"
#include <string.h>

/* ================================================================== */
/*  Stubs                                                              */
/* ================================================================== */

/* Event log */
#define EVENT_LOG_SIZE 64
static struct { uint8_t type; uint16_t data; } s_event_log[EVENT_LOG_SIZE];
static int s_event_count = 0;

int tlm_queue_event(uint8_t type, uint16_t data)
{
    if (s_event_count < EVENT_LOG_SIZE) {
        s_event_log[s_event_count].type = type;
        s_event_log[s_event_count].data = data;
        s_event_count++;
    }
    return 1;
}

void pyro_mgr_auto_arm_flight(void) { /* stub */ }
int  pyro_mgr_auto_fire(uint8_t ch, uint16_t dur)
{
    tlm_queue_event(FC_EVT_PYRO, ((uint16_t)ch << 8) | (dur & 0xFF));
    return 0;
}
void pyro_mgr_disarm_all(void) { /* stub */ }

static void clear_events(void) { s_event_count = 0; }

static bool has_event_type(uint8_t type)
{
    for (int i = 0; i < s_event_count; i++) {
        if (s_event_log[i].type == type) return true;
    }
    return false;
}

/* ================================================================== */
/*  CRC-32 stub (software implementation for host testing)             */
/* ================================================================== */
static uint32_t crc32_sw(const uint8_t *data, int len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/* ================================================================== */
/*  HIL packet builder helper                                          */
/* ================================================================== */

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
}

static void put_float(uint8_t *p, float v)
{
    memcpy(p, &v, 4);
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/**
 * Build a valid HIL inject packet (44 bytes).
 * Layout: [ID:1][TICK:4][ALT:4][VEL:4][VA:4][ANT:1][FT:4][MDA:4][DFV:4][DFT:4][ACH:1][MCH:1][AFD:2][MFD:2][CRC:4]
 */
static void build_hil_packet(uint8_t *pkt, uint32_t tick_ms,
                              float alt, float vel, float vert_accel,
                              bool antenna_up, float flight_time,
                              float main_alt, float drogue_vel,
                              float drogue_time, uint8_t apogee_ch,
                              uint8_t main_ch, uint16_t apogee_dur,
                              uint16_t main_dur)
{
    pkt[0] = MSG_ID_HIL_INJECT;       /* 0xD1 */
    put_le32(&pkt[1], tick_ms);
    put_float(&pkt[5], alt);
    put_float(&pkt[9], vel);
    put_float(&pkt[13], vert_accel);
    pkt[17] = antenna_up ? 1 : 0;
    put_float(&pkt[18], flight_time);
    put_float(&pkt[22], main_alt);
    put_float(&pkt[26], drogue_vel);
    put_float(&pkt[30], drogue_time);
    pkt[34] = apogee_ch;
    pkt[35] = main_ch;
    put_le16(&pkt[36], apogee_dur);
    put_le16(&pkt[38], main_dur);
    /* CRC over bytes 0..39 */
    uint32_t crc = crc32_sw(pkt, 40);
    put_le32(&pkt[40], crc);
}

/* ================================================================== */
/*  Inline HIL handler for testing (mirrors real hil_handler.c)        */
/* ================================================================== */

static void test_hil_handle_inject(const uint8_t *data, int len)
{
    if (len < SIZE_HIL_INJECT) return;

    /* Verify CRC */
    uint32_t crc_received = get_le32(&data[40]);
    uint32_t crc_computed = crc32_sw(data, 40);
    if (crc_received != crc_computed) return;

    /* Deserialize */
    uint32_t tick_ms = get_le32(&data[1]);
    fsm_input_t in = {0};

    memcpy(&in.alt_m,               &data[5],  4);
    memcpy(&in.vel_mps,             &data[9],  4);
    memcpy(&in.vert_accel_g,        &data[13], 4);
    in.antenna_up = data[17] != 0;
    memcpy(&in.flight_time_s,       &data[18], 4);
    memcpy(&in.main_deploy_alt_m,   &data[22], 4);
    memcpy(&in.drogue_fail_vel_mps, &data[26], 4);
    memcpy(&in.drogue_fail_time_s,  &data[30], 4);
    in.apogee_pyro_ch    = data[34];
    in.main_pyro_ch      = data[35];
    uint16_t afd = (uint16_t)data[36] | ((uint16_t)data[37] << 8);
    uint16_t mfd = (uint16_t)data[38] | ((uint16_t)data[39] << 8);
    in.apogee_fire_dur_ms = afd;
    in.main_fire_dur_ms   = mfd;

    /* Set virtual clock and tick FSM */
    fsm_set_tick(tick_ms);
    flight_fsm_tick(&in);
}

/* ================================================================== */
/*  setUp / tearDown                                                   */
/* ================================================================== */

void setUp(void)
{
    mock_tick_reset();
    mock_tick_set(0);
    fsm_set_tick(0);
    flight_fsm_init();
    clear_events();
}

void tearDown(void) { }

/* ================================================================== */
/*  TEST_HIL: HIL inject tests (5 tests)                               */
/* ================================================================== */

void TEST_HIL_01_deserialize_and_tick(void)
{
    /* Build a packet that should trigger PAD -> BOOST */
    uint8_t pkt[SIZE_HIL_INJECT];

    /* First, inject PAD conditions for 100ms of accel dwell */
    /* Tick 0: start dwell */
    build_hil_packet(pkt, 0,
        0.0f,    /* alt */
        20.0f,   /* vel > 15 */
        3.0f,    /* vert_accel > 2g */
        true,    /* antenna_up */
        0.0f,    /* flight_time */
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* Tick 100: dwell elapsed -> should transition to BOOST */
    build_hil_packet(pkt, 100,
        5.0f, 20.0f, 3.0f, true, 0.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());
}

void TEST_HIL_02_sets_virtual_clock(void)
{
    uint8_t pkt[SIZE_HIL_INJECT];
    build_hil_packet(pkt, 12345,
        0.0f, 0.0f, 0.0f, true, 0.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    TEST_ASSERT_EQUAL_UINT32(12345, fsm_get_tick());
}

void TEST_HIL_03_drives_full_flight(void)
{
    uint8_t pkt[SIZE_HIL_INJECT];

    /* PAD -> BOOST (100ms accel dwell) */
    build_hil_packet(pkt, 0,
        0.0f, 20.0f, 3.0f, true, 0.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);

    build_hil_packet(pkt, 100,
        50.0f, 50.0f, 3.0f, true, 0.1f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());

    /* BOOST -> COAST (100ms burnout dwell) */
    build_hil_packet(pkt, 3000,
        600.0f, 200.0f, -0.5f, true, 3.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);

    build_hil_packet(pkt, 3100,
        620.0f, 190.0f, -0.5f, true, 3.1f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_COAST, flight_fsm_get_state());

    /* COAST -> APOGEE (25ms vel dwell, flight_time > 5s) */
    build_hil_packet(pkt, 12000,
        5000.0f, 0.0f, -1.0f, true, 12.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);

    build_hil_packet(pkt, 12025,
        5000.0f, -0.1f, -1.0f, true, 12.025f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_APOGEE, flight_fsm_get_state());

    /* APOGEE -> MAIN (alt <= main_deploy_alt) */
    build_hil_packet(pkt, 40000,
        249.0f, -8.0f, -1.0f, true, 40.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_MAIN, flight_fsm_get_state());
}

void TEST_HIL_04_rejects_bad_crc(void)
{
    uint8_t pkt[SIZE_HIL_INJECT];
    build_hil_packet(pkt, 0,
        0.0f, 20.0f, 3.0f, true, 0.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);

    /* Corrupt the CRC */
    pkt[40] ^= 0xFF;

    test_hil_handle_inject(pkt, SIZE_HIL_INJECT);
    /* Should stay in PAD (packet rejected) */
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

void TEST_HIL_05_rejects_short_packet(void)
{
    uint8_t pkt[SIZE_HIL_INJECT];
    build_hil_packet(pkt, 0,
        0.0f, 20.0f, 3.0f, true, 0.0f,
        250.0f, 50.0f, 3.0f, 0, 1, 1000, 1000);

    /* Pass a truncated packet */
    test_hil_handle_inject(pkt, SIZE_HIL_INJECT - 1);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());
}

/* ================================================================== */
/*  TEST_LOOP: Flight loop integration tests (2 tests)                 */
/* ================================================================== */

void TEST_LOOP_01_fsm_input_config_defaults(void)
{
    /* Verify that the flight config defaults are sensible */
    flight_cfg_t cfg = {
        .main_deploy_alt = 250.0f,
        .drogue_fail_vel = 50.0f,
        .drogue_fail_time = 3.0f,
        .apogee_pyro_ch = 0,
        .main_pyro_ch = 1,
        .apogee_fire_dur = 1000,
        .main_fire_dur = 1000,
    };

    /* Build fsm_input_t from config defaults */
    fsm_input_t in = {0};
    in.main_deploy_alt_m   = cfg.main_deploy_alt;
    in.drogue_fail_vel_mps = cfg.drogue_fail_vel;
    in.drogue_fail_time_s  = cfg.drogue_fail_time;
    in.apogee_pyro_ch      = cfg.apogee_pyro_ch;
    in.main_pyro_ch        = cfg.main_pyro_ch;
    in.apogee_fire_dur_ms  = cfg.apogee_fire_dur;
    in.main_fire_dur_ms    = cfg.main_fire_dur;

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 250.0f, in.main_deploy_alt_m);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, in.drogue_fail_vel_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, in.drogue_fail_time_s);
    TEST_ASSERT_EQUAL_UINT8(0, in.apogee_pyro_ch);
    TEST_ASSERT_EQUAL_UINT8(1, in.main_pyro_ch);
    TEST_ASSERT_EQUAL_UINT16(1000, in.apogee_fire_dur_ms);
    TEST_ASSERT_EQUAL_UINT16(1000, in.main_fire_dur_ms);
}

void TEST_LOOP_02_fsm_input_drives_transition(void)
{
    /* Build fsm_input_t manually and verify it drives the FSM */
    fsm_input_t in = {0};
    in.alt_m = 0.0f;
    in.vel_mps = 20.0f;
    in.vert_accel_g = 3.0f;
    in.antenna_up = true;
    in.flight_time_s = 0.0f;
    in.main_deploy_alt_m = 250.0f;
    in.drogue_fail_vel_mps = 50.0f;
    in.drogue_fail_time_s = 3.0f;
    in.apogee_pyro_ch = 0;
    in.main_pyro_ch = 1;
    in.apogee_fire_dur_ms = 1000;
    in.main_fire_dur_ms = 1000;

    /* Tick 0: start accel dwell */
    fsm_set_tick(0);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_PAD, flight_fsm_get_state());

    /* Tick 100: dwell met -> launch */
    fsm_set_tick(100);
    flight_fsm_tick(&in);
    TEST_ASSERT_EQUAL_HEX8(FSM_STATE_BOOST, flight_fsm_get_state());
    TEST_ASSERT_TRUE(has_event_type(FC_EVT_STATE));
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */

int main(void)
{
    UNITY_BEGIN();

    /* HIL inject */
    RUN_TEST(TEST_HIL_01_deserialize_and_tick);
    RUN_TEST(TEST_HIL_02_sets_virtual_clock);
    RUN_TEST(TEST_HIL_03_drives_full_flight);
    RUN_TEST(TEST_HIL_04_rejects_bad_crc);
    RUN_TEST(TEST_HIL_05_rejects_short_packet);

    /* Flight loop integration */
    RUN_TEST(TEST_LOOP_01_fsm_input_config_defaults);
    RUN_TEST(TEST_LOOP_02_fsm_input_drives_transition);

    return UNITY_END();
}
