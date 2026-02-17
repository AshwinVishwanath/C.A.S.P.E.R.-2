/*
 * test_flight_config.c -- Host-side unit tests for flight_config parser
 *
 * Compile:
 *   gcc -DHOST_TEST -Wall -Werror \
 *       -I../flight -I../fsm -I../telemetry -I. \
 *       test_flight_config.c ../flight/flight_config.c -lm -o test_flight_config
 */

#define HOST_TEST 1

#include "test_shim.h"
#include "flight_config.h"
#include <string.h>
#include <math.h>

/* ── Helper: software CRC-32 (matches crc32_sw_compute in flight_config.c) ── */
static uint32_t test_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── Helper: write little-endian float into buffer ───────────────── */
static void write_f32_le(uint8_t *p, float val)
{
    memcpy(p, &val, 4);
}

/* ── Helper: write little-endian uint32 into buffer ──────────────── */
static void write_u32_le(uint8_t *p, uint32_t val)
{
    p[0] = (uint8_t)(val & 0xFF);
    p[1] = (uint8_t)((val >> 8) & 0xFF);
    p[2] = (uint8_t)((val >> 16) & 0xFF);
    p[3] = (uint8_t)((val >> 24) & 0xFF);
}

/* ── Helper: build a valid 163-byte config payload ───────────────── */
/*
 * Binary layout (byte offsets):
 *   [0]       version (1 byte)
 *   [1-4]     pad_lat_deg (float LE)
 *   [5-8]     pad_lon_deg (float LE)
 *   [9-12]    pad_alt_m (float LE)
 *   [13-16]   main_deploy_alt_m (float LE)
 *   [17-20]   launch_accel_g (float LE)
 *   [21-24]   min_batt_v (float LE)
 *   [25-28]   min_integrity_pct (float LE)
 *   [29]      num_channels (1 byte)
 *   [30..]    channel blocks, 24 bytes each
 *   [159-162] CRC-32 (last 4 bytes of 163)
 */
static void build_payload(uint8_t *buf, uint8_t version,
                           float pad_lat, float pad_lon, float pad_alt,
                           float main_deploy, float launch_accel,
                           float min_batt, float min_integrity,
                           uint8_t num_ch)
{
    memset(buf, 0, FLIGHT_CONFIG_PAYLOAD_SIZE);
    buf[0] = version;
    write_f32_le(&buf[1],  pad_lat);
    write_f32_le(&buf[5],  pad_lon);
    write_f32_le(&buf[9],  pad_alt);
    write_f32_le(&buf[13], main_deploy);
    write_f32_le(&buf[17], launch_accel);
    write_f32_le(&buf[21], min_batt);
    write_f32_le(&buf[25], min_integrity);
    buf[29] = num_ch;
    /* Channels and CRC filled separately */
}

/* Write a 24-byte channel block at the given channel index (0-3) */
static void write_channel(uint8_t *buf, uint8_t ch_idx,
                           uint8_t role, uint8_t alt_source,
                           float deploy_alt, float fire_delay,
                           float max_angle, uint8_t backup_mode,
                           float backup_timer, float backup_alt,
                           uint8_t primary_ch)
{
    uint16_t base = 30 + (uint16_t)(ch_idx * FLIGHT_CONFIG_CHANNEL_SIZE);
    buf[base + 0] = role;
    buf[base + 1] = alt_source;
    write_f32_le(&buf[base + 2],  deploy_alt);
    write_f32_le(&buf[base + 6],  fire_delay);
    write_f32_le(&buf[base + 10], max_angle);
    buf[base + 14] = backup_mode;
    write_f32_le(&buf[base + 15], backup_timer);
    write_f32_le(&buf[base + 19], backup_alt);
    buf[base + 23] = primary_ch;
}

/* Stamp CRC-32 into the last 4 bytes of a 163-byte buffer */
static void stamp_crc(uint8_t *buf)
{
    uint32_t crc = test_crc32(buf, FLIGHT_CONFIG_PAYLOAD_SIZE - 4);
    write_u32_le(&buf[FLIGHT_CONFIG_PAYLOAD_SIZE - 4], crc);
}

/* ================================================================ */
/*                         TEST CASES                               */
/* ================================================================ */

static void test_default_config_safe(void)
{
    TEST_BEGIN("default_config_safe");
    flight_config_full_t cfg;
    flight_config_default(&cfg);

    ASSERT_EQ(cfg.motors_expected, 1, "motors_expected == 1");
    ASSERT_EQ(cfg.config_loaded, false, "config_loaded == false");
    ASSERT_FLOAT_NEAR(cfg.basic.main_deploy_alt_m, 300.0f, 0.01f,
                      "main_deploy_alt_m == 300");
    ASSERT_FLOAT_NEAR(cfg.basic.launch_accel_g, 4.0f, 0.01f,
                      "launch_accel_g == 4.0");
    ASSERT_FLOAT_NEAR(cfg.min_batt_v, 6.0f, 0.01f,
                      "min_batt_v == 6.0");
    ASSERT_FLOAT_NEAR(cfg.min_integrity_pct, 80.0f, 0.01f,
                      "min_integrity_pct == 80.0");
    ASSERT_EQ(cfg.num_channels, 0, "num_channels == 0");
    ASSERT_EQ(cfg.basic.config_hash, 0, "config_hash == 0");
    TEST_END("default_config_safe");
}

static void test_parse_valid_config(void)
{
    TEST_BEGIN("parse_valid_config");
    uint8_t buf[FLIGHT_CONFIG_PAYLOAD_SIZE];
    flight_config_full_t cfg;

    /* Build payload: version=1, 2 channels */
    build_payload(buf, 1,
                  28.5f,    /* pad_lat */
                  -80.6f,   /* pad_lon */
                  5.0f,     /* pad_alt */
                  250.0f,   /* main_deploy */
                  3.5f,     /* launch_accel */
                  7.0f,     /* min_batt */
                  90.0f);   /* min_integrity */
    buf[29] = 2;            /* num_channels */

    /* Channel 0: Apogee, EKF source, no backup */
    write_channel(buf, 0,
                  PYRO_ROLE_APOGEE, ALT_SOURCE_EKF,
                  0.0f,   /* deploy_alt (N/A for apogee) */
                  0.0f,   /* fire_delay */
                  0.0f,   /* max_angle (N/A) */
                  BACKUP_NONE, 0.0f, 0.0f, 0);

    /* Channel 1: Main, baro source, 250m deploy, timer backup */
    write_channel(buf, 1,
                  PYRO_ROLE_MAIN, ALT_SOURCE_BARO,
                  250.0f,  /* deploy_alt */
                  0.5f,    /* fire_delay */
                  0.0f,    /* max_angle (N/A) */
                  BACKUP_TIMER, 3.0f, 200.0f, 0);

    stamp_crc(buf);

    bool ok = flight_config_parse(buf, FLIGHT_CONFIG_PAYLOAD_SIZE, &cfg);

    ASSERT_TRUE(ok, "parse returns true");
    ASSERT_EQ(cfg.config_loaded, true, "config_loaded == true");
    ASSERT_EQ(cfg.config_version, 1, "config_version == 1");
    ASSERT_FLOAT_NEAR(cfg.basic.pad_lat_deg, 28.5f, 0.01f, "pad_lat");
    ASSERT_FLOAT_NEAR(cfg.basic.pad_lon_deg, -80.6f, 0.01f, "pad_lon");
    ASSERT_FLOAT_NEAR(cfg.basic.pad_alt_m, 5.0f, 0.01f, "pad_alt");
    ASSERT_FLOAT_NEAR(cfg.basic.main_deploy_alt_m, 250.0f, 0.01f, "main_deploy");
    ASSERT_FLOAT_NEAR(cfg.basic.launch_accel_g, 3.5f, 0.01f, "launch_accel");
    ASSERT_FLOAT_NEAR(cfg.min_batt_v, 7.0f, 0.01f, "min_batt_v");
    ASSERT_FLOAT_NEAR(cfg.min_integrity_pct, 90.0f, 0.01f, "min_integrity_pct");
    ASSERT_EQ(cfg.num_channels, 2, "num_channels == 2");

    /* Verify channel 0 */
    ASSERT_EQ(cfg.channels[0].role, PYRO_ROLE_APOGEE, "ch0 role");
    ASSERT_EQ(cfg.channels[0].alt_source, ALT_SOURCE_EKF, "ch0 alt_source");

    /* Verify channel 1 */
    ASSERT_EQ(cfg.channels[1].role, PYRO_ROLE_MAIN, "ch1 role");
    ASSERT_EQ(cfg.channels[1].alt_source, ALT_SOURCE_BARO, "ch1 alt_source");
    ASSERT_FLOAT_NEAR(cfg.channels[1].deploy_alt_m, 250.0f, 0.01f, "ch1 deploy_alt");
    ASSERT_FLOAT_NEAR(cfg.channels[1].fire_delay_s, 0.5f, 0.01f, "ch1 fire_delay");
    ASSERT_EQ(cfg.channels[1].backup_mode, BACKUP_TIMER, "ch1 backup_mode");
    ASSERT_FLOAT_NEAR(cfg.channels[1].backup_timer_s, 3.0f, 0.01f, "ch1 backup_timer");

    /* No ignition channel => motors_expected = 1 */
    ASSERT_EQ(cfg.motors_expected, 1, "motors_expected == 1 (no ignition)");

    /* Hash should be non-zero */
    ASSERT_TRUE(cfg.config_hash != 0, "config_hash non-zero");
    ASSERT_EQ(cfg.basic.config_hash, cfg.config_hash,
              "basic.config_hash matches full config_hash");

    TEST_END("parse_valid_config");
}

static void test_parse_bad_crc(void)
{
    TEST_BEGIN("parse_bad_crc");
    uint8_t buf[FLIGHT_CONFIG_PAYLOAD_SIZE];
    flight_config_full_t cfg;

    build_payload(buf, 1, 0.0f, 0.0f, 0.0f, 300.0f, 4.0f, 6.0f, 80.0f);
    buf[29] = 0;   /* no channels */
    stamp_crc(buf);

    /* Corrupt one byte in the CRC */
    buf[FLIGHT_CONFIG_PAYLOAD_SIZE - 1] ^= 0xFF;

    bool ok = flight_config_parse(buf, FLIGHT_CONFIG_PAYLOAD_SIZE, &cfg);
    ASSERT_TRUE(!ok, "parse returns false on bad CRC");

    TEST_END("parse_bad_crc");
}

static void test_motors_expected_derivation(void)
{
    TEST_BEGIN("motors_expected_derivation");
    uint8_t buf[FLIGHT_CONFIG_PAYLOAD_SIZE];
    flight_config_full_t cfg;

    /* ── Config WITH ignition channel => motors_expected = 2 ──── */
    build_payload(buf, 1, 0.0f, 0.0f, 0.0f, 300.0f, 4.0f, 6.0f, 80.0f);
    buf[29] = 3;   /* 3 channels */

    /* Ch0: Apogee */
    write_channel(buf, 0, PYRO_ROLE_APOGEE, ALT_SOURCE_EKF,
                  0.0f, 0.0f, 0.0f, BACKUP_NONE, 0.0f, 0.0f, 0);
    /* Ch1: Main */
    write_channel(buf, 1, PYRO_ROLE_MAIN, ALT_SOURCE_BARO,
                  300.0f, 0.0f, 0.0f, BACKUP_NONE, 0.0f, 0.0f, 0);
    /* Ch2: Ignition */
    write_channel(buf, 2, PYRO_ROLE_IGNITION, ALT_SOURCE_EKF,
                  0.0f, 0.0f, 15.0f, BACKUP_NONE, 0.0f, 0.0f, 0);

    stamp_crc(buf);

    bool ok = flight_config_parse(buf, FLIGHT_CONFIG_PAYLOAD_SIZE, &cfg);
    ASSERT_TRUE(ok, "parse with ignition succeeds");
    ASSERT_EQ(cfg.motors_expected, 2, "motors_expected == 2 (ignition present)");

    /* ── Config WITHOUT ignition channel => motors_expected = 1 ── */
    build_payload(buf, 1, 0.0f, 0.0f, 0.0f, 300.0f, 4.0f, 6.0f, 80.0f);
    buf[29] = 2;   /* 2 channels */

    /* Ch0: Apogee */
    write_channel(buf, 0, PYRO_ROLE_APOGEE, ALT_SOURCE_EKF,
                  0.0f, 0.0f, 0.0f, BACKUP_NONE, 0.0f, 0.0f, 0);
    /* Ch1: Main */
    write_channel(buf, 1, PYRO_ROLE_MAIN, ALT_SOURCE_BARO,
                  300.0f, 0.0f, 0.0f, BACKUP_NONE, 0.0f, 0.0f, 0);

    stamp_crc(buf);

    ok = flight_config_parse(buf, FLIGHT_CONFIG_PAYLOAD_SIZE, &cfg);
    ASSERT_TRUE(ok, "parse without ignition succeeds");
    ASSERT_EQ(cfg.motors_expected, 1, "motors_expected == 1 (no ignition)");

    TEST_END("motors_expected_derivation");
}

/* ================================================================ */
int main(void)
{
    test_default_config_safe();
    test_parse_valid_config();
    test_parse_bad_crc();
    test_motors_expected_derivation();

    TEST_SUMMARY();
}
