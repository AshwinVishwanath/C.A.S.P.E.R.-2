/*
 * test_max_m10m.c — host-side characterization + edge-case suite for the
 * U-blox MAX-M10M GPS driver (App/drivers/max_m10m.c).
 *
 * WRITTEN BEFORE THE HAL->seam MIGRATION.  The CURRENT behavior of the
 * driver (golden byte sequences on the wire, UBX checksums, NAV-PVT numeric
 * conversions, tick state machine, fix logic) IS the spec these tests pin.
 *
 * RED STATE NOTE
 * --------------
 * This suite is written against the *target* migrated public API:
 *
 *     bool max_m10m_init      (max_m10m_t*, casper_i2c_t*, casper_pin_t nrst);
 *     bool max_m10m_init_minimal(max_m10m_t*, casper_i2c_t*, casper_pin_t nrst);
 *     int  max_m10m_tick      (max_m10m_t*);
 *     int  max_m10m_tick_nmea (max_m10m_t*);
 *     bool max_m10m_has_3d_fix(const max_m10m_t*);
 *     void max_m10m_irq_handler(max_m10m_t*);
 *     bool max_m10m_configure_gps_test(max_m10m_t*);
 *     bool max_m10m_poll_mon_rf(max_m10m_t*);
 *
 * and against the seam struct fields (dev->i2c : casper_i2c_t*,
 * dev->nrst : casper_pin_t).  Until max_m10m.{c,h} are migrated off the
 * HAL (they still use I2C_HandleTypeDef* / GPIO_TypeDef* today), this file
 * WILL NOT COMPILE.  That is the intended RED state — it goes GREEN the
 * moment the implementation lands.  Do NOT modify the driver to satisfy
 * these tests; the goldens below are the contract.
 *
 * MOCK SEMANTICS THAT SHAPE THESE TESTS
 * -------------------------------------
 *  - casper_i2c_mem_read / master_tx / dev_ready in board_mock.c ALWAYS
 *    return CASPER_OK (never TIMEOUT/ERR) unless the per-bus log fills to
 *    MOCK_I2C_LOG_MAX, after which they return CASPER_ERR.  Error-path
 *    coverage for I2C therefore uses the log-overflow technique.
 *  - The I2C RX queue is a SINGLE FIFO per bus, shared by every mem_read
 *    regardless of `reg`.  A read of the bytes-available register (0xFD)
 *    and a read of the data-stream register (0xFF) both pull from the same
 *    queue in call order — so scripted bytes must be laid out in the exact
 *    order the driver will consume them.
 *  - casper_delay_ms advances the virtual clock, so blocking wait loops in
 *    the driver terminate deterministically.
 */

#include "test.h"
#include "board_mock.h"
#include "max_m10m.h"

/* ====================================================================== */
/*  Golden frames (computed offline; UBX Fletcher checksums verified)      */
/* ====================================================================== */

/* UBX-CFG-VALSET frames the driver emits during max_m10m_init(), in order. */
static const uint8_t VS_I2COUTPROT_UBX[17] =
    {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x72,0x10,0x01,0x1F,0xB6};
static const uint8_t VS_I2COUTPROT_NMEA[17] =
    {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x02,0x00,0x72,0x10,0x00,0x1F,0xBA};
static const uint8_t VS_MSGOUT_NAVPVT[17] =
    {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x07,0x00,0x91,0x20,0x01,0x54,0x51};
static const uint8_t VS_RATE_MEAS[18] =
    {0xB5,0x62,0x06,0x8A,0x0A,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x21,0x30,0x64,0x00,0x52,0xC3};

/* configure_gps_test() frames */
static const uint8_t VS_DYNMODEL[17] =
    {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x21,0x00,0x11,0x20,0x08,0xF5,0x5A};
static const uint8_t VS_LNA_MODE[17] =
    {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x38,0x00,0xA3,0x20,0x02,0x98,0x7D};

/* CFG-RST cold start (clear BBR+RAM) */
static const uint8_t RST_FRAME[12] =
    {0xB5,0x62,0x06,0x04,0x04,0x00,0xFF,0xFF,0x00,0x00,0x0C,0x5D};

/* UBX-MON-RF poll (empty payload) */
static const uint8_t MONRF_POLL[8] =
    {0xB5,0x62,0x0A,0x38,0x00,0x00,0x42,0xD0};

/* A complete, checksum-valid NAV-PVT (92-byte payload) describing:
 *   lat = +37.4190000 deg, lon = -122.1970000 deg, h_msl = 152.400 m,
 *   velD = -5.000 m/s, fixType = 3 (3D), num_sv = 11, pDOP = 175. */
static const uint8_t NAVPVT[100] = {
    0xB5,0x62,0x01,0x07,0x5C,0x00,0x15,0xCD,0x5B,0x07,
    0xEA,0x07,0x06,0x10,0x0C,0x1E,0x2D,0x37,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x0B,
    0xB0,0x37,0x2A,0xB7,0xB0,0xAF,0x4D,0x16,0x00,0x00,
    0x00,0x00,0x50,0x53,0x02,0x00,0xC4,0x09,0x00,0x00,
    0xAC,0x0D,0x00,0x00,0xE8,0x03,0x00,0x00,0x30,0xF8,
    0xFF,0xFF,0x78,0xEC,0xFF,0xFF,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0xAF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x22,0x9B};

/* ACK-ACK / ACK-NAK for class/id (0x06,0x8A) = CFG-VALSET. */
static const uint8_t ACKACK[10] = {0xB5,0x62,0x05,0x01,0x02,0x00,0x06,0x8A,0x98,0xC1};
static const uint8_t ACKNAK[10] = {0xB5,0x62,0x05,0x00,0x02,0x00,0x06,0x8A,0x97,0xBC};

#define GPS_ADDR8  (MAX_M10M_I2C_ADDR << 1)   /* 0x42<<1 = 0x84 */

/* Shared bus + pin used by every test (re-made fresh in each test). */
static casper_i2c_t g_i2c;
static casper_pin_t g_nrst = { (void*)0x5000, 1u << 0 };

/* ---------------------------------------------------------------------- */
/*  Helpers                                                                */
/* ---------------------------------------------------------------------- */

/* Push a little-endian uint16 bytes-available count into the RX FIFO, in
 * the MSB-then-LSB order the driver reads it (reg 0xFD = HI, 0xFE = LO). */
static void push_avail(casper_i2c_t *bus, uint16_t avail)
{
    uint8_t b[2] = { (uint8_t)(avail >> 8), (uint8_t)(avail & 0xFF) };
    mock_i2c_push_rx(bus, b, 2);
}

/* Find the idx-th MASTER_TX entry (UBX frame write) in the log; -1 if none. */
static int nth_master_tx(casper_i2c_t *bus, int n)
{
    int seen = 0;
    for (int i = 0; i < mock_i2c_log_count(bus); i++) {
        const mock_I2cLog_t *e = mock_i2c_log_get(bus, i);
        if (e->op == MOCK_I2C_MASTER_TX) {
            if (seen == n) return i;
            seen++;
        }
    }
    return -1;
}

/* ====================================================================== */
/*  1. UBX checksum + frame builder goldens                                */
/* ====================================================================== */

/* The driver builds + checksums VALSET frames internally; we observe them
 * on the wire via configure_gps_test (a short, ACK-free-by-design path that
 * still emits exactly two known frames). */
TEST(configure_gps_test_emits_exact_dynmodel_and_lna_frames)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.i2c = &g_i2c;
    dev.alive = true;          /* skip init; exercise configure path */

    /* No ACK scripted → wait_ack times out, but the two TX frames are
     * emitted regardless and that is what we pin. */
    max_m10m_configure_gps_test(&dev);

    int i0 = nth_master_tx(&g_i2c, 0);
    int i1 = nth_master_tx(&g_i2c, 1);
    ASSERT_TRUE(i0 >= 0);
    ASSERT_TRUE(i1 >= 0);

    const mock_I2cLog_t *e0 = mock_i2c_log_get(&g_i2c, i0);
    const mock_I2cLog_t *e1 = mock_i2c_log_get(&g_i2c, i1);

    ASSERT_EQ_U(e0->addr8, GPS_ADDR8);
    ASSERT_EQ_INT(e0->len, sizeof(VS_DYNMODEL));
    ASSERT_EQ_MEM(e0->buf, VS_DYNMODEL, sizeof(VS_DYNMODEL));

    ASSERT_EQ_U(e1->addr8, GPS_ADDR8);
    ASSERT_EQ_INT(e1->len, sizeof(VS_LNA_MODE));
    ASSERT_EQ_MEM(e1->buf, VS_LNA_MODE, sizeof(VS_LNA_MODE));
}

/* configure_gps_test returns false when the device is not alive (guard). */
TEST(configure_gps_test_not_alive_returns_false)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.alive = false;
    ASSERT_TRUE(max_m10m_configure_gps_test(&dev) == false);
    /* No frames emitted when guarded out. */
    ASSERT_EQ_INT(nth_master_tx(&g_i2c, 0), -1);
}

/* ====================================================================== */
/*  2. MON-RF poll: exact empty-payload frame + addr                       */
/* ====================================================================== */
TEST(poll_mon_rf_emits_empty_poll_frame)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;

    /* No MON-RF response scripted → returns false after timeout, but the
     * poll frame must already be on the wire. */
    bool ok = max_m10m_poll_mon_rf(&dev);
    ASSERT_TRUE(ok == false);

    int i0 = nth_master_tx(&g_i2c, 0);
    ASSERT_TRUE(i0 >= 0);
    const mock_I2cLog_t *e0 = mock_i2c_log_get(&g_i2c, i0);
    ASSERT_EQ_U(e0->addr8, GPS_ADDR8);
    ASSERT_EQ_INT(e0->len, sizeof(MONRF_POLL));
    ASSERT_EQ_MEM(e0->buf, MONRF_POLL, sizeof(MONRF_POLL));
}

TEST(poll_mon_rf_not_alive_returns_false)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.alive = false;
    ASSERT_TRUE(max_m10m_poll_mon_rf(&dev) == false);
}

/* ====================================================================== */
/*  3. bytes-available register: addr/reg/size + byte order                */
/* ====================================================================== */

/* tick() reads reg 0xFD as a 2-byte big-endian count.  Drive one full
 * IDLE->READ_AVAIL->READ_DATA pass and inspect the bytes-available read. */
TEST(tick_reads_bytes_available_from_reg_FD_8bit_addr)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.parse_state = UBX_PARSE_SYNC1;
    dev.tick_state  = GPS_TICK_IDLE;

    mock_set_millis(1000);

    /* avail = 8, then 8 data bytes (junk; not a full frame). */
    push_avail(&g_i2c, 8);
    uint8_t junk[8] = {1,2,3,4,5,6,7,8};
    mock_i2c_push_rx(&g_i2c, junk, 8);

    max_m10m_tick(&dev);

    /* First log entry is the bytes-available MEM_READ. */
    const mock_I2cLog_t *e = mock_i2c_log_get(&g_i2c, 0);
    ASSERT_TRUE(e != NULL);
    ASSERT_EQ_INT(e->op, MOCK_I2C_MEM_READ);
    ASSERT_EQ_U(e->addr8, GPS_ADDR8);
    ASSERT_EQ_U(e->reg, MAX_M10M_REG_BYTES_HI);   /* 0xFD */
    ASSERT_EQ_INT(e->reg_sz, 1);                  /* I2C_MEMADD_SIZE_8BIT */
    ASSERT_EQ_INT(e->len, 2);
}

/* Big-endian assembly: HI=0x01, LO=0x2C  ->  avail = 0x012C = 300. */
TEST(bytes_available_is_big_endian)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(2000);

    push_avail(&g_i2c, 300);
    /* Provide >= 64 data bytes so the read clamps to 64. */
    uint8_t data[64]; memset(data, 0, sizeof(data));
    mock_i2c_push_rx(&g_i2c, data, 64);

    max_m10m_tick(&dev);

    /* The data-stream read should clamp the 300-byte avail to 64. */
    const mock_I2cLog_t *rd = mock_i2c_log_get(&g_i2c, 1);
    ASSERT_TRUE(rd != NULL);
    ASSERT_EQ_INT(rd->op, MOCK_I2C_MEM_READ);
    ASSERT_EQ_U(rd->reg, MAX_M10M_REG_DATA_STREAM);  /* 0xFF */
    ASSERT_EQ_INT(rd->len, 64);
    /* 300 - 64 = 236 bytes still pending -> stay in READ_DATA. */
    ASSERT_EQ_INT(dev.bytes_avail, 236);
    ASSERT_EQ_INT(dev.tick_state, GPS_TICK_READ_DATA);
}

/* avail == 0 -> no data read, counter bumped, return to IDLE. */
TEST(tick_avail_zero_returns_to_idle_no_data_read)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(5000);

    push_avail(&g_i2c, 0);

    int r = max_m10m_tick(&dev);
    ASSERT_EQ_INT(r, 0);
    ASSERT_EQ_INT(dev.tick_state, GPS_TICK_IDLE);
    ASSERT_EQ_U(dev.dbg_avail_zero, 1);
    /* Only the bytes-available read happened, no data-stream read. */
    ASSERT_EQ_INT(mock_i2c_log_count(&g_i2c), 1);
}

/* avail == 0xFFFF (bus stuck high) -> treated as no data. */
TEST(tick_avail_ffff_returns_to_idle)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(7000);

    push_avail(&g_i2c, 0xFFFF);

    int r = max_m10m_tick(&dev);
    ASSERT_EQ_INT(r, 0);
    ASSERT_EQ_INT(dev.tick_state, GPS_TICK_IDLE);
    ASSERT_EQ_U(dev.dbg_avail_ffff, 1);
}

/* Poll throttle: a second tick within 25 ms of the first must not poll. */
TEST(tick_poll_throttle_25ms)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;

    mock_set_millis(1000);
    push_avail(&g_i2c, 0);
    max_m10m_tick(&dev);
    int after_first = mock_i2c_log_count(&g_i2c);
    ASSERT_EQ_INT(after_first, 1);

    /* +10 ms: below the 25 ms threshold -> no new I2C activity. */
    mock_advance_ms(10);
    max_m10m_tick(&dev);
    ASSERT_EQ_INT(mock_i2c_log_count(&g_i2c), after_first);

    /* +25 ms total: now allowed to poll again. */
    mock_advance_ms(15);
    push_avail(&g_i2c, 0);
    max_m10m_tick(&dev);
    ASSERT_EQ_INT(mock_i2c_log_count(&g_i2c), after_first + 1);
}

/* Not-alive guard: tick returns 0 and touches no I2C. */
TEST(tick_not_alive_no_io)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.alive = false;
    mock_set_millis(1000);
    ASSERT_EQ_INT(max_m10m_tick(&dev), 0);
    ASSERT_EQ_INT(mock_i2c_log_count(&g_i2c), 0);
}

/* ====================================================================== */
/*  4. NAV-PVT parse: full numeric conversion goldens                      */
/* ====================================================================== */

/* Feed a whole NAV-PVT in one tick and verify every parsed field + the
 * convenience-float conversions exactly (these conversions are the spec). */
TEST(tick_parses_navpvt_full_field_set)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(123456);

    /* 100-byte framed NAV-PVT (sync..ck) fits in one 64? No: 100 > 64.
     * The driver clamps to 64 per read and stays in READ_DATA; drive two
     * ticks so the parser sees all 100 bytes. */
    push_avail(&g_i2c, sizeof(NAVPVT));
    mock_i2c_push_rx(&g_i2c, NAVPVT, sizeof(NAVPVT));

    int r1 = max_m10m_tick(&dev);          /* reads first 64 bytes  */
    /* still bytes pending -> second tick reads remaining 36 + parses CK */
    int r2 = max_m10m_tick(&dev);
    int got = r1 | r2;

    ASSERT_EQ_INT(got, 1);                 /* NAV-PVT signalled */
    ASSERT_EQ_U(dev.pvt_count, 1);
    ASSERT_EQ_U(dev.dbg_frames_ok, 1);

    /* Raw integer fields */
    ASSERT_EQ_U(dev.iTOW, 123456789u);
    ASSERT_EQ_INT(dev.year, 2026);
    ASSERT_EQ_INT(dev.month, 6);
    ASSERT_EQ_INT(dev.day, 16);
    ASSERT_EQ_INT(dev.hour, 12);
    ASSERT_EQ_INT(dev.min, 30);
    ASSERT_EQ_INT(dev.sec, 45);
    ASSERT_EQ_U(dev.valid_flags, 0x37);
    ASSERT_EQ_INT(dev.fix_type, GPS_FIX_3D);
    ASSERT_EQ_INT(dev.num_sv, 11);
    ASSERT_EQ_INT(dev.lon_deg7, -1221970000);
    ASSERT_EQ_INT(dev.lat_deg7,  374190000);
    ASSERT_EQ_INT(dev.h_msl_mm,  152400);
    ASSERT_EQ_U(dev.h_acc_mm, 2500);
    ASSERT_EQ_U(dev.v_acc_mm, 3500);
    ASSERT_EQ_INT(dev.vel_n_mm_s, 1000);
    ASSERT_EQ_INT(dev.vel_e_mm_s, -2000);
    ASSERT_EQ_INT(dev.vel_d_mm_s, -5000);
    ASSERT_EQ_U(dev.pDOP, 175);

    /* Convenience-float conversions (golden) */
    ASSERT_NEAR(dev.lat_deg,    37.4190000f, 1e-4);
    ASSERT_NEAR(dev.lon_deg,  -122.1970000f, 1e-4);
    ASSERT_NEAR(dev.alt_msl_m,  152.400f,    1e-3);
    ASSERT_NEAR(dev.vel_d_m_s,   -5.000f,    1e-3);

    /* has_fix latched (fixType >= 2D) and timestamp captured. */
    ASSERT_TRUE(dev.has_fix);
    ASSERT_EQ_U(dev.last_pvt_tick, 123456u);
}

/* A byte-by-byte parser drive (no I2C) is the cleanest characterization of
 * the UBX state machine.  We reach the parser through tick() with 1-byte
 * reads is awkward; instead feed the frame via a single 100-byte avail and
 * the clamped two-read path already covers reassembly.  Here we additionally
 * confirm a corrupted checksum is rejected (no PVT, frame discarded). */
TEST(navpvt_bad_checksum_rejected)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    uint8_t bad[100];
    memcpy(bad, NAVPVT, sizeof(NAVPVT));
    bad[99] ^= 0xFF;                       /* corrupt CK_B */

    push_avail(&g_i2c, sizeof(bad));
    mock_i2c_push_rx(&g_i2c, bad, sizeof(bad));

    int r1 = max_m10m_tick(&dev);
    int r2 = max_m10m_tick(&dev);
    ASSERT_EQ_INT(r1 | r2, 0);
    ASSERT_EQ_U(dev.pvt_count, 0);
    ASSERT_EQ_U(dev.dbg_frames_ok, 0);
    ASSERT_TRUE(!dev.has_fix);
    /* Parser resynchronised back to SYNC1. */
    ASSERT_EQ_INT(dev.parse_state, UBX_PARSE_SYNC1);
}

/* fixType < 2D must NOT set has_fix. */
TEST(navpvt_no_fix_clears_has_fix)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    /* Recompute a NAV-PVT with fixType = 0 (offset 20 in payload =
     * absolute index 6+20 = 26) and fix the CK_B.  Simpler: mutate fix
     * byte and recompute checksum inline. */
    uint8_t f[100];
    memcpy(f, NAVPVT, sizeof(f));
    f[26] = GPS_FIX_NONE;                  /* fixType = 0 */
    /* Recompute Fletcher checksum over class..payload (idx 2..97). */
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 98; i++) { a += f[i]; b += a; }
    f[98] = a; f[99] = b;

    push_avail(&g_i2c, sizeof(f));
    mock_i2c_push_rx(&g_i2c, f, sizeof(f));
    max_m10m_tick(&dev);
    max_m10m_tick(&dev);

    ASSERT_EQ_U(dev.pvt_count, 1);         /* frame still parsed */
    ASSERT_TRUE(!dev.has_fix);             /* but no fix */
    ASSERT_EQ_INT(dev.fix_type, GPS_FIX_NONE);
}

/* ====================================================================== */
/*  5. has_3d_fix gate                                                      */
/* ====================================================================== */
TEST(has_3d_fix_requires_both_flag_and_fixtype3)
{
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));

    dev.has_fix = false; dev.fix_type = GPS_FIX_3D;
    ASSERT_TRUE(max_m10m_has_3d_fix(&dev) == false);

    dev.has_fix = true;  dev.fix_type = GPS_FIX_2D;
    ASSERT_TRUE(max_m10m_has_3d_fix(&dev) == false);

    dev.has_fix = true;  dev.fix_type = GPS_FIX_3D;
    ASSERT_TRUE(max_m10m_has_3d_fix(&dev) == true);

    dev.has_fix = true;  dev.fix_type = GPS_FIX_GNSS_DR;  /* 4 >= 3 */
    ASSERT_TRUE(max_m10m_has_3d_fix(&dev) == true);
}

/* ====================================================================== */
/*  6. EXTI irq stub                                                       */
/* ====================================================================== */
TEST(irq_handler_sets_data_ready)
{
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    ASSERT_TRUE(!dev.data_ready);
    max_m10m_irq_handler(&dev);
    ASSERT_TRUE(dev.data_ready);
}

/* ====================================================================== */
/*  7. init(): reset GPIO sequence + addr8 + exact VALSET stream           */
/* ====================================================================== */

/* Characterize the full init wire behavior: NRST pulse, dev_ready at
 * addr8 = 0x84, CFG-RST cold start frame, then the four config VALSET
 * frames in exact order with golden bytes.
 *
 * MOCK LIMITATION (documented): the I2C RX queue is a single FIFO shared
 * by every read, and init's max_m10m_drain() runs BEFORE the SEND_CFG
 * block — it would consume any pre-scripted ACK bytes.  There is no way to
 * inject a response that only appears AFTER the driver's VALSET write.  So
 * we script nothing: every wait_ack() times out, init_ack_mask stays 0,
 * but the emitted frames (the wire contract we are pinning) are identical
 * to the ACKed case.  init() returns true regardless (alive is gated only
 * on dev_ready, which the mock always ACKs).  ACK-bit registration is
 * separately characterized by parser_ackack_sets_ack_flag. */
TEST(init_full_sequence_goldens)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    mock_set_millis(0);

    bool ok = max_m10m_init(&dev, &g_i2c, g_nrst);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(dev.alive);

    /* NRST pin ended HIGH (released). */
    ASSERT_EQ_INT(mock_gpio_get_state(g_nrst), CASPER_PIN_HIGH);

    /* MASTER_TX #0 = CFG-RST cold start. */
    int rst = nth_master_tx(&g_i2c, 0);
    ASSERT_TRUE(rst >= 0);
    const mock_I2cLog_t *er = mock_i2c_log_get(&g_i2c, rst);
    ASSERT_EQ_U(er->addr8, GPS_ADDR8);
    ASSERT_EQ_INT(er->len, sizeof(RST_FRAME));
    ASSERT_EQ_MEM(er->buf, RST_FRAME, sizeof(RST_FRAME));

    /* MASTER_TX #1..#4 = the four config VALSET frames, in order. */
    const uint8_t *want[4]  = { VS_I2COUTPROT_UBX, VS_I2COUTPROT_NMEA,
                                VS_MSGOUT_NAVPVT,  VS_RATE_MEAS };
    const int      wantn[4] = { 17, 17, 17, 18 };
    for (int k = 0; k < 4; k++) {
        int idx = nth_master_tx(&g_i2c, k + 1);
        ASSERT_TRUE(idx >= 0);
        const mock_I2cLog_t *e = mock_i2c_log_get(&g_i2c, idx);
        ASSERT_EQ_U(e->addr8, GPS_ADDR8);
        ASSERT_EQ_INT(e->len, wantn[k]);
        ASSERT_EQ_MEM(e->buf, want[k], wantn[k]);
    }

    /* Four commands attempted; ACK mask 0 because no response can be
     * injected post-write in the single-FIFO mock (see header note). */
    ASSERT_EQ_INT(dev.init_cmd_count, 4);
    ASSERT_EQ_U(dev.init_ack_mask & 0x0F, 0x00);

    /* A dev_ready was issued at addr8 = 0x84. */
    int found_devready = 0;
    for (int i = 0; i < mock_i2c_log_count(&g_i2c); i++) {
        const mock_I2cLog_t *e = mock_i2c_log_get(&g_i2c, i);
        if (e->op == MOCK_I2C_DEV_READY) {
            ASSERT_EQ_U(e->addr8, GPS_ADDR8);
            found_devready = 1;
            break;
        }
    }
    ASSERT_TRUE(found_devready);
}

/* init_minimal(): NRST pulse + dev_ready + drain, no VALSET frames. */
TEST(init_minimal_no_config_frames)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    mock_set_millis(0);

    /* drain() reads avail; empty FIFO returns 0 -> drain stops immediately. */
    bool ok = max_m10m_init_minimal(&dev, &g_i2c, g_nrst);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(dev.alive);
    ASSERT_EQ_INT(mock_gpio_get_state(g_nrst), CASPER_PIN_HIGH);

    /* No UBX config frames should be emitted (no MASTER_TX). */
    ASSERT_EQ_INT(nth_master_tx(&g_i2c, 0), -1);
}

/* ====================================================================== */
/*  8. ACK/NAK parser path (init_ack_mask bit logic)                       */
/* ====================================================================== */

/* A NAK response must leave the corresponding ack-mask bit clear.  We drive
 * the parser via tick() with an ACK-NAK frame and confirm nak_received. */
TEST(parser_acknak_sets_nak_flag)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    push_avail(&g_i2c, sizeof(ACKNAK));
    mock_i2c_push_rx(&g_i2c, ACKNAK, sizeof(ACKNAK));
    max_m10m_tick(&dev);

    ASSERT_TRUE(dev.nak_received);
    ASSERT_TRUE(!dev.ack_received);
    ASSERT_EQ_U(dev.ack_class, 0x06);   /* echoed class of NAKed msg */
    ASSERT_EQ_U(dev.ack_id,    0x8A);
}

TEST(parser_ackack_sets_ack_flag)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    push_avail(&g_i2c, sizeof(ACKACK));
    mock_i2c_push_rx(&g_i2c, ACKACK, sizeof(ACKACK));
    max_m10m_tick(&dev);

    ASSERT_TRUE(dev.ack_received);
    ASSERT_TRUE(!dev.nak_received);
    ASSERT_EQ_U(dev.ack_class, 0x06);
    ASSERT_EQ_U(dev.ack_id,    0x8A);
}

/* ====================================================================== */
/*  9. Parser resync / sync-byte edge cases                                */
/* ====================================================================== */

/* A lone 0xB5 followed by a non-0x62 must drop back to SYNC1 (unless the
 * next byte is another 0xB5, which keeps us armed).  Verify via a frame
 * preceded by garbage that includes a spurious 0xB5. */
TEST(parser_resyncs_through_garbage_then_parses)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    /* Garbage: 0x00, 0xB5 (false sync), 0x99 (not 0x62 -> reset),
     * then a clean ACK-ACK frame. */
    uint8_t garbage[3] = {0x00, 0xB5, 0x99};
    uint16_t total = (uint16_t)(sizeof(garbage) + sizeof(ACKACK));

    push_avail(&g_i2c, total);
    mock_i2c_push_rx(&g_i2c, garbage, sizeof(garbage));
    mock_i2c_push_rx(&g_i2c, ACKACK, sizeof(ACKACK));

    max_m10m_tick(&dev);
    /* total (13) < 64 -> single read, fully parsed in one tick. */
    ASSERT_TRUE(dev.ack_received);
    ASSERT_EQ_U(dev.dbg_frames_ok, 1);
}

/* Back-to-back 0xB5 0xB5 0x62 ... : the double-sync must still parse. */
TEST(parser_double_sync_b5_b5)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    uint8_t lead[1] = {0xB5};              /* extra sync1 before the frame */
    uint16_t total = (uint16_t)(1 + sizeof(ACKACK));
    push_avail(&g_i2c, total);
    mock_i2c_push_rx(&g_i2c, lead, 1);
    mock_i2c_push_rx(&g_i2c, ACKACK, sizeof(ACKACK));

    max_m10m_tick(&dev);
    ASSERT_TRUE(dev.ack_received);
}

/* Oversized declared length (> parse_buf, 100) must be discarded without
 * overflow, parser resyncs. */
TEST(parser_oversized_length_discarded)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    /* sync,sync,class,id,len=0xFFFF (65535 > 100) -> discard at LEN2. */
    uint8_t f[6] = {0xB5,0x62,0x01,0x07,0xFF,0xFF};
    push_avail(&g_i2c, sizeof(f));
    mock_i2c_push_rx(&g_i2c, f, sizeof(f));

    max_m10m_tick(&dev);
    ASSERT_EQ_U(dev.pvt_count, 0);
    ASSERT_EQ_U(dev.dbg_frames_ok, 0);
    ASSERT_EQ_INT(dev.parse_state, UBX_PARSE_SYNC1);
}

/* ====================================================================== */
/*  10. NMEA passthrough tick                                              */
/* ====================================================================== */

/* tick_nmea buffers bytes into nmea_line until CR/LF, then flags ready. */
TEST(tick_nmea_buffers_line_until_newline)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    mock_set_millis(1000);

    const char *line = "$GNGGA,123\r\n";
    push_avail(&g_i2c, (uint16_t)strlen(line));
    mock_i2c_push_rx(&g_i2c, (const uint8_t*)line, (int)strlen(line));

    int got = max_m10m_tick_nmea(&dev);
    ASSERT_EQ_INT(got, 1);
    ASSERT_TRUE(dev.nmea_line_ready);
    ASSERT_EQ_INT(strcmp(dev.nmea_line, "$GNGGA,123"), 0);
}

/* ====================================================================== */
/*  11. I2C error propagation via log-overflow                            */
/* ====================================================================== */

/* The mock returns CASPER_ERR once the per-bus log fills to MOCK_I2C_LOG_MAX.
 * After saturating the log, a tick's data-stream read fails and dbg_i2c_err
 * increments while no PVT is produced. */
TEST(tick_i2c_error_increments_err_counter)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    dev.tick_state = GPS_TICK_IDLE;
    mock_set_millis(1000);

    /* Saturate the log up to one slot before the cap so the bytes-available
     * read succeeds but the following data-stream read overflows -> ERR. */
    while (mock_i2c_log_count(&g_i2c) < MOCK_I2C_LOG_MAX - 1) {
        (void)casper_i2c_dev_ready(&g_i2c, GPS_ADDR8, 1, 1);
    }
    /* One slot left: push avail so READ_AVAIL succeeds and moves to
     * READ_DATA; the data read then hits the cap and returns ERR. */
    push_avail(&g_i2c, 64);
    uint8_t data[64]; memset(data, 0, sizeof(data));
    mock_i2c_push_rx(&g_i2c, data, 64);

    int r = max_m10m_tick(&dev);
    ASSERT_EQ_INT(r, 0);
    ASSERT_EQ_U(dev.dbg_i2c_err, 1);
    ASSERT_EQ_INT(dev.tick_state, GPS_TICK_IDLE);
}

/* ====================================================================== */
/*  12. MON-RF response parse (antenna diagnostics)                        */
/* ====================================================================== */

/* poll_mon_rf returns true and fills diagnostics when a valid MON-RF frame
 * is scripted as the response. */
TEST(poll_mon_rf_parses_response)
{
    mock_reset();
    g_i2c = (casper_i2c_t)mock_i2c_make("GPS");
    max_m10m_t dev;
    memset(&dev, 0, sizeof(dev));
    dev.i2c = &g_i2c;
    dev.alive = true;
    mock_set_millis(0);

    /* Build a minimal MON-RF: class 0x0A id 0x38, payload len 28.
     * Layout: payload[0..3] header, then RF block #0 at offset 4:
     *   b[2]=ant_status, b[3]=ant_power, b[8..9]=noise, b[10..11]=agc,
     *   b[12]=jam_ind. */
    uint8_t pay[28]; memset(pay, 0, sizeof(pay));
    pay[4 + 2]  = 2;            /* ant_status = OK            */
    pay[4 + 3]  = 1;            /* ant_power  = ON            */
    pay[4 + 8]  = 0x10;         /* noise LSB                  */
    pay[4 + 9]  = 0x00;         /* noise MSB -> 0x0010 = 16   */
    pay[4 + 10] = 0xFF;         /* agc LSB                    */
    pay[4 + 11] = 0x1F;         /* agc MSB -> 0x1FFF = 8191   */
    pay[4 + 12] = 42;           /* jam_ind                    */

    uint8_t frame[8 + 28];
    frame[0] = 0xB5; frame[1] = 0x62; frame[2] = 0x0A; frame[3] = 0x38;
    frame[4] = 28;   frame[5] = 0;
    memcpy(&frame[6], pay, 28);
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6 + 28; i++) { a += frame[i]; b += a; }
    frame[6 + 28] = a; frame[7 + 28] = b;

    push_avail(&g_i2c, sizeof(frame));
    mock_i2c_push_rx(&g_i2c, frame, sizeof(frame));

    bool ok = max_m10m_poll_mon_rf(&dev);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(dev.mon_rf_valid);
    ASSERT_EQ_U(dev.ant_status, 2);
    ASSERT_EQ_U(dev.ant_power, 1);
    ASSERT_EQ_U(dev.rf_noise_per_ms, 16);
    ASSERT_EQ_U(dev.rf_agc_cnt, 8191);
    ASSERT_EQ_U(dev.rf_jam_ind, 42);
}

/* ====================================================================== */
/*  Runner                                                                 */
/* ====================================================================== */
int main(void)
{
    RUN(configure_gps_test_emits_exact_dynmodel_and_lna_frames);
    RUN(configure_gps_test_not_alive_returns_false);
    RUN(poll_mon_rf_emits_empty_poll_frame);
    RUN(poll_mon_rf_not_alive_returns_false);
    RUN(tick_reads_bytes_available_from_reg_FD_8bit_addr);
    RUN(bytes_available_is_big_endian);
    RUN(tick_avail_zero_returns_to_idle_no_data_read);
    RUN(tick_avail_ffff_returns_to_idle);
    RUN(tick_poll_throttle_25ms);
    RUN(tick_not_alive_no_io);
    RUN(tick_parses_navpvt_full_field_set);
    RUN(navpvt_bad_checksum_rejected);
    RUN(navpvt_no_fix_clears_has_fix);
    RUN(has_3d_fix_requires_both_flag_and_fixtype3);
    RUN(irq_handler_sets_data_ready);
    RUN(init_full_sequence_goldens);
    RUN(init_minimal_no_config_frames);
    RUN(parser_acknak_sets_nak_flag);
    RUN(parser_ackack_sets_ack_flag);
    RUN(parser_resyncs_through_garbage_then_parses);
    RUN(parser_double_sync_b5_b5);
    RUN(parser_oversized_length_discarded);
    RUN(tick_nmea_buffers_line_until_newline);
    RUN(tick_i2c_error_increments_err_counter);
    RUN(poll_mon_rf_parses_response);
    return test_summary();
}
