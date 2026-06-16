/*
 * test_mmc5983ma.c — host-side characterization + edge-case suite for the
 * MMC5983MA 3-axis magnetometer driver (App/drivers/mmc5983ma.c).
 *
 * WRITTEN BEFORE the HAL->seam migration so the refactor is provably
 * behaviour-preserving. These tests pin the CURRENT (golden) wire protocol,
 * register sequence, and numeric conversions exactly.
 *
 * RED STATE NOTE
 * --------------
 * This file targets the *migrated* (post-refactor) public API in which the
 * driver takes a `casper_i2c_t *` bus handle instead of `I2C_HandleTypeDef *`
 * and calls casper_i2c_* / casper_millis instead of HAL_*. It WILL NOT COMPILE
 * against the current HAL-based driver (mmc5983ma.h still includes
 * stm32h7xx_hal.h and declares I2C_HandleTypeDef* params). That RED state is
 * intended; the suite goes green once the driver is migrated to the seam.
 *
 * Target migrated API (mirrors the existing arity; I2C has no CS/pin param):
 *   bool mmc5983ma_init(mmc5983ma_t *dev, casper_i2c_t *bus);
 *   bool mmc5983ma_init_oneshot(mmc5983ma_t *dev, casper_i2c_t *bus);
 *   int  mmc5983ma_read(mmc5983ma_t *dev);
 *   int  mmc5983ma_trigger_oneshot(mmc5983ma_t *dev);
 *   void mmc5983ma_irq_handler(mmc5983ma_t *dev);
 * and the handle struct's first/bus field is `casper_i2c_t *bus` (was hi2c).
 *
 * GOLDEN SPEC extracted from the pre-migration mmc5983ma.c (do not change):
 *   - All I2C ops use addr8 = 0x60 (7-bit 0x30 << 1), reg_sz = 1 byte.
 *   - init():  W CTRL1(0x0A)=SW_RST(0x80); delay 15ms; R PROD_ID(0x2F) 1B;
 *              if read err -> false; if id!=0x30 -> false;
 *              W CTRL0(0x09)=AUTO_SR_EN(0x20); W CTRL1(0x0A)=BW_800HZ(0x03);
 *              W CTRL2(0x0B)=CMM_EN(0x08)|CM_100HZ(0x05)=0x0D; -> true.
 *   - init_oneshot(): identical to init() but WITHOUT the CTRL2 write.
 *   - read(): burst R X_OUT_0(0x00) 7 bytes; assemble 18-bit per axis;
 *             gauss=(raw-131072)/16384; ut=gauss*100; data_ready=false.
 *   - trigger_oneshot(): W CTRL0(0x09)=TM_M(0x01)|AUTO_SR_EN(0x20)=0x21;
 *             poll R STATUS(0x08) until bit0 (MEAS_M_DONE) set, 5ms timeout
 *             (returns MMC5983MA_ERR_I2C on timeout); then read().
 *   - irq_handler(): sets dev->data_ready = true.
 */

#include "test.h"
#include "board_mock.h"
#include "mmc5983ma.h"

/* ── Golden constants (mirror the driver's #defines for self-checking) ───── */
#define G_ADDR8        0x60u   /* 0x30 << 1                                  */
#define G_REG_X_OUT0   0x00u
#define G_REG_STATUS   0x08u
#define G_REG_CTRL0    0x09u
#define G_REG_CTRL1    0x0Au
#define G_REG_CTRL2    0x0Bu
#define G_REG_PROD_ID  0x2Fu
#define G_PROD_ID_VAL  0x30u

#define G_CTRL1_SW_RST 0x80u
#define G_BW_800HZ     0x03u
#define G_CTRL0_AUTOSR 0x20u
#define G_CTRL0_TM_M   0x01u
#define G_CTRL2_CONT   0x0Du   /* CMM_EN(0x08) | CM_100HZ(0x05)              */

/* Fill an I2C bus log to capacity so the next casper_i2c_* call returns
 * CASPER_ERR (the mock only errors when the log array is full). This lets us
 * exercise the driver's error-propagation paths. */
static void i2c_fill_log_to_cap(casper_i2c_t *bus)
{
    uint8_t junk = 0;
    while (mock_i2c_log_count(bus) < MOCK_I2C_LOG_MAX) {
        casper_i2c_mem_write(bus, G_ADDR8, 0x00, 1, &junk, 1, 1);
    }
}

/* Convenience: assert a logged I2C entry matches op/addr/reg/len. */
static void expect_i2c(const mock_I2cLog_t *e, mock_i2c_op_t op,
                       uint16_t reg, uint16_t len)
{
    ASSERT_TRUE(e != NULL);
    if (!e) return;
    ASSERT_EQ_INT(e->op, op);
    ASSERT_EQ_U(e->addr8, G_ADDR8);
    ASSERT_EQ_U(e->reg_sz, 1u);
    ASSERT_EQ_U(e->reg, reg);
    ASSERT_EQ_U(e->len, len);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  init() — continuous-mode happy path: full golden register sequence
 * ───────────────────────────────────────────────────────────────────────── */
TEST(init_continuous_sequence)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    /* The only read during init is PROD_ID -> return the correct ID. */
    uint8_t id = G_PROD_ID_VAL;
    mock_i2c_push_rx(&i2c, &id, 1);

    mmc5983ma_t dev;
    bool ok = mmc5983ma_init(&dev, &i2c);
    ASSERT_TRUE(ok);

    /* Exactly 5 transactions, in order:
     *   0: W CTRL1 = SW_RST
     *   1: R PROD_ID (1B)
     *   2: W CTRL0 = AUTO_SR_EN
     *   3: W CTRL1 = BW_800HZ
     *   4: W CTRL2 = CMM_EN|CM_100HZ
     */
    ASSERT_EQ_INT(mock_i2c_log_count(&i2c), 5);

    const mock_I2cLog_t *e;

    e = mock_i2c_log_get(&i2c, 0);
    expect_i2c(e, MOCK_I2C_MEM_WRITE, G_REG_CTRL1, 1);
    ASSERT_EQ_U(e->buf[0], G_CTRL1_SW_RST);

    e = mock_i2c_log_get(&i2c, 1);
    expect_i2c(e, MOCK_I2C_MEM_READ, G_REG_PROD_ID, 1);

    e = mock_i2c_log_get(&i2c, 2);
    expect_i2c(e, MOCK_I2C_MEM_WRITE, G_REG_CTRL0, 1);
    ASSERT_EQ_U(e->buf[0], G_CTRL0_AUTOSR);

    e = mock_i2c_log_get(&i2c, 3);
    expect_i2c(e, MOCK_I2C_MEM_WRITE, G_REG_CTRL1, 1);
    ASSERT_EQ_U(e->buf[0], G_BW_800HZ);

    e = mock_i2c_log_get(&i2c, 4);
    expect_i2c(e, MOCK_I2C_MEM_WRITE, G_REG_CTRL2, 1);
    ASSERT_EQ_U(e->buf[0], G_CTRL2_CONT);

    /* product_id recorded; data_ready cleared by hw_init zeroing. */
    ASSERT_EQ_U(dev.product_id, G_PROD_ID_VAL);
    ASSERT_TRUE(dev.data_ready == false);
}

/* The stored I2C address must be the 8-bit shifted value (0x60). */
TEST(init_stores_shifted_addr)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");
    uint8_t id = G_PROD_ID_VAL;
    mock_i2c_push_rx(&i2c, &id, 1);

    mmc5983ma_t dev;
    (void)mmc5983ma_init(&dev, &i2c);
    ASSERT_EQ_U(dev.addr, G_ADDR8);
}

/* init() leaves nav fields zeroed before first read. */
TEST(init_zeroes_fields)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");
    uint8_t id = G_PROD_ID_VAL;
    mock_i2c_push_rx(&i2c, &id, 1);

    mmc5983ma_t dev;
    /* Pre-dirty the struct to prove init clears it. */
    for (int i = 0; i < 3; i++) {
        dev.mag_gauss[i] = 9.9f;
        dev.mag_ut[i]    = 9.9f;
        dev.raw_mag[i]   = 0xDEAD;
    }
    dev.raw_temp   = 0x55;
    dev.data_ready = true;

    (void)mmc5983ma_init(&dev, &i2c);

    for (int i = 0; i < 3; i++) {
        ASSERT_NEAR(dev.mag_gauss[i], 0.0f, 1e-9);
        ASSERT_NEAR(dev.mag_ut[i],    0.0f, 1e-9);
        ASSERT_EQ_U(dev.raw_mag[i], 0u);
    }
    ASSERT_EQ_INT(dev.raw_temp, 0);
    ASSERT_TRUE(dev.data_ready == false);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  init() — product-ID mismatch -> false, and CTRL0/1/2 are NOT written.
 * ───────────────────────────────────────────────────────────────────────── */
TEST(init_wrong_prod_id_fails)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    uint8_t bad_id = 0x42;
    mock_i2c_push_rx(&i2c, &bad_id, 1);

    mmc5983ma_t dev;
    bool ok = mmc5983ma_init(&dev, &i2c);
    ASSERT_TRUE(ok == false);

    /* Only the SW_RST write and the PROD_ID read happened (2 entries);
     * configuration writes are skipped on ID mismatch. */
    ASSERT_EQ_INT(mock_i2c_log_count(&i2c), 2);
    ASSERT_EQ_INT(mock_i2c_log_get(&i2c, 0)->op, MOCK_I2C_MEM_WRITE);
    ASSERT_EQ_INT(mock_i2c_log_get(&i2c, 1)->op, MOCK_I2C_MEM_READ);

    /* product_id field still captured the (wrong) value. */
    ASSERT_EQ_U(dev.product_id, bad_id);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  init() — PROD_ID read I2C error -> false (error propagation).
 * ───────────────────────────────────────────────────────────────────────── */
TEST(init_prod_id_read_error_fails)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    /* Fill the log so EVERY subsequent casper_i2c_* call returns CASPER_ERR.
     * The driver ignores the SW_RST write return but must bail on the
     * PROD_ID read != CASPER_OK -> init returns false. */
    i2c_fill_log_to_cap(&i2c);

    mmc5983ma_t dev;
    bool ok = mmc5983ma_init(&dev, &i2c);
    ASSERT_TRUE(ok == false);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  init_oneshot() — same as init() but NO CTRL2 (continuous) write.
 * ───────────────────────────────────────────────────────────────────────── */
TEST(init_oneshot_skips_ctrl2)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");
    uint8_t id = G_PROD_ID_VAL;
    mock_i2c_push_rx(&i2c, &id, 1);

    mmc5983ma_t dev;
    bool ok = mmc5983ma_init_oneshot(&dev, &i2c);
    ASSERT_TRUE(ok);

    /* 4 transactions: W SW_RST, R PROD_ID, W CTRL0, W CTRL1. No CTRL2. */
    ASSERT_EQ_INT(mock_i2c_log_count(&i2c), 4);

    expect_i2c(mock_i2c_log_get(&i2c, 0), MOCK_I2C_MEM_WRITE, G_REG_CTRL1, 1);
    ASSERT_EQ_U(mock_i2c_log_get(&i2c, 0)->buf[0], G_CTRL1_SW_RST);
    expect_i2c(mock_i2c_log_get(&i2c, 1), MOCK_I2C_MEM_READ, G_REG_PROD_ID, 1);
    expect_i2c(mock_i2c_log_get(&i2c, 2), MOCK_I2C_MEM_WRITE, G_REG_CTRL0, 1);
    ASSERT_EQ_U(mock_i2c_log_get(&i2c, 2)->buf[0], G_CTRL0_AUTOSR);
    expect_i2c(mock_i2c_log_get(&i2c, 3), MOCK_I2C_MEM_WRITE, G_REG_CTRL1, 1);
    ASSERT_EQ_U(mock_i2c_log_get(&i2c, 3)->buf[0], G_BW_800HZ);

    /* No CTRL2 write must appear anywhere. */
    for (int i = 0; i < mock_i2c_log_count(&i2c); i++) {
        const mock_I2cLog_t *e = mock_i2c_log_get(&i2c, i);
        if (e->op == MOCK_I2C_MEM_WRITE)
            ASSERT_TRUE(e->reg != G_REG_CTRL2);
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 *  read() — golden midpoint vector decodes to exactly 0 Gauss / 0 uT.
 *  raw=131072 (2^17) -> 0 Gauss for every axis.
 *  byte layout: buf0=0x80,buf1=0x00 -> (0x80<<10)|(0<<2) = 131072, low2=0.
 * ───────────────────────────────────────────────────────────────────────── */
TEST(read_midpoint_zero_field)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    uint8_t frame[7] = { 0x80, 0x00,   /* X hi,lo */
                         0x80, 0x00,   /* Y hi,lo */
                         0x80, 0x00,   /* Z hi,lo */
                         0x00 };       /* XYZ_OUT_2 low-2 bits all 0 */
    mock_i2c_push_rx(&i2c, frame, 7);

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;
    dev.data_ready = true;

    int rc = mmc5983ma_read(&dev);
    ASSERT_EQ_INT(rc, MMC5983MA_OK);

    /* Exactly one burst read of 7 bytes from X_OUT_0. */
    ASSERT_EQ_INT(mock_i2c_log_count(&i2c), 1);
    expect_i2c(mock_i2c_log_get(&i2c, 0), MOCK_I2C_MEM_READ, G_REG_X_OUT0, 7);

    for (int i = 0; i < 3; i++) {
        ASSERT_EQ_U(dev.raw_mag[i], 131072u);
        ASSERT_NEAR(dev.mag_gauss[i], 0.0f, 1e-6);
        ASSERT_NEAR(dev.mag_ut[i],    0.0f, 1e-4);
    }
    /* read() always clears data_ready. */
    ASSERT_TRUE(dev.data_ready == false);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  read() — distinct per-axis vector: pins 18-bit assembly, byte order,
 *  low-2-bit packing from XYZ_OUT_2, and gauss/uT conversion.
 *
 *  frame = {0xAB,0xCD, 0x12,0x34, 0xFF,0xFF, 0xE4}
 *  XYZ_OUT_2 = 0xE4 = 0b1110_0100
 *    X low2 = (0xE4>>6)&3 = 3
 *    Y low2 = (0xE4>>4)&3 = 2
 *    Z low2 = (0xE4>>2)&3 = 1
 *  raw_X = (0xAB<<10)|(0xCD<<2)|3 = 175927
 *  raw_Y = (0x12<<10)|(0x34<<2)|2 = 18642
 *  raw_Z = (0xFF<<10)|(0xFF<<2)|1 = 262141
 * ───────────────────────────────────────────────────────────────────────── */
TEST(read_distinct_axes_decode)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    uint8_t frame[7] = { 0xAB, 0xCD, 0x12, 0x34, 0xFF, 0xFF, 0xE4 };
    mock_i2c_push_rx(&i2c, frame, 7);

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;

    int rc = mmc5983ma_read(&dev);
    ASSERT_EQ_INT(rc, MMC5983MA_OK);

    ASSERT_EQ_U(dev.raw_mag[0], 175927u);
    ASSERT_EQ_U(dev.raw_mag[1], 18642u);
    ASSERT_EQ_U(dev.raw_mag[2], 262141u);

    /* gauss = (raw - 131072) / 16384 */
    ASSERT_NEAR(dev.mag_gauss[0],  2.73773193359375f, 1e-5);
    ASSERT_NEAR(dev.mag_gauss[1], -6.86218261718750f, 1e-5);
    ASSERT_NEAR(dev.mag_gauss[2],  7.99981689453125f, 1e-5);

    /* uT = gauss * 100 */
    ASSERT_NEAR(dev.mag_ut[0],  273.773193359375f, 1e-3);
    ASSERT_NEAR(dev.mag_ut[1], -686.218261718750f, 1e-3);
    ASSERT_NEAR(dev.mag_ut[2],  799.981689453125f, 1e-3);
}

/* read() — all-0xFF frame: maximum 18-bit raw (262143) per axis. */
TEST(read_max_field)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    uint8_t frame[7] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    mock_i2c_push_rx(&i2c, frame, 7);

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;

    int rc = mmc5983ma_read(&dev);
    ASSERT_EQ_INT(rc, MMC5983MA_OK);

    for (int i = 0; i < 3; i++) {
        ASSERT_EQ_U(dev.raw_mag[i], 262143u);
        ASSERT_NEAR(dev.mag_gauss[i], 7.99993896484375f, 1e-5);
        ASSERT_NEAR(dev.mag_ut[i],    799.993896484375f, 1e-3);
    }
}

/* read() — only the upper-2 bits of XYZ_OUT_2 reach X; bits[1:0] are ignored.
 * Confirms the >>6/>>4/>>2 & 0x03 masking does not leak adjacent axis bits. */
TEST(read_xyz_out2_bit_isolation)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    /* XYZ_OUT_2 = 0x03 -> only bits[1:0] set. Those belong to NO axis
     * (axes use bits 7:6, 5:4, 3:2). So every axis low2 = 0. */
    uint8_t frame[7] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };
    mock_i2c_push_rx(&i2c, frame, 7);

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;

    (void)mmc5983ma_read(&dev);
    ASSERT_EQ_U(dev.raw_mag[0], 0u);
    ASSERT_EQ_U(dev.raw_mag[1], 0u);
    ASSERT_EQ_U(dev.raw_mag[2], 0u);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  read() — I2C burst error -> MMC5983MA_ERR_I2C, fields untouched.
 * ───────────────────────────────────────────────────────────────────────── */
TEST(read_i2c_error_propagates)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;
    dev.raw_mag[0] = 0xCAFE;   /* sentinel that must survive a failed read */

    /* Force the burst read to fail. */
    i2c_fill_log_to_cap(&i2c);

    int rc = mmc5983ma_read(&dev);
    ASSERT_EQ_INT(rc, MMC5983MA_ERR_I2C);
    /* Driver returns before touching raw_mag on burst failure. */
    ASSERT_EQ_U(dev.raw_mag[0], 0xCAFEu);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  trigger_oneshot() — happy path: TM_M trigger, one poll that's already
 *  done, then a 7-byte burst read decoded normally.
 * ───────────────────────────────────────────────────────────────────────── */
TEST(trigger_oneshot_happy_path)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");
    mock_set_millis(1000);

    /* RX queue is consumed FIFO across reads:
     *   1) STATUS read -> 0x01 (MEAS_M_DONE set immediately)
     *   2) 7-byte burst -> midpoint frame (raw=131072 -> 0 Gauss)
     */
    uint8_t status_done = 0x01;
    uint8_t frame[7] = { 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00 };
    mock_i2c_push_rx(&i2c, &status_done, 1);
    mock_i2c_push_rx(&i2c, frame, 7);

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;

    int rc = mmc5983ma_trigger_oneshot(&dev);
    ASSERT_EQ_INT(rc, MMC5983MA_OK);

    /* Sequence: W CTRL0=TM_M|AUTO_SR_EN, R STATUS, R X_OUT_0(7). */
    ASSERT_EQ_INT(mock_i2c_log_count(&i2c), 3);

    const mock_I2cLog_t *e = mock_i2c_log_get(&i2c, 0);
    expect_i2c(e, MOCK_I2C_MEM_WRITE, G_REG_CTRL0, 1);
    ASSERT_EQ_U(e->buf[0], (uint8_t)(G_CTRL0_TM_M | G_CTRL0_AUTOSR)); /* 0x21 */

    expect_i2c(mock_i2c_log_get(&i2c, 1), MOCK_I2C_MEM_READ, G_REG_STATUS, 1);
    expect_i2c(mock_i2c_log_get(&i2c, 2), MOCK_I2C_MEM_READ, G_REG_X_OUT0, 7);

    for (int i = 0; i < 3; i++)
        ASSERT_NEAR(dev.mag_gauss[i], 0.0f, 1e-6);
}

/* NOTE on trigger_oneshot() timeout coverage:
 *
 * The driver's poll loop is `start = casper_millis(); while(!done){ if
 * casper_millis()-start > 5 return ERR; read STATUS; }`. The host mock clock
 * (board_mock.c) does NOT auto-advance on casper_millis() or on each I2C read,
 * so if STATUS never reports done the loop would spin forever and HANG the
 * host test. There is no mock hook to advance virtual time mid-spin. The
 * timeout branch is therefore NOT covered here by a live call (a hanging test
 * is worse than an uncovered branch). See coverage gaps for the recommended
 * mock enhancement (auto-advance clock per casper_millis() call) that would
 * let this branch be exercised deterministically. The happy-path test above
 * already covers the "status done immediately" branch of the same loop.
 */

/* trigger_oneshot() — CTRL0 trigger byte is exactly TM_M | AUTO_SR_EN even
 * when the very first STATUS poll already reports done. */
TEST(trigger_oneshot_trigger_byte)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");

    uint8_t status_done = 0x01;
    uint8_t frame[7] = { 0,0,0,0,0,0,0 };
    mock_i2c_push_rx(&i2c, &status_done, 1);
    mock_i2c_push_rx(&i2c, frame, 7);

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;

    (void)mmc5983ma_trigger_oneshot(&dev);

    const mock_I2cLog_t *e = mock_i2c_log_get(&i2c, 0);
    ASSERT_EQ_INT(e->op, MOCK_I2C_MEM_WRITE);
    ASSERT_EQ_U(e->reg, G_REG_CTRL0);
    ASSERT_EQ_U(e->buf[0], 0x21u);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  irq_handler() — sets data_ready true (and only that).
 * ───────────────────────────────────────────────────────────────────────── */
TEST(irq_handler_sets_data_ready)
{
    mock_reset();
    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.data_ready = false;
    mmc5983ma_irq_handler(&dev);
    ASSERT_TRUE(dev.data_ready == true);
}

/* read() clears data_ready that irq_handler set (consumer pattern). */
TEST(read_clears_irq_data_ready)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");
    uint8_t frame[7] = { 0x80,0x00,0x80,0x00,0x80,0x00,0x00 };
    mock_i2c_push_rx(&i2c, frame, 7);

    mmc5983ma_t dev = (mmc5983ma_t){0};
    dev.bus  = &i2c;
    dev.addr = G_ADDR8;

    mmc5983ma_irq_handler(&dev);
    ASSERT_TRUE(dev.data_ready == true);

    (void)mmc5983ma_read(&dev);
    ASSERT_TRUE(dev.data_ready == false);
}

/* ─────────────────────────────────────────────────────────────────────────
 *  Timeout argument: all I2C ops must pass the 50ms driver timeout (I2C_TIMEOUT).
 * ───────────────────────────────────────────────────────────────────────── */
TEST(i2c_timeout_argument_is_50ms)
{
    mock_reset();
    casper_i2c_t i2c = mock_i2c_make("I2C_MAG");
    uint8_t id = G_PROD_ID_VAL;
    mock_i2c_push_rx(&i2c, &id, 1);

    mmc5983ma_t dev;
    (void)mmc5983ma_init(&dev, &i2c);

    for (int i = 0; i < mock_i2c_log_count(&i2c); i++) {
        const mock_I2cLog_t *e = mock_i2c_log_get(&i2c, i);
        ASSERT_EQ_U(e->to_ms, 50u);
    }
}

int main(void)
{
    RUN(init_continuous_sequence);
    RUN(init_stores_shifted_addr);
    RUN(init_zeroes_fields);
    RUN(init_wrong_prod_id_fails);
    RUN(init_prod_id_read_error_fails);
    RUN(init_oneshot_skips_ctrl2);
    RUN(read_midpoint_zero_field);
    RUN(read_distinct_axes_decode);
    RUN(read_max_field);
    RUN(read_xyz_out2_bit_isolation);
    RUN(read_i2c_error_propagates);
    RUN(trigger_oneshot_happy_path);
    RUN(trigger_oneshot_trigger_byte);
    RUN(irq_handler_sets_data_ready);
    RUN(read_clears_irq_data_ready);
    RUN(i2c_timeout_argument_is_50ms);
    return test_summary();
}
