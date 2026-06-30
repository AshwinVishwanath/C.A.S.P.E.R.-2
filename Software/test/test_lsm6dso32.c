/* test_lsm6dso32.c — golden characterization + edge-case lock for
 * App/drivers/lsm6dso32.c against the migrated casper_port seam API.
 *
 * PURPOSE
 *   Pin the EXACT wire protocol, register sequence and numeric conversions of
 *   the LSM6DSO32 SPI IMU driver BEFORE the HAL->seam migration, so the
 *   refactor is provably behaviour-preserving.  The current HAL implementation
 *   IS the golden spec; every byte sequence and computed float below was
 *   extracted from App/drivers/lsm6dso32.c as it stands today.
 *
 * TARGET (migrated) API this suite is written against:
 *     bool    lsm6dso32_init(lsm6dso32_t *dev, casper_spi_t *bus, casper_pin_t cs);
 *     int     lsm6dso32_read(lsm6dso32_t *dev);
 *     uint8_t lsm6dso32_read_reg_ext(lsm6dso32_t *dev, uint8_t reg);
 *     void    lsm6dso32_irq_handler(lsm6dso32_t *dev);
 *   Driver body calls casper_gpio_write(dev->cs, ...) + casper_spi_transceive(
 *   dev->bus, tx, rx, n, 100) + casper_delay_ms(20).  Struct keeps the public
 *   engineering-units fields (accel_g[3], gyro_dps[3], temp_c, raw_temp,
 *   device_id, data_ready).
 *
 *   This file WILL NOT COMPILE until the driver is migrated (the driver header
 *   still pulls in stm32h7xx_hal.h and the init signature still takes HAL
 *   handles).  That RED state is intended — it goes GREEN the moment the
 *   implementation lands.
 *
 * WIRE-PROTOCOL GOLDEN FACTS (from current code):
 *   - SPI access is always full-duplex (HAL_SPI_TransmitReceive -> transceive).
 *   - Read  address byte = reg | 0x80 (MSB set).   Write address byte = reg & 0x7F.
 *   - write_reg: tx={reg&0x7F, val}, len=2.
 *   - read_reg : tx={reg|0x80, 0x00}, len=2, result = rx[1].
 *   - burst    : tx[0]=reg|0x80, len = nbytes+1, data = rx[1..nbytes].
 *   - CS driven LOW before every transaction, HIGH after.
 *   - timeout passed to every transfer = 100 ms.
 *   - init order: CTRL3_C=0x01, delay 20ms, read WHO_AM_I, CTRL3_C=0x44,
 *                 CTRL1_XL=0x74, CTRL2_G=0x7C, CTRL6_C=0x00, CTRL7_G=0x00,
 *                 INT2_CTRL=0x01.   device_id stores WHO_AM_I read-back.
 *   - read(): burst from OUT_TEMP_L(0x20), 14 data bytes.
 *       temp_c   = (int16 LE of buf[0..1]) / 256.0f + 25.0f
 *       gyro_dps = (int16 LE of buf[2..7])  * 0.070f
 *       accel_g  = (int16 LE of buf[8..13]) * 0.000976f
 *       data_ready cleared to false; returns LSM6DSO32_READ_OK (0).
 */
#include "test.h"
#include "board_mock.h"
#include "lsm6dso32.h"

#define TOL 1e-4f

/* Register constants (mirrored from lsm6dso32.h; re-declared here as literals
 * so the golden byte assertions are self-documenting and independent of any
 * later header churn). */
#define R_INT2_CTRL  0x0E
#define R_WHO_AM_I   0x0F
#define R_CTRL1_XL   0x10
#define R_CTRL2_G    0x11
#define R_CTRL3_C    0x12
#define R_CTRL6_C    0x15
#define R_CTRL7_G    0x16
#define R_OUT_TEMP_L 0x20

#define WHO_AM_I_OK  0x6C

/* A unique opaque pin descriptor for the IMU CS line. */
static casper_pin_t CS_PIN = { (void *)0x5000u, 1u << 14 };  /* PC14 on Casper 2 */

/* ------------------------------------------------------------------ */
/*  Helper: find the Nth transceive log entry whose tx[0] == addr     */
/* ------------------------------------------------------------------ */
static const mock_SpiLog_t *find_txn_by_addr(casper_spi_t *bus, uint8_t addr0)
{
    int n = mock_spi_log_count(bus);
    for (int i = 0; i < n; i++) {
        const mock_SpiLog_t *e = mock_spi_log_get(bus, i);
        if (e->op == MOCK_SPI_TRANSCEIVE && e->tx[0] == addr0) return e;
    }
    return NULL;
}

/* ================================================================== */
/*  INIT — register sequence is the golden spec                        */
/* ================================================================== */

/* Every init transaction must be a 2-byte full-duplex transfer with the
 * correct WRITE address byte (reg & 0x7F) and value, in the exact order the
 * datasheet bring-up demands.  WHO_AM_I is a READ (reg | 0x80). */
TEST(init_register_sequence_exact)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;

    /* The mock RX FIFO is shared across ALL transceive calls.  The reset write
     * (transaction 0) consumes 2 bytes first, so pad with 2 dummy bytes, then
     * script the WHO_AM_I read-back (transaction 1) rx[1] = 0x6C. */
    uint8_t init_rx[4] = { 0x00, 0x00, 0x00, WHO_AM_I_OK };
    mock_spi_push_rx(&bus, init_rx, 4);

    bool ok = lsm6dso32_init(&dev, &bus, CS_PIN);
    ASSERT_TRUE(ok);

    /* Exactly 8 SPI transactions during init:
     *   0: CTRL3_C  = 0x01  (soft reset)
     *   1: WHO_AM_I read
     *   2: CTRL3_C  = 0x44
     *   3: CTRL1_XL = 0x74
     *   4: CTRL2_G  = 0x7C
     *   5: CTRL6_C  = 0x00
     *   6: CTRL7_G  = 0x00
     *   7: INT2_CTRL= 0x01
     */
    ASSERT_EQ_INT(mock_spi_log_count(&bus), 8);

    const mock_SpiLog_t *e;

    /* [0] soft reset: write CTRL3_C = 0x01 */
    e = mock_spi_log_get(&bus, 0);
    ASSERT_EQ_INT(e->op, MOCK_SPI_TRANSCEIVE);
    ASSERT_EQ_INT(e->len, 2);
    { uint8_t exp[2] = { R_CTRL3_C & 0x7F, 0x01 }; ASSERT_EQ_MEM(e->tx, exp, 2); }
    ASSERT_EQ_U(e->to_ms, 100);

    /* [1] WHO_AM_I read: tx = {0x0F|0x80, 0x00} */
    e = mock_spi_log_get(&bus, 1);
    ASSERT_EQ_INT(e->op, MOCK_SPI_TRANSCEIVE);
    ASSERT_EQ_INT(e->len, 2);
    { uint8_t exp[2] = { R_WHO_AM_I | 0x80, 0x00 }; ASSERT_EQ_MEM(e->tx, exp, 2); }

    /* [2] CTRL3_C = 0x44  (BDU=1, IF_INC=1) */
    e = mock_spi_log_get(&bus, 2);
    { uint8_t exp[2] = { R_CTRL3_C & 0x7F, 0x44 }; ASSERT_EQ_MEM(e->tx, exp, 2); }

    /* [3] CTRL1_XL = 0x74  (ODR=833Hz, FS=+/-32g) */
    e = mock_spi_log_get(&bus, 3);
    { uint8_t exp[2] = { R_CTRL1_XL & 0x7F, 0x74 }; ASSERT_EQ_MEM(e->tx, exp, 2); }

    /* [4] CTRL2_G = 0x7C  (ODR=833Hz, FS=+/-2000dps) */
    e = mock_spi_log_get(&bus, 4);
    { uint8_t exp[2] = { R_CTRL2_G & 0x7F, 0x7C }; ASSERT_EQ_MEM(e->tx, exp, 2); }

    /* [5] CTRL6_C = 0x00 */
    e = mock_spi_log_get(&bus, 5);
    { uint8_t exp[2] = { R_CTRL6_C & 0x7F, 0x00 }; ASSERT_EQ_MEM(e->tx, exp, 2); }

    /* [6] CTRL7_G = 0x00 */
    e = mock_spi_log_get(&bus, 6);
    { uint8_t exp[2] = { R_CTRL7_G & 0x7F, 0x00 }; ASSERT_EQ_MEM(e->tx, exp, 2); }

    /* [7] INT2_CTRL = 0x01  (accel data-ready -> INT2) */
    e = mock_spi_log_get(&bus, 7);
    { uint8_t exp[2] = { R_INT2_CTRL & 0x7F, 0x01 }; ASSERT_EQ_MEM(e->tx, exp, 2); }
}

/* The WHO_AM_I read-back is stored in dev->device_id and is independent of the
 * returned value (driver does not currently gate init on it). */
TEST(init_stores_whoami_in_device_id)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;

    uint8_t init_rx[4] = { 0x00, 0x00, 0x00, WHO_AM_I_OK };  /* pad + whoami */
    mock_spi_push_rx(&bus, init_rx, 4);

    bool ok = lsm6dso32_init(&dev, &bus, CS_PIN);
    ASSERT_TRUE(ok);
    ASSERT_EQ_U(dev.device_id, WHO_AM_I_OK);
}

/* Even with a wrong WHO_AM_I byte the current driver returns true and records
 * whatever it read.  Golden behaviour: init does NOT fail on bad id. */
TEST(init_does_not_gate_on_bad_whoami)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;

    uint8_t init_rx[4] = { 0x00, 0x00, 0x00, 0xAB };   /* pad + bad whoami (not 0x6C) */
    mock_spi_push_rx(&bus, init_rx, 4);

    bool ok = lsm6dso32_init(&dev, &bus, CS_PIN);
    ASSERT_TRUE(ok);
    ASSERT_EQ_U(dev.device_id, 0xAB);
}

/* init() zeroes the engineering-units arrays + data_ready before configuring. */
TEST(init_clears_state_fields)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;

    /* Poison the struct so we can prove init clears it. */
    memset(&dev, 0xAA, sizeof(dev));

    uint8_t whoami_rx[2] = { 0x00, WHO_AM_I_OK };
    mock_spi_push_rx(&bus, whoami_rx, 2);

    lsm6dso32_init(&dev, &bus, CS_PIN);

    for (int i = 0; i < 3; i++) {
        ASSERT_NEAR(dev.accel_g[i],  0.0f, 0.0f);
        ASSERT_NEAR(dev.gyro_dps[i], 0.0f, 0.0f);
    }
    ASSERT_TRUE(dev.data_ready == false);
}

/* CS sequencing: the soft-reset write (first transaction) drives CS LOW then
 * HIGH.  After all of init, CS is left idle HIGH. */
TEST(init_cs_idle_high_after)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;

    uint8_t whoami_rx[2] = { 0x00, WHO_AM_I_OK };
    mock_spi_push_rx(&bus, whoami_rx, 2);

    lsm6dso32_init(&dev, &bus, CS_PIN);

    /* Last state written to CS must be HIGH (idle / deselected). */
    ASSERT_EQ_INT(mock_gpio_get_state(CS_PIN), CASPER_PIN_HIGH);
}

/* ================================================================== */
/*  READ — burst layout + numeric conversions are the golden spec      */
/* ================================================================== */

/* Build a 14-byte sensor payload (temp[2], gyro[6], accel[6], little-endian)
 * and queue it as the burst rx.  The driver issues ONE 15-byte transceive
 * whose tx[0] = OUT_TEMP_L | 0x80 and reads rx[1..14]. */
static void queue_burst(casper_spi_t *bus,
                        int16_t t, int16_t gx, int16_t gy, int16_t gz,
                        int16_t ax, int16_t ay, int16_t az)
{
    uint8_t payload[15];
    payload[0] = 0x00;                 /* rx[0] echoes the address phase */
    payload[1]  = (uint8_t)(t  & 0xFF); payload[2]  = (uint8_t)((t  >> 8) & 0xFF);
    payload[3]  = (uint8_t)(gx & 0xFF); payload[4]  = (uint8_t)((gx >> 8) & 0xFF);
    payload[5]  = (uint8_t)(gy & 0xFF); payload[6]  = (uint8_t)((gy >> 8) & 0xFF);
    payload[7]  = (uint8_t)(gz & 0xFF); payload[8]  = (uint8_t)((gz >> 8) & 0xFF);
    payload[9]  = (uint8_t)(ax & 0xFF); payload[10] = (uint8_t)((ax >> 8) & 0xFF);
    payload[11] = (uint8_t)(ay & 0xFF); payload[12] = (uint8_t)((ay >> 8) & 0xFF);
    payload[13] = (uint8_t)(az & 0xFF); payload[14] = (uint8_t)((az >> 8) & 0xFF);
    mock_spi_push_rx(bus, payload, 15);
}

/* The read burst is a single 15-byte full-duplex transfer with address byte
 * OUT_TEMP_L | 0x80 and 100 ms timeout. */
TEST(read_burst_is_single_15byte_transfer)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;   /* manual wire-up (skip init noise) */

    queue_burst(&bus, 0, 0, 0, 0, 0, 0, 0);
    int rc = lsm6dso32_read(&dev);
    ASSERT_EQ_INT(rc, LSM6DSO32_READ_OK);

    ASSERT_EQ_INT(mock_spi_log_count(&bus), 1);
    const mock_SpiLog_t *e = mock_spi_log_get(&bus, 0);
    ASSERT_EQ_INT(e->op, MOCK_SPI_TRANSCEIVE);
    ASSERT_EQ_INT(e->len, 15);
    ASSERT_EQ_U(e->tx[0], (uint8_t)(R_OUT_TEMP_L | 0x80));
    ASSERT_EQ_U(e->to_ms, 100);
    /* CS released HIGH after the read. */
    ASSERT_EQ_INT(mock_gpio_get_state(CS_PIN), CASPER_PIN_HIGH);
}

/* Golden conversion: a hand-picked payload maps to exact engineering units.
 *   temp  raw 0x0140 = 320  -> 320/256 + 25 = 26.25
 *   gyro  x 0x03E8 = 1000  -> 70.0 dps ; y -1000 -> -70.0 ; z 0 -> 0
 *   accel x 0x2710 = 10000 -> 9.76 g  ; y -10000 -> -9.76 ; z 0 -> 0 */
TEST(read_converts_units_golden)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;

    queue_burst(&bus,
                /*t */ 320,
                /*gx*/ 1000, /*gy*/ -1000, /*gz*/ 0,
                /*ax*/ 10000, /*ay*/ -10000, /*az*/ 0);

    lsm6dso32_read(&dev);

    ASSERT_EQ_INT(dev.raw_temp, 320);
    ASSERT_NEAR(dev.temp_c, 26.25f, TOL);

    ASSERT_NEAR(dev.gyro_dps[0],  70.0f, TOL);
    ASSERT_NEAR(dev.gyro_dps[1], -70.0f, TOL);
    ASSERT_NEAR(dev.gyro_dps[2],   0.0f, TOL);

    ASSERT_NEAR(dev.accel_g[0],  9.76f, TOL);
    ASSERT_NEAR(dev.accel_g[1], -9.76f, TOL);
    ASSERT_NEAR(dev.accel_g[2],  0.0f, TOL);
}

/* Byte-order / channel-independence: distinct values per axis must not bleed
 * across channels.  Proves the LE assembly and the buf-offset mapping
 * (gyro = bytes 2..7, accel = bytes 8..13) are correct. */
TEST(read_axis_channel_mapping)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;

    queue_burst(&bus,
                /*t */ 0,
                /*gx*/ 100, /*gy*/ 200, /*gz*/ 300,
                /*ax*/ 1000, /*ay*/ 2000, /*az*/ 3000);

    lsm6dso32_read(&dev);

    ASSERT_NEAR(dev.gyro_dps[0], 100.0f * 0.070f, TOL);
    ASSERT_NEAR(dev.gyro_dps[1], 200.0f * 0.070f, TOL);
    ASSERT_NEAR(dev.gyro_dps[2], 300.0f * 0.070f, TOL);

    ASSERT_NEAR(dev.accel_g[0], 1000.0f * 0.000976f, TOL);
    ASSERT_NEAR(dev.accel_g[1], 2000.0f * 0.000976f, TOL);
    ASSERT_NEAR(dev.accel_g[2], 3000.0f * 0.000976f, TOL);
}

/* Boundary values: full-scale int16 extremes convert without overflow and with
 * correct sign (two's-complement assembly via int16 cast). */
TEST(read_boundary_int16_extremes)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;

    queue_burst(&bus,
                /*t */ -256,        /* 0xFF00 -> 24.0 C */
                /*gx*/ 32767, /*gy*/ -32768, /*gz*/ 0,
                /*ax*/ 32767, /*ay*/ -32768, /*az*/ 0);

    lsm6dso32_read(&dev);

    ASSERT_EQ_INT(dev.raw_temp, -256);
    ASSERT_NEAR(dev.temp_c, 24.0f, TOL);

    ASSERT_NEAR(dev.gyro_dps[0],  32767.0f * 0.070f, 1e-2f);
    ASSERT_NEAR(dev.gyro_dps[1], -32768.0f * 0.070f, 1e-2f);

    ASSERT_NEAR(dev.accel_g[0],  32767.0f * 0.000976f, TOL);
    ASSERT_NEAR(dev.accel_g[1], -32768.0f * 0.000976f, TOL);
}

/* Temperature reference offset: raw 0 -> exactly 25.0 C (the datasheet 0-LSB
 * reference point). */
TEST(read_temp_reference_offset)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;

    queue_burst(&bus, 0, 0,0,0, 0,0,0);
    lsm6dso32_read(&dev);

    ASSERT_EQ_INT(dev.raw_temp, 0);
    ASSERT_NEAR(dev.temp_c, 25.0f, TOL);
}

/* read() clears data_ready (consumes the IRQ-set flag). */
TEST(read_clears_data_ready)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;
    dev.data_ready = true;

    queue_burst(&bus, 0, 0,0,0, 0,0,0);
    lsm6dso32_read(&dev);

    ASSERT_TRUE(dev.data_ready == false);
}

/* ================================================================== */
/*  IRQ handler                                                        */
/* ================================================================== */

/* irq_handler only sets data_ready = true; it performs NO bus traffic
 * (ISR-safe). */
TEST(irq_handler_sets_flag_no_bus_traffic)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;
    dev.data_ready = false;

    lsm6dso32_irq_handler(&dev);

    ASSERT_TRUE(dev.data_ready == true);
    ASSERT_EQ_INT(mock_spi_log_count(&bus), 0);
}

/* ================================================================== */
/*  read_reg_ext — single-register read path                           */
/* ================================================================== */

/* read_reg_ext issues a 2-byte transceive {reg|0x80, 0x00} and returns rx[1]. */
TEST(read_reg_ext_addressing_and_value)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;

    /* Script rx so rx[1] = 0x3C is the register value clocked back. */
    uint8_t rx[2] = { 0x00, 0x3C };
    mock_spi_push_rx(&bus, rx, 2);

    uint8_t v = lsm6dso32_read_reg_ext(&dev, 0x1E /* STATUS_REG */);
    ASSERT_EQ_U(v, 0x3C);

    ASSERT_EQ_INT(mock_spi_log_count(&bus), 1);
    const mock_SpiLog_t *e = mock_spi_log_get(&bus, 0);
    ASSERT_EQ_INT(e->op, MOCK_SPI_TRANSCEIVE);
    ASSERT_EQ_INT(e->len, 2);
    { uint8_t exp[2] = { 0x1E | 0x80, 0x00 }; ASSERT_EQ_MEM(e->tx, exp, 2); }
    ASSERT_EQ_INT(mock_gpio_get_state(CS_PIN), CASPER_PIN_HIGH);
}

/* The READ address MSB must be set: any register read sets bit7 of tx[0]. */
TEST(read_reg_ext_sets_read_bit)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;

    uint8_t rx[2] = { 0x00, 0x6C };
    mock_spi_push_rx(&bus, rx, 2);

    (void)lsm6dso32_read_reg_ext(&dev, R_WHO_AM_I);
    const mock_SpiLog_t *e = find_txn_by_addr(&bus, R_WHO_AM_I | 0x80);
    ASSERT_TRUE(e != NULL);
    ASSERT_TRUE((e->tx[0] & 0x80) != 0);
}

/* ================================================================== */
/*  Multiple reads / repeatability                                     */
/* ================================================================== */

/* Two consecutive reads with different payloads produce independent results
 * and exactly one transaction each (no stale-buffer carry-over). */
TEST(read_repeated_independent)
{
    mock_reset();
    casper_spi_t bus = mock_spi_make("IMU_SPI");
    lsm6dso32_t  dev;
    memset(&dev, 0, sizeof(dev));
    dev.bus = &bus; dev.cs = CS_PIN;

    queue_burst(&bus, 0, 1000,0,0, 10000,0,0);
    queue_burst(&bus, 0, 0,0,1000, 0,0,10000);

    lsm6dso32_read(&dev);
    ASSERT_NEAR(dev.gyro_dps[0], 70.0f, TOL);
    ASSERT_NEAR(dev.accel_g[0],  9.76f, TOL);

    lsm6dso32_read(&dev);
    ASSERT_NEAR(dev.gyro_dps[0], 0.0f, TOL);
    ASSERT_NEAR(dev.gyro_dps[2], 70.0f, TOL);
    ASSERT_NEAR(dev.accel_g[0], 0.0f, TOL);
    ASSERT_NEAR(dev.accel_g[2], 9.76f, TOL);

    ASSERT_EQ_INT(mock_spi_log_count(&bus), 2);
}

int main(void)
{
    RUN(init_register_sequence_exact);
    RUN(init_stores_whoami_in_device_id);
    RUN(init_does_not_gate_on_bad_whoami);
    RUN(init_clears_state_fields);
    RUN(init_cs_idle_high_after);

    RUN(read_burst_is_single_15byte_transfer);
    RUN(read_converts_units_golden);
    RUN(read_axis_channel_mapping);
    RUN(read_boundary_int16_extremes);
    RUN(read_temp_reference_offset);
    RUN(read_clears_data_ready);

    RUN(irq_handler_sets_flag_no_bus_traffic);

    RUN(read_reg_ext_addressing_and_value);
    RUN(read_reg_ext_sets_read_bit);

    RUN(read_repeated_independent);

    return test_summary();
}
