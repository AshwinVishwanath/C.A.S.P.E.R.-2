/*
 * test_ms5611.c — host-side characterization + edge-case suite for App/drivers/ms5611.c
 *
 * AUTHORED BEFORE THE HAL->SEAM MIGRATION.  These tests pin the EXACT current
 * wire protocol, register/command bytes, byte order, state-machine ordering,
 * and numeric conversions of the MS5611 driver so the upcoming refactor is
 * provably behaviour-preserving.
 *
 * They are written against the TARGET (post-migration) public API:
 *   bool  ms5611_init(ms5611_t*, casper_spi_t* bus, casper_pin_t cs);
 *   ...with the struct fields  dev->bus (casper_spi_t*) and dev->cs (casper_pin_t)
 *   replacing  hspi / cs_port / cs_pin.
 *
 * => These tests INTENTIONALLY DO NOT COMPILE until ms5611.{c,h} is migrated to
 *    the casper_port seam (RED state).  They go GREEN the moment the
 *    behaviour-preserving migration lands.  Do NOT relax them to match HAL.
 *
 * Golden numeric values were generated from the exact float math in
 * ms5611_compute()/ms5611_get_altitude() with the MS5611 datasheet example
 * PROM + raw ADC values (D1=9085466, D2=8569150 → 2007 / 100009).
 *
 * Build (wired by run.ps1):
 *   gcc ... test/test_ms5611.c App/drivers/ms5611.c test/board_mock.c
 *       -Itest -IApp/port -IApp/drivers -lm
 */

#include "test.h"
#include "board_mock.h"
#include "ms5611.h"

/* ----------------------------------------------------------------------- */
/*  Command / register bytes from ms5611.h (the golden wire constants)      */
/* ----------------------------------------------------------------------- */
/* MS5611_CMD_RESET      0x1E
 * MS5611_CMD_READ_ADC   0x00
 * MS5611_CMD_READ_PROM  0xA0   (+ reg*2)
 * MS5611_CMD_CONVERT_D1 0x40   (+ osr_idx*2)
 * MS5611_CMD_CONVERT_D2 0x50   (+ osr_idx*2)                                */

/* Datasheet example factory PROM values (C1..C6) — index 0 is the reserved
 * factory/setup word, indices 1..6 are the calibration coefficients. */
#define PROM0 0x1234u  /* reserved word (factory data + setup, non-zero) */
#define PROM1 40127u
#define PROM2 36924u
#define PROM3 23317u
#define PROM4 23282u
#define PROM5 33464u
#define PROM6 28312u

/* Datasheet example raw conversions */
#define RAW_D1 9085466u   /* pressure  */
#define RAW_D2 8569150u   /* temperature */

/* ----------------------------------------------------------------------- */
/*  Shared fixtures                                                         */
/* ----------------------------------------------------------------------- */

static casper_spi_t g_spi;
static casper_pin_t g_cs = { (void *)0x4011C000, (uint32_t)(1u << 11) }; /* PE11-ish */

/* Push one PROM word as the 2 RX bytes returned by a PROM read (MSB first). */
static void push_prom(casper_spi_t *bus, uint16_t w)
{
    uint8_t b[2] = { (uint8_t)(w >> 8), (uint8_t)(w & 0xFF) };
    mock_spi_push_rx(bus, b, 2);
}

/* Push a 24-bit ADC result as the 3 RX bytes (MSB first). */
static void push_adc24(casper_spi_t *bus, uint32_t v)
{
    uint8_t b[3] = { (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)(v & 0xFF) };
    mock_spi_push_rx(bus, b, 3);
}

/* Queue the full RX stream that ms5611_init() consumes:
 * 7 PROM reads (reg 0..6), each returning 2 bytes. */
static void push_init_prom_stream(casper_spi_t *bus)
{
    push_prom(bus, PROM0);
    push_prom(bus, PROM1);
    push_prom(bus, PROM2);
    push_prom(bus, PROM3);
    push_prom(bus, PROM4);
    push_prom(bus, PROM5);
    push_prom(bus, PROM6);
}

/* Run a successful init with the datasheet PROM and return its bool result. */
static bool do_init(ms5611_t *dev)
{
    mock_reset();
    g_spi = (casper_spi_t)mock_spi_make("MS5611_SPI");
    push_init_prom_stream(&g_spi);
    return ms5611_init(dev, &g_spi, g_cs);
}

/* ======================================================================= */
/*  INIT — reset command + PROM read wire protocol                          */
/* ======================================================================= */

/* The very first SPI op must be a 1-byte transmit of MS5611_CMD_RESET. */
TEST(init_emits_reset_command_first)
{
    ms5611_t dev = {0};
    do_init(&dev);

    ASSERT_TRUE(mock_spi_log_count(&g_spi) >= 1);
    const mock_SpiLog_t *e0 = mock_spi_log_get(&g_spi, 0);
    ASSERT_EQ_INT(e0->op, MOCK_SPI_TX);
    ASSERT_EQ_INT(e0->len, 1);
    ASSERT_EQ_U(e0->tx[0], 0x1Eu);   /* MS5611_CMD_RESET */
}

/* After reset, init reads PROM regs 0..6.  Each read is a separate
 * TX(cmd=0xA0+reg*2, 1 byte) followed by RX(2 bytes).  Verify the exact
 * command byte sequence and the TX/RX op interleave. */
TEST(init_prom_read_command_sequence)
{
    ms5611_t dev = {0};
    do_init(&dev);

    /* Layout: [0]=RESET TX, then for each of 7 regs: TX(cmd) + RX(2). */
    int n = mock_spi_log_count(&g_spi);
    ASSERT_EQ_INT(n, 1 + 7 * 2);   /* 1 reset + 7*(tx+rx) = 15 */

    for (uint8_t reg = 0; reg < 7; reg++) {
        int tx_idx = 1 + reg * 2;
        int rx_idx = tx_idx + 1;
        const mock_SpiLog_t *tx = mock_spi_log_get(&g_spi, tx_idx);
        const mock_SpiLog_t *rx = mock_spi_log_get(&g_spi, rx_idx);

        ASSERT_EQ_INT(tx->op, MOCK_SPI_TX);
        ASSERT_EQ_INT(tx->len, 1);
        ASSERT_EQ_U(tx->tx[0], (uint8_t)(0xA0u + reg * 2u));  /* READ_PROM + reg*2 */

        ASSERT_EQ_INT(rx->op, MOCK_SPI_RX);
        ASSERT_EQ_INT(rx->len, 2);
    }
}

/* PROM words are big-endian: buf[0]<<8 | buf[1].  Verify the stored raw
 * prom[] array exactly matches what was clocked in (byte order golden). */
TEST(init_prom_byte_order_and_storage)
{
    ms5611_t dev = {0};
    do_init(&dev);

    ASSERT_EQ_U(dev.prom[0], PROM0);
    ASSERT_EQ_U(dev.prom[1], PROM1);
    ASSERT_EQ_U(dev.prom[2], PROM2);
    ASSERT_EQ_U(dev.prom[3], PROM3);
    ASSERT_EQ_U(dev.prom[4], PROM4);
    ASSERT_EQ_U(dev.prom[5], PROM5);
    ASSERT_EQ_U(dev.prom[6], PROM6);
}

/* CS must idle HIGH after init completes (last write to cs is HIGH). */
TEST(init_cs_idle_high)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ASSERT_EQ_INT(mock_gpio_get_state(g_cs), CASPER_PIN_HIGH);
}

/* Default OSR after init is MS5611_OSR_4096; compensation enabled. */
TEST(init_defaults)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ASSERT_EQ_INT(dev.osr, MS5611_OSR_4096);
    ASSERT_TRUE(dev.compensation == true);
    ASSERT_EQ_INT(dev.last_result, MS5611_ERROR); /* no read yet */
}

/* Scaling constants: C[reg] = prescale * prom[reg].  Pin the exact products
 * that feed ms5611_compute().
 *
 * NOTE: the reset() multiply loop runs over ALL 7 indices, so even C[0]
 * (prescale 1.0) is multiplied by prom[0] — C[0] == prom[0], NOT 1.0.
 * This is a deliberate golden capture of the current behaviour. */
TEST(init_scaled_constants)
{
    ms5611_t dev = {0};
    do_init(&dev);

    ASSERT_NEAR(dev.C[0], 1.0f            * PROM0,     1e-3f);
    ASSERT_NEAR(dev.C[1], 32768.0f       * PROM1,     1.0f);
    ASSERT_NEAR(dev.C[2], 65536.0f       * PROM2,     1.0f);
    ASSERT_NEAR(dev.C[3], 3.90625E-3f    * PROM3,     1e-3f);
    ASSERT_NEAR(dev.C[4], 7.8125E-3f     * PROM4,     1e-3f);
    ASSERT_NEAR(dev.C[5], 256.0f         * PROM5,     1.0f);
    ASSERT_NEAR(dev.C[6], 1.1920928955E-7f * PROM6,   1e-9f);
}

/* device_id is folded as (id<<4) ^ prom_word over all 7 words. */
TEST(init_device_id_fold)
{
    ms5611_t dev = {0};
    do_init(&dev);

    uint32_t id = 0;
    uint16_t w[7] = { PROM0, PROM1, PROM2, PROM3, PROM4, PROM5, PROM6 };
    for (int i = 0; i < 7; i++) { id <<= 4; id ^= w[i]; }
    ASSERT_EQ_U(dev.device_id, id);
}

/* ROM check: init returns true when all calibration words (reg 1..6) are
 * non-zero. */
TEST(init_returns_true_on_good_prom)
{
    ms5611_t dev = {0};
    bool ok = do_init(&dev);
    ASSERT_TRUE(ok == true);
}

/* ROM check edge: a zero in any calibration word (reg>0) => init returns
 * false.  reg 0 == 0 is allowed (reserved word). */
TEST(init_returns_false_on_zero_cal_word)
{
    mock_reset();
    g_spi = (casper_spi_t)mock_spi_make("MS5611_SPI");
    push_prom(&g_spi, PROM0);
    push_prom(&g_spi, PROM1);
    push_prom(&g_spi, 0u);       /* reg 2 == 0 → bad */
    push_prom(&g_spi, PROM3);
    push_prom(&g_spi, PROM4);
    push_prom(&g_spi, PROM5);
    push_prom(&g_spi, PROM6);

    ms5611_t dev = {0};
    bool ok = ms5611_init(&dev, &g_spi, g_cs);
    ASSERT_TRUE(ok == false);
}

/* reg0 == 0 must NOT fail the ROM check (only reg>0 words are validated). */
TEST(init_reg0_zero_allowed)
{
    mock_reset();
    g_spi = (casper_spi_t)mock_spi_make("MS5611_SPI");
    push_prom(&g_spi, 0u);       /* reg 0 == 0 is allowed */
    push_prom(&g_spi, PROM1);
    push_prom(&g_spi, PROM2);
    push_prom(&g_spi, PROM3);
    push_prom(&g_spi, PROM4);
    push_prom(&g_spi, PROM5);
    push_prom(&g_spi, PROM6);

    ms5611_t dev = {0};
    bool ok = ms5611_init(&dev, &g_spi, g_cs);
    ASSERT_TRUE(ok == true);
}

/* ======================================================================= */
/*  BLOCKING READ — ms5611_read() command sequence + numeric golden         */
/* ======================================================================= */

/* ms5611_read() issues: CONVERT_D1, READ_ADC(3), CONVERT_D2, READ_ADC(3).
 * Verify the exact command bytes for OSR_4096 (idx=4):
 *   D1 conv = 0x40 + 8 = 0x48 ; D2 conv = 0x50 + 8 = 0x58 ; READ_ADC = 0x00. */
TEST(read_command_sequence_osr4096)
{
    ms5611_t dev = {0};
    do_init(&dev);

    /* Re-arm log inspection: queue the two ADC results read() will consume. */
    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, RAW_D2);

    int base = mock_spi_log_count(&g_spi);  /* ops logged so far (init) */
    int rc = ms5611_read(&dev);
    ASSERT_EQ_INT(rc, MS5611_READ_OK);

    /* Expected new ops after init:
     *  [+0] TX CONVERT_D1 (0x48)
     *  [+1] TX READ_ADC   (0x00)   <- ms5611_command-style? No: read_adc = TX(1)+RX(3)
     *  ...                                                                       */
    const mock_SpiLog_t *d1cmd = mock_spi_log_get(&g_spi, base + 0);
    ASSERT_EQ_INT(d1cmd->op, MOCK_SPI_TX);
    ASSERT_EQ_INT(d1cmd->len, 1);
    ASSERT_EQ_U(d1cmd->tx[0], 0x48u);          /* CONVERT_D1 + 4*2 */

    const mock_SpiLog_t *d1adc_cmd = mock_spi_log_get(&g_spi, base + 1);
    ASSERT_EQ_INT(d1adc_cmd->op, MOCK_SPI_TX);
    ASSERT_EQ_U(d1adc_cmd->tx[0], 0x00u);      /* READ_ADC */

    const mock_SpiLog_t *d1adc_rx = mock_spi_log_get(&g_spi, base + 2);
    ASSERT_EQ_INT(d1adc_rx->op, MOCK_SPI_RX);
    ASSERT_EQ_INT(d1adc_rx->len, 3);

    const mock_SpiLog_t *d2cmd = mock_spi_log_get(&g_spi, base + 3);
    ASSERT_EQ_INT(d2cmd->op, MOCK_SPI_TX);
    ASSERT_EQ_U(d2cmd->tx[0], 0x58u);          /* CONVERT_D2 + 4*2 */

    const mock_SpiLog_t *d2adc_cmd = mock_spi_log_get(&g_spi, base + 4);
    ASSERT_EQ_INT(d2adc_cmd->op, MOCK_SPI_TX);
    ASSERT_EQ_U(d2adc_cmd->tx[0], 0x00u);

    const mock_SpiLog_t *d2adc_rx = mock_spi_log_get(&g_spi, base + 5);
    ASSERT_EQ_INT(d2adc_rx->op, MOCK_SPI_RX);
    ASSERT_EQ_INT(d2adc_rx->len, 3);
}

/* Numeric golden: datasheet example → temperature 2007 (0.01C), pressure
 * 100009 Pa, temp_C = 20.07. */
TEST(read_compute_golden_warm)
{
    ms5611_t dev = {0};
    do_init(&dev);
    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, RAW_D2);

    ms5611_read(&dev);

    ASSERT_EQ_INT(dev.temperature, 2007);
    ASSERT_EQ_INT(dev.pressure,    100009);
    ASSERT_NEAR(ms5611_get_temperature(&dev), 20.07f, 1e-4f);
    ASSERT_EQ_INT(dev.last_result, MS5611_READ_OK);
}

/* last_read is stamped from casper_millis() at end of read().
 *
 * ms5611_read() runs two blocking ms5611_convert() calls; each calls
 * casper_delay_ms(delay_ms).  On the host mock, delay advances the virtual
 * clock.  At the default OSR_4096 delay_ms = ceil(9100/1000) = 10 ms, so the
 * clock advances 2*10 = 20 ms across the read.  last_read is stamped AFTER
 * both converts → 123456 + 20 = 123476.  This pins the timing relationship. */
TEST(read_stamps_last_read_millis)
{
    ms5611_t dev = {0};
    do_init(&dev);                 /* default OSR_4096, delay_ms unset until... */
    ms5611_set_oversampling(&dev, MS5611_OSR_4096); /* ...explicit for clarity */
    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, RAW_D2);

    mock_set_millis(123456u);
    ms5611_read(&dev);
    ASSERT_EQ_U(dev.last_read, 123456u + 20u);
}

/* OSR command byte changes with oversampling.  OSR_256 (idx0): D1=0x40,
 * D2=0x50. */
TEST(read_command_sequence_osr256)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_256);

    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, RAW_D2);
    int base = mock_spi_log_count(&g_spi);
    ms5611_read(&dev);

    ASSERT_EQ_U(mock_spi_log_get(&g_spi, base + 0)->tx[0], 0x40u); /* CONVERT_D1 */
    ASSERT_EQ_U(mock_spi_log_get(&g_spi, base + 3)->tx[0], 0x50u); /* CONVERT_D2 */
}

/* OSR_2048 (idx3): D1=0x40+6=0x46, D2=0x50+6=0x56. */
TEST(read_command_sequence_osr2048)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_2048);

    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, RAW_D2);
    int base = mock_spi_log_count(&g_spi);
    ms5611_read(&dev);

    ASSERT_EQ_U(mock_spi_log_get(&g_spi, base + 0)->tx[0], 0x46u);
    ASSERT_EQ_U(mock_spi_log_get(&g_spi, base + 3)->tx[0], 0x56u);
}

/* ======================================================================= */
/*  ADC 24-bit byte order                                                   */
/* ======================================================================= */

/* Verify the 3 RX bytes are assembled MSB-first into a 24-bit value, by
 * choosing distinctive D1/D2 that drive a known pressure/temperature.
 * Use D1=0x010203 etc only to confirm assembly via the resulting raw_*
 * fields in the non-blocking path (which exposes raw_pressure/raw_temp). */
TEST(adc_byte_order_msb_first)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_256);

    /* tick path exposes raw_pressure / raw_temperature directly */
    mock_set_millis(0);
    ms5611_tick(&dev);                 /* IDLE → issue D1 convert */

    push_adc24(&g_spi, 0x123456u);     /* D1 raw */
    push_adc24(&g_spi, 0xABCDEFu);     /* D2 raw */

    mock_advance_ms(dev.nb_delay_ms);  /* D1 done */
    ms5611_tick(&dev);                 /* read D1, issue D2 convert */
    mock_advance_ms(dev.nb_delay_ms);  /* D2 done */
    ms5611_tick(&dev);                 /* read D2, compute */

    ASSERT_EQ_U(dev.raw_pressure,    0x123456u);
    ASSERT_EQ_U(dev.raw_temperature, 0xABCDEFu);
}

/* ======================================================================= */
/*  NON-BLOCKING TICK state machine                                         */
/* ======================================================================= */

/* IDLE tick: issues CONVERT_D1 command, returns 0, transitions to
 * CONVERTING_D1, stamps nb_convert_start. */
TEST(tick_idle_starts_d1)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_4096);

    mock_set_millis(1000u);
    int base = mock_spi_log_count(&g_spi);
    int rc = ms5611_tick(&dev);

    ASSERT_EQ_INT(rc, 0);
    ASSERT_EQ_INT(dev.nb_state, MS5611_NB_CONVERTING_D1);
    ASSERT_EQ_U(dev.nb_convert_start, 1000u);

    const mock_SpiLog_t *cmd = mock_spi_log_get(&g_spi, base);
    ASSERT_EQ_INT(cmd->op, MOCK_SPI_TX);
    ASSERT_EQ_U(cmd->tx[0], 0x48u);    /* CONVERT_D1 @ OSR_4096 */
}

/* CONVERTING_D1: before the conversion delay elapses, tick is a no-op
 * (returns 0, stays in CONVERTING_D1, issues NO new SPI op). */
TEST(tick_d1_waits_for_delay)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_4096);  /* nb_delay_ms = 10 */

    mock_set_millis(0);
    ms5611_tick(&dev);                 /* → CONVERTING_D1 */
    int after_start = mock_spi_log_count(&g_spi);

    mock_advance_ms(dev.nb_delay_ms - 1);  /* not yet elapsed */
    int rc = ms5611_tick(&dev);

    ASSERT_EQ_INT(rc, 0);
    ASSERT_EQ_INT(dev.nb_state, MS5611_NB_CONVERTING_D1);
    ASSERT_EQ_INT(mock_spi_log_count(&g_spi), after_start); /* no new op */
}

/* CONVERTING_D1 at/after delay: reads D1 ADC, issues CONVERT_D2, advances to
 * CONVERTING_D2, re-stamps nb_convert_start = now. */
TEST(tick_d1_done_reads_and_starts_d2)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_4096);

    mock_set_millis(0);
    ms5611_tick(&dev);                 /* → CONVERTING_D1 */
    push_adc24(&g_spi, RAW_D1);

    mock_set_millis(dev.nb_delay_ms);  /* exactly the delay */
    int base = mock_spi_log_count(&g_spi);
    int rc = ms5611_tick(&dev);

    ASSERT_EQ_INT(rc, 0);
    ASSERT_EQ_INT(dev.nb_state, MS5611_NB_CONVERTING_D2);
    ASSERT_EQ_U(dev.raw_pressure, RAW_D1);
    ASSERT_EQ_U(dev.nb_convert_start, (uint32_t)dev.nb_delay_ms);

    /* New ops: READ_ADC TX(0x00) + RX(3), then CONVERT_D2 TX(0x58). */
    ASSERT_EQ_U(mock_spi_log_get(&g_spi, base + 0)->tx[0], 0x00u); /* READ_ADC */
    ASSERT_EQ_INT(mock_spi_log_get(&g_spi, base + 1)->op, MOCK_SPI_RX);
    ASSERT_EQ_INT(mock_spi_log_get(&g_spi, base + 1)->len, 3);
    ASSERT_EQ_U(mock_spi_log_get(&g_spi, base + 2)->tx[0], 0x58u); /* CONVERT_D2 */
}

/* CONVERTING_D2 at/after delay: reads D2 ADC, computes, returns 1 (new data),
 * returns to IDLE, stamps last_read = now and last_result = OK. */
TEST(tick_full_cycle_returns_data_and_idle)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_4096);

    mock_set_millis(0);
    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, RAW_D2);

    ms5611_tick(&dev);                       /* IDLE → D1 */
    mock_set_millis(dev.nb_delay_ms);
    ms5611_tick(&dev);                       /* D1 read → D2 */
    mock_set_millis(2u * dev.nb_delay_ms);
    int rc = ms5611_tick(&dev);              /* D2 read → compute */

    ASSERT_EQ_INT(rc, 1);
    ASSERT_EQ_INT(dev.nb_state, MS5611_NB_IDLE);
    ASSERT_EQ_U(dev.raw_temperature, RAW_D2);
    ASSERT_EQ_INT(dev.temperature, 2007);    /* same golden as blocking read */
    ASSERT_EQ_INT(dev.pressure,    100009);
    ASSERT_EQ_U(dev.last_read, (uint32_t)(2u * dev.nb_delay_ms));
    ASSERT_EQ_INT(dev.last_result, MS5611_READ_OK);
}

/* CONVERTING_D2 before delay is a no-op (stays D2, returns 0, no SPI op). */
TEST(tick_d2_waits_for_delay)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_4096);

    mock_set_millis(0);
    push_adc24(&g_spi, RAW_D1);
    ms5611_tick(&dev);                       /* IDLE → D1 */
    mock_set_millis(dev.nb_delay_ms);
    ms5611_tick(&dev);                       /* D1 → D2 */

    int after = mock_spi_log_count(&g_spi);
    mock_advance_ms(1);                      /* far short of delay */
    int rc = ms5611_tick(&dev);

    ASSERT_EQ_INT(rc, 0);
    ASSERT_EQ_INT(dev.nb_state, MS5611_NB_CONVERTING_D2);
    ASSERT_EQ_INT(mock_spi_log_count(&g_spi), after);
}

/* ======================================================================= */
/*  set_oversampling: nb_delay_ms golden table (ceil(us/1000))              */
/* ======================================================================= */

/* conv_delay_us[5] = {600,1200,2300,4600,9100} → ceil/1000 = {1,2,3,5,10}. */
TEST(set_oversampling_delay_table)
{
    ms5611_t dev = {0};
    do_init(&dev);

    ms5611_set_oversampling(&dev, MS5611_OSR_256);
    ASSERT_EQ_INT(dev.nb_delay_ms, 1);
    ms5611_set_oversampling(&dev, MS5611_OSR_512);
    ASSERT_EQ_INT(dev.nb_delay_ms, 2);
    ms5611_set_oversampling(&dev, MS5611_OSR_1024);
    ASSERT_EQ_INT(dev.nb_delay_ms, 3);
    ms5611_set_oversampling(&dev, MS5611_OSR_2048);
    ASSERT_EQ_INT(dev.nb_delay_ms, 5);
    ms5611_set_oversampling(&dev, MS5611_OSR_4096);
    ASSERT_EQ_INT(dev.nb_delay_ms, 10);

    /* osr field stored as given */
    ASSERT_EQ_INT(dev.osr, MS5611_OSR_4096);
}

/* ======================================================================= */
/*  ALTITUDE conversion                                                     */
/* ======================================================================= */

/* Golden altitude at the warm datasheet pressure (100009 Pa = 1000.09 hPa)
 * with sea level 1013.25 hPa → ~110.08 m. */
TEST(altitude_golden)
{
    ms5611_t dev = {0};
    do_init(&dev);
    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, RAW_D2);
    ms5611_read(&dev);

    float alt = ms5611_get_altitude(&dev, 1013.25f);
    ASSERT_NEAR(alt, 110.082489f, 1e-2f);
}

/* Zero/negative-pressure guard: ms5611_get_altitude clamps pressure_hPa to
 * 0.01 before powf so it never returns NaN.  Force pressure <= 0. */
TEST(altitude_guard_nonpositive_pressure)
{
    ms5611_t dev = {0};
    do_init(&dev);
    dev.pressure = 0;     /* pathological */

    float alt = ms5611_get_altitude(&dev, 1013.25f);
    /* clamped to 0.01 hPa → large positive altitude, NOT NaN */
    ASSERT_TRUE(alt == alt);            /* not NaN */
    ASSERT_NEAR(alt, 39364.914062f, 1.0f);

    dev.pressure = -500;
    float alt2 = ms5611_get_altitude(&dev, 1013.25f);
    ASSERT_TRUE(alt2 == alt2);          /* not NaN */
}

/* Altitude at sea-level pressure is ~0 m. */
TEST(altitude_at_sea_level)
{
    ms5611_t dev = {0};
    do_init(&dev);
    dev.pressure = 101325;              /* 1013.25 hPa in Pa */
    float alt = ms5611_get_altitude(&dev, 1013.25f);
    ASSERT_NEAR(alt, 0.0f, 1e-2f);
}

/* ======================================================================= */
/*  TEMPERATURE compensation branch (cold path)                             */
/* ======================================================================= */

/* When temperature < 2000 (20C), the second-order compensation kicks in.
 * Drive a cold D2 and verify the compensated temperature differs from the
 * raw first-order temperature by the exact T2 term.  Golden from float math:
 *   raw_first_order = -3287 (0.01C), compensated = -4430, pressure = 85696. */
TEST(compute_cold_compensation_branch)
{
    ms5611_t dev = {0};
    do_init(&dev);
    ms5611_set_oversampling(&dev, MS5611_OSR_256);

    mock_set_millis(0);
    push_adc24(&g_spi, RAW_D1);          /* same pressure raw */
    push_adc24(&g_spi, 7000000u);        /* cold temperature raw */

    ms5611_tick(&dev);
    mock_set_millis(dev.nb_delay_ms);
    ms5611_tick(&dev);
    mock_set_millis(2u * dev.nb_delay_ms);
    int rc = ms5611_tick(&dev);

    ASSERT_EQ_INT(rc, 1);
    ASSERT_EQ_INT(dev.temperature, -4430);   /* compensated */
    ASSERT_EQ_INT(dev.pressure,    85696);
}

/* With compensation disabled, the cold path is skipped: temperature stays at
 * the first-order value and pressure uses uncompensated offset/sens. */
TEST(compute_cold_no_compensation)
{
    ms5611_t dev = {0};
    do_init(&dev);
    dev.compensation = false;
    ms5611_set_oversampling(&dev, MS5611_OSR_256);

    mock_set_millis(0);
    push_adc24(&g_spi, RAW_D1);
    push_adc24(&g_spi, 7000000u);

    ms5611_tick(&dev);
    mock_set_millis(dev.nb_delay_ms);
    ms5611_tick(&dev);
    mock_set_millis(2u * dev.nb_delay_ms);
    ms5611_tick(&dev);

    ASSERT_EQ_INT(dev.temperature, -3287);   /* first-order only */
    ASSERT_EQ_INT(dev.pressure,    89823);
}

/* ======================================================================= */
/*  Timeout passed to the bus layer                                         */
/* ======================================================================= */

/* The driver passes a 100 ms timeout on its SPI transfers (golden constant).
 * Verify on the reset TX and a PROM RX. */
TEST(spi_timeout_is_100ms)
{
    ms5611_t dev = {0};
    do_init(&dev);

    ASSERT_EQ_U(mock_spi_log_get(&g_spi, 0)->to_ms, 100u);   /* reset */
    ASSERT_EQ_U(mock_spi_log_get(&g_spi, 2)->to_ms, 100u);   /* first PROM RX */
}

/* ======================================================================= */
/*  main                                                                    */
/* ======================================================================= */
int main(void)
{
    RUN(init_emits_reset_command_first);
    RUN(init_prom_read_command_sequence);
    RUN(init_prom_byte_order_and_storage);
    RUN(init_cs_idle_high);
    RUN(init_defaults);
    RUN(init_scaled_constants);
    RUN(init_device_id_fold);
    RUN(init_returns_true_on_good_prom);
    RUN(init_returns_false_on_zero_cal_word);
    RUN(init_reg0_zero_allowed);

    RUN(read_command_sequence_osr4096);
    RUN(read_compute_golden_warm);
    RUN(read_stamps_last_read_millis);
    RUN(read_command_sequence_osr256);
    RUN(read_command_sequence_osr2048);

    RUN(adc_byte_order_msb_first);

    RUN(tick_idle_starts_d1);
    RUN(tick_d1_waits_for_delay);
    RUN(tick_d1_done_reads_and_starts_d2);
    RUN(tick_full_cycle_returns_data_and_idle);
    RUN(tick_d2_waits_for_delay);

    RUN(set_oversampling_delay_table);

    RUN(altitude_golden);
    RUN(altitude_guard_nonpositive_pressure);
    RUN(altitude_at_sea_level);

    RUN(compute_cold_compensation_branch);
    RUN(compute_cold_no_compensation);

    RUN(spi_timeout_is_100ms);
    return test_summary();
}
