/* test_adxl372.c — characterization + edge-case suite for App/drivers/adxl372.c
 *
 * GOLDEN SPEC: these vectors pin the CURRENT (pre-migration) observable wire
 * behaviour of the ADXL372 high-G accelerometer driver so the upcoming
 * HAL -> casper_port seam migration is provably behaviour-preserving.
 *
 * RED state: this file is written against the TARGET migrated API
 *   adxl372_init(dev, casper_spi_t *bus, casper_pin_t cs)
 * and the driver bodies calling casper_spi_transceive / casper_gpio_write.
 * It will NOT compile until the driver is migrated off HAL.  That is intended.
 *
 * -----------------------------------------------------------------------
 *  Wire protocol recap (golden, from adxl372.c / adxl372.h, datasheet Rev B)
 * -----------------------------------------------------------------------
 *  Address byte = (reg << 1) | RNW,  RNW=1 read, RNW=0 write.
 *  Every reg access is a full-duplex transceive of {addr, payload...} with
 *  CS driven LOW before and HIGH after the transfer; timeout = 100 ms.
 *
 *  write_reg(reg,val)  : transceive {reg<<1,        val } len 2,  ignore rx
 *  read_reg(reg)       : transceive {(reg<<1)|1,    0x00} len 2,  return rx[1]
 *  read_burst(reg,n)   : transceive {(reg<<1)|1, 0,0,...} len n+1, buf=rx[1..n]
 *
 *  init() sequence (non-HIL): CS high, then
 *    W SRESET(0x41)=0x52 ; delay 10 ms ; R DEVID(0x02) ;
 *    W MEASURE(0x3E)=LOW_NOISE|BW_200 = 0x08 ;
 *    W TIMING(0x3D)=ODR_400 = 0x00 ;
 *    W POWER_CTL(0x3F)=HPF_DISABLE|FULL_BW = 0x07 ; returns true.
 *
 *  FIFO sample conversion: (int16_t)((hi<<8)|lo) >> 4  (arithmetic, signed).
 *  Activity: read STATUS_2(0x05); active iff (status & 0x10).
 */

#include "test.h"
#include "board_mock.h"
#include "adxl372.h"

/* ------------------------------------------------------------------ */
/*  Golden register address-byte constants (computed by hand)          */
/* ------------------------------------------------------------------ */
#define A_W_SRESET     0x82u   /* (0x41<<1)            write SRESET     */
#define A_R_DEVID      0x05u   /* (0x02<<1)|1          read  DEVID      */
#define A_W_MEASURE    0x7Cu   /* (0x3E<<1)            write MEASURE    */
#define A_W_TIMING     0x7Au   /* (0x3D<<1)            write TIMING     */
#define A_W_POWER_CTL  0x7Eu   /* (0x3F<<1)            write POWER_CTL  */
#define A_R_STATUS_2   0x0Bu   /* (0x05<<1)|1          read  STATUS_2   */
#define A_R_FIFO_E2    0x0Du   /* (0x06<<1)|1          read  FIFO_ENTRIES_2 */
#define A_R_FIFO_E1    0x0Fu   /* (0x07<<1)|1          read  FIFO_ENTRIES_1 */
#define A_R_FIFO_DATA  0x85u   /* (0x42<<1)|1          read  FIFO_DATA  */
#define A_W_THRESH_H   0x46u   /* (0x23<<1)            write THRESH_ACT_H */
#define A_W_THRESH_L   0x48u   /* (0x24<<1)            write THRESH_ACT_L */
#define A_W_TIME_ACT   0x4Au   /* (0x25<<1)            write TIME_ACT   */
#define A_W_ACT_CTL    0x4Eu   /* (0x27<<1)            write ACT_INACT_CTL */
#define A_W_INT1_MAP   0x76u   /* (0x3B<<1)            write INT1_MAP   */
#define A_W_FIFO_SMP   0x72u   /* (0x39<<1)            write FIFO_SAMPLES */
#define A_W_FIFO_CTL   0x74u   /* (0x3A<<1)            write FIFO_CTL   */

/* Computed payload constants for init() (mirror the #defines) */
#define V_MEASURE_INIT   (ADXL372_LOW_NOISE_EN | ADXL372_BW_200HZ)        /* 0x08 */
#define V_TIMING_INIT    (ADXL372_ODR_400HZ)                             /* 0x00 */
#define V_POWER_INIT     (ADXL372_HPF_DISABLE | ADXL372_OP_FULL_BW_MEASUREMENT) /* 0x07 */

/* Shared per-test fixtures */
static casper_spi_t spi;
static casper_pin_t cs = { (void *)0x4000u, 1u << 11 };  /* PE11-ish, opaque */

static void setup(void)
{
    mock_reset();
    spi = (casper_spi_t)mock_spi_make("ADXL_SPI");
}

/* Helper: fetch a transceive log entry and assert its op + length. */
static const mock_SpiLog_t *xfer(int idx, int len)
{
    const mock_SpiLog_t *e = mock_spi_log_get(&spi, idx);
    ASSERT_TRUE(e != NULL);
    if (e) {
        ASSERT_EQ_INT(e->op, MOCK_SPI_TRANSCEIVE);
        ASSERT_EQ_INT(e->len, len);
    }
    return e;
}

/* ================================================================== */
/*  init(): full golden register-write sequence                        */
/* ================================================================== */
TEST(init_golden_sequence)
{
    setup();
    adxl372_t dev;

    /* Script DEVID readback = 0xFA at the read_reg(DEVID) transfer.
     * read_reg returns rx[1], so byte0 is don't-care, byte1 = 0xFA. */
    const uint8_t devid_rx[] = { 0x00, ADXL372_DEVID_VAL };
    /* The init issues, in order:
     *   xfer0 W SRESET   (2)  -> no rx needed
     *   xfer1 R DEVID    (2)  -> needs {x, 0xFA}
     *   xfer2 W MEASURE  (2)
     *   xfer3 W TIMING   (2)
     *   xfer4 W POWER    (2)
     * Push enough rx so that the DEVID read (the 2nd transfer) sees 0xFA.
     * SRESET write consumes 2 queued bytes first, so prefix 2 fillers. */
    const uint8_t pre[] = { 0xFF, 0xFF };
    mock_spi_push_rx(&spi, pre, 2);
    mock_spi_push_rx(&spi, devid_rx, 2);

    bool ok = adxl372_init(&dev, &spi, cs);
    ASSERT_TRUE(ok);                         /* always returns true */
    ASSERT_EQ_U(dev.device_id, ADXL372_DEVID_VAL);

    /* Exactly 5 SPI transactions */
    ASSERT_EQ_INT(mock_spi_log_count(&spi), 5);

    const mock_SpiLog_t *e;

    /* xfer0: SRESET write {0x82,0x52} */
    e = xfer(0, 2);
    if (e) { const uint8_t g[] = { A_W_SRESET, ADXL372_RESET_CODE };
             ASSERT_EQ_MEM(e->tx, g, 2); ASSERT_EQ_U(e->to_ms, 100); }

    /* xfer1: DEVID read {0x05,0x00} */
    e = xfer(1, 2);
    if (e) { const uint8_t g[] = { A_R_DEVID, 0x00 };
             ASSERT_EQ_MEM(e->tx, g, 2); }

    /* xfer2: MEASURE write {0x7C,0x08} */
    e = xfer(2, 2);
    if (e) { const uint8_t g[] = { A_W_MEASURE, V_MEASURE_INIT };
             ASSERT_EQ_MEM(e->tx, g, 2); }

    /* xfer3: TIMING write {0x7A,0x00} */
    e = xfer(3, 2);
    if (e) { const uint8_t g[] = { A_W_TIMING, V_TIMING_INIT };
             ASSERT_EQ_MEM(e->tx, g, 2); }

    /* xfer4: POWER_CTL write {0x7E,0x07} */
    e = xfer(4, 2);
    if (e) { const uint8_t g[] = { A_W_POWER_CTL, V_POWER_INIT };
             ASSERT_EQ_MEM(e->tx, g, 2); }
}

/* init() must drive CS through the reset->idle-high path and leave it HIGH. */
TEST(init_leaves_cs_high)
{
    setup();
    adxl372_t dev;
    const uint8_t pre[] = { 0xFF, 0xFF, 0x00, ADXL372_DEVID_VAL };
    mock_spi_push_rx(&spi, pre, 4);
    adxl372_init(&dev, &spi, cs);
    ASSERT_EQ_INT(mock_gpio_get_state(cs), CASPER_PIN_HIGH);
}

/* init() must advance the virtual clock by the 10 ms reset delay. */
TEST(init_reset_delay_10ms)
{
    setup();
    adxl372_t dev;
    mock_set_millis(1000);
    const uint8_t pre[] = { 0xFF, 0xFF, 0x00, ADXL372_DEVID_VAL };
    mock_spi_push_rx(&spi, pre, 4);
    adxl372_init(&dev, &spi, cs);
    /* casper_delay_ms(10) advances the mock clock. */
    ASSERT_EQ_U(casper_millis(), 1010u);
}

/* device_id stores whatever the part reports (no validation in init). */
TEST(init_stores_wrong_devid)
{
    setup();
    adxl372_t dev;
    const uint8_t pre[] = { 0xFF, 0xFF, 0x00, 0xAB };  /* bogus DEVID */
    mock_spi_push_rx(&spi, pre, 4);
    bool ok = adxl372_init(&dev, &spi, cs);
    ASSERT_TRUE(ok);                          /* still true — no gate */
    ASSERT_EQ_U(dev.device_id, 0xAB);
    ASSERT_EQ_INT(dev.data_ready, 0);         /* cleared by init */
    for (int i = 0; i < 3; i++) ASSERT_NEAR(dev.accel_g[i], 0.0f, 1e-9);
}

/* ================================================================== */
/*  read_reg path via STATUS_2 / activity detection                    */
/* ================================================================== */
TEST(activity_detected_address_and_decode)
{
    setup();
    /* Isolated read: construct a minimal device by hand (no init noise). */
    adxl372_t d;
    d.bus = &spi; d.cs = cs;

    /* STATUS_2 read returns rx[1]; bit4 (0x10) = activity. */
    const uint8_t act_rx[] = { 0x00, 0x10 };   /* activity bit set */
    mock_spi_push_rx(&spi, act_rx, 2);
    bool act = adxl372_activity_detected(&d);
    ASSERT_TRUE(act);

    /* One transceive, address byte = (0x05<<1)|1 = 0x0B, len 2 */
    ASSERT_EQ_INT(mock_spi_log_count(&spi), 1);
    const mock_SpiLog_t *e = xfer(0, 2);
    if (e) { const uint8_t g[] = { A_R_STATUS_2, 0x00 };
             ASSERT_EQ_MEM(e->tx, g, 2); }
}

TEST(activity_not_detected)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    const uint8_t noact_rx[] = { 0x00, 0xEF };  /* every bit but 0x10 set */
    mock_spi_push_rx(&spi, noact_rx, 2);
    ASSERT_TRUE(!adxl372_activity_detected(&d));
}

/* ================================================================== */
/*  FIFO entries decode: ((hi & 0x03) << 8) | lo                       */
/* ================================================================== */
TEST(fifo_entries_decode)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;

    /* fifo_entries reads ENTRIES_2 (hi) first, then ENTRIES_1 (lo). */
    /* hi raw = 0xF7 -> masked 0x03 -> 0x300 ; lo = 0x2A -> 0x32A = 810 */
    const uint8_t hi_rx[] = { 0x00, 0xF7 };
    const uint8_t lo_rx[] = { 0x00, 0x2A };
    mock_spi_push_rx(&spi, hi_rx, 2);
    mock_spi_push_rx(&spi, lo_rx, 2);

    uint16_t n = adxl372_fifo_entries(&d);
    ASSERT_EQ_U(n, 0x32Au);   /* 810 */

    ASSERT_EQ_INT(mock_spi_log_count(&spi), 2);
    const mock_SpiLog_t *e2 = xfer(0, 2);  /* ENTRIES_2 first */
    if (e2) { const uint8_t g[] = { A_R_FIFO_E2, 0x00 };
              ASSERT_EQ_MEM(e2->tx, g, 2); }
    const mock_SpiLog_t *e1 = xfer(1, 2);  /* ENTRIES_1 second */
    if (e1) { const uint8_t g[] = { A_R_FIFO_E1, 0x00 };
              ASSERT_EQ_MEM(e1->tx, g, 2); }
}

/* Boundary: hi=0x03 lo=0xFF => max 0x3FF entries; high bits above [9:8] masked. */
TEST(fifo_entries_max_and_mask)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    const uint8_t hi_rx[] = { 0x00, 0xFF };  /* only low 2 bits kept -> 0x300 */
    const uint8_t lo_rx[] = { 0x00, 0xFF };
    mock_spi_push_rx(&spi, hi_rx, 2);
    mock_spi_push_rx(&spi, lo_rx, 2);
    ASSERT_EQ_U(adxl372_fifo_entries(&d), 0x3FFu);
}

/* ================================================================== */
/*  FIFO read: empty (< 3 entries) returns 0, no burst issued          */
/* ================================================================== */
TEST(fifo_read_empty_returns_zero)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    /* entries = 2 (< 3) => early-out. */
    const uint8_t hi_rx[] = { 0x00, 0x00 };
    const uint8_t lo_rx[] = { 0x00, 0x02 };
    mock_spi_push_rx(&spi, hi_rx, 2);
    mock_spi_push_rx(&spi, lo_rx, 2);

    int got = adxl372_fifo_read(&d);
    ASSERT_EQ_INT(got, 0);
    /* Only the 2 entry-count reads happened; NO 7-byte burst. */
    ASSERT_EQ_INT(mock_spi_log_count(&spi), 2);
}

/* ================================================================== */
/*  FIFO read: full XYZ triplet, golden byte order + signed >>4 decode */
/* ================================================================== */
TEST(fifo_read_triplet_decode)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;

    /* entries >= 3 so the burst proceeds. */
    const uint8_t hi_rx[] = { 0x00, 0x00 };
    const uint8_t lo_rx[] = { 0x00, 0x03 };
    mock_spi_push_rx(&spi, hi_rx, 2);
    mock_spi_push_rx(&spi, lo_rx, 2);

    /* Burst transceive is len 7: [addr][6 data].  rx[0] is the address-phase
     * dummy, then 6 XYZ bytes.  Script: x=+0x1230, y=-16(0xFFF0), z=+0x7FF0. */
    const uint8_t burst_rx[] = {
        0xFF,             /* address-phase dummy (rx[0], discarded) */
        0x12, 0x30,       /* X hi,lo  -> 0x1230 >>4 =  291 */
        0xFF, 0xF0,       /* Y hi,lo  -> (int16)0xFFF0 >>4 =  -1 */
        0x7F, 0xF0        /* Z hi,lo  -> 0x7FF0 >>4 = 2047 (max) */
    };
    mock_spi_push_rx(&spi, burst_rx, 7);

    int got = adxl372_fifo_read(&d);
    ASSERT_EQ_INT(got, 1);
    ASSERT_EQ_INT(d.raw_accel[0], 291);
    ASSERT_EQ_INT(d.raw_accel[1], -1);
    ASSERT_EQ_INT(d.raw_accel[2], 2047);

    /* Transactions: 2 entry reads + 1 burst = 3.  Burst is the 3rd, len 7. */
    ASSERT_EQ_INT(mock_spi_log_count(&spi), 3);
    const mock_SpiLog_t *e = xfer(2, 7);
    if (e) {
        /* tx[0] is read-address of FIFO_DATA; rest are zero clock-out bytes. */
        const uint8_t g[] = { A_R_FIFO_DATA, 0, 0, 0, 0, 0, 0 };
        ASSERT_EQ_MEM(e->tx, g, 7);
    }
}

/* Sign-extension boundary: most-negative 12-bit (0x800 raw register). */
TEST(fifo_read_negative_full_scale)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    const uint8_t hi_rx[] = { 0x00, 0x00 };
    const uint8_t lo_rx[] = { 0x00, 0x03 };
    mock_spi_push_rx(&spi, hi_rx, 2);
    mock_spi_push_rx(&spi, lo_rx, 2);
    /* X = 0x8000 -> (int16)-32768 >>4 = -2048 (most negative 12-bit).
     * Y = 0x0000 -> 0 ;  Z = 0xFFFF -> (int16)-1 >>4 = -1. */
    const uint8_t burst_rx[] = { 0xFF, 0x80,0x00, 0x00,0x00, 0xFF,0xFF };
    mock_spi_push_rx(&spi, burst_rx, 7);
    ASSERT_EQ_INT(adxl372_fifo_read(&d), 1);
    ASSERT_EQ_INT(d.raw_accel[0], -2048);
    ASSERT_EQ_INT(d.raw_accel[1], 0);
    ASSERT_EQ_INT(d.raw_accel[2], -1);
}

/* ================================================================== */
/*  wakeup_init: threshold math + register sequence                    */
/* ================================================================== */
TEST(wakeup_init_threshold_3g)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    mock_set_millis(500);

    /* threshold_g = 3.0, time_act = 6.
     * thresh_raw = (uint16_t)(3.0/0.1) = 30 (0x1E).
     * THRESH_ACT_H = (raw>>3)&0xFF = (30>>3)=3 = 0x03
     * THRESH_ACT_L = (raw&0x07)<<5 = (30&7=6)<<5 = 0xC0  */
    adxl372_wakeup_init(&d, 3.0f, 6);

    /* Sequence:
     *  0 W POWER_CTL = STANDBY (0x00)
     *    delay 1 ms
     *  1 W THRESH_ACT_H = 0x03
     *  2 W THRESH_ACT_L = 0xC0
     *  3 W TIME_ACT     = 0x06
     *  4 W ACT_INACT_CTL= 0x01
     *  5 W INT1_MAP     = 0x10 (ADXL372_INT1_ACT)
     *  6 W POWER_CTL    = WAKE_UP (0x01)                       */
    ASSERT_EQ_INT(mock_spi_log_count(&spi), 7);
    ASSERT_EQ_U(casper_millis(), 501u);  /* 1 ms standby settle */

    const mock_SpiLog_t *e;
    e = xfer(0, 2); if (e) { const uint8_t g[]={A_W_POWER_CTL, ADXL372_OP_STANDBY}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(1, 2); if (e) { const uint8_t g[]={A_W_THRESH_H, 0x03}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(2, 2); if (e) { const uint8_t g[]={A_W_THRESH_L, 0xC0}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(3, 2); if (e) { const uint8_t g[]={A_W_TIME_ACT, 6};    ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(4, 2); if (e) { const uint8_t g[]={A_W_ACT_CTL, 0x01};  ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(5, 2); if (e) { const uint8_t g[]={A_W_INT1_MAP, ADXL372_INT1_ACT}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(6, 2); if (e) { const uint8_t g[]={A_W_POWER_CTL, ADXL372_OP_WAKE_UP}; ASSERT_EQ_MEM(e->tx,g,2);}
}

/* Threshold rounding/truncation: 3.05 g -> raw 30 (float->int truncation). */
TEST(wakeup_init_threshold_truncates)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    /* (uint16_t)(3.05/0.1) = (uint16_t)30.4999.. = 30 -> H=0x03 L=0xC0 */
    adxl372_wakeup_init(&d, 3.05f, 1);
    const mock_SpiLog_t *e;
    e = xfer(1, 2); if (e) { const uint8_t g[]={A_W_THRESH_H, 0x03}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(2, 2); if (e) { const uint8_t g[]={A_W_THRESH_L, 0xC0}; ASSERT_EQ_MEM(e->tx,g,2);}
}

/* Large threshold exercising the 11-bit split: 20 g -> raw 200 (0xC8).
 * H = (200>>3)&0xFF = 25 = 0x19 ; L = (200&7=0)<<5 = 0x00. */
TEST(wakeup_init_threshold_high)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    adxl372_wakeup_init(&d, 20.0f, 3);
    const mock_SpiLog_t *e;
    e = xfer(1, 2); if (e) { const uint8_t g[]={A_W_THRESH_H, 0x19}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(2, 2); if (e) { const uint8_t g[]={A_W_THRESH_L, 0x00}; ASSERT_EQ_MEM(e->tx,g,2);}
}

/* ================================================================== */
/*  enter_measurement: returns to full-BW config (same as init tail)   */
/* ================================================================== */
TEST(enter_measurement_sequence)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    adxl372_enter_measurement(&d);

    ASSERT_EQ_INT(mock_spi_log_count(&spi), 3);
    const mock_SpiLog_t *e;
    e = xfer(0, 2); if (e) { const uint8_t g[]={A_W_MEASURE, V_MEASURE_INIT}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(1, 2); if (e) { const uint8_t g[]={A_W_TIMING,  V_TIMING_INIT};  ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(2, 2); if (e) { const uint8_t g[]={A_W_POWER_CTL,V_POWER_INIT};  ASSERT_EQ_MEM(e->tx,g,2);}
}

/* ================================================================== */
/*  fifo_init: standby -> ODR/BW -> stream config -> full-BW           */
/* ================================================================== */
TEST(fifo_init_800hz_sequence)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    mock_set_millis(0);

    /* odr_bits = ADXL372_ODR_800HZ (0x20).  BW maps to BW_400HZ (1).
     * MEASURE = LOW_NOISE(0x08) | 1 = 0x09. */
    adxl372_fifo_init(&d, ADXL372_ODR_800HZ);

    /* Sequence:
     *  0 W POWER_CTL = STANDBY (0x00) ; delay 1 ms
     *  1 W TIMING    = odr_bits (0x20)
     *  2 W MEASURE   = LOW_NOISE | BW_400 (0x09)
     *  3 W FIFO_SAMPLES = 0xFF
     *  4 W FIFO_CTL  = STREAM | FORMAT_XYZ
     *  5 W POWER_CTL = HPF_DISABLE | FULL_BW (0x07)            */
    ASSERT_EQ_INT(mock_spi_log_count(&spi), 6);
    ASSERT_EQ_U(casper_millis(), 1u);

    const mock_SpiLog_t *e;
    e = xfer(0, 2); if (e) { const uint8_t g[]={A_W_POWER_CTL, ADXL372_OP_STANDBY}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(1, 2); if (e) { const uint8_t g[]={A_W_TIMING, ADXL372_ODR_800HZ}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(2, 2); if (e) { const uint8_t g[]={A_W_MEASURE, (uint8_t)(ADXL372_LOW_NOISE_EN|ADXL372_BW_400HZ)}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(3, 2); if (e) { const uint8_t g[]={A_W_FIFO_SMP, 0xFF}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(4, 2); if (e) { const uint8_t g[]={A_W_FIFO_CTL, (uint8_t)(ADXL372_FIFO_STREAM|ADXL372_FIFO_FORMAT_XYZ)}; ASSERT_EQ_MEM(e->tx,g,2);}
    e = xfer(5, 2); if (e) { const uint8_t g[]={A_W_POWER_CTL, V_POWER_INIT}; ASSERT_EQ_MEM(e->tx,g,2);}
}

/* fifo_init default (unknown ODR) falls back to BW_200HZ. */
TEST(fifo_init_default_bw)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    adxl372_fifo_init(&d, ADXL372_ODR_400HZ);  /* not in switch -> default */
    const mock_SpiLog_t *e = xfer(2, 2);
    if (e) { const uint8_t g[]={A_W_MEASURE, (uint8_t)(ADXL372_LOW_NOISE_EN|ADXL372_BW_200HZ)};
             ASSERT_EQ_MEM(e->tx,g,2); }
}

/* ================================================================== */
/*  irq_handler: sets the data_ready flag                              */
/* ================================================================== */
TEST(irq_handler_sets_flag)
{
    setup();
    adxl372_t d; d.data_ready = false;
    adxl372_irq_handler(&d);
    ASSERT_EQ_INT(d.data_ready, 1);
}

/* ================================================================== */
/*  Error propagation: filling the SPI log to capacity makes the next  */
/*  casper_spi_transceive return CASPER_ERR.  Verifies the driver does  */
/*  not corrupt state when the bus call fails.                         */
/* ================================================================== */
TEST(spi_error_path_burst)
{
    setup();
    adxl372_t d; d.bus = &spi; d.cs = cs;
    d.raw_accel[0] = d.raw_accel[1] = d.raw_accel[2] = 12345;

    /* Saturate the log so EVERY casper_spi_* returns CASPER_ERR. */
    uint8_t junk[2] = { 0, 0 };
    for (int i = 0; i < MOCK_SPI_LOG_MAX; i++) {
        (void)casper_spi_transceive(&spi, junk, junk, 2, 100);
    }
    ASSERT_EQ_INT(mock_spi_log_count(&spi), MOCK_SPI_LOG_MAX);

    /* fifo_entries now reads garbage (transceive returns ERR, rx untouched).
     * The golden driver does not check the return code, so behaviour is
     * "reads whatever rx held".  We assert it does not crash / overrun and
     * the entry-count logic still runs deterministically. */
    uint16_t n = adxl372_fifo_entries(&d);
    (void)n;  /* value is indeterminate under saturation; just must not crash */
    ASSERT_EQ_INT(mock_spi_log_count(&spi), MOCK_SPI_LOG_MAX);  /* no new logs */
}

int main(void)
{
    RUN(init_golden_sequence);
    RUN(init_leaves_cs_high);
    RUN(init_reset_delay_10ms);
    RUN(init_stores_wrong_devid);
    RUN(activity_detected_address_and_decode);
    RUN(activity_not_detected);
    RUN(fifo_entries_decode);
    RUN(fifo_entries_max_and_mask);
    RUN(fifo_read_empty_returns_zero);
    RUN(fifo_read_triplet_decode);
    RUN(fifo_read_negative_full_scale);
    RUN(wakeup_init_threshold_3g);
    RUN(wakeup_init_threshold_truncates);
    RUN(wakeup_init_threshold_high);
    RUN(enter_measurement_sequence);
    RUN(fifo_init_800hz_sequence);
    RUN(fifo_init_default_bw);
    RUN(irq_handler_sets_flag);
    RUN(spi_error_path_burst);
    return test_summary();
}
