/**
 * @file sx1276.c
 * @brief SX1276 LoRa radio low-level SPI driver.
 *
 * Raw register-level SPI via SPI1 (bypasses HAL polling which hangs on H7).
 * CS (PB0) managed in software.
 * All functions run in main-loop context (never from ISR).
 */

#include "sx1276.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

/* ── Module state ──────────────────────────────────────────────── */

static SPI_TypeDef *s_spi;

/* ── CS helpers ────────────────────────────────────────────────── */

static void cs_select(void)
{
    HAL_GPIO_WritePin(Radio_CS_GPIO_Port, Radio_CS_Pin, GPIO_PIN_RESET);
}

static void cs_deselect(void)
{
    HAL_GPIO_WritePin(Radio_CS_GPIO_Port, Radio_CS_Pin, GPIO_PIN_SET);
}

/* ── Raw SPI byte transfer (bypass HAL) ────────────────────────── */

/**
 * Exchange one byte over SPI using direct register access.
 * Returns 0xFF on timeout (~5 ms at 432 MHz).
 */
static uint8_t spi_xfer_byte(uint8_t tx_byte)
{
    volatile uint32_t timeout;

    /* Wait for TXP (TX FIFO has space) */
    timeout = 200000;
    while (!(s_spi->SR & SPI_SR_TXP)) {
        if (--timeout == 0) return 0xFF;
    }
    *(volatile uint8_t *)&s_spi->TXDR = tx_byte;

    /* Wait for RXP (RX FIFO has data) */
    timeout = 200000;
    while (!(s_spi->SR & SPI_SR_RXP)) {
        if (--timeout == 0) return 0xFF;
    }
    return *(volatile uint8_t *)&s_spi->RXDR;
}

/**
 * Start an SPI transaction: set TSIZE, enable SPI, assert CSTART.
 */
static void spi_start(uint16_t size)
{
    /* Clear any pending flags */
    SET_BIT(s_spi->IFCR, SPI_IFCR_EOTC | SPI_IFCR_TXTFC);

    /* Must disable SPI to write TSIZE */
    CLEAR_BIT(s_spi->CR1, SPI_CR1_SPE);
    MODIFY_REG(s_spi->CR2, SPI_CR2_TSIZE, size);

    /* Force SSI high — master with software NSS needs this to avoid MODF.
     * HAL_SPI_Init sets it, but CLEAR_BIT(CR1, SPE) can lose it on H7. */
    SET_BIT(s_spi->CR1, SPI_CR1_SSI);

    SET_BIT(s_spi->CR1, SPI_CR1_SPE);
    SET_BIT(s_spi->CR1, SPI_CR1_CSTART);
}

/**
 * End an SPI transaction: wait for EOT with timeout, then disable SPI.
 */
static void spi_end(void)
{
    volatile uint32_t timeout = 200000;
    while (!(s_spi->SR & SPI_SR_EOT)) {
        if (--timeout == 0) break;
    }
    SET_BIT(s_spi->IFCR, SPI_IFCR_EOTC | SPI_IFCR_TXTFC);
    CLEAR_BIT(s_spi->CR1, SPI_CR1_SPE);
}

/* ── Register access ───────────────────────────────────────────── */

uint8_t sx1276_read_reg(uint8_t addr)
{
    uint8_t val;

    cs_select();
    spi_start(2);
    (void)spi_xfer_byte(addr & 0x7F);   /* send address, discard dummy */
    val = spi_xfer_byte(0x00);           /* send dummy, receive data */
    spi_end();
    cs_deselect();

    return val;
}

void sx1276_write_reg(uint8_t addr, uint8_t val)
{
    cs_select();
    spi_start(2);
    (void)spi_xfer_byte(addr | 0x80);
    (void)spi_xfer_byte(val);
    spi_end();
    cs_deselect();
}

void sx1276_read_fifo(uint8_t *buf, uint8_t len)
{
    cs_select();
    spi_start((uint16_t)(1 + len));
    (void)spi_xfer_byte(SX1276_REG_FIFO & 0x7F);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_xfer_byte(0x00);
    }
    spi_end();
    cs_deselect();
}

void sx1276_write_fifo(const uint8_t *buf, uint8_t len)
{
    cs_select();
    spi_start((uint16_t)(1 + len));
    (void)spi_xfer_byte(SX1276_REG_FIFO | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        (void)spi_xfer_byte(buf[i]);
    }
    spi_end();
    cs_deselect();
}

/* ── Reset ─────────────────────────────────────────────────────── */

void sx1276_reset(void)
{
    HAL_GPIO_WritePin(RADIO_NRST_GPIO_Port, RADIO_NRST_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(RADIO_NRST_GPIO_Port, RADIO_NRST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);  /* PC13 backup domain (3mA) — extra margin for slow rise */
}

/* ── Init ──────────────────────────────────────────────────────── */

static void sx1276_dbg(const char *msg)
{
    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    HAL_Delay(20);
}

int sx1276_init(SPI_HandleTypeDef *hspi)
{
    char dbg[80];
    s_spi = hspi->Instance;

    cs_deselect();

    sx1276_dbg("[SX1276] resetting...\r\n");
    sx1276_reset();
    sx1276_dbg("[SX1276] reset done, reading version\r\n");

    /* Read version register — expect 0x12 for SX1276 */
    uint8_t ver = sx1276_read_reg(SX1276_REG_VERSION);

    snprintf(dbg, sizeof(dbg), "[SX1276] version=0x%02X (expect 0x%02X)\r\n",
             ver, SX1276_VERSION_EXPECTED);
    sx1276_dbg(dbg);

    if (ver != SX1276_VERSION_EXPECTED) {
        sx1276_dbg("[SX1276] FAIL: bad version\r\n");
        return -1;
    }

    /* Set LoRa mode */
    sx1276_set_lora_mode();
    sx1276_dbg("[SX1276] LoRa mode set\r\n");

    /* Set FIFO base addresses */
    sx1276_write_reg(SX1276_REG_FIFO_TX_BASE_ADDR, SX1276_FIFO_TX_BASE);
    sx1276_write_reg(SX1276_REG_FIFO_RX_BASE_ADDR, SX1276_FIFO_RX_BASE);

    sx1276_dbg("[SX1276] init OK\r\n");
    return 0;
}

/* ── Mode control ──────────────────────────────────────────────── */

void sx1276_set_mode(uint8_t mode)
{
    uint8_t reg = sx1276_read_reg(SX1276_REG_OP_MODE);
    reg = (reg & ~SX1276_OPMODE_MASK) | (mode & SX1276_OPMODE_MASK);
    sx1276_write_reg(SX1276_REG_OP_MODE, reg);
}

void sx1276_set_lora_mode(void)
{
    /* Must be in sleep mode before switching to LoRa */
    sx1276_set_mode(SX1276_MODE_SLEEP);
    sx1276_write_reg(SX1276_REG_OP_MODE,
                     SX1276_OPMODE_LORA | SX1276_MODE_SLEEP);
}

/* ── Configuration ─────────────────────────────────────────────── */

void sx1276_set_frequency(uint32_t freq_hz)
{
    uint32_t frf = (uint32_t)((uint64_t)freq_hz * (1UL << 19) / SX1276_FXOSC);

    sx1276_write_reg(SX1276_REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1276_write_reg(SX1276_REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1276_write_reg(SX1276_REG_FRF_LSB, (uint8_t)(frf));
}

void sx1276_set_tx_power(int8_t dbm)
{
    if (dbm == 20) {
        /* +20 dBm mode: PA_BOOST max + high-power PA DAC */
        sx1276_write_reg(SX1276_REG_PA_CONFIG, 0xFF);
        sx1276_write_reg(SX1276_REG_PA_DAC, 0x87);
        sx1276_write_reg(SX1276_REG_OCP, 0x3B);  /* 240 mA */
    } else {
        /* PA_BOOST normal: OutputPower = dbm - 2, clamped 0..15 */
        int8_t op = dbm - 2;
        if (op < 0)  op = 0;
        if (op > 15) op = 15;

        /* PA_BOOST on (bit 7), MaxPower=7 (bits 6:4), OutputPower (bits 3:0) */
        sx1276_write_reg(SX1276_REG_PA_CONFIG,
                         0x80 | (7 << 4) | (uint8_t)op);
        sx1276_write_reg(SX1276_REG_PA_DAC, 0x84);  /* default DAC */
    }
}

void sx1276_set_modulation(uint8_t sf, uint32_t bw_hz, uint8_t cr)
{
    /* Map bandwidth Hz to register value */
    uint8_t bw_val;
    switch (bw_hz) {
    case 7800:    bw_val = SX1276_BW_7K8;   break;
    case 10400:   bw_val = SX1276_BW_10K4;  break;
    case 15600:   bw_val = SX1276_BW_15K6;  break;
    case 20800:   bw_val = SX1276_BW_20K8;  break;
    case 31250:   bw_val = SX1276_BW_31K25; break;
    case 41700:   bw_val = SX1276_BW_41K7;  break;
    case 62500:   bw_val = SX1276_BW_62K5;  break;
    case 125000:  bw_val = SX1276_BW_125K;  break;
    case 250000:  bw_val = SX1276_BW_250K;  break;
    case 500000:  bw_val = SX1276_BW_500K;  break;
    default:      bw_val = SX1276_BW_125K;  break;
    }

    /* Map coding rate denominator to register value */
    uint8_t cr_val;
    switch (cr) {
    case 5:  cr_val = SX1276_CR_4_5; break;
    case 6:  cr_val = SX1276_CR_4_6; break;
    case 7:  cr_val = SX1276_CR_4_7; break;
    case 8:  cr_val = SX1276_CR_4_8; break;
    default: cr_val = SX1276_CR_4_5; break;
    }

    /* RegModemConfig1: BW | CR | explicit header */
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG_1,
                     bw_val | cr_val | SX1276_EXPLICIT_HEADER);

    /* RegModemConfig2: SF | CRC on */
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG_2,
                     (sf << 4) | SX1276_RX_CRC_ON);

    /* RegModemConfig3: AGC auto on, LowDataRateOptimize if needed */
    uint8_t cfg3 = 0x04;  /* AGC auto on */
    if (sf >= 11 && bw_hz <= 125000)
        cfg3 |= 0x08;  /* LowDataRateOptimize on */
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG_3, cfg3);

    /* Detection optimize and threshold for SF7-12 */
    sx1276_write_reg(SX1276_REG_DETECT_OPTIMIZE, 0xC3);
    sx1276_write_reg(SX1276_REG_DETECTION_THRESHOLD, 0x0A);
}

void sx1276_set_sync_word(uint8_t sw)
{
    sx1276_write_reg(SX1276_REG_SYNC_WORD, sw);
}

void sx1276_set_preamble(uint16_t symbols)
{
    sx1276_write_reg(SX1276_REG_PREAMBLE_MSB, (uint8_t)(symbols >> 8));
    sx1276_write_reg(SX1276_REG_PREAMBLE_LSB, (uint8_t)(symbols));
}

void sx1276_set_payload_length(uint8_t len)
{
    sx1276_write_reg(SX1276_REG_PAYLOAD_LENGTH, len);
}

/* ── IRQ ───────────────────────────────────────────────────────── */

uint8_t sx1276_get_irq_flags(void)
{
    return sx1276_read_reg(SX1276_REG_IRQ_FLAGS);
}

void sx1276_clear_irq_flags(uint8_t mask)
{
    sx1276_write_reg(SX1276_REG_IRQ_FLAGS, mask);
}

/* ── RSSI / SNR ────────────────────────────────────────────────── */

int16_t sx1276_get_packet_rssi(void)
{
    uint8_t val = sx1276_read_reg(SX1276_REG_PKT_RSSI_VALUE);
    return (int16_t)val - 157;  /* HF band (>862 MHz) */
}

int8_t sx1276_get_packet_snr(void)
{
    uint8_t val = sx1276_read_reg(SX1276_REG_PKT_SNR_VALUE);
    return (int8_t)val / 4;
}
