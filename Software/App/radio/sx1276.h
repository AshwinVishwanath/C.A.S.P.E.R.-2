/**
 * @file sx1276.h
 * @brief SX1276 LoRa radio low-level SPI driver.
 *
 * Register map, enums, and API for the Ai-Thinker RA-01H module.
 * All SPI transactions are polling-based (no DMA). CS is software-managed.
 */

#ifndef APP_RADIO_SX1276_H
#define APP_RADIO_SX1276_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ── SX1276 Register Map (LoRa mode) ──────────────────────────── */

#define SX1276_REG_FIFO                 0x00
#define SX1276_REG_OP_MODE              0x01
#define SX1276_REG_FRF_MSB              0x06
#define SX1276_REG_FRF_MID              0x07
#define SX1276_REG_FRF_LSB              0x08
#define SX1276_REG_PA_CONFIG            0x09
#define SX1276_REG_PA_RAMP              0x0A
#define SX1276_REG_OCP                  0x0B
#define SX1276_REG_LNA                  0x0C
#define SX1276_REG_FIFO_ADDR_PTR       0x0D
#define SX1276_REG_FIFO_TX_BASE_ADDR   0x0E
#define SX1276_REG_FIFO_RX_BASE_ADDR   0x0F
#define SX1276_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX1276_REG_IRQ_FLAGS_MASK      0x11
#define SX1276_REG_IRQ_FLAGS           0x12
#define SX1276_REG_RX_NB_BYTES         0x13
#define SX1276_REG_RX_HEADER_CNT_MSB   0x14
#define SX1276_REG_RX_HEADER_CNT_LSB   0x15
#define SX1276_REG_RX_PACKET_CNT_MSB   0x16
#define SX1276_REG_RX_PACKET_CNT_LSB   0x17
#define SX1276_REG_MODEM_STAT          0x18
#define SX1276_REG_PKT_SNR_VALUE       0x19
#define SX1276_REG_PKT_RSSI_VALUE      0x1A
#define SX1276_REG_RSSI_VALUE          0x1B
#define SX1276_REG_HOP_CHANNEL         0x1C
#define SX1276_REG_MODEM_CONFIG_1      0x1D
#define SX1276_REG_MODEM_CONFIG_2      0x1E
#define SX1276_REG_SYMB_TIMEOUT_LSB    0x1F
#define SX1276_REG_PREAMBLE_MSB        0x20
#define SX1276_REG_PREAMBLE_LSB        0x21
#define SX1276_REG_PAYLOAD_LENGTH      0x22
#define SX1276_REG_MAX_PAYLOAD_LENGTH  0x23
#define SX1276_REG_HOP_PERIOD          0x24
#define SX1276_REG_FIFO_RX_BYTE_ADDR   0x25
#define SX1276_REG_MODEM_CONFIG_3      0x26
#define SX1276_REG_DETECT_OPTIMIZE     0x31
#define SX1276_REG_INVERT_IQ           0x33
#define SX1276_REG_DETECTION_THRESHOLD 0x37
#define SX1276_REG_SYNC_WORD           0x39
#define SX1276_REG_DIO_MAPPING_1       0x40
#define SX1276_REG_DIO_MAPPING_2       0x41
#define SX1276_REG_VERSION             0x42
#define SX1276_REG_PA_DAC              0x4D

/* ── Expected silicon revision ──────────────────────────────────── */
#define SX1276_VERSION_EXPECTED         0x12

/* ── RegOpMode bit fields ───────────────────────────────────────── */
#define SX1276_OPMODE_LORA              0x80  /* bit 7: LoRa mode */
#define SX1276_OPMODE_ACCESS_SHARED     0x40  /* bit 6: access shared regs */
#define SX1276_OPMODE_MASK              0x07  /* bits 2:0: mode */

/* Operating modes (bits 2:0 of RegOpMode) */
#define SX1276_MODE_SLEEP               0x00
#define SX1276_MODE_STDBY               0x01
#define SX1276_MODE_FSTX                0x02
#define SX1276_MODE_TX                  0x03
#define SX1276_MODE_FSRX                0x04
#define SX1276_MODE_RXCONTINUOUS        0x05
#define SX1276_MODE_RXSINGLE            0x06
#define SX1276_MODE_CAD                 0x07

/* ── IRQ flags (RegIrqFlags 0x12) ──────────────────────────────── */
#define SX1276_IRQ_RX_TIMEOUT           0x80
#define SX1276_IRQ_RX_DONE              0x40
#define SX1276_IRQ_PAYLOAD_CRC_ERROR    0x20
#define SX1276_IRQ_VALID_HEADER         0x10
#define SX1276_IRQ_TX_DONE              0x08
#define SX1276_IRQ_CAD_DONE             0x04
#define SX1276_IRQ_FHSS_CHANGE_CHANNEL 0x02
#define SX1276_IRQ_CAD_DETECTED         0x01
#define SX1276_IRQ_ALL                  0xFF

/* ── RegModemConfig1 bit fields ─────────────────────────────────── */
/* BW [7:4] */
#define SX1276_BW_7K8                   0x00
#define SX1276_BW_10K4                  0x10
#define SX1276_BW_15K6                  0x20
#define SX1276_BW_20K8                  0x30
#define SX1276_BW_31K25                 0x40
#define SX1276_BW_41K7                  0x50
#define SX1276_BW_62K5                  0x60
#define SX1276_BW_125K                  0x70
#define SX1276_BW_250K                  0x80
#define SX1276_BW_500K                  0x90
/* CR [3:1] */
#define SX1276_CR_4_5                   0x02
#define SX1276_CR_4_6                   0x04
#define SX1276_CR_4_7                   0x06
#define SX1276_CR_4_8                   0x08
/* Implicit header [0] */
#define SX1276_IMPLICIT_HEADER          0x01
#define SX1276_EXPLICIT_HEADER          0x00

/* ── RegModemConfig2 bit fields ─────────────────────────────────── */
/* SF [7:4] */
#define SX1276_SF6                      0x60
#define SX1276_SF7                      0x70
#define SX1276_SF8                      0x80
#define SX1276_SF9                      0x90
#define SX1276_SF10                     0xA0
#define SX1276_SF11                     0xB0
#define SX1276_SF12                     0xC0
/* TxContinuousMode [3] */
#define SX1276_TX_CONTINUOUS            0x08
/* RxPayloadCrcOn [2] */
#define SX1276_RX_CRC_ON               0x04

/* ── FIFO size ──────────────────────────────────────────────────── */
#define SX1276_FIFO_SIZE                256
#define SX1276_FIFO_TX_BASE             0x80
#define SX1276_FIFO_RX_BASE             0x00

/* ── F_XOSC for frequency calculation ──────────────────────────── */
#define SX1276_FXOSC                    32000000UL

/* ── Driver API ─────────────────────────────────────────────────── */

/**
 * Initialise SX1276: hardware reset, verify silicon ID (0x12),
 * enter LoRa + sleep mode.
 *
 * @param hspi  SPI1 handle (must be initialised before calling)
 * @return      0 on success, -1 if silicon ID mismatch (SPI bus broken)
 */
int sx1276_init(SPI_HandleTypeDef *hspi);

/**
 * Hardware reset: pulse RADIO_NRST low for 1 ms, wait 5 ms.
 */
void sx1276_reset(void);

/* ── Register access ────────────────────────────────────────────── */

uint8_t sx1276_read_reg(uint8_t addr);
void    sx1276_write_reg(uint8_t addr, uint8_t val);
void    sx1276_read_fifo(uint8_t *buf, uint8_t len);
void    sx1276_write_fifo(const uint8_t *buf, uint8_t len);

/* ── Mode control ───────────────────────────────────────────────── */

void sx1276_set_mode(uint8_t mode);
void sx1276_set_lora_mode(void);

/* ── Configuration ──────────────────────────────────────────────── */

void sx1276_set_frequency(uint32_t freq_hz);
void sx1276_set_tx_power(int8_t dbm);
void sx1276_set_modulation(uint8_t sf, uint32_t bw_hz, uint8_t cr);
void sx1276_set_sync_word(uint8_t sw);
void sx1276_set_preamble(uint16_t symbols);
void sx1276_set_payload_length(uint8_t len);

/* ── IRQ ────────────────────────────────────────────────────────── */

uint8_t sx1276_get_irq_flags(void);
void    sx1276_clear_irq_flags(uint8_t mask);

/* ── RSSI / SNR (read after RX) ─────────────────────────────────── */

int16_t sx1276_get_packet_rssi(void);
int8_t  sx1276_get_packet_snr(void);

#endif /* APP_RADIO_SX1276_H */
