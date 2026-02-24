/**
 * @file radio_config.h
 * @brief Radio profile definitions and configuration constants.
 *
 * Profile A (SF7) for standard flights, Profile B (SF8) for extended range.
 * Both use 868 MHz, 250 kHz BW, CR 4/5, +20 dBm TX power.
 */

#ifndef APP_RADIO_RADIO_CONFIG_H
#define APP_RADIO_RADIO_CONFIG_H

#include <stdint.h>

/* ── Radio profile structure ────────────────────────────────────── */

typedef struct {
    uint8_t  sf;            /* Spreading factor: 7 or 8                */
    uint32_t bw_hz;         /* Bandwidth in Hz: 250000                 */
    uint8_t  cr;            /* Coding rate denominator: 5 (for 4/5)    */
    uint8_t  sync_word;     /* LoRa sync word: 0x12 (private network)  */
    uint16_t preamble;      /* Preamble length in symbols: 8           */
    int8_t   tx_power_dbm;  /* TX power: +20 dBm (PA_BOOST + PA_DAC)   */
    uint32_t freq_hz;       /* Carrier frequency: 868000000            */
} radio_profile_t;

/* ── Profile instances (defined in radio_manager.c) ─────────────── */

extern const radio_profile_t RADIO_PROFILE_A;  /* SF7, 250k, CR4/5 */
extern const radio_profile_t RADIO_PROFILE_B;  /* SF8, 250k, CR4/5 */

/* ── TX timing ──────────────────────────────────────────────────── */

#define RADIO_TX_PERIOD_MS          100     /* 10 Hz TX cadence             */
#define RADIO_TX_TIMEOUT_MS         200     /* Max time to wait for TxDone  */
#define RADIO_RX_WINDOW_SYMBOLS     80      /* RX timeout in symbols (~80ms)*/

/* ── Profile switch thresholds (defaults, overridden by config) ── */

#define RADIO_PROFILE_SWITCH_ALT_M  20000.0f  /* Switch A->B above 20 km  */
#define RADIO_PROFILE_SWITCH_VEL_MPS 500.0f   /* Switch A->B above 500 m/s*/

/* ── DIO mapping values ─────────────────────────────────────────── */

/* RegDioMapping1 (0x40):
 * DIO0 [7:6] = 00 (RxDone in RX / TxDone in TX)
 * DIO1 [5:4] = 00 (RxTimeout)
 * DIO2 [3:2] = 00 (FhssChangeChannel - unused)
 * DIO3 [1:0] = 01 (ValidHeader)
 */
#define RADIO_DIO_MAPPING_1         0x01

/* RegDioMapping2 (0x41):
 * DIO4 [7:6] = 10 (PllLock)
 * DIO5 [5:4] = 10 (ModeReady)
 * reserved [3:0]
 */
#define RADIO_DIO_MAPPING_2         0xA0

/* ── Error counter thresholds ───────────────────────────────────── */

#define RADIO_MAX_CONSEC_CRC_ERRORS 10
#define RADIO_MAX_TX_RETRIES        3

/* ── Response queue ─────────────────────────────────────────────── */

#define RADIO_RESP_QUEUE_SIZE       4
#define RADIO_MAX_PACKET_SIZE       32

#endif /* APP_RADIO_RADIO_CONFIG_H */
