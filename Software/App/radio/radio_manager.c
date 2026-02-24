/**
 * @file radio_manager.c
 * @brief Radio state machine: TX scheduler, RX window, profile switching.
 *
 * Non-blocking radio manager for SX1276 LoRa. Runs from the superloop at
 * ~833 Hz. Schedules 10 Hz TX with priority queuing, opens an RX window
 * after each TX for uplink commands, and switches radio profile based on
 * EKF altitude/velocity.
 */

#include "radio_manager.h"
#include "sx1276.h"
#include "radio_config.h"
#include "radio_irq.h"
#include "crc32_hw.h"
#include "cac_handler.h"
#include "quat_pack.h"
#include "status_pack.h"
#include "tlm_types.h"
#include <string.h>
#include <math.h>

/* ── Profile instances ─────────────────────────────────────────────── */

const radio_profile_t RADIO_PROFILE_A = {
    .sf = 7, .bw_hz = 250000, .cr = 5,
    .sync_word = 0x12, .preamble = 8,
    .tx_power_dbm = 20, .freq_hz = 868000000
};

const radio_profile_t RADIO_PROFILE_B = {
    .sf = 8, .bw_hz = 250000, .cr = 5,
    .sync_word = 0x12, .preamble = 8,
    .tx_power_dbm = 20, .freq_hz = 868000000
};

/* ── State machine ─────────────────────────────────────────────────── */

typedef enum {
    RADIO_STATE_IDLE,
    RADIO_STATE_TX,
    RADIO_STATE_RX,
    RADIO_STATE_DISABLED
} radio_state_t;

/* ── Response queue entry ──────────────────────────────────────────── */

typedef struct {
    uint8_t data[RADIO_MAX_PACKET_SIZE];
    uint8_t len;
} resp_entry_t;

/* ── Event queue entry ─────────────────────────────────────────────── */

typedef struct {
    uint8_t  type;
    uint16_t data;
} event_entry_t;

/* ── Module-level state ────────────────────────────────────────────── */

static radio_state_t s_radio_state = RADIO_STATE_DISABLED;
static const radio_profile_t *s_current_profile;
static uint8_t  s_seq;
static uint32_t s_last_tx_ms;
static uint32_t s_tx_start_ms;
static uint32_t s_rx_start_ms;
static uint32_t s_consec_crc_errors;
static uint32_t s_tx_error_count;

/* TX packet buffer */
static uint8_t s_tx_buf[RADIO_MAX_PACKET_SIZE];

/* Response ring buffer */
static resp_entry_t s_resp_queue[RADIO_RESP_QUEUE_SIZE];
static uint8_t s_resp_head;
static uint8_t s_resp_tail;
static uint8_t s_resp_count;

/* Event ring buffer (4 entries) */
#define EVENT_QUEUE_SIZE 4
static event_entry_t s_event_queue[EVENT_QUEUE_SIZE];
static uint8_t s_evt_head;
static uint8_t s_evt_tail;
static uint8_t s_evt_count;

/* GPS queue (single slot, latest overwrites) */
static uint8_t s_gps_buf[SIZE_FC_MSG_GPS];
static uint8_t s_gps_pending;

/* RX buffer */
static uint8_t s_rx_buf[RADIO_MAX_PACKET_SIZE];

/* ── Helpers (same as tlm_manager.c) ───────────────────────────────── */

static void put_le16(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void put_le32(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
    dst[2] = (uint8_t)((val >> 16) & 0xFF);
    dst[3] = (uint8_t)((val >> 24) & 0xFF);
}

/* ── Apply radio profile ───────────────────────────────────────────── */

static void radio_apply_profile(const radio_profile_t *prof)
{
    sx1276_set_mode(SX1276_MODE_STDBY);
    sx1276_set_frequency(prof->freq_hz);
    sx1276_set_modulation(prof->sf, prof->bw_hz, prof->cr);
    sx1276_set_sync_word(prof->sync_word);
    sx1276_set_preamble(prof->preamble);
    sx1276_set_tx_power(prof->tx_power_dbm);
}

/* ── Profile switching ─────────────────────────────────────────────── */

static void check_profile_switch(const casper_ekf_t *ekf)
{
    if (s_current_profile == &RADIO_PROFILE_B) return;
    if (ekf->x[0] > RADIO_PROFILE_SWITCH_ALT_M ||
        ekf->x[1] > RADIO_PROFILE_SWITCH_VEL_MPS) {
        radio_apply_profile(&RADIO_PROFILE_B);
        s_current_profile = &RADIO_PROFILE_B;
        radio_queue_event(FC_EVT_STATE, 0x00BB);
    }
}

/* ── Packet builders ───────────────────────────────────────────────── */

static int build_fast_packet(uint8_t *buf,
                             const fc_telem_state_t *tstate,
                             const pyro_state_t *pstate,
                             fsm_state_t fsm)
{
    uint8_t *p = buf;

    /* [0] msg_id */
    *p++ = MSG_ID_FAST;

    /* [1-2] status bitmap */
    status_pack_build(p, pstate, fsm, false);
    p += 2;

    /* [3-4] altitude in decametres, u16 LE */
    float alt_dam = tstate->alt_m / ALT_SCALE_M;
    if (alt_dam < 0.0f) alt_dam = 0.0f;
    if (alt_dam > 65535.0f) alt_dam = 65535.0f;
    put_le16(p, (uint16_t)alt_dam);
    p += 2;

    /* [5-6] velocity in dm/s, i16 LE */
    float vel_dms = tstate->vel_mps / VEL_SCALE_DMS;
    if (vel_dms > 32767.0f) vel_dms = 32767.0f;
    if (vel_dms < -32768.0f) vel_dms = -32768.0f;
    put_le16(p, (uint16_t)(int16_t)vel_dms);
    p += 2;

    /* [7-11] quaternion packed (5 bytes) */
    quat_pack_smallest_three(p, tstate->quat);
    p += 5;

    /* [12-13] flight time in 0.1s ticks, u16 LE */
    float time_ds = tstate->flight_time_s / TIME_SCALE_100MS;
    if (time_ds < 0.0f) time_ds = 0.0f;
    if (time_ds > 65535.0f) time_ds = 65535.0f;
    put_le16(p, (uint16_t)time_ds);
    p += 2;

    /* [14] battery voltage, u8 */
    float batt_encoded = (tstate->batt_v - BATT_OFFSET_V) / BATT_STEP_V;
    if (batt_encoded < 0.0f) batt_encoded = 0.0f;
    if (batt_encoded > 255.0f) batt_encoded = 255.0f;
    *p++ = (uint8_t)batt_encoded;

    /* [15] sequence counter */
    *p++ = s_seq++;

    /* [16-19] CRC-32 over [0-15] */
    uint32_t crc = crc32_hw_compute(buf, 16);
    put_le32(p, crc);

    return SIZE_FC_MSG_FAST;
}

static int build_gps_packet(uint8_t *buf, const fc_gps_state_t *gps)
{
    uint8_t *p = buf;

    /* [0] msg_id */
    *p++ = MSG_ID_GPS;

    /* [1-4] dlat_mm, i32 LE */
    put_le32(p, (uint32_t)gps->dlat_mm);
    p += 4;

    /* [5-8] dlon_mm, i32 LE */
    put_le32(p, (uint32_t)gps->dlon_mm);
    p += 4;

    /* [9-10] alt_msl in decametres, u16 LE */
    float alt_dam = gps->alt_msl_m / ALT_SCALE_M;
    if (alt_dam < 0.0f) alt_dam = 0.0f;
    if (alt_dam > 65535.0f) alt_dam = 65535.0f;
    put_le16(p, (uint16_t)alt_dam);
    p += 2;

    /* [11] fix_type */
    *p++ = gps->fix_type;

    /* [12] sat_count */
    *p++ = gps->sat_count;

    /* [13-16] CRC-32 over [0-12] */
    uint32_t crc = crc32_hw_compute(buf, 13);
    put_le32(p, crc);

    return SIZE_FC_MSG_GPS;
}

static int build_event_packet(uint8_t *buf, uint8_t type, uint16_t data)
{
    uint8_t *p = buf;

    /* [0] msg_id */
    *p++ = MSG_ID_EVENT;

    /* [1] event type */
    *p++ = type;

    /* [2-3] event data, u16 LE */
    put_le16(p, data);
    p += 2;

    /* [4-5] event time (0.1s ticks), u16 LE */
    extern float flight_fsm_get_time_s(void);
    float time_ds = flight_fsm_get_time_s() / TIME_SCALE_100MS;
    if (time_ds < 0.0f) time_ds = 0.0f;
    if (time_ds > 65535.0f) time_ds = 65535.0f;
    put_le16(p, (uint16_t)time_ds);
    p += 2;

    /* [6] reserved */
    *p++ = 0x00;

    /* [7-10] CRC-32 over [0-6] */
    uint32_t crc = crc32_hw_compute(buf, 7);
    put_le32(p, crc);

    return SIZE_FC_MSG_EVENT;
}

/* ── TX: load packet into FIFO and start transmission ──────────────── */

static void start_tx(const uint8_t *pkt, uint8_t len)
{
    sx1276_set_mode(SX1276_MODE_STDBY);
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, SX1276_FIFO_TX_BASE);
    sx1276_write_fifo(pkt, len);
    sx1276_write_reg(SX1276_REG_PAYLOAD_LENGTH, len);
    sx1276_set_mode(SX1276_MODE_TX);

    s_radio_state = RADIO_STATE_TX;
    s_tx_start_ms = HAL_GetTick();
}

/* ── RX: open single-receive window after TX ───────────────────────── */

static void open_rx_window(void)
{
    sx1276_set_mode(SX1276_MODE_STDBY);
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, SX1276_FIFO_RX_BASE);

    /* Set symbol timeout for RXSINGLE */
    uint8_t mc2 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG_2);
    /* SymbTimeout[9:8] in bits 1:0 of MC2 */
    mc2 = (mc2 & 0xFC) | ((RADIO_RX_WINDOW_SYMBOLS >> 8) & 0x03);
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG_2, mc2);
    sx1276_write_reg(SX1276_REG_SYMB_TIMEOUT_LSB,
                     (uint8_t)(RADIO_RX_WINDOW_SYMBOLS & 0xFF));

    sx1276_clear_irq_flags(SX1276_IRQ_ALL);
    g_radio_dio0_flag = 0;
    g_radio_dio1_flag = 0;

    sx1276_set_mode(SX1276_MODE_RXSINGLE);

    s_radio_state = RADIO_STATE_RX;
    s_rx_start_ms = HAL_GetTick();
}

/* ── RX: validate and dispatch received packet ─────────────────────── */

static void handle_rx_packet(void)
{
    uint8_t irq = sx1276_get_irq_flags();
    sx1276_clear_irq_flags(SX1276_IRQ_ALL);
    g_radio_dio0_flag = 0;

    /* Check hardware CRC error */
    if (irq & SX1276_IRQ_PAYLOAD_CRC_ERROR) {
        s_consec_crc_errors++;
        return;
    }

    /* Read packet from FIFO */
    uint8_t nb_bytes = sx1276_read_reg(SX1276_REG_RX_NB_BYTES);
    uint8_t rx_addr  = sx1276_read_reg(SX1276_REG_FIFO_RX_CURRENT_ADDR);
    sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, rx_addr);

    if (nb_bytes > RADIO_MAX_PACKET_SIZE) {
        nb_bytes = RADIO_MAX_PACKET_SIZE;
    }
    sx1276_read_fifo(s_rx_buf, nb_bytes);

    /* Minimum packet: 1 msg_id + 4 CRC = 5 bytes */
    if (nb_bytes < 5) return;

    uint8_t msg_id = s_rx_buf[0];

    /* Only accept known command IDs */
    if (msg_id != MSG_ID_CMD_ARM &&
        msg_id != MSG_ID_CMD_FIRE &&
        msg_id != MSG_ID_CMD_TESTMODE &&
        msg_id != MSG_ID_CMD_POLL &&
        msg_id != MSG_ID_CONFIRM &&
        msg_id != MSG_ID_ABORT) {
        return;
    }

    /* Check magic bytes for CAC packets */
    if (msg_id >= MSG_ID_CMD_ARM && msg_id <= MSG_ID_CMD_POLL) {
        if (nb_bytes < 3) return;
        if (s_rx_buf[1] != CAC_MAGIC_1 || s_rx_buf[2] != CAC_MAGIC_2) return;
    }
    if (msg_id == MSG_ID_CONFIRM || msg_id == MSG_ID_ABORT) {
        if (nb_bytes < 3) return;
        if (s_rx_buf[1] != CAC_MAGIC_1 || s_rx_buf[2] != CAC_MAGIC_2) return;
    }

    /* Validate CRC-32: compute over [0..len-5], compare last 4 bytes */
    uint32_t payload_len = (uint32_t)(nb_bytes - 4);
    uint32_t computed_crc = crc32_hw_compute(s_rx_buf, payload_len);
    uint32_t received_crc = (uint32_t)s_rx_buf[payload_len]
                          | ((uint32_t)s_rx_buf[payload_len + 1] << 8)
                          | ((uint32_t)s_rx_buf[payload_len + 2] << 16)
                          | ((uint32_t)s_rx_buf[payload_len + 3] << 24);
    if (computed_crc != received_crc) {
        s_consec_crc_errors++;
        return;
    }

    /* CRC passed — reset error counter */
    s_consec_crc_errors = 0;

    /* Dispatch to CAC handlers */
    switch (msg_id) {
    case MSG_ID_CMD_ARM:
        cac_handle_arm(s_rx_buf, nb_bytes);
        break;
    case MSG_ID_CMD_FIRE:
        cac_handle_fire(s_rx_buf, nb_bytes);
        break;
    case MSG_ID_CMD_TESTMODE:
        cac_handle_testmode(s_rx_buf, nb_bytes);
        break;
    case MSG_ID_CMD_POLL:
        cac_handle_config_poll(s_rx_buf, nb_bytes);
        break;
    case MSG_ID_CONFIRM:
        cac_handle_confirm(s_rx_buf, nb_bytes);
        break;
    case MSG_ID_ABORT:
        cac_handle_abort(s_rx_buf, nb_bytes);
        break;
    default:
        break;
    }
}

/* ── TX scheduler: select highest-priority packet ──────────────────── */

static int select_and_build_packet(const fc_telem_state_t *tstate,
                                   const pyro_state_t *pstate,
                                   fsm_state_t fsm)
{
    /* Priority 1: response (ACK/NACK) */
    if (s_resp_count > 0) {
        resp_entry_t *e = &s_resp_queue[s_resp_tail];
        memcpy(s_tx_buf, e->data, e->len);
        int len = e->len;
        s_resp_tail = (s_resp_tail + 1) % RADIO_RESP_QUEUE_SIZE;
        s_resp_count--;
        return len;
    }

    /* Priority 2: event */
    if (s_evt_count > 0) {
        event_entry_t *e = &s_event_queue[s_evt_tail];
        int len = build_event_packet(s_tx_buf, e->type, e->data);
        s_evt_tail = (s_evt_tail + 1) % EVENT_QUEUE_SIZE;
        s_evt_count--;
        return len;
    }

    /* Priority 3: fast telemetry (default) */
    if (tstate != NULL && pstate != NULL) {
        return build_fast_packet(s_tx_buf, tstate, pstate, fsm);
    }

    /* Priority 4: GPS */
    if (s_gps_pending) {
        memcpy(s_tx_buf, s_gps_buf, SIZE_FC_MSG_GPS);
        s_gps_pending = 0;
        return SIZE_FC_MSG_GPS;
    }

    return 0;
}

/* ── Radio reset and re-init (after TX timeout) ────────────────────── */

static void radio_reinit(void)
{
    sx1276_reset();
    sx1276_set_lora_mode();

    /* DIO mapping */
    sx1276_write_reg(SX1276_REG_DIO_MAPPING_1, RADIO_DIO_MAPPING_1);
    sx1276_write_reg(SX1276_REG_DIO_MAPPING_2, RADIO_DIO_MAPPING_2);

    /* FIFO base addresses */
    sx1276_write_reg(SX1276_REG_FIFO_TX_BASE_ADDR, SX1276_FIFO_TX_BASE);
    sx1276_write_reg(SX1276_REG_FIFO_RX_BASE_ADDR, SX1276_FIFO_RX_BASE);

    /* Enable HW CRC */
    uint8_t mc2 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG_2);
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG_2, mc2 | SX1276_RX_CRC_ON);

    /* Re-apply current profile */
    if (s_current_profile) {
        radio_apply_profile(s_current_profile);
    }

    radio_irq_clear_all();
    s_radio_state = RADIO_STATE_IDLE;
}

/* ── Public API ────────────────────────────────────────────────────── */

int radio_manager_init(SPI_HandleTypeDef *hspi)
{
    /* Reset module state */
    s_seq = 0;
    s_last_tx_ms = 0;
    s_tx_start_ms = 0;
    s_rx_start_ms = 0;
    s_consec_crc_errors = 0;
    s_tx_error_count = 0;
    s_resp_head = 0;
    s_resp_tail = 0;
    s_resp_count = 0;
    s_evt_head = 0;
    s_evt_tail = 0;
    s_evt_count = 0;
    s_gps_pending = 0;

    /* Init SX1276 low-level driver */
    if (sx1276_init(hspi) != 0) {
        s_radio_state = RADIO_STATE_DISABLED;
        return -1;
    }

    /* Configure LoRa mode */
    sx1276_set_lora_mode();

    /* DIO mapping: DIO0=RxDone/TxDone, DIO1=RxTimeout */
    sx1276_write_reg(SX1276_REG_DIO_MAPPING_1, RADIO_DIO_MAPPING_1);
    sx1276_write_reg(SX1276_REG_DIO_MAPPING_2, RADIO_DIO_MAPPING_2);

    /* FIFO base addresses */
    sx1276_write_reg(SX1276_REG_FIFO_TX_BASE_ADDR, SX1276_FIFO_TX_BASE);
    sx1276_write_reg(SX1276_REG_FIFO_RX_BASE_ADDR, SX1276_FIFO_RX_BASE);

    /* Enable hardware CRC on RX */
    uint8_t mc2 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG_2);
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG_2, mc2 | SX1276_RX_CRC_ON);

    /* Apply default profile A */
    s_current_profile = &RADIO_PROFILE_A;
    radio_apply_profile(s_current_profile);

    radio_irq_clear_all();
    s_radio_state = RADIO_STATE_IDLE;

    return 0;
}

void radio_manager_tick(const casper_ekf_t *ekf,
                        const fc_telem_state_t *tstate,
                        const pyro_state_t *pstate,
                        fsm_state_t fsm)
{
    if (s_radio_state == RADIO_STATE_DISABLED) return;

    uint32_t now = HAL_GetTick();

    switch (s_radio_state) {

    case RADIO_STATE_IDLE:
        /* Check profile switch */
        if (ekf != NULL) {
            check_profile_switch(ekf);
        }

        /* TX scheduler: 10 Hz cadence */
        if (now - s_last_tx_ms >= RADIO_TX_PERIOD_MS) {
            int pkt_len = select_and_build_packet(tstate, pstate, fsm);
            if (pkt_len > 0) {
                start_tx(s_tx_buf, (uint8_t)pkt_len);
                s_last_tx_ms = now;
            }
        }
        break;

    case RADIO_STATE_TX:
        /* Check for TxDone (DIO0) */
        if (g_radio_dio0_flag) {
            sx1276_clear_irq_flags(SX1276_IRQ_ALL);
            g_radio_dio0_flag = 0;

            /* TX complete — open RX window */
            open_rx_window();
        }
        /* TX timeout */
        else if (now - s_tx_start_ms > RADIO_TX_TIMEOUT_MS) {
            s_tx_error_count++;
            radio_reinit();
        }
        break;

    case RADIO_STATE_RX:
        /* Check for RxDone (DIO0) */
        if (g_radio_dio0_flag) {
            handle_rx_packet();
            sx1276_set_mode(SX1276_MODE_STDBY);
            s_radio_state = RADIO_STATE_IDLE;
        }
        /* Check for RxTimeout (DIO1) */
        else if (g_radio_dio1_flag) {
            sx1276_clear_irq_flags(SX1276_IRQ_ALL);
            g_radio_dio1_flag = 0;
            sx1276_set_mode(SX1276_MODE_STDBY);
            s_radio_state = RADIO_STATE_IDLE;
        }
        /* Safety timeout (200 ms) */
        else if (now - s_rx_start_ms > RADIO_TX_TIMEOUT_MS) {
            sx1276_clear_irq_flags(SX1276_IRQ_ALL);
            radio_irq_clear_all();
            sx1276_set_mode(SX1276_MODE_STDBY);
            s_radio_state = RADIO_STATE_IDLE;
        }
        break;

    case RADIO_STATE_DISABLED:
        break;
    }
}

void radio_send_gps(const fc_gps_state_t *gps_state)
{
    if (s_radio_state == RADIO_STATE_DISABLED) return;
    /* Build GPS packet into the single-slot buffer */
    build_gps_packet(s_gps_buf, gps_state);
    s_gps_pending = 1;
}

void radio_queue_event(uint8_t type, uint16_t data)
{
    if (s_radio_state == RADIO_STATE_DISABLED) return;
    if (s_evt_count >= EVENT_QUEUE_SIZE) return;

    event_entry_t *e = &s_event_queue[s_evt_head];
    e->type = type;
    e->data = data;
    s_evt_head = (s_evt_head + 1) % EVENT_QUEUE_SIZE;
    s_evt_count++;
}

int radio_send_response(const uint8_t *buf, uint8_t len)
{
    if (s_radio_state == RADIO_STATE_DISABLED) return -1;
    if (s_resp_count >= RADIO_RESP_QUEUE_SIZE) return -1;
    if (len > RADIO_MAX_PACKET_SIZE) return -1;

    resp_entry_t *e = &s_resp_queue[s_resp_head];
    memcpy(e->data, buf, len);
    e->len = len;
    s_resp_head = (s_resp_head + 1) % RADIO_RESP_QUEUE_SIZE;
    s_resp_count++;

    return 0;
}

int radio_is_active(void)
{
    return (s_radio_state != RADIO_STATE_DISABLED) ? 1 : 0;
}
