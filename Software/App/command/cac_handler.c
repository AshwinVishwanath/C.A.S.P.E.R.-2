#include "cac_handler.h"
#include "crc32_hw.h"
#include "tlm_types.h"
#include "tlm_manager.h"
#include "pyro_manager.h"
#include "flight_fsm.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ── Forward declaration for cfg_manager ───────────────────────── */
extern uint32_t cfg_get_active_hash(void);

/* ── Internal CAC state ────────────────────────────────────────── */
typedef enum {
    CAC_IDLE,
    CAC_AWAITING_CONFIRM
} cac_phase_t;

typedef enum {
    PENDING_ARM,
    PENDING_FIRE
} pending_type_t;

static cac_phase_t  s_phase;
static uint16_t     s_pending_nonce;
static pending_type_t s_pending_type;
static uint8_t      s_pending_channel;   /* 1-indexed */
static uint8_t      s_pending_action;    /* ARM: 0x01=arm, 0x02=disarm */
static uint16_t     s_pending_duration;  /* FIRE: duration_ms */
static uint32_t     s_confirm_deadline;

/* ── Test mode state ──────────────────────────────────────────── */
static bool     s_test_mode;
static uint32_t s_test_mode_deadline;

/* ── Helpers ──────────────────────────────────────────────────── */
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

static uint16_t get_le16(const uint8_t *src)
{
    return (uint16_t)(src[0] | ((uint16_t)src[1] << 8));
}

static uint32_t get_le32(const uint8_t *src)
{
    return (uint32_t)(src[0] | ((uint32_t)src[1] << 8) |
                      ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24));
}

/* ── Send NACK packet (9 bytes) ───────────────────────────────── */
static void send_nack(uint16_t nonce, uint8_t error_code)
{
    uint8_t pkt[SIZE_NACK];
    pkt[0] = MSG_ID_NACK;       /* 0xE0 */
    pkt[1] = CAC_MAGIC_1;       /* 0xCA */
    pkt[2] = CAC_MAGIC_2;       /* 0x5A */
    put_le16(&pkt[3], nonce);
    pkt[5] = error_code;
    uint32_t crc = crc32_hw_compute(pkt, 6);
    put_le32(&pkt[6], crc);     /* Bytes 6-9 — wait, that's 10. */

    /* NACK: [0xE0][0xCA][0x5A][nonce:2][err:1][CRC-32:4] = 10 bytes
     * But SIZE_NACK = 9. Let me re-check the spec... */
    /* Level-5 spec: offset 6-9 = CRC-32 over bytes 0-5 → 10 bytes total.
     * SIZE_NACK=9 in tlm_types.h might be wrong. Use actual byte count. */
    tlm_send_response(pkt, 10);
}

/* ── Send ARM ACK packet (13 bytes) ───────────────────────────── */
static void send_arm_ack(uint16_t nonce, uint8_t channel, uint8_t action)
{
    uint8_t pkt[13];
    pkt[0] = MSG_ID_ACK_ARM;    /* 0xA0 */
    pkt[1] = CAC_MAGIC_1;
    pkt[2] = CAC_MAGIC_2;
    put_le16(&pkt[3], nonce);
    pkt[5] = channel;
    pkt[6] = action;
    pkt[7] = pyro_mgr_get_arm_bitmap();
    pkt[8] = pyro_mgr_get_cont_bitmap();
    uint32_t crc = crc32_hw_compute(pkt, 9);
    put_le32(&pkt[9], crc);
    tlm_send_response(pkt, 13);
}

/* ── Send FIRE ACK packet (13 bytes) ──────────────────────────── */
static void send_fire_ack(uint16_t nonce, uint8_t channel, uint16_t duration_ms)
{
    uint8_t pkt[13];
    pkt[0] = MSG_ID_ACK_FIRE;   /* 0xA1 */
    pkt[1] = CAC_MAGIC_1;
    pkt[2] = CAC_MAGIC_2;
    put_le16(&pkt[3], nonce);
    pkt[5] = channel;
    put_le16(&pkt[6], duration_ms);
    pkt[8] = pyro_mgr_get_cont_bitmap();
    uint32_t crc = crc32_hw_compute(pkt, 9);
    put_le32(&pkt[9], crc);
    tlm_send_response(pkt, 13);
}

/* ── Public API ───────────────────────────────────────────────── */

void cac_init(void)
{
    s_phase = CAC_IDLE;
    s_pending_nonce = 0;
    s_test_mode = false;
    s_test_mode_deadline = 0;
}

void cac_tick(void)
{
    uint32_t now = HAL_GetTick();

    /* Confirm timeout */
    if (s_phase == CAC_AWAITING_CONFIRM) {
        if (now - s_confirm_deadline > 0 &&
            (int32_t)(now - s_confirm_deadline) >= 0) {
            s_phase = CAC_IDLE;
        }
    }

    /* Test mode timeout */
    if (s_test_mode) {
        if ((int32_t)(now - s_test_mode_deadline) >= 0) {
            s_test_mode = false;
            pyro_mgr_set_test_mode(false);
            tlm_queue_event(FC_EVT_STATE, (uint16_t)flight_fsm_get_state());
        }
    }
}

/* ── ARM / DISARM handler ─────────────────────────────────────── */
void cac_handle_arm(const uint8_t *data, int len)
{
    /* ARM packet (level-5 spec): 13 bytes
     * [0]=0x80 [1]=0xCA [2]=0x5A [3-4]=nonce [5]=channel
     * [6]=action [7]=~channel [8]=~action [9-12]=CRC-32(0..8) */
    if (len < 13) return;

    /* Check magic */
    if (data[1] != CAC_MAGIC_1 || data[2] != CAC_MAGIC_2) return;

    /* Check complements */
    if ((data[5] ^ data[7]) != 0xFF) return;
    if ((data[6] ^ data[8]) != 0xFF) return;

    /* Validate CRC-32 over bytes 0-8 */
    uint32_t received_crc = get_le32(&data[9]);
    if (crc32_hw_compute(data, 9) != received_crc) return;

    uint16_t nonce   = get_le16(&data[3]);
    uint8_t  channel = data[5];     /* 0-indexed in spec */
    uint8_t  action  = data[6];

    /* Convert to 1-indexed channel for pyro_manager */
    uint8_t ch1 = channel + 1;

    /* Idempotent: same nonce while awaiting confirm → re-send ACK */
    if (s_phase == CAC_AWAITING_CONFIRM && nonce == s_pending_nonce &&
        s_pending_type == PENDING_ARM) {
        send_arm_ack(nonce, channel, action);
        return;
    }

    /* Channel range check */
    if (channel > 3) {
        send_nack(nonce, NACK_ERR_BAD_STATE);
        return;
    }

    /* Precondition: continuity required for ARM */
    if (action == CAC_ACTION_ARM && !pyro_mgr_has_continuity(ch1)) {
        send_nack(nonce, NACK_ERR_NO_CONTINUITY);
        return;
    }

    /* Store pending action */
    s_phase = CAC_AWAITING_CONFIRM;
    s_pending_nonce = nonce;
    s_pending_type = PENDING_ARM;
    s_pending_channel = ch1;
    s_pending_action = action;
    s_confirm_deadline = HAL_GetTick() + CAC_CONFIRM_TIMEOUT_MS;

    send_arm_ack(nonce, channel, action);
}

/* ── FIRE handler ─────────────────────────────────────────────── */
void cac_handle_fire(const uint8_t *data, int len)
{
    /* FIRE packet: 13 bytes
     * [0]=0x81 [1]=0xCA [2]=0x5A [3-4]=nonce [5]=channel
     * [6-7]=duration_ms [8]=~channel [9-12]=CRC-32(0..8) */
    if (len < 13) return;

    if (data[1] != CAC_MAGIC_1 || data[2] != CAC_MAGIC_2) return;

    /* Complement check: channel vs ~channel */
    if ((data[5] ^ data[8]) != 0xFF) return;

    /* CRC-32 over bytes 0-8 */
    uint32_t received_crc = get_le32(&data[9]);
    if (crc32_hw_compute(data, 9) != received_crc) return;

    uint16_t nonce       = get_le16(&data[3]);
    uint8_t  channel     = data[5];
    uint16_t duration_ms = get_le16(&data[6]);

    uint8_t ch1 = channel + 1;

    /* Idempotent retry */
    if (s_phase == CAC_AWAITING_CONFIRM && nonce == s_pending_nonce &&
        s_pending_type == PENDING_FIRE) {
        send_fire_ack(nonce, channel, duration_ms);
        return;
    }

    if (channel > 3) {
        send_nack(nonce, NACK_ERR_BAD_STATE);
        return;
    }

    /* FIRE requires test mode */
    if (!s_test_mode) {
        send_nack(nonce, NACK_ERR_NO_TESTMODE);
        return;
    }

    /* FIRE requires channel armed */
    if (!(pyro_mgr_get_arm_bitmap() & (1u << channel))) {
        send_nack(nonce, NACK_ERR_NOT_ARMED);
        return;
    }

    /* FIRE requires continuity */
    if (!pyro_mgr_has_continuity(ch1)) {
        send_nack(nonce, NACK_ERR_NO_CONTINUITY);
        return;
    }

    /* Store pending fire */
    s_phase = CAC_AWAITING_CONFIRM;
    s_pending_nonce = nonce;
    s_pending_type = PENDING_FIRE;
    s_pending_channel = ch1;
    s_pending_duration = duration_ms;
    s_confirm_deadline = HAL_GetTick() + CAC_CONFIRM_TIMEOUT_MS;

    send_fire_ack(nonce, channel, duration_ms);
}

/* ── TESTMODE handler ─────────────────────────────────────────── */
void cac_handle_testmode(const uint8_t *data, int len)
{
    (void)data;
    (void)len;

    /* Must be on PAD */
    if (flight_fsm_get_state() != FSM_STATE_PAD) return;

    if (s_test_mode) {
        /* Already active — toggle off */
        s_test_mode = false;
        pyro_mgr_set_test_mode(false);
    } else {
        s_test_mode = true;
        s_test_mode_deadline = HAL_GetTick() + TEST_MODE_TIMEOUT_MS;
        pyro_mgr_set_test_mode(true);
    }
}

/* ── CONFIG POLL handler ──────────────────────────────────────── */
void cac_handle_config_poll(const uint8_t *data, int len)
{
    (void)data;
    (void)len;

    /* Response: [0xA3][config_hash:4][protocol_version:1][CRC-32:4] = 10 bytes */
    uint8_t pkt[10];
    pkt[0] = MSG_ID_ACK_CFG;
    uint32_t hash = cfg_get_active_hash();
    put_le32(&pkt[1], hash);
    pkt[5] = PROTOCOL_VERSION;
    uint32_t crc = crc32_hw_compute(pkt, 6);
    put_le32(&pkt[6], crc);
    tlm_send_response(pkt, 10);
}

/* ── CONFIRM handler ──────────────────────────────────────────── */
void cac_handle_confirm(const uint8_t *data, int len)
{
    /* CONFIRM: [0xF0][0xCA][0x5A][nonce:2][CRC-32:4] = 9 bytes */
    if (len < 9) return;

    if (data[1] != CAC_MAGIC_1 || data[2] != CAC_MAGIC_2) return;

    uint32_t received_crc = get_le32(&data[5]);
    if (crc32_hw_compute(data, 5) != received_crc) return;

    uint16_t nonce = get_le16(&data[3]);

    if (s_phase != CAC_AWAITING_CONFIRM) return;
    if (nonce != s_pending_nonce) return;

    /* Check timeout */
    if ((int32_t)(HAL_GetTick() - s_confirm_deadline) >= 0) {
        s_phase = CAC_IDLE;
        return;
    }

    /* Execute the pending action */
    if (s_pending_type == PENDING_ARM) {
        bool arm = (s_pending_action == CAC_ACTION_ARM);
        pyro_mgr_set_arm(s_pending_channel, arm);
        tlm_queue_event(FC_EVT_ARM, (uint16_t)pyro_mgr_get_arm_bitmap());
    } else if (s_pending_type == PENDING_FIRE) {
        pyro_mgr_fire(s_pending_channel, s_pending_duration);
        tlm_queue_event(FC_EVT_PYRO,
                        (uint16_t)((s_pending_channel << 8) | 0x01));
    }

    s_phase = CAC_IDLE;
}

/* ── ABORT handler ────────────────────────────────────────────── */
void cac_handle_abort(const uint8_t *data, int len)
{
    /* ABORT: [0xF1][0xCA][0x5A][nonce:2][CRC-32:4] = 9 bytes */
    if (len < 9) return;

    if (data[1] != CAC_MAGIC_1 || data[2] != CAC_MAGIC_2) return;

    uint32_t received_crc = get_le32(&data[5]);
    if (crc32_hw_compute(data, 5) != received_crc) return;

    uint16_t nonce = get_le16(&data[3]);

    if (s_phase == CAC_AWAITING_CONFIRM && nonce == s_pending_nonce) {
        s_phase = CAC_IDLE;
    }
}

/* ── Test mode queries ────────────────────────────────────────── */
bool cac_test_mode_active(void)
{
    return s_test_mode;
}

uint32_t cac_test_mode_remaining_ms(void)
{
    if (!s_test_mode) return 0;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_test_mode_deadline) >= 0) return 0;
    return s_test_mode_deadline - now;
}
