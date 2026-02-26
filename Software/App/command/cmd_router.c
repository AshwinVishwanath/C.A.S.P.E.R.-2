#include "cmd_router.h"
#include "cobs.h"
#include "crc32_hw.h"
#include "tlm_types.h"
#include "tlm_manager.h"
#include "flight_fsm.h"
#include "endian.h"
#include "usbd_cdc_if.h"
#include <string.h>
#ifdef HIL_MODE
#include "hil_handler.h"
#endif

/* ── Forward declarations for handlers in other modules ────── */
extern void cac_handle_arm(const uint8_t *data, int len);
extern void cac_handle_fire(const uint8_t *data, int len);
extern void cac_handle_testmode(const uint8_t *data, int len);
extern void cac_handle_config_poll(const uint8_t *data, int len);
extern void cac_handle_confirm(const uint8_t *data, int len);
extern void cac_handle_abort(const uint8_t *data, int len);
extern void cfg_handle_upload(const uint8_t *data, int len);
extern void cfg_handle_readlog(const uint8_t *data, int len);
extern void cfg_handle_eraselog(const uint8_t *data, int len);
extern int  self_test_run_and_send(void);
extern uint32_t cfg_get_active_hash(void);

/* ── Private state ──────────────────────────────────────────── */
#define FRAME_BUF_SIZE  64
static uint8_t s_frame_buf[FRAME_BUF_SIZE];
static int     s_frame_pos;

#define DECODE_BUF_SIZE 64
static uint8_t s_decode_buf[DECODE_BUF_SIZE];

/* ── Handshake handler (0xC0) — implemented here ────────────── */
static void cmd_handle_handshake(void)
{
    uint8_t resp[SIZE_HANDSHAKE_RESP];

    resp[0] = MSG_ID_HANDSHAKE;      /* 0xC0 */
    resp[1] = PROTOCOL_VERSION;      /* 5    */
    resp[2] = FW_VERSION_MAJOR;
    resp[3] = FW_VERSION_MINOR;
    resp[4] = FW_VERSION_PATCH;

    uint32_t hash = cfg_get_active_hash();
    put_le32(&resp[5], hash);

    uint32_t crc = crc32_hw_compute(resp, 9);
    put_le32(&resp[9], crc);

    tlm_send_response(resp, SIZE_HANDSHAKE_RESP);
}

/* ── Sim flight handler (0xD0) — implemented here ───────────── */
static void cmd_handle_sim_flight(const uint8_t *data, int len)
{
    (void)data;
    (void)len;

    if (flight_fsm_sim_active()) {
        flight_fsm_sim_stop();
    } else {
        flight_fsm_sim_start();
    }
}

/* ── Diagnostics handler (0xC2) — implemented here ──────────── */
static void cmd_handle_diag(void)
{
    self_test_run_and_send();
}

/* ── Frame dispatch ─────────────────────────────────────────── */
static void dispatch_frame(const uint8_t *decoded, int len)
{
    if (len < 1) {
        return;
    }

    uint8_t msg_id = decoded[0];

    switch (msg_id) {
    /* CAC commands */
    case MSG_ID_CMD_ARM:       cac_handle_arm(decoded, len);         break;
    case MSG_ID_CMD_FIRE:      cac_handle_fire(decoded, len);        break;
    case MSG_ID_CMD_TESTMODE:  cac_handle_testmode(decoded, len);    break;
    case MSG_ID_CMD_POLL:      cac_handle_config_poll(decoded, len); break;
    case MSG_ID_CONFIRM:       cac_handle_confirm(decoded, len);     break;
    case MSG_ID_ABORT:         cac_handle_abort(decoded, len);       break;

    /* System commands */
    case MSG_ID_HANDSHAKE:     cmd_handle_handshake();               break;
    case MSG_ID_UPLOAD:        cfg_handle_upload(decoded, len);      break;
    case MSG_ID_DIAG:          cmd_handle_diag();                    break;
    case MSG_ID_READLOG:       cfg_handle_readlog(decoded, len);     break;
    case MSG_ID_ERASELOG:      cfg_handle_eraselog(decoded, len);     break;
    case MSG_ID_SIM_FLIGHT:    cmd_handle_sim_flight(decoded, len);  break;

#ifdef HIL_MODE
    case MSG_ID_HIL_INJECT:    hil_handle_inject(decoded, len);  break;
#endif

    default:
        /* Unknown message — silently discard */
        break;
    }
}

/* ── Public API ─────────────────────────────────────────────── */

void cmd_router_init(void)
{
    s_frame_pos = 0;
}

void cmd_router_process(void)
{
    while (cdc_ring_available() > 0) {
        uint8_t byte = cdc_ring_read_byte();

        if (byte == 0x00) {
            /* End-of-frame delimiter — decode and dispatch */
            if (s_frame_pos > 0) {
                int dec_len = cobs_decode(s_frame_buf, s_frame_pos,
                                          s_decode_buf, DECODE_BUF_SIZE);
                if (dec_len > 0) {
                    dispatch_frame(s_decode_buf, dec_len);
                }
            }
            s_frame_pos = 0;
        } else {
            /* Accumulate COBS-encoded byte */
            if (s_frame_pos < FRAME_BUF_SIZE) {
                s_frame_buf[s_frame_pos++] = byte;
            } else {
                /* Frame too long — reset and wait for next delimiter */
                s_frame_pos = 0;
            }
        }
    }
}
