/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Telemetry Types
 *  SUMMARY:  Message ID enums, packed-struct layouts, size constants.
 * ============================================================ */
#ifndef APP_TELEMETRY_TLM_TYPES_H
#define APP_TELEMETRY_TLM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ── Message IDs ─────────────────────────────────────────────────── */
#define MSG_ID_FAST          0x01
#define MSG_ID_GPS           0x02
#define MSG_ID_EVENT         0x03
#define MSG_ID_CMD_ARM       0x80
#define MSG_ID_CMD_FIRE      0x81
#define MSG_ID_CMD_TESTMODE  0x82
#define MSG_ID_CMD_POLL      0x83
#define MSG_ID_ACK_ARM       0xA0
#define MSG_ID_ACK_FIRE      0xA1
#define MSG_ID_ACK_CFG       0xA3
#define MSG_ID_HANDSHAKE     0xC0
#define MSG_ID_UPLOAD        0xC1
#define MSG_ID_DIAG          0xC2
#define MSG_ID_READLOG       0xC3
#define MSG_ID_ERASELOG      0xC4
#define MSG_ID_CONFIRM       0xF0
#define MSG_ID_ABORT         0xF1
#define MSG_ID_NACK          0xE0
#define MSG_ID_SIM_FLIGHT    0xD0

/* ── Magic / CAC ─────────────────────────────────────────────────── */
#define CAC_MAGIC_1          0xCA
#define CAC_MAGIC_2          0x5A
#define CAC_ACTION_ARM       0x01
#define CAC_ACTION_DISARM    0x00

/* ── NACK error codes ────────────────────────────────────────────── */
#define NACK_ERR_CRC_FAIL       0x01
#define NACK_ERR_BAD_STATE      0x02
#define NACK_ERR_NOT_ARMED      0x03
#define NACK_ERR_NO_TESTMODE    0x04
#define NACK_ERR_NONCE_REUSE    0x05
#define NACK_ERR_NO_CONTINUITY  0x06
#define NACK_ERR_LOW_BATTERY    0x07
#define NACK_ERR_SELF_TEST      0x08
#define NACK_ERR_CFG_TOO_LARGE  0x09
#define NACK_ERR_FLASH_FAIL     0x0A
#define NACK_ERR_NONCE_REPLAY   0x0B

/* ── Event types ─────────────────────────────────────────────────── */
#define FC_EVT_STATE         0x01
#define FC_EVT_PYRO          0x02
#define FC_EVT_APOGEE        0x03
#define FC_EVT_ERROR         0x04
#define FC_EVT_ORIGIN        0x05
#define FC_EVT_BURNOUT       0x06
#define FC_EVT_STAGING       0x07
#define FC_EVT_ARM           0x08
/* 0x09 / 0x0A are emitted by the CASPER-3 FC, not by this firmware. They are
 * declared here because the groundstation build (Software/App/ground/) lives
 * in this repo and relays C3 telemetry: the COBS path forwards every event
 * opaquely and needs no per-type knowledge, but the ASCII console names them,
 * and an event it cannot name prints as "UNK". Keep in step with the C3 repo's
 * flight/telemetry/tlm_types.h. */
#define FC_EVT_PYRO_MODE     0x09  /* C3 MC_FC_ALIGNMENT.md S7: boot pyro-mode
                                     * visibility; data = pyro_live_mask |
                                     * (stored_config_valid << 8)            */
#define FC_EVT_LOGIC_SHADOW  0x0A  /* C3 MC_FC_ALIGNMENT.md S13c: SHADOW-mode
                                     * Logic-VM decision edge (logged, NEVER
                                     * actuated); data = (ch << 8) |
                                     * min(duration_ms/10, 255)              */

/* ── Error codes ─────────────────────────────────────────────────── */
#define ERR_DROGUE_FAIL      0x01

/* ── FSM states ──────────────────────────────────────────────────── */
#define FSM_STATE_PAD        0x0
#define FSM_STATE_BOOST      0x1
#define FSM_STATE_COAST      0x2
#define FSM_STATE_COAST_1    0x3
#define FSM_STATE_SUSTAIN    0x4
#define FSM_STATE_COAST_2    0x5
#define FSM_STATE_APOGEE     0x6
#define FSM_STATE_DROGUE     0x7
#define FSM_STATE_MAIN       0x8
#define FSM_STATE_RECOVERY   0x9
#define FSM_STATE_TUMBLE     0xA
#define FSM_STATE_LANDED     0xB

/* ── Packet sizes (byte-counted per INTERFACE_SPEC.md) ───────────── */
#define SIZE_FC_MSG_FAST     21  /* [ID:1][STATUS:2][ALT:3][VEL:2][QUAT:5][TIME:2][BATT:1][SEQ:1][CRC:4] = 21 */
#define SIZE_FC_MSG_GPS      18  /* [ID:1][LAT:4][LON:4][ALT:3][FIX:1][SAT:1][CRC:4] = 18 */
#define SIZE_FC_MSG_EVENT    11  /* [ID:1][TYPE:1][DATA:2][TIME:2][RSVD:1][CRC:4] = 11 */
#define SIZE_CMD_ARM         12  /* [ID:1][MAG:2][NONCE:2][CH:1][ACT:1][~CH:1][CRC:4] = 12 */
#define SIZE_CMD_FIRE        13  /* [ID:1][MAG:2][NONCE:2][CH:1][DUR:1][~CH:1][~DUR:1][CRC:4] = 13 */
#define SIZE_CMD_TESTMODE    10  /* [ID:1][MAG:2][NONCE:2][EN:1][CRC:4] = 10 */
#define SIZE_CONFIRM         9   /* [ID:1][MAG:2][NONCE:2][CRC:4] = 9 */
#define SIZE_NACK            10  /* [ID:1][NONCE:2][ERR:1][RSVD:2][CRC:4] = 10 */
#define SIZE_ACK_ARM         12  /* [ID:1][NONCE:2][CH:1][ACT:1][ARM:1][CONT:1][RSVD:1][CRC:4] = 12 */
#define SIZE_ACK_FIRE        13  /* [ID:1][NONCE:2][CH:1][DUR:1][FLAGS:1][CONT:1][RSVD:2][CRC:4] = 13 */
#define SIZE_ACK_CFG         13  /* [ID:1][NONCE:2][HASH:4][VER:1][RSVD:1][CRC:4] = 13 */
#define SIZE_HANDSHAKE_RESP  13

/* ── Timeouts ────────────────────────────────────────────────────── */
#define CAC_CONFIRM_TIMEOUT_MS   5000
#define TEST_MODE_TIMEOUT_MS     60000
#define TLM_FAST_PERIOD_MS       100

/* ── Version ─────────────────────────────────────────────────────── */
#define PROTOCOL_VERSION     5
#define FW_VERSION_MAJOR     0
#define FW_VERSION_MINOR     1
#define FW_VERSION_PATCH     0

/* ── Pyro ────────────────────────────────────────────────────────── */
#define PYRO_MGR_NUM_CHANNELS    4
#define PYRO_MAX_FIRE_MS         2000

/* ── Encoding scales ─────────────────────────────────────────────── */
#define BATT_OFFSET_V        6.0f
#define BATT_STEP_V          0.012f
#define ALT_SCALE_M          0.01f   /* 1 cm per LSB, u24 max = 167,772 m */
#define VEL_SCALE_DMS        0.1f
#define TIME_SCALE_100MS     0.1f

/* ── State structs ───────────────────────────────────────────────── */
typedef uint8_t fsm_state_t;

typedef struct {
    float alt_m;
    float vel_mps;
    float quat[4];
    float batt_v;
    float flight_time_s;
    /* v2.1 additions for FSM launch/landing detection */
    float accel_mag_g;      /* |accel| from LSM6DSO32, in g          */
    float baro_alt_m;       /* barometric altitude AGL (for landing) */
    bool  adxl_activity;    /* ADXL372 activity INT asserted         */
    bool  adxl_available;   /* ADXL372 passed init (degraded mode)   */
} fc_telem_state_t;

/* FC_MSG_GPS payload state. Bytes 1-8 are ABSOLUTE coordinates as of the
 * 2026-08-29 wire-contract change -- they were dlat_mm/dlon_mm, millimetres
 * from a pad origin the vehicle captured on its first valid fix and never
 * transmitted. The packet size did not change, so the two encodings are
 * indistinguishable on the wire; ground_radio.c's MSG_ID_GPS decoder and
 * Mission Control both read absolute now, and this struct must agree with
 * them or the GS prints degrees as though they were millimetres.
 *
 * Nothing in the Casper-2 flight build currently emits FC_MSG_GPS
 * (tlm_send_gps() and radio_send_gps() have no callers) -- this is the
 * shared contract, kept correct so it cannot drift back. */
typedef struct {
    int32_t lat_deg7;       /* absolute latitude,  degrees x 1e-7 (UBX NAV-PVT encoding) */
    int32_t lon_deg7;       /* absolute longitude, degrees x 1e-7 */
    float alt_msl_m;
    uint8_t fix_type;
    uint8_t sat_count;
} fc_gps_state_t;

typedef struct {
    bool armed[PYRO_MGR_NUM_CHANNELS];
    bool continuity[PYRO_MGR_NUM_CHANNELS];
    bool fired;
    float cont_v[PYRO_MGR_NUM_CHANNELS];
} pyro_state_t;

/* ── Diagnostic result entry ─────────────────────────────────────── */
typedef struct {
    uint8_t test_id;
    uint8_t result;
    uint16_t detail;
} diag_result_t;

/* ── Flight config ───────────────────────────────────────────────── */
typedef struct {
    float pad_lat_deg;
    float pad_lon_deg;
    float pad_alt_m;
    float main_deploy_alt_m;
    float launch_accel_g;
    uint32_t config_hash;
} flight_config_t;

/* ── COBS max overhead ───────────────────────────────────────────── */
#define COBS_MAX_OVERHEAD    2   /* 1 overhead byte + 1 delimiter */
#define TLM_TX_BUF_SIZE      (SIZE_FC_MSG_FAST + COBS_MAX_OVERHEAD + 1)

/* ── Ground station message IDs ─────────────────────────────────── */
#define MSG_ID_GS_TELEM      0x10
#define MSG_ID_GS_STATUS     0x13

/* ── Ground station packet sizes ────────────────────────────────── */
/* GS_MSG_TELEM (0x10) byte layout — MUST match Mission Control decoder exactly:
 * [ID:1][FC_RELAY:15][SEQ:1][RSSI:2][SNR:1][FREQ_ERR:2][DATA_AGE:2]
 * [RECOV:1][MACH:2][QBAR:2][ROLL:2][PITCH:2][YAW:2][CRC:4] = 39 */
#define SIZE_GS_MSG_TELEM    39
#define SIZE_GS_MSG_STATUS   24  /* [ID:1][PROF:1][RSSI:1][SNR:1][PKTS:2][FAIL:2][BARO:4][LAT:4][LON:4][CRC:4] = 24 */

/* ── Ground station state structs ───────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint8_t  msg_id;              /* 0x13                          */
    uint8_t  radio_profile;       /* 0=A (SF7), 1=B (SF8)          */
    int8_t   last_rssi;           /* dBm                           */
    int8_t   last_snr;            /* dB (signed)                   */
    uint16_t rx_pkt_count;        /* total packets received        */
    uint16_t rx_crc_fail;         /* CRC failures                  */
    uint32_t ground_pressure_pa;  /* Pa as integer (range 0-120000)*/
    int32_t  ground_lat_1e7;      /* degrees x 10^7 (UBX encoding) */
    int32_t  ground_lon_1e7;      /* degrees x 10^7 (UBX encoding) */
    uint32_t crc32;               /* CRC over bytes 0..N-5         */
} gs_msg_status_t;
_Static_assert(sizeof(gs_msg_status_t) == SIZE_GS_MSG_STATUS,
              "gs_msg_status_t must be SIZE_GS_MSG_STATUS bytes");

/* ── Flight FSM configuration ───────────────────────────────── */
typedef struct {
    float    main_deploy_alt;     /* m AGL, default 250.0 */
    float    drogue_fail_vel;     /* m/s, default 50.0    */
    float    drogue_fail_time;    /* s, default 3.0       */
    uint8_t  apogee_pyro_ch;     /* 0-indexed, default 0 */
    uint8_t  main_pyro_ch;       /* 0-indexed, default 1 */
    uint16_t apogee_fire_dur;    /* ms, default 1000     */
    uint16_t main_fire_dur;      /* ms, default 1000     */
} flight_cfg_t;

/* ── Flash dump ─────────────────────────────────────────────── */
#define MSG_ID_DUMP_FLASH    0xD2

/* ── HIL messages ───────────────────────────────────────────── */
#define MSG_ID_HIL_INJECT        0xD1
#define SIZE_HIL_INJECT          44  /* [ID:1][TICK:4][ALT:4][VEL:4][VA:4][ANT:1][FT:4][MDA:4][DFV:4][DFT:4][ACH:1][MCH:1][AFD:2][MFD:2][CRC:4] = 44 */
#define MSG_ID_HIL_RAW_INJECT    0xD3
#define SIZE_HIL_RAW_INJECT      50  /* [ID:1][TICK:4][ACCEL:12][GYRO:12][BARO:4][MAG:12][FLAGS:1][CRC:4] = 50 */

#endif /* APP_TELEMETRY_TLM_TYPES_H */
