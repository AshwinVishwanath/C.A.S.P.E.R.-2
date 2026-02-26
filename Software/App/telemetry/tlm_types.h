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

/* ── Event types ─────────────────────────────────────────────────── */
#define FC_EVT_STATE         0x01
#define FC_EVT_PYRO          0x02
#define FC_EVT_APOGEE        0x03
#define FC_EVT_ERROR         0x04
#define FC_EVT_ORIGIN        0x05
#define FC_EVT_BURNOUT       0x06
#define FC_EVT_STAGING       0x07
#define FC_EVT_ARM           0x08

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
#define SIZE_FC_MSG_FAST     20  /* [ID:1][STATUS:2][ALT:2][VEL:2][QUAT:5][TIME:2][BATT:1][SEQ:1][CRC:4] = 20 */
#define SIZE_FC_MSG_GPS      17  /* [ID:1][DLAT:4][DLON:4][ALT:2][FIX:1][SAT:1][CRC:4] = 17 */
#define SIZE_FC_MSG_EVENT    11  /* [ID:1][TYPE:1][DATA:2][TIME:2][RSVD:1][CRC:4] = 11 */
#define SIZE_CMD_ARM         12  /* [ID:1][MAG:2][NONCE:2][CH:1][ACT:1][~CH:1][CRC:4] = 12 */
#define SIZE_CMD_FIRE        13  /* [ID:1][MAG:2][NONCE:2][CH:1][DUR:1][~CH:1][~DUR:1][CRC:4] = 13 */
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
#define ALT_SCALE_M          1.0f
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

typedef struct {
    int32_t dlat_mm;        /* delta latitude from pad, millimetres (spec: ÷1000 → m) */
    int32_t dlon_mm;        /* delta longitude from pad, millimetres */
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
#define MSG_ID_GS_STATUS     0x13

/* ── Ground station packet sizes ────────────────────────────────── */
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

/* ── HIL message ────────────────────────────────────────────── */
#define MSG_ID_HIL_INJECT    0xD1
#define SIZE_HIL_INJECT      44  /* [ID:1][TICK:4][ALT:4][VEL:4][VA:4][ANT:1][FT:4][MDA:4][DFV:4][DFT:4][ACH:1][MCH:1][AFD:2][MFD:2][CRC:4] = 44 */

#endif /* APP_TELEMETRY_TLM_TYPES_H */
