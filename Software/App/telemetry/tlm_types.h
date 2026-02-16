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
#define CAC_ACTION_DISARM    0x02

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

/* ── Packet sizes ────────────────────────────────────────────────── */
#define SIZE_FC_MSG_FAST     19
#define SIZE_FC_MSG_GPS      12
#define SIZE_FC_MSG_EVENT    9
#define SIZE_CMD_ARM         12
#define SIZE_CMD_FIRE        13
#define SIZE_CONFIRM         9
#define SIZE_NACK            9
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
#define ALT_SCALE_DAM        10.0f
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
} fc_telem_state_t;

typedef struct {
    int16_t dlat_m;
    int16_t dlon_m;
    float alt_msl_m;
    uint8_t fix_type;
    uint8_t sat_count;
    float pdop;
    bool new_fix;
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

#endif /* APP_TELEMETRY_TLM_TYPES_H */
