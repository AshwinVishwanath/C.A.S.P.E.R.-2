/**
 * @file max_m10m.h
 * @brief U-blox MAX-M10M GPS driver over I2C (DDC protocol).
 *
 * Non-blocking tick-based architecture. Parses UBX-NAV-PVT for
 * position, velocity, fix status. Configured for 10 Hz GPS-only.
 *
 * Reference: u-blox M10 SPG 5.10 Interface Description (UBX-21035062)
 */

#ifndef MAX_M10M_H
#define MAX_M10M_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Return codes ───────────────────────────────────────────────── */
#define MAX_M10M_OK      0
#define MAX_M10M_ERROR  -1

/* ── I2C DDC registers ──────────────────────────────────────────── */
#define MAX_M10M_I2C_ADDR          0x42u   /* 7-bit default address */
#define MAX_M10M_REG_BYTES_HI      0xFDu   /* Bytes available MSB   */
#define MAX_M10M_REG_BYTES_LO      0xFEu   /* Bytes available LSB   */
#define MAX_M10M_REG_DATA_STREAM   0xFFu   /* Data stream register  */

/* ── UBX protocol constants ─────────────────────────────────────── */
#define UBX_SYNC_1                 0xB5u
#define UBX_SYNC_2                 0x62u

/* Message classes */
#define UBX_CLASS_NAV              0x01u
#define UBX_CLASS_ACK              0x05u
#define UBX_CLASS_CFG              0x06u
#define UBX_CLASS_MON              0x0Au

/* Message IDs */
#define UBX_NAV_PVT_ID            0x07u
#define UBX_ACK_ACK_ID            0x01u
#define UBX_ACK_NAK_ID            0x00u
#define UBX_CFG_VALSET_ID         0x8Au
#define UBX_MON_VER_ID            0x04u

/* NAV-PVT payload length */
#define UBX_NAV_PVT_LEN           92u

/* ── M10 configuration keys (UBX-CFG-VALSET / VALGET) ───────────── */
#define CFG_MSGOUT_UBX_NAV_PVT_I2C  0x20910007u
#define CFG_MSGOUT_NMEA_GGA_I2C     0x209100BAu
#define CFG_MSGOUT_NMEA_GLL_I2C     0x209100C9u
#define CFG_MSGOUT_NMEA_GSA_I2C     0x209100BFu
#define CFG_MSGOUT_NMEA_GSV_I2C     0x209100C4u
#define CFG_MSGOUT_NMEA_RMC_I2C     0x209100ABu
#define CFG_MSGOUT_NMEA_VTG_I2C     0x209100B0u
#define CFG_RATE_MEAS                0x30210001u

/* GNSS signal enable/disable keys (for GPS-only 10 Hz) */
#define CFG_SIGNAL_GPS_ENA          0x1031001Fu
#define CFG_SIGNAL_GAL_ENA          0x10310021u
#define CFG_SIGNAL_BDS_ENA          0x10310022u
#define CFG_SIGNAL_QZSS_ENA        0x10310024u
#define CFG_SIGNAL_GLO_ENA         0x10310025u

/* ── UBX parser state machine ───────────────────────────────────── */
typedef enum {
    UBX_PARSE_SYNC1,
    UBX_PARSE_SYNC2,
    UBX_PARSE_CLASS,
    UBX_PARSE_ID,
    UBX_PARSE_LEN1,
    UBX_PARSE_LEN2,
    UBX_PARSE_PAYLOAD,
    UBX_PARSE_CK_A,
    UBX_PARSE_CK_B
} ubx_parse_state_t;

/* ── Non-blocking tick state ────────────────────────────────────── */
typedef enum {
    GPS_TICK_IDLE,
    GPS_TICK_READ_AVAIL,
    GPS_TICK_READ_DATA
} gps_tick_state_t;

/* ── GPS fix type (NAV-PVT fixType field) ───────────────────────── */
typedef enum {
    GPS_FIX_NONE           = 0,
    GPS_FIX_DEAD_RECKONING = 1,
    GPS_FIX_2D             = 2,
    GPS_FIX_3D             = 3,
    GPS_FIX_GNSS_DR        = 4,
    GPS_FIX_TIME_ONLY      = 5
} gps_fix_type_t;

/* ── Driver struct ──────────────────────────────────────────────── */
typedef struct {
    /* HAL handles */
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef      *nrst_port;
    uint16_t           nrst_pin;

    /* Parsed NAV-PVT data (raw integers) */
    int32_t            lat_deg7;       /* degrees * 1e-7  */
    int32_t            lon_deg7;       /* degrees * 1e-7  */
    int32_t            h_msl_mm;       /* mm above MSL    */
    int32_t            vel_n_mm_s;     /* NED north mm/s  */
    int32_t            vel_e_mm_s;     /* NED east  mm/s  */
    int32_t            vel_d_mm_s;     /* NED down  mm/s  */
    uint32_t           h_acc_mm;       /* horiz accuracy mm */
    uint32_t           v_acc_mm;       /* vert accuracy mm  */
    uint32_t           iTOW;           /* GPS time of week ms */
    uint16_t           pDOP;           /* position DOP * 100  */
    uint8_t            fix_type;       /* gps_fix_type_t      */
    uint8_t            num_sv;         /* satellites used     */
    uint8_t            valid_flags;    /* NAV-PVT valid byte  */
    uint16_t           year;
    uint8_t            month, day, hour, min, sec;

    /* Convenience floats for EKF / telemetry */
    float              lat_deg;
    float              lon_deg;
    float              alt_msl_m;
    float              vel_d_m_s;      /* positive = downward */

    /* Status */
    bool               alive;          /* I2C ACK received during init */
    bool               has_fix;        /* fixType >= 2D               */
    volatile bool      data_ready;     /* EXTI stub (future use)      */
    uint32_t           last_pvt_tick;  /* HAL_GetTick of last NAV-PVT */
    uint32_t           pvt_count;      /* total NAV-PVT messages      */

    /* UBX frame parser */
    ubx_parse_state_t  parse_state;
    uint8_t            parse_class;
    uint8_t            parse_id;
    uint16_t           parse_len;
    uint16_t           parse_idx;
    uint8_t            parse_ck_a;
    uint8_t            parse_ck_b;
    uint8_t            parse_buf[100]; /* >= UBX_NAV_PVT_LEN (92) */

    /* Non-blocking tick state */
    gps_tick_state_t   tick_state;
    uint32_t           tick_last_poll;
    uint16_t           bytes_avail;

    /* ACK tracking (init only) */
    uint8_t            ack_class;
    uint8_t            ack_id;
    bool               ack_received;
    bool               nak_received;
} max_m10m_t;

/* ── Public API ─────────────────────────────────────────────────── */

/**
 * Hard-reset GPS, verify I2C communication, configure 10 Hz GPS-only UBX output.
 * Returns true if I2C ACK received (module alive). Does NOT require antenna.
 */
bool max_m10m_init(max_m10m_t *dev, I2C_HandleTypeDef *hi2c,
                   GPIO_TypeDef *nrst_port, uint16_t nrst_pin);

/**
 * Non-blocking tick — call every main loop iteration.
 * Polls GPS for data, feeds bytes through UBX parser.
 * Returns 1 when a new NAV-PVT has been parsed, 0 otherwise.
 */
int max_m10m_tick(max_m10m_t *dev);

/**
 * Check if GPS has a valid 3D fix.
 */
bool max_m10m_has_3d_fix(const max_m10m_t *dev);

/**
 * Call from HAL_GPIO_EXTI_Callback when I2C1_INT (PE0) fires.
 */
void max_m10m_irq_handler(max_m10m_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* MAX_M10M_H */
