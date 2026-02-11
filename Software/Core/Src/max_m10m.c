/**
 * U-blox MAX-M10M GPS driver over I2C (DDC protocol).
 * Reference: u-blox M10 SPG 5.10 Interface Description (UBX-21035062)
 *
 * Configured for 10 Hz GPS-only navigation via UBX-NAV-PVT.
 * Non-blocking tick architecture — max_m10m_tick() polls the GPS
 * bytes-available register and reads data in 64-byte chunks, feeding
 * each byte through a UBX frame parser state machine.
 */

#include "max_m10m.h"
#include <string.h>

/* ================================================================== */
/* Internal helpers                                                    */
/* ================================================================== */

/**
 * Compute UBX Fletcher checksum (CK_A, CK_B) over a buffer.
 * Covers class + id + length + payload (NOT sync bytes).
 */
static void ubx_checksum(const uint8_t *buf, uint16_t len,
                          uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < len; i++) {
        a += buf[i];
        b += a;
    }
    *ck_a = a;
    *ck_b = b;
}

/**
 * Send a complete UBX message to the GPS over I2C.
 * Builds sync + header + payload + checksum into a stack buffer.
 */
static HAL_StatusTypeDef max_m10m_i2c_write_ubx(max_m10m_t *dev,
    uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t pay_len)
{
    uint8_t frame[64];
    if (pay_len + 8 > sizeof(frame)) return HAL_ERROR;

    frame[0] = UBX_SYNC_1;
    frame[1] = UBX_SYNC_2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = (uint8_t)(pay_len & 0xFF);
    frame[5] = (uint8_t)(pay_len >> 8);
    if (pay_len > 0)
        memcpy(&frame[6], payload, pay_len);

    uint8_t ck_a, ck_b;
    ubx_checksum(&frame[2], 4 + pay_len, &ck_a, &ck_b);
    frame[6 + pay_len] = ck_a;
    frame[7 + pay_len] = ck_b;

    return HAL_I2C_Master_Transmit(dev->hi2c,
        MAX_M10M_I2C_ADDR << 1, frame, 8 + pay_len, 100);
}

/**
 * Read the 2-byte bytes-available count from registers 0xFD-0xFE.
 * Returns the count, or 0 on I2C error.
 */
static uint16_t max_m10m_read_bytes_available(max_m10m_t *dev)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(dev->hi2c,
        MAX_M10M_I2C_ADDR << 1, MAX_M10M_REG_BYTES_HI,
        I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
    if (rc != HAL_OK) return 0;
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * Send UBX-CFG-VALSET for a single key-value pair.
 * val_size: 1 = U1/L, 2 = U2, 4 = U4.
 */
static HAL_StatusTypeDef max_m10m_send_valset(max_m10m_t *dev,
    uint32_t key, uint32_t value, uint8_t val_size)
{
    uint8_t payload[12]; /* 4 header + 4 key + up to 4 value */
    payload[0] = 0x01;   /* version */
    payload[1] = 0x01;   /* layers: RAM only */
    payload[2] = 0x00;   /* reserved */
    payload[3] = 0x00;   /* reserved */
    /* Key ID (little-endian) */
    payload[4] = (uint8_t)(key);
    payload[5] = (uint8_t)(key >> 8);
    payload[6] = (uint8_t)(key >> 16);
    payload[7] = (uint8_t)(key >> 24);
    /* Value (little-endian) */
    payload[8]  = (uint8_t)(value);
    if (val_size >= 2) payload[9]  = (uint8_t)(value >> 8);
    if (val_size >= 4) {
        payload[10] = (uint8_t)(value >> 16);
        payload[11] = (uint8_t)(value >> 24);
    }

    return max_m10m_i2c_write_ubx(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID,
                                   payload, 8 + val_size);
}

/* Forward declarations for parser */
static int  ubx_parse_byte(max_m10m_t *dev, uint8_t byte);
static void ubx_handle_frame(max_m10m_t *dev);

/**
 * Blocking wait for ACK/NAK during init.  Reads data from GPS and
 * feeds through parser until ACK/NAK is received or timeout.
 */
static bool max_m10m_wait_ack(max_m10m_t *dev, uint8_t cls, uint8_t id,
                               uint32_t timeout_ms)
{
    (void)cls; (void)id;
    dev->ack_received = false;
    dev->nak_received = false;

    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
        uint16_t avail = max_m10m_read_bytes_available(dev);
        if (avail == 0 || avail == 0xFFFF) {
            HAL_Delay(10);
            continue;
        }
        /* Read up to 64 bytes at a time */
        uint16_t chunk = avail > 64 ? 64 : avail;
        uint8_t buf[64];
        HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(dev->hi2c,
            MAX_M10M_I2C_ADDR << 1, MAX_M10M_REG_DATA_STREAM,
            I2C_MEMADD_SIZE_8BIT, buf, chunk, 50);
        if (rc != HAL_OK) continue;

        for (uint16_t i = 0; i < chunk; i++)
            ubx_parse_byte(dev, buf[i]);

        if (dev->ack_received || dev->nak_received)
            return dev->ack_received;
    }
    return false;
}

/* ================================================================== */
/* UBX frame parser                                                    */
/* ================================================================== */

/**
 * Feed one byte into the UBX frame parser state machine.
 * Returns 1 if a complete valid NAV-PVT was parsed, 0 otherwise.
 */
static int ubx_parse_byte(max_m10m_t *dev, uint8_t byte)
{
    switch (dev->parse_state) {
    case UBX_PARSE_SYNC1:
        if (byte == UBX_SYNC_1)
            dev->parse_state = UBX_PARSE_SYNC2;
        return 0;

    case UBX_PARSE_SYNC2:
        if (byte == UBX_SYNC_2) {
            dev->parse_state = UBX_PARSE_CLASS;
        } else if (byte != UBX_SYNC_1) {
            dev->parse_state = UBX_PARSE_SYNC1;
        }
        /* If byte == UBX_SYNC_1, stay in SYNC2 (could be new frame start) */
        return 0;

    case UBX_PARSE_CLASS:
        dev->parse_class = byte;
        dev->parse_ck_a = byte;
        dev->parse_ck_b = byte;
        dev->parse_state = UBX_PARSE_ID;
        return 0;

    case UBX_PARSE_ID:
        dev->parse_id = byte;
        dev->parse_ck_a += byte;
        dev->parse_ck_b += dev->parse_ck_a;
        dev->parse_state = UBX_PARSE_LEN1;
        return 0;

    case UBX_PARSE_LEN1:
        dev->parse_len = byte;
        dev->parse_ck_a += byte;
        dev->parse_ck_b += dev->parse_ck_a;
        dev->parse_state = UBX_PARSE_LEN2;
        return 0;

    case UBX_PARSE_LEN2:
        dev->parse_len |= (uint16_t)byte << 8;
        dev->parse_ck_a += byte;
        dev->parse_ck_b += dev->parse_ck_a;
        dev->parse_idx = 0;
        if (dev->parse_len > sizeof(dev->parse_buf)) {
            /* Frame too large — discard */
            dev->parse_state = UBX_PARSE_SYNC1;
        } else if (dev->parse_len == 0) {
            dev->parse_state = UBX_PARSE_CK_A;
        } else {
            dev->parse_state = UBX_PARSE_PAYLOAD;
        }
        return 0;

    case UBX_PARSE_PAYLOAD:
        dev->parse_buf[dev->parse_idx++] = byte;
        dev->parse_ck_a += byte;
        dev->parse_ck_b += dev->parse_ck_a;
        if (dev->parse_idx >= dev->parse_len)
            dev->parse_state = UBX_PARSE_CK_A;
        return 0;

    case UBX_PARSE_CK_A:
        if (byte == dev->parse_ck_a) {
            dev->parse_state = UBX_PARSE_CK_B;
        } else {
            dev->parse_state = UBX_PARSE_SYNC1;
        }
        return 0;

    case UBX_PARSE_CK_B:
        dev->parse_state = UBX_PARSE_SYNC1;
        if (byte == dev->parse_ck_b) {
            ubx_handle_frame(dev);
            /* Return 1 if it was a NAV-PVT */
            if (dev->parse_class == UBX_CLASS_NAV &&
                dev->parse_id == UBX_NAV_PVT_ID)
                return 1;
        }
        return 0;
    }

    dev->parse_state = UBX_PARSE_SYNC1;
    return 0;
}

/**
 * Helper: read a little-endian int32 from a byte buffer.
 */
static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/**
 * Helper: read a little-endian uint32 from a byte buffer.
 */
static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/**
 * Helper: read a little-endian uint16 from a byte buffer.
 */
static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/**
 * Dispatch a fully parsed and checksum-verified UBX frame.
 */
static void ubx_handle_frame(max_m10m_t *dev)
{
    if (dev->parse_class == UBX_CLASS_NAV &&
        dev->parse_id == UBX_NAV_PVT_ID &&
        dev->parse_len == UBX_NAV_PVT_LEN) {
        /* ── NAV-PVT ── */
        const uint8_t *p = dev->parse_buf;

        dev->iTOW        = read_u32_le(&p[0]);
        dev->year         = read_u16_le(&p[4]);
        dev->month        = p[6];
        dev->day          = p[7];
        dev->hour         = p[8];
        dev->min          = p[9];
        dev->sec          = p[10];
        dev->valid_flags  = p[11];
        dev->fix_type     = p[20];
        dev->num_sv       = p[23];
        dev->lon_deg7     = read_i32_le(&p[24]);
        dev->lat_deg7     = read_i32_le(&p[28]);
        dev->h_msl_mm     = read_i32_le(&p[36]);
        dev->h_acc_mm     = read_u32_le(&p[40]);
        dev->v_acc_mm     = read_u32_le(&p[44]);
        dev->vel_n_mm_s   = read_i32_le(&p[48]);
        dev->vel_e_mm_s   = read_i32_le(&p[52]);
        dev->vel_d_mm_s   = read_i32_le(&p[56]);
        dev->pDOP         = read_u16_le(&p[76]);

        /* Convenience floats */
        dev->lat_deg   = (float)dev->lat_deg7 * 1e-7f;
        dev->lon_deg   = (float)dev->lon_deg7 * 1e-7f;
        dev->alt_msl_m = (float)dev->h_msl_mm * 0.001f;
        dev->vel_d_m_s = (float)dev->vel_d_mm_s * 0.001f;

        dev->has_fix       = (dev->fix_type >= GPS_FIX_2D);
        dev->last_pvt_tick = HAL_GetTick();
        dev->pvt_count++;

    } else if (dev->parse_class == UBX_CLASS_ACK &&
               dev->parse_id == UBX_ACK_ACK_ID &&
               dev->parse_len >= 2) {
        /* ── ACK-ACK ── */
        dev->ack_class    = dev->parse_buf[0];
        dev->ack_id       = dev->parse_buf[1];
        dev->ack_received = true;

    } else if (dev->parse_class == UBX_CLASS_ACK &&
               dev->parse_id == UBX_ACK_NAK_ID &&
               dev->parse_len >= 2) {
        /* ── ACK-NAK ── */
        dev->ack_class    = dev->parse_buf[0];
        dev->ack_id       = dev->parse_buf[1];
        dev->nak_received = true;
    }
    /* All other messages silently discarded */
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

bool max_m10m_init(max_m10m_t *dev, I2C_HandleTypeDef *hi2c,
                   GPIO_TypeDef *nrst_port, uint16_t nrst_pin)
{
    memset(dev, 0, sizeof(*dev));
    dev->hi2c      = hi2c;
    dev->nrst_port = nrst_port;
    dev->nrst_pin  = nrst_pin;
    dev->parse_state = UBX_PARSE_SYNC1;

    /* 1. Hard reset: NRST_GPS LOW 10ms, release, wait 1s for boot */
    HAL_GPIO_WritePin(nrst_port, nrst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(nrst_port, nrst_pin, GPIO_PIN_SET);
    HAL_Delay(1000);

    /* 2. I2C comms check */
    HAL_StatusTypeDef rc = HAL_I2C_IsDeviceReady(hi2c,
        MAX_M10M_I2C_ADDR << 1, 3, 100);
    if (rc != HAL_OK) {
        dev->alive = false;
        return false;
    }
    dev->alive = true;

    /* 3. Disable all NMEA output on I2C port */
    max_m10m_send_valset(dev, CFG_MSGOUT_NMEA_GGA_I2C, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_MSGOUT_NMEA_GLL_I2C, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_MSGOUT_NMEA_GSA_I2C, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_MSGOUT_NMEA_GSV_I2C, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_MSGOUT_NMEA_RMC_I2C, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_MSGOUT_NMEA_VTG_I2C, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    /* 4. Enable UBX-NAV-PVT on I2C at every measurement epoch */
    max_m10m_send_valset(dev, CFG_MSGOUT_UBX_NAV_PVT_I2C, 1, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    /* 5. Set measurement rate to 100ms (10 Hz) */
    max_m10m_send_valset(dev, CFG_RATE_MEAS, 100, 2);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    /* 6. GPS-only constellation (required for 10 Hz) */
    max_m10m_send_valset(dev, CFG_SIGNAL_GPS_ENA, 1, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_SIGNAL_GAL_ENA, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_SIGNAL_BDS_ENA, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_SIGNAL_GLO_ENA, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    max_m10m_send_valset(dev, CFG_SIGNAL_QZSS_ENA, 0, 1);
    max_m10m_wait_ack(dev, UBX_CLASS_CFG, UBX_CFG_VALSET_ID, 200);

    return true;
}

int max_m10m_tick(max_m10m_t *dev)
{
    if (!dev->alive) return 0;

    uint32_t now = HAL_GetTick();

    switch (dev->tick_state) {
    case GPS_TICK_IDLE:
        /* Poll every 25ms (40 Hz) — timely for 10 Hz NAV-PVT */
        if (now - dev->tick_last_poll < 25) return 0;
        dev->tick_last_poll = now;
        dev->tick_state = GPS_TICK_READ_AVAIL;
        /* fall through */

    case GPS_TICK_READ_AVAIL: {
        uint16_t avail = max_m10m_read_bytes_available(dev);
        if (avail == 0 || avail == 0xFFFF) {
            dev->tick_state = GPS_TICK_IDLE;
            return 0;
        }
        dev->bytes_avail = avail;
        dev->tick_state = GPS_TICK_READ_DATA;
    }
        /* fall through */

    case GPS_TICK_READ_DATA: {
        /* Read up to 64 bytes per tick to limit blocking time */
        uint16_t chunk = dev->bytes_avail;
        if (chunk > 64) chunk = 64;

        uint8_t buf[64];
        HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(dev->hi2c,
            MAX_M10M_I2C_ADDR << 1, MAX_M10M_REG_DATA_STREAM,
            I2C_MEMADD_SIZE_8BIT, buf, chunk, 50);

        if (rc != HAL_OK) {
            dev->tick_state = GPS_TICK_IDLE;
            return 0;
        }

        int got_pvt = 0;
        for (uint16_t i = 0; i < chunk; i++) {
            if (ubx_parse_byte(dev, buf[i]))
                got_pvt = 1;
        }

        dev->bytes_avail -= chunk;
        if (dev->bytes_avail == 0)
            dev->tick_state = GPS_TICK_IDLE;
        /* else remain in GPS_TICK_READ_DATA for next tick call */

        return got_pvt;
    }
    }

    return 0;
}

bool max_m10m_has_3d_fix(const max_m10m_t *dev)
{
    return dev->has_fix && (dev->fix_type >= GPS_FIX_3D);
}

void max_m10m_irq_handler(max_m10m_t *dev)
{
    dev->data_ready = true;
}
