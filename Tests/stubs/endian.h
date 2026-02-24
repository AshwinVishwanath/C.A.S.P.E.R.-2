/* Stub for App/util/endian.h — matches firmware's actual API */
#ifndef APP_UTIL_ENDIAN_H
#define APP_UTIL_ENDIAN_H

#include <stdint.h>
#include <string.h>

/* ── Little-endian serialisation helpers (firmware API) ── */

static inline void put_le16(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

static inline void put_le32(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
    dst[2] = (uint8_t)((val >> 16) & 0xFF);
    dst[3] = (uint8_t)((val >> 24) & 0xFF);
}

static inline uint16_t get_le16(const uint8_t *src)
{
    return (uint16_t)(src[0] | ((uint16_t)src[1] << 8));
}

static inline uint32_t get_le32(const uint8_t *src)
{
    return (uint32_t)(src[0] | ((uint32_t)src[1] << 8) |
                      ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24));
}

/* ── Aliases for tests that use read/write naming ── */
#define write_le16  put_le16
#define write_le32  put_le32
#define read_le16   get_le16
#define read_le32   get_le32

static inline void write_le_float(uint8_t *buf, float val) {
    uint32_t u;
    memcpy(&u, &val, sizeof(u));
    put_le32(buf, u);
}

static inline float read_le_float(const uint8_t *buf) {
    uint32_t u = get_le32(buf);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

#endif /* APP_UTIL_ENDIAN_H */
