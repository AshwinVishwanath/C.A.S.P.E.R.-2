#ifndef APP_UTIL_ENDIAN_H
#define APP_UTIL_ENDIAN_H

#include <stdint.h>

/* ── Little-endian serialisation helpers ──────────────────────── */

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

#endif /* APP_UTIL_ENDIAN_H */
