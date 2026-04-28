/* ============================================================
 *  TIER:     CORE-FLIGHT
 *  MODULE:   Hamming SECDED
 *  SUMMARY:  Single-error-correct/double-error-detect for log records.
 * ============================================================ */
#include "hamming.h"

/* ═══════════════════════════════════════════════════════════════════════
 *  Hamming SECDED — whole-record encoder/decoder
 *
 *  For data_len bytes = data_len*8 data bits, we need P parity bits
 *  where 2^P >= data_len*8 + P + 1.
 *
 *  For 60 bytes (480 bits): P=9 gives 512 >= 490. Plus 1 overall
 *  parity for SECDED = 10 bits total, packed into a uint16_t.
 *
 *  Bit numbering: position 1..N (1-indexed). Parity bit i occupies
 *  position 2^i. Data bits fill the remaining positions.
 *
 *  Parity bit i covers all positions where bit i of the position
 *  number is set. The overall parity bit (bit 9) is XOR of all data
 *  bits and all syndrome parity bits.
 * ═══════════════════════════════════════════════════════════════════════ */

/* Number of syndrome bits (not counting overall parity) */
#define PARITY_BITS  9

/* Get bit `bit` (0-indexed) from data array */
static inline int get_data_bit(const uint8_t *data, size_t bit)
{
    return (data[bit >> 3] >> (bit & 7)) & 1;
}

/* Flip bit `bit` (0-indexed) in data array */
static inline void flip_data_bit(uint8_t *data, size_t bit)
{
    data[bit >> 3] ^= (1u << (bit & 7));
}

/*
 * Map a 1-indexed Hamming position to a data bit index.
 * Positions that are powers of 2 are parity bits (return -1).
 * Other positions map to data bits in order.
 *
 * We pre-compute the mapping lazily: for a given position pos (1-based),
 * skip the parity positions (1, 2, 4, 8, 16, ...) and count the
 * remaining data-bit positions sequentially.
 */
static int pos_is_parity(uint16_t pos)
{
    return (pos & (pos - 1)) == 0;  /* true if pos is a power of 2 */
}

/*
 * For a given data bit index (0-based), compute the Hamming position
 * (1-based) that it occupies. Data bits fill non-power-of-2 positions.
 */
static uint16_t data_bit_to_pos(size_t data_idx)
{
    uint16_t pos = 1;
    size_t count = 0;

    while (1) {
        if (!pos_is_parity(pos)) {
            if (count == data_idx)
                return pos;
            count++;
        }
        pos++;
    }
}

uint16_t hamming_encode(const uint8_t *data, size_t data_len)
{
    size_t total_data_bits = data_len * 8;
    uint16_t syndrome = 0;
    uint16_t overall = 0;

    for (size_t i = 0; i < total_data_bits; i++) {
        int bit = get_data_bit(data, i);
        if (bit) {
            uint16_t pos = data_bit_to_pos(i);
            syndrome ^= pos;
            overall ^= 1;
        }
    }

    /* syndrome holds the 9-bit parity (bits 0..8) */
    /* overall parity = XOR of all data bits and all syndrome bits */
    uint16_t syn_parity = 0;
    for (int i = 0; i < PARITY_BITS; i++)
        syn_parity ^= (syndrome >> i) & 1;
    overall ^= syn_parity;

    /* Pack: bits [8:0] = syndrome, bit [9] = overall parity */
    return (uint16_t)((syndrome & 0x1FF) | ((overall & 1) << 9));
}

int hamming_decode(uint8_t *data, size_t data_len, uint16_t parity)
{
    /* Recompute syndrome from current data */
    size_t total_data_bits = data_len * 8;
    uint16_t recomputed_syn = 0;
    uint16_t all_data_parity = 0;

    for (size_t i = 0; i < total_data_bits; i++) {
        int bit = get_data_bit(data, i);
        if (bit) {
            uint16_t pos = data_bit_to_pos(i);
            recomputed_syn ^= pos;
            all_data_parity ^= 1;
        }
    }

    /* Extract stored values */
    uint16_t stored_syn = parity & 0x1FF;
    uint16_t stored_overall = (parity >> 9) & 1;

    /* XOR syndromes to find error position */
    uint16_t error_syn = recomputed_syn ^ stored_syn;

    /* Recompute overall parity including the syndrome bits */
    uint16_t syn_parity = 0;
    for (int i = 0; i < PARITY_BITS; i++)
        syn_parity ^= (stored_syn >> i) & 1;
    uint16_t expected_overall = all_data_parity ^ syn_parity;
    uint16_t overall_err = expected_overall ^ stored_overall;

    if (error_syn == 0 && overall_err == 0) {
        return 0;  /* No error */
    }

    if (error_syn != 0 && overall_err != 0) {
        /* Single-bit error at position error_syn */
        /* Check if it's a parity bit position or data bit position */
        if (pos_is_parity(error_syn)) {
            /* Error in a parity bit — data is fine, just parity was wrong */
            return 1;
        }
        /* Map position back to data bit index and flip it */
        uint16_t pos = 1;
        size_t data_idx = 0;
        while (pos < error_syn) {
            if (!pos_is_parity(pos))
                data_idx++;
            pos++;
        }
        if (data_idx < total_data_bits) {
            flip_data_bit(data, data_idx);
        }
        return 1;  /* Single-bit corrected */
    }

    if (error_syn == 0 && overall_err != 0) {
        /* Error in overall parity bit only */
        return 1;
    }

    /* error_syn != 0 && overall_err == 0: double-bit error */
    return -1;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  CRC16-CCITT (polynomial 0x1021, init 0xFFFF)
 * ═══════════════════════════════════════════════════════════════════════ */

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}
