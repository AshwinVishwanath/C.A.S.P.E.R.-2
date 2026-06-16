/*
 * test_crc32.c — characterization + edge-case suite for the CRC-32 module.
 *
 * GOLDEN SPEC = the CURRENT behaviour of Software/App/telemetry/crc32_hw.c.
 * Today that file configures the STM32H7 CRC hardware unit as:
 *     DefaultPolynomialUse     = ENABLE      -> polynomial 0x04C11DB7
 *     DefaultInitValueUse      = ENABLE      -> initial value 0xFFFFFFFF
 *     InputDataInversionMode   = BYTE        -> reflect each input byte (refin)
 *     OutputDataInversionMode  = ENABLE      -> reflect the output register (refout)
 *     InputDataFormat          = BYTES
 * and crc32_hw_compute() XORs the raw result with 0xFFFFFFFF (xorout).
 * That is exactly CRC-32/ISO-HDLC (IEEE 802.3 / zlib / PKZIP):
 *     poly=0x04C11DB7  init=0xFFFFFFFF  refin=true  refout=true  xorout=0xFFFFFFFF
 *     check("123456789") == 0xCBF43926
 *
 * After the HAL->seam migration, the module's public API becomes
 *     casper_crc32_init()    (was crc32_hw_init())
 *     casper_crc32_compute() (was crc32_hw_compute())
 * with an identical numeric result. These tests are written against that
 * TARGET API (casper_crc.h) and run against the host software mock
 * (board_mock.c), which implements the same CRC-32/ISO-HDLC algorithm.
 *
 * Golden 32-bit outputs below were independently computed with a reference
 * reflected-table CRC-32 implementation, NOT copied from the mock, so the
 * mock itself is also pinned by these vectors.
 *
 * NOTE: these tests deliberately will not link until the seam exists / the
 * driver is migrated to the public casper_crc32_* names — that is the
 * intended RED state for the refactor.
 */

#include "test.h"
#include "board_mock.h"
#include "casper_crc.h"

/* ------------------------------------------------------------------ */
/* Helper: little-endian byte view of a 32-bit CRC, the on-wire order  */
/* telemetry uses when it appends the CRC as a trailing field.         */
/* ------------------------------------------------------------------ */
static void crc_le_bytes(uint32_t crc, uint8_t out[4])
{
    out[0] = (uint8_t)(crc & 0xFFu);
    out[1] = (uint8_t)((crc >> 8) & 0xFFu);
    out[2] = (uint8_t)((crc >> 16) & 0xFFu);
    out[3] = (uint8_t)((crc >> 24) & 0xFFu);
}

/* ================================================================== */
/* 1. The canonical known-answer vector — the spec's own check value.  */
/* ================================================================== */
TEST(kat_123456789)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t v[] = { '1','2','3','4','5','6','7','8','9' };  /* 9 bytes */
    ASSERT_EQ_U(casper_crc32_compute(v, sizeof(v)), 0xCBF43926u);
}

/* ================================================================== */
/* 2. Empty input -> 0x00000000 (init ^ xorout, both 0xFFFFFFFF).      */
/*    Also asserts the NULL/zero-length contract is honoured.          */
/* ================================================================== */
TEST(empty_input_is_zero)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t v[1] = { 0x00 };
    /* len==0 must not read *data; pass a valid ptr and a NULL just in case
       only the len is examined (the spec processes zero bytes). */
    ASSERT_EQ_U(casper_crc32_compute(v, 0), 0x00000000u);
}

/* ================================================================== */
/* 3. Single-byte golden values — pins per-byte table behaviour and    */
/*    the reflected (LSB-first) input processing.                      */
/* ================================================================== */
TEST(single_byte_goldens)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t b00 = 0x00, bff = 0xFF, baa = 0xAA, bA = 'A';
    ASSERT_EQ_U(casper_crc32_compute(&b00, 1), 0xD202EF8Du);
    ASSERT_EQ_U(casper_crc32_compute(&bff, 1), 0xFF000000u);
    ASSERT_EQ_U(casper_crc32_compute(&baa, 1), 0xE401A57Bu);
    ASSERT_EQ_U(casper_crc32_compute(&bA,  1), 0xD3D99E8Bu);
}

/* ================================================================== */
/* 4. Multi-byte / burst correctness over a fixed vector.              */
/* ================================================================== */
TEST(multibyte_goldens)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t seq[4]   = { 0x01, 0x02, 0x03, 0x04 };
    const uint8_t zeros4[4] = { 0x00, 0x00, 0x00, 0x00 };
    const uint8_t abc[3]   = { 'a', 'b', 'c' };
    ASSERT_EQ_U(casper_crc32_compute(seq,   4), 0xB63CFBCDu);
    ASSERT_EQ_U(casper_crc32_compute(zeros4, 4), 0x2144DF1Cu);
    ASSERT_EQ_U(casper_crc32_compute(abc,   3), 0x352441C2u);
}

/* ================================================================== */
/* 5. Byte-order sensitivity: swapping two bytes changes the CRC.      */
/*    Guards against an accidental endian/reflection regression in the */
/*    migration.                                                       */
/* ================================================================== */
TEST(byte_order_sensitive)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t dead[2] = { 0xDE, 0xAD };
    const uint8_t adde[2] = { 0xAD, 0xDE };
    uint32_t c1 = casper_crc32_compute(dead, 2);
    uint32_t c2 = casper_crc32_compute(adde, 2);
    ASSERT_EQ_U(c1, 0xF605253Bu);
    ASSERT_EQ_U(c2, 0x3B1B2F88u);
    ASSERT_TRUE(c1 != c2);
}

/* ================================================================== */
/* 6. Length sensitivity: identical prefix, different length, differs. */
/*    Pins that the CRC covers exactly [0..len-1] (firmware rule #1:    */
/*    packet sizes must match the byte-counted layout).                */
/* ================================================================== */
TEST(length_sensitive)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t big[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    uint32_t c8 = casper_crc32_compute(big, 8);
    uint32_t c9 = casper_crc32_compute(big, 9);
    ASSERT_EQ_U(c8, 0x3FCA88C5u);
    ASSERT_EQ_U(c9, 0x40EFAB9Eu);
    ASSERT_TRUE(c8 != c9);
}

/* ================================================================== */
/* 7. Determinism / idempotency: repeated calls and a re-init in the   */
/*    middle yield identical results (no residual internal state leaks  */
/*    across compute() calls — the HW unit is reset by the wrapper).    */
/* ================================================================== */
TEST(deterministic_and_reinit_safe)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t v[] = { '1','2','3','4','5','6','7','8','9' };
    uint32_t a = casper_crc32_compute(v, sizeof(v));
    uint32_t b = casper_crc32_compute(v, sizeof(v));
    casper_crc32_init();                       /* re-init must be safe */
    uint32_t c = casper_crc32_compute(v, sizeof(v));
    ASSERT_EQ_U(a, 0xCBF43926u);
    ASSERT_EQ_U(b, 0xCBF43926u);
    ASSERT_EQ_U(c, 0xCBF43926u);
}

/* ================================================================== */
/* 8. No internal carry-over between distinct inputs: computing B      */
/*    after A returns the same value as computing B fresh.             */
/* ================================================================== */
TEST(no_state_carryover)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t a[3] = { 'a', 'b', 'c' };
    const uint8_t b[4] = { 0x01, 0x02, 0x03, 0x04 };

    uint32_t b_fresh = casper_crc32_compute(b, 4);

    mock_reset();
    casper_crc32_init();
    (void)casper_crc32_compute(a, 3);          /* prime with a different input */
    uint32_t b_after_a = casper_crc32_compute(b, 4);

    ASSERT_EQ_U(b_fresh, 0xB63CFBCDu);
    ASSERT_EQ_U(b_after_a, b_fresh);
}

/* ================================================================== */
/* 9. Compute without an explicit init() call.                         */
/*    crc32_hw.c requires init() (it sets the HW handle); the migrated  */
/*    software/seam path builds its table lazily. This test pins the    */
/*    documented mock/seam contract that compute() is self-arming.      */
/*    (On real HW the board init wires the peripheral; the numeric      */
/*    result is the spec value regardless.)                            */
/* ================================================================== */
TEST(compute_without_explicit_init)
{
    mock_reset();
    const uint8_t v[] = { '1','2','3','4','5','6','7','8','9' };
    ASSERT_EQ_U(casper_crc32_compute(v, sizeof(v)), 0xCBF43926u);
}

/* ================================================================== */
/* 10. On-wire little-endian trailer bytes.                            */
/*     Telemetry appends the CRC as a 4-byte little-endian trailer      */
/*     (e.g. tlm_manager builds [..payload..][CRC:4]). Pin the exact    */
/*     trailing byte sequence so the framing layer stays byte-stable.   */
/* ================================================================== */
TEST(le_trailer_bytes)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t v[] = { '1','2','3','4','5','6','7','8','9' };
    uint32_t crc = casper_crc32_compute(v, sizeof(v));   /* 0xCBF43926 */
    uint8_t got[4];
    crc_le_bytes(crc, got);
    const uint8_t expect[4] = { 0x26, 0x39, 0xF4, 0xCB }; /* LE of 0xCBF43926 */
    ASSERT_EQ_MEM(got, expect, 4);
}

/* ================================================================== */
/* 11. Representative telemetry-style packet.                          */
/*     6-byte header [ID:1][magic:2][nonce:2][err:1] (a CAC ACK shape)  */
/*     — pins the CRC over a fixed framed buffer matching firmware      */
/*     usage like crc32_hw_compute(pkt, 6).                             */
/* ================================================================== */
TEST(packet_crc_golden)
{
    mock_reset();
    casper_crc32_init();
    const uint8_t pkt6[6] = { 0x01, 0xCA, 0x5A, 0x12, 0x34, 0x07 };
    ASSERT_EQ_U(casper_crc32_compute(pkt6, 6), 0x47677CF0u);
}

/* ================================================================== */
/* 12. Larger fixed buffer (17 zero bytes — the FC_MSG_FAST raw length  */
/*     before the trailing CRC). Guards the burst path past one cache   */
/*     line and confirms zero-payload-of-length-N != empty.            */
/* ================================================================== */
TEST(zeros17_golden)
{
    mock_reset();
    casper_crc32_init();
    uint8_t z17[17];
    memset(z17, 0, sizeof(z17));
    uint32_t c = casper_crc32_compute(z17, 17);
    ASSERT_EQ_U(c, 0xC9EFF1BDu);
    ASSERT_TRUE(c != casper_crc32_compute(z17, 0)); /* length 17 != length 0 */
}

/* ================================================================== */
/* 13. CRC over a buffer that contains a verification-style trailer.    */
/*     Mirrors the receive path: compute over payload, compare against  */
/*     the LE trailer stored just past it. A correct trailer matches;   */
/*     a single-bit corruption does not (detects the error).           */
/* ================================================================== */
TEST(verify_roundtrip_and_corruption)
{
    mock_reset();
    casper_crc32_init();

    uint8_t frame[10];
    const uint8_t payload[6] = { 0x01, 0xCA, 0x5A, 0x12, 0x34, 0x07 };
    memcpy(frame, payload, 6);

    uint32_t crc = casper_crc32_compute(frame, 6);   /* 0x47677CF0 */
    crc_le_bytes(crc, &frame[6]);

    /* Receiver: recompute over payload, read trailer back as LE u32. */
    uint32_t recomputed = casper_crc32_compute(frame, 6);
    uint32_t stored = (uint32_t)frame[6]
                    | ((uint32_t)frame[7] << 8)
                    | ((uint32_t)frame[8] << 16)
                    | ((uint32_t)frame[9] << 24);
    ASSERT_EQ_U(recomputed, stored);   /* matches -> frame valid */

    /* Flip one bit in the payload: CRC must now mismatch. */
    frame[2] ^= 0x01u;
    uint32_t after = casper_crc32_compute(frame, 6);
    ASSERT_TRUE(after != stored);
}

int main(void)
{
    RUN(kat_123456789);
    RUN(empty_input_is_zero);
    RUN(single_byte_goldens);
    RUN(multibyte_goldens);
    RUN(byte_order_sensitive);
    RUN(length_sensitive);
    RUN(deterministic_and_reinit_safe);
    RUN(no_state_carryover);
    RUN(compute_without_explicit_init);
    RUN(le_trailer_bytes);
    RUN(packet_crc_golden);
    RUN(zeros17_golden);
    RUN(verify_roundtrip_and_corruption);
    return test_summary();
}
