"""COBS framing + CRC32 — byte-for-byte compatible with the firmware.

Firmware references:
  Software/App/telemetry/cobs.c     — encoder/decoder
  Software/App/telemetry/crc32_hw.c — STM32 HW CRC, configured for
                                      standard CRC-32 (reflected I/O,
                                      init=0xFFFFFFFF, xor=0xFFFFFFFF,
                                      poly=0x04C11DB7) which matches
                                      Python's binascii.crc32 exactly.
"""
from __future__ import annotations

import binascii


def cobs_encode(data: bytes) -> bytes:
    """Consistent Overhead Byte Stuffing — appends a 0x00 delimiter.

    Matches `cobs_encode` in cobs.c. The delimiter is what the firmware
    looks for to terminate a frame in `cmd_router_process`.
    """
    out = bytearray()
    idx = 0
    while idx < len(data):
        block_start = idx
        while idx < len(data) and data[idx] != 0x00 and (idx - block_start) < 254:
            idx += 1
        code = idx - block_start + 1
        out.append(code)
        out.extend(data[block_start:idx])
        if idx < len(data) and data[idx] == 0x00:
            idx += 1
    return bytes(out) + b"\x00"


def cobs_decode(frame: bytes) -> bytes | None:
    """Decode a single COBS frame (no trailing 0x00). Returns None on error."""
    out = bytearray()
    i = 0
    n = len(frame)
    while i < n:
        code = frame[i]
        if code == 0:
            return None
        i += 1
        for _ in range(code - 1):
            if i >= n:
                return None
            out.append(frame[i])
            i += 1
        if code < 0xFF and i < n:
            out.append(0x00)
    return bytes(out)


def crc32(data: bytes) -> int:
    """Standard CRC-32 (matches firmware crc32_hw_compute)."""
    return binascii.crc32(data) & 0xFFFFFFFF


class CobsRxAccumulator:
    """Stream-decoder that splits a CDC byte stream into COBS frames.

    Feed bytes via `feed()`; complete frames come out of `frames()` as
    decoded payloads (zero-delimiter stripped, CRC NOT yet checked —
    that's the protocol layer's job).
    """

    def __init__(self, max_frame: int = 64) -> None:
        self._buf = bytearray()
        self._max = max_frame
        self._frames: list[bytes] = []

    def feed(self, chunk: bytes) -> None:
        for b in chunk:
            if b == 0x00:
                if self._buf:
                    decoded = cobs_decode(bytes(self._buf))
                    if decoded is not None:
                        self._frames.append(decoded)
                self._buf.clear()
            else:
                if len(self._buf) < self._max:
                    self._buf.append(b)
                else:
                    # Frame too long — drop and resync at next delimiter.
                    self._buf.clear()

    def frames(self) -> list[bytes]:
        out, self._frames = self._frames, []
        return out
