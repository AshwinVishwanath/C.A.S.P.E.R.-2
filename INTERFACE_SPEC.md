# C.A.S.P.E.R. 2 Mission Control — Interface Specification

**Protocol Version:** 5
**CRC Standard:** CRC-32/ISO-HDLC
**Transport:** COBS-framed USB serial (115200 baud)
**Byte Order:** Little-endian throughout
**Last Updated:** 2026-02-16

This document specifies every interface between Mission Control (MC), the Flight Computer (FC), and the future Ground Station (GS). It serves as the authoritative contract for firmware and software development.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Transport Layer](#2-transport-layer)
3. [CRC-32 Specification](#3-crc-32-specification)
4. [Message Format & Dispatch](#4-message-format--dispatch)
5. [FC Telemetry Messages (0x01–0x03)](#5-fc-telemetry-messages-0x010x03)
6. [GS Relay Messages (0x10–0x14)](#6-gs-relay-messages-0x100x14)
7. [Command Messages (MC → FC)](#7-command-messages-mc--fc)
8. [Response Messages (FC → MC)](#8-response-messages-fc--mc)
9. [Handshake & System Messages](#9-handshake--system-messages)
10. [CAC State Machine](#10-cac-state-machine)
11. [Telemetry Store](#11-telemetry-store)
12. [IPC Channels & Handlers](#12-ipc-channels--handlers)
13. [Preload Bridge API (`window.casper`)](#13-preload-bridge-api-windowcasper)
14. [Frontend Hooks & UI](#14-frontend-hooks--ui)
15. [Flight Configuration Format](#15-flight-configuration-format)
16. [Recovery Pipeline](#16-recovery-pipeline)
17. [Derived Computations](#17-derived-computations)
18. [Appendices](#18-appendices)

---

## 1. Architecture Overview

```
┌─────────────┐    USB/COBS     ┌──────────────────────────────────────────────┐
│ Flight      │◄───115200───────│  Mission Control (Electron)                  │
│ Computer    │    0x00 delim   │                                              │
│ (STM32)     │                 │  ┌─────────┐  ┌───────┐  ┌──────┐  ┌──────┐│
│             │                 │  │ FcUsb   │→│Parser │→│Store │→│ IPC  ││
│ msg_id at   │                 │  │ COBS    │  │       │  │      │  │      ││
│ byte[0]     │                 │  │ decode  │  │       │  │      │  │      ││
└─────────────┘                 │  └─────────┘  └───────┘  └──────┘  └──┬───┘│
                                │                    ↑                    │    │
┌─────────────┐    USB/COBS     │  ┌─────────┐      │      ┌──────┐     │    │
│ Ground      │◄───115200───────│  │ GsUsb   │──────┘      │ CAC  │←────┘    │
│ Station     │    0x00 delim   │  │ COBS    │             │Machine│          │
│ (future)    │                 │  │ decode  │             └──────┘          │
└─────────────┘                 │  └─────────┘                               │
                                │                         ┌──────────────┐   │
                                │  Preload Bridge ────────│  Renderer    │   │
                                │  (window.casper)        │  React UI   │   │
                                │                         └──────────────┘   │
                                └──────────────────────────────────────────────┘
```

**Connection Modes:**
- **FC Direct:** MC ↔ FC over USB. Telemetry arrives as 0x01/0x02/0x03 packets. Commands sent directly.
- **GS Relay:** MC ↔ GS over USB. GS relays FC telemetry as 0x10–0x14 packets via LoRa. Commands relayed through GS.
- Both modes can be active simultaneously. The CAC machine prefers GS if connected, falls back to FC.

---

## 2. Transport Layer

### 2.1 COBS Framing

Both FC and GS USB links use COBS (Consistent Overhead Byte Stuffing) framing.

| Parameter | Value |
|---|---|
| Frame delimiter | `0x00` |
| Max block length | 254 data bytes + 1 overhead byte |
| Worst-case overhead | `ceil(N / 254) + 1` bytes for N-byte payload |
| Empty payload encodes to | `[0x01]` |

**Wire format:** `[COBS-encoded payload bytes] [0x00]`

The `0x00` delimiter byte never appears inside a COBS-encoded payload. On receive, bytes accumulate until `0x00` is seen, then the accumulated bytes are COBS-decoded to recover the original payload. Back-to-back `0x00` delimiters are silently ignored. Malformed frames (decode failure) are silently discarded.

### 2.2 Serial Port Configuration

| Parameter | FC USB | GS USB |
|---|---|---|
| Baud rate | 115200 (default) | 115200 (default) |
| Data bits | 8 | 8 |
| Stop bits | 1 | 1 |
| Parity | None | None |
| Flow control | None | None |
| RX buffer guard | 65536 bytes | 65536 bytes |

### 2.3 Connection Lifecycle

1. **Scan:** `SerialPort.list()` enumerates available USB ports (path, VID, PID, manufacturer)
2. **Connect:** Open serial port at specified path + baud rate
3. **Handshake (FC only):** Send `[0xC0]` immediately after connect, await handshake response
4. **Data flow:** Bidirectional COBS-framed packets
5. **Disconnect:** Close port, discard partial RX buffer, mark connection as down

### 2.4 Events Emitted by Transport

| Event | Payload | When |
|---|---|---|
| `'frame'` | `Uint8Array` (decoded payload) | Valid COBS frame received |
| `'error'` | `Error` | Serial port error (port disconnect, hardware fault) |
| `'close'` | — | Port closed |

---

## 3. CRC-32 Specification

**Standard:** CRC-32/ISO-HDLC (same as Ethernet, ZIP, PNG)

| Parameter | Value |
|---|---|
| Polynomial (normal) | `0x04C11DB7` |
| Polynomial (reflected) | `0xEDB88320` |
| Initial value | `0xFFFFFFFF` |
| Reflect input | Yes |
| Reflect output | Yes |
| Final XOR | `0xFFFFFFFF` |
| Test vector | `CRC32("123456789") = 0xCBF43926` |
| Processing | Byte-by-byte with 256-entry lookup table |

**Computation (pseudocode):**
```
crc = 0xFFFFFFFF
for each byte b in data:
    crc = TABLE[(crc XOR b) AND 0xFF] XOR (crc >>> 8)
crc = crc XOR 0xFFFFFFFF
```

**Placement in packets:** CRC-32 is stored as the last 4 bytes of each packet, little-endian. CRC is computed over all preceding bytes (everything except the CRC field itself).

---

## 4. Message Format & Dispatch

Every decoded COBS payload has `msg_id` at byte `[0]`. The parser dispatches by this byte:

| msg_id | Hex | Direction | Name | Decoded Size |
|---|---|---|---|---|
| 1 | `0x01` | FC → MC | FC_MSG_FAST | 20 bytes |
| 2 | `0x02` | FC → MC | FC_MSG_GPS | 17 bytes |
| 3 | `0x03` | FC → MC | FC_MSG_EVENT | 11 bytes |
| 16 | `0x10` | GS → MC | GS_MSG_TELEM | 39 bytes |
| 17 | `0x11` | GS → MC | GS_MSG_GPS | variable |
| 18 | `0x12` | GS → MC | GS_MSG_EVENT | variable |
| 19 | `0x13` | GS → MC | GS_MSG_STATUS | variable |
| 20 | `0x14` | GS → MC | GS_MSG_CORRUPT | variable |
| 128 | `0x80` | MC → FC | CMD_ARM | 12 bytes |
| 129 | `0x81` | MC → FC | CMD_FIRE | 13 bytes |
| 160 | `0xA0` | FC → MC | ACK_ARM | 12 bytes |
| 161 | `0xA1` | FC → MC | ACK_FIRE | 13 bytes |
| 163 | `0xA3` | FC → MC | ACK_CONFIG | 13 bytes |
| 192 | `0xC0` | Bidirectional | HANDSHAKE | 1 (req) / variable (resp) |
| 208 | `0xD0` | MC → FC | SIM_FLIGHT | 5 bytes |
| 224 | `0xE0` | FC → MC | NACK | 10 bytes |
| 240 | `0xF0` | MC → FC | CONFIRM | 9 bytes |
| 241 | `0xF1` | MC → FC | ABORT | 9 bytes |

**Magic bytes** (used in safety-critical commands): `MAGIC_1 = 0xCA`, `MAGIC_2 = 0x5A`

---

## 5. FC Telemetry Messages (0x01–0x03)

### 5.1 FC_MSG_FAST (0x01) — High-Rate Telemetry

**Size:** 20 bytes
**CRC coverage:** bytes [0–15], CRC at [16–19]

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x01` |
| 1–2 | status | u16 LE | — | bitmap | FC telemetry status (see §5.4) |
| 3–4 | altitude | u16 LE | × 10.0 | m | Altitude AGL |
| 5–6 | velocity | i16 LE | × 0.1 | m/s | Velocity (signed, +up) |
| 7–11 | quaternion | 5 bytes | — | — | Smallest-three packed (see §5.5) |
| 12–13 | flight_time | u16 LE | × 0.1 | s | Mission elapsed time |
| 14 | battery | u8 | 6.0 + raw × 0.012 | V | Battery voltage |
| 15 | seq | u8 | — | — | Rolling sequence counter |
| 16–19 | crc32 | u32 LE | — | — | CRC-32 over [0–15] |

### 5.2 FC_MSG_GPS (0x02) — GPS Position

**Size:** 17 bytes
**CRC coverage:** bytes [0–12], CRC at [13–16]

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x02` |
| 1–4 | dlat_mm | i32 LE | ÷ 1000 | m | Delta latitude from pad origin |
| 5–8 | dlon_mm | i32 LE | ÷ 1000 | m | Delta longitude from pad origin |
| 9–10 | alt_msl | u16 LE | × 10.0 | m | GPS altitude MSL |
| 11 | fix_type | u8 | — | — | 0=none, 2=2D, 3=3D |
| 12 | sat_count | u8 | — | — | Satellites in use |
| 13–16 | crc32 | u32 LE | — | — | CRC-32 over [0–12] |

**Range saturation:** If `dlat_mm` or `dlon_mm` equals `±0x7FFFFFFF`, the delta has overflowed.

### 5.3 FC_MSG_EVENT (0x03) — Discrete Event

**Size:** 11 bytes
**CRC coverage:** bytes [0–6], CRC at [7–10]

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x03` |
| 1 | event_type | u8 | — | — | Event code (see table below) |
| 2–3 | event_data | u16 LE | — | — | Event-specific payload |
| 4–5 | flight_time | u16 LE | × 0.1 | s | When event occurred |
| 6 | reserved | u8 | — | — | Unused |
| 7–10 | crc32 | u32 LE | — | — | CRC-32 over [0–6] |

**Event type codes:**

| Code | Name | event_data meaning |
|---|---|---|
| `0x01` | State | New FSM state value |
| `0x02` | Pyro | Channel (hi nibble) + duration (lo byte) |
| `0x03` | Apogee | Peak altitude in decametres (×10 → m) |
| `0x04` | Error | Error code |
| `0x05` | Origin | Satellite count at pad-origin lock |
| `0x06` | Burnout | Peak acceleration (mg) |
| `0x07` | Staging | Stage number |
| `0x08` | Arm | Channel (hi nibble) + arm/disarm (lo bit) |

### 5.4 FC Telemetry Status Bitmap (16-bit LE)

```
Byte 0 (LSB):
  Bit 0: CNT1 — Continuity channel 1
  Bit 1: CNT2 — Continuity channel 2
  Bit 2: CNT3 — Continuity channel 3
  Bit 3: CNT4 — Continuity channel 4
  Bit 4: ARM1 — Armed channel 1
  Bit 5: ARM2 — Armed channel 2
  Bit 6: ARM3 — Armed channel 3
  Bit 7: ARM4 — Armed channel 4

Byte 1 (MSB):
  Bits 1:0: Reserved
  Bit 2:    ERROR — System error flag
  Bit 3:    FIRED — Any pyro has fired
  Bits 7:4: FSM_STATE — Flight state (4-bit, see §5.6)
```

### 5.5 Quaternion Encoding (Smallest-Three, 5 bytes / 40 bits)

```
Byte 0:  [drop_idx:2][rsvd:2][A_hi:4]
Byte 1:  [A_lo:8]
Byte 2:  [B_hi:8]
Byte 3:  [B_lo:4][C_hi:4]
Byte 4:  [C_lo:8]
```

- `drop_idx` (2 bits): Index of dropped (largest) component. 0=w, 1=x, 2=y, 3=z
- Components A, B, C are 12-bit signed integers (two's complement)
- Scale factor: `QUAT_SCALE = 4096.0` (see ORIENTATION_SPEC.md §5 for ground truth)
- Decode: `component = raw_12bit / QUAT_SCALE`
- Reconstruct dropped: `dropped = √(1 − A² − B² − C²)` (always positive)
- Result: unit quaternion `[w, x, y, z]`

> **ERRATA:** The scale factor was previously documented as `2047.0 × √2 ≈ 2895.27` and byte
> order as MSB-first. The actual firmware uses **4096.0** scale and **LSB-first** byte order.
> See ORIENTATION_SPEC.md §5 for the authoritative encoding specification.
> The body frame convention is **+Y = nose** (not +Z). See ORIENTATION_SPEC.md §1.2.

### 5.6 Flight State Machine (FSM) States

| Value | Name | Description |
|---|---|---|
| `0x0` | PAD | On pad, awaiting launch |
| `0x1` | BOOST | Motor burning (1st stage) |
| `0x2` | COAST | Coasting after burnout |
| `0x3` | COAST_1 | Coast after 1st stage (multi-stage) |
| `0x4` | SUSTAIN | 2nd stage motor burning |
| `0x5` | COAST_2 | Coast after 2nd stage |
| `0x6` | APOGEE | Apogee detected |
| `0x7` | DROGUE | Drogue chute deployed |
| `0x8` | MAIN | Main chute deployed |
| `0x9` | RECOVERY | Recovery (single deploy) |
| `0xA` | TUMBLE | Tumble detected (no drogue) |
| `0xB` | LANDED | On ground |

---

## 6. GS Relay Messages (0x10–0x14)

### 6.1 GS_MSG_TELEM (0x10) — Ground Station Telemetry Relay

**Size:** 39 bytes
**CRC coverage:** bytes [0–34], CRC at [35–38]

Contains all FC_MSG_FAST fields plus GS-added radio link quality and derived values.

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x10` |
| 1–2 | status | u16 LE | — | bitmap | Same as FC_MSG_FAST |
| 3–4 | altitude | u16 LE | × 10.0 | m | Altitude AGL |
| 5–6 | velocity | i16 LE | × 0.1 | m/s | Velocity |
| 7–11 | quaternion | 5 bytes | — | — | Smallest-three |
| 12–13 | flight_time | u16 LE | × 0.1 | s | Mission elapsed time |
| 14 | battery | u8 | 6.0 + raw × 0.012 | V | Battery voltage |
| 15 | seq | u8 | — | — | GS sequence number |
| 16–17 | rssi | i16 LE | × 0.1 | dBm | Received signal strength |
| 18 | snr | i8 | × 0.25 | dB | Signal-to-noise ratio |
| 19–20 | freq_err | i16 LE | × 1 | Hz | Frequency error |
| 21–22 | data_age | u16 LE | × 1 | ms | Time since last valid FC packet |
| 23 | recovery | u8 | — | bitmap | See below |
| 24–25 | mach | u16 LE | × 0.001 | — | Mach number |
| 26–27 | qbar | u16 LE | × 1 | Pa | Dynamic pressure |
| 28–29 | roll | i16 LE | × 0.1 | deg | Roll angle |
| 30–31 | pitch | i16 LE | × 0.1 | deg | Pitch angle |
| 32–33 | yaw | i16 LE | × 0.1 | deg | Yaw angle |
| 34 | reserved | u8 | — | — | — |
| 35–38 | crc32 | u32 LE | — | — | CRC-32 over [0–34] |

**Recovery byte (offset 23):**
- Bit 7: `recovered` — packet was error-corrected
- Bits 6:4: `method` — correction method code
- Bits 3:0: `confidence` — correction confidence (0–15)

### 6.2–6.5 GS_MSG_GPS (0x11), GS_MSG_EVENT (0x12), GS_MSG_STATUS (0x13), GS_MSG_CORRUPT (0x14)

**Status:** Stub (future implementation). Parser stores raw bytes for forward compatibility.

---

## 7. Command Messages (MC → FC)

All safety-critical commands use the CAC (Command-Acknowledge-Confirm) protocol (see §10).

### 7.1 CMD_ARM (0x80)

**Size:** 12 bytes
**CRC coverage:** bytes [0–7], CRC at [8–11]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0x80` |
| 1 | magic_1 | u8 | `0xCA` |
| 2 | magic_2 | u8 | `0x5A` |
| 3–4 | nonce | u16 LE | Transaction ID (random) |
| 5 | channel | u8 | Pyro channel (0–3) |
| 6 | action | u8 | `0x01` = arm, `0x00` = disarm |
| 7 | ~channel | u8 | Bitwise complement of channel |
| 8–11 | crc32 | u32 LE | CRC-32 over [0–7] |

### 7.2 CMD_FIRE (0x81)

**Size:** 13 bytes
**CRC coverage:** bytes [0–8], CRC at [9–12]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0x81` |
| 1 | magic_1 | u8 | `0xCA` |
| 2 | magic_2 | u8 | `0x5A` |
| 3–4 | nonce | u16 LE | Transaction ID |
| 5 | channel | u8 | Pyro channel (0–3) |
| 6 | duration | u8 | Fire duration (clamped 0–255) |
| 7 | ~channel | u8 | Complement of channel |
| 8 | ~duration | u8 | Complement of duration |
| 9–12 | crc32 | u32 LE | CRC-32 over [0–8] |

### 7.3 CONFIRM (0xF0)

**Size:** 9 bytes
**CRC coverage:** bytes [0–4], CRC at [5–8]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xF0` |
| 1 | magic_1 | u8 | `0xCA` |
| 2 | magic_2 | u8 | `0x5A` |
| 3–4 | nonce | u16 LE | Nonce from ACK |
| 5–8 | crc32 | u32 LE | CRC-32 over [0–4] |

### 7.4 ABORT (0xF1)

**Size:** 9 bytes (identical layout to CONFIRM, different msg_id)

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xF1` |
| 1 | magic_1 | u8 | `0xCA` |
| 2 | magic_2 | u8 | `0x5A` |
| 3–4 | nonce | u16 LE | Transaction nonce |
| 5–8 | crc32 | u32 LE | CRC-32 over [0–4] |

---

## 8. Response Messages (FC → MC)

### 8.1 ACK_ARM (0xA0)

**Size:** 12 bytes
**CRC coverage:** bytes [0–7], CRC at [8–11]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xA0` |
| 1–2 | nonce | u16 LE | Echoed from CMD_ARM |
| 3 | echo_channel | u8 | Echoed channel |
| 4 | echo_action | u8 | Echoed action |
| 5 | arm_state | u8 | Current arm bitmap (bits 0–3 = ch 1–4) |
| 6 | cont_state | u8 | Current continuity bitmap |
| 7 | reserved | u8 | — |
| 8–11 | crc32 | u32 LE | CRC-32 over [0–7] |

### 8.2 ACK_FIRE (0xA1)

**Size:** 13 bytes
**CRC coverage:** bytes [0–8], CRC at [9–12]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xA1` |
| 1–2 | nonce | u16 LE | Echoed from CMD_FIRE |
| 3 | echo_channel | u8 | Echoed channel |
| 4 | echo_duration | u8 | Echoed duration |
| 5 | flags | u8 | bit 0: test_mode, bit 1: channel_armed |
| 6 | cont_state | u8 | Continuity bitmap |
| 7–8 | reserved | u16 | — |
| 9–12 | crc32 | u32 LE | CRC-32 over [0–8] |

### 8.3 NACK (0xE0)

**Size:** 10 bytes
**CRC coverage:** bytes [0–5], CRC at [6–9]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xE0` |
| 1–2 | nonce | u16 LE | Echoed from rejected command |
| 3 | error_code | u8 | Error code (see table) |
| 4–5 | reserved | u16 | — |
| 6–9 | crc32 | u32 LE | CRC-32 over [0–5] |

**NACK error codes:**

| Code | Name | Description |
|---|---|---|
| `0x01` | CrcFail | CRC check failed |
| `0x02` | BadState | Command invalid in current flight state |
| `0x03` | NotArmed | Channel not armed (fire rejected) |
| `0x04` | NoTestMode | Test mode not available |
| `0x05` | NonceReuse | Nonce already used |
| `0x06` | NoContinuity | No continuity on channel |
| `0x07` | LowBattery | Battery voltage too low |
| `0x08` | SelfTest | Self-test failure |
| `0x09` | CfgTooLarge | Config payload exceeds limit |
| `0x0A` | FlashFail | Flash write error |

### 8.4 ACK_CONFIG (0xA3)

**Size:** 13 bytes
**CRC coverage:** bytes [0–8], CRC at [9–12]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xA3` |
| 1–2 | nonce | u16 LE | Echoed from config upload |
| 3–6 | config_hash | u32 LE | CRC-32 of accepted config |
| 7 | protocol_version | u8 | FC protocol version |
| 8 | reserved | u8 | — |
| 9–12 | crc32 | u32 LE | CRC-32 over [0–8] |

---

## 9. Handshake & System Messages

### 9.1 HANDSHAKE Request (MC → FC)

**Size:** 1 byte (no CRC)

| Offset | Field | Type |
|---|---|---|
| 0 | msg_id | u8 (`0xC0`) |

Sent immediately after USB serial connection is opened.

### 9.2 HANDSHAKE Response (FC → MC)

**Size:** Variable (minimum 6 bytes)
**CRC coverage:** bytes [0 .. N-5], CRC at [N-4 .. N-1]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xC0` |
| 1 | protocol_version | u8 | FC protocol version (must be 5) |
| 2 .. N-5 | fw_version | ASCII | Firmware version string |
| N-4 .. N-1 | crc32 | u32 LE | CRC-32 |

### 9.3 SIM_FLIGHT (MC → FC)

**Size:** 5 bytes
**CRC coverage:** bytes [0–0], CRC at [1–4]

| Offset | Field | Type |
|---|---|---|
| 0 | msg_id | u8 (`0xD0`) |
| 1–4 | crc32 | u32 LE |

Triggers simulated flight on the FC (bench testing only).

---

## 10. CAC State Machine

The CAC (Command-Acknowledge-Confirm) protocol ensures safety-critical commands (ARM, DISARM, FIRE) are reliably delivered and verified.

### 10.1 Phases

```
IDLE → SENDING_CMD → AWAITING_ACK → VERIFYING_ACK → SENDING_CONFIRM → COMPLETE
                         ↑ (retry)          |
                         └──────────────────┘ (leg timeout)
Any phase → FAILED (on NACK, overall timeout, echo mismatch, operator abort)
```

| Phase | Description |
|---|---|
| `idle` | Ready for new command |
| `sending_cmd` | Building and transmitting command packet |
| `awaiting_ack` | Waiting for ACK/NACK from FC |
| `verifying_ack` | Verifying echoed fields in ACK match request |
| `sending_confirm` | Transmitting CONFIRM packet |
| `complete` | Exchange succeeded |
| `failed` | Exchange failed (see error message) |

### 10.2 Timing

| Parameter | Value |
|---|---|
| Leg timeout (per retry) | 2000 ms |
| Overall timeout (entire exchange) | 10000 ms |
| Max retries | 10 |
| Confirm delay (after echo verified) | 1000 ms |

### 10.3 Echo Verification

**ARM/DISARM:** ACK must echo the exact `channel` and `action` from the request.
**FIRE:** ACK must echo the exact `channel` and `duration` from the request.
**Mismatch:** MC sends ABORT and transitions to FAILED.

### 10.4 Telemetry-as-Parallel-ACK

While awaiting ACK for an ARM command, if an FC telemetry status bitmap arrives showing the target channel's armed state matches the requested state, the CAC machine advances to VERIFYING_ACK using telemetry as the echo source. This handles cases where the ACK packet was lost but the FC did act on the command.

### 10.5 Nonce Generation

Each CAC exchange uses a random 16-bit nonce (`0x0000`–`0xFFFF`). The same nonce is used for CMD, ACK matching, CONFIRM, and ABORT within one exchange. Generated via `crypto.getRandomValues()` when available, else `Math.random()`.

### 10.6 Busy Reset

If the CAC machine is stuck (e.g., waiting for an ACK that never arrives), it is automatically reset before accepting a new command from the UI. This prevents the user from being locked out during bench testing.

### 10.7 UI State Exposed

```typescript
{
  busy: boolean;              // true during active exchange
  command_type: string | null; // 'arm' | 'disarm' | 'fire' | null
  target_channel: number | null; // 1–4 or null
  error: string | null;       // human-readable error or null
  nack_code: number | null;   // raw NACK code or null
  retry_count: number;        // retransmissions so far
}
```

---

## 11. Telemetry Store

The telemetry store maintains a single `TelemetrySnapshot` object updated by incoming packets and pushed to the renderer on every change.

### 11.1 Snapshot Fields

**Connection State:**

| Field | Type | Default | Source |
|---|---|---|---|
| `fc_conn` | boolean | false | `set_connection('fc', ...)` |
| `gs_conn` | boolean | false | `set_connection('gs', ...)` |
| `protocol_ok` | boolean | false | Handshake response |
| `fw_version` | string \| null | null | Handshake response |
| `config_hash` | number \| null | null | ACK_CONFIG |
| `config_hash_verified` | boolean | false | Hash comparison |

**Core Telemetry (from FC_MSG_FAST / GS_MSG_TELEM):**

| Field | Type | Default | Unit | Source |
|---|---|---|---|---|
| `alt_m` | number | 0 | metres | altitude × 10.0 |
| `vel_mps` | number | 0 | m/s | velocity × 0.1 |
| `quat` | [n,n,n,n] | [1,0,0,0] | — | smallest-three decode |
| `roll_deg` | number | 0 | degrees | derived from quat |
| `pitch_deg` | number | 0 | degrees | derived from quat |
| `yaw_deg` | number | 0 | degrees | derived from quat |
| `mach` | number | 0 | — | derived (ISA model) |
| `qbar_pa` | number | 0 | Pa | derived (exp density) |
| `batt_v` | number | 0 | V | 6.0 + raw × 0.012 |
| `fsm_state` | number | 0 | enum | status bits 15:12 |
| `flight_time_s` | number | 0 | s | raw × 0.1 |
| `seq` | number | 0 | — | Rolling sequence counter |

**Pyro Status (4 channels):**

| Field | Type | Default | Description |
|---|---|---|---|
| `channel` | number | 1–4 | Hardware channel |
| `armed` | boolean | false | From status bitmap |
| `continuity` | boolean | false | From status bitmap |
| `fired` | boolean | false | From EVENT packets |
| `role` | string | '' | MC-side config only |
| `cont_v` | number | 0 | Continuity voltage |

**GPS (from FC_MSG_GPS):**

| Field | Type | Default | Unit |
|---|---|---|---|
| `gps_dlat_m` | number | 0 | metres |
| `gps_dlon_m` | number | 0 | metres |
| `gps_alt_msl_m` | number | 0 | metres |
| `gps_fix` | number | 0 | 0/2/3 |
| `gps_sats` | number | 0 | count |
| `gps_pdop` | number | 0 | — |
| `gps_range_saturated` | boolean | false | — |

**Link Quality (from GS_MSG_TELEM):**

| Field | Type | Default | Unit |
|---|---|---|---|
| `rssi_dbm` | number | 0 | dBm |
| `snr_db` | number | 0 | dB |
| `freq_err_hz` | number | 0 | Hz |
| `data_age_ms` | number | 0 | ms |
| `stale` | boolean | false | — |
| `stale_since_ms` | number | 0 | ms |

**Event Log:**

| Field | Type | Default |
|---|---|---|
| `events` | EventLogEntry[] | [] |
| `apogee_alt_m` | number | 0 |

### 11.2 Ring Buffers

- **Depth:** 150 samples
- **Fields buffered:** `buf_alt`, `buf_vel`, `buf_qbar`
- **Behaviour:** FIFO — oldest dropped when depth exceeded

### 11.3 Stale Detection

- **Threshold:** 500 ms
- **Tick rate:** 100 ms (main-process interval)
- **Logic:** If `(now - last_valid_packet_time) > 500ms`, mark `stale = true`
- **Reset:** Any valid FC_MSG_FAST or GS_MSG_TELEM clears stale

### 11.4 Subscriber Pattern

```typescript
subscribe(callback: (snapshot: TelemetrySnapshot) => void): () => void
```

Every store update calls all subscribers with an isolated (shallow-copied) snapshot. The IPC layer subscribes to push snapshots to the renderer via `webContents.send()`.

---

## 12. IPC Channels & Handlers

### 12.1 Main → Renderer (Push)

| Channel | Payload | Trigger |
|---|---|---|
| `casper:telemetry` | TelemetrySnapshot | Every store update |
| `casper:cac-update` | CacUiState | CAC phase change |
| `casper:diag-result` | Diagnostic result | After self-test |
| `casper:serial-ports` | PortInfo[] | After port scan |

### 12.2 Renderer → Main (Invoke, returns Promise)

| Channel | Arguments | Returns | Action |
|---|---|---|---|
| `casper:connect-fc` | port: string | void | Open FC serial, send handshake |
| `casper:connect-gs` | port: string | void | Open GS serial |
| `casper:upload-config` | config: FlightConfig | {ok, hash?, error?} | Serialize + send config |
| `casper:verify-config-hash` | — | {ok, fc_hash?, verified?} | Compare config hashes |
| `casper:download-flight-log` | — | Uint8Array | Download log (stub) |

### 12.3 Renderer → Main (Send, fire-and-forget)

| Channel | Arguments | Action |
|---|---|---|
| `casper:disconnect-fc` | — | Close FC serial |
| `casper:disconnect-gs` | — | Close GS serial |
| `casper:scan-ports` | — | Scan ports, push result |
| `casper:cmd-arm` | channel (1–4) | CAC ARM exchange |
| `casper:cmd-disarm` | channel (1–4) | CAC DISARM exchange |
| `casper:cmd-fire` | channel (1–4), duration_ms | CAC FIRE exchange |
| `casper:cmd-confirm` | — | Manual confirm (stub) |
| `casper:cmd-abort` | — | Abort CAC exchange |
| `casper:cmd-enter-test-mode` | — | Enter test mode (stub) |
| `casper:cmd-exit-test-mode` | — | Exit test mode (stub) |
| `casper:run-diagnostics` | — | Trigger FC self-test |
| `casper:erase-flight-log` | — | Erase log (stub) |
| `casper:cmd-sim-flight` | — | Send SIM_FLIGHT (0xD0) |

**Note:** ARM, DISARM, and FIRE handlers auto-reset the CAC machine if it is stuck before initiating the new command.

---

## 13. Preload Bridge API (`window.casper`)

All 22 methods exposed to the renderer via Electron's `contextBridge`:

### Subscriptions (main → renderer)

| Method | Callback Signature | Returns |
|---|---|---|
| `on_telemetry(cb)` | `(snapshot: TelemetrySnapshot) => void` | unsubscribe fn |
| `on_cac_update(cb)` | `(state: CacUiState) => void` | unsubscribe fn |
| `on_diag_result(cb)` | `(results: DiagResult) => void` | unsubscribe fn |
| `on_serial_ports(cb)` | `(ports: PortInfo[]) => void` | unsubscribe fn |

### Commands (renderer → main)

| Method | Arguments | Returns | IPC Type |
|---|---|---|---|
| `connect_fc(port)` | port: string | Promise | invoke |
| `disconnect_fc()` | — | void | send |
| `connect_gs(port)` | port: string | Promise | invoke |
| `disconnect_gs()` | — | void | send |
| `scan_ports()` | — | void | send |
| `cmd_arm(channel)` | channel: number (1–4) | void | send |
| `cmd_disarm(channel)` | channel: number (1–4) | void | send |
| `cmd_fire(channel, duration_ms)` | channel, duration: number | void | send |
| `cmd_confirm()` | — | void | send |
| `cmd_abort()` | — | void | send |
| `cmd_enter_test_mode()` | — | void | send |
| `cmd_exit_test_mode()` | — | void | send |
| `upload_config(config)` | config: FlightConfig | Promise | invoke |
| `verify_config_hash()` | — | Promise | invoke |
| `run_diagnostics()` | — | void | send |
| `download_flight_log()` | — | Promise<Uint8Array> | invoke |
| `erase_flight_log()` | — | void | send |
| `cmd_sim_flight()` | — | void | send |

**Channel indexing convention:** Preload API uses 1-indexed channels (1–4). The backend converts to 0-indexed (0–3) for the wire protocol.

---

## 14. Frontend Hooks & UI

### 14.1 useTelemetry()

Subscribes to `window.casper.on_telemetry()` and maps the raw TelemetrySnapshot to a UI-friendly shape:

| UI Field | Source | Transform |
|---|---|---|
| `rssi` | `snapshot.rssi_dbm` | direct |
| `dataAge` | `snapshot.data_age_ms` | direct |
| `batt` | `snapshot.batt_v` | direct |
| `gpsLat` | `snapshot.gps_dlat_m` | `/ 111320` (flat-earth approx) |
| `gpsLon` | `snapshot.gps_dlon_m` | `/ 111320` |
| `gpsFix` | `snapshot.gps_fix` | 3→"3D", 2→"2D", else "NONE" |
| `gpsSats` | `snapshot.gps_sats` | direct |
| `ekfAlt` / `alt` | `snapshot.alt_m` | direct |
| `vel` | `snapshot.vel_mps` | direct |
| `roll/pitch/yaw` | `snapshot.*_deg` | direct |
| `mach` | `snapshot.mach` | direct |
| `state` | `snapshot.fsm_state` | mapped to name string |
| `t` | `snapshot.flight_time_s` | `× 1000` (ms) |
| `stale` | `snapshot.stale` | direct |
| `staleSince` | `snapshot.stale_since_ms` | `/ 1000` (seconds) |
| `qbar` | `snapshot.qbar_pa` | direct |
| `integrity` | `snapshot.integrity_pct` | direct |
| `pyro[i]` | `snapshot.pyro[i]` | mapped (see below) |

**Pyro mapping per channel:**
- `hwCh`: hardware channel (1–4)
- `role`: MC-side role assignment (local state, not from FC)
- `cont`: continuity boolean
- `contV`: continuity voltage
- `armed`: armed boolean
- `firing`: fired boolean

**Command functions provided:**
- `toggleArm(i)` — 0-indexed, sends `cmd_arm(i+1)` or `cmd_disarm(i+1)`
- `firePyro(i)` — 0-indexed, sends `cmd_fire(i+1, 1200)` (1200 ms default)
- `setRole(i, role)` — MC-side only, not sent to FC

**Default pyro roles:** Apogee, Main, Apogee Backup, Main Backup

### 14.2 useCommand()

Subscribes to `window.casper.on_cac_update()`. Returns: `{ busy, command_type, target_channel, error, nack_code, retry_count, abort }`.

### 14.3 useSerial()

Subscribes to `window.casper.on_serial_ports()` and `on_telemetry()` (for connection flags). Returns: `{ ports, fc_connected, gs_connected, scan, connect_fc, connect_gs, disconnect_fc, disconnect_gs }`.

### 14.4 useDiagnostics()

7 built-in tests: IMU, Magnetometer, Barometer, EKF Init, Attitude, Flash, Config. Returns: `{ tests, runAll, reset }`.

### 14.5 UI Tabs

| Tab | Icon | Purpose |
|---|---|---|
| **SETUP** | ⚙ | Serial port picker, sensor diagnostics, pyro config, config upload |
| **TEST** | ⚡ | Bench testing: live telemetry summary, pyro arm/fire controls, CAC status, SIM FLIGHT |
| **FLIGHT** | ▲ | Real-time flight monitoring: GPS, altitude, velocity, graphs, pyro status, pre-flight checklist, terminal countdown, 3D orientation, vertical state bar |
| **TRACKING** | ◎ | 3D attitude canvas + ground track radar |

**Connection gating:** Flight and Test tabs show live data when **either** FC or GS is connected (`connected = fcConn || gsConn`).

### 14.6 Pre-Flight Checks

| Check | Condition | Configurable |
|---|---|---|
| Battery Voltage | `batt >= minBatt` (default 7.4V) | Yes |
| GPS Fix | `gpsFix === "3D" && gpsSats >= 6` | No |
| Pyro Continuity | All non-Custom channels have continuity | No |
| IMU Health | `|pitch − 90| < 15°` (vertical) | No |
| Radio Link | `!stale && dataAge < 500ms` | No |
| Data Integrity | `integrity >= minIntegrity` (default 90%) | Yes |

### 14.7 Flight State TTS Callouts

| Transition | Callout |
|---|---|
| → COAST (from BOOST) | "Motor burnout." |
| → SUSTAIN | "Second stage ignition confirmed." |
| → APOGEE | "Apogee detected." |
| → DROGUE | "Drogue parachute deployed." |
| → MAIN / RECOVERY | "Main parachute deployed." |
| → TUMBLE | "Warning. Tumble detected. No drogue deployment." |
| → LANDED | "The rocket has landed." |

---

## 15. Flight Configuration Format

### 15.1 Binary Serialization

**Total size:** 163 bytes (3 header + 156 payload + 4 CRC)

**Header (3 bytes):**

| Offset | Field | Type |
|---|---|---|
| 0 | config_version | u8 (`0x01`) |
| 1–2 | total_length | u16 LE (163) |

**Per-Channel Block (32 bytes × 4 channels = 128 bytes, offset 3–130):**

| Offset | Field | Type | Description |
|---|---|---|---|
| +0 | hw_channel | u8 | 0–3 |
| +1 | role | u8 | PyroRole enum (0–6) |
| +2 | altitude_source | u8 | 0=EKF, 1=baro |
| +3 | flags | u8 | bit 0: early_deploy, bit 1: backup_height |
| +4–7 | fire_duration_s | f32 LE | seconds |
| +8–11 | deploy_alt_m | f32 LE | metres AGL |
| +12–15 | time_after_apogee_s | f32 LE | seconds |
| +16–19 | early_deploy_vel_mps | f32 LE | m/s |
| +20–23 | backup_value | f32 LE | time (s) or height (m) |
| +24 | motor_number | u8 | motor index |
| +25 | max_ignition_angle_deg | u8 | degrees |
| +26 | max_flight_angle_deg | u8 | degrees |
| +27–28 | min_velocity_mps | i16 LE | scaled ×10 |
| +29–30 | min_altitude_m | i16 LE | metres |
| +31 | fire_delay_s | u8 | scaled ×10 |

**PyroRole values:** 0=Apogee, 1=Apogee Backup, 2=Main, 3=Main Backup, 4=Ignition, 5=Ignition Backup, 6=Custom

**Pad Location (12 bytes, offset 131–142):**

| Offset | Field | Type |
|---|---|---|
| 131–134 | pad_lat_deg | f32 LE |
| 135–138 | pad_lon_deg | f32 LE |
| 139–142 | pad_alt_msl_m | f32 LE |

**FSM Fallback (8 bytes, offset 143–150):**

| Offset | Field | Type |
|---|---|---|
| 143–146 | alt_threshold_m | f32 LE |
| 147–150 | vel_threshold_mps | f32 LE |

**Pre-Flight Thresholds (8 bytes, offset 151–158):**

| Offset | Field | Type |
|---|---|---|
| 151–154 | min_batt_v | f32 LE |
| 155–158 | min_integrity_pct | f32 LE |

**CRC (4 bytes, offset 159–162):**
CRC-32 over bytes [0–158].

**Config hash:** `CRC-32(serialized_bytes[0 .. length-5])` — used for upload verification.

---

## 16. Recovery Pipeline

### 16.1 Stage 1 — Single-Bit CRC Correction

Corrects single-bit errors in telemetry packets using CRC syndrome lookup.

1. Compute CRC of received payload
2. XOR with received CRC → syndrome
3. If syndrome = 0: packet valid, no correction needed
4. Look up syndrome in precomputed table (one entry per bit position)
5. If found: flip the identified bit, verify CRC, return corrected packet
6. If not found: multi-bit corruption, pass to next stage

**Syndrome tables are cached per payload length.**

### 16.2 Stage 3 — Temporal Interpolation

**Status:** Stub (returns null). Future: Kalman-based prediction from historical data.

### 16.3 Stage 4 — Zero-Order Hold

Repeats last known-good telemetry values during communication gaps. Tracks staleness duration for UI display. Threshold: 500 ms.

---

## 17. Derived Computations

### 17.1 Mach Number

ISA standard atmosphere model:
- Sea-level temp: 288.15 K
- Lapse rate: 0.0065 K/m (below 11 km)
- Tropopause temp: 216.65 K (above 11 km)
- Speed of sound: `a = √(γ × R × T)` where γ=1.4, R=287.05 J/(kg·K)
- Mach = `|velocity| / a`

### 17.2 Dynamic Pressure (q̄)

Exponential density model:
- Sea-level density: 1.225 kg/m³
- Scale height: 8500 m
- `ρ = 1.225 × e^(-alt/8500)`
- `q̄ = 0.5 × ρ × v²`

### 17.3 Euler Angles

Aerospace convention (ZYX rotation) from quaternion [w,x,y,z]:
- Roll: `atan2(2(wx+yz), 1−2(x²+y²))`
- Pitch: `asin(clamp(2(wy−zx), −1, 1))`
- Yaw: `atan2(2(wz+xy), 1−2(y²+z²))`

---

## 18. Appendices

### A. Byte Layout Quick Reference

```
FC_MSG_FAST (20 bytes):
  [0]     0x01
  [1-2]   status (u16 LE)
  [3-4]   altitude (u16 LE, ×10.0 → m)
  [5-6]   velocity (i16 LE, ×0.1 → m/s)
  [7-11]  quaternion (5 bytes, smallest-three)
  [12-13] flight_time (u16 LE, ×0.1 → s)
  [14]    battery (u8, 6.0 + raw×0.012 → V)
  [15]    seq (u8, rolling counter)
  [16-19] CRC-32 (u32 LE)

FC_MSG_GPS (17 bytes):
  [0]     0x02
  [1-4]   dlat_mm (i32 LE, ÷1000 → m)
  [5-8]   dlon_mm (i32 LE, ÷1000 → m)
  [9-10]  alt_msl (u16 LE, ×10.0 → m)
  [11]    fix_type (u8)
  [12]    sat_count (u8)
  [13-16] CRC-32 (u32 LE)

FC_MSG_EVENT (11 bytes):
  [0]     0x03
  [1]     event_type (u8)
  [2-3]   event_data (u16 LE)
  [4-5]   flight_time (u16 LE, ×0.1 → s)
  [6]     reserved
  [7-10]  CRC-32 (u32 LE)

CMD_ARM (12 bytes):
  [0]     0x80
  [1]     0xCA  [2] 0x5A
  [3-4]   nonce (u16 LE)
  [5]     channel (0-3)
  [6]     action (1=arm, 0=disarm)
  [7]     ~channel
  [8-11]  CRC-32 (u32 LE)

CMD_FIRE (13 bytes):
  [0]     0x81
  [1]     0xCA  [2] 0x5A
  [3-4]   nonce (u16 LE)
  [5]     channel (0-3)
  [6]     duration (u8, clamped 0-255)
  [7]     ~channel
  [8]     ~duration
  [9-12]  CRC-32 (u32 LE)

CONFIRM (9 bytes):
  [0]     0xF0
  [1]     0xCA  [2] 0x5A
  [3-4]   nonce (u16 LE)
  [5-8]   CRC-32 (u32 LE)

ABORT (9 bytes):
  [0]     0xF1
  [1]     0xCA  [2] 0x5A
  [3-4]   nonce (u16 LE)
  [5-8]   CRC-32 (u32 LE)

ACK_ARM (12 bytes):
  [0]     0xA0
  [1-2]   nonce (u16 LE)
  [3]     echo_channel
  [4]     echo_action
  [5]     arm_state bitmap
  [6]     cont_state bitmap
  [7]     reserved
  [8-11]  CRC-32 (u32 LE)

ACK_FIRE (13 bytes):
  [0]     0xA1
  [1-2]   nonce (u16 LE)
  [3]     echo_channel
  [4]     echo_duration
  [5]     flags (bit0=test_mode, bit1=armed)
  [6]     cont_state bitmap
  [7-8]   reserved
  [9-12]  CRC-32 (u32 LE)

NACK (10 bytes):
  [0]     0xE0
  [1-2]   nonce (u16 LE)
  [3]     error_code
  [4-5]   reserved
  [6-9]   CRC-32 (u32 LE)

HANDSHAKE request (1 byte):
  [0]     0xC0

HANDSHAKE response (variable, min 6):
  [0]     0xC0
  [1]     protocol_version
  [2..N-5] fw_version (ASCII)
  [N-4..N-1] CRC-32 (u32 LE)

SIM_FLIGHT (5 bytes):
  [0]     0xD0
  [1-4]   CRC-32 (u32 LE)

GS_MSG_TELEM (39 bytes):
  [0]     0x10
  [1-2]   status (u16 LE)
  [3-4]   altitude (u16 LE)
  [5-6]   velocity (i16 LE)
  [7-11]  quaternion (5 bytes)
  [12-13] flight_time (u16 LE)
  [14]    battery (u8)
  [15]    seq (u8)
  [16-17] rssi (i16 LE, ×0.1 → dBm)
  [18]    snr (i8, ×0.25 → dB)
  [19-20] freq_err (i16 LE → Hz)
  [21-22] data_age (u16 LE → ms)
  [23]    recovery byte
  [24-25] mach (u16 LE, ×0.001)
  [26-27] qbar (u16 LE → Pa)
  [28-29] roll (i16 LE, ×0.1 → deg)
  [30-31] pitch (i16 LE, ×0.1 → deg)
  [32-33] yaw (i16 LE, ×0.1 → deg)
  [34]    reserved
  [35-38] CRC-32 (u32 LE)
```

### B. Scaling Factor Summary

| Raw Field | Formula | Result Unit |
|---|---|---|
| Altitude (u16) | `raw × 10.0` | metres |
| Velocity (i16) | `raw × 0.1` | m/s |
| Flight time (u16) | `raw × 0.1` | seconds |
| Battery (u8) | `6.0 + raw × 0.012` | volts |
| GPS altitude (u16) | `raw × 10.0` | metres MSL |
| GPS delta lat/lon (i32) | `raw / 1000` | metres |
| RSSI (i16) | `raw × 0.1` | dBm |
| SNR (i8) | `raw × 0.25` | dB |
| Mach (u16) | `raw × 0.001` | — |
| Roll/Pitch/Yaw (i16) | `raw × 0.1` | degrees |

### C. End-to-End Data Flow

```
FC Hardware
  │  USB serial (115200, 8N1)
  ▼
COBS decode (0x00 delimiter)
  │  Uint8Array payload
  ▼
parse_packet(data)
  │  Dispatches by data[0] msg_id
  ▼
TelemetryStore.update_from_*()
  │  Updates snapshot, pushes ring buffers
  ▼
store.subscribe() callback
  │  Isolated snapshot copy
  ▼
webContents.send('casper:telemetry', snapshot)
  │  Electron IPC (main → renderer)
  ▼
ipcRenderer.on('casper:telemetry')
  │  Preload bridge
  ▼
window.casper.on_telemetry(callback)
  │  React hook subscription
  ▼
useTelemetry() → mapSnapshot()
  │  Maps to UI-friendly shape
  ▼
React component re-render
  │  Flight tab, Test tab, graphs, pyro boxes
  ▼
User sees live data
```
