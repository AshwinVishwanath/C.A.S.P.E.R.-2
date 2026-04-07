# C.A.S.P.E.R. 2 Mission Control — Interface Specification

**Protocol Version:** 5
**CRC Standard:** CRC-32/ISO-HDLC
**Transport:** COBS-framed USB serial (115200 baud) + SX1276 LoRa 868 MHz
**Byte Order:** Little-endian throughout
**Last Updated:** 2026-03-14

This document specifies every interface between Mission Control (MC), the Flight Computer (FC), and the Ground Station (GS). It serves as the authoritative contract for firmware and software development.

> ### Mismatches vs CASPER_TELEM_SET_V5 (msgset v5, Rev 0.5)
>
> This INTERFACE_SPEC is the authoritative, up-to-date document. The msgset v5 (Feb 2026) predates the current firmware implementation and has the following discrepancies. **This spec is correct; the msgset is stale.**
>
> **1. FC_MSG_FAST — size & altitude encoding**
> - Msgset: **19 bytes**, altitude = uint16 decametres (2B, 10 m resolution), CRC over bytes 0–14
> - This spec: **21 bytes**, altitude = u24 LE × 0.01 (3B, 1 cm resolution), CRC over bytes 0–16
>
> **2. FC_MSG_GPS — completely different layout**
> - Msgset: **12 bytes** — dlat/dlon int16 metres (2B each), alt uint16 decametres (2B), fixsat packed u8 ([7:6] fix + [5:0] sats), pdop u8, CRC over 0–7
> - This spec: **18 bytes** — dlat/dlon i32 millimetres (4B each), alt u24 × 0.01 (3B), fix_type u8, sat_count u8, no pDOP, CRC over 0–13
>
> **3. FC_MSG_EVENT — size**
> - Msgset: **9 bytes**, CRC over bytes 0–4
> - This spec: **11 bytes** (extra reserved byte), CRC over bytes 0–6
>
> **4. FC_TLM_STATUS bitmap — reserved bits now assigned**
> - Msgset byte 1 bits [1:0]: reserved
> - This spec byte 1 bits [1:0]: GPS_FIX, TEST_MODE
>
> **5. NACK — size & layout**
> - Msgset: **9 bytes** — includes magic 0xCA/0x5A, no reserved bytes
> - This spec: **10 bytes** — no magic bytes, has 2 reserved bytes
>
> **6. ACK_ARM — magic bytes removed**
> - Msgset: 13 bytes (includes magic 0xCA 0x5A after msg_id)
> - This spec: 12 bytes (no magic, starts [ID][nonce:2])
>
> **7. CMD_POLL / ACK_CONFIG — CRC width**
> - Msgset: CMD_POLL 5 bytes with CRC-16; ACK_CONFIG uses CRC-16
> - This spec: CMD_POLL variable; ACK_CONFIG 13 bytes with CRC-32
>
> **8. GS_MSG_STATUS — no longer TBD**
> - Msgset: deferred to Rev 0.6
> - This spec: fully defined, 24 bytes (§7.3)
>
> **9. GS_MSG_TELEM — no longer TBD**
> - Msgset: deferred to Rev 0.6
> - This spec: 39 bytes with Mach, Euler, recovery metadata (§7.1)
>
> **10. New messages not in msgset**
> - HANDSHAKE (0xC0), SIM_FLIGHT (0xD0), HIL_INJECT (0xD1), DUMP_FLASH (0xD2), DIAG (0xC2), UPLOAD (0xC1)
> - §6.7 GPS Subsystem (MAX-M10M driver interface)
> - §16 Flight configuration binary format
> - §17 Recovery pipeline
>
> **11. CRC polynomial — no longer TBD**
> - Msgset: "polynomial TBD"
> - This spec: CRC-32/ISO-HDLC fully specified (§4)
>
> **12. Bandwidth budget — stale**
> - Msgset: 19B FAST + 12B GPS = 214 B/s
> - This spec: 21B FAST + 18B GPS = higher actual load

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [USB Transport Layer](#2-usb-transport-layer)
3. [Radio Transport Layer (LoRa)](#3-radio-transport-layer-lora)
4. [CRC-32 Specification](#4-crc-32-specification)
5. [Message Format & Dispatch](#5-message-format--dispatch)
6. [FC Telemetry Messages (0x01–0x03)](#6-fc-telemetry-messages-0x010x03)
   - [6.7 GPS Subsystem (FC ↔ MAX-M10M)](#67-gps-subsystem-fc--max-m10m)
7. [GS Relay Messages (0x10–0x14)](#7-gs-relay-messages-0x100x14)
8. [Command Messages (MC → FC)](#8-command-messages-mc--fc)
9. [Response Messages (FC → MC)](#9-response-messages-fc--mc)
10. [Handshake & System Messages](#10-handshake--system-messages)
11. [CAC State Machine](#11-cac-state-machine)
12. [Telemetry Store](#12-telemetry-store)
13. [IPC Channels & Handlers](#13-ipc-channels--handlers)
14. [Preload Bridge API (`window.casper`)](#14-preload-bridge-api-windowcasper)
15. [Frontend Hooks & UI](#15-frontend-hooks--ui)
16. [Flight Configuration Format](#16-flight-configuration-format)
17. [Recovery Pipeline](#17-recovery-pipeline)
18. [Derived Computations](#18-derived-computations)
19. [Ground Station Implementation Guide](#19-ground-station-implementation-guide)
20. [Appendices](#20-appendices)

---

## 1. Architecture Overview

```
                        LoRa 868 MHz
┌─────────────┐    SX1276 raw packets     ┌─────────────┐
│ Flight      │◄═══════════════════════════│  Ground     │
│ Computer    │  No COBS, no delimiter     │  Station    │
│ (STM32H750) │  LoRa HW CRC + SW CRC-32  │  (SX1276)   │
│             │                            └──────┬──────┘
│ msg_id at   │    USB/COBS                       │ USB/COBS
│ byte[0]     │◄───115200─────┐              115200, 0x00 delim
└─────────────┘   0x00 delim  │                   │
                              │                   │
               ┌──────────────┴───────────────────┴────────────┐
               │  Mission Control (Electron)                    │
               │                                                │
               │  ┌─────────┐  ┌───────┐  ┌──────┐  ┌──────┐ │
               │  │ FcUsb   │→│Parser │→│Store │→│ IPC  │ │
               │  │ COBS    │  │       │  │      │  │      │ │
               │  │ decode  │  │       │  │      │  │      │ │
               │  └─────────┘  └───────┘  └──────┘  └──┬───┘ │
               │                    ↑                    │     │
               │  ┌─────────┐      │      ┌──────┐      │     │
               │  │ GsUsb   │──────┘      │ CAC  │←─────┘     │
               │  │ COBS    │             │Machine│            │
               │  │ decode  │             └──────┘            │
               │  └─────────┘                                  │
               │                         ┌──────────────┐     │
               │  Preload Bridge ────────│  Renderer    │     │
               │  (window.casper)        │  React UI   │     │
               │                         └──────────────┘     │
               └───────────────────────────────────────────────┘
```

**Connection Modes:**
- **FC Direct:** MC ↔ FC over USB. Telemetry arrives as 0x01/0x02/0x03 packets. Commands sent directly.
- **GS Relay:** MC ↔ GS over USB. GS relays FC telemetry as 0x10–0x14 packets via LoRa. Commands relayed through GS.
- **Radio Link:** FC ↔ GS over SX1276 LoRa at 868 MHz. Same packet formats as USB but **no COBS framing** — raw packets go directly into the SX1276 FIFO. LoRa explicit header mode provides framing. Dual integrity: LoRa hardware CRC (silicon) + software CRC-32 in every packet.
- Both USB modes can be active simultaneously. The CAC machine prefers GS if connected, falls back to FC.

---

## 2. USB Transport Layer

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

## 3. Radio Transport Layer (LoRa)

The FC transmits telemetry and receives commands over an SX1276 LoRa radio (Ai-Thinker RA-01H module) at 868 MHz ISM band. This is a peer transport to USB/COBS — the same packet formats (§6–§9) are used, but the framing is different.

### 3.1 Key Difference from USB: No COBS Framing

Over radio, raw packets go directly into the SX1276 FIFO. LoRa **explicit header mode** provides framing — the LoRa header contains payload length, coding rate, and CRC flag. There is no `0x00` delimiter and no COBS encoding.

**Integrity is provided by two layers:**
1. **LoRa hardware CRC** — checked by SX1276 silicon on receive. CRC errors are flagged in the IRQ register. Packets failing HW CRC are silently discarded.
2. **Software CRC-32** — the last 4 bytes of every packet are a CRC-32 (same as USB, see §4). This catches errors that slip past the LoRa CRC and provides end-to-end integrity.

### 3.2 Radio Module

| Parameter | Value |
|---|---|
| Module | Ai-Thinker RA-01H (SX1276 LoRa transceiver) |
| SPI bus | SPI1 (PA5/SCK, PA7/MOSI, PB4/MISO, PB0/CS) |
| DIO0 (TxDone/RxDone) | PB1 (GPIO polling, not EXTI) |
| DIO1 (RxTimeout) | PD7 (GPIO polling) |
| Carrier frequency | 868.000 MHz |
| Modulation | LoRa (CSS) |
| Header mode | Explicit (payload length in header) |
| Max packet size | 32 bytes (`RADIO_MAX_PACKET_SIZE`) |
| TX power | +20 dBm (PA_BOOST + PA_DAC high-power mode) |

### 3.3 Radio Profiles

Two profiles are defined. Both share the same frequency, sync word, preamble, and TX power. Only the spreading factor differs.

| Parameter | Profile A | Profile B |
|---|---|---|
| Spreading factor | SF7 | SF8 |
| Bandwidth | 250 kHz | 250 kHz |
| Coding rate | 4/5 (CR=5) | 4/5 (CR=5) |
| Sync word | `0x12` (private LoRa network) | `0x12` |
| Preamble | 8 symbols | 8 symbols |
| TX power | +20 dBm | +20 dBm |
| Frequency | 868.000 MHz | 868.000 MHz |

**Profile A** is the default, used at launch. It provides higher data rate and lower air time at shorter range.

**Profile B** is switched to automatically when the EKF estimates **altitude > 20,000 m** OR **velocity > 500 m/s**. The switch is **one-way** — the FC never reverts from B to A. The GS must also switch to Profile B to maintain the link (see §3.5).

### 3.4 FC Radio State Machine (TX/RX Cycle)

The FC radio manager runs from the superloop at ~833 Hz and implements a non-blocking state machine:

```
         ┌──────────────────────────────┐
         │                              │
         ▼                              │
    ┌─────────┐   TX period     ┌──────┴──┐
    │  IDLE   │───(100 ms)─────►│   TX    │
    └────┬────┘                 └────┬────┘
         │                           │
         │     ┌─────────────────────┘
         │     │  TxDone (DIO0)
         │     ▼
         │  ┌─────────┐
         │  │   RX    │ RXSINGLE, 80 symbols timeout
         │  └────┬────┘
         │       │  RxDone / RxTimeout / safety timeout
         └───────┘
```

| Parameter | Value | Source |
|---|---|---|
| TX cadence | 100 ms (10 Hz) | `RADIO_TX_PERIOD_MS` |
| TX timeout | 200 ms (radio reinit on timeout) | `RADIO_TX_TIMEOUT_MS` |
| RX window | 80 symbols, RXSINGLE mode | `RADIO_RX_WINDOW_SYMBOLS` |
| RX safety timeout | 200 ms | Same as `RADIO_TX_TIMEOUT_MS` |

**Cycle detail:**
1. **IDLE → TX:** Every 100 ms, the scheduler selects the highest-priority packet (see §3.6) and loads it into the SX1276 FIFO. Mode transitions to TX.
2. **TX → RX:** On TxDone (DIO0 asserted), the radio immediately opens an RXSINGLE window with an 80-symbol timeout. This is the **only window** in which the FC can receive uplink commands.
3. **RX → IDLE:** On RxDone (valid packet received), RxTimeout (no preamble detected within 80 symbols), or safety timeout (200 ms), the radio returns to IDLE.

**Implication for GS:** The GS must transmit commands **during the ~80-symbol RX window** that opens after each FC TX. Commands sent outside this window will be missed. The GS should listen continuously and transmit immediately after receiving an FC packet.

### 3.5 GS Profile Switching

The GS uses a simpler approach: it starts on Profile A in RX-continuous mode. If no valid packets are received for 2000 ms (`GS_PROFILE_LOSS_TIMEOUT_MS`), it switches one-way from Profile A to Profile B. This handles the case where the FC switches mid-flight while the GS is still on A.

### 3.6 TX Priority Queue

When the TX scheduler fires (every 100 ms), it selects the highest-priority pending packet:

| Priority | Type | Description |
|---|---|---|
| 1 (highest) | Response (ACK/NACK) | Ring buffer, 4 slots. Queued by CAC handlers after receiving a command. |
| 2 | Event | Ring buffer, 4 slots. Queued by FSM transitions, pyro events, etc. |
| 3 | Fast telemetry | Built on-the-fly from current EKF/pyro/FSM state. Default if nothing else is pending. |
| 4 (lowest) | GPS | Single-slot buffer (latest overwrites). Displaces one fast telemetry packet when pending. |

Each TX cycle sends exactly **one packet**. A response or event displaces that cycle's fast telemetry.

### 3.7 Radio Downlink Packets (FC → GS)

The radio uses the **same packet formats** as USB (§6):

| msg_id | Name | Size | Rate |
|---|---|---|---|
| `0x01` | FC_MSG_FAST | 21 bytes | ~10 Hz (default every TX cycle) |
| `0x02` | FC_MSG_GPS | 18 bytes | On GPS update (displaces one fast packet) |
| `0x03` | FC_MSG_EVENT | 11 bytes | On flight events (highest priority after ACK/NACK) |
| `0xA0` | ACK_ARM | 12 bytes | Response to CMD_ARM |
| `0xA1` | ACK_FIRE | 13 bytes | Response to CMD_FIRE |
| `0xA3` | ACK_CONFIG | 13 bytes | Response to CMD_POLL |
| `0xE0` | NACK | 10 bytes | Response to rejected command |

### 3.8 Radio Uplink Commands (GS → FC)

The FC accepts the following commands over LoRa (same formats as USB, §8):

| msg_id | Name | Size | Description |
|---|---|---|---|
| `0x80` | CMD_ARM | 12 bytes | Arm/disarm pyro channel |
| `0x81` | CMD_FIRE | 13 bytes | Fire pyro channel |
| `0x82` | CMD_TESTMODE | variable | Toggle test mode |
| `0x83` | CMD_POLL | variable | Config poll |
| `0xF0` | CONFIRM | 9 bytes | CAC confirm step |
| `0xF1` | ABORT | 9 bytes | CAC abort |

**Validation on FC:**
1. Only the above message IDs are accepted; all others are silently dropped
2. Commands 0x80–0x83, 0xF0, 0xF1 must contain magic bytes `0xCA, 0x5A` at bytes [1–2]
3. Software CRC-32 is validated (last 4 bytes)
4. On CRC pass, the command is dispatched to the same CAC handlers as the USB path
5. Response (ACK/NACK) is queued for the next TX slot (highest priority)

---

## 4. CRC-32 Specification

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

## 5. Message Format & Dispatch

Every decoded payload has `msg_id` at byte `[0]`. The parser dispatches by this byte. Over USB, the payload is COBS-decoded first. Over radio, the raw packet from the SX1276 FIFO is used directly.

| msg_id | Hex | Direction | Name | Decoded Size |
|---|---|---|---|---|
| 1 | `0x01` | FC → MC/GS | FC_MSG_FAST | 21 bytes |
| 2 | `0x02` | FC → MC/GS | FC_MSG_GPS | 18 bytes |
| 3 | `0x03` | FC → MC/GS | FC_MSG_EVENT | 11 bytes |
| 16 | `0x10` | GS → MC | GS_MSG_TELEM | 39 bytes (unimplemented) |
| 17 | `0x11` | GS → MC | GS_MSG_GPS | variable (unimplemented) |
| 18 | `0x12` | GS → MC | GS_MSG_EVENT | variable (unimplemented) |
| 19 | `0x13` | GS → MC | GS_MSG_STATUS | 24 bytes |
| 20 | `0x14` | GS → MC | GS_MSG_CORRUPT | variable (unimplemented) |
| 128 | `0x80` | MC/GS → FC | CMD_ARM | 12 bytes |
| 129 | `0x81` | MC/GS → FC | CMD_FIRE | 13 bytes |
| 130 | `0x82` | MC/GS → FC | CMD_TESTMODE | variable |
| 131 | `0x83` | MC/GS → FC | CMD_POLL | variable |
| 160 | `0xA0` | FC → MC/GS | ACK_ARM | 12 bytes |
| 161 | `0xA1` | FC → MC/GS | ACK_FIRE | 13 bytes |
| 163 | `0xA3` | FC → MC/GS | ACK_CONFIG | 13 bytes |
| 192 | `0xC0` | Bidirectional | HANDSHAKE | 1 (req) / 13 (resp) |
| 194 | `0xC2` | MC → FC | DIAG | 32 bytes |
| 195 | `0xC3` | MC → FC | READLOG | variable |
| 196 | `0xC4` | MC → FC | ERASELOG | variable |
| 208 | `0xD0` | MC → FC | SIM_FLIGHT | 5 bytes |
| 209 | `0xD1` | MC → FC | HIL_INJECT | 44 bytes |
| 210 | `0xD2` | MC → FC | DUMP_FLASH | variable |
| 224 | `0xE0` | FC → MC/GS | NACK | 10 bytes |
| 240 | `0xF0` | MC/GS → FC | CONFIRM | 9 bytes |
| 241 | `0xF1` | MC/GS → FC | ABORT | 9 bytes |

**Magic bytes** (used in safety-critical commands): `MAGIC_1 = 0xCA`, `MAGIC_2 = 0x5A`

---

## 6. FC Telemetry Messages (0x01–0x03)

### 6.1 FC_MSG_FAST (0x01) — High-Rate Telemetry

**Size:** 21 bytes
**CRC coverage:** bytes [0–16], CRC at [17–20]

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x01` |
| 1–2 | status | u16 LE | — | bitmap | FC telemetry status (see §6.4) |
| 3–5 | altitude | u24 LE | × 0.01 | m | Altitude AGL (1 cm resolution, max 167,772 m) |
| 6–7 | velocity | i16 LE | × 0.1 | m/s | Velocity (signed, +up) |
| 8–12 | quaternion | 5 bytes | — | — | Smallest-three packed (see §6.5) |
| 13–14 | flight_time | u16 LE | × 0.1 | s | Mission elapsed time |
| 15 | battery | u8 | 6.0 + raw × 0.012 | V | Battery voltage |
| 16 | seq | u8 | — | — | Rolling sequence counter |
| 17–20 | crc32 | u32 LE | — | — | CRC-32 over [0–16] |

### 6.2 FC_MSG_GPS (0x02) — GPS Position

**Size:** 18 bytes
**CRC coverage:** bytes [0–13], CRC at [14–17]

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x02` |
| 1–4 | dlat_mm | i32 LE | ÷ 1000 | m | Delta latitude from pad origin |
| 5–8 | dlon_mm | i32 LE | ÷ 1000 | m | Delta longitude from pad origin |
| 9–11 | alt_msl | u24 LE | × 0.01 | m | GPS altitude MSL (1 cm resolution, max 167,772 m) |
| 12 | fix_type | u8 | — | — | 0=none, 2=2D, 3=3D |
| 13 | sat_count | u8 | — | — | Satellites in use |
| 14–17 | crc32 | u32 LE | — | — | CRC-32 over [0–13] |

**Range saturation:** If `dlat_mm` or `dlon_mm` equals `±0x7FFFFFFF`, the delta has overflowed.

### 6.3 FC_MSG_EVENT (0x03) — Discrete Event

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

### 6.4 FC Telemetry Status Bitmap (16-bit LE)

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

### 6.5 Quaternion Encoding (Smallest-Three, 5 bytes / 40 bits)

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

### 6.6 Flight State Machine (FSM) States

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

### 6.7 GPS Subsystem (FC ↔ MAX-M10M)

This section documents the internal GPS interface — what the FC sends to and receives from the u-blox MAX-M10M module. For the downlink packet format, see §6.2 (FC_MSG_GPS).

#### 6.7.1 Hardware Interface

| Parameter | Value |
|---|---|
| Module | u-blox MAX-M10M |
| Bus | I2C1 (u-blox DDC protocol) |
| 7-bit address | `0x42` |
| Reset pin | NRST_GPS (PE15), active-low, 10 ms pulse + 1 s boot |
| EXTI pin | PE0 (allocated, stub only — driver uses polling) |
| Source files | `App/drivers/max_m10m.c`, `App/drivers/max_m10m.h` |

#### 6.7.2 FC → GPS: Init Configuration

All 13 UBX-CFG-VALSET commands sent during `max_m10m_init()`:

| # | Config Key | Key ID | Value | Size | Purpose |
|---|---|---|---|---|---|
| 1 | CFG_MSGOUT_NMEA_GGA_I2C | `0x209100BA` | 0 | U1 | Disable NMEA GGA |
| 2 | CFG_MSGOUT_NMEA_GLL_I2C | `0x209100C9` | 0 | U1 | Disable NMEA GLL |
| 3 | CFG_MSGOUT_NMEA_GSA_I2C | `0x209100BF` | 0 | U1 | Disable NMEA GSA |
| 4 | CFG_MSGOUT_NMEA_GSV_I2C | `0x209100C4` | 0 | U1 | Disable NMEA GSV |
| 5 | CFG_MSGOUT_NMEA_RMC_I2C | `0x209100AB` | 0 | U1 | Disable NMEA RMC |
| 6 | CFG_MSGOUT_NMEA_VTG_I2C | `0x209100B0` | 0 | U1 | Disable NMEA VTG |
| 7 | CFG_MSGOUT_UBX_NAV_PVT_I2C | `0x20910007` | 1 | U1 | Enable NAV-PVT every epoch |
| 8 | CFG_RATE_MEAS | `0x30210001` | 100 | U2 | 10 Hz measurement rate |
| 9 | CFG_SIGNAL_GPS_ENA | `0x1031001F` | 1 | U1 | Enable GPS constellation |
| 10 | CFG_SIGNAL_GAL_ENA | `0x10310021` | 0 | U1 | Disable Galileo |
| 11 | CFG_SIGNAL_BDS_ENA | `0x10310022` | 0 | U1 | Disable BeiDou |
| 12 | CFG_SIGNAL_GLO_ENA | `0x10310025` | 0 | U1 | Disable GLONASS |
| 13 | CFG_SIGNAL_QZSS_ENA | `0x10310024` | 0 | U1 | Disable QZSS |

**Notes:**
- All config written to **RAM only** (layer byte = `0x01`). Configuration is lost on power cycle.
- GPS-only constellation is required for 10 Hz on the M10 platform.
- Each command waits for UBX-ACK-ACK/NAK with 200 ms timeout.
- **Not configured:** Dynamic platform model — module defaults to "Portable" mode (12 km altitude cap, ~500 m/s velocity cap). Airborne <4g (`CFG_NAVSPG_DYNMODEL` key `0x20110021`, value 8) should be added before flight.

#### 6.7.3 GPS → FC: UBX-NAV-PVT Fields Received

UBX-NAV-PVT payload is 92 bytes, received at 10 Hz. The driver parses the following fields:

| UBX Offset | Field | Type | Unit | Driver Field | Downstream Use |
|---|---|---|---|---|---|
| 0–3 | iTOW | u32 LE | ms (GPS week) | `iTOW` | Parsed, unused |
| 4–5 | year | u16 LE | — | `year` | Parsed, not downlinked |
| 6 | month | u8 | — | `month` | Parsed, not downlinked |
| 7 | day | u8 | — | `day` | Parsed, not downlinked |
| 8 | hour | u8 | — | `hour` | Parsed, not downlinked |
| 9 | min | u8 | — | `min` | Parsed, not downlinked |
| 10 | sec | u8 | — | `sec` | Parsed, not downlinked |
| 11 | valid | u8 | bitfield | `valid_flags` | Parsed, not downlinked |
| 20 | fixType | u8 | enum | `fix_type` | → FC_MSG_GPS[12]; gates EKF GPS calls |
| 23 | numSV | u8 | count | `num_sv` | → FC_MSG_GPS[13] |
| 24–27 | lon | i32 LE | deg × 10⁻⁷ | `lon_deg7` → `lon_deg` (float) | → FC_MSG_GPS[5–8] as dlon_mm delta |
| 28–31 | lat | i32 LE | deg × 10⁻⁷ | `lat_deg7` → `lat_deg` (float) | → FC_MSG_GPS[1–4] as dlat_mm delta |
| 36–39 | hMSL | i32 LE | mm | `h_msl_mm` → `alt_msl_m` (÷1000) | → FC_MSG_GPS[9–11]; passed to EKF stub (no-op) |
| 40–43 | hAcc | u32 LE | mm | `h_acc_mm` | Parsed, not downlinked |
| 44–47 | vAcc | u32 LE | mm | `v_acc_mm` | Parsed, not downlinked |
| 48–51 | velN | i32 LE | mm/s | `vel_n_mm_s` | Parsed, not downlinked |
| 52–55 | velE | i32 LE | mm/s | `vel_e_mm_s` | Parsed, not downlinked |
| 56–59 | velD | i32 LE | mm/s | `vel_d_mm_s` → `vel_d_m_s` (÷1000) | Passed to EKF stub (no-op) |
| 76–77 | pDOP | u16 LE | × 0.01 | `pDOP` | Parsed, not downlinked |

**fixType values:** 0 = No fix, 1 = Dead reckoning, 2 = 2D, 3 = 3D, 4 = GNSS+DR, 5 = Time only

**NAV-PVT fields NOT parsed** (present in 92-byte payload but skipped): tAcc, nano, flags, flags2, flags3, headMot, sAcc, headAcc, headVeh, magDec, magAcc, height (ellipsoid), gSpeed.

#### 6.7.4 Polling Architecture

- `max_m10m_tick()` is called every main loop iteration (~416 Hz)
- 25 ms poll interval (40 Hz effective) — reads bytes-available register (`0xFD:0xFE`)
- Data read from stream register (`0xFF`) in 64-byte chunks per tick
- UBX frame parser: 9-state FSM (`SYNC1 → SYNC2 → CLASS → ID → LEN1 → LEN2 → PAYLOAD → CK_A → CK_B`)
- Fletcher checksum verified on every complete frame
- Parse buffer: 100 bytes (fits 92-byte NAV-PVT; too small for NAV-SAT with >7 SVs)
- Returns 1 to caller when a complete NAV-PVT has been parsed

#### 6.7.5 EKF Integration

On each new NAV-PVT, **if** fixType ≥ 3D AND calibration complete (`flight_loop.c:506–514`), the flight loop calls:
- `casper_ekf_update_gps_alt(&ekf, gps.alt_msl_m)`
- `casper_ekf_update_gps_vel(&ekf, gps.vel_d_m_s)`

**Both functions are stubs** (`casper_ekf.c:350–362`) — they cast all parameters to `(void)` and return immediately. GPS data does **not** currently influence the EKF state. The TODO comments read "enable when GPS antenna populated."

Horizontal position/velocity (velN, velE, lat, lon) are not fed to the EKF (vertical-only 4-state design).

#### 6.7.6 GPS → Telemetry Path (NOT YET WIRED)

The telemetry pipeline for GPS is designed but **not connected** in the current build:

1. **Pad origin capture:** `flight_config_t` has `pad_lat_deg`, `pad_lon_deg`, `pad_alt_m` — intended to be set when GPS first locks on the pad. `FC_EVT_ORIGIN` (`0x05`) is defined in `tlm_types.h` to announce this event.

2. **Delta computation:** `fc_gps_state_t` has `dlat_mm` and `dlon_mm` (millimetres from pad). Intended transform:
   ```
   dlat_mm = (lat_deg7 - pad_lat_deg7) × 0.0111320   [deg×1e-7 → mm, using 111.32 km/deg]
   dlon_mm = (lon_deg7 - pad_lon_deg7) × 0.0111320 × cos(pad_lat)
   ```

3. **Packet build:** `tlm_send_gps()` and `radio_send_gps()` pack `fc_gps_state_t` into the 18-byte FC_MSG_GPS format (§6.2).

4. **Missing wiring:** No code in `flight_loop.c` currently:
   - Captures pad origin from GPS
   - Computes dlat_mm / dlon_mm deltas from raw lat_deg7 / lon_deg7
   - Calls `tlm_send_gps()` or `radio_send_gps()`

> **Note:** The flight logger (`flight_logger.c:377–378`) stores raw `lat_deg7` / `lon_deg7` into fields named `gps_dlat_mm` / `gps_dlon_mm` — this is a naming mismatch (absolute position stored in delta-named fields).

#### 6.7.7 M10M Supported Messages (Not Currently Used)

The MAX-M10M supports these additional UBX messages that are not currently enabled:

| UBX Message | Payload Size | M10 Support | Status |
|---|---|---|---|
| NAV-PVT | 92 B (fixed) | Yes | **Active** — parsed at 10 Hz |
| NAV-SAT | 8 + 12×N B (variable) | Yes | Not enabled. Per-SV signal info. Needs parse buffer >100 B for >7 SVs |
| NAV-STATUS | 16 B (fixed) | Yes | Not enabled. TTFF + spoofing/jamming detection flags |
| NAV-DOP | 18 B (fixed) | Yes | Not enabled. Full DOP breakdown (GDOP, PDOP, HDOP, VDOP, TDOP) |
| MON-RF | 4 + 24×N B (variable) | Yes | Not enabled. RF jamming/interference indicator |
| MON-VER | variable | Yes | Key ID defined in header but never requested/parsed |
| TIM-TP | — | **No** | Not supported on the M10 platform |

---

## 7. GS Relay Messages (0x10–0x14)

### 7.1 GS_MSG_TELEM (0x10) — Ground Station Telemetry Relay

> **Status:** Unimplemented / aspirational. No firmware code currently produces or consumes this packet. Defined for future Mission Control integration where the GS would repackage FC telemetry with added link-quality metadata before relaying to MC over USB.

**Size:** 39 bytes
**CRC coverage:** bytes [0–34], CRC at [35–38]

Contains all FC_MSG_FAST fields plus GS-added radio link quality and derived values.

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x10` |
| 1–2 | status | u16 LE | — | bitmap | Same as FC_MSG_FAST |
| 3–5 | altitude | u24 LE | × 0.01 | m | Altitude AGL |
| 6–7 | velocity | i16 LE | × 0.1 | m/s | Velocity |
| 8–12 | quaternion | 5 bytes | — | — | Smallest-three |
| 13–14 | flight_time | u16 LE | × 0.1 | s | Mission elapsed time |
| 15 | battery | u8 | 6.0 + raw × 0.012 | V | Battery voltage |
| 16 | seq | u8 | — | — | GS sequence number |
| 17–18 | rssi | i16 LE | × 0.1 | dBm | Received signal strength |
| 19 | snr | i8 | × 0.25 | dB | Signal-to-noise ratio |
| 20–21 | freq_err | i16 LE | × 1 | Hz | Frequency error |
| 22–23 | data_age | u16 LE | × 1 | ms | Time since last valid FC packet |
| 24 | recovery | u8 | — | bitmap | See below |
| 25–26 | mach | u16 LE | × 0.001 | — | Mach number |
| 27–28 | qbar | u16 LE | × 1 | Pa | Dynamic pressure |
| 29–30 | roll | i16 LE | × 0.1 | deg | Roll angle |
| 31–32 | pitch | i16 LE | × 0.1 | deg | Pitch angle |
| 33–34 | yaw | i16 LE | × 0.1 | deg | Yaw angle |
| 35 | reserved | u8 | — | — | — |
| 36–39 | crc32 | u32 LE | — | — | CRC-32 over [0–35] |

**Recovery byte (offset 24):**
- Bit 7: `recovered` — packet was error-corrected
- Bits 6:4: `method` — correction method code
- Bits 3:0: `confidence` — correction confidence (0–15)

### 7.2 GS_MSG_GPS (0x11), GS_MSG_EVENT (0x12), GS_MSG_CORRUPT (0x14)

> **Status:** Unimplemented. Reserved for future use. Parser stores raw bytes for forward compatibility.

### 7.3 GS_MSG_STATUS (0x13) — Ground Station Status

**Size:** 24 bytes
**CRC coverage:** bytes [0–19], CRC at [20–23]
**Direction:** GS → MC (over USB/COBS)

The GS periodically sends its own status to Mission Control. This packet is fully defined in firmware (`gs_msg_status_t` in `tlm_types.h`).

| Offset | Field | Type | Scale | Unit | Description |
|---|---|---|---|---|---|
| 0 | msg_id | u8 | — | — | `0x13` |
| 1 | radio_profile | u8 | — | enum | `0` = Profile A (SF7), `1` = Profile B (SF8) |
| 2 | last_rssi | i8 | — | dBm | RSSI of last received FC packet |
| 3 | last_snr | i8 | — | dB | SNR of last received FC packet (signed) |
| 4–5 | rx_pkt_count | u16 LE | — | count | Total valid packets received |
| 6–7 | rx_crc_fail | u16 LE | — | count | Total CRC failures (HW + SW) |
| 8–11 | ground_pressure_pa | u32 LE | — | Pa | Ground-level barometric pressure |
| 12–15 | ground_lat_1e7 | i32 LE | × 10⁻⁷ | deg | Ground station latitude (UBX encoding) |
| 16–19 | ground_lon_1e7 | i32 LE | × 10⁻⁷ | deg | Ground station longitude (UBX encoding) |
| 20–23 | crc32 | u32 LE | — | — | CRC-32 over [0–19] |

---

## 8. Command Messages (MC → FC)

All safety-critical commands use the CAC (Command-Acknowledge-Confirm) protocol (see §10).

### 8.1 CMD_ARM (0x80)

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

### 8.2 CMD_FIRE (0x81)

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

### 8.3 CONFIRM (0xF0)

**Size:** 9 bytes
**CRC coverage:** bytes [0–4], CRC at [5–8]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xF0` |
| 1 | magic_1 | u8 | `0xCA` |
| 2 | magic_2 | u8 | `0x5A` |
| 3–4 | nonce | u16 LE | Nonce from ACK |
| 5–8 | crc32 | u32 LE | CRC-32 over [0–4] |

### 8.4 ABORT (0xF1)

**Size:** 9 bytes (identical layout to CONFIRM, different msg_id)

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xF1` |
| 1 | magic_1 | u8 | `0xCA` |
| 2 | magic_2 | u8 | `0x5A` |
| 3–4 | nonce | u16 LE | Transaction nonce |
| 5–8 | crc32 | u32 LE | CRC-32 over [0–4] |

---

## 9. Response Messages (FC → MC)

### 9.1 ACK_ARM (0xA0)

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

### 9.2 ACK_FIRE (0xA1)

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

### 9.3 NACK (0xE0)

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

### 9.4 ACK_CONFIG (0xA3)

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

## 10. Handshake & System Messages

### 10.1 HANDSHAKE Request (MC → FC)

**Size:** 1 byte (no CRC)

| Offset | Field | Type |
|---|---|---|
| 0 | msg_id | u8 (`0xC0`) |

Sent immediately after USB serial connection is opened.

### 10.2 HANDSHAKE Response (FC → MC)

**Size:** 13 bytes (fixed)
**CRC coverage:** bytes [0–8], CRC at [9–12]

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | msg_id | u8 | `0xC0` |
| 1 | protocol_version | u8 | FC protocol version (must be 5) |
| 2 | fw_version_major | u8 | Firmware major version |
| 3 | fw_version_minor | u8 | Firmware minor version |
| 4 | fw_version_patch | u8 | Firmware patch version |
| 5–8 | config_hash | u32 LE | CRC-32 hash of active flight configuration |
| 9–12 | crc32 | u32 LE | CRC-32 over [0–8] |

### 10.3 SIM_FLIGHT (MC → FC)

**Size:** 5 bytes
**CRC coverage:** bytes [0–0], CRC at [1–4]

| Offset | Field | Type |
|---|---|---|
| 0 | msg_id | u8 (`0xD0`) |
| 1–4 | crc32 | u32 LE |

Triggers simulated flight on the FC (bench testing only).

---

## 11. CAC State Machine

The CAC (Command-Acknowledge-Confirm) protocol ensures safety-critical commands (ARM, DISARM, FIRE) are reliably delivered and verified.

### 11.1 Phases

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

### 11.2 Timing

| Parameter | Value |
|---|---|
| Leg timeout (per retry) | 2000 ms |
| Overall timeout (entire exchange) | 10000 ms |
| Max retries | 10 |
| Confirm delay (after echo verified) | 1000 ms |

### 11.3 Echo Verification

**ARM/DISARM:** ACK must echo the exact `channel` and `action` from the request.
**FIRE:** ACK must echo the exact `channel` and `duration` from the request.
**Mismatch:** MC sends ABORT and transitions to FAILED.

### 11.4 Telemetry-as-Parallel-ACK

While awaiting ACK for an ARM command, if an FC telemetry status bitmap arrives showing the target channel's armed state matches the requested state, the CAC machine advances to VERIFYING_ACK using telemetry as the echo source. This handles cases where the ACK packet was lost but the FC did act on the command.

### 11.5 Nonce Generation

Each CAC exchange uses a random 16-bit nonce (`0x0000`–`0xFFFF`). The same nonce is used for CMD, ACK matching, CONFIRM, and ABORT within one exchange. Generated via `crypto.getRandomValues()` when available, else `Math.random()`.

### 11.6 Busy Reset

If the CAC machine is stuck (e.g., waiting for an ACK that never arrives), it is automatically reset before accepting a new command from the UI. This prevents the user from being locked out during bench testing.

### 11.7 UI State Exposed

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

## 12. Telemetry Store

The telemetry store maintains a single `TelemetrySnapshot` object updated by incoming packets and pushed to the renderer on every change.

### 12.1 Snapshot Fields

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
| `alt_m` | number | 0 | metres | altitude × 0.01 |
| `vel_mps` | number | 0 | m/s | velocity × 0.1 |
| `quat` | [n,n,n,n] | [1,0,0,0] | — | smallest-three decode |
| `roll_deg` | number | 0 | degrees | derived from quat |
| `pitch_deg` | number | 0 | degrees | derived from quat |
| `yaw_deg` | number | 0 | degrees | derived from quat |
| `mach` | number | 0 | — | derived (ISA model) |
| `qbar_pa` | number | 0 | Pa | derived (exp density) |
| `batt_v` | number | 0 | V | 6.0 + raw × 0.012 |
| `fsm_state` | number | 0 | enum | status byte 1, bits 7:4 |
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
| `gps_pdop` | number | 0 | — | **Unpopulated.** pDOP is parsed from NAV-PVT (see §6.7.3) but not included in the 18-byte FC_MSG_GPS packet. Reserved for future expansion. |
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

### 12.2 Ring Buffers

- **Depth:** 150 samples
- **Fields buffered:** `buf_alt`, `buf_vel`, `buf_qbar`
- **Behaviour:** FIFO — oldest dropped when depth exceeded

### 12.3 Stale Detection

- **Threshold:** 500 ms
- **Tick rate:** 100 ms (main-process interval)
- **Logic:** If `(now - last_valid_packet_time) > 500ms`, mark `stale = true`
- **Reset:** Any valid FC_MSG_FAST or GS_MSG_TELEM clears stale

### 12.4 Subscriber Pattern

```typescript
subscribe(callback: (snapshot: TelemetrySnapshot) => void): () => void
```

Every store update calls all subscribers with an isolated (shallow-copied) snapshot. The IPC layer subscribes to push snapshots to the renderer via `webContents.send()`.

---

## 13. IPC Channels & Handlers

### 13.1 Main → Renderer (Push)

| Channel | Payload | Trigger |
|---|---|---|
| `casper:telemetry` | TelemetrySnapshot | Every store update |
| `casper:cac-update` | CacUiState | CAC phase change |
| `casper:diag-result` | Diagnostic result | After self-test |
| `casper:serial-ports` | PortInfo[] | After port scan |

### 13.2 Renderer → Main (Invoke, returns Promise)

| Channel | Arguments | Returns | Action |
|---|---|---|---|
| `casper:connect-fc` | port: string | void | Open FC serial, send handshake |
| `casper:connect-gs` | port: string | void | Open GS serial |
| `casper:upload-config` | config: FlightConfig | {ok, hash?, error?} | Serialize + send config |
| `casper:verify-config-hash` | — | {ok, fc_hash?, verified?} | Compare config hashes |
| `casper:download-flight-log` | — | Uint8Array | Download log (stub) |

### 13.3 Renderer → Main (Send, fire-and-forget)

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

## 14. Preload Bridge API (`window.casper`)

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

## 15. Frontend Hooks & UI

### 15.1 useTelemetry()

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

### 15.2 useCommand()

Subscribes to `window.casper.on_cac_update()`. Returns: `{ busy, command_type, target_channel, error, nack_code, retry_count, abort }`.

### 15.3 useSerial()

Subscribes to `window.casper.on_serial_ports()` and `on_telemetry()` (for connection flags). Returns: `{ ports, fc_connected, gs_connected, scan, connect_fc, connect_gs, disconnect_fc, disconnect_gs }`.

### 15.4 useDiagnostics()

7 built-in tests: IMU, Magnetometer, Barometer, EKF Init, Attitude, Flash, Config. Returns: `{ tests, runAll, reset }`.

### 15.5 UI Tabs

| Tab | Icon | Purpose |
|---|---|---|
| **SETUP** | ⚙ | Serial port picker, sensor diagnostics, pyro config, config upload |
| **TEST** | ⚡ | Bench testing: live telemetry summary, pyro arm/fire controls, CAC status, SIM FLIGHT |
| **FLIGHT** | ▲ | Real-time flight monitoring: GPS, altitude, velocity, graphs, pyro status, pre-flight checklist, terminal countdown, 3D orientation, vertical state bar |
| **TRACKING** | ◎ | 3D attitude canvas + ground track radar |

**Connection gating:** Flight and Test tabs show live data when **either** FC or GS is connected (`connected = fcConn || gsConn`).

### 15.6 Pre-Flight Checks

| Check | Condition | Configurable |
|---|---|---|
| Battery Voltage | `batt >= minBatt` (default 7.4V) | Yes |
| GPS Fix | `gpsFix === "3D" && gpsSats >= 6` | No |
| Pyro Continuity | All non-Custom channels have continuity | No |
| IMU Health | `|pitch − 90| < 15°` (vertical) | No |
| Radio Link | `!stale && dataAge < 500ms` | No |
| Data Integrity | `integrity >= minIntegrity` (default 90%) | Yes |

### 15.7 Flight State TTS Callouts

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

## 16. Flight Configuration Format

### 16.1 Binary Serialization

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

## 17. Recovery Pipeline

### 17.1 Stage 1 — Single-Bit CRC Correction

Corrects single-bit errors in telemetry packets using CRC syndrome lookup.

1. Compute CRC of received payload
2. XOR with received CRC → syndrome
3. If syndrome = 0: packet valid, no correction needed
4. Look up syndrome in precomputed table (one entry per bit position)
5. If found: flip the identified bit, verify CRC, return corrected packet
6. If not found: multi-bit corruption, pass to next stage

**Syndrome tables are cached per payload length.**

### 17.2 Stage 3 — Temporal Interpolation

**Status:** Stub (returns null). Future: Kalman-based prediction from historical data.

### 17.3 Stage 4 — Zero-Order Hold

Repeats last known-good telemetry values during communication gaps. Tracks staleness duration for UI display. Threshold: 500 ms.

---

## 18. Derived Computations

### 18.1 Mach Number

ISA standard atmosphere model:
- Sea-level temp: 288.15 K
- Lapse rate: 0.0065 K/m (below 11 km)
- Tropopause temp: 216.65 K (above 11 km)
- Speed of sound: `a = √(γ × R × T)` where γ=1.4, R=287.05 J/(kg·K)
- Mach = `|velocity| / a`

### 18.2 Dynamic Pressure (q̄)

Exponential density model:
- Sea-level density: 1.225 kg/m³
- Scale height: 8500 m
- `ρ = 1.225 × e^(-alt/8500)`
- `q̄ = 0.5 × ρ × v²`

### 18.3 Euler Angles

Aerospace convention (ZYX rotation) from quaternion [w,x,y,z]:
- Roll: `atan2(2(wx+yz), 1−2(x²+y²))`
- Pitch: `asin(clamp(2(wy−zx), −1, 1))`
- Yaw: `atan2(2(wz+xy), 1−2(y²+z²))`

---

## 19. Ground Station Implementation Guide

This section provides implementation details for building a ground station that communicates with the C.A.S.P.E.R.-2 FC over LoRa radio. It is written for a developer (or AI agent) building GS software from scratch.

### 19.1 SX1276 Configuration

The GS must configure its SX1276 to match the FC exactly. Any mismatch in spreading factor, bandwidth, sync word, or frequency will prevent communication.

**Required register configuration (after LoRa mode set):**

| Register | Value | Purpose |
|---|---|---|
| RegOpMode | `0x80` | Sleep + LoRa mode |
| RegFrMsb/Mid/Lsb | `0xD9/0x00/0x00` | 868.000 MHz (`868e6 / 61.035`) |
| RegModemConfig1 | `0x82` | BW=250kHz, CR=4/5, explicit header |
| RegModemConfig2 | `0x74` (Profile A) / `0x84` (Profile B) | SF7/SF8, CRC on, normal RX timeout |
| RegModemConfig3 | `0x04` | LNA gain auto, low-data-rate optimize off |
| RegSyncWord | `0x12` | Private LoRa network sync word |
| RegPreambleMsb/Lsb | `0x00/0x08` | 8-symbol preamble |
| RegPaConfig | `0xFF` | PA_BOOST, max power, +20 dBm |
| RegPaDac | `0x87` | High-power +20 dBm mode |
| RegDioMapping1 | `0x01` | DIO0=RxDone/TxDone, DIO1=RxTimeout |
| RegDioMapping2 | `0xA0` | DIO4=PllLock, DIO5=ModeReady |
| RegFifoTxBaseAddr | `0x00` | TX FIFO base |
| RegFifoRxBaseAddr | `0x00` | RX FIFO base |

### 19.2 Recommended GS Architecture

```
┌─────────────────────────────────────────────────────┐
│  Ground Station MCU                                  │
│                                                      │
│  ┌──────────────┐                                    │
│  │  SX1276      │  RX-continuous mode                │
│  │  LoRa Radio  │  (always listening)                │
│  └──────┬───────┘                                    │
│         │                                            │
│  ┌──────┴───────┐   ┌─────────────────┐             │
│  │ Radio RX/TX  │───│ Packet Parser   │             │
│  │ Handler      │   │ (CRC validate,  │             │
│  │              │   │  decode fields)  │             │
│  └──────────────┘   └────────┬────────┘             │
│                              │                       │
│  ┌───────────────────────────┴──────────────────┐   │
│  │ USB CDC Output (ASCII or COBS to MC)         │   │
│  └──────────────────────────────────────────────┘   │
│                                                      │
│  ┌──────────────────────────────────────────────┐   │
│  │ Command Relay (USB CDC RX → LoRa TX)         │   │
│  │ Wait for FC RX window after receiving packet  │   │
│  └──────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

**Key design constraints:**
1. **RX-continuous mode:** The GS should remain in RXCONTINUOUS at all times (not RXSINGLE). The FC transmits at 10 Hz and the GS needs to receive every packet.
2. **TX timing:** To send a command, the GS must switch from RX to TX. Since the FC only listens for 80 symbols after its own TX, the GS should transmit immediately after receiving an FC packet (within ~10 ms). After TX completes (TxDone), return to RXCONTINUOUS.
3. **Profile switching:** Start on Profile A. If no valid packets for 2000 ms, switch to Profile B (one-way).
4. **No COBS:** The radio link is raw packets. COBS is only used on the USB link between GS and MC.

### 19.3 RSSI and SNR Calculation

**RSSI (HF band, >862 MHz):**
```
rssi_dBm = packet_rssi_register - 157
```
Where `packet_rssi_register` is the value read from SX1276 `RegPktRssiValue` (0x1A).

**SNR:**
```
snr_dB = packet_snr_register / 4.0
```
Where `packet_snr_register` is the signed value read from `RegPktSnrValue` (0x19).

### 19.4 Decoding FC Packets

On receiving a valid LoRa packet (DIO0 RxDone, no HW CRC error):

1. Read `RegRxNbBytes` for packet length
2. Read `RegFifoRxCurrentAddr`, set `RegFifoAddrPtr`, read FIFO
3. Validate minimum length (5 bytes: 1 ID + 4 CRC)
4. Compute CRC-32 over bytes `[0 .. len-5]`, compare with last 4 bytes (LE)
5. If CRC passes, dispatch by `byte[0]` (msg_id):
   - `0x01`: FC_MSG_FAST (21 bytes) — decode per §6.1
   - `0x02`: FC_MSG_GPS (18 bytes) — decode per §6.2
   - `0x03`: FC_MSG_EVENT (11 bytes) — decode per §6.3
   - `0xA0/0xA1/0xA3/0xE0`: ACK/NACK responses — relay to MC

### 19.5 Altitude Decoding Example

```c
/* FC_MSG_FAST altitude at bytes [3-5], u24 LE */
uint32_t alt_raw = (uint32_t)buf[3]
                 | ((uint32_t)buf[4] << 8)
                 | ((uint32_t)buf[5] << 16);
float alt_m = (float)alt_raw * 0.01f;  /* ALT_SCALE_M */

/* FC_MSG_GPS altitude at bytes [9-11], u24 LE */
uint32_t gps_alt_raw = (uint32_t)buf[9]
                     | ((uint32_t)buf[10] << 8)
                     | ((uint32_t)buf[11] << 16);
float gps_alt_m = (float)gps_alt_raw * 0.01f;
```

### 19.6 Sending Commands to FC

To send a command (e.g., CMD_ARM):
1. Build the packet per §8 (same format as USB, with magic bytes and CRC-32)
2. Switch SX1276 from RXCONTINUOUS to STDBY
3. Write packet to FIFO, set payload length, switch to TX mode
4. Wait for TxDone (DIO0), then return to RXCONTINUOUS
5. The FC will respond with ACK/NACK on the next TX slot (within ~100 ms)

**Timing constraint:** The FC only opens a ~80-symbol RX window after each TX. For best results, queue the command and transmit immediately after receiving the next FC packet.

---

## 20. Appendices

### A. Byte Layout Quick Reference

```
FC_MSG_FAST (21 bytes):
  [0]     0x01
  [1-2]   status (u16 LE)
  [3-5]   altitude (u24 LE, ×0.01 → m, max 167,772 m)
  [6-7]   velocity (i16 LE, ×0.1 → m/s)
  [8-12]  quaternion (5 bytes, smallest-three)
  [13-14] flight_time (u16 LE, ×0.1 → s)
  [15]    battery (u8, 6.0 + raw×0.012 → V)
  [16]    seq (u8, rolling counter)
  [17-20] CRC-32 (u32 LE)

FC_MSG_GPS (18 bytes):
  [0]     0x02
  [1-4]   dlat_mm (i32 LE, ÷1000 → m)
  [5-8]   dlon_mm (i32 LE, ÷1000 → m)
  [9-11]  alt_msl (u24 LE, ×0.01 → m, max 167,772 m)
  [12]    fix_type (u8)
  [13]    sat_count (u8)
  [14-17] CRC-32 (u32 LE)

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

HANDSHAKE response (13 bytes):
  [0]     0xC0
  [1]     protocol_version (u8, must be 5)
  [2]     fw_version_major (u8)
  [3]     fw_version_minor (u8)
  [4]     fw_version_patch (u8)
  [5-8]   config_hash (u32 LE)
  [9-12]  CRC-32 (u32 LE)

SIM_FLIGHT (5 bytes):
  [0]     0xD0
  [1-4]   CRC-32 (u32 LE)

GS_MSG_TELEM (39 bytes) [UNIMPLEMENTED]:
  [0]     0x10
  [1-2]   status (u16 LE)
  [3-5]   altitude (u24 LE, ×0.01 → m)
  [6-7]   velocity (i16 LE)
  [8-12]  quaternion (5 bytes)
  [13-14] flight_time (u16 LE)
  [15]    battery (u8)
  [16]    seq (u8)
  [17-18] rssi (i16 LE, ×0.1 → dBm)
  [19]    snr (i8, ×0.25 → dB)
  [20-21] freq_err (i16 LE → Hz)
  [22-23] data_age (u16 LE → ms)
  [24]    recovery byte
  [25-26] mach (u16 LE, ×0.001)
  [27-28] qbar (u16 LE → Pa)
  [29-30] roll (i16 LE, ×0.1 → deg)
  [31-32] pitch (i16 LE, ×0.1 → deg)
  [33-34] yaw (i16 LE, ×0.1 → deg)
  [35]    reserved
  [36-39] CRC-32 (u32 LE)

GS_MSG_STATUS (24 bytes):
  [0]     0x13
  [1]     radio_profile (0=A/SF7, 1=B/SF8)
  [2]     last_rssi (i8, dBm)
  [3]     last_snr (i8, dB)
  [4-5]   rx_pkt_count (u16 LE)
  [6-7]   rx_crc_fail (u16 LE)
  [8-11]  ground_pressure_pa (u32 LE, Pa)
  [12-15] ground_lat_1e7 (i32 LE, deg × 10^7)
  [16-19] ground_lon_1e7 (i32 LE, deg × 10^7)
  [20-23] CRC-32 (u32 LE)
```

### B. Scaling Factor Summary

| Raw Field | Type | Formula | Result Unit | Firmware Constant |
|---|---|---|---|---|
| Altitude | u24 LE | `raw × 0.01` | metres | `ALT_SCALE_M = 0.01f` |
| Velocity | i16 LE | `raw × 0.1` | m/s | `VEL_SCALE_DMS = 0.1f` |
| Flight time | u16 LE | `raw × 0.1` | seconds | `TIME_SCALE_100MS = 0.1f` |
| Battery | u8 | `6.0 + raw × 0.012` | volts | `BATT_OFFSET_V`, `BATT_STEP_V` |
| GPS altitude | u24 LE | `raw × 0.01` | metres MSL | `ALT_SCALE_M = 0.01f` |
| GPS delta lat/lon | i32 LE | `raw / 1000` | metres | — |
| RSSI (GS_MSG_TELEM) | i16 LE | `raw × 0.1` | dBm | — |
| SNR (GS_MSG_TELEM) | i8 | `raw × 0.25` | dB | — |
| RSSI (radio raw) | i8 | `register - 157` | dBm | SX1276 HF formula |
| SNR (radio raw) | i8 | `register / 4.0` | dB | SX1276 formula |
| Mach | u16 LE | `raw × 0.001` | — | — |
| Roll/Pitch/Yaw | i16 LE | `raw × 0.1` | degrees | — |
| pDOP (NAV-PVT internal) | u16 LE | `raw × 0.01` | dimensionless | Parsed by driver, not downlinked |

### C. End-to-End Data Flow

**USB Direct Path (FC → MC):**
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
store.subscribe() callback → webContents.send() → React UI
```

**Radio Path (FC → GS → MC):**
```
FC Radio Manager
  │  SX1276 TX (10 Hz, raw LoRa packets, no COBS)
  ▼
868 MHz LoRa RF Link
  │  Profile A (SF7) or Profile B (SF8)
  │  LoRa HW CRC + SW CRC-32
  ▼
GS SX1276 RX (RXCONTINUOUS)
  │  DIO0 RxDone → read FIFO
  │  Validate HW CRC, then SW CRC-32
  ▼
GS Packet Parser
  │  Decode fields by msg_id
  │  Output ASCII to USB CDC (or relay raw to MC)
  ▼
MC USB serial → COBS decode → parse → TelemetryStore → React UI
```
