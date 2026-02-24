# PRD: C.A.S.P.E.R.-2 Radio Subsystem — FC Side

**Rev 1.0 — February 2026**
**Author:** Ashwin Vishwanath
**Status:** Draft

---

## 1. Purpose

Bring up the RA-01H (SX1276) LoRa radio module on the C.A.S.P.E.R.-2 flight computer. This PRD covers the full FC-side radio stack: SPI driver, register-level configuration, TX pipeline (telemetry), and RX pipeline (CAC commands from ground station). Ground station firmware is **out of scope**.

### 1.1 Boundaries — Non-Interference with Test Harness PRD

This PRD and the test harness PRD run as **parallel, independent Claude Code sessions**. They do not share files, directories, or agent tasks.

| | Radio PRD (this doc) | Test Harness PRD |
|---|---|---|
| **Directories** | `App/drivers/`, `App/radio/` | `Tests/`, `Scripts/` |
| **Build** | STM32 firmware (CubeIDE / Makefile) | Host-side CMake + Unity |
| **Hardware** | SPI1, GPIO, EXTI, DMA | None (pure software) |
| **Shared deps** | `App/common/crc32_hw.*`, `App/telemetry/tlm_types.h` | Stubs of the same |

**Rule:** The radio agent must NOT modify any file under `Tests/`, `Scripts/`, or any CMakeLists.txt. The test harness agent must NOT modify any file under `App/drivers/` or `App/radio/`. Shared headers (e.g., `tlm_types.h`) may be read by both but written only by whichever agent created them; the other must adapt.

### 1.2 Success Criteria

- SX1276 silicon ID (RegVersion = 0x12) reads correctly over SPI1.
- FC transmits FC_MSG_FAST at 10 Hz, received and decoded by a known-good SX1276 eval board or SDR.
- FC receives and validates a CAC ARM command (0x80) from a known-good transmitter.
- Profile A → B switchover occurs at configured altitude/velocity threshold.
- Radio stack survives 1-hour pad soak with no SPI errors, no FIFO overflows, no missed TX slots.

### 1.3 Agent Discovery Principles

The implementing agent works inside the CASPER-2 repo and must:

- **Find the telemetry msgset doc** (`.docx` in the repo) for packet formats, msg_ids, and byte layouts.
- **Find `tlm_types.h`** (or equivalent) for packet struct definitions and status bitmap layouts.
- **Read HARDWARE_SPEC.md** for SPI1 clock source (PLL2P = 84 MHz), CRC-32 polynomial, and CubeMX pitfalls.
- **Read INTERFACE_SPEC.md** for CAC protocol state machine, COBS framing, and handshake details.
- **Read PYRO_SPEC.md** for CAC FC-side handler requirements (§6).
- **Check CubeMX `.ioc` file** for SPI1 configuration and GPIO labels. Fix any CubeMX defaults per HARDWARE_SPEC §4.2 (DataSize, BaudRatePrescaler, CPOL/CPHA).

---

## 2. Hardware Interface

### 2.1 Module

| Parameter | Value |
|-----------|-------|
| Module | Ai-Thinker RA-01H |
| IC | Semtech SX1276 |
| Frequency | 868 MHz |
| TX power | +20 dBm max (PA_BOOST) |
| SPI interface | Mode 0 (CPOL=0, CPHA=0) |
| Max SPI clock | 10 MHz |
| Supply | 3.3V (from FC LDO) |

### 2.2 Pin Mapping

All GPIO names in firmware must use the `RADIO_` prefix.

| Function | STM32 Pin | Port | GPIO Label | Direction | Config |
|----------|-----------|------|------------|-----------|--------|
| SPI1_SCK | PA5 | GPIOA | — | AF5 | SPI1 alternate function |
| SPI1_MOSI | PA7 | GPIOA | — | AF5 | SPI1 alternate function |
| SPI1_MISO | PB4 | GPIOB | — | AF5 | SPI1 alternate function |
| Chip Select | PB0 | GPIOB | `RADIO_CS` | Output PP | Active low, software managed |
| DIO0 / INT | PB1 | GPIOB | `RADIO_INT` | Input EXTI | Rising edge, TX/RX Done |
| DIO1 | PD7 | GPIOD | `RADIO_DIO1` | Input EXTI | Rising edge, RX Timeout |
| DIO2 | PD6 | GPIOD | `RADIO_DIO2` | Input | FHSS (unused, input float) |
| DIO3 | PA4 | GPIOA | `RADIO_DIO3` | Input EXTI | Rising edge, ValidHeader |
| DIO4 | PB12 | GPIOB | `RADIO_DIO4` | Input | CadDetected (future) |
| DIO5 | PB13 | GPIOB | `RADIO_DIO5` | Input | ModeReady (future) |
| Reset | PC13 | GPIOC | `RADIO_NRST` | Output OD | Active low, open-drain |

**Critical pin notes:**

- **PA4** is SPI1_NSS in some AF configs. Must be configured as plain GPIO, NOT SPI NSS. CS is software-managed on PB0.
- **PA5/PA7/PB4** must be AF5 for SPI1. Verify CubeMX hasn't assigned conflicting alternate functions.
- **PC13** is on the power-limited VBAT domain on some STM32H7 variants. Verify it can sink enough current for the RA-01H NRESET line. If not, use push-pull with series resistor.

### 2.3 SPI1 Configuration

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Clock source | PLL2P = 84 MHz | HARDWARE_SPEC §3.5 |
| Prescaler | **16** | 84 / 16 = 5.25 MHz (under SX1276 10 MHz max) |
| Mode | Mode 0 (CPOL=0, CPHA=0) | SX1276 datasheet §4.3 |
| Data size | 8-bit | **CubeMX defaults to 4-bit — must override** (HARDWARE_SPEC §4.2) |
| First bit | MSB first | SX1276 requirement |
| NSS | Software (GPIO) | CS on PB0, not hardware NSS |
| NSSPMode | `SPI_NSS_PULSE_DISABLE` | Software CS |

**CubeMX override required** (in USER CODE SPI1_Init 2):
```c
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;   /* Mode 0 */
hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;     /* Mode 0 */
hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
HAL_SPI_Init(&hspi1);
```

### 2.4 DIO Mapping (RegDioMapping1/2)

SX1276 LoRa mode DIO function configuration:

| DIO | Pin | RegDioMapping | Function | IRQ Use |
|-----|-----|---------------|----------|---------|
| DIO0 | PB1 | `00` (bits 7:6 of 0x40) | TxDone / RxDone | **Primary interrupt** — triggers TX complete and RX complete |
| DIO1 | PD7 | `00` (bits 5:4 of 0x40) | RxTimeout | **RX window timeout** — ends RX listen window |
| DIO2 | PD6 | `00` (bits 3:2 of 0x40) | FhssChangeChannel | Not used (no FHSS). Leave as input, no EXTI |
| DIO3 | PA4 | `01` (bits 7:6 of 0x41) | ValidHeader | **Optional** — early RX indication for latency metrics |
| DIO4 | PB12 | `10` (bits 5:4 of 0x41) | PllLock | **Diagnostic** — verify PLL locks during TX/RX |
| DIO5 | PB13 | `10` (bits 3:2 of 0x41) | ModeReady | **Diagnostic** — verify mode transitions complete |

**Minimum viable set:** DIO0 (TX/RX Done) and DIO1 (RX Timeout). DIO3–5 are diagnostics and can be deferred.

### 2.5 EXTI Configuration

| Pin | EXTI Line | IRQ Handler | Priority | Trigger |
|-----|-----------|-------------|----------|---------|
| PB1 (DIO0) | EXTI1 | `EXTI1_IRQHandler` | 5, 0 | Rising edge |
| PD7 (DIO1) | EXTI9_5 | `EXTI9_5_IRQHandler` | 5, 1 | Rising edge |
| PA4 (DIO3) | EXTI4 | `EXTI4_IRQHandler` | 6, 0 | Rising edge (optional) |

**Note:** EXTI9_5 is shared with PC8 (MMC5983MA DRDY) per HARDWARE_SPEC §5.1. The ISR must check which pin triggered and dispatch accordingly. The agent must verify this shared handler exists and add radio dispatch to it.

---

## 3. File Structure

```
App/
├── drivers/
│   ├── sx1276.c           ← SPI register R/W, reset, FIFO ops
│   └── sx1276.h           ← Register map, enums, driver API
├── radio/
│   ├── radio_manager.c    ← TX scheduler, RX windows, profile switching
│   ├── radio_manager.h    ← Public API (init, tick, send, on_rx)
│   ├── radio_config.h     ← Profile A/B parameters, DIO map, constants
│   └── radio_irq.c        ← EXTI callbacks, flag-based dispatch to main loop
```

**Naming convention:** All public symbols use `sx1276_` prefix (driver) or `radio_` prefix (manager). Internal statics use `s_` prefix. Follows JSF/SKILL.md patterns.

---

## 4. Stage 1 — SPI Driver & Silicon Verification

### 4.1 `sx1276.c/h` — Low-Level Driver

**Register map:** Define all SX1276 registers as `#define SX1276_REG_*` constants. Minimum required set:

| Register | Address | Purpose |
|----------|---------|---------|
| RegFifo | 0x00 | FIFO read/write |
| RegOpMode | 0x01 | Operating mode + LoRa bit |
| RegFrfMsb/Mid/Lsb | 0x06–0x08 | Carrier frequency |
| RegPaConfig | 0x09 | TX power + PA select |
| RegPaDac | 0x4D | +20 dBm enable |
| RegOcp | 0x0B | Over-current protection |
| RegLna | 0x0C | LNA gain |
| RegFifoAddrPtr | 0x0D | FIFO pointer |
| RegFifoTxBaseAddr | 0x0E | TX FIFO base |
| RegFifoRxBaseAddr | 0x0F | RX FIFO base |
| RegFifoRxCurrentAddr | 0x10 | Current RX address |
| RegIrqFlagsMask | 0x11 | IRQ mask |
| RegIrqFlags | 0x12 | IRQ flags (write to clear) |
| RegRxNbBytes | 0x13 | Received byte count |
| RegModemStat | 0x18 | Modem status |
| RegPktSnrValue | 0x19 | Packet SNR |
| RegPktRssiValue | 0x1A | Packet RSSI |
| RegModemConfig1 | 0x1D | BW + CR + implicit header |
| RegModemConfig2 | 0x1E | SF + TxContinuous + CRC |
| RegSymbTimeoutLsb | 0x1F | RX timeout (symbols) |
| RegPreambleMsb/Lsb | 0x20–0x21 | Preamble length |
| RegPayloadLength | 0x22 | Payload length |
| RegModemConfig3 | 0x26 | LowDataRateOptimize + AGC |
| RegDetectOptimize | 0x31 | LoRa detection optimize |
| RegDetectionThreshold | 0x37 | Detection threshold |
| RegSyncWord | 0x39 | LoRa sync word |
| RegDioMapping1 | 0x40 | DIO0–3 mapping |
| RegDioMapping2 | 0x41 | DIO4–5 mapping |
| RegVersion | 0x42 | Silicon revision (**expect 0x12**) |

**Driver API:**

```c
/* Lifecycle */
int  sx1276_init(SPI_HandleTypeDef *hspi);   /* reset, verify ID, set sleep */
void sx1276_reset(void);                     /* pulse RADIO_NRST low 1ms, wait 5ms */

/* Register access */
uint8_t sx1276_read_reg(uint8_t addr);
void    sx1276_write_reg(uint8_t addr, uint8_t val);
void    sx1276_read_fifo(uint8_t *buf, uint8_t len);
void    sx1276_write_fifo(const uint8_t *buf, uint8_t len);

/* Mode control */
void sx1276_set_mode(uint8_t mode);          /* SLEEP, STDBY, TX, RXSINGLE, RXCONT */
void sx1276_set_lora_mode(void);             /* Set LoRa bit in RegOpMode (must be in SLEEP) */

/* Configuration */
void sx1276_set_frequency(uint32_t freq_hz);
void sx1276_set_tx_power(int8_t dbm);        /* PA_BOOST, up to +20 dBm */
void sx1276_set_modulation(uint8_t sf, uint32_t bw_hz, uint8_t cr);
void sx1276_set_sync_word(uint8_t sw);
void sx1276_set_preamble(uint16_t symbols);
void sx1276_set_payload_length(uint8_t len); /* Implicit header mode */

/* IRQ */
uint8_t sx1276_get_irq_flags(void);
void    sx1276_clear_irq_flags(uint8_t mask);

/* RSSI/SNR (read after RX) */
int16_t sx1276_get_packet_rssi(void);
int8_t  sx1276_get_packet_snr(void);
```

**SPI transaction format (SX1276):**
- Write: CS low → send `(addr | 0x80)` → send data byte(s) → CS high
- Read: CS low → send `(addr & 0x7F)` → read data byte(s) → CS high
- FIFO burst: same as above with addr = 0x00, multiple bytes

### 4.2 Bringup Verification

The agent must verify the following before proceeding to Stage 2:

1. **SPI1 init:** `HAL_SPI_Init(&hspi1)` returns `HAL_OK` with correct prescaler/mode.
2. **Reset pulse:** Toggle RADIO_NRST low for ≥1 ms, wait ≥5 ms. Verify RegOpMode reads back default value (0x09 = FSK/OOK sleep after reset).
3. **Silicon ID:** `sx1276_read_reg(0x42)` returns `0x12`. If not, SPI bus is misconfigured — stop.
4. **LoRa mode:** Write RegOpMode with LoRa bit set (must be in SLEEP first). Verify readback.
5. **Register round-trip:** Write a known pattern to RegSyncWord (0x39), read it back, verify match.
6. **FIFO round-trip:** Write 8 known bytes to FIFO, reset pointer, read back, verify match.

---

## 5. Stage 2 — Radio Configuration

### 5.1 Profile Definitions

From msgset §3.1:

| Parameter | Profile A (Standard) | Profile B (Extended Range) |
|-----------|---------------------|---------------------------|
| Spreading Factor | SF7 | SF8 |
| Bandwidth | 250 kHz | 250 kHz |
| Coding Rate | CR 4/5 | CR 4/5 |
| Effective data rate | ~1365 bytes/sec | ~685 bytes/sec |
| Sensitivity gain | Baseline | +2.5 dB |
| Use case | Normal flights | Extreme altitude/range |

Both profiles share:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Frequency | 868.0 MHz | UK/EU ISM band |
| TX power | +20 dBm | PA_BOOST + PA_DAC (RegPaDac = 0x87) |
| Sync word | 0x12 | Private network (not LoRaWAN 0x34) |
| Preamble | 8 symbols | Standard LoRa |
| Header mode | Explicit | Variable packet lengths |
| CRC | Enabled (LoRa HW CRC) | Additional to FC_SYS_CRC |
| LNA | Max gain, AGC on | RegLna = 0x23, RegModemConfig3 bit 2 |
| OCP | 240 mA | For +20 dBm PA_BOOST (RegOcp = 0x3B) |
| LowDataRateOptimize | OFF for both profiles | Symbol time < 16ms at BW250 for SF7/SF8 |

### 5.2 Frequency Register Calculation

```
Frf = (Freq_Hz * 2^19) / F_XOSC
F_XOSC = 32 MHz (SX1276 crystal)

868 MHz: Frf = (868000000 * 524288) / 32000000 = 14221312 = 0xD90000
  RegFrfMsb = 0xD9
  RegFrfMid = 0x00
  RegFrfLsb = 0x00
```

### 5.3 TX Power Configuration (+20 dBm)

```
RegPaConfig (0x09) = 0xFF
  bit 7 = 1 (PA_BOOST output pin)
  bits 6:4 = 0x7 (MaxPower — doesn't matter for PA_BOOST)
  bits 3:0 = 0xF (OutputPower = 15)
  → Pout = 17 - (15 - OutputPower) = 17 dBm

RegPaDac (0x4D) = 0x87
  → Enables +20 dBm mode (overrides to Pout = 20 dBm)

RegOcp (0x0B) = 0x3B
  bit 5 = 1 (OCP on)
  bits 4:0 = 0x1B = 27 → Imax = 45 + 5*27 = 180 mA
  (Actually for +20 dBm, set to 0x3B for 240 mA: bits 4:0 = 0x1B → 240 mA)
```

**Agent note:** Double-check the OCP trim calculation against the SX1276 datasheet Table 10. The formula changes above 120 mA.

### 5.4 `radio_config.h`

```c
typedef struct {
    uint8_t  sf;            /* 7 or 8 */
    uint32_t bw_hz;         /* 250000 */
    uint8_t  cr;            /* 5 (for 4/5) */
    uint8_t  sync_word;     /* 0x12 */
    uint16_t preamble;      /* 8 */
    int8_t   tx_power_dbm;  /* 20 */
    uint32_t freq_hz;       /* 868000000 */
} radio_profile_t;

extern const radio_profile_t RADIO_PROFILE_A;  /* SF7, 250k, CR4/5 */
extern const radio_profile_t RADIO_PROFILE_B;  /* SF8, 250k, CR4/5 */
```

---

## 6. Stage 3 — TX Pipeline

### 6.1 Architecture

```
flight_loop (833 Hz)
  │
  ├─ EKF predict/update → state estimates
  │
  └─ radio_manager_tick() (called every iteration)
       │
       ├─ tx_scheduler: checks if 100ms slot elapsed
       │    ├─ YES: build packet, load FIFO, trigger TX
       │    └─ NO: nop
       │
       ├─ irq_handler: check DIO0 flag (set by ISR)
       │    ├─ TX_DONE: log success, open RX window
       │    └─ RX_DONE: read FIFO, validate, dispatch
       │
       └─ rx_window: manage post-TX RX listen period
```

**Critical constraint:** All SPI transactions happen in the main loop, never in ISR context. The EXTI ISR sets a volatile flag only. This prevents SPI bus contention with other sensors.

### 6.2 TX Scheduler

The FC transmits at a fixed 10 Hz cadence:

| Slot | Period | Packet | Size | Priority |
|------|--------|--------|------|----------|
| Every 100 ms | 10 Hz | FC_MSG_FAST (0x01) | 19 bytes | Highest — never skip |
| Every 200–1000 ms | 1–5 Hz | FC_MSG_GPS (0x02) | 12 bytes | When GPS fix available |
| On event | Async | FC_MSG_EVENT (0x03) | 9 bytes | Queue, send in next free slot |

**Timing:** At Profile A (SF7, BW250, CR4/5), a 19-byte payload has an airtime of approximately 7.4 ms. The 100 ms TX cadence leaves >90 ms per slot for RX windows and processing.

**TX sequence:**
1. Set FIFO pointer to TxBaseAddr (0x80).
2. Write packet bytes to FIFO via burst write.
3. Set RegPayloadLength to packet size.
4. Set mode to TX (`sx1276_set_mode(SX1276_MODE_TX)`).
5. Wait for DIO0 (TxDone) interrupt flag.
6. Clear IRQ flags.
7. Record TX timestamp and sequence number.

### 6.3 Packet Building

The agent must find `tlm_types.h` and the msgset doc to build packets matching the exact byte layout. Summary from msgset:

**FC_MSG_FAST (0x01, 19 bytes):**

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | msg_id | u8 | 0x01 |
| 1–2 | FC_TLM_STATUS | u16 LE | Arm(4) + Cont(4) + State(4) + Flags(4) |
| 3–4 | FC_TLM_ALT | u16 LE | EKF altitude AGL in decametres |
| 5–6 | FC_TLM_VEL | i16 LE | EKF vertical velocity in dm/s |
| 7–11 | FC_ATT_QPACKED | 5 bytes | Smallest-three quaternion |
| 12–13 | FC_FSM_TIME | u16 LE | Mission time in 0.1s ticks |
| 14 | FC_PWR_VBATT | u8 | Battery voltage (offset + scale TBD) |
| 15–18 | FC_SYS_CRC | u32 LE | CRC-32 over bytes [0–14] |

**The agent must cross-reference the msgset doc for the remaining packet types (FC_MSG_GPS, FC_MSG_EVENT) and ACK packets (0xA0, 0xA1, 0xE0).**

### 6.4 CRC-32

Use the STM32H7 hardware CRC unit. From HARDWARE_SPEC §10:

| Parameter | Value |
|-----------|-------|
| Polynomial | 0x04C11DB7 (ISO-HDLC) |
| Init | 0xFFFFFFFF |
| Input/Output reflection | Yes |
| Final XOR | 0xFFFFFFFF |

Call `crc32_hw_compute()` if it already exists in the codebase. If not, implement it using `HAL_CRC_Calculate()` with the above parameters. The agent must verify the CRC init matches by computing the CRC of a known test vector (e.g., `"123456789"` → 0xCBF43926).

---

## 7. Stage 4 — RX Pipeline

### 7.1 RX Window Strategy

The FC is primarily a transmitter. It listens for commands in brief RX windows between TX slots.

```
Time:  0ms        7ms          90ms        100ms
       |-- TX ---|--- RX ---------|-- idle --|-- TX ...
       FC_MSG_FAST  Listen for     Gap       Next TX
                    GS commands
```

After each TX_DONE:
1. Set FIFO pointer to RxBaseAddr (0x00).
2. Configure RX timeout: ~80 ms worth of symbols (RegSymbTimeoutLsb).
3. Set mode to RXSINGLE (`sx1276_set_mode(SX1276_MODE_RXSINGLE)`).
4. Wait for DIO0 (RxDone) or DIO1 (RxTimeout).
5. On RxDone: read FIFO, validate, dispatch.
6. On RxTimeout: return to standby, wait for next TX slot.

### 7.2 RX Packet Validation

All received packets must pass:

1. **LoRa CRC** — handled by hardware. If `RegIrqFlags & PayloadCrcError`, discard.
2. **Minimum length** — reject packets < 5 bytes (1 msg_id + 4 CRC).
3. **msg_id recognition** — must be a known command type (0x80, 0x81, 0x82, 0x83, 0xF0, 0xF1).
4. **Magic bytes** — CAC packets (0x80–0x83, 0xF0, 0xF1) must have bytes [1:2] = 0xCA, 0x5A.
5. **CRC-32** — compute over payload, compare against last 4 bytes.
6. **Complement check** — for ARM (0x80) and FIRE (0x81), verify field complements.

On validation pass: dispatch to the CAC handler (`cac_handler.c` from PYRO_SPEC §6).

### 7.3 Command Dispatch

```c
void radio_on_rx_packet(const uint8_t *buf, uint8_t len)
{
    /* Already validated CRC and magic */
    uint8_t msg_id = buf[0];
    switch (msg_id) {
        case 0x80: cac_handle_arm(buf, len);     break;
        case 0x81: cac_handle_fire(buf, len);    break;
        case 0x82: cac_handle_testmode(buf, len); break;
        case 0x83: cac_handle_poll(buf, len);    break;
        case 0xF0: cac_handle_confirm(buf, len); break;
        case 0xF1: cac_handle_abort(buf, len);   break;
        default:   /* unknown — ignore */ break;
    }
}
```

The CAC handlers are defined in PYRO_SPEC §6. They already exist (or will be implemented per the test harness PRD's `test_cac_handler.c` spec). The radio RX pipeline only needs to call them — it does NOT reimplement CAC logic.

### 7.4 ACK Transmission

When the CAC handler generates an ACK/NACK response, it calls `radio_send_response()`:

```c
int radio_send_response(const uint8_t *buf, uint8_t len);
```

This queues the response for immediate transmission (pre-empts the next scheduled FC_MSG_FAST). The TX scheduler checks the response queue before building the next telemetry packet.

**Priority:** ACK/NACK > FC_MSG_EVENT > FC_MSG_FAST > FC_MSG_GPS.

---

## 8. Stage 5 — Profile Switching & Integration

### 8.1 Automatic Profile Switchover

From msgset §3.4:

- FC switches from Profile A → Profile B when EKF state crosses any configured threshold (altitude or velocity).
- Once switched, **no switch-back** during the remainder of the flight.
- The switchover reconfigures SF only (SF7 → SF8). All other params are identical.

```c
void radio_check_profile_switch(const ekf_state_t *state)
{
    if (s_current_profile == RADIO_PROFILE_B) return;  /* already switched */
    if (state->alt_m > cfg.profile_switch_alt_m ||
        state->vel_mps > cfg.profile_switch_vel_mps) {
        radio_apply_profile(&RADIO_PROFILE_B);
        s_current_profile = RADIO_PROFILE_B;
        /* Emit FC_EVT_PROFILE_SWITCH event */
    }
}
```

**Thresholds:** Stored in the flight configuration block (uploaded via MC_CMD_UPLOAD before flight). The agent must find where config is stored and read from there. Default values for testing: altitude > 20,000 m or velocity > 500 m/s.

### 8.2 RSSI/SNR Reporting

After every RX (successful or timeout), read and store:

- `sx1276_get_packet_rssi()` → used by GS, not directly by FC, but useful for FC logging.
- `sx1276_get_packet_snr()` → same.

These values are logged to flash (not transmitted — the GS computes its own link metrics).

### 8.3 Integration with Flight Loop

The radio manager hooks into the main flight loop:

```c
/* In main.c flight_loop(), called at 833 Hz: */
radio_manager_tick(&flight_ctx);
```

`radio_manager_tick()` must be **non-blocking**. It checks flags, manages state, and initiates SPI transactions, but never busy-waits for TX/RX completion. The DIO0 EXTI sets a flag; the next tick processes it.

**Initialization sequence** (in `main()` before flight loop):

```c
sx1276_init(&hspi1);                    /* Reset, verify 0x12 */
radio_manager_init(&RADIO_PROFILE_A);   /* Configure LoRa, set DIO mapping */
```

### 8.4 Error Handling

| Error | Detection | Recovery |
|-------|-----------|----------|
| SPI timeout | `HAL_SPI_Transmit` returns `HAL_TIMEOUT` | Log error, attempt re-init |
| Silicon ID mismatch | RegVersion ≠ 0x12 | Log error, disable radio, continue flight on inertial only |
| TX timeout | DIO0 not asserted within 200 ms | Reset radio module, re-init, log error |
| FIFO overflow | Attempt to write > 256 bytes | Truncate packet, log error |
| Persistent RX CRC errors | >10 consecutive CRC failures | Log, continue (could be interference) |
| Profile switch during TX | EKF crosses threshold mid-transmit | Complete current TX, switch before next TX |

**No radio failure is ever flight-critical.** The FC flies, navigates, and deploys parachutes entirely on onboard sensors. Radio loss means no telemetry and no ground commands — recovery ops rely on GPS beacon and/or visual tracking.

---

## 9. Implementation Stages

Each stage is independently testable. The agent should verify each stage before proceeding.

### Stage 1 — SPI Driver (1 sub-agent)

- [ ] `sx1276.c/h` — register R/W, reset, FIFO ops.
- [ ] CubeMX SPI1 config verified and overridden per §2.3.
- [ ] GPIO init for RADIO_CS, RADIO_NRST.
- [ ] Silicon ID test: read RegVersion, assert 0x12.
- [ ] Register round-trip test.
- [ ] FIFO round-trip test.

### Stage 2 — Radio Configuration (1 sub-agent)

- [ ] `radio_config.h` — Profile A and B constants.
- [ ] LoRa mode init: frequency, power, modulation, sync word, preamble.
- [ ] DIO mapping configured per §2.4.
- [ ] EXTI handlers for DIO0 and DIO1 (flag-only, no SPI in ISR).
- [ ] Profile apply function: reconfigure SF without full re-init.

### Stage 3 — TX Pipeline (1 sub-agent)

- [ ] Packet builder for FC_MSG_FAST using tlm_types.h structs/fields.
- [ ] Packet builder for FC_MSG_GPS, FC_MSG_EVENT.
- [ ] TX scheduler: 10 Hz cadence, priority queue.
- [ ] CRC-32 integration (hardware CRC unit).
- [ ] TX sequence: FIFO write → mode TX → wait DIO0 → clear flags.
- [ ] Sequence number counter (wrapping u8 or u16 per msgset).

### Stage 4 — RX Pipeline (1 sub-agent)

- [ ] RX window: RXSINGLE after each TX_DONE, ~80 ms timeout.
- [ ] RX validation chain: LoRa CRC → length → msg_id → magic → CRC-32 → complement.
- [ ] Command dispatch to CAC handlers.
- [ ] ACK/NACK TX via response queue.
- [ ] RSSI/SNR readback after each RX.

### Stage 5 — Integration (lead agent)

- [ ] `radio_manager_tick()` non-blocking state machine.
- [ ] Profile switchover from EKF state.
- [ ] Error handling and recovery per §8.4.
- [ ] Integration into `main.c` flight loop.
- [ ] 1-hour pad soak test (on real hardware).

---

## 10. Agent Team Structure

```
Lead Agent ─── owns radio_manager.c/h, integration, final verification
    │
    ├── Sub-Agent A ── sx1276.c/h driver (Stage 1 + 2)
    │                  Deliverables: verified SPI comms, LoRa config
    │
    ├── Sub-Agent B ── TX pipeline (Stage 3)
    │                  Deliverables: packet builders, TX scheduler
    │                  Depends on: Sub-Agent A (driver API)
    │
    └── Sub-Agent C ── RX pipeline (Stage 4)
                       Deliverables: RX window, validation, dispatch
                       Depends on: Sub-Agent A (driver API), CAC handlers existing
```

**Parallelism:** Sub-Agents B and C can work in parallel once Sub-Agent A delivers the driver. The lead agent handles Stage 5 integration after all sub-agents complete.

**Boundary with test harness:** If the test harness agent creates `mock_spi.c` or stub headers, the radio agent must NOT depend on them. The radio code runs on real hardware only. If the radio agent needs to verify code without hardware, it uses `printf` debug over SWD/ITM, not Unity tests.

---

## 11. Open Questions

1. ~~**Pin mapping:**~~ **RESOLVED.** Full DIO0–DIO5 + NRST + SPI1 pin mapping provided.

2. **SPI1 DMA:** Should TX/RX FIFO access use DMA or polling? At 5.25 MHz, a 19-byte transfer takes ~29 µs — polling is likely fine and simpler. DMA adds complexity for minimal gain at these packet sizes. **Recommendation: polling.** Agent should implement polling first, profile, and add DMA only if SPI contention is observed.

3. **Frequency regulation:** 868 MHz ISM band has duty cycle limits in the EU (1% for some sub-bands). At 10 Hz with ~7.4 ms TX airtime, duty cycle is ~7.4%. The agent should verify which 868 MHz sub-band the RA-01H targets and whether the duty cycle complies with ETSI EN 300 220. For UK launches this is likely fine but worth documenting.

4. **GS sync word:** Both FC and GS must use the same sync word (0x12). The agent must verify this is not configurable per-flight (it should be hardcoded, not in the config block).

5. **EXTI9_5 shared handler:** PD7 (DIO1) shares EXTI9_5 with PC8 (MMC5983MA DRDY). The agent must verify the existing ISR dispatches correctly and add DIO1 handling without breaking magnetometer interrupts.
