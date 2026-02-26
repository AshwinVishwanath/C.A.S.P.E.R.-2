<p align="center">
  <h1 align="center">C.A.S.P.E.R.-2</h1>
  <p align="center"><b>Control and Stability Package for Experimental Rocketry</b></p>
</p>

<p align="center">
  <a href=""><img src="https://img.shields.io/badge/Hardware-Manufactured-brightgreen" alt="Hardware Status"/></a>
  <a href=""><img src="https://img.shields.io/badge/Software-Alpha-orange" alt="Software Status"/></a>
  <a href=""><img src="https://img.shields.io/badge/MCU-STM32H750VBT6-blue" alt="Platform"/></a>
  <a href=""><img src="https://img.shields.io/badge/Application-Rocket%20Avionics-red" alt="Category"/></a>
  <a href=""><img src="https://img.shields.io/badge/Build_Size-~89_KB-lightgrey" alt="Build Size"/></a>
</p>

---

## Overview

**C.A.S.P.E.R.-2** is a compact avionics platform for **high-power** and **actively controlled experimental rockets**. It provides real-time inertial navigation, LoRa telemetry, GPS tracking, and multi-channel pyro/servo control in a single embedded system designed for high-G, Mach-capable flight.

### Key Capabilities

- Dual-range inertial sensing (±32 g / ±200 g) with 833 Hz sampling
- Onboard EKF state estimation with transonic barometric gating
- Bidirectional LoRa telemetry and command uplink at 868 MHz
- 4-channel pyro deployment with multi-layer safety interlocks
- 4-channel PWM servo output for active flight control
- 64 MB onboard flash logging with USB mass storage retrieval
- GPS-aided navigation with supercapacitor warm-start

---

## Hardware

<table>
<tr><td>

### Core

| Subsystem | Component | Specs |
|:----------|:----------|:------|
| MCU | STM32H750VBT6 | Cortex-M7, 480 MHz max, 128 KB flash, 1 MB RAM |
| Storage | W25Q512JV | 64 MB QSPI NOR, FAT filesystem |
| Power | TPS63070 | 2-16 V buck-boost, TVS + reverse-polarity protection |

</td></tr>
<tr><td>

### Sensors

| Sensor | Type | Specs |
|:-------|:-----|:------|
| LSM6DSO32 | 6-DoF IMU | ±32 g accel, ±2000 dps gyro, 833 Hz |
| ADXL372 | High-G Accel | ±200 g, FIFO, boost/transonic regime |
| MS5611 | Barometer | 24-bit, PROM-calibrated |
| MMC5983MA | Magnetometer | 18-bit 3-axis, heading reference |
| MAX-M10M | GPS | 10 Hz, U.FL antenna, supercap warm-start |

</td></tr>
<tr><td>

### Comms & Control

| Subsystem | Component | Specs |
|:----------|:----------|:------|
| Radio | SX1276 (RA-01H) | LoRa 868 MHz, +20 dBm, bidirectional |
| Pyro | 4 channels | N-MOSFET, ADC continuity sensing |
| Servos | 4 PWM channels | Hardware timer, 7.4 V rail |

</td></tr>
</table>

### Sensing

The inertial navigation suite uses a dual-range accelerometer configuration. The **LSM6DSO32** provides high-resolution 6-DoF data (accel + gyro) at 833 Hz for attitude estimation and EKF prediction during normal flight. The **ADXL372** covers the ±200 g range for boost and transonic phases where the primary IMU saturates. The **MS5611** barometer provides altitude observations to the EKF, and the **MMC5983MA** magnetometer provides heading reference for the attitude estimator on the pad.

### GPS

The **U-blox MAX-M10M** GPS receiver operates at 10 Hz in GPS-only constellation mode, connected via an external U.FL antenna for nose-cone mounting. An onboard supercapacitor bank provides up to one hour of warm-start retention after power loss, significantly reducing time-to-first-fix on the pad. Standard COCOM limits apply.

### Radio

The **SX1276** LoRa transceiver (RA-01H module) operates at 868 MHz with +20 dBm output power. The radio supports bidirectional communication: downlink telemetry at 10 Hz and uplink commands for arming, calibration, and configuration. Two spreading factor profiles allow the system to trade range for data rate depending on flight phase.

### Pyro & Actuation

Four independent pyro channels, each driven by a dedicated N-channel MOSFET with ADC-based continuity sensing for pre-flight e-match verification. Four PWM servo channels driven by hardware timers provide low-latency actuation for active control surfaces.

### Power

The **TPS63070** buck-boost converter accepts 2-16 V input, supporting single- or dual-cell Li-ion/LiPo configurations. TVS diodes and reverse-polarity protection guard the main input. Ferrite-bead filtering isolates the sensor rails from pyro discharge transients.

### PCB

4-layer stackup with controlled impedance traces (50 ohm RF, 90 ohm USB). Designed for 38 mm and 54 mm avionics bays. Screw terminals for power, pyro, and servo I/O. SWD header and USB DFU for programming.

---

## Software

### Firmware Overview

Bare-metal embedded C running at **432 MHz** (HSI + PLL). **~89 KB** build size within the 128 KB internal flash. No RTOS: a single superloop drives all sensor polling, navigation, telemetry, and control at deterministic rates.

### System Architecture

```
 ┌──────────────────────────────────────────────────────────────────┐
 │                        SUPERLOOP (833 Hz)                        │
 ├──────────────────────────────────────────────────────────────────┤
 │                                                                  │
 │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
 │  │ LSM6DSO32│  │  MS5611  │  │ MMC5983MA│  │    MAX-M10M      │ │
 │  │  833 Hz  │  │  async   │  │  100 Hz  │  │     10 Hz        │ │
 │  │ accel+gyr│  │ baro tick│  │   mag    │  │      GPS         │ │
 │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └───────┬──────────┘ │
 │       │              │             │                │            │
 │       v              v             v                v            │
 │  ┌─────────────────────────────────────────────────────────┐     │
 │  │              NAVIGATION ENGINE                          │     │
 │  │                                                         │     │
 │  │  Attitude (833 Hz)          EKF (416 Hz)                │     │
 │  │  Mahony + RK4 quat          4-state vertical            │     │
 │  │  mag correction (pad)       baro gating (transonic)     │     │
 │  └──────────────────────┬──────────────────────────────────┘     │
 │                         │                                        │
 │                         v                                        │
 │  ┌──────────────────────────────────────────────┐                │
 │  │              FLIGHT FSM (12 states)           │                │
 │  │  PAD > BOOST > COAST > APOGEE > MAIN > LANDED│                │
 │  └──────────┬────────────────────┬──────────────┘                │
 │             │                    │                               │
 │             v                    v                               │
 │  ┌──────────────────┐  ┌─────────────────┐  ┌────────────────┐  │
 │  │  PYRO MANAGER    │  │  TELEMETRY      │  │  RADIO         │  │
 │  │  4-ch arm/fire   │  │  COBS + CRC-32  │  │  SX1276 LoRa   │  │
 │  │  ADC continuity  │  │  10 Hz msgset   │  │  868 MHz TX/RX │  │
 │  │  safety interlock│  │  USB CDC out    │  │  CAC uplink    │  │
 │  └──────────────────┘  └─────────────────┘  └────────────────┘  │
 │                                                                  │
 └──────────────────────────────────────────────────────────────────┘
```

### Navigation

A 4-state vertical **Extended Kalman Filter** (altitude, velocity, accelerometer bias, barometer bias) runs at 416 Hz using CMSIS-DSP matrix operations. A **Mahony complementary filter** with RK4 quaternion integration provides full 3-axis attitude at 833 Hz, using magnetometer corrections on the pad and pure gyro propagation in flight. Barometric updates include transonic gating to reject shock-induced pressure errors during Mach transition.

### Telemetry

Binary telemetry protocol (msgset v5) with COBS framing and hardware CRC-32. Transmitted simultaneously over **USB CDC** and **LoRa radio**.

| Message | ID | Size | Rate | Content |
|:--------|:--:|:----:|:----:|:--------|
| FAST | `0x01` | 20 B | 10 Hz | Altitude, velocity, quaternion, flight time, battery, status |
| GPS | `0x02` | 17 B | On fix | Delta lat/lon from pad, altitude MSL, fix type, sat count |
| EVENT | `0x03` | 11 B | Immediate | State changes, pyro fires, errors, staging events |

### Radio Profiles

The SX1276 driver automatically switches between two profiles based on altitude and velocity:

| Profile | SF | BW | Use Case |
|:--------|:--:|:--:|:---------|
| **A** | SF7 | 125 kHz | Pad / low altitude: higher data rate, bidirectional commands |
| **B** | SF8 | 125 kHz | High altitude / velocity: longer range, recovery beacon |

### Command & Control (CAC)

Three-step handshake for safety-critical operations with nonce replay protection and 5-second timeout:

```
  Ground Station                    Flight Computer
       │                                  │
       │──── CMD  (magic + nonce) ───────>│
       │                                  │
       │<──── ACK (nonce echo) ───────────│
       │                                  │
       │──── CONFIRM (nonce) ────────────>│
       │                                  │
       │<──── EXEC result ───────────────│
       │                                  │
```

Supported commands: pyro arm/disarm, fire, test mode, calibration triggers, configuration updates.

### Flight State Machine

12-state FSM with sensor-driven transitions. All transitions use dwell timers (sustained condition checks) to reject transient noise.

```
                            ┌───────────────────────────────┐
                            │       sustain re-light         │
                            │       (accel > 3g, 100ms)      │
                            v                                │
  ┌─────────┐  launch  ┌─────────┐  burnout  ┌──────────┐   │
  │         │ detected │         │ detected  │          │───┘
  │   PAD   │────────> │  BOOST  │────────── │  COAST   │
  │         │          │         │           │          │
  └─────────┘          └─────────┘           └─────┬────┘
                                                   │
    Launch requires ALL:                           │  vel <= 0 for 25ms
    * Antenna vertical (±30deg)                    │  AND flight_time > 5s
    * Accel > 2g for 100ms                         │
    * EKF velocity > 15 m/s                        v
                                            ┌──────────┐
                                            │          │
                                            │  APOGEE  │ ---- Drogue pyro fires
                                            │          │
                                            └─────┬────┘
                                                  │
                                  ┌───────────────┤
                                  │               │
                            alt <= main     drogue failure
                            deploy alt      (high descent rate)
                                  │               │
                                  v               v
                             ┌──────────┐
                             │          │
                             │   MAIN   │ ---- Main chute pyro fires
                             │          │
                             └─────┬────┘
                                   │
                                   │  |vel| < 1 m/s AND
                                   │  |delta_alt| < 2m for 3s
                                   v
                             ┌──────────┐   5 min   ┌──────────┐
                             │          │ ────────>  │          │
                             │  LANDED  │           │ RECOVERY │
                             │          │           │          │
                             └──────────┘           └──────────┘
                                  │                       │
                             Pyros disarmed          Low-power mode
                             All channels            Periodic LoRa beacon
                             auto-disarmed           Sensors shut down
```

### Pyro Safety

Multi-layer safety architecture prevents unintentional pyro discharge:

```
                      ┌───────────────────────────┐
                      │       FIRE COMMAND         │
                      │    (CAC or FSM auto-fire)  │
                      └─────────────┬──────────────┘
                                    │
                      ┌─────────────v──────────────┐
                      │   PAD lockout check        │─── BLOCKED if FSM == PAD
                      │   (unless test mode)       │    (test mode: 50ms cap)
                      └─────────────┬──────────────┘
                                    │ PASS
                      ┌─────────────v──────────────┐
                      │   Channel armed?           │─── BLOCKED if not armed
                      └─────────────┬──────────────┘
                                    │ PASS
                      ┌─────────────v──────────────┐
                      │   Continuity present?      │─── BLOCKED if open circuit
                      └─────────────┬──────────────┘
                                    │ PASS
                      ┌─────────────v──────────────┐
                      │   Duration capped          │
                      │   Normal: max 2000ms       │
                      │   Test:   max 50ms         │
                      └─────────────┬──────────────┘
                                    │
                      ┌─────────────v──────────────┐
                      │         FIRE               │
                      │     MOSFET gate driven     │
                      │     Auto-stop on timeout   │
                      └────────────────────────────┘
```

- **Auto-arm** on launch detection (PAD -> BOOST) for all channels with continuity
- **Auto-disarm** on landing (MAIN -> LANDED)
- **Test mode** allows controlled fires on PAD with 50ms duration cap and 60s timeout

### Storage & Data Collection

64 MB QSPI NOR flash with FAT filesystem. Three compile-time USB modes:

| Mode | Type | Function |
|:-----|:-----|:---------|
| `USB_MODE=1` | CDC | Flight loop with EKF navigation, 10 Hz COBS telemetry |
| `USB_MODE=2` | MSC | Mount flash as USB drive for data retrieval |
| `USB_MODE=3` | CDC | High-rate sensor logging to CSV files on flash |

### Calibration

Built-in calibration modes for field use:
- **Magnetometer calibration** - hard/soft iron compensation via rotation sampling
- **Magnetometer validation** - verify calibration quality against expected field magnitude
- **Gyro temperature calibration** - characterize bias drift across temperature range

---

<p align="center">
  <b>C.A.S.P.E.R. Avionics / Project Sunride - University of Sheffield</b><br/>
  Contact: <b>Ashwin Vishwanath</b> - <a href="https://github.com/AshwinVishwanath">GitHub</a> | Open an issue in this repository
</p>
