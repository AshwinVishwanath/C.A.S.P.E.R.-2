# C.A.S.P.E.R 2 — Control and Stability Package for Experimental Rocketry

[![Hardware Status](https://img.shields.io/badge/Hardware-Final%20Design%20%E2%80%93%20Off%20for%20Manufacture-brightgreen)]()
[![Software Status](https://img.shields.io/badge/Software-Early%20Development-yellow)]()
[![Platform](https://img.shields.io/badge/MCU-STM32H750-blue)]()
[![Category](https://img.shields.io/badge/Application-Rocket%20Avionics-orange)]()

---

## Overview  
**C.A.S.P.E.R 2** (Control and Stability Package for Experimental Rocketry) is a next-generation avionics platform developed for **High-Power** and **actively controlled experimental rockets**.  
It provides **real-time state estimation, telemetry, and control capability** in an ultra-compact form factor optimized for high-G, Mach-capable flight environments.  
This system integrates high-rate inertial sensing, barometric altitude tracking, GPS assistance, LoRa telemetry, and multi-channel pyro/servo control — all within a single embedded architecture.

---

## Aim  
Develop an **ultra-compact, GPS-aided, Mach-capable tracking, telemetry, and control avionics system** for both **High-Power Rockets (HPR)** and **actively controlled experimental rockets**, designed for high reliability, modularity, and performance in extreme flight conditions.

---

## Architecture Overview  
**Core MCU:** STM32H750  
**Inertial Navigation Suite:** Dual redundant IMUs (ADXL372 High-G + LSM6DSO32 Low-G), MS5611 barometer, MMC5983MA magnetometer  
**Communications:** LoRa radio transceiver, U-blox M10M GPS (external antenna)  
**Storage:** 64 MB QSPI flash memory  
**Pyro / Actuation:** 4 pyro channels with 16-bit continuity detection, 4 PWM servo channels  
**Auxiliary:** Fully integrated power management, onboard warm-start supercapacitor bank for GPS retention, compact 4-layer PCB design.

---

##  Section 1 — Hardware

### 1.1 Microcontroller  
- **STM32H750** ARM Cortex-M7 
- Deterministic real-time operation with DMA-driven peripheral access  
- Dual SPI buses for flash and sensor isolation  
- Separate I²C buses for IMU/barometer and magnetometer domains  
- Integrated 16-bit ADC for continuity sensing and power rail monitoring  
- Hardware timers dedicated to PWM and pyro control  
- Ample computational headroom for EKF, telemetry, and diagnostics

---

### 1.2 Power Management  
- Input range **2 V – 16 V**, regulated by **TPS63070** buck-boost converter  
- TVS and reverse-polarity protection on main input  
- Ferrite-bead filtering for sensor rail stability during pyro discharge  
- Current-sense shunt for real-time current telemetry  
- Optimized for single- or dual-cell Li-ion/LiPo operation

---

### 1.3 Inertial Navigation Suite  
**Dual-range IMU configuration for wide-band state estimation:**
- **ADXL372 — High-G Accelerometer** ±200 g range, used during boost/transonic regimes  
- **LSM6DSO32 — 6-DoF IMU** ±32 g accel, ±4000 dps gyro for fine-scale motion  
- **MS5611 — Barometer** 24-bit ΔΣ ADC with PROM-based temperature/pressure compensation  
- **MMC5983MA — Magnetometer** 18-bit 3-axis field sensor for yaw and post-flight orientation  
- Sensors isolated on independent I²C domains to minimize cross-talk  
- Noise-model-based fusion prepared for onboard EKF integration

---

### 1.4 GPS Subsystem  
- **U-blox M10M** GPS receiver (**standard COCOM limits**)  
- External antenna via **U.FL** connector for nose-cone integration  
- Supports active antennas with switchable bias-tee and surge suppression  
- 1 Hz – 10 Hz configurable update rate  
- Exposed **SAFEBoot** and **EXTINT** pins for firmware update and hot-start control  
- Integrated **supercapacitor bank** provides **up to one hour of warm-start capability** after MCU power loss  
- Controlled-impedance RF path and dedicated ground plane for signal integrity

---

### 1.5 Radio & Telemetry  
- **LoRa Transceiver (SX1276/SX1262 family)**  
- 868 MHz operation with adjustable spreading factor and bandwidth  
- SPI interface for reliable bidirectional telemetry  
- 50 Ω controlled-impedance RF trace with ESD protection and matched antenna network  
- Supports full-duplex command/response protocol:  
  - Arming/disarming  
  - Calibration/test commands  
  - System status and flight state feedback  
- Transmission rate configurable up to **10 Hz**

---

### 1.6 Pyro Subsystem  
- **Four independent pyro channels**  
  - Each driven by **Dedicated N-MOSFET** with Kelvin-sense feedback  
  - **16-bit ADC-based continuity detection** via precision sense resistor  
  - Supports both pulse and sustained fire modes  
- Integrated flyback suppression and gate clamping   

---

### 1.7 Actuation  
- **Four PWM channels** driven by hardware timers for low-latency response  
- Compatible with 7.4 V servo rail  
- Shared timing source with navigation loop for synchronized control  
- Upstream filtering implemented at regulator level 

---

### 1.8 Storage  
- **64 MB (512 Mbit) Winbond W25Q512JV QSPI Flash**  
- 133 MHz QSPI interface for continuous logging  
- Dual-bank configuration for redundancy and safety  
- Capable of streaming 1 kHz sensor data for full flight records

---

### 1.9 PCB & Mechanical Design  
- **4-layer PCB stackup** 
- Controlled impedance traces (50 Ω RF, 90 Ω USB)  
- Conformal coating for environmental resilience  
- Compact layout for 38 mm and 54 mm avionics bays  
- Screw connectors for power, pyro, and servo I/O  
- SWD programming header for debug and firmware development

---

## 💻 Section 2 — Software  
*(To be populated in future revisions.)*  
Planned subsections will include:  
- System Architecture  
- Sensor Fusion and EKF Implementation  
- State Machine and Flight Phases  
- Telemetry Encoding & Ground Link Protocol  
- Logging and Diagnostics  
- Firmware Update Procedure  
- Error Handling and Redundancy Management

---

## 📬 Contact  
For inquiries or collaboration requests, please contact **Ashwin Vishwanath** via the [Project C.A.S.P.E.R. GitHub organization](https://github.com/AshwinVishwanath) or open an issue in this repository.

---

**© 2025 — C.A.S.P.E.R Avionics / Project Sunride — University of Sheffield**
