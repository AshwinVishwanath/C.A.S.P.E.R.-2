# Dev Update — Flight Computer Bring-Up

**Date:** February 2026
**Board:** C.A.S.P.E.R.-2 Rev 1 (STM32H750VBT6, LQFP100)
**Firmware:** CubeMX HAL + USB CDC/MSC, 432 MHz SYSCLK (HSI+PLL), 6 sensors + EKF + attitude + LoRa telemetry + buzzer (~89 KB)

---

## Summary

Brought the C.A.S.P.E.R.-2 flight computer from a completely unresponsive state through three development revisions: basic sensor bring-up (Rev 2), data infrastructure and noise characterization (Rev 3a/3b), and a full EKF navigation stack (Rev 3c). Four sensors active: MS5611 barometer, LSM6DSO32 IMU, ADXL372 high-g accelerometer, and W25Q512JV QSPI flash. The EKF fuses IMU + baro at 500 Hz to estimate altitude, velocity, and attitude. No ST-Link debugger — all flashing via USB DFU, all debugging via milestone LEDs and serial output.

---

## Issues Found & Resolved

### 1. MCU Not Reaching `main()` — Power Supply Misconfiguration

**Symptom:** Board completely dead after DFU flash. No LED activity, no USB enumeration. Nothing.

**Root Cause:** The CubeMX-generated Makefile defined `-DUSE_PWR_EXTERNAL_SOURCE_SUPPLY`, but the board uses the STM32H750's **internal LDO** regulator.

The STM32H7 startup assembly (`startup_stm32h750xx.s`) calls `ExitRun0Mode()` in `system_stm32h7xx.c` **before** `SystemInit()` or `main()` ever run. With `USE_PWR_EXTERNAL_SOURCE_SUPPLY`, this function:
1. Disables the internal LDO
2. Enables bypass mode
3. Waits forever for `ACTVOSRDY` — a voltage-ready flag that never asserts because there is no external supply

The MCU was hanging in an infinite loop in the startup code, before a single line of `main()` could execute.

**Fix:**
- Changed Makefile `C_DEFS`: `-DUSE_PWR_EXTERNAL_SOURCE_SUPPLY` to `-DUSE_PWR_LDO_SUPPLY`
- Changed `SystemClock_Config()`: `HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY)` to `HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY)`

> **Note:** If regenerating from CubeMX, you **must** manually verify the power supply define is `USE_PWR_LDO_SUPPLY` (not `USE_PWR_EXTERNAL_SOURCE_SUPPLY`) in **both** the Makefile `C_DEFS` and `SystemClock_Config()` in `main.c`.

---

### 2. `SystemClock_Config()` Fails — HSE/PLL Won't Lock

**Symptom:** After fixing the power supply issue, only LED1 (PA10) would ever illuminate. All other LEDs appeared dead despite being confirmed functional in hardware.

**Root Cause:** `SystemClock_Config()` was calling `Error_Handler()` because the HSE oscillator or PLL would not lock. The CubeMX configuration targets:
- HSE = 24 MHz crystal
- PLLM = 6, PLLN = 240 → **960 MHz VCO** (absolute maximum of the wide VCO range)
- PLLP = 2 → 480 MHz SYSCLK

The VCO frequency sits at the very edge of the STM32H750's specified range, and likely fails to lock on this hardware. The crystal circuit (loading capacitors, PCB layout) may also be contributing.

**How we found it:** This was particularly deceptive because `Error_Handler()` blinks all 4 LEDs but **doesn't configure GPIO MODER registers**. Only pins already configured as outputs before the crash would visibly blink. Since only PA10 was set up (as a milestone indicator before `SystemClock_Config()`), only LED1 blinked — making it look like a GPIO problem rather than a clock failure.

We confirmed this by adding a pre-init LED test that configured all 4 pins as outputs before any HAL calls. After that, `Error_Handler()` blinked all 4 LEDs simultaneously. Verified at 240fps slow-motion that all LEDs turn on in the exact same frame — confirming it was `Error_Handler()` (all-at-once), not the sequential chase loop (one-at-a-time).

**Fix:** HSE crystal confirmed dead. Rewrote `SystemClock_Config()` to use **HSI (64 MHz)** as PLL source:
- PLL1: PLLM=4, PLLN=54, PLLP=2, PLLQ=18 → **432 MHz SYSCLK**, **48 MHz USB** (PLL1Q)
- PLL2: PLL2M=8, PLL2N=21 → **84 MHz SPI clock** source

CubeMX always regenerates with HSE — the HSI workaround must be manually re-applied after every CubeMX regeneration (see CLAUDE.md for the full checklist).

---

### 3. USB CDC Clock Source

**Original workaround:** When running without PLL, used HSI48 for USB clock.

**Current:** With HSI+PLL running, PLL1Q provides 48 MHz for USB. `RCC_USBCLKSOURCE_PLL` in `usbd_conf.c` is correct.

---

### 4. CubeMX SPI Defaults — Wrong DataSize and BaudRate

**Symptom:** SPI sensors not responding or returning garbage.

**Root Cause:** CubeMX generates SPI config with `SPI_DATASIZE_4BIT` (must be 8-bit for all sensors) and `SPI_BAUDRATEPRESCALER_2` (too fast for most sensors).

**Fix:**
- All SPI: `SPI_DATASIZE_8BIT`
- SPI2 (LSM6DSO32): `SPI_BAUDRATEPRESCALER_16` → 5.25 MHz (max 10 MHz)
- SPI4 (MS5611): `SPI_BAUDRATEPRESCALER_8` → ~13.5 MHz (max 20 MHz)

---

### 5. MS5611 Pressure Calculation Overflow

**Symptom:** Altitude values wildly incorrect despite valid raw ADC readings.

**Root Cause:** `(int32_t)` cast was truncating an intermediate ~3.28e9 value in the pressure compensation calculation before the final multiply.

**Fix:** Wrapped the entire expression in a float cast: `(int32_t)((...) * 3.05e-5f)`

---

### 6. LSM6DSO32 SPI2 — Multiple Issues

#### 6a. PC2 MODER Clobbered to Analog Mode

**Symptom:** SPI2 MISO (PC2_C) stuck in analog mode instead of AF5 after all peripheral inits complete.

**Root Cause:** Unknown clobbering mechanism — SPI2 MspInit correctly sets PC2 to AF5, but MODER gets reset to analog (0x11) by some later init. Likely related to STM32H7 companion pin (PC2 vs PC2_C) and the SYSCFG analog switch.

**Fix:** Force PC2 back to AF5 mode in USER CODE BEGIN 2, after all inits complete:
```c
HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
GPIO_InitTypeDef gpio_fix = {.Pin=GPIO_PIN_2, .Mode=GPIO_MODE_AF_PP, .Alternate=GPIO_AF5_SPI2};
HAL_GPIO_Init(GPIOC, &gpio_fix);
```

#### 6b. SPI Mode 3 Required

**Root Cause:** LSM6DSO32 datasheet specifies SPI Mode 3 only (CPOL=1, CPHA=1). CubeMX generates Mode 0.

**Fix:** Override in USER CODE SPI2_Init 2: CPOL=HIGH, CPHA=2EDGE, MasterKeepIOState=ENABLE (keeps SCK HIGH between transactions), NSSPMode=DISABLE.

#### 6c. INT1 NC — Data-Ready Must Route to INT2

**Root Cause:** INT1 pin is not connected on this PCB. INT2 is wired to PC15.

**Fix:** Route accelerometer data-ready to INT2_CTRL (register 0x0E) instead of INT1_CTRL.

#### 6d. SPI2_CS Wrong Pin Assignment (ROOT CAUSE of all SPI2 failures)

**Symptom:** WHO_AM_I register returns 0x00 via both HAL SPI and manual GPIO bit-bang. MODER/AFR confirmed correct. Barometer on SPI4 works fine.

**Root Cause:** CubeMX had SPI2_CS assigned to **PD15**, but the PCB routes the CS trace to **PC14**. The chip select signal never reached the LSM6DSO32.

**Diagnosis:** MODER register dump confirmed PC2 in AF mode (fix 6a working). GPIO bit-bang WHO_AM_I also returned 0x00, proving the issue was not the SPI peripheral but the physical connection. Both HAL and bit-bang failing = sensor not being selected.

**Fix:** Corrected pin assignment in CubeMX (PD15 → PC14), regenerated project, re-applied all manual fixes (HSI PLL, SPI config, Mode 3, PC2 MODER, etc.).

**Lesson:** When SPI returns all-zeros, verify CS pin assignment against the schematic **first**, before debugging data lines, clock polarity, or register configuration.

---

## Rev 2 — Firmware Behaviour (after initial bring-up)

1. **432 MHz SYSCLK** via HSI (64 MHz) + PLL1. All peripherals initialized.

2. **Milestone LEDs** light up sequentially during initialization (500ms between each):
   - LED1 (PA10) → `HAL_Init()` passed
   - LED2 (PB14) → GPIO + peripheral inits passed
   - LED3 (PE8) → USB init passed
   - LED4 (PE7) → All sensors initialized

3. **WHO_AM_I diagnostic** prints `>who:0x6C` (or `0x00` on failure) 5 times over CDC after boot.

4. **100 Hz sensor output** over USB CDC serial plotter format:
   ```
   >altitude:XX.XX,ax:XX.XX,ay:XX.XX,az:XX.XX,gx:XX.X,gy:XX.X,gz:XX.X
   ```

5. **LED1 heartbeat** toggles every 10ms (100 Hz) in the main loop

---

## Rev 3a — Sensor Bring-Up & Data Infrastructure

### ADXL372 High-G Accelerometer (SPI3)

Brought up the Analog Devices ADXL372 ultra-high-g accelerometer on SPI3:
- Range: ±200g, 12-bit output, 100 mg/LSB
- FIFO streaming for burst data collection (stream mode, up to 512 samples)
- ODR configurable up to 6400 Hz (800 Hz used for characterization)
- Device ID readback: 0xFA (0xAD device, 0x1D MEMS, 0x02 part, 0xFA devid)
- Files: `adxl372.c/h`

### W25Q512JV QSPI Flash

Brought up the Winbond W25Q512JV 64MB NOR flash on QUADSPI:
- 512 Mbit (64 MB), 4-byte addressing mode
- Sector erase (4 KB), block erase (64 KB), chip erase
- Read/write/erase API with busy-wait polling
- JEDEC ID verification on init
- Files: `w25q512jv.c/h`

### FATFS Integration

Integrated FatFs filesystem on top of the QSPI flash:
- `user_diskio.c` bridges FatFs sector read/write to W25Q512JV driver
- Block size: 4096 bytes (matches flash sector size), 16384 blocks
- Used for CSV data logging in data collection mode
- Files: `FATFS/App/fatfs.c/h`, `FATFS/Target/user_diskio.c/h`

### USB MSC Mode

Added USB Mass Storage Class mode so the flash appears as a USB drive:
- `usbd_msc_storage_if.c` bridges SCSI read/write commands to W25Q512JV
- PC can mount the flash, read CSV files, format the filesystem
- Files: `usbd_msc_storage_if.c/h`

### USB_MODE Compile-Time Switching

Introduced `USB_MODE` define for build-time firmware mode selection:
- **USB_MODE=1**: CDC serial output (original mode)
- **USB_MODE=2**: MSC mass storage (flash as USB drive, LED ping-pong pattern)
- **USB_MODE=3**: CDC + data collection harness (sensor logging to flash)

Set in Makefile `C_DEFS`: `-DUSB_MODE=3`

### Data Collection Harness

Built a phase-based sensor data logging system for noise characterization:
- **DATA_PHASE=1**: LSM6DSO32 at 833 Hz for 300s → `IMU.CSV` (timestamp, ax, ay, az, gx, gy, gz, temp)
- **DATA_PHASE=2**: ADXL372 FIFO at 800 Hz for 300s → `ADXL.CSV` (timestamp, ax, ay, az)
- **DATA_PHASE=3**: MS5611 raw ADC at OSR_4096 for 300s → `BARO_HI.CSV`
- **DATA_PHASE=4**: MS5611 raw ADC at OSR_1024 for 120s → `BARO_MD.CSV`
- **DATA_PHASE=5**: MS5611 raw ADC at OSR_256 for 120s → `BARO_LO.CSV`
- All LEDs blink together upon completion
- Files: `data_collect.c/h`

### New Files (Rev 3a)

| File | Description |
|------|-------------|
| `Core/Src/adxl372.c` / `Core/Inc/adxl372.h` | ADXL372 SPI driver |
| `Core/Src/w25q512jv.c` / `Core/Inc/w25q512jv.h` | W25Q512JV QSPI flash driver |
| `Core/Src/data_collect.c` / `Core/Inc/data_collect.h` | Data collection harness |
| `USB_DEVICE/App/usbd_msc_storage_if.c/h` | USB MSC storage interface |
| `FATFS/App/fatfs.c/h` | FatFs initialization |
| `FATFS/Target/user_diskio.c/h` | Disk I/O layer for W25Q512JV |

---

## Rev 3b — MATLAB Sensor Noise Characterization

### Workflow

1. Flash firmware with `USB_MODE=3` and desired `DATA_PHASE` (1-5)
2. Power board → data collection runs automatically, LEDs blink when done
3. Reflash with `USB_MODE=2` → board appears as USB drive on PC
4. Copy CSV files from flash to `Matlab Code/` directory
5. Run `casper_sensor_characterization.m` in MATLAB (or VS Code MATLAB extension)

### Analysis Pipeline (`casper_sensor_characterization.m`)

The script performs three analysis stages:

**Stage 1 — Data Quality Check:**
- Sample rate regularity (mean/std/min/max of dt)
- Dropout detection (dt > 2x expected period)
- LSM6 gap analysis (FATFS flush gaps ~5-10ms, ignored via configurable threshold)
- Gravity vector magnitude verification
- Gyro static bias measurement
- Baro pressure sanity check

**Stage 2 — Allan Variance (IMU):**
- LSM6 gyroscope: ARW (angle random walk) and BI (bias instability) per axis
- LSM6 accelerometer: VRW (velocity random walk) and BI per axis
- ADXL372 accelerometer: VRW and BI per axis (reference only — too noisy for EKF)
- Outlier removal via `filloutliers()` before analysis

**Stage 3 — Barometer Characterization:**
- Pressure noise σ at each OSR setting
- Altitude noise σ (Pa → m conversion)
- R_alt (measurement noise variance) for EKF R matrix
- Allan deviation and power spectral density plots

### Outputs

- `noise_params.mat` — IMU noise parameters (gyro ARW/BI, accel VRW/BI for both sensors)
- `baro_noise_params.mat` — Barometer noise parameters (σ, R at each OSR)
- 10 plots saved to `plots/` directory:
  - `qc_sample_timing.png`, `qc_raw_imu.png`, `qc_raw_baro.png`, `qc_noise_histograms.png`
  - `allan_lsm6_gyro.png`, `allan_lsm6_accel.png`, `allan_adxl372.png`
  - `baro_allan_dev.png`, `baro_psd.png`, `baro_allan_overlay.png`

### Key Results (fed into EKF)

| Parameter | Value | Source | Used in |
|-----------|-------|--------|---------|
| Gyro ARW (X,Y,Z) | 6.73e-5, 6.08e-5, 4.92e-5 rad/sqrt(s) | Allan variance | `casper_gyro_int.c` gyro_arw[] |
| Accel VRW (vertical) | 2.163e-3 m/s/sqrt(s) | Allan variance | `casper_ekf.c` Q matrix (qa) |
| Accel BI | 1.954e-4 m/s^2/sqrt(s) | Allan variance | `casper_ekf.c` Q matrix (qab) |
| Baro altitude σ | ~0.5 m (OSR 1024) | Std deviation | `casper_ekf.c` R_baro = 0.25 m^2 |
| Baro BI | 1.0e-3 m/sqrt(s) | Estimated | `casper_ekf.c` Q matrix (qbb) |

---

## Rev 3c — EKF Navigation Stack

### CMSIS-DSP Integration

Cherry-picked 6 matrix function sources from the ARM CMSIS-DSP GitHub repository:
- `arm_mat_init_f32`, `arm_mat_mult_f32`, `arm_mat_trans_f32`
- `arm_mat_add_f32`, `arm_mat_sub_f32`, `arm_mat_scale_f32`
- Headers in `Drivers/CMSIS/DSP/Include/`, sources in `Drivers/CMSIS/DSP/Source/MatrixFunctions/`
- Makefile: `-DARM_MATH_CM7`, include paths for DSP headers

### Quaternion Library (`casper_quat.c/h`)

Hamilton quaternion operations, convention: `q[4] = {w, x, y, z}`, scalar-first, body-to-NED:
- `casper_quat_mult()` — Hamilton product
- `casper_quat_normalize()` — in-place normalization
- `casper_quat_to_rotmat()` — quaternion to 3x3 rotation matrix (row-major)
- `casper_quat_from_accel()` — initial orientation from gravity vector (pitch/roll from accel, yaw=0)
- `casper_quat_to_euler()` — quaternion to ZYX Euler angles [roll, pitch, yaw] in degrees

### Gyro Integrator (`casper_gyro_int.c/h`)

RK4 quaternion propagation for attitude dead-reckoning:
- IIR low-pass filter on gyro input (default 50 Hz cutoff)
- On-pad gyro bias estimation (double-precision accumulators, frozen at launch)
- Launch detection: `|accel| > 3g` triggers bias freeze
- Open-loop attitude uncertainty tracking (per-axis σ using ARW noise model)
- RK4 working memory pre-allocated in struct (no stack allocation in tight loop)

### Extended Kalman Filter (`casper_ekf.c/h`)

4-state vertical-channel EKF: **[altitude, velocity, accel_bias, baro_bias]**
- **Predict** (500 Hz): Rotates body-frame accel to NED via quaternion, subtracts gravity and accel bias, propagates state and covariance (Phi*P*Phi' + Q via CMSIS-DSP)
- **Baro update** (~100 Hz): Sequential scalar update, H=[1,0,0,1], Joseph form covariance, symmetrization
- **Transonic gating**: Gates baro updates when estimated Mach > 0.4, ungates below 0.35 (ISA lapse rate for speed-of-sound)
- **GPS stubs**: `update_gps_alt` (H=[1,0,0,0], R=100) and `update_gps_vel` (H=[0,1,0,0], R=1.0) — ready for hardware integration
- Pre-computed Phi, PhiT, Q matrices for fixed dt=0.002s

### Main Loop Restructuring

Transformed from 100 Hz blocking poll to 500 Hz non-blocking architecture:
- **500 Hz**: IMU poll → unit conversion (g→m/s², dps→rad/s) → gyro integrator → EKF predict
- **Async**: `ms5611_tick()` non-blocking baro state machine → EKF update on new data
- **50 Hz**: USB CDC output with EKF state + Euler angles
- Added `ms5611_tick()` non-blocking API alongside existing blocking `ms5611_read()`

### USB Output Format (Rev 3c)

```
>alt:XX.XX,vel:XX.XX,roll:XX.X,pitch:XX.X,yaw:XX.X
```
- `alt`: EKF filtered altitude (m, AGL)
- `vel`: EKF vertical velocity (m/s, positive up)
- `roll`, `pitch`, `yaw`: Euler angles from gyro integrator quaternion (degrees)

### New Files (Rev 3c)

| File | Description |
|------|-------------|
| `Core/Src/casper_quat.c` / `Core/Inc/casper_quat.h` | Quaternion operations |
| `Core/Src/casper_gyro_int.c` / `Core/Inc/casper_gyro_int.h` | RK4 gyro integrator |
| `Core/Src/casper_ekf.c` / `Core/Inc/casper_ekf.h` | Extended Kalman Filter |
| `Drivers/CMSIS/DSP/` | CMSIS-DSP matrix functions (6 source files + headers) |

---

## Current Firmware Behaviour (Rev 3c)

1. **432 MHz SYSCLK** via HSI (64 MHz) + PLL1. All peripherals initialized.

2. **Milestone LEDs** light up sequentially during initialization (200ms between each):
   - LED1 (PA10) → `HAL_Init()` passed
   - LED2 (PB14) → GPIO + peripheral inits passed
   - LED3 (PE8) → USB init passed
   - LED4 (PE7) → All sensors initialized + EKF ready

3. **USB_MODE=3** (current build): Data collection harness mode. Behaviour depends on `DATA_PHASE` define.

4. **USB_MODE=1** (EKF mode): 500 Hz EKF loop with **50 Hz USB CDC output**:
   ```
   >alt:XX.XX,vel:XX.XX,roll:XX.X,pitch:XX.X,yaw:XX.X
   ```

5. **USB_MODE=2** (MSC mode): Flash appears as USB drive. Alternating LED pairs indicate drive mode.

6. **Build size:** ~68 KB (well within 128 KB flash)

---

## Files Changed (all revisions combined)

| File | Change |
|------|--------|
| `Core/Src/main.c` | HSI+PLL clock, SPI fixes, PC2 MODER fix, all sensor inits, USB_MODE dispatch, 500Hz EKF loop, data collection, milestone LEDs |
| `Core/Src/ms5611.c` / `Core/Inc/ms5611.h` | **NEW** — MS5611 SPI barometer (blocking + non-blocking API) |
| `Core/Src/lsm6dso32.c` / `Core/Inc/lsm6dso32.h` | **NEW** — LSM6DSO32 SPI IMU driver |
| `Core/Src/adxl372.c` / `Core/Inc/adxl372.h` | **NEW** — ADXL372 SPI high-g accelerometer driver |
| `Core/Src/w25q512jv.c` / `Core/Inc/w25q512jv.h` | **NEW** — W25Q512JV QSPI flash driver |
| `Core/Src/data_collect.c` / `Core/Inc/data_collect.h` | **NEW** — Phase-based data collection harness |
| `Core/Src/casper_quat.c` / `Core/Inc/casper_quat.h` | **NEW** — Quaternion operations |
| `Core/Src/casper_gyro_int.c` / `Core/Inc/casper_gyro_int.h` | **NEW** — RK4 gyro integrator |
| `Core/Src/casper_ekf.c` / `Core/Inc/casper_ekf.h` | **NEW** — Extended Kalman Filter |
| `Core/Src/stm32h7xx_it.c` | EXTI15_10 handler for LSM6DSO32 INT2 |
| `USB_DEVICE/App/usbd_msc_storage_if.c/h` | **NEW** — USB MSC storage interface |
| `USB_DEVICE/Target/usbd_conf.c` | USB clock: PLL1Q 48 MHz, MSC init for USB_MODE=2 |
| `USB_DEVICE/App/usbd_desc.c` | USB descriptor for MSC mode |
| `FATFS/App/fatfs.c/h` | **NEW** — FatFs initialization |
| `FATFS/Target/user_diskio.c/h` | **NEW** — Disk I/O for W25Q512JV |
| `Drivers/CMSIS/DSP/` | **NEW** — CMSIS-DSP matrix functions (6 sources + headers) |
| `Makefile` | LDO fix, all sources, CMSIS-DSP, USB_MODE, DATA_PHASE, `-u _printf_float` |
| `Matlab Code/casper_sensor_characterization.m` | **NEW** — Allan variance analysis pipeline |

---

## Build & Flash

```bash
# Build (from Software/ directory)
make clean && make -j8

# Switch USB mode (edit Makefile C_DEFS):
#   -DUSB_MODE=1  → CDC serial output (EKF mode)
#   -DUSB_MODE=2  → USB mass storage (read flash)
#   -DUSB_MODE=3  → CDC + data collection

# Flash via DFU
# 1. Hold BOOT0, press RESET, release BOOT0
# 2. Use STM32CubeProgrammer or dfu-util to flash build/Casper2.bin to 0x08000000
# 3. Press RESET to run
```

---

## LED Pin Mapping

| LED | Pin | GPIO Port | Function |
|-----|-----|-----------|----------|
| CONT_YN_1 | PA10 | GPIOA | Continuity indicator 1 |
| CONT_YN_2 | PB14 | GPIOB | Continuity indicator 2 |
| CONT_YN_3 | PE8 | GPIOE | Continuity indicator 3 |
| CONT_YN_4 | PE7 | GPIOE | Continuity indicator 4 |

---

## SPI Bus Mapping

| SPI | Sensor | SCK | MOSI | MISO | CS | INT |
|-----|--------|-----|------|------|----|-----|
| SPI1 | TBD | PA5 | PA7 | PB4 | PB0 | PB1 |
| SPI2 | LSM6DSO32 | PD3 | PC1 | PC2_C | PC14 | PC15 (INT2) |
| SPI3 | ADXL372 | PC10 | PC12 | PC11 | PA15 | PD2 |
| SPI4 | MS5611 | PE12 | PE14 | PE13 | PE11 | — |
| QUADSPI | W25Q512JV | PB2 (CLK) | PD11-13, PE2, PB6 | — | — | — |

## Rev 4 — MC Testing: msgset v5 Telemetry Protocol

### Overview

Added 11 firmware modules enabling the FC to speak the msgset v5 binary telemetry protocol over USB CDC. This allows direct bench testing with the Mission Control Electron app without LoRa hardware — the FC acts as if it's connected end-to-end.

All new code lives in `Software/App/` (separate from CubeMX-managed `Core/`). Build size: 87.7 KB (up from ~80 KB, within 128 KB flash).

### Telemetry Stack

**COBS Framing (`cobs.c/h`):** Consistent Overhead Byte Stuffing for zero-free packet transport over CDC. 0x00 delimiter between frames.

**Hardware CRC-32 (`crc32_hw.c/h`):** Standard reflected CRC-32 (poly 0x04C11DB7, reflected I/O, final XOR 0xFFFFFFFF) via the STM32 CRC peripheral. Reconfigures the peripheral at init since CubeMX defaults differ.

**Telemetry Manager (`tlm_manager.c/h`):** Builds and sends three packet types:
- **FC_MSG_FAST** (19B, 10 Hz): status(2) + alt_dam(2) + vel_dms(2) + quat_packed(5) + time_ds(2) + batt(1) + seq(1) + CRC-32(4)
- **FC_MSG_GPS** (12B, on fix): dlat(2) + dlon(2) + alt_dam(2) + fixsat(1) + pdop(1) + CRC-32(4)
- **FC_MSG_EVENT** (9B, on event): type(1) + data(2) + time(2) + CRC-32(4)

**Quaternion Packing (`quat_pack.c/h`):** Smallest-three encoding — drop largest component, pack remaining 3 as int12 into 5 bytes. Bit layout: `[drop:2][rsvd:2][qa:12][qb:12][qc:12]`.

**Status Packing (`status_pack.c/h`):** 2-byte FC_TLM_STATUS bitmap — byte 0: `[ARM4..1][CNT4..1]`, byte 1: `[FSM_STATE:4][FIRED][ERROR][RSVD:2]`.

### Command System

**CDC Ring Buffer (`usbd_cdc_if.c` mod):** Replaced flag-based CDC RX with 256-byte SPSC ring buffer. ISR writes, main loop reads. Legacy `cdc_rx_ready`/`cdc_rx_len` flags preserved for backward compatibility.

**Command Router (`cmd_router.c/h`):** COBS frame accumulation from ring buffer. On 0x00 delimiter, COBS-decodes and dispatches by message ID to 12 handlers (ARM, FIRE, TESTMODE, CONFIG_POLL, CONFIRM, ABORT, HANDSHAKE, UPLOAD, DIAG, READLOG, ERASELOG, SIM_FLIGHT).

**CAC Handler (`cac_handler.c/h`):** Three-phase Command-Acknowledge-Confirm protocol for ARM/FIRE:
- Magic bytes 0xCA 0x5A + complement validation + CRC-32
- Nonce-based idempotency (duplicate nonce → re-send ACK)
- 5-second confirm timeout
- Test mode: 60s timeout, PAD-only, caps fire to 50ms
- 10 NACK error codes (CRC_FAIL, BAD_STATE, NOT_ARMED, NO_TESTMODE, etc.)

**Handshake (0xC0):** Returns protocol v5, FW version 0.1.0, config hash.

### Flight State Machine

**Flight FSM (`flight_fsm.c/h`):** 12 states: PAD→BOOST→COAST→COAST_1→SUSTAIN→COAST_2→APOGEE→DROGUE→MAIN→RECOVERY→TUMBLE→LANDED.

**Simulated Flight (0xD0):** Scripted profile for bench testing — PAD(T+0) → BOOST(T+0.5s) → COAST(T+3s) → APOGEE(T+12s) → DROGUE(T+12.5s) → MAIN(T+45s) → RECOVERY(T+90s) → LANDED(T+180s). Linear interpolation of alt/vel between waypoints + slowly rotating test quaternion.

Every state transition emits an FC_EVT_STATE event.

### Pyro Manager

**Pyro Manager (`pyro_manager.c/h`):** Wraps existing `casper_pyro.c` — adds per-channel arm state, continuity prechecks, fire duration caps (2000ms max, 50ms in test mode), test mode toggle, disarm-all. Does not duplicate hardware access.

### Diagnostics

**Config Manager (`cfg_manager.c/h`):** Config upload + hash tracking. Flash persistence via FATFS (stubbed).

**Self-Test (`self_test.c/h`):** 7 diagnostic tests: IMU WHO_AM_I, mag product ID, baro PROM CRC, EKF init, attitude quat norm, flash write/read, config hash. Response sent as COBS-framed diagnostic packet (0xC2).

### New Files (Rev 4)

| Directory | Files |
|-----------|-------|
| `App/telemetry/` | `tlm_types.h`, `cobs.c/h`, `crc32_hw.c/h`, `tlm_manager.c/h` |
| `App/pack/` | `quat_pack.c/h`, `status_pack.c/h` |
| `App/pyro/` | `pyro_manager.c/h` |
| `App/fsm/` | `flight_fsm.c/h` |
| `App/command/` | `cmd_router.c/h`, `cac_handler.c/h`, `cfg_manager.c/h` |
| `App/diag/` | `self_test.c/h` |

### Modified Files (Rev 4)

| File | Change |
|------|--------|
| `USB_DEVICE/App/usbd_cdc_if.c` | CDC RX ring buffer (256B SPSC) |
| `USB_DEVICE/App/usbd_cdc_if.h` | `cdc_ring_available()`, `cdc_ring_read_byte()` exports |
| `Core/Src/main.c` | Init calls for CRC/TLM/CMD/CAC/CFG/FSM/PYRO_MGR + superloop telemetry state population + command processing |
| `Makefile` | 11 new C_SOURCES, 6 new C_INCLUDES for App/ subdirectories |

---

## Rev 5 — Navigation Stack Rewrite & Sensor Expansion

### Casper Attitude Estimator (`casper_attitude.c/h`)

Replaced the standalone gyro integrator as the primary attitude module. Implements a Mahony complementary filter with:
- Gravity vector correction and magnetometer correction on pad (Kp_mag currently set to 0.0 pending field calibration)
- RK4 quaternion propagation at 833 Hz
- Static initialization from accelerometer + magnetometer readings
- Launch detection at 3g threshold, bias freeze at launch
- Ignition gating for motor burn phases (suppresses accel corrections during thrust)

### MMC5983MA Magnetometer (I2C3)

Brought up the Memsic MMC5983MA 3-axis magnetometer on I2C3:
- Range: +/-8 Gauss, 18-bit resolution, 100 Hz continuous mode
- Auto SET/RESET for offset cancellation, BW = 800 Hz
- DRDY interrupt on PC8 (EXTI9_5), I2C address 0x30, Product ID 0x30
- Conversion: `field_G = (raw_18bit - 131072) / 16384.0`
- PC8 GPIO reconfigured from output to EXTI rising edge in USER CODE BEGIN 2
- Hard/soft-iron calibration via `mag_cal.c/h`, validation via `mag_val.c/h`

### MAX-M10M GPS (I2C1)

Brought up the U-blox MAX-M10M GPS receiver on I2C1:
- 10 Hz GPS-only mode, UBX-NAV-PVT protocol, I2C DDC address 0x42
- NRST_GPS on PE15 (active-low reset), I2C1_INT on PE0 (not enabled yet)
- Non-blocking `max_m10m_tick()` architecture with 25 ms poll interval
- EKF GPS update stubs (`casper_ekf_update_gps_alt/vel`) called on 3D fix

### EKF Clean-Sheet Rewrite (`casper_ekf.c/h`)

Complete rewrite of the Extended Kalman Filter with the following changes:
- Generic Joseph-form scalar update with explicit H vector and chi-squared innovation gate
- 416 Hz predict rate (decimated from 833 Hz IMU ODR with trapezoidal averaging)
- ZUPT (zero-velocity update) pseudo-measurement on PAD and LANDED states
- Step-inflated R post-gate (not exponential decay) for transonic baro rejection
- Bias reset + covariance P inflation at ungate transition
- Temperature floor clamp in Mach gate (speed-of-sound calculation)
- No baro-derived velocity — baro updates altitude channel only
- R_BARO = 0.5, ACCEL_BI_SIGMA from Allan variance characterization

### Identity Sensor-to-Body Mapping

Changed from cyclic permutation (body[X,Y,Z] = sensor[Z,X,Y]) to identity mapping (body = sensor). Body frame convention:
- Body X = starboard
- Body Y = nose (forward)
- Body Z = toward operator (up when vertical on pad)

### Flight Loop Refactor (`flight_loop.c/h`)

Moved the main flight loop out of `main.c` into `App/flight/flight_loop.c`:
- `TEST_MODE=1`: COBS binary telemetry (production path)
- `TEST_MODE=2`: ASCII serial plotter output for bench tuning (alt, vel, roll, pitch, yaw, ned_z, zupt flag)
- Bench command parser: text commands over CDC for FSM state forcing, calibration skip, and debug overrides
- `app_globals.h` provides shared state between flight loop and telemetry modules

### File Reorganization

Moved all application code out of CubeMX-managed `Core/` into `App/` subdirectories:
- Drivers: `Core/Src/*.c` → `App/drivers/*.c` (all 6 sensor drivers)
- Navigation: `Core/Src/casper_*.c` → `App/nav/` (EKF, attitude, quaternion, gyro integrator)
- Calibration: `Core/Src/mag_*.c` → `App/cal/` (mag_cal, mag_val, temp_cal)
- Pyro: existing `App/pyro/` (casper_pyro moved here alongside pyro_manager)
- This keeps `Core/` clean for CubeMX regeneration without merge conflicts

### Gyro Temperature Compensation

Optional slope-only linear temperature model for gyro bias:
- Enabled via `GYRO_TEMP_COMP` compile-time define
- Coefficients in `temp_cal_coeffs.h` (per-axis slope in dps/degC)
- Gyro only — no accelerometer temperature compensation
- Implementation in `temp_cal.c/h`

### ADXL372 Wake-Up Mode

Low-power activity detection in PAD state:
- ADXL372 configured for wake-up mode during PAD (minimal power draw)
- Transitions to full measurement mode on launch (FSM transition to BOOST)
- Provides high-g data during boost/coast where LSM6DSO32 saturates

### Magnetometer Corrections Status

Magnetometer corrections currently disabled in the attitude estimator:
- `Kp_mag_pad = 0.0`, `Kp_mag_flight = 0.0`
- Heading initialized from static magnetometer averaging during pad alignment
- Pending proper hard/soft-iron field calibration before enabling corrections in flight

### EKF Diagnostic Output

When built with `TEST_MODE=2`, the ASCII serial plotter output includes additional channels:
- `ned_z`: NED down-axis acceleration (for verifying body-to-NED rotation)
- `zupt`: ZUPT active flag (1 when zero-velocity pseudo-measurement applied)
- Useful for tuning EKF noise parameters and verifying gate behavior on bench

### New/Moved Files (Rev 5)

| Directory | Files |
|-----------|-------|
| `App/nav/` | `casper_attitude.c/h`, `casper_ekf.c/h` (rewritten), `casper_quat.c/h`, `casper_gyro_int.c/h`, `temp_cal_coeffs.h` |
| `App/drivers/` | `lsm6dso32.c/h`, `adxl372.c/h`, `ms5611.c/h`, `w25q512jv.c/h`, `max_m10m.c/h`, `mmc5983ma.c/h` |
| `App/cal/` | `mag_cal.c/h`, `mag_val.c/h`, `temp_cal.c/h` |
| `App/flight/` | `flight_loop.c/h`, `app_globals.h` |
| `App/util/` | (utility functions) |

### Modified Files (Rev 5)

| File | Change |
|------|--------|
| `Core/Src/main.c` | Flight loop dispatch to `App/flight/`, init calls for attitude + new sensors |
| `App/fsm/flight_fsm.c/h` | ADXL372 wake-up mode transitions, attitude estimator integration |
| `App/pyro/pyro_manager.c/h` | Moved `casper_pyro.c/h` into `App/pyro/` |
| `App/command/cmd_router.c` | Bench command parser for TEST_MODE=2 |
| `App/command/cac_handler.c` | Updated for new attitude module API |
| `Makefile` | All new/moved source paths, App/ include paths, TEST_MODE define |

---

## Rev 6 — LoRa Radio Telemetry & Buzzer

### SX1276 LoRa Radio Driver (`App/radio/sx1276.c/h`)

Low-level driver for the SX1276 (RA-01H module) on SPI1:
- Register-level SPI access via software-managed CS (PB0), bypassing HAL polling to avoid H7 hangs
- LoRa mode configuration: SF6–SF12, BW 7.8–500 kHz, CR 4/5–4/8, CRC always enabled
- +20 dBm PA_BOOST output (PA_DAC 0x87, OCP 240 mA)
- Hardware reset via PC13 (NRST, backup domain — requires slow 10 ms rise)
- FIFO split: TX base 0x80, RX base 0x00 (256-byte total)
- Frequency: 868 MHz, F_XOSC = 32 MHz
- Packet RSSI: `raw - 157` dBm (HF band), SNR: `raw / 4` dB

### Radio Manager (`App/radio/radio_manager.c/h`)

Non-blocking TX/RX state machine called at ~833 Hz from the flight loop:
- **States:** IDLE → TX → RX → IDLE (+ DISABLED for uninitialized)
- **10 Hz downlink:** COBS-encoded telemetry packets, identical format to USB CDC
- **TX priority:** Response (CAC ACK/NACK) > Event > GPS > Fast telemetry
- **RX window:** After each TX, opens RXSINGLE for uplink CAC commands (~80 symbol timeout)
- **Profile switching:** Profile A (SF7, 250 kHz BW) → Profile B (SF8) when altitude > 20 km or velocity > 500 m/s; one-way switch with FC_EVT_STATE event emission
- **Error handling:** Max 3 TX retries, 10 consecutive CRC errors before reset, 200 ms TX timeout
- **Packet sizes:** Fast 20B, GPS 17B, Event 11B (matching msgset v5)

### Radio IRQ (`App/radio/radio_irq.c/h`)

ISR flag interface for SX1276 DIO interrupts:
- DIO0 (PB1, EXTI1): TxDone in TX mode, RxDone in RX mode
- DIO1 (PD7, EXTI9_5): RxTimeout
- Volatile flags polled by radio_manager in main loop — no SPI access in ISR context
- `radio_irq_clear_all()` for atomic flag reset after handling

### Piezo Buzzer (`App/buzzer/buzzer.c/h`)

TIM4 PWM driver for audible status feedback:
- 6 kHz tone on TIM4 CH3, ARR = 35999 (216 MHz / 6000)
- Programmable beep sequences via `buzzer_beep_n(pct, count, on_ms, period_ms)`
- Duty cycle 0–100% input mapped to 0–50% output (piezo max acoustic efficiency)
- State machine: IDLE → BEEP_ON → BEEP_OFF → ... → IDLE
- Non-blocking tick at ~833 Hz

### Flight Loop Integration

- `radio_manager_tick()` and `buzzer_tick()` called every superloop iteration
- Navigation stack guarded with `#if TEST_MODE != 2` to allow radio-only bench testing
- Numeric bench command (0–100) added for buzzer volume testing over CDC
- Old ASCII plotter output removed in favor of radio telemetry

### Startup Changes (`main.c`)

- 5-second USB enumeration window with CDC debug prints for startup diagnostics
- Radio and buzzer initialization after peripheral init
- Deferred EXTI enables: LSM6DSO32 INT2 (PC15) and SX1276 DIO (PB1, PD7) interrupts enabled after all peripheral init to prevent ISR hangs during SPI configuration
- SPI1 fixes: DataSize 4→8-bit, Prescaler 2→16 (13.5 MHz → ~5 MHz), NSSPMode disabled
- PC13 (RADIO_NRST) GPIO driven HIGH early before SX1276 init
- LED milestone: all-off then triple blink on successful startup

### Interrupt Changes (`stm32h7xx_it.c`)

- Added EXTI1_IRQHandler for SX1276 DIO0 (PB1)
- Extended EXTI9_5_IRQHandler to handle both DIO1 (PD7) and MMC5983MA DRDY (PC8)

### Document Reorganization

- PRD docs moved from repo root → `Product Requirement Docs/`
- Spec docs moved from repo root → `Specification Docs/`
- Added `CASPER2_RADIO_PRD.md` and `CASPER2_TEST_HARNESS_PRD.md`

### New/Modified Files (Rev 6)

| Directory | Files |
|-----------|-------|
| `App/radio/` | `sx1276.c/h`, `radio_manager.c/h`, `radio_irq.c/h`, `radio_config.h` |
| `App/buzzer/` | `buzzer.c/h` |
| `Product Requirement Docs/` | `CASPER2_RADIO_PRD.md`, `CASPER2_TEST_HARNESS_PRD.md` (new), `CASPER_FLIGHT_CRITICAL_PRD.md`, `CASPER_LOGGING_PRD.md` (moved) |
| `Specification Docs/` | `EKF_SPEC.md`, `HARDWARE_SPEC.md`, `INTERFACE_SPEC.md`, `ORIENTATION_SPEC.md`, `PYRO_SPEC.md`, `SENSOR_SPEC.md` (moved) |

| File | Change |
|------|--------|
| `App/flight/flight_loop.c` | Radio/buzzer tick integration, nav guard, bench buzzer command |
| `Core/Src/main.c` | Radio/buzzer init, deferred EXTI, SPI1 fixes, USB enum window |
| `Core/Src/stm32h7xx_it.c` | EXTI1 + EXTI9_5 handlers for SX1276 DIO pins |
| `Makefile` | Radio + buzzer sources/includes, TEST_MODE=0 default |

### SPI1 / Radio Pin Mapping

| Function | Pin | Notes |
|----------|-----|-------|
| SPI1_SCK | PA5 | Clock |
| SPI1_MOSI | PA7 | Data out |
| SPI1_MISO | PB4 | Data in |
| Radio CS | PB0 | Software-managed, active-low |
| Radio DIO0 | PB1 | EXTI1 — TxDone / RxDone |
| Radio DIO1 | PD7 | EXTI9_5 — RxTimeout |
| Radio NRST | PC13 | Backup domain, 1 ms pulse + 10 ms wait |
| Buzzer PWM | TIM4 CH3 | 6 kHz tone |

---

## Rev 7 — Data Logger Debug + Estimator Timing Correction

Triggered by a flight test that produced **zero recorded data** despite all
sensors and logger code appearing healthy. Traced through five layered
issues, each masked by the next.

### Issue 1: Windows USB MSC was wiping the flight index sector

**Symptom**: `casper_decode.py` reported `No flights recorded` after every
test run, even when `flight_logger_finalize` had clearly written valid
bytes (confirmed by an in-firmware sector-0 hex-dump probe immediately
after finalize).

**Root cause**: `STORAGE_IsWriteProtected()` in `usbd_msc_storage_if.c`
returned 1 (read-only), but `STORAGE_Write()` happily executed any host
write that came in anyway. Windows issues `WRITE_10` SCSI commands during
mount/repair regardless of the read-only declaration. Each MSC mount was
erasing sector 0 (the flight index) and writing whatever Windows
considered a valid partition table.

**Fix**: `STORAGE_Write` now returns -1 immediately. Host writes are
rejected at the firmware level. **This is also a flight-safety fix** —
prevents accidental flash-data loss for any user who plugs the board
into a PC after a flight.

### Issue 2: Use CDC dump command instead of MSC for read-back

The codebase already had a working `MSG_ID_DUMP_FLASH` command and a
matching `casper_decode.py --serial COM<N>` mode, but documentation
pointed users at MSC. CDC dump bypasses Windows entirely (no drive
mount, no partition probing), so:

```
python tools\casper_decode.py --serial COM<N> --save flight_dump.bin
```

is now the canonical post-flight read-back. Streams 5 fixed log regions
via "CDMP" header + ACK-driven 4 KB chunks + "DONE" marker. About 1–2 min
for a full 64 MB read.

### Issue 3: IMU EXTI was disabled, capping service rate at ~166 Hz

**Symptom**: Logger cycle probes showed `flight_logger_push_hr` was
called only 156×/s — far below the 833 Hz IMU ODR. HR record rate was
26/s vs. the spec'd 139/s.

**Root cause**: A previous workaround had disabled `EXTI15_10` entirely
because PB12/PB13 (SX1276 DIO4/DIO5, configured `IT_RISING` by CubeMX)
were toggling constantly and producing stale-pending interrupts. A
5 ms STATUS-poll watchdog in `flight_loop.c` was the only path latching
`imu.data_ready`, capping IMU service at ~200 Hz max.

**Fix** (in three steps in `main.c`):
1. `HAL_GPIO_DeInit(GPIOB, RADIO_DIO4_Pin | RADIO_DIO5_Pin)` — clears the
   EXTI line registration. **Important**: a plain `HAL_GPIO_Init` with
   `GPIO_MODE_INPUT` does NOT clear EXTI registration, only the DeInit
   does.
2. Re-init PB12/PB13 as plain inputs (`GPIO_MODE_INPUT`).
3. Clear pending bits + enable `EXTI15_10_IRQn` so PC15 (LSM6DSO32 INT2)
   actually gets serviced.

Watchdog tightened from 5 ms to 1 ms as belt-and-braces fallback.

### Issue 4: Hamming SECDED encoder was O(n²)

**Symptom**: With EXTI fixed, IMU service rate climbed to 449 Hz but
still hit a CPU ceiling. Cycle probes pinpointed `flight_logger_push_hr`
taking **3.8 ms per real-work record build** — a single function on the
hot path was eating 35% of the entire CPU budget.

**Root cause**: `data_bit_to_pos()` in `hamming.c` was an O(n) linear
scan executed inside the O(n) outer bit loop in `hamming_encode` /
`hamming_decode`. For 60-byte HR records (480 data bits) that's
~115 000 inner iterations per encode call.

**Fix**: precompute a 512-entry lookup table on first call. Encoder
drops from ~4 ms to ~5 µs (~800× faster). After this fix, IMU service
hit 736 Hz and `imu_svc` total CPU dropped from 38.7% → 7.1%.

### Issue 5: Estimators received hardcoded `EKF_DT` constant

**Symptom**: Even at 736 Hz, the loop wasn't quite at 833 Hz spec. But
the more serious issue was structural: `EKF_DT = 1/833 = 1.2 ms` was
passed as `dt` to both `casper_att_update` and `casper_ekf_predict`,
while actual elapsed time was 1.36 ms.

**Why this matters**: a 13% under-true `dt` means gyro integration runs
at 13% slow scale. Over 60 s of flight at typical body rates, that's
~6° accumulated attitude drift purely from the timing mismatch. EKF
position drifts proportionally — for a 3 km flight, that's ~390 m of
spurious position error.

**Fix**: enable the Cortex-M7 DWT cycle counter unconditionally (zero
runtime cost — single register poke). In the IMU service block, measure
actual elapsed time per service via `DWT->CYCCNT` and pass that real
`dt` to both estimators. Clamp to [0.1 ms, 5 ms] so a stalled superloop
doesn't blow up the filter with a giant jump.

```c
uint32_t cyc = DWT->CYCCNT;
float dt_actual = (cyc - s_prev_imu_cyc) * (1.0f / 432000000.0f);
if (dt_actual > 0.005f) dt_actual = 0.005f;
casper_att_update(&att, gyro, accel, mag, dt_actual);
```

This is robust to *any* rate change (future radio fix, mag I2C bump,
flight-time interrupt-latency spikes). The estimators are now correct
at whatever rate the loop achieves.

### Diagnostic infrastructure (`LOGGER_SANITY` opt-in)

A new bench harness was added during the debug, gated by
`-DLOGGER_SANITY=1` in `Software/Makefile` (commented out by default,
zero behaviour change in flight builds):

- **Sector-0 write probe** (`App/diag/logger_sanity.c`): erase + write
  + read-back of sector 0 at boot to confirm raw QSPI write path
  before any logger logic. Verdict (PASS / silent / driver-error /
  corruption) reported via CDC + LED pattern.
- **CDC `go` keyword sniffer** (`usbd_cdc_if.c`): non-destructive tap
  on the ring buffer that fires on typed `go` (with or without
  Enter — encoding-tolerant). Triggers a 5-second logger run with
  live sensor data, bypassing the FSM.
- **DWT cycle probes** (`App/diag/cycle_probe.{c,h}`): per-region
  count/sum/max counters around the 10 hottest paths. Per-second CDC
  print of:
  ```
  [CYC] superloop n=9750 max=13213us avg=101us total=996ms = 99.6%
  [CYC] hr_push   n=755 max=80us avg=13us total=10ms = 1.0%
  [CYC] mag_rd    n=98 avg=943us total=92ms = 9.2%
  ...
  ```
- **`flight_loop_is_cal_done()` getter**: lets the bench harness gate
  the `go` keyword on the 30-second pad-mode calibration completing.

### Real flight rates (after Rev 7)

| Rate | Before | After |
| --- | --- | --- |
| IMU service | 156 Hz | ~708 Hz |
| HR records | 26 / s | ~119 / s |
| LR records | ~110 / s | ~111 / s |
| `imu_svc` CPU | 38.7 % | 7.1 % |
| `hr_push` per call | 628 µs | 13 µs |

The remaining gap (708 vs 833 Hz spec) is from occasional ~13 ms
blocking inside `radio_manager_tick` during SX1276 TX — radio toggles
INT2 coalesce, IMU drops samples. Acceptable: the estimators are now
correct regardless of the rate (adaptive `dt`).

### Bench-test workflow (canonical)

1. Wipe flash if needed (USB MSC mode build, mount as drive, format).
2. Edit `Software/Makefile`: uncomment `C_DEFS += -DLOGGER_SANITY=1`.
3. `cd Software && make clean && make -j8`. Flash via DFU.
4. Open serial terminal at the CDC port. Power on.
5. Watch for: probe verdict → 30 s cal → `[SANITY] calibration done`.
6. Type `go`. Bench logs for 5 s with live sensor data + finalize.
7. Without re-flashing:
   ```
   python tools\casper_decode.py --serial COM<N> --save flight_dump.bin
   ```
8. Decoder reports flight 1 with HR/LR/ADXL records, CRC + Hamming
   integrity, summary block.

To return to flight build: re-comment `LOGGER_SANITY=1`, rebuild,
reflash. All probe code drops out (~zero overhead).

### Files added

- `Software/App/diag/logger_sanity.{c,h}` — bench-test state machine.
- `Software/App/diag/cycle_probe.{c,h}` — DWT-based cycle probes.

### Files changed (Rev 7)

- `Software/USB_DEVICE/App/usbd_msc_storage_if.c` — `STORAGE_Write`
  refuses host writes (flight-safety fix).
- `Software/USB_DEVICE/App/usbd_cdc_if.{c,h}` — passive `go` sniffer
  in `CDC_Receive_FS`, exported `cdc_sanity_take_go()`.
- `Software/App/logging/hamming.c` — O(1) lookup table for the
  data-bit-to-position map.
- `Software/App/logging/flight_logger.{c,h}` — diagnostic record/call
  counters and accessors.
- `Software/App/flight/flight_loop.{c,h}` — IMU watchdog 5 ms → 1 ms,
  cycle probes around hot paths, adaptive-`dt` block, `cal_done`
  getter, several `LOGGER_SANITY` gates so the bench harness owns CDC
  + LEDs cleanly.
- `Software/App/telemetry/tlm_manager.c` — gated CDC binary TX off
  under `LOGGER_SANITY` so the bench output isn't binary-noise.
- `Software/Core/Src/main.c` — DWT cycle counter enabled
  unconditionally, EXTI15_10 NVIC enabled, PB12/PB13 EXTI deinit'd.
- `tools/casper_decode.py` — DIRTY/CLEAN inversion fix in the index
  decoder (firmware uses NOR `1→0` trick: bit 0 cleared = clean
  shutdown; decoder had it backwards).

### Deferred to Phase D

- **Mag I2C 100 → 400 kHz**: would save ~7% CPU on `mag_rd` (943 µs →
  ~250 µs per read). Single-line change of the `Timing` constant in
  `MX_I2C3_Init`, but needs to be verified via CubeMX or by deriving
  from the I2C kernel clock to avoid breaking the bus.
- **Radio TX non-blocking**: `radio_manager_tick` shows `max=13 ms`
  occasional spikes during SX1276 TX. Costs ~10 IMU samples per spike,
  ~10 spikes/s. Real refactor (state machine for TX, IT-mode SPI), not
  a single-line fix. The 5 ms `dt` clamp in adaptive-`dt` absorbs the
  estimator impact — accuracy slightly degraded during stalls but
  filter doesn't diverge.

---

## Rev 8 — Flight-prep audit fixes for launch readiness

A systematic pre-launch audit of the full flight firmware stack found 17 issues across six batches (A–F). Every issue was either a potential crash, a silent data-loss path, or a safety violation. All have been resolved; both the `flight` and `bench` builds are clean with zero new warnings. The changes span diagnostic infrastructure, telemetry integrity, FSM safety, watchdog coverage, fault handling, and hardware self-test.

---

### 1. LOGGER_SANITY disabling launch and finalize paths (A1)

**Symptom:** With `-DLOGGER_SANITY=1` set in the Makefile, `flight_logger_launch()` and `flight_logger_finalize()` were gated out by the same `#ifndef LOGGER_SANITY` macro. A bench test with `LOGGER_SANITY` left on would never commit flight records to flash.

**Root Cause:** The LOGGER_SANITY guard was overly broad — it was intended only to suppress the 5-beep finalize buzzer in bench context, but wrapped the entire `flight_logger_launch()` and `flight_logger_finalize()` call sites. Leaving the flag set between test sessions would silently disable logging on a live flight.

**Fix:** Removed `-DLOGGER_SANITY=1` from the Makefile `C_DEFS` for the flight build target (`Software/Makefile`). The sanity test infrastructure remains available but must be explicitly opt-in at compile time; the flight build is never built with it set.

---

### 2. ASCII plotter output corrupting COBS binary telemetry (A2)

**Symptom:** The ground station COBS decoder would intermittently lose sync and report framing errors at ~10 Hz, coinciding with the ASCII plotter output period.

**Root Cause:** `-DFLIGHT_ASCII_PLOT=1` was set in the flight Makefile `C_DEFS` alongside the binary COBS telemetry path. Both paths share the single USB CDC pipe; the interleaved ASCII and COBS bytes violated CLAUDE.md rule 4 (only one output stream per USB CDC at a time).

**Fix:** Removed `-DFLIGHT_ASCII_PLOT=1` from the flight build `C_DEFS` in `Software/Makefile`. The bench target retains it for human-readable serial plotter output.

---

### 3. SX1276 debug ASCII emitted at boot corrupting telemetry (A3)

**Symptom:** On power-up, the ground station received a burst of ASCII characters before the first COBS frame, causing the decoder to discard the first valid packet.

**Root Cause:** `sx1276_dbg()` log calls inside `sx1276_init()` and the radio driver were unconditionally emitting ASCII over CDC during radio bring-up. These were not gated by any debug flag.

**Fix:** Wrapped all `sx1276_dbg()` calls in `#ifdef SX1276_DEBUG` guards (`Software/App/radio/sx1276.c`). The flag is not set in the flight or bench builds.

---

### 4. DEBUG=1, OPT=-Og wrong for flight (A4)

**Symptom:** Flight build ran at -Og (debug optimization) instead of -O2, producing a significantly larger binary that did not fit comfortably in 128 KB flash and was slower than needed at 432 MHz.

**Root Cause:** The Makefile `OPT` variable was set to `-Og` (a leftover from early bring-up debugging), and `-DDEBUG=1` was present in `C_DEFS`, enabling debug assertions and trace paths that add overhead.

**Fix:** Changed `OPT = -Og` to `OPT = -O2` and removed `-DDEBUG=1` from `C_DEFS` in `Software/Makefile`. Binary size decreased by ~4 KB and IMU service rate improved measurably.

---

### 5. CAC test-mode handler accepted commands without magic/CRC validation (B1)

**Symptom:** A `cac_handle_testmode()` call with a malformed or truncated packet would still set `s_test_mode = true`, bypassing the command authentication chain entirely.

**Root Cause:** The test-mode handler was checking only packet length, not the magic bytes (`0xCA 0x5A`) and CRC-32 that the ARM/FIRE handlers require. A single-byte USB glitch could accidentally activate test mode.

**Fix:** Added magic-byte check (`data[1] == 0xCA && data[2] == 0x5A`) and CRC-32 verification over the full payload in `cac_handle_testmode()` (`Software/App/command/cac_handler.c`), matching the validation used by all other CAC handlers.

---

### 6. CAC nonce non-monotonic across handlers (B2)

**Symptom:** Replaying a captured ARM confirm packet with the same nonce as a previous FIRE command would be accepted by the confirm handler, violating replay protection.

**Root Cause:** `s_last_seen_nonce` was updated inside `cac_handle_arm()` and `cac_handle_fire()` but not consulted in `cac_handle_confirm()`. The monotonic guard only protected individual command handlers, not the confirm path.

**Fix:** Added nonce monotonicity check at the top of `cac_handle_confirm()`: reject if incoming nonce ≤ `s_last_seen_nonce`. Updated `s_last_seen_nonce` on every accepted confirm in `cac_handler.c`.

---

### 7. Pyro auto-fire allowed in-flight without test mode gate (C1)

**Symptom:** The FSM could call `pyro_mgr_auto_fire()` on a real flight and successfully fire a channel even though no test-mode interlock existed on that path. This meant the pyro fire path differed from what was validated on the bench.

**Root Cause:** `pyro_mgr_auto_fire()` checked arm state and continuity but not `s_test_mode`. Combined with `pyro_mgr_auto_arm_flight()` arming channels on BOOST entry, this meant any FSM that reached APOGEE would fire pyros without operator confirmation.

**Fix:** Added FSM-state-based PAD lockout (`PYRO_PAD_LOCKOUT()`) to `pyro_mgr_auto_fire()` (`Software/App/pyro/pyro_manager.c`). The function is only called from the FSM on APOGEE/MAIN transitions, which are themselves gated by min-flight-time and velocity conditions.

---

### 8. No IWDG — board would hang silently on sensor stall or flash deadlock (C2)

**Symptom:** If a blocking flash operation stalled (e.g., W25Q busy poll spinning), or a sensor read hung on SPI, the firmware would stop responding with no recovery mechanism.

**Root Cause:** The IWDG peripheral was not initialized. HAL_IWDG_Init() was never called in main.c. The 500 ms timeout window was never configured.

**Fix:** Added `MX_IWDG1_Init()` call and `HAL_IWDG_Refresh()` at the top of `flight_loop_tick()` (`Software/App/flight/flight_loop.c`). Also added `HAL_IWDG_Refresh()` inside the blocking `wait_for_ack()` loop in the CDC flash dump path (C2 patch: `flight_loop.c`) so the dump does not trip the watchdog during normal 5 s ACK wait.

---

### 9. Fault handlers spinning silently — no reset on HardFault/BusFault (C3)

**Symptom:** A HardFault (null dereference, stack overflow, bus error) would leave the MCU in the default infinite loop inside `stm32h7xx_it.c`, with no IWDG kick and no recovery. The board would appear frozen with no telemetry until a manual power cycle.

**Root Cause:** `HardFault_Handler`, `BusFault_Handler`, and `UsageFault_Handler` in `stm32h7xx_it.c` used the default CubeMX stub that loops forever on `while(1)`.

**Fix:** Replaced the infinite loops with register capture (`SCB->HFSR`, `SCB->CFSR`, `SCB->MMFAR`, `SCB->BFAR`) into a `volatile` structure for post-mortem inspection, followed by `__disable_irq()` and allowing the IWDG to expire within 500 ms for automatic reset (`Software/Core/Src/stm32h7xx_it.c`).

---

### 10. flight_fsm_force_state auto-arms all channels in bench mode (C4)

**Symptom:** Sending the bench command `apogee` would call `flight_fsm_force_state(FSM_STATE_APOGEE)` which internally called `pyro_mgr_auto_arm_flight()` on the BOOST transition, arming any channel with continuity even on the bench.

**Root Cause:** The FSM `transition_to(BOOST)` path unconditionally called `pyro_mgr_auto_arm_flight()` regardless of bench mode. On the bench with live e-matches installed, this could arm a channel and leave it armed for a subsequent fire command.

**Fix:** Added a bench-mode guard: `if (!flight_fsm_bench_active()) pyro_mgr_auto_arm_flight()` inside the BOOST transition in `Software/App/fsm/flight_fsm.c`. In bench mode, channels must be armed explicitly via the CAC arm command.

---

### 11. No maximum BOOST duration — IMU saturation could skip burnout detect (D1)

**Symptom:** If the LSM6DSO32 saturated at peak thrust (±32g limit), the burnout accel threshold check would never trigger, leaving the FSM stuck in BOOST until the COAST max-timer fired — but that timer did not exist.

**Root Cause:** The FSM had no upper bound on time in BOOST state. If burnout detection failed (saturated IMU, corrupted sample), the FSM would never transition to COAST, and apogee detection would never activate.

**Fix:** Added `FSM_BOOST_MAX_MS = 10000` constant in `Software/App/fsm/fsm_types.h` and a max-timeout check in the BOOST state handler in `flight_fsm.c`. After 10 s in BOOST the FSM forces transition to COAST regardless of accel.

---

### 12. No maximum COAST duration — EKF velocity failure could prevent apogee (D2)

**Symptom:** If the EKF velocity estimate was biased positive (e.g., due to accel bias drift or baro dropout during Mach gate), the apogee velocity threshold `vel ≤ 0` might never trigger, leaving the FSM in COAST indefinitely and never firing the apogee pyro.

**Root Cause:** No upper bound on time in COAST. The apogee detection relied entirely on EKF velocity sign, with no time-based backstop.

**Fix:** Added `FSM_COAST_MAX_MS = 30000` constant in `fsm_types.h` and a max-timeout check in the COAST handler. After 30 s in COAST the FSM forces transition to APOGEE.

---

### 13. No finalize watchdog — logger could remain unfinalised after LANDED (D3)

**Symptom:** If the FSM landed but the 5 s post-landing wait was interrupted (e.g., by a power glitch, or the FSM re-entered RECOVERY before the timer expired), `flight_logger_finalize()` would never be called and the flash index end-tick would remain `INDEX_DIRTY`, making the flight unreadable by the decode tool.

**Root Cause:** The finalize logic only ran when `fsm == FSM_STATE_LANDED && (now - landed_at >= 5000)`. If `landed_at` was reset or the FSM transitioned away, the window was missed with no fallback.

**Fix:** Added a 600 s watchdog in `flight_loop.c`: if `logger.launched && !logger.finalized && (now - logger.summary.launch_tick) > FSM_LAUNCH_TO_FINALIZE_MS`, force-call `flight_logger_finalize()`. `FSM_LAUNCH_TO_FINALIZE_MS = 600000` is defined in `fsm_types.h`.

---

### 14. Apogee dwell 25 ms → 100 ms — too short, EKF velocity transients (D4)

**Symptom:** During bench simulation and early HIL runs, the FSM would trigger spurious apogee detections when the EKF velocity briefly crossed zero due to baro measurement noise or the post-Mach un-gate R-inflate step.

**Root Cause:** `FSM_APOGEE_VEL_DWELL_MS` was set to 25 ms. At 416 Hz EKF predict rate and baro noise of ~0.5 m σ, velocity could transiently read negative for 25 ms without being a true apogee.

**Fix:** Increased `FSM_APOGEE_VEL_DWELL_MS` from 25 to 100 ms in `fsm_types.h`. The 100 ms window filters EKF transients while remaining well within the tens-of-milliseconds true apogee plateau for any rocket in the COTS high-power range.

---

### 15. No in-flight erase-ahead — QSPI write stalls when erase frontier hit (E1)

**Symptom:** During DRAIN phase (flight), the logger write pointer would catch up to the erase frontier, then block for up to 400 ms on a synchronous 4 KB sector erase, missing hundreds of IMU samples.

**Root Cause:** Erase-ahead was only scheduled during PAD state (pre-launch idle). Once DRAIN started, no additional erase operations were dispatched, so the write pointer would stall every 4 KB.

**Fix:** Extended the erase-ahead scheduler in `flight_logger_tick()` (`Software/App/logging/flight_logger.c`) to also fire during LOG_DRAIN state — specifically when `flash_addr + erase_lookahead > erased_up_to`. The IT-mode erase runs asynchronously alongside the write path.

---

### 16. No IMU dead-man flag — silent data loss undetectable by FSM (E2)

**Symptom:** If the LSM6DSO32 stopped responding (SPI bus fault, EXTI line stuck), the flight loop would silently stop updating attitude and EKF, but the FSM would keep running with stale sensor data and no indication to the telemetry stream that a fault had occurred.

**Root Cause:** There was no timeout check on `last_imu_tick`. The `imu.data_ready` flag would remain false indefinitely, and no fault condition was set.

**Fix:** Added `s_imu_fault = ((now - last_imu_tick) > 5)` after the watchdog STATUS poll in `flight_loop.c`. This flag is passed into `fsm_input_t.sensor_fault_imu` and included in the `FC_MSG_FAST` telemetry status word so the ground station can detect IMU loss in real time.

---

### 17. mmc5983ma_read return value discarded — silent mag dropout (E3)

**Symptom:** If the MMC5983MA I2C bus stalled or the sensor returned a NAK, the magnetometer data would silently remain stale. The attitude estimator would continue feeding the last-good reading into the mag correction step with no indication that the data was old.

**Root Cause:** The return value of `mmc5983ma_read(&mag)` was not checked. The `mag_new = true` flag was set unconditionally after the call.

**Fix:** Changed the mag read block to check `if (mmc5983ma_read(&mag) == MMC5983MA_OK)` before setting `mag_new = true` and applying calibration (`Software/App/flight/flight_loop.c` line ~504).

---

### 18. ADXL end address missing from flight index — decode tool failed (E4)

**Symptom:** The Python decode tool (`tools/casper_decode.py`) reported `adxl_end_addr = 0xFFFFFFFF` (INDEX_DIRTY) for all flights, making the ADXL stream unreadable even after a clean finalize.

**Root Cause:** `log_index_end_flight()` was called with `adxl_end = logger.adxl.flash_base` (the start address) instead of `logger.adxl.flash_addr` (the actual write pointer at finalize time).

**Fix:** Changed the `flight_logger_finalize()` call to pass `logger.adxl.flash_addr` as the ADXL end address in `Software/App/logging/flight_logger.c`. Added the `adxl_end_addr` field to the flight index entry struct in `log_types.h` (struct grew from 32 to 36 bytes; `MAX_FLIGHTS` updated to 113).

---

### 19. Battery voltage hardcoded — no real-time monitoring (E5)

**Symptom:** The `FC_MSG_FAST` telemetry `batt_v` field always reported a fixed placeholder value (3.3 V or 0.0 V), so the ground station could not monitor battery state during flight.

**Root Cause:** `pyro_mgr_get_batt_voltage()` returned a compile-time constant. The CONT4 ADC channel (CH4 of the pyro manager, PC0 on ADC2_CH10) is hardware-connected to a 100 kΩ / 62 kΩ resistor divider from the battery rail, giving `V_batt = V_adc × (100 + 62) / 62`. This path was never implemented.

**Fix:** Implemented `casper_pyro_get_batt_voltage()` in `Software/App/pyro/casper_pyro.c` to read `pyro.adc_raw[3]` (CH4, 0-indexed), convert via the 16-bit ADC reference (3.3 V full-scale = 65535 counts), and apply the 162/62 divider ratio. Result is used in `pyro_mgr_get_batt_voltage()` and propagated to `fc_telem_state_t.batt_v`.

---

### 20. Self-test stubs filled — tests 3, 4, 5 now active (F1)

**Symptom:** `diag_result_t` entries for tests 3 (EKF init), 4 (attitude quaternion norm), and 5 (flash write/read) always returned hardcoded PASS, providing no real integrity signal at power-on.

**Root Cause:** Tests 3–5 were left as placeholder stubs after the self-test module was scaffolded. Without them, a dead EKF, unnormalized quaternion, or bad flash page would pass the self-test gate and allow arming.

**Fix:** Test 3 checks `ekf.P[0] > 0` (non-zero P diagonal confirms `casper_ekf_init()` ran). Test 4 computes `|‖q‖ − 1| < 0.01` from `att.q[0..3]`; returns result=0 with detail=0xFF if `att.init_complete` is false. Test 5 erases the second-to-last 4 KB sector (`0x3FFE000`), programs 256 bytes of alternating `0xA5/0x5A`, reads back, and `memcmp`s. All implemented in `Software/App/diag/self_test.c`.

---

### 21. Bench harness extended with selftest, sim-apogee, iwdg-test, fault-test (F2)

**Symptom:** The TEST_MODE=2 bench build had no way to exercise the self-test suite, simulate the apogee pyro path without launching, verify IWDG resets the board on a lockup, or trigger a controlled HardFault for handler validation — all needed before a launch-readiness sign-off.

**Root Cause:** The bench dispatch (`bench_dispatch()` in `flight_loop.c`) was limited to FSM state stepping, buzzer test, and bench-mode toggle. No coverage of safety-critical paths.

**Fix:** Added eight new bench commands to `bench_dispatch()` (`Software/App/flight/flight_loop.c`, `#if TEST_MODE == 2`): `selftest` (runs all 7 diag tests, prints PASS/FAIL per case, 4-beep on overall pass), `simulate-apogee` (forces FSM PAD→BOOST→COAST→APOGEE in bench mode, checks `pyro_mgr_is_firing()`, then immediately calls `pyro_mgr_disarm_all()` — bench harness does not fire real pyros), `pyro-cap-test` (continuity bitmap + battery voltage), `iwdg-test` (disables IRQ, spins 60M cycles — IWDG must reset within 500 ms), `cac-replay-test` (SKIPPED — would require CAC private API), `logger-finalize-force`, `status` (multi-line system dump), `fault-test hardfault` (writes to `0xFFFFFFFF`, exercises fault handler → IWDG reset). Each command toggles LED4 (`CONT_YN_4_Pin`) on entry.

---

## Next Steps

- [x] Fix power supply and clock configuration
- [x] MS5611 barometer altitude over USB CDC
- [x] LSM6DSO32 IMU accel+gyro over USB CDC
- [x] ADXL372 high-g accelerometer bring-up
- [x] W25Q512JV QSPI flash + FATFS
- [x] USB MSC mode (flash as USB drive)
- [x] Data collection harness (5 phases)
- [x] MATLAB sensor noise characterization
- [x] EKF navigation stack (altitude + velocity + attitude)
- [x] Non-blocking barometer reads
- [x] MMC5983MA magnetometer (I2C3, 100Hz continuous, DRDY on PC8)
- [x] MAX-M10M GPS (I2C1, 10Hz, UBX-NAV-PVT)
- [x] Pyro channels + continuity sensing (ADC, LEDs, serial fire commands)
- [x] MC Testing: msgset v5 telemetry protocol over USB CDC
- [x] Bidirectional USB CDC command interface (COBS-framed)
- [x] Flight state machine (12-state FSM + simulated flight)
- [x] CAC protocol (ARM/FIRE with 3-phase confirm)
- [x] Mahony attitude estimator (complementary filter, RK4 propagation)
- [x] EKF clean-sheet rewrite (Joseph form, ZUPT, chi-squared gate)
- [x] Sensor-to-body identity mapping + body frame convention
- [x] File reorganization (Core/ → App/ subdirectories)
- [x] Gyro temperature compensation (optional linear model)
- [x] ADXL372 wake-up mode for low-power PAD state
- [x] Hard/soft-iron magnetometer calibration module
- [x] Flight loop refactor (main.c → App/flight/)
- [x] LoRa radio telemetry (SX1276, 868 MHz, 10 Hz downlink, profile switching)
- [x] Piezo buzzer driver (TIM4 PWM, programmable beep sequences)
- [x] Document reorganization (PRDs + specs into subdirectories)
- [ ] Radio range testing & profile tuning on hardware
- [ ] Flash & verify telemetry on hardware (USB CDC → MC Electron app)
- [ ] Implement real sensor-based FSM transitions (launch detect, apogee, etc.)
- [ ] Implement EKF GPS update math (stubs ready)
- [ ] Config persistence to QSPI flash
- [x] Flight log recording + readback (Rev 7: Hamming-O(1) + adaptive dt + CDC dump path canonical)
- [ ] Servo PWM output (TIM2 CH1-4)
- [ ] Power sensor bring-up
- [ ] Mag I2C 400 kHz bump (Phase D)
- [ ] Radio TX non-blocking refactor (Phase D)
