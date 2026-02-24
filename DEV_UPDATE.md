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
- [ ] Flight log recording + readback
- [ ] Servo PWM output (TIM2 CH1-4)
- [ ] Power sensor bring-up
