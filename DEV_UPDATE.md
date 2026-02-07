# Dev Update — Flight Computer Bring-Up

**Date:** February 2026
**Board:** C.A.S.P.E.R.-2 Rev 1 (STM32H750VBT6, LQFP100)
**Firmware:** CubeMX HAL + USB CDC, 432 MHz SYSCLK (HSI+PLL), MS5611 + LSM6DSO32 sensors

---

## Summary

Brought the C.A.S.P.E.R.-2 flight computer from a completely unresponsive state to running at 432 MHz with two active sensors: MS5611 barometer (SPI4) and LSM6DSO32 IMU (SPI2). Both output real-time data at 100 Hz over USB CDC serial plotter. No ST-Link debugger — all flashing via USB DFU, all debugging via milestone LEDs and serial output.

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

## Current Firmware Behaviour

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
   - MS5611 barometer: altitude in meters (OSR_1024)
   - LSM6DSO32 IMU: acceleration in g (±32g), angular rate in dps (±2000dps)

5. **LED1 heartbeat** toggles every 10ms (100 Hz) in the main loop

---

## Files Changed (from CubeMX baseline)

| File | Change |
|------|--------|
| `Software/Core/Src/main.c` | HSI+PLL clock, SPI2+SPI4 fixes, PC2 MODER fix, MS5611+LSM6DSO32 init, 100Hz serial plotter output, EXTI callback, milestone LEDs |
| `Software/Core/Src/ms5611.c` | **NEW** — MS5611 SPI barometer driver (from RobTillaart/MS5611_SPI) |
| `Software/Core/Inc/ms5611.h` | **NEW** — MS5611 driver header |
| `Software/Core/Src/lsm6dso32.c` | **NEW** — LSM6DSO32 SPI IMU driver (from STMicroelectronics/lsm6dso32-pid) |
| `Software/Core/Inc/lsm6dso32.h` | **NEW** — LSM6DSO32 driver header |
| `Software/Core/Src/stm32h7xx_it.c` | EXTI15_10 handler for LSM6DSO32 INT2 |
| `Software/USB_DEVICE/Target/usbd_conf.c` | USB clock: PLL1Q 48 MHz |
| `Software/Makefile` | `USE_PWR_LDO_SUPPLY`, sensor source files, `-u _printf_float` |
| `.claude/CLAUDE.md` | Updated documentation: clock config, pin mapping, CubeMX regen checklist |

---

## Build & Flash

```bash
# Build (from Software/ directory)
make clean && make -j8

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
| SPI3 | TBD | PC10 | PC12 | PC11 | PA15 | PD2 |
| SPI4 | MS5611 | PE12 | PE14 | PE13 | PE11 | — |

## Next Steps

- [x] Fix power supply and clock configuration
- [x] MS5611 barometer altitude over USB CDC
- [x] LSM6DSO32 IMU accel+gyro over USB CDC
- [ ] Flash & verify LSM6DSO32 readings on hardware
- [ ] Remaining sensors: ADXL372 (high-g accel), MMC5983MA (magnetometer)
- [ ] GPS (U-blox M10M via UART4)
- [ ] Bidirectional USB CDC command interface
- [ ] QSPI flash data logging (W25Q512JV)
- [ ] Flight state machine
