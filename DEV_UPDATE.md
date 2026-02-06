# Dev Update — First Boot & Bring-Up

**Date:** February 2026
**Board:** C.A.S.P.E.R.-2 Rev 1 (STM32H750VBT6, LQFP100)
**Firmware:** CubeMX HAL + USB CDC on HSI 64 MHz

---

## Summary

Brought the C.A.S.P.E.R.-2 flight computer from a completely unresponsive state to running firmware with all 4 continuity LEDs operational and USB CDC transmitting over serial. No ST-Link debugger is available on this board — all flashing is done via USB DFU (BOOT0 pin), which made debugging significantly harder.

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

**Workaround:** `SystemClock_Config()` is disabled. The MCU runs on the default **HSI 64 MHz** internal oscillator. This is sufficient for bring-up and testing. All other peripheral inits (ADC, I2C, SPI, UART, etc.) are also disabled for now — only GPIO and USB are active.

**TODO:** Investigate the HSE crystal circuit. A more conservative PLL configuration (e.g. 768 MHz VCO → 384 MHz SYSCLK) should also be tested.

---

### 3. USB CDC Clock Source — PLL to HSI48

**Symptom:** With `SystemClock_Config()` disabled, PLL1 is not running. The CubeMX-generated USB configuration in `usbd_conf.c` uses `RCC_USBCLKSOURCE_PLL` (PLL1Q = 48 MHz) as the USB clock.

**Fix:** Changed USB clock source to `RCC_USBCLKSOURCE_HSI48` (internal 48 MHz oscillator). HSI48 is enabled via direct register write before USB init:

```c
RCC->CR |= RCC_CR_HSI48ON;
while (!(RCC->CR & RCC_CR_HSI48RDY));
```

---

## Current Firmware Behaviour

1. **Milestone LEDs** light up sequentially during initialization (500ms between each):
   - LED1 (PA10) → `HAL_Init()` passed
   - LED2 (PB14) → `MX_GPIO_Init()` passed
   - LED3 (PE8) → `MX_USB_DEVICE_Init()` passed
   - LED4 (PE7) → All initialization complete

2. **USB CDC** transmits `"hello world this is C.A.S.P.E.R 2"` every 5 seconds

3. **LED chase pattern** cycles through all 4 LEDs sequentially at 500ms per LED

---

## Files Changed (from CubeMX baseline)

| File | Change |
|------|--------|
| `Software/Core/Src/main.c` | LDO power supply fix, disabled `SystemClock_Config()`, HSI48 enable, milestone LEDs, USB CDC hello, sequential LED chase |
| `Software/USB_DEVICE/Target/usbd_conf.c` | USB clock source: `RCC_USBCLKSOURCE_PLL` → `RCC_USBCLKSOURCE_HSI48` |
| `Software/Makefile` | `USE_PWR_EXTERNAL_SOURCE_SUPPLY` → `USE_PWR_LDO_SUPPLY` |

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

## Next Steps

- [ ] Investigate HSE crystal circuit and fix PLL configuration
- [ ] Enable remaining peripheral inits (sensors, radio, flash)
- [ ] Implement bidirectional USB CDC command interface
- [ ] Sensor bring-up (IMU, barometer, magnetometer, GPS)
- [ ] Flight state machine and data logging
