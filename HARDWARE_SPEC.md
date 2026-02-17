# HARDWARE_SPEC.md — STM32 Peripheral & CubeMX Configuration

**Ground-truth reference for the C.A.S.P.E.R.-2 hardware configuration.**
Covers the MCU, clock tree, power supply, all peripherals, GPIO mapping, memory layout, and build system.

---

## 1. MCU Overview

| Parameter | Value |
|-----------|-------|
| Part | STM32H750VBT6 |
| Core | ARM Cortex-M7, FPU + DSP |
| Package | LQFP100 |
| Internal Flash | 128 KB |
| DTCMRAM | 128 KB |
| AXI SRAM | 512 KB |
| D2 SRAM | 288 KB |
| D3 SRAM | 64 KB |
| ITCMRAM | 64 KB |
| Max clock | 480 MHz (running at 432 MHz) |

---

## 2. Power Supply — CRITICAL

| Parameter | Value |
|-----------|-------|
| Supply mode | **Internal LDO** (`PWR_LDO_SUPPLY`) |
| Voltage scaling | Scale 0 (highest performance) |

**The #1 brick-your-board bug:** CubeMX generates `-DUSE_PWR_EXTERNAL_SOURCE_SUPPLY` in the Makefile, which is **WRONG** for this board. The correct define is `-DUSE_PWR_LDO_SUPPLY`.

This must be correct in **TWO places:**

1. **Makefile `C_DEFS`:** `-DUSE_PWR_LDO_SUPPLY`
2. **`SystemClock_Config()`:** `HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);`

If either is wrong, `ExitRun0Mode()` in the startup assembly hangs **before `main()` ever executes** — the board appears completely dead with no LED output. This was the first critical issue found during bring-up.

---

## 3. Clock Tree

### 3.1 Oscillator Source

| Parameter | Value | Note |
|-----------|-------|------|
| HSE (24 MHz crystal) | **DEAD** | Confirmed dead on hardware |
| HSI (64 MHz RC) | Active | Used as PLL source |

CubeMX always regenerates with HSE. After every CubeMX regen, manually change to HSI.

### 3.2 PLL1 — System Clock

```
HSI (64 MHz) → /PLLM=4 → 16 MHz VCO input
              → *PLLN=54 → 864 MHz VCO output
              → /PLLP=2 → 432 MHz SYSCLK
              → /PLLQ=18 → 48 MHz USB clock
              → /PLLR=4 → 216 MHz (unused)
```

| Output | Frequency | Usage |
|--------|-----------|-------|
| PLL1P (SYSCLK) | 432 MHz | CPU, AHB (216 MHz after /2) |
| PLL1Q | 48 MHz | USB OTG FS |
| PLL1R | 216 MHz | Not used |

### 3.3 PLL2 — Peripheral Clock

```
HSI (64 MHz) → /PLL2M=8 → 8 MHz VCO input
              → *PLL2N=21 → 168 MHz VCO output
              → /PLL2P=2 → 84 MHz
              → /PLL2Q=2 → 84 MHz
              → /PLL2R=2 → 84 MHz
```

| Output | Frequency | Usage |
|--------|-----------|-------|
| PLL2P | 84 MHz | SPI1/2/3 clock source |
| PLL2Q | 84 MHz | ADC clock source (after /2 = 42 MHz) |
| PLL2R | 84 MHz | UART4 clock source |

### 3.4 Bus Clocks

| Bus | Divider | Frequency |
|-----|---------|-----------|
| SYSCLK | /1 | 432 MHz |
| AHB (HCLK) | /2 | 216 MHz |
| APB1 | /2 | 108 MHz |
| APB2 | /2 | 108 MHz |
| APB3 | /2 | 108 MHz |
| APB4 | /2 | 108 MHz |
| Flash latency | 4 WS | |

### 3.5 Peripheral Clock Routing

```c
PeriphClkInitStruct.Spi123ClockSelection   = RCC_SPI123CLKSOURCE_PLL2;   /* 84 MHz */
PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
PeriphClkInitStruct.AdcClockSelection      = RCC_ADCCLKSOURCE_PLL2;      /* 84 MHz → /2 = 42 MHz */
```

USB clock is PLL1Q (48 MHz) — configured automatically by the USB middleware.

---

## 4. SPI Configuration

### 4.1 SPI Bus Summary

| SPI | Sensor | Prescaler | Bus Freq | SPI Mode | CS Pin |
|-----|--------|-----------|----------|----------|--------|
| SPI1 | TBD | TBD | TBD | Mode 0 | PB0 |
| SPI2 | LSM6DSO32 | 16 | 5.25 MHz | **Mode 3** | PC14 |
| SPI3 | ADXL372 | TBD | TBD | Mode 0 | PA15 |
| SPI4 | MS5611 | 8 | 13.5 MHz | Mode 0 | PE11 |

SPI clock source: PLL2P = 84 MHz (for SPI1/2/3) or APB2 = 108 MHz (for SPI4).

### 4.2 CubeMX SPI Pitfalls

CubeMX generates incorrect defaults for all SPI buses:

| Setting | CubeMX Default | Correct Value |
|---------|----------------|---------------|
| DataSize | **4-bit** | 8-bit (`SPI_DATASIZE_8BIT`) |
| BaudRatePrescaler | **2** (way too fast) | Per-sensor (8, 16, etc.) |
| CPOL/CPHA | Mode 0 | SPI2 must be Mode 3 |

These MUST be manually verified after every CubeMX regen.

### 4.3 SPI2 Mode 3 Override

The LSM6DSO32 requires SPI Mode 3 (CPOL=1, CPHA=1). CubeMX generates Mode 0. The fix is in USER CODE SPI2_Init 2:

```c
hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
HAL_SPI_Init(&hspi2);
```

### 4.4 SPI Pin Mapping

| SPI | SCK | MOSI | MISO | CS (GPIO) | INT (EXTI) |
|-----|-----|------|------|-----------|------------|
| SPI1 | PA5 | PA7 | PB4 | PB0 | PB1 |
| SPI2 | PD3 | PC1 | PC2_C | PC14 | PC15 |
| SPI3 | PC10 | PC12 | PC11 | PA15 | PD2 |
| SPI4 | PE12 | PE14 | PE13 | PE11 | — |

**PC2_C note:** PC2 is a dual-pad pin (PC2 and PC2_C). CubeMX sometimes clobbers PC2 to analog mode during GPIO init, which kills SPI2 MISO. The firmware forces PC2 back to AF5 in USER CODE BEGIN 2.

---

## 5. I2C Configuration

| I2C | Sensor | SCL | SDA | Speed | INT Pin |
|-----|--------|-----|-----|-------|---------|
| I2C1 | MAX-M10M GPS | PB8 | PB9 | 400 kHz | PE0 (not enabled) |
| I2C2 | TBD | PB10 | PB11 | 400 kHz | — |
| I2C3 | MMC5983MA Mag | PA8 | PC9 | 400 kHz | PC8 (EXTI) |

### 5.1 PC8 EXTI Override

PC8 (MMC5983MA DRDY) is configured as GPIO output by CubeMX. The firmware overrides it to EXTI rising-edge input in USER CODE BEGIN 2:

```c
GPIO_InitTypeDef gi = {0};
gi.Pin  = GPIO_PIN_8;
gi.Mode = GPIO_MODE_IT_RISING;
gi.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOC, &gi);
HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
```

---

## 6. QUADSPI Configuration

| Parameter | Value |
|-----------|-------|
| Peripheral | QUADSPI |
| Flash | W25Q512JV (64 MB) |
| FSIZE | 25 (2^26 = 64 MB) |
| CLK | PB2 |
| NCS | PB6 |
| IO0 | PD11 |
| IO1 | PD12 |
| IO2 | PD13 |
| IO3 | PE2 |

The QUADSPI operates in indirect mode (not memory-mapped) for maximum flexibility with FATFS and USB MSC.

---

## 7. USB OTG FS

| Parameter | Value |
|-----------|-------|
| Peripheral | USB_OTG_FS |
| DM | PA11 |
| DP | PA12 |
| Speed | Full Speed (12 Mbps) |
| Clock | PLL1Q = 48 MHz |
| Mode | Compile-time via USB_MODE |

### 7.1 USB_MODE System

| Mode | Define | Class | Description |
|------|--------|-------|-------------|
| 1 | `USB_MODE=1` | CDC | Telemetry serial output, command receive |
| 2 | `USB_MODE=2` | MSC | QSPI flash as USB drive |
| 3 | `USB_MODE=3` | CDC | Data collection logging |

### 7.2 CDC Ring Buffer

The USB CDC receive path uses a ring buffer in `usbd_cdc_if.c`:

```c
#define CDC_RING_SIZE 256
static volatile uint8_t  cdc_ring_buf[CDC_RING_SIZE];
static volatile uint16_t cdc_ring_head;
static volatile uint16_t cdc_ring_tail;
```

**API:**
```c
uint16_t cdc_ring_available(void);   /* Bytes waiting */
uint8_t  cdc_ring_read_byte(void);   /* Pop one byte */
```

Data flows: USB ISR → `CDC_Receive_FS()` → ring buffer → `cmd_router_process()` (main loop) → COBS decode → dispatch.

---

## 8. ADC Configuration

Three ADC peripherals are used for pyro continuity sensing:

| ADC | Resolution | Clock | Channels Used |
|-----|------------|-------|---------------|
| ADC1 | 16-bit | PLL2/2 = 42 MHz → /2 async = 21 MHz | CH3 (PA6 = CONT2), CH4 (PC4 = CONT1) |
| ADC2 | 16-bit | Same | CH10 (PC0 = CONT4) |
| ADC3 | 16-bit | Same | CH1 (PC3_C = CONT3) |

All ADCs operate in single-shot software-triggered mode. Channels are reconfigured at runtime by `casper_pyro_tick()` before each conversion.

### 8.1 Continuity Sensing

| Channel | ADC | ADC Channel | Pin |
|---------|-----|-------------|-----|
| CONT1 | ADC1 | CH4 | PC4 |
| CONT2 | ADC1 | CH3 | PA6 |
| CONT3 | ADC3 | CH1 | PC3_C |
| CONT4 | ADC2 | CH10 | PC0 |

Threshold: `PYRO_CONTINUITY_THRESHOLD = 8000` (16-bit ADC raw value).

---

## 9. Timers

### 9.1 TIM2 — Servo PWM

| Parameter | Value |
|-----------|-------|
| Channels | CH1-CH4 |
| Pins | PA0, PA1, PA2, PA3 |
| Usage | Servo PWM for TVC (future) |

### 9.2 TIM4 — General Purpose

Used for internal timing. Not connected to external pins.

---

## 10. CRC Hardware Unit

| Parameter | Value |
|-----------|-------|
| Peripheral | CRC |
| Polynomial | CRC-32 ISO-HDLC (0x04C11DB7) |
| Init value | 0xFFFFFFFF |
| Input/Output reflection | Yes |
| Final XOR | 0xFFFFFFFF |

The STM32H7's hardware CRC unit is used for all telemetry packet CRC-32 computation via `crc32_hw_compute()`.

---

## 11. GPIO Pin Map

### 11.1 LEDs (Continuity Indicators / Debug)

| LED | Pin | Port | Usage |
|-----|-----|------|-------|
| LED1 / CONT_YN_1 | PA10 | GPIOA | Pyro 1 continuity / heartbeat |
| LED2 / CONT_YN_2 | PB14 | GPIOB | Pyro 2 continuity / debug |
| LED3 / CONT_YN_3 | PE8 | GPIOE | Pyro 3 continuity |
| LED4 / CONT_YN_4 | PE7 | GPIOE | Pyro 4 continuity |

During init, LEDs are used as milestone indicators (raw register writes before HAL init). In normal operation, they indicate pyro continuity via `casper_pyro_tick()`.

### 11.2 Pyro Fire Outputs

| Channel | Pin | Port | Type |
|---------|-----|------|------|
| PY1 | PD10 | GPIOD | Push-pull, active high (MOSFET gate) |
| PY2 | PD9 | GPIOD | Push-pull, active high |
| PY3 | PD8 | GPIOD | Push-pull, active high |
| PY4 | PB15 | GPIOB | Push-pull, active high |

### 11.3 GPS Control

| Signal | Pin | Port | Description |
|--------|-----|------|-------------|
| NRST_GPS | PE15 | GPIOE | Active-low reset |
| I2C1_INT | PE0 | GPIOE | Data-ready (not enabled yet) |

### 11.4 Misc

| Signal | Pin | Description |
|--------|-----|-------------|
| USB_DM | PA11 | USB OTG FS |
| USB_DP | PA12 | USB OTG FS |
| BOOT0 | (built-in) | Hold high at reset for DFU mode |

---

## 12. Memory Layout (Linker Script)

```
MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K   ← stack + data
  RAM     (xrw) : ORIGIN = 0x24000000, LENGTH = 512K   ← AXI SRAM
  RAM_D2  (xrw) : ORIGIN = 0x30000000, LENGTH = 288K   ← D2 SRAM
  RAM_D3  (xrw) : ORIGIN = 0x38000000, LENGTH = 64K    ← D3 SRAM
  ITCMRAM (xrw) : ORIGIN = 0x00000000, LENGTH = 64K    ← instruction TCM
}
```

- Stack grows down from end of DTCMRAM
- Minimum stack: 0x400 (1 KB)
- Minimum heap: 0x200 (512 bytes)
- Current build size: ~85 KB text + data (fits in 128 KB flash)

---

## 13. Startup Sequence

```
1. Reset vector → startup_stm32h750xx.s
2. ExitRun0Mode() — power supply init (system_stm32h7xx.c)
   ↑ HANGS HERE if USE_PWR_EXTERNAL_SOURCE_SUPPLY is wrong
3. SystemInit() — FPU enable, VTOR set
4. __libc_init_array() — C runtime init
5. main()
   a. MPU_Config()
   b. HAL_Init()
   c. LED milestone 1 (raw register writes)
   d. SystemClock_Config() — HSI + PLL1 = 432 MHz
   e. PeriphCommonClock_Config() — PLL2 = 84 MHz
   f. MX_GPIO_Init() — all GPIO pins
   g. MX_SPI*_Init() — all SPI buses
   h. MX_I2C*_Init() — all I2C buses
   i. MX_QUADSPI_Init()
   j. MX_ADC*_Init()
   k. MX_TIM*_Init()
   l. MX_CRC_Init()
   m. MX_FATFS_Init()
   n. MX_USB_DEVICE_Init()
   o. USER CODE BEGIN 2:
      - PC2 MODER fix
      - PC8 EXTI override (mag DRDY)
      - Sensor init (ms5611, lsm6dso32, adxl372, w25q512jv, gps, mag)
      - EKF init
      - Attitude init
      - Pyro init
      - Telemetry/FSM/command init
   p. Main loop (superloop, no RTOS)
```

---

## 14. Build System

### 14.1 Toolchain

| Tool | Version |
|------|---------|
| Compiler | arm-none-eabi-gcc 13.3.rel1 |
| Make | GNU Make (from STM32CubeIDE plugins) |
| IDE | STM32CubeIDE 2.0.0 |

### 14.2 Key Makefile Defines

```makefile
C_DEFS = \
-DUSE_PWR_LDO_SUPPLY \        # CRITICAL — not EXTERNAL_SOURCE_SUPPLY
-DUSE_HAL_DRIVER \
-DSTM32H750xx \
-DARM_MATH_CM7 \               # CMSIS-DSP optimized for Cortex-M7
-DUSB_MODE=1 \                 # 1=CDC, 2=MSC, 3=CDC+data collection
-DMSC_MEDIA_PACKET=4096        # USB MSC transfer block size
```

### 14.3 Linker Flags

```makefile
LDFLAGS = -specs=nano.specs     # newlib-nano for small footprint
          -u _printf_float      # enable float formatting in snprintf
          -T STM32H750XX_FLASH.ld
```

### 14.4 Build Commands

```bash
cd Software/
make clean && make -j8
# Output: build/Casper2.elf, build/Casper2.hex, build/Casper2.bin
```

### 14.5 Flash via DFU

1. Hold BOOT0, press RESET, release BOOT0
2. Flash `build/Casper2.bin` to `0x08000000` via STM32CubeProgrammer or dfu-util
3. Press RESET to run

---

## 15. CubeMX Regeneration Checklist

After any CubeMX regeneration, verify ALL of the following:

- [ ] Makefile `C_DEFS` contains `-DUSE_PWR_LDO_SUPPLY` (not `EXTERNAL_SOURCE_SUPPLY`)
- [ ] `SystemClock_Config()` uses `PWR_LDO_SUPPLY` and HSI+PLL (not HSE)
- [ ] `PeriphCommonClock_Config()` PLL2 values for HSI (PLL2M=8, PLL2N=21) + UART4 clock selection
- [ ] All SPI `DataSize` = `SPI_DATASIZE_8BIT` (CubeMX defaults to 4-bit!)
- [ ] SPI `BaudRatePrescaler` per sensor: SPI2=16, SPI4=8
- [ ] SPI2 Mode 3 override in USER CODE SPI2_Init 2 (CPOL=1, CPHA=1)
- [ ] SPI2_CS (PC14) initial GPIO level = HIGH (CS idle)
- [ ] PC2 MODER fix in USER CODE BEGIN 2
- [ ] PC8 EXTI override in USER CODE BEGIN 2
- [ ] EXTI callback pin name matches CubeMX label (`SPI2_INT_Pin`)
- [ ] USB_MODE, ARM_MATH_CM7 defines preserved in Makefile C_DEFS
- [ ] CMSIS-DSP sources and include paths preserved in Makefile
- [ ] All App/ source files preserved in Makefile C_SOURCES
