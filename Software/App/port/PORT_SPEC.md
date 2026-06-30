# CASPER Port Layer — Specification

`Software/App/port/`  
Pure C11. No HAL or CMSIS types in any header. Board-specific implementations live in `board_casper2/board_casper2.c` (or a future `board_casper3/board_casper3.c`).

---

## 1. casper_types.h

### casper_status_t

```c
typedef enum {
    CASPER_OK      =  0,
    CASPER_ERR     = -1,
    CASPER_TIMEOUT = -2
} casper_status_t;
```

Every port function that can fail returns one of these. `CASPER_OK` means the operation completed without error. `CASPER_ERR` means the peripheral reported a fault (HAL_ERROR on STM32). `CASPER_TIMEOUT` means the operation did not complete before the `to_ms` deadline expired.

### Opaque handle types

```c
typedef struct casper_spi_s  casper_spi_t;
typedef struct casper_i2c_s  casper_i2c_t;
typedef struct casper_qspi_s casper_qspi_t;
typedef struct casper_adc_s  casper_adc_t;
typedef struct casper_pwm_s  casper_pwm_t;
```

These are **forward declarations only**. The concrete `struct` definitions exist exclusively inside the board `.c` file. App/ code may hold and pass _pointers_ to these types but must never dereference or size them.

On Casper 2:
- `casper_spi_s` wraps `SPI_HandleTypeDef *h`.
- `casper_i2c_s` wraps `I2C_HandleTypeDef *h`.
- `casper_qspi_s` wraps `QSPI_HandleTypeDef *h` plus the registered IT handler and context pointers.
- `casper_adc_s` wraps `ADC_HandleTypeDef *h`, the channel number, and the per-channel `ADC_ChannelConfTypeDef`.
- `casper_pwm_s` wraps `TIM_HandleTypeDef *h` and the TIM_CHANNEL_n constant.

### casper_pin_t

```c
typedef struct {
    void    *port;
    uint32_t pin;
} casper_pin_t;
```

Generic GPIO pin descriptor. `port` is a `void *` that the board layer casts to the concrete GPIO port type (e.g. `GPIO_TypeDef *` on STM32). `pin` carries the bit-mask value as used natively by the HAL (`GPIO_PIN_n = 1u << n`). App/ code constructs `casper_pin_t` values from board-supplied constants defined in `board_casper2.h` (e.g. `BSP_PIN_IMU_CS`) and never forms these directly.

---

## 2. casper_time.h

### `uint32_t casper_millis(void)`

Returns milliseconds elapsed since power-on / reset. Equivalent to `HAL_GetTick()` on STM32. Wraps at 2^32 ms (~49.7 days). Safe to call from any execution context including ISRs. No side effects.

### `void casper_delay_ms(uint32_t ms)`

Blocking delay of at least `ms` milliseconds. Equivalent to `HAL_Delay()`. Must **not** be called from an ISR context. A value of 0 is a no-op.

### `uint32_t casper_micros(void)`

Returns microseconds elapsed since power-on / reset. On Casper 2 implemented using `DWT->CYCCNT / (SystemCoreClock / 1 000 000U)`. The board layer owns the `SystemCoreClock` constant — callers must never hard-code 432 000 000 or any other frequency (that is the entire purpose of this function). Wraps at 2^32 µs (~71.6 minutes). Use only for short-interval timing; use `casper_millis()` for longer intervals.

---

## 3. casper_gpio.h

### `casper_pin_state_t`

```c
typedef enum {
    CASPER_PIN_LOW  = 0,
    CASPER_PIN_HIGH = 1
} casper_pin_state_t;
```

### `void casper_gpio_write(casper_pin_t pin, casper_pin_state_t s)`

Drive `pin` to the given logic level. Equivalent to `HAL_GPIO_WritePin(port, pin, state)`. The pin must already be configured as a push-pull output by the board init. Calling on an input-configured pin has undefined behaviour.

### `casper_pin_state_t casper_gpio_read(casper_pin_t pin)`

Sample the current logic level of `pin`. Equivalent to `HAL_GPIO_ReadPin()`. Works for both input- and output-configured pins. Returns `CASPER_PIN_LOW` or `CASPER_PIN_HIGH`.

### `void casper_gpio_toggle(casper_pin_t pin)`

Toggle `pin` between HIGH and LOW. Equivalent to `HAL_GPIO_TogglePin()`. Must be an output-configured pin.

---

## 4. casper_spi.h

All three functions are **blocking**: they do not return until the transfer is complete or the timeout fires.

CS assertion and de-assertion are the **caller's responsibility** (use `casper_gpio_write()` with the device's `casper_pin_t cs`).

### `casper_status_t casper_spi_transmit(casper_spi_t *bus, const uint8_t *tx, uint16_t n, uint32_t to_ms)`

Transmit `n` bytes from `tx`. Received bytes are discarded. Equivalent to `HAL_SPI_Transmit()`. `tx` must not be NULL. Returns `CASPER_OK`, `CASPER_TIMEOUT`, or `CASPER_ERR`.

### `casper_status_t casper_spi_receive(casper_spi_t *bus, uint8_t *rx, uint16_t n, uint32_t to_ms)`

Receive `n` bytes into `rx`. The board layer transmits dummy bytes (0xFF) to clock in data. Equivalent to `HAL_SPI_Receive()`. `rx` must not be NULL.

### `casper_status_t casper_spi_transceive(casper_spi_t *bus, const uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t to_ms)`

Full-duplex transfer: simultaneously transmit `tx` and receive into `rx`, `n` bytes each. Equivalent to `HAL_SPI_TransmitReceive()`. `tx` and `rx` must be distinct non-overlapping buffers of at least `n` bytes.

---

## 5. casper_i2c.h

### addr8 convention — CRITICAL

All address parameters are the **8-bit, left-shifted** I2C device address:

```
addr8 = i2c_7bit_address << 1
```

Examples:
- MAX-M10M GPS: 7-bit = 0x42 → addr8 = 0x84
- MMC5983MA mag: 7-bit = 0x30 → addr8 = 0x60

This matches the STM32 HAL convention exactly. The board layer passes `addr8` unchanged to `HAL_I2C_*`. Do not re-shift inside the board layer. Existing call sites in the driver files already supply the shifted value and must not be modified.

### `casper_status_t casper_i2c_mem_read(casper_i2c_t *bus, uint16_t addr8, uint16_t reg, uint16_t reg_sz, uint8_t *buf, uint16_t n, uint32_t to_ms)`

Read `n` bytes from register `reg` of device at `addr8`. Equivalent to `HAL_I2C_Mem_Read()`.

- `reg_sz`: 1 for 8-bit register address (`I2C_MEMADD_SIZE_8BIT`), 2 for 16-bit (`I2C_MEMADD_SIZE_16BIT`).
- `buf` must be at least `n` bytes.
- Returns `CASPER_OK`, `CASPER_TIMEOUT`, or `CASPER_ERR`.

### `casper_status_t casper_i2c_mem_write(casper_i2c_t *bus, uint16_t addr8, uint16_t reg, uint16_t reg_sz, const uint8_t *buf, uint16_t n, uint32_t to_ms)`

Write `n` bytes to register `reg` of device at `addr8`. Equivalent to `HAL_I2C_Mem_Write()`. Parameters as above.

### `casper_status_t casper_i2c_master_tx(casper_i2c_t *bus, uint16_t addr8, const uint8_t *buf, uint16_t n, uint32_t to_ms)`

Transmit `n` bytes directly to `addr8` with no register sub-address phase. Equivalent to `HAL_I2C_Master_Transmit()`. Used for protocols where the payload itself encodes the address (e.g. UBX config frames sent to the GPS).

### `casper_status_t casper_i2c_dev_ready(casper_i2c_t *bus, uint16_t addr8, uint32_t trials, uint32_t to_ms)`

Check whether the device at `addr8` ACKs on the bus. Equivalent to `HAL_I2C_IsDeviceReady()`. Makes up to `trials` attempts; each attempt waits up to `to_ms` ms. Returns `CASPER_OK` if the device ACKed, `CASPER_TIMEOUT` or `CASPER_ERR` otherwise.

---

## 6. casper_adc.h

### `void casper_adc_init_all(void)`

Performs board-level ADC calibration and channel configuration for all logical ADC channels. On Casper 2 this calls `HAL_ADCEx_Calibration_Start()` for each ADC peripheral (`hadc1`, `hadc2`, `hadc3`) and configures the `ADC_ChannelConfTypeDef` for each continuity-sensing channel. Must be called once at startup after the generated `MX_ADC*_Init()` functions have run.

### `casper_status_t casper_adc_read(casper_adc_t *ch, uint16_t *out_raw)`

Perform a single-shot conversion on the channel represented by `ch`. Blocking.

On Casper 2: selects the channel rank, calls `HAL_ADC_Start()`, polls for conversion complete (`HAL_ADC_PollForConversion()`), reads the value (`HAL_ADC_GetValue()`), and calls `HAL_ADC_Stop()`.

`*out_raw` receives a 12-bit result in a `uint16_t` (range 0–4095 on STM32H7, 12-bit mode). The caller is responsible for converting raw counts to physical values (voltage → continuity present/absent). Returns `CASPER_OK`, `CASPER_TIMEOUT`, or `CASPER_ERR`.

---

## 7. casper_qspi.h

### casper_qspi_cmd_t — field meanings

| Field | Type | Meaning |
|---|---|---|
| `instruction` | `uint8_t` | Opcode byte (first byte of every QSPI frame). E.g. `0x12` = PAGE_PROGRAM_4B. |
| `instruction_lines` | `uint8_t` | Data lines for instruction phase. `1` = single SPI, `4` = quad. Maps to `QSPI_INSTRUCTION_1_LINE` / `_4_LINES`. |
| `address_lines` | `uint8_t` | Data lines for address phase. `0` = no address, `1` = single, `4` = quad. |
| `address_bytes` | `uint8_t` | Width of address in bytes: `3` = 24-bit (`QSPI_ADDRESS_24BITS`), `4` = 32-bit (`QSPI_ADDRESS_32BITS`). |
| `address` | `uint32_t` | Flash byte address. Ignored when `address_lines == 0`. |
| `data_lines` | `uint8_t` | Data lines for data phase. `0` = no data phase, `1` = single, `4` = quad. |
| `dummy_cycles` | `uint8_t` | Clock cycles inserted between address/alt-byte phase and data phase. Device-dependent. E.g. `8` for W25Q Fast Read. `0` = no dummies. |
| `data_len` | `uint32_t` | Bytes to transfer in data phase. Mapped to `NbData` in `QSPI_CommandTypeDef`. Ignored when `data_lines == 0`. |

### casper_qspi_poll_t — field meanings

| Field | Type | Meaning |
|---|---|---|
| `instruction` | `uint8_t` | Status-read opcode. `0x05` = READ_SR1 for W25Q. |
| `match` | `uint8_t` | Expected result after masking: `(polled_byte & mask) == match` signals completion. |
| `mask` | `uint8_t` | Bitmask applied to the polled byte. E.g. `0x01` to test only the WIP bit. |
| `poll_interval` | `uint8_t` | Polling interval in QSPI clock cycles between successive status reads. Mapped to `HAL_QSPI_AutoPolling.Interval`. Typical value: 16. |

### casper_qspi_evt_t

| Value | Meaning |
|---|---|
| `CASPER_QSPI_EVT_TX_DONE` | `casper_qspi_transmit_it()` completed without error. |
| `CASPER_QSPI_EVT_MATCH` | `casper_qspi_autopoll_it()` detected the expected status-register pattern. |
| `CASPER_QSPI_EVT_ERROR` | A peripheral or transfer error occurred during an IT operation. |

### Blocking functions

**`casper_qspi_command(dev, cmd, to_ms)`**  
Configures the QSPI peripheral and sends the command frame (instruction + address + dummy). Equivalent to `HAL_QSPI_Command()`. If `data_lines == 0` the operation is complete after this call. If `data_lines != 0`, immediately follow with `casper_qspi_transmit()` or `casper_qspi_receive()` to transfer data.

**`casper_qspi_transmit(dev, buf, to_ms)`**  
Transmit `data_len` bytes from `buf` in the data phase of the previously issued command. Equivalent to `HAL_QSPI_Transmit()`. Must be called immediately after `casper_qspi_command()` — the QSPI peripheral holds the command state until data transfer begins.

**`casper_qspi_receive(dev, buf, to_ms)`**  
Receive `data_len` bytes into `buf`. Equivalent to `HAL_QSPI_Receive()`. Same timing constraint as `casper_qspi_transmit()`.

All blocking functions: return `CASPER_OK`, `CASPER_TIMEOUT`, or `CASPER_ERR`.

### Non-blocking (IT) functions

**`casper_qspi_transmit_it(dev, buf)`**  
Starts a non-blocking transmit immediately after `casper_qspi_command()`. Equivalent to `HAL_QSPI_Transmit_IT()`. Returns `CASPER_OK` if the transfer was queued, `CASPER_ERR` if the peripheral is busy or a HAL error occurred. `buf` must remain valid until the `CASPER_QSPI_EVT_TX_DONE` or `CASPER_QSPI_EVT_ERROR` event fires.

**`casper_qspi_autopoll_it(dev, poll)`**  
Starts non-blocking status-register polling. Equivalent to `HAL_QSPI_AutoPolling_IT()`. The QSPI hardware issues `poll->instruction` repeatedly (at `poll->poll_interval` cycle intervals), compares `(received & poll->mask)` against `poll->match`, and on a match fires `CASPER_QSPI_EVT_MATCH`. On peripheral error fires `CASPER_QSPI_EVT_ERROR`. Returns `CASPER_OK` if polling started, `CASPER_ERR` otherwise.

### Event handler registration

**`casper_qspi_set_handler(dev, fn, ctx)`**  
Register a completion/error callback for IT operations. Only one handler per `casper_qspi_t`; a subsequent call replaces the previous registration.

The board layer stores `fn` and `ctx` inside the `casper_qspi_s` struct. When the QSPI peripheral fires `HAL_QSPI_TxCpltCallback`, `HAL_QSPI_StatusMatchCallback`, or `HAL_QSPI_ErrorCallback`, the board implementation translates each to the corresponding `casper_qspi_evt_t` and calls `fn(ctx, evt)`.

**The handler is invoked from an ISR context.** Handlers must be ISR-safe:
- No blocking calls (`casper_delay_ms`, `HAL_Delay`, etc.)
- No CDC transmit
- Only volatile/atomic data access or NVIC-masked critical sections

Passing `fn = NULL` disables the handler (subsequent IT events are silently dropped).

---

## 8. casper_crc.h

### `void casper_crc32_init(void)`

Initialises the CRC hardware unit for standard CRC-32 (polynomial 0x04C11DB7, reflected input, reflected output, initial value 0xFFFFFFFF, final XOR 0xFFFFFFFF — the IEEE 802.3 / PKZIP algorithm). On Casper 2 equivalent to `crc32_hw_init()`. Must be called once at startup after the CRC peripheral clock is enabled.

### `uint32_t casper_crc32_compute(const uint8_t *data, uint32_t len)`

Compute the CRC-32 of `len` bytes at `data`. Returns the 32-bit CRC. If `len == 0`, the return value is the CRC of an empty input (implementation-defined but consistent). `data` must not be NULL if `len > 0`.

---

## 9. casper_pwm.h

### `void casper_pwm_tone_start(casper_pwm_t *pwm)`

Enable the PWM channel. Equivalent to `HAL_TIM_PWM_Start()`. After this call the PWM pin is active at the rate set by the current ARR/CCR values. No sound is produced until `casper_pwm_tone_set()` is called with a non-zero CCR.

### `void casper_pwm_tone_set(casper_pwm_t *pwm, uint32_t arr, uint32_t ccr)`

Update frequency and duty cycle:

```
f_tone  = f_tim_clock / (arr + 1)
duty_%  = (ccr * 100) / (arr + 1)
```

The board layer writes `arr` to the timer's ARR register and `ccr` to the capture-compare register for the buzzer channel. On Casper 2 equivalent to `__HAL_TIM_SET_AUTORELOAD(htim, arr)` + `__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, ccr)`. The caller (`buzzer.c`) computes `arr` and `ccr` from the desired frequency and volume; the board layer must not re-interpret these values.

### `void casper_pwm_tone_off(casper_pwm_t *pwm)`

Stop the PWM channel and drive the pin to its idle (low) state. Equivalent to `HAL_TIM_PWM_Stop()`.

---

## 10. casper_port.h

Umbrella include. Every App/ file that migrates off HAL replaces:

```c
#include "stm32h7xx_hal.h"
#include "main.h"
```

with:

```c
#include "casper_port.h"
```

`casper_port.h` pulls in `<stdint.h>` and all sub-headers in dependency order. No HAL or CMSIS types are exposed through any of these headers.

---

## Implementation notes for board_casper2.c

- Include `main.h` and `stm32h7xx_hal.h` at the top of `board_casper2.c`. This is the **only** App/ file allowed to do so after migration (plus `App/radio/sx1276.c` by approved exception).
- Define the concrete structs `casper_spi_s`, `casper_i2c_s`, `casper_qspi_s`, `casper_adc_s`, `casper_pwm_s` before the function bodies.
- Export board singleton instances as `extern`-declared in `board_casper2.h` (e.g. `BSP_SPI_IMU`, `BSP_I2C_GPS`).
- Host the three QSPI IT callbacks (`HAL_QSPI_TxCpltCallback`, `HAL_QSPI_StatusMatchCallback`, `HAL_QSPI_ErrorCallback`) that dispatch to the registered handler via `casper_qspi_set_handler()`.
- All implementations are one-liner wrappers around their HAL equivalents; no business logic belongs here.
