# CASPER-2 Flight Data Logging PRD

**Purpose:** Implementation instructions for dual-rate flight data logging to QSPI flash, post-flight readout, and host-side decoding on the C.A.S.P.E.R.-2 flight computer.

**Target:** Claude Code agents working in STM32CubeIDE C project.

**Hardware:** STM32H750VBT6, 432 MHz, W25Q512JV 64 MB QSPI NOR flash.

**Repository:** https://github.com/AshwinVishwanath/C.A.S.P.E.R.-2

**CRITICAL: Fork from the `mc-testing` branch, NOT `main`.** The `main` branch does not contain the telemetry stack, command router, or sensor infrastructure.

**Replaces:** Levels 3-4 of CASPER_FLIGHT_CRITICAL_PRD.md. This document is the authoritative logging specification.

---

## HOW TO USE THIS DOCUMENT

This PRD is divided into **4 levels**. Each level is a self-contained unit of work.

**RULES FOR AGENTS:**

1. **Complete one level fully before starting the next.** Do not read ahead.
2. **Every level has a "DONE WHEN" section.** You are not done until every criterion passes.
3. **Every level has a "DO NOT TOUCH" section.** Violating this is a blocking error.
4. **Write tests first, then implementation.** Every level includes test specifications.
5. **All code is C99, no dynamic allocation (malloc/free/calloc), no floating-point in ISRs.**
6. **All structs must be initialized to zero at declaration.**
7. **Every function that can fail returns int (0 = success, negative = error).**

**File locations:**
- New modules go in `Software/App/flight/` (source and headers)
- Test harnesses go in `Software/App/test/`
- Existing code is in `Software/Core/Src/`, `Software/Core/Inc/`, and `Software/App/`
- The Makefile at `Software/Makefile` must be updated to include new `.c` files

---

## ARCHITECTURE OVERVIEW

### Design Philosophy

The logging system splits flight data into two streams based on update rate, not processing stage:

| Stream | Content | Entry Size | Rate | Bandwidth |
|--------|---------|-----------|------|-----------|
| **High-rate** | Flight dynamics: inertial sensors, baro, EKF state, attitude | 64 bytes | 250 Hz | 16 KB/s |
| **Low-rate** | System health: mag, GPS, power, radio, continuity | 64 bytes | 10 Hz | 640 B/s |
| **Summary** | Human-readable event strings | variable | event-driven | ~bytes |

Both entry types are 64 bytes — 4 entries per 256-byte flash page, clean alignment throughout. Total bandwidth is ~16.6 KB/s, giving ~6 full 10-minute flights in 60 MB of flash.

### Why This Split

- **High-rate captures everything needed for EKF reprocessing in MATLAB.** Raw sensor data + EKF state at 250 Hz means you can validate the EKF, retune parameters, and replay flights offline with full fidelity.
- **Low-rate captures everything else at the rate it actually changes.** GPS updates at 1-10 Hz, magnetometer at 10 Hz, power monitoring is slow — logging these at 250 Hz wastes flash with stale copies.
- **Both are 64 bytes for uniform flash alignment.** No mixed-size packing, no variable-length entries in the main log. Simplifies the writer, the reader, and the Python decoder.

### Flash Layout

```
Address         Region                    Size
─────────────────────────────────────────────────────
0x00000000      High-rate log region      56 MB   (0x00000000 - 0x037FFFFF)
0x03800000      Low-rate log region        6 MB   (0x03800000 - 0x03DFFFFF)
0x03E00000      Summary log region         1 MB   (0x03E00000 - 0x03EFFFFF)
0x03F00000      Config storage             4 KB   (0x03F00000 - 0x03F00FFF)
0x03F01000      Flight index / reserved   ~1 MB   (0x03F01000 - 0x03FFFFFF)
0x04000000      End of 64 MB chip
```

Capacity per region at design rates:

| Region | Size | Entry Rate | Capacity |
|--------|------|-----------|----------|
| High-rate | 56 MB | 16 KB/s | ~58 minutes = ~5.8 flights × 10 min |
| Low-rate | 6 MB | 640 B/s | ~2.6 hours = ~15.6 flights × 10 min |
| Summary | 1 MB | ~bytes/event | thousands of flights |

The high-rate region is the bottleneck. 56 MB / 16 KB/s = 3584 seconds ≈ 59.7 minutes of total logging.

---

## REFERENCE: Existing Code You Interface With (READ ONLY)

### Flash Driver — `Software/Core/Src/w25q512jv.c`

```c
// Initialize QSPI flash. Returns 0 on success.
int w25q512jv_init(void);

// Write data to flash. addr must be page-aligned (256 bytes).
// len must be <= 256. Flash must be erased first.
int w25q512jv_write(uint32_t addr, const uint8_t *data, uint32_t len);

// Read data from flash.
int w25q512jv_read(uint32_t addr, uint8_t *data, uint32_t len);

// Erase a 4 KB sector. addr must be sector-aligned.
int w25q512jv_erase_sector(uint32_t addr);

// Erase a 64 KB block. addr must be 64KB-aligned.
int w25q512jv_erase_block64(uint32_t addr);

// Erase entire chip. Takes up to 400 seconds.
int w25q512jv_erase_chip(void);
```

Page program time: ~0.7ms per 256-byte page. Sector erase: ~45ms. Block64 erase: ~150ms.

### FSM State — `Software/App/telemetry/tlm_types.h`

```c
typedef uint8_t fsm_state_t;

#define FSM_STATE_PAD        0x0
#define FSM_STATE_BOOST      0x1
#define FSM_STATE_COAST      0x2
#define FSM_STATE_COAST_1    0x3
#define FSM_STATE_SUSTAIN    0x4
#define FSM_STATE_COAST_2    0x5
#define FSM_STATE_APOGEE     0x6
#define FSM_STATE_DROGUE     0x7
#define FSM_STATE_MAIN       0x8
#define FSM_STATE_RECOVERY   0x9
#define FSM_STATE_TUMBLE     0xA
#define FSM_STATE_LANDED     0xB
```

### EKF — `Software/Core/Src/casper_ekf.c`

```c
typedef struct {
    float x[4];         // [altitude_m, velocity_mps, accel_bias, baro_bias]
    float P[4][4];      // covariance
    float baro_ref;     // ground-level baro altitude
} casper_ekf_t;
```

### Attitude — `Software/Core/Src/casper_attitude.c`

```c
typedef struct {
    float q[4];         // [w, x, y, z] body-to-NED quaternion
    bool  in_flight;
} casper_attitude_t;
```

### Sensor Globals (in main.c)

```c
extern lsm6dso32_t imu;           // .data_ready set by EXTI ISR
extern adxl372_t   adxl;          // .data_ready set by EXTI ISR
extern ms5611_t    baro;          // async state machine, .pressure_pa, .temperature_c100
extern mmc5983ma_t mag;           // .data_ready set by EXTI ISR
extern max_m10m_t  gps;           // polled via I2C, .lat_deg7, .lon_deg7, etc.
extern casper_ekf_t     ekf;
extern casper_attitude_t att;
```

### CRC-32 Hardware — `Software/App/telemetry/crc32_hw.c`

```c
uint32_t crc32_hw_compute(const uint8_t *data, uint32_t len);
```

Uses the STM32H7 hardware CRC unit. Ethernet/zlib polynomial (0x04C11DB7), init 0xFFFFFFFF, input/output reflect, final XOR 0xFFFFFFFF.

---

## LEVEL 1: Log Entry Types, Flash Layout, and Core Writer

**Goal:** Define the two log entry structs, flash region constants, ring buffer, double-buffered flash writer, and summary logger. This is the complete logging infrastructure — no sensor integration yet.

### DO NOT TOUCH
- Any existing files in `Software/Core/`, `Software/App/pyro/`, `Software/App/command/`, `Software/App/telemetry/`, `Software/App/pack/`, `Software/App/diag/`, `Software/App/fsm/`
- `Software/Core/Src/w25q512jv.c` — use its API, don't modify it

### Files to Create

**`Software/App/flight/flight_log.h`**

```c
#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ════════════════════════════════════════════════════════════════
//  HIGH-RATE ENTRY — 64 bytes, logged at 250 Hz
//  Complete flight dynamics record: inertial sensors + baro + EKF + attitude
// ════════════════════════════════════════════════════════════════

typedef struct __attribute__((packed)) {
    // ── Timestamp + Header (6 bytes) ──
    uint32_t timestamp_us;          // 4  DWT cycle counter / TIM, µs since boot
    uint8_t  fresh;                 // 1  bitfield: b0=imu, b1=highg, b2=baro
    uint8_t  fsm_state;             // 1  FSM_STATE_* from tlm_types.h

    // ── LSM6DSO32 IMU (14 bytes) ──
    int16_t  accel_mg[3];           // 6  body-frame accel, milligees (±32000)
    int16_t  gyro_raw[3];           // 6  raw register LSBs (70 mdps/LSB at ±2000 dps)
    int16_t  imu_temp_c100;         // 2  die temp, °C × 100

    // ── ADXL372 high-G accel (6 bytes) ──
    int16_t  highg_10mg[3];         // 6  body-frame accel, 10 mg units (±200g = ±20000)

    // ── MS5611 barometer (6 bytes) ──
    uint32_t baro_pa;               // 4  compensated pressure, Pascals (typ ~101325)
    int16_t  baro_temp_c100;        // 2  die temp, °C × 100

    // ── EKF state (12 bytes) ──
    int32_t  ekf_alt_cm;            // 4  altitude AGL, cm (±21,474 km — NO CLAMP)
    int32_t  ekf_vel_cmps;          // 4  vertical velocity, cm/s +up (±21,474 km/s — NO CLAMP)
    int16_t  ekf_abias_mmps2;       // 2  accel bias estimate, mm/s² (±32.7 m/s²)
    int16_t  ekf_bbias_cm;          // 2  baro bias estimate, cm (±327 m)

    // ── Attitude (8 bytes) ──
    int16_t  quat_packed[3];        // 6  smallest-three quaternion encoding (Q14 fixed-point)
    int16_t  tilt_cdeg;             // 2  tilt from vertical, centidegrees (0 = vertical)

    // ── Flags + Reserved (12 bytes) ──
    uint8_t  flags;                 // 1  b0=baro_gated b1=launched b2=mag_valid
    uint8_t  reserved[11];          // 11 future use (e.g. second baro, TVC)
} highrate_entry_t;                 // = 64 bytes

_Static_assert(sizeof(highrate_entry_t) == 64, "High-rate entry must be exactly 64 bytes");


// ════════════════════════════════════════════════════════════════
//  LOW-RATE ENTRY — 64 bytes, logged at 10 Hz
//  System health: magnetometer, GPS, power, radio, continuity
// ════════════════════════════════════════════════════════════════

typedef struct __attribute__((packed)) {
    // ── Timestamp + Header (6 bytes) ──
    uint32_t timestamp_us;          // 4  µs since boot
    uint8_t  fsm_state;             // 1
    uint8_t  flags;                 // 1  b0=firing b1=test_mode b2=sim_active

    // ── Magnetometer (8 bytes) ──
    int16_t  mag_raw[3];            // 6  raw register values, MMC5983MA
    int16_t  mag_temp_c100;         // 2  die temp °C × 100 (0x7FFF if unavailable)

    // ── Power (8 bytes) ──
    uint16_t batt_mv;               // 2  battery voltage, millivolts
    int16_t  batt_ma;               // 2  battery current, milliamps (negative = charging)
    uint8_t  cont_scaled[4];        // 4  per-channel continuity ADC, ~20 mV resolution
                                    //    = (raw_adc >> 8) for 16-bit ADC

    // ── GPS (16 bytes) ──
    int32_t  gps_lat_deg7;          // 4  latitude, degrees × 1e7 (u-blox standard)
    int32_t  gps_lon_deg7;          // 4  longitude, degrees × 1e7
    int16_t  gps_alt_dm;            // 2  altitude MSL, decimetres (±3276.7 m)
    int16_t  gps_vel_d_cmps;        // 2  downward velocity, cm/s (NED convention)
    uint8_t  gps_sats;              // 1  satellites used in fix
    uint8_t  gps_fix;               // 1  fix type (0=none, 2=2D, 3=3D)
    uint8_t  gps_pdop;              // 1  PDOP × 10 (0-25.5, lower=better)
    uint8_t  gps_fresh;             // 1  b0=new fix this tick

    // ── Radio + Pyro Status (6 bytes) ──
    uint8_t  radio_tx_seq;          // 1  wrapping TX packet counter
    uint8_t  radio_rx_good;         // 1  wrapping good RX counter
    uint8_t  radio_rx_bad;          // 1  wrapping bad RX counter
    int8_t   radio_rssi;            // 1  last RSSI, dBm (typ -30 to -120)
    int8_t   radio_snr;             // 1  last SNR, dB (LoRa demod)
    uint8_t  pyro_arm_cont;         // 1  high nibble=arm bitmap, low nibble=continuity bitmap

    // ── Reserved (20 bytes) ──
    uint8_t  reserved[20];          // 20 future: servo pos, TVC angles, second baro, etc.
} lowrate_entry_t;                  // = 64 bytes

_Static_assert(sizeof(lowrate_entry_t) == 64, "Low-rate entry must be exactly 64 bytes");


// ════════════════════════════════════════════════════════════════
//  SUMMARY EVENT — variable length, human-readable
// ════════════════════════════════════════════════════════════════

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;          // mission time, ms
    uint8_t  msg_len;               // length of message string (max 250)
    char     msg[];                 // flexible array member, NOT null-terminated in flash
} flight_log_summary_entry_t;


// ════════════════════════════════════════════════════════════════
//  FLASH LAYOUT CONSTANTS
// ════════════════════════════════════════════════════════════════

// High-rate log region: 56 MB
#define FLASH_HR_START          0x00000000
#define FLASH_HR_END            0x03800000  // exclusive

// Low-rate log region: 6 MB
#define FLASH_LR_START          0x03800000
#define FLASH_LR_END            0x03E00000  // exclusive

// Summary log region: 1 MB
#define FLASH_SUMMARY_START     0x03E00000
#define FLASH_SUMMARY_END       0x03F00000  // exclusive

// Config region: 4 KB
#define FLASH_CONFIG_START      0x03F00000
#define FLASH_CONFIG_END        0x03F01000

// Flash geometry
#define FLASH_PAGE_SIZE         256
#define FLASH_SECTOR_SIZE       4096
#define FLASH_BLOCK64_SIZE      65536

// Entries per geometry unit
#define HR_ENTRIES_PER_PAGE     (FLASH_PAGE_SIZE / sizeof(highrate_entry_t))    // 4
#define HR_ENTRIES_PER_SECTOR   (FLASH_SECTOR_SIZE / sizeof(highrate_entry_t))  // 64
#define LR_ENTRIES_PER_PAGE     (FLASH_PAGE_SIZE / sizeof(lowrate_entry_t))     // 4
#define LR_ENTRIES_PER_SECTOR   (FLASH_SECTOR_SIZE / sizeof(lowrate_entry_t))   // 64

// Logging rates
#define HR_LOG_RATE_HZ          250
#define LR_LOG_RATE_HZ          10

// Ring buffer (pre-launch, in AXI SRAM)
#define HR_RING_SECONDS         5
#define HR_RING_ENTRIES         (HR_RING_SECONDS * HR_LOG_RATE_HZ)             // 1250
#define HR_RING_BYTES           (HR_RING_ENTRIES * sizeof(highrate_entry_t))    // 80000 = ~78 KB

#define LR_RING_SECONDS         10
#define LR_RING_ENTRIES         (LR_RING_SECONDS * LR_LOG_RATE_HZ)            // 100
#define LR_RING_BYTES           (LR_RING_ENTRIES * sizeof(lowrate_entry_t))    // 6400


// ════════════════════════════════════════════════════════════════
//  FRESH BITFIELD (highrate_entry_t.fresh)
// ════════════════════════════════════════════════════════════════

#define FRESH_IMU               0x01    // bit 0: LSM6DSO32 updated this tick
#define FRESH_HIGHG             0x02    // bit 1: ADXL372 updated this tick
#define FRESH_BARO              0x04    // bit 2: MS5611 updated this tick


// ════════════════════════════════════════════════════════════════
//  API
// ════════════════════════════════════════════════════════════════

// Initialize logging system. Call after w25q512jv_init().
// Scans flash for write pointers (finds first erased page in each region).
// Returns 0 on success, negative on flash read error.
int flight_log_init(void);

// Write one high-rate entry.
// PAD state: writes to HR ring buffer in RAM.
// Flight states (BOOST through MAIN): writes to double buffer, flushes to flash.
// LANDED state: ignored.
void flight_log_write_hr(const highrate_entry_t *entry, fsm_state_t state);

// Write one low-rate entry.
// Same PAD/flight/landed behavior as high-rate.
void flight_log_write_lr(const lowrate_entry_t *entry, fsm_state_t state);

// Call when launch is detected. Commits both ring buffers to flash,
// then switches to direct flash logging.
// Erases sectors ahead as needed. Blocking for the ring buffer flush.
void flight_log_commit_ring_buffers(void);

// Write a summary string event to the summary flash region.
// Uses snprintf internally — pass printf-style format + args.
// Max message length: 250 bytes.
void flight_log_summary(uint32_t timestamp_ms, const char *fmt, ...);

// Flush any remaining buffered data to flash. Call at landing.
void flight_log_flush(void);

// Return current write address for each region (for telemetry/debug).
uint32_t flight_log_get_hr_addr(void);
uint32_t flight_log_get_lr_addr(void);
uint32_t flight_log_get_summary_addr(void);

// Return entry counts for each region.
uint32_t flight_log_get_hr_count(void);
uint32_t flight_log_get_lr_count(void);

// Erase all flight log data. Erases HR, LR, and summary regions.
// WARNING: This erases 63 MB. Takes up to ~60 seconds using block64 erase.
// Only call when USB-connected and idle (PAD or LANDED state).
int flight_log_erase_all(void);

// Process pending flash writes. Call from main loop.
// Returns number of pages written this call (0 if nothing pending).
// Non-blocking: writes at most one sector per call to keep latency bounded.
int flight_log_process(void);

// Diagnostics
uint32_t flight_log_get_dropped_hr(void);
uint32_t flight_log_get_dropped_lr(void);

#endif
```

**`Software/App/flight/flight_log.c`** — Implementation

#### Internal State

```c
#include "flight_log.h"
#include "w25q512jv.h"
#include "crc32_hw.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// ── High-rate ring buffer (pre-launch, AXI SRAM) ──
static highrate_entry_t hr_ring[HR_RING_ENTRIES]
    __attribute__((section(".axi_sram")));
static uint32_t hr_ring_head  = 0;   // next write position
static uint32_t hr_ring_count = 0;   // entries written (saturates at HR_RING_ENTRIES)

// ── Low-rate ring buffer (pre-launch, AXI SRAM) ──
static lowrate_entry_t lr_ring[LR_RING_ENTRIES]
    __attribute__((section(".axi_sram")));
static uint32_t lr_ring_head  = 0;
static uint32_t lr_ring_count = 0;

// ── High-rate double buffer (in-flight) ──
static highrate_entry_t hr_buf_a[HR_ENTRIES_PER_SECTOR];  // 4096 bytes
static highrate_entry_t hr_buf_b[HR_ENTRIES_PER_SECTOR];  // 4096 bytes
static highrate_entry_t *hr_active = hr_buf_a;
static highrate_entry_t *hr_flush  = hr_buf_b;
static uint32_t hr_buf_idx     = 0;
static bool     hr_flush_pending = false;

// ── Low-rate double buffer (in-flight) ──
static lowrate_entry_t lr_buf_a[LR_ENTRIES_PER_SECTOR];   // 4096 bytes
static lowrate_entry_t lr_buf_b[LR_ENTRIES_PER_SECTOR];   // 4096 bytes
static lowrate_entry_t *lr_active = lr_buf_a;
static lowrate_entry_t *lr_flush  = lr_buf_b;
static uint32_t lr_buf_idx     = 0;
static bool     lr_flush_pending = false;

// ── Flash write pointers ──
static uint32_t hr_write_addr      = FLASH_HR_START;
static uint32_t lr_write_addr      = FLASH_LR_START;
static uint32_t summary_write_addr = FLASH_SUMMARY_START;

// ── Entry counters ──
static uint32_t hr_entry_count     = 0;
static uint32_t lr_entry_count     = 0;

// ── Drop counters ──
static uint32_t hr_dropped = 0;
static uint32_t lr_dropped = 0;

// ── State ──
static bool logging_active = false;  // true after commit_ring_buffers
```

#### Architecture

```
Pre-launch (PAD state):
  flight_log_write_hr() → hr_ring[hr_ring_head++] (circular)
  flight_log_write_lr() → lr_ring[lr_ring_head++] (circular)

Launch detected:
  flight_log_commit_ring_buffers()
    → erase sectors as needed in HR and LR regions
    → write ring buffers to flash sequentially (oldest first)
    → set write pointers to next free address
    → set logging_active = true

In-flight (BOOST through MAIN):
  flight_log_write_hr() → hr_active[hr_buf_idx++]
    When hr_buf_idx == HR_ENTRIES_PER_SECTOR (64):
      → swap hr_active and hr_flush
      → set hr_flush_pending = true, reset hr_buf_idx = 0
      → if previous flush not done: increment hr_dropped

  flight_log_process() — called from main loop
    if hr_flush_pending:
      → erase sector at hr_write_addr (if needed)
      → write hr_flush to flash page-by-page (16 pages × 256 bytes)
      → hr_write_addr += FLASH_SECTOR_SIZE
      → hr_flush_pending = false
    (same for lr_flush_pending)

Landing:
  flight_log_flush()
    → write any partial hr/lr buffers to flash (pad with 0xFF)
    → write final summary entry
```

#### Sector Erase Strategy

**Do NOT pre-erase the entire flash at init.** Instead, erase sectors just-in-time:

1. At `flight_log_commit_ring_buffers()`: erase the first N sectors needed for the ring buffer data.
2. During flight: after writing a sector, erase the **next** sector ahead while the current write buffer fills. This gives 64 entries ÷ 250 Hz = 256 ms of fill time, far more than the ~45ms sector erase.
3. For the low-rate stream: erase ahead similarly, but timing is even more relaxed (64 entries ÷ 10 Hz = 6.4 seconds).

```c
// Erase-ahead tracking
static uint32_t hr_erased_up_to = FLASH_HR_START;  // exclusive
static uint32_t lr_erased_up_to = FLASH_LR_START;

static void ensure_erased(uint32_t addr, uint32_t *erased_up_to) {
    while (*erased_up_to <= addr) {
        w25q512jv_erase_sector(*erased_up_to);
        *erased_up_to += FLASH_SECTOR_SIZE;
    }
}
```

#### Ring Buffer Commit

When `flight_log_commit_ring_buffers()` is called:

```c
void flight_log_commit_ring_buffers(void) {
    // Write HR ring buffer to flash
    uint32_t hr_count = (hr_ring_count < HR_RING_ENTRIES) ? hr_ring_count : HR_RING_ENTRIES;
    uint32_t hr_start = (hr_ring_count < HR_RING_ENTRIES) ? 0 : hr_ring_head;

    // Calculate sectors needed and erase
    uint32_t hr_sectors = (hr_count + HR_ENTRIES_PER_SECTOR - 1) / HR_ENTRIES_PER_SECTOR;
    for (uint32_t s = 0; s < hr_sectors; s++) {
        w25q512jv_erase_sector(FLASH_HR_START + s * FLASH_SECTOR_SIZE);
    }
    hr_erased_up_to = FLASH_HR_START + hr_sectors * FLASH_SECTOR_SIZE;

    // Write entries in chronological order
    for (uint32_t i = 0; i < hr_count; i++) {
        uint32_t ring_idx = (hr_start + i) % HR_RING_ENTRIES;
        // Accumulate into page buffer, flush every 4 entries
        // ... (write page when 4 entries accumulated)
    }

    hr_write_addr = FLASH_HR_START + hr_count * sizeof(highrate_entry_t);
    hr_entry_count = hr_count;

    // Same for LR ring buffer, writing to FLASH_LR_START
    // ...

    // Erase one sector ahead for both streams
    ensure_erased(hr_write_addr, &hr_erased_up_to);
    ensure_erased(lr_write_addr, &lr_erased_up_to);

    logging_active = true;
}
```

**IMPORTANT:** The ring buffer commit is blocking. At 250 Hz × 5 seconds = 1250 entries × 64 bytes = 80 KB, this requires 80000 / 256 = 313 page writes × 0.7ms = ~219ms. During this time, the sensor loop is stalled. This is acceptable because it happens exactly once — at the moment of launch detection. The rocket is already accelerating and the first ~200ms of boost data is in the ring buffer.

#### Double Buffer Flush

`flight_log_process()` writes one complete sector per call:

```c
int flight_log_process(void) {
    int pages_written = 0;

    if (hr_flush_pending) {
        // Ensure target sector is erased
        ensure_erased(hr_write_addr, &hr_erased_up_to);

        // Write 16 pages (16 × 256 = 4096 bytes = one sector)
        for (int p = 0; p < (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE); p++) {
            uint32_t page_addr = hr_write_addr + p * FLASH_PAGE_SIZE;
            const uint8_t *page_data = (const uint8_t *)hr_flush + p * FLASH_PAGE_SIZE;
            w25q512jv_write(page_addr, page_data, FLASH_PAGE_SIZE);
            pages_written++;
        }

        hr_write_addr += FLASH_SECTOR_SIZE;
        hr_entry_count += HR_ENTRIES_PER_SECTOR;
        hr_flush_pending = false;

        // Erase next sector ahead (for next flush)
        if (hr_write_addr < FLASH_HR_END) {
            ensure_erased(hr_write_addr, &hr_erased_up_to);
        }
    }

    // Same for lr_flush_pending
    if (lr_flush_pending) {
        ensure_erased(lr_write_addr, &lr_erased_up_to);
        for (int p = 0; p < (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE); p++) {
            uint32_t page_addr = lr_write_addr + p * FLASH_PAGE_SIZE;
            const uint8_t *page_data = (const uint8_t *)lr_flush + p * FLASH_PAGE_SIZE;
            w25q512jv_write(page_addr, page_data, FLASH_PAGE_SIZE);
            pages_written++;
        }
        lr_write_addr += FLASH_SECTOR_SIZE;
        lr_entry_count += LR_ENTRIES_PER_SECTOR;
        lr_flush_pending = false;

        if (lr_write_addr < FLASH_LR_END) {
            ensure_erased(lr_write_addr, &lr_erased_up_to);
        }
    }

    return pages_written;
}
```

**Timing:** One HR sector flush = 16 pages × 0.7ms = 11.2ms. At 250 Hz, the active buffer fills in 64/250 = 256ms. The flush comfortably completes before the next buffer swap.

#### Summary Log Writer

```c
void flight_log_summary(uint32_t timestamp_ms, const char *fmt, ...) {
    char msg_buf[251];   // max 250 chars + NUL
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len > 250) len = 250;

    // Build entry in page buffer
    uint8_t entry_buf[256];
    flight_log_summary_entry_t *entry = (flight_log_summary_entry_t *)entry_buf;
    entry->timestamp_ms = timestamp_ms;
    entry->msg_len = (uint8_t)len;
    memcpy(entry->msg, msg_buf, len);

    uint32_t total_len = 5 + len;  // header + message

    // Check region bounds
    uint32_t aligned_addr = summary_write_addr & ~(FLASH_PAGE_SIZE - 1);
    if (summary_write_addr + total_len > FLASH_SUMMARY_END) return;  // region full

    // Ensure sector is erased
    uint32_t sector_addr = summary_write_addr & ~(FLASH_SECTOR_SIZE - 1);
    if (sector_addr >= summary_write_addr) {
        // Need to erase this sector first
        // (simplified — in practice track erased-up-to like HR/LR)
    }

    // Write page-by-page as needed
    // Note: summary entries may not be page-aligned. Handle partial page writes.
    // The simplest approach: accumulate in a static page buffer, flush when full.
    // For summary entries (infrequent, small), direct unaligned writes are OK
    // if the flash supports sub-page writes (W25Q512JV does — any byte within
    // an erased page can be programmed, and you can program the same page
    // multiple times as long as you only change 1→0 bits).

    // Direct write (works because flash is erased to 0xFF)
    while (total_len > 0) {
        uint32_t page_offset = summary_write_addr % FLASH_PAGE_SIZE;
        uint32_t chunk = FLASH_PAGE_SIZE - page_offset;
        if (chunk > total_len) chunk = total_len;

        w25q512jv_write(summary_write_addr, entry_buf + (5 + len - total_len), chunk);
        summary_write_addr += chunk;
        total_len -= chunk;
    }
}
```

**Note:** The W25Q512JV supports partial page programming — you can write to any byte within a page as long as that byte is still 0xFF. Summary entries are infrequent (typically <20 per flight), so there's no performance concern.

#### Memory Budget

```
AXI SRAM usage:
  HR ring buffer:  1250 × 64 = 80,000 bytes (78.1 KB)
  LR ring buffer:   100 × 64 =  6,400 bytes ( 6.3 KB)
  HR double buf:    2 × 4096  =  8,192 bytes ( 8.0 KB)
  LR double buf:    2 × 4096  =  8,192 bytes ( 8.0 KB)
  ──────────────────────────────────────────────────────
  Total:                        102,784 bytes (100.4 KB)

Available AXI SRAM: 512 KB
Remaining after logging: ~412 KB (for EKF, attitude, telemetry, etc.)
```

### Test Harness

**`Software/App/test/test_flight_log.c`**

For host-PC testing, create a mock `w25q512jv` that writes to a local byte array (simulating 1 MB of flash). The mock tracks erased sectors and validates that writes only go to erased pages.

**Test 1: HR ring buffer wraps correctly**
```
Write 1500 entries to HR ring (capacity 1250).
Verify: hr_ring_count = 1500, hr_ring_head = 1500 % 1250 = 250.
Commit ring buffers.
Verify: 1250 entries written to flash. First entry is #250, last is #1499.
Timestamps are monotonically increasing.
```

**Test 2: LR ring buffer wraps correctly**
```
Write 150 entries to LR ring (capacity 100).
Verify: lr_ring_count = 150, lr_ring_head = 150 % 100 = 50.
Commit ring buffers.
Verify: 100 entries written to flash. First entry is #50, last is #149.
```

**Test 3: HR double buffer swap**
```
Commit ring buffers (empty), then switch to flight logging.
Write 64 HR entries (one sector). Verify: hr_flush_pending = true.
Call flight_log_process(). Verify: 4096 bytes written to flash at FLASH_HR_START.
Write 64 more. Verify: second buffer active, first completed.
```

**Test 4: LR double buffer swap**
```
Same as Test 3 but for low-rate entries at FLASH_LR_START.
```

**Test 5: No drops under normal timing**
```
Simulate 10 seconds of flight:
  250 HR writes/s, call flight_log_process() every 4ms.
  10 LR writes/s.
Verify: hr_dropped == 0, lr_dropped == 0.
```

**Test 6: Drop detection under stress**
```
Write 64 HR entries without calling flight_log_process().
Write 64 more (triggers second swap while first flush pending).
Verify: hr_dropped == 64 (one full buffer dropped).
```

**Test 7: Summary log write and read-back**
```
Write 5 summary strings with known timestamps and messages.
Read them back from summary flash region.
Verify: all timestamps and message strings match.
```

**Test 8: Entry struct sizes**
```
Verify: sizeof(highrate_entry_t) == 64.
Verify: sizeof(lowrate_entry_t) == 64.
```

**Test 9: Region boundary protection**
```
Set hr_write_addr near FLASH_HR_END. Attempt to write.
Verify: no write past FLASH_HR_END. Entries are dropped.
Same for LR region.
```

**Test 10: Erase-ahead correctness**
```
After commit, verify: at least one sector ahead is erased in both HR and LR regions.
After each sector flush, verify: next sector is erased before it's needed.
```

### DONE WHEN

- [ ] All 10 test cases pass
- [ ] `highrate_entry_t` is exactly 64 bytes
- [ ] `lowrate_entry_t` is exactly 64 bytes
- [ ] Ring buffers wrap correctly and commit in chronological order
- [ ] Double buffers swap without blocking the sensor loop
- [ ] `flight_log_process()` is non-blocking (bounded by one sector write)
- [ ] Sector erase-ahead prevents write stalls
- [ ] No dynamic memory allocation
- [ ] Flash writes use `w25q512jv_write()` and `w25q512jv_erase_sector()` APIs only
- [ ] Memory budget fits in AXI SRAM with >400 KB remaining
- [ ] Region boundaries are respected (no writes past region end)

---

## LEVEL 2: Post-Flight Readout

**Goal:** Implement USB-based flight log retrieval. Stream raw binary log data over USB CDC with CRC validation. Provide a Python decoder that converts both log streams to CSV.

**Depends on:** Level 1 (flight_log.h, flash layout, entry types)

### DO NOT TOUCH
- Level 1 code
- `Software/Core/Src/usbd_cdc_if.c` — existing CDC implementation (use its API)
- `Software/Core/Src/w25q512jv.c`

### Files to Create

**`Software/App/flight/flight_readout.h`**

```c
#ifndef FLIGHT_READOUT_H
#define FLIGHT_READOUT_H

#include <stdint.h>

// ════════════════════════════════════════════════════════════════
//  READOUT PROTOCOL
// ════════════════════════════════════════════════════════════════
//
//  The host (MC or Python tool) sends a single-byte command over CDC.
//  The FC responds with a binary stream.
//
//  Commands:
//    0x01  Stream high-rate log
//    0x02  Stream low-rate log
//    0x03  Stream summary log
//    0x04  Get log metadata (counts, addresses)
//    0x05  Erase all logs
//
//  Stream format (0x01, 0x02):
//    [4 bytes] Magic: "CASP" (0x43 0x41 0x53 0x50)
//    [1 byte]  Stream ID (0x01 = HR, 0x02 = LR)
//    [1 byte]  Entry size (64)
//    [2 bytes] Reserved (0x00 0x00)
//    [4 bytes] Entry count (u32 LE)
//    [4 bytes] CRC-32 of above 16 bytes
//    [N × 64 bytes] Log entries, sequential
//    [4 bytes] CRC-32 of all entry data
//
//  Summary stream format (0x03):
//    [4 bytes] Magic: "SUMM" (0x53 0x55 0x4D 0x4D)
//    [4 bytes] Total payload size in bytes (u32 LE)
//    [4 bytes] CRC-32 of above 8 bytes
//    [N bytes] Summary entries (length-prefixed: [u32 timestamp][u8 len][char[] msg])
//    [4 bytes] CRC-32 of all summary data
//
//  Metadata format (0x04):
//    [4 bytes] Magic: "META" (0x4D 0x45 0x54 0x41)
//    [4 bytes] HR entry count (u32 LE)
//    [4 bytes] LR entry count (u32 LE)
//    [4 bytes] Summary bytes used (u32 LE)
//    [4 bytes] HR write address (u32 LE)
//    [4 bytes] LR write address (u32 LE)
//    [4 bytes] CRC-32 of above 28 bytes
//
// ════════════════════════════════════════════════════════════════

// Stream high-rate log data over USB CDC.
// Blocking — call only in LANDED or PAD state with USB connected.
// Returns 0 on success, -1 on USB error.
int flight_readout_stream_hr(void);

// Stream low-rate log data over USB CDC.
int flight_readout_stream_lr(void);

// Stream summary log over USB CDC.
int flight_readout_stream_summary(void);

// Send metadata response.
int flight_readout_send_metadata(void);

// Process a readout command byte. Called from CDC receive handler.
// Returns 0 if command handled, -1 if invalid.
int flight_readout_handle_command(uint8_t cmd);

// Return total number of entries for each stream.
uint32_t flight_readout_get_hr_count(void);
uint32_t flight_readout_get_lr_count(void);

#endif
```

**`Software/App/flight/flight_readout.c`**

The streaming implementation reads flash in page-sized chunks and transmits via `CDC_Transmit_FS()`:

```c
static int stream_log_region(uint32_t start_addr, uint32_t entry_count,
                             uint8_t entry_size, uint8_t stream_id) {
    uint8_t header[16];
    uint32_t crc;

    // Build header
    header[0] = 'C'; header[1] = 'A'; header[2] = 'S'; header[3] = 'P';
    header[4] = stream_id;
    header[5] = entry_size;
    header[6] = 0x00; header[7] = 0x00;
    memcpy(&header[8], &entry_count, 4);  // LE
    crc = crc32_hw_compute(header, 12);
    memcpy(&header[12], &crc, 4);

    // Send header
    cdc_transmit_blocking(header, 16);

    // Stream entries page-by-page, computing running CRC
    uint32_t data_crc = 0xFFFFFFFF;  // CRC init
    uint32_t total_bytes = entry_count * entry_size;
    uint8_t page_buf[FLASH_PAGE_SIZE];
    uint32_t addr = start_addr;

    while (total_bytes > 0) {
        uint32_t chunk = (total_bytes < FLASH_PAGE_SIZE) ? total_bytes : FLASH_PAGE_SIZE;
        w25q512jv_read(addr, page_buf, chunk);

        // Update running CRC
        data_crc = crc32_hw_compute_continue(data_crc, page_buf, chunk);

        // Transmit over CDC (with flow control / retry)
        cdc_transmit_blocking(page_buf, chunk);

        addr += chunk;
        total_bytes -= chunk;
    }

    // Send data CRC
    cdc_transmit_blocking((uint8_t *)&data_crc, 4);

    return 0;
}
```

**Note on CDC flow control:** `CDC_Transmit_FS()` can fail if the TX buffer is full. Implement `cdc_transmit_blocking()` as a wrapper that retries with a small delay:

```c
static void cdc_transmit_blocking(const uint8_t *data, uint32_t len) {
    uint32_t offset = 0;
    while (offset < len) {
        uint32_t chunk = len - offset;
        if (chunk > 512) chunk = 512;  // CDC max packet
        while (CDC_Transmit_FS((uint8_t *)(data + offset), chunk) == USBD_BUSY) {
            // Spin — host should be draining the buffer
        }
        offset += chunk;
    }
}
```

**`tools/decode_flight_log.py`** — Host-side decoder

```python
#!/usr/bin/env python3
"""
Casper-2 Flight Log Decoder

Reads binary log streams from USB CDC (serial port) or saved binary files.
Outputs CSV files for high-rate and low-rate data.

Usage:
  python decode_flight_log.py --port /dev/ttyACM0 --output flight_data/
  python decode_flight_log.py --file hr_dump.bin --type hr --output flight_data/
"""

import struct
import argparse
import csv
import sys
from pathlib import Path

# ── Entry definitions ──────────────────────────────────────────

HR_ENTRY_SIZE = 64
LR_ENTRY_SIZE = 64

HR_FIELDS = [
    ('timestamp_us',      'I'),    # uint32
    ('fresh',             'B'),    # uint8
    ('fsm_state',         'B'),    # uint8
    ('accel_mg_x',        'h'),    # int16
    ('accel_mg_y',        'h'),
    ('accel_mg_z',        'h'),
    ('gyro_raw_x',        'h'),    # int16
    ('gyro_raw_y',        'h'),
    ('gyro_raw_z',        'h'),
    ('imu_temp_c100',     'h'),    # int16
    ('highg_10mg_x',      'h'),    # int16
    ('highg_10mg_y',      'h'),
    ('highg_10mg_z',      'h'),
    ('baro_pa',           'I'),    # uint32
    ('baro_temp_c100',    'h'),    # int16
    ('ekf_alt_cm',        'i'),    # int32
    ('ekf_vel_cmps',      'i'),    # int32
    ('ekf_abias_mmps2',   'h'),    # int16
    ('ekf_bbias_cm',      'h'),    # int16
    ('quat_p0',           'h'),    # int16
    ('quat_p1',           'h'),
    ('quat_p2',           'h'),
    ('tilt_cdeg',         'h'),    # int16
    ('flags',             'B'),    # uint8
]

HR_STRUCT_FMT = '<' + ''.join(f for _, f in HR_FIELDS)
HR_STRUCT_SIZE = struct.calcsize(HR_STRUCT_FMT)
# Remaining bytes are reserved padding
HR_RESERVED = HR_ENTRY_SIZE - HR_STRUCT_SIZE

LR_FIELDS = [
    ('timestamp_us',      'I'),
    ('fsm_state',         'B'),
    ('flags',             'B'),
    ('mag_raw_x',         'h'),
    ('mag_raw_y',         'h'),
    ('mag_raw_z',         'h'),
    ('mag_temp_c100',     'h'),
    ('batt_mv',           'H'),    # uint16
    ('batt_ma',           'h'),    # int16
    ('cont_1',            'B'),
    ('cont_2',            'B'),
    ('cont_3',            'B'),
    ('cont_4',            'B'),
    ('gps_lat_deg7',      'i'),    # int32
    ('gps_lon_deg7',      'i'),
    ('gps_alt_dm',        'h'),
    ('gps_vel_d_cmps',    'h'),
    ('gps_sats',          'B'),
    ('gps_fix',           'B'),
    ('gps_pdop',          'B'),
    ('gps_fresh',         'B'),
    ('radio_tx_seq',      'B'),
    ('radio_rx_good',     'B'),
    ('radio_rx_bad',      'B'),
    ('radio_rssi',        'b'),    # int8
    ('radio_snr',         'b'),    # int8
    ('pyro_arm_cont',     'B'),
]

LR_STRUCT_FMT = '<' + ''.join(f for _, f in LR_FIELDS)
LR_STRUCT_SIZE = struct.calcsize(LR_STRUCT_FMT)
LR_RESERVED = LR_ENTRY_SIZE - LR_STRUCT_SIZE


def decode_hr_entry(data):
    """Decode a 64-byte high-rate entry into a dict."""
    values = struct.unpack(HR_STRUCT_FMT, data[:HR_STRUCT_SIZE])
    entry = {}
    for (name, _), val in zip(HR_FIELDS, values):
        entry[name] = val
    # Derived fields
    entry['accel_g_x'] = entry['accel_mg_x'] / 1000.0
    entry['accel_g_y'] = entry['accel_mg_y'] / 1000.0
    entry['accel_g_z'] = entry['accel_mg_z'] / 1000.0
    entry['gyro_dps_x'] = entry['gyro_raw_x'] * 0.070
    entry['gyro_dps_y'] = entry['gyro_raw_y'] * 0.070
    entry['gyro_dps_z'] = entry['gyro_raw_z'] * 0.070
    entry['imu_temp_c'] = entry['imu_temp_c100'] / 100.0
    entry['highg_g_x'] = entry['highg_10mg_x'] * 0.01
    entry['highg_g_y'] = entry['highg_10mg_y'] * 0.01
    entry['highg_g_z'] = entry['highg_10mg_z'] * 0.01
    entry['baro_hpa'] = entry['baro_pa'] / 100.0
    entry['baro_temp_c'] = entry['baro_temp_c100'] / 100.0
    entry['ekf_alt_m'] = entry['ekf_alt_cm'] / 100.0
    entry['ekf_vel_mps'] = entry['ekf_vel_cmps'] / 100.0
    entry['ekf_abias_mps2'] = entry['ekf_abias_mmps2'] / 1000.0
    entry['ekf_bbias_m'] = entry['ekf_bbias_cm'] / 100.0
    entry['tilt_deg'] = entry['tilt_cdeg'] / 100.0
    entry['time_s'] = entry['timestamp_us'] / 1e6
    entry['fresh_imu'] = bool(entry['fresh'] & 0x01)
    entry['fresh_highg'] = bool(entry['fresh'] & 0x02)
    entry['fresh_baro'] = bool(entry['fresh'] & 0x04)
    entry['baro_gated'] = bool(entry['flags'] & 0x01)
    return entry


def decode_lr_entry(data):
    """Decode a 64-byte low-rate entry into a dict."""
    values = struct.unpack(LR_STRUCT_FMT, data[:LR_STRUCT_SIZE])
    entry = {}
    for (name, _), val in zip(LR_FIELDS, values):
        entry[name] = val
    # Derived
    entry['time_s'] = entry['timestamp_us'] / 1e6
    entry['batt_v'] = entry['batt_mv'] / 1000.0
    entry['gps_lat'] = entry['gps_lat_deg7'] / 1e7
    entry['gps_lon'] = entry['gps_lon_deg7'] / 1e7
    entry['gps_alt_m'] = entry['gps_alt_dm'] / 10.0
    entry['radio_rssi_dbm'] = entry['radio_rssi']
    entry['radio_snr_db'] = entry['radio_snr']
    entry['arm_bitmap'] = (entry['pyro_arm_cont'] >> 4) & 0x0F
    entry['cont_bitmap'] = entry['pyro_arm_cont'] & 0x0F
    return entry


def write_hr_csv(entries, output_path):
    """Write decoded high-rate entries to CSV."""
    columns = [
        'time_s', 'timestamp_us', 'fsm_state',
        'fresh_imu', 'fresh_highg', 'fresh_baro',
        'accel_g_x', 'accel_g_y', 'accel_g_z',
        'gyro_dps_x', 'gyro_dps_y', 'gyro_dps_z',
        'imu_temp_c',
        'highg_g_x', 'highg_g_y', 'highg_g_z',
        'baro_hpa', 'baro_temp_c',
        'ekf_alt_m', 'ekf_vel_mps',
        'ekf_abias_mps2', 'ekf_bbias_m',
        'tilt_deg', 'baro_gated'
    ]
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for e in entries:
            writer.writerow(e)


def write_lr_csv(entries, output_path):
    """Write decoded low-rate entries to CSV."""
    columns = [
        'time_s', 'timestamp_us', 'fsm_state',
        'mag_raw_x', 'mag_raw_y', 'mag_raw_z', 'mag_temp_c100',
        'batt_v', 'batt_ma',
        'cont_1', 'cont_2', 'cont_3', 'cont_4',
        'gps_lat', 'gps_lon', 'gps_alt_m',
        'gps_vel_d_cmps', 'gps_sats', 'gps_fix', 'gps_pdop',
        'radio_tx_seq', 'radio_rx_good', 'radio_rx_bad',
        'radio_rssi_dbm', 'radio_snr_db',
        'arm_bitmap', 'cont_bitmap'
    ]
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for e in entries:
            writer.writerow(e)
```

The full script should also include:
- Serial port reading with header validation
- CRC-32 verification (using `zlib.crc32`)
- Summary log decoding (parse length-prefixed strings)
- Command-line interface for selecting port/file, stream type, and output directory
- Error reporting for CRC failures and truncated data

### Test Harness

**Test 1: HR round-trip encode/decode**
```
Create 100 HR entries with known values (e.g. entry N has accel_mg[0] = N * 100).
Write to flash via flight_log_write_hr(). Read back via flight_readout_stream_hr().
Pipe binary through Python decoder. Verify: all values match originals.
```

**Test 2: LR round-trip encode/decode**
```
Same for 50 LR entries via flight_log_write_lr() / flight_readout_stream_lr().
```

**Test 3: CRC validation**
```
Stream HR log. Corrupt one byte in the binary output.
Verify: Python decoder reports CRC failure.
```

**Test 4: Summary round-trip**
```
Write 5 summary strings. Stream via flight_readout_stream_summary().
Verify: Python decoder outputs all 5 messages with correct timestamps.
```

**Test 5: Metadata response**
```
After writing some entries, send 0x04 command.
Verify: metadata response contains correct HR count, LR count, addresses.
```

### DONE WHEN

- [ ] All 5 test cases pass
- [ ] HR binary stream decodes to correct CSV via Python script
- [ ] LR binary stream decodes to correct CSV via Python script
- [ ] Summary strings are readable
- [ ] CRC catches corruption
- [ ] Metadata command returns accurate counts
- [ ] Works over USB CDC without data loss
- [ ] Python script handles both serial port and file input

---

## LEVEL 3: Entry Population Helpers

**Goal:** Create helper functions that populate `highrate_entry_t` and `lowrate_entry_t` from the actual sensor/EKF/attitude state. These are called from the main loop (Level 4) but are separate for testability.

**Depends on:** Level 1 (entry types)

### DO NOT TOUCH
- Levels 1-2 code
- Any sensor drivers

### Files to Create

**`Software/App/flight/flight_log_helpers.h`**

```c
#ifndef FLIGHT_LOG_HELPERS_H
#define FLIGHT_LOG_HELPERS_H

#include "flight_log.h"
#include <stdint.h>

// ── Sensor snapshot structs ──────────────────────────────────
// These intermediate structs decouple the log entry population from
// the specific sensor driver types. The main loop fills these from
// the actual sensor globals, then passes them to the populate functions.

typedef struct {
    float    accel_ms2[3];       // body-frame, m/s²
    int16_t  gyro_raw[3];       // raw register LSBs (from lsm6dso32)
    float    imu_temp_c;        // die temperature, °C
    bool     updated;           // EXTI data-ready fired this tick
} imu_snapshot_t;

typedef struct {
    float    accel_ms2[3];      // body-frame, m/s² (from ADXL372)
    bool     updated;
} highg_snapshot_t;

typedef struct {
    uint32_t pressure_pa;       // compensated pressure, Pascals
    float    temp_c;            // die temperature, °C
    bool     updated;           // new conversion completed this tick
} baro_snapshot_t;

typedef struct {
    float    alt_m;             // EKF x[0], altitude AGL
    float    vel_mps;           // EKF x[1], vertical velocity +up
    float    accel_bias;        // EKF x[2], accelerometer bias
    float    baro_bias;         // EKF x[3], barometer bias
} ekf_snapshot_t;

typedef struct {
    float    q[4];              // [w, x, y, z] body-to-NED
    float    tilt_deg;          // tilt from vertical
} attitude_snapshot_t;

typedef struct {
    int16_t  raw[3];            // raw register values
    float    temp_c;            // die temperature (0x7FFF/100 if unavail)
    bool     updated;
} mag_snapshot_t;

typedef struct {
    int32_t  lat_deg7;          // degrees × 1e7
    int32_t  lon_deg7;
    int16_t  alt_dm;            // decimetres MSL
    int16_t  vel_d_cmps;        // downward cm/s
    uint8_t  sats;
    uint8_t  fix;
    uint8_t  pdop;              // ×10
    bool     fresh;             // new fix this tick
} gps_snapshot_t;

typedef struct {
    uint16_t batt_mv;
    int16_t  batt_ma;
    uint16_t cont_raw[4];       // raw 16-bit ADC values per channel
} power_snapshot_t;

typedef struct {
    uint8_t  tx_seq;
    uint8_t  rx_good;
    uint8_t  rx_bad;
    int8_t   rssi;
    int8_t   snr;
} radio_snapshot_t;


// ── Populate functions ───────────────────────────────────────

// Populate a high-rate entry from sensor snapshots.
// timestamp_us: DWT/TIM counter value.
// fsm_state: current FSM state.
// baro_gated: true if baro is currently gated (transonic).
void flight_log_populate_hr(
    highrate_entry_t *entry,
    uint32_t timestamp_us,
    uint8_t  fsm_state,
    const imu_snapshot_t     *imu,
    const highg_snapshot_t   *highg,
    const baro_snapshot_t    *baro,
    const ekf_snapshot_t     *ekf,
    const attitude_snapshot_t *att,
    bool baro_gated
);

// Populate a low-rate entry from sensor snapshots.
void flight_log_populate_lr(
    lowrate_entry_t *entry,
    uint32_t timestamp_us,
    uint8_t  fsm_state,
    const mag_snapshot_t    *mag,
    const gps_snapshot_t    *gps,
    const power_snapshot_t  *pwr,
    const radio_snapshot_t  *radio,
    uint8_t pyro_arm_bitmap,
    uint8_t pyro_cont_bitmap,
    bool firing,
    bool test_mode,
    bool sim_active
);

#endif
```

**`Software/App/flight/flight_log_helpers.c`**

```c
#include "flight_log_helpers.h"
#include <string.h>
#include <math.h>

// ── Quaternion smallest-three packing ────────────────────────
// Find the component with the largest absolute value, drop it.
// Encode the other three as Q14 fixed-point (int16, range [-1, +1]).
// Store the drop index in the upper 2 bits of quat_packed[0].

static void pack_quaternion(const float q[4], int16_t packed[3]) {
    // Find largest component
    int max_idx = 0;
    float max_val = fabsf(q[0]);
    for (int i = 1; i < 4; i++) {
        float a = fabsf(q[i]);
        if (a > max_val) { max_val = a; max_idx = i; }
    }

    // Ensure the dropped component is positive (negate quat if needed)
    float sign = (q[max_idx] < 0.0f) ? -1.0f : 1.0f;

    // Pack the other three
    int j = 0;
    for (int i = 0; i < 4; i++) {
        if (i == max_idx) continue;
        // Q14: multiply by 16384, clamp to int16 range
        float val = q[i] * sign * 16384.0f;
        if (val > 16383.0f) val = 16383.0f;
        if (val < -16384.0f) val = -16384.0f;
        packed[j] = (int16_t)val;
        j++;
    }

    // Store drop index in upper 2 bits of packed[0]
    // Clear upper 2 bits, then set
    packed[0] = (packed[0] & 0x3FFF) | ((int16_t)(max_idx & 0x3) << 14);
}


void flight_log_populate_hr(
    highrate_entry_t *entry,
    uint32_t timestamp_us,
    uint8_t  fsm_state,
    const imu_snapshot_t     *imu,
    const highg_snapshot_t   *highg,
    const baro_snapshot_t    *baro,
    const ekf_snapshot_t     *ekf,
    const attitude_snapshot_t *att,
    bool baro_gated)
{
    memset(entry, 0, sizeof(*entry));

    entry->timestamp_us = timestamp_us;
    entry->fsm_state    = fsm_state;

    // Fresh bitfield
    entry->fresh = 0;
    if (imu->updated)   entry->fresh |= FRESH_IMU;
    if (highg->updated)  entry->fresh |= FRESH_HIGHG;
    if (baro->updated)   entry->fresh |= FRESH_BARO;

    // LSM6DSO32 — convert m/s² to milligees
    for (int i = 0; i < 3; i++) {
        entry->accel_mg[i] = (int16_t)(imu->accel_ms2[i] / 9.80665f * 1000.0f);
    }
    // Gyro: store raw register LSBs directly (caller passes raw values)
    for (int i = 0; i < 3; i++) {
        entry->gyro_raw[i] = imu->gyro_raw[i];
    }
    entry->imu_temp_c100 = (int16_t)(imu->imu_temp_c * 100.0f);

    // ADXL372 — convert m/s² to 10-milligee units
    for (int i = 0; i < 3; i++) {
        entry->highg_10mg[i] = (int16_t)(highg->accel_ms2[i] / 9.80665f * 100.0f);
    }

    // MS5611 — store compensated pressure directly
    entry->baro_pa       = baro->pressure_pa;
    entry->baro_temp_c100 = (int16_t)(baro->temp_c * 100.0f);

    // EKF state — float to fixed-point, NO CLAMPING (int32 range is enormous)
    entry->ekf_alt_cm      = (int32_t)(ekf->alt_m * 100.0f);
    entry->ekf_vel_cmps    = (int32_t)(ekf->vel_mps * 100.0f);
    entry->ekf_abias_mmps2 = (int16_t)(ekf->accel_bias * 1000.0f);
    entry->ekf_bbias_cm    = (int16_t)(ekf->baro_bias * 100.0f);

    // Attitude — smallest-three quaternion packing
    pack_quaternion(att->q, entry->quat_packed);
    entry->tilt_cdeg = (int16_t)(att->tilt_deg * 100.0f);

    // Flags
    entry->flags = 0;
    if (baro_gated) entry->flags |= 0x01;
    if (fsm_state != 0 /* FSM_STATE_PAD */) entry->flags |= 0x02;  // launched
}


void flight_log_populate_lr(
    lowrate_entry_t *entry,
    uint32_t timestamp_us,
    uint8_t  fsm_state,
    const mag_snapshot_t    *mag,
    const gps_snapshot_t    *gps,
    const power_snapshot_t  *pwr,
    const radio_snapshot_t  *radio,
    uint8_t pyro_arm_bitmap,
    uint8_t pyro_cont_bitmap,
    bool firing,
    bool test_mode,
    bool sim_active)
{
    memset(entry, 0, sizeof(*entry));

    entry->timestamp_us = timestamp_us;
    entry->fsm_state    = fsm_state;

    // Flags
    entry->flags = 0;
    if (firing)    entry->flags |= 0x01;
    if (test_mode) entry->flags |= 0x02;
    if (sim_active) entry->flags |= 0x04;

    // Magnetometer
    for (int i = 0; i < 3; i++) {
        entry->mag_raw[i] = mag->raw[i];
    }
    entry->mag_temp_c100 = (int16_t)(mag->temp_c * 100.0f);

    // Power
    entry->batt_mv = pwr->batt_mv;
    entry->batt_ma = pwr->batt_ma;
    for (int i = 0; i < 4; i++) {
        entry->cont_scaled[i] = (uint8_t)(pwr->cont_raw[i] >> 8);  // 16-bit → 8-bit
    }

    // GPS
    entry->gps_lat_deg7   = gps->lat_deg7;
    entry->gps_lon_deg7   = gps->lon_deg7;
    entry->gps_alt_dm     = gps->alt_dm;
    entry->gps_vel_d_cmps = gps->vel_d_cmps;
    entry->gps_sats       = gps->sats;
    entry->gps_fix        = gps->fix;
    entry->gps_pdop       = gps->pdop;
    entry->gps_fresh      = gps->fresh ? 0x01 : 0x00;

    // Radio
    entry->radio_tx_seq  = radio->tx_seq;
    entry->radio_rx_good = radio->rx_good;
    entry->radio_rx_bad  = radio->rx_bad;
    entry->radio_rssi    = radio->rssi;
    entry->radio_snr     = radio->snr;

    // Pyro: arm in high nibble, continuity in low nibble
    entry->pyro_arm_cont = ((pyro_arm_bitmap & 0x0F) << 4) | (pyro_cont_bitmap & 0x0F);
}
```

### Test Harness

**Test 1: HR populate — value conversion accuracy**
```
Create an imu_snapshot_t with accel = [9.80665, 0, 0] m/s².
Populate HR entry. Verify: accel_mg[0] = 1000, accel_mg[1] = 0, accel_mg[2] = 0.
```

**Test 2: HR populate — EKF no-clamp verification**
```
Create an ekf_snapshot_t with alt_m = 50000.0 (50 km), vel_mps = 3000.0 (3 km/s).
Populate HR entry. Verify: ekf_alt_cm = 5000000, ekf_vel_cmps = 300000.
Both fit in int32 without clamping.
```

**Test 3: Quaternion round-trip**
```
For 10 known quaternions (unit quaternions with various orientations):
  Pack via pack_quaternion().
  Unpack in Python.
  Verify: recovered quaternion matches original within ±0.001 per component.
```

**Test 4: LR populate — GPS encoding**
```
Create gps_snapshot with lat_deg7 = 515678900, lon_deg7 = -1234567.
Populate LR entry. Verify exact values preserved.
```

**Test 5: LR populate — pyro bitmap packing**
```
arm_bitmap = 0x05 (CH1 + CH3 armed), cont_bitmap = 0x0F (all continuity).
Populate LR entry. Verify: pyro_arm_cont = 0x5F.
```

**Test 6: Fresh bitfield**
```
imu.updated=true, highg.updated=false, baro.updated=true.
Populate HR entry. Verify: fresh = 0x05 (FRESH_IMU | FRESH_BARO).
```

### DONE WHEN

- [ ] All 6 test cases pass
- [ ] Float-to-fixed conversion matches expected values
- [ ] Quaternion smallest-three encoding recoverable to ±0.001
- [ ] No clamping on EKF altitude or velocity for extreme values
- [ ] GPS coordinates preserved exactly (integer passthrough)
- [ ] All bitfield packing correct

---

## LEVEL 4: Main Loop Integration

**Goal:** Wire Levels 1-3 into the actual `main.c` superloop. This adds dual-rate logging to the existing sensor loop.

**Depends on:** All previous levels passing their tests.

### DO NOT TOUCH
- Sensor drivers (`lsm6dso32.c`, `adxl372.c`, `ms5611.c`, etc.)
- Telemetry stack (`tlm_manager.c`, etc.)
- Pyro system (`pyro_manager.c`, `pyro_logic.c`)
- EKF and attitude modules

### Files to Modify

**`Software/Core/Src/main.c`** — the main superloop

### Integration Architecture

Add these includes and state variables at the top of main.c:

```c
#include "flight_log.h"
#include "flight_log_helpers.h"

// ── Logging state ──
static uint32_t hr_log_counter = 0;         // counts IMU ticks
static uint32_t lr_log_counter = 0;         // counts IMU ticks
static const uint32_t HR_LOG_EVERY = 833 / HR_LOG_RATE_HZ;   // 833/250 ≈ 3
static const uint32_t LR_LOG_EVERY = 833 / LR_LOG_RATE_HZ;   // 833/10  = 83

// ── Sensor snapshots (updated each tick) ──
static imu_snapshot_t      snap_imu   = {0};
static highg_snapshot_t    snap_highg = {0};
static baro_snapshot_t     snap_baro  = {0};
static ekf_snapshot_t      snap_ekf   = {0};
static attitude_snapshot_t snap_att   = {0};
static mag_snapshot_t      snap_mag   = {0};
static gps_snapshot_t      snap_gps   = {0};
static power_snapshot_t    snap_pwr   = {0};
static radio_snapshot_t    snap_radio = {0};
```

**Initialization (after existing sensor/EKF/attitude init):**

```c
flight_log_init();
```

**Main loop — integrated logging:**

```c
while (1) {
    // === Layer 1: Always runs ===
    cmd_router_process();
    cac_tick();
    pyro_mgr_tick();
    flight_log_process();       // ← NEW: process pending flash writes

    // === Layer 2: IMU-driven (833 Hz) ===
    if (imu.data_ready) {
        imu.data_ready = false;

        // Read sensors (existing code)
        float accel_ms2[3], gyro_rads[3];
        // ... existing sensor read + axis remap ...

        // ── Update snapshots ──
        snap_imu.accel_ms2[0] = accel_ms2[0];
        snap_imu.accel_ms2[1] = accel_ms2[1];
        snap_imu.accel_ms2[2] = accel_ms2[2];
        snap_imu.gyro_raw[0]  = imu.gyro_raw[0];  // raw register values
        snap_imu.gyro_raw[1]  = imu.gyro_raw[1];
        snap_imu.gyro_raw[2]  = imu.gyro_raw[2];
        snap_imu.imu_temp_c   = imu.temperature_c;
        snap_imu.updated      = true;

        // High-G: check if ADXL updated this tick
        if (adxl.data_ready) {
            adxl.data_ready = false;
            // ... read ADXL372 ...
            snap_highg.accel_ms2[0] = /* converted */;
            snap_highg.accel_ms2[1] = /* converted */;
            snap_highg.accel_ms2[2] = /* converted */;
            snap_highg.updated = true;
        } else {
            snap_highg.updated = false;
        }

        // Attitude update (existing)
        casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, dt);

        // EKF predict (existing)
        casper_ekf_predict(&ekf, &att, accel_ms2, dt);

        // Update EKF and attitude snapshots
        snap_ekf.alt_m      = ekf.x[0];
        snap_ekf.vel_mps    = ekf.x[1];
        snap_ekf.accel_bias = ekf.x[2];
        snap_ekf.baro_bias  = ekf.x[3];

        snap_att.q[0] = att.q[0];
        snap_att.q[1] = att.q[1];
        snap_att.q[2] = att.q[2];
        snap_att.q[3] = att.q[3];
        snap_att.tilt_deg = flight_fsm_tilt_from_quat(att.q);

        // FSM tick (existing from flight-critical PRD)
        // ...

        // ── HIGH-RATE LOGGING (250 Hz) ──
        hr_log_counter++;
        if (hr_log_counter >= HR_LOG_EVERY) {
            hr_log_counter = 0;

            highrate_entry_t hr_entry;
            flight_log_populate_hr(
                &hr_entry,
                get_timestamp_us(),     // DWT or TIM counter
                flight_fsm_get_state(),
                &snap_imu, &snap_highg, &snap_baro,
                &snap_ekf, &snap_att,
                baro_gated              // from EKF baro gating logic
            );

            flight_log_write_hr(&hr_entry, flight_fsm_get_state());

            // Reset fresh flags after logging
            snap_imu.updated   = false;
            snap_baro.updated  = false;
        }

        // ── LOW-RATE LOGGING (10 Hz) ──
        lr_log_counter++;
        if (lr_log_counter >= LR_LOG_EVERY) {
            lr_log_counter = 0;

            lowrate_entry_t lr_entry;
            flight_log_populate_lr(
                &lr_entry,
                get_timestamp_us(),
                flight_fsm_get_state(),
                &snap_mag, &snap_gps, &snap_pwr, &snap_radio,
                pyro_mgr_get_arm_bitmap(),
                pyro_mgr_get_cont_bitmap(),
                pyro_mgr_is_firing(),
                pyro_mgr_is_test_mode(),
                flight_fsm_sim_active()
            );

            flight_log_write_lr(&lr_entry, flight_fsm_get_state());
        }

        // On launch: commit ring buffers
        if (/* state just transitioned from PAD */) {
            flight_log_commit_ring_buffers();
            flight_log_summary(HAL_GetTick(), "LAUNCH accel=%.1fg",
                max_accel / 9.80665f);
        }

        // On landing: flush and write summary
        if (/* state just transitioned to LANDED */) {
            flight_log_summary(HAL_GetTick(),
                "LANDED maxAlt=%.1fm maxVel=%.1fm/s peakTilt=%.1fdeg",
                ctx->max_altitude_m, ctx->max_velocity_mps, ctx->peak_tilt_deg);
            flight_log_flush();
        }
    }

    // === Layer 3: Async sensor updates ===
    if (ms5611_tick(&baro) == 1) {
        snap_baro.pressure_pa = baro.pressure_pa;
        snap_baro.temp_c      = baro.temperature_c;
        snap_baro.updated     = true;

        float baro_alt = ms5611_get_altitude(&baro, 1013.25f) - flight.baro_ref_m;
        casper_ekf_update_baro(&ekf, baro_alt);
    }

    if (mag.data_ready) {
        // ... existing mag read ...
        snap_mag.raw[0]  = mag.raw[0];
        snap_mag.raw[1]  = mag.raw[1];
        snap_mag.raw[2]  = mag.raw[2];
        snap_mag.temp_c  = mag.temperature_c;
        snap_mag.updated = true;
    }

    if (max_m10m_tick(&gps) == 1) {
        snap_gps.lat_deg7   = gps.lat_deg7;
        snap_gps.lon_deg7   = gps.lon_deg7;
        snap_gps.alt_dm     = gps.alt_dm;
        snap_gps.vel_d_cmps = gps.vel_d_cmps;
        snap_gps.sats       = gps.sats;
        snap_gps.fix        = gps.fix;
        snap_gps.pdop       = gps.pdop;
        snap_gps.fresh      = true;
    } else {
        snap_gps.fresh = false;
    }

    // Power monitoring (every N ticks)
    // snap_pwr.batt_mv = read_batt_mv();
    // snap_pwr.cont_raw[0..3] = read_continuity_adc(0..3);

    // Radio stats (from LoRa driver)
    // snap_radio.tx_seq = radio_get_tx_seq();
    // snap_radio.rssi = radio_get_last_rssi();
    // etc.

    // === Layer 4: Periodic telemetry (existing) ===
    tlm_tick();

    // === Layer 5: Readout (only when idle) ===
    if (flight_fsm_get_state() == FSM_STATE_PAD ||
        flight_fsm_get_state() == FSM_STATE_LANDED) {
        // Check for readout commands from CDC
        if (cdc_ring_available() > 0) {
            uint8_t cmd = cdc_ring_read_byte();
            flight_readout_handle_command(cmd);
        }
    }
}
```

#### Timestamp Source

Use the DWT cycle counter for microsecond timestamps:

```c
static inline uint32_t get_timestamp_us(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);  // 432 MHz → µs
}
```

The DWT counter wraps every ~9.9 seconds at 432 MHz in µs (uint32 max / 1e6 / 432 ≈ 9.9s). For flights longer than this, either use a 64-bit software extension or switch to a hardware timer. For the initial implementation, the 32-bit µs counter is sufficient — the Python decoder can detect and handle wraps.

**Alternative:** Use TIM2 or TIM5 (32-bit timers on STM32H7) configured to count at 1 MHz for a non-wrapping 32-bit µs counter good for ~71.6 minutes.

### Summary Event Triggers

Add `flight_log_summary()` calls at key flight events:

| Event | Example String |
|-------|---------------|
| Boot | `"BOOT fw=v0.2.0 cfg=0xA3B1C2D4"` |
| Launch | `"LAUNCH accel=5.2g"` |
| Burnout | `"BURNOUT motor=1 t=+2.3s"` |
| Staging | `"STAGING motor=2 t=+8.1s"` |
| Apogee | `"APOGEE alt=5148m vel=0.3m/s votes=3/3"` |
| Pyro fire | `"PYRO ch=1 role=APOGEE dur=1000ms"` |
| Main deploy | `"MAIN alt=298m vel=-15.2m/s"` |
| Landing | `"LANDED alt=2.1m maxAlt=5148m maxVel=312m/s t=95.4s"` |
| Baro gate on | `"BARO_GATE_ON vel=135m/s"` |
| Baro gate off | `"BARO_GATE_OFF vel=89m/s"` |
| Safety violation | `"SAFETY tilt=35.2deg limit=30 ch=3 BLOCKED"` |

### Test Procedure (requires hardware)

**Test A: Power-on → PAD state logging**
1. Power on FC, wait 10 seconds
2. Connect USB, send 0x04 (metadata)
3. Verify: HR count ≈ 2500 (250 Hz × 10s, ring buffer saturated at 1250)
4. Verify: LR count ≈ 100 (10 Hz × 10s, ring buffer at 100)

**Test B: Sim flight end-to-end**
1. Start sim flight via MC
2. Wait for LANDED
3. Send 0x01 (stream HR), decode with Python
4. Verify: timestamps span full flight, FSM states transition correctly
5. Send 0x02 (stream LR), decode
6. Verify: GPS, power, and radio data present
7. Send 0x03 (stream summary)
8. Verify: LAUNCH, APOGEE, PYRO, LANDED events present with plausible values

**Test C: Drop counter verification**
1. After sim flight, check `flight_log_get_dropped_hr()` and `flight_log_get_dropped_lr()`
2. Verify: both are 0 (no drops under normal conditions)

**Test D: Flash capacity**
1. Run a 10-minute sim (or calculate from entry counts)
2. Verify: HR data ≈ 9.6 MB, LR data ≈ 384 KB
3. Verify: both fit within their flash regions

### DONE WHEN

- [ ] Sim flight produces correct FSM transitions in decoded HR CSV
- [ ] EKF altitude and velocity in HR log match telemetry output
- [ ] Baro pressure values are plausible (not all zeros or stale)
- [ ] Fresh bitfield correctly flags which sensors updated each tick
- [ ] LR log contains GPS data (at least when GPS has fix)
- [ ] LR log contains plausible battery voltage and continuity values
- [ ] LR log contains radio RSSI/SNR
- [ ] Summary log contains all expected flight events
- [ ] No drops (dropped_hr == 0, dropped_lr == 0) during sim flight
- [ ] No hard faults or watchdog resets during logging
- [ ] Build fits in 128 KB flash
- [ ] Total AXI SRAM usage < 200 KB (logging + existing allocations)
- [ ] USB telemetry still works during flight (not blocked by logging)
- [ ] Readout commands work in PAD and LANDED states

---

## APPENDIX A: Encoding Reference

### Gyro Raw LSBs

The `gyro_raw[3]` field stores raw register values from the LSM6DSO32. To convert to degrees per second:

```
gyro_dps = gyro_raw * 0.070   (at ±2000 dps, 70 mdps/LSB)
gyro_rads = gyro_raw * 0.070 * π / 180
```

Storing raw LSBs avoids floating-point conversion in the hot path and preserves full sensor resolution.

### Accelerometer milligees

The `accel_mg[3]` field stores body-frame acceleration in milligees:

```
accel_mg = accel_ms2 / 9.80665 * 1000
```

Range: ±32000 mg for ±32g. The LSM6DSO32 sensitivity is 0.976 mg/LSB, so int16 in milligees provides better resolution than the sensor itself.

### High-G 10-milligee units

The `highg_10mg[3]` field stores ADXL372 acceleration in 10 mg units:

```
highg_10mg = accel_ms2 / 9.80665 * 100
```

Range: ±20000 for ±200g. ADXL372 sensitivity is 100 mg/LSB, so 10 mg units give 10× better resolution than the raw sensor register.

### EKF State Encoding

| Field | Unit | Type | Range | Resolution |
|-------|------|------|-------|------------|
| `ekf_alt_cm` | cm | int32 | ±21,474 km | 1 cm |
| `ekf_vel_cmps` | cm/s | int32 | ±21,474 km/s | 1 cm/s |
| `ekf_abias_mmps2` | mm/s² | int16 | ±32.7 m/s² | 1 mm/s² |
| `ekf_bbias_cm` | cm | int16 | ±327 m | 1 cm |

The altitude and velocity fields use int32 specifically to avoid any clamping. A Mach 1.05 flight peaks at ~312 m/s = 31200 cm/s — well within int32 range. Even km/s velocities fit.

### Quaternion Smallest-Three Encoding

The quaternion `q = [w, x, y, z]` is packed into 3 × int16 using smallest-three:

1. Find the component with the largest absolute value.
2. If it's negative, negate the entire quaternion (unit quaternions: q ≡ -q).
3. Drop the largest component (it can be reconstructed from the other three since |q| = 1).
4. Encode the remaining three as Q14 fixed-point (multiply by 16384).
5. Store the index of the dropped component in the upper 2 bits of `packed[0]`.

**Reconstruction (Python):**

```python
def unpack_quat(packed):
    drop_idx = (packed[0] >> 14) & 0x03
    p = [(packed[0] & 0x3FFF) / 16384.0,
         packed[1] / 16384.0,
         packed[2] / 16384.0]
    # Handle sign extension for negative Q14 values
    for i in range(3):
        if packed[i] if i > 0 else (packed[i] & 0x3FFF) > 8191:
            p[i] -= 2.0 if i > 0 else (p[i] - 1.0 + 1.0)  # sign extend
    sum_sq = p[0]**2 + p[1]**2 + p[2]**2
    dropped = (1.0 - sum_sq)**0.5 if sum_sq < 1.0 else 0.0
    q = [0.0] * 4
    j = 0
    for i in range(4):
        if i == drop_idx:
            q[i] = dropped
        else:
            q[i] = p[j]
            j += 1
    return q
```

Note: The Python decoder in `tools/decode_flight_log.py` must implement proper sign extension for the Q14 packed values. The upper 2 bits of `packed[0]` are the drop index, leaving 14 bits of signed value.

### GPS Coordinates

u-blox standard: degrees × 10^7 as int32. Example:

```
51.5074° N  → gps_lat_deg7 = 515074000
-0.1278° W  → gps_lon_deg7 = -1278000
```

### Continuity Scaling

The 16-bit ADC raw values are scaled to 8-bit by right-shifting 8 bits:

```
cont_scaled = cont_raw >> 8
```

This gives ~20 mV resolution (3.3V / 256 = 12.9 mV per LSB). Sufficient for go/no-go continuity detection (threshold is typically ~1V). For precise resistance measurement, the raw 16-bit values would be needed — these are available via direct ADC read during pre-flight checks, not in the log.

---

## APPENDIX B: Bandwidth and Timing Analysis

### Flash Write Budget

At 250 Hz high-rate logging:

```
HR entries per second:     250
HR bytes per second:       250 × 64 = 16,000
HR pages per second:       16,000 / 256 = 62.5
HR page write time/s:      62.5 × 0.7ms = 43.75ms
HR CPU fraction:           43.75ms / 1000ms = 4.4%

LR entries per second:     10
LR bytes per second:       10 × 64 = 640
LR pages per second:       640 / 256 = 2.5
LR page write time/s:      2.5 × 0.7ms = 1.75ms

Total flash write time:    45.5ms per second = 4.55% CPU
```

### Double Buffer Fill Timing

```
HR buffer: 64 entries / 250 Hz = 256ms to fill
HR flush:  16 pages × 0.7ms = 11.2ms to write + 45ms sector erase = 56.2ms worst case
Margin:    256ms - 56.2ms = 199.8ms — comfortable

LR buffer: 64 entries / 10 Hz = 6,400ms (6.4 seconds) to fill
LR flush:  Same 56.2ms worst case
Margin:    6,344ms — massive
```

### Ring Buffer Commit at Launch

```
HR ring: 1250 entries × 64 bytes = 80,000 bytes
Pages:   80,000 / 256 = 313 pages
Sectors: ceil(80,000 / 4096) = 20 sectors
Erase:   20 × 45ms = 900ms (worst case, can overlap with writes)
Write:   313 × 0.7ms = 219ms
Total:   ~1.1 seconds (blocking)

LR ring: 100 entries × 64 bytes = 6,400 bytes
Pages:   25 pages
Sectors: 2 sectors
Total:   ~110ms

Combined: ~1.2 seconds from launch detection to normal logging
```

This is acceptable. The ring buffer contains the first 5 seconds of pad data + early boost, and the commit happens while the rocket is accelerating. The first ~1.2s of post-launch data is buffered in the double buffers during commit.

---

## APPENDIX C: File Tree

```
Software/
├── App/
│   ├── flight/                 # Flight data logging
│   │   ├── flight_log.h             # Entry types, flash layout, core API
│   │   ├── flight_log.c             # Ring buffer, double buffer, flash writer
│   │   ├── flight_log_helpers.h     # Snapshot structs, populate functions
│   │   ├── flight_log_helpers.c     # Entry population from sensor state
│   │   └── flight_readout.h/c       # USB CDC readout protocol
│   └── test/
│       ├── test_flight_log.c        # Ring buffer, double buffer, flash tests
│       ├── test_flight_log_helpers.c # Populate function tests
│       └── mock_w25q512jv.c         # Flash mock for host testing
├── tools/
│   └── decode_flight_log.py         # Python binary decoder → CSV
└── Makefile                         # UPDATE: add new .c files
```

---

## APPENDIX D: Pending Design Decisions

These decisions were identified during architecture review and deferred. They do not block implementation but should be resolved before flight:

1. **Baro pressure encoding:** Currently stores compensated Pascals (uint32). Alternative: store raw 24-bit MS5611 ADC values and PROM coefficients in the summary log, enabling post-flight recomputation. Compensated Pascals is simpler and used in the current design.

2. **Continuity resolution:** uint8 (~20mV) is sufficient for go/no-go. If precise resistance measurement is needed for post-flight analysis, bump to uint16 (steals 4 bytes from LR reserved).

3. **Mag temperature:** The MMC5983MA datasheet does not expose a standard die temperature register. The `mag_temp_c100` field uses a sentinel value (0x7FFF = 327.67°C) when unavailable. Verify if the MMC5983MA has an undocumented temp register or if this field should be repurposed.

4. **Radio counters:** Currently wrapping uint8. Alternative: delta encoding per-entry (bytes since last log tick). Wrapping is simpler to decode and is used in the current design.

5. **Timestamp wrapping:** DWT-based µs timestamps wrap every ~9.9s. Consider using TIM2/TIM5 (32-bit, 1 MHz) for a 71.6-minute non-wrapping counter. The Python decoder must handle wraps regardless.
