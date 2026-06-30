# Flight-Firmware Static-Analysis Optimization — Summary Report

**Date:** 2026-06-09 · **Branch:** `optimization-2026-06-09` · **Scope:** flight-compiled `App/` modules only (the sources linked into `Casper2_Flight.elf`). Middlewares, CubeMX-generated drivers, FATFS, the ground-station target, MATLAB, and the non-compiled `hse_test` bring-up tool were **out of scope** and left untouched.

**Method:** A Sonnet sub-agent **swarm** (22 agents) fanned out across 11 module shards to find dead/unused/duplicated code; every proposed removal was then **adversarially re-verified** by a second agent (whole-tree grep for direct calls, function pointers, X-macros, weak aliases, `#ifdef` paths, linker refs). An Opus orchestrator vetted the findings, rejected the unsafe ones, and a second Sonnet swarm + manual passes applied the changes. A purpose-built **host-gcc test harness** (`Software/test/`) locks the behaviour of every pure module that was refactored.

---

## Headline results

| Metric | Before | After | Δ |
|---|---|---|---|
| FLIGHT `.text` | 99 472 B | 99 232 B | **−240 B** |
| FLIGHT `.bss` | 117 864 B | 117 768 B | **−96 B** |
| FLIGHT compiler warnings | 1 | **0** | −1 |
| GROUND compiler warnings | 6 | **5** | −1 (0 new) |
| Source lines (App + main.c + Makefile) | — | — | **+46 / −872 (net −826)** |
| Files touched / fully deleted | — | 50 / **2** | — |
| Dead functions removed | — | **~32** | — |
| Dead vars / fields / macros removed | — | **~24** | — |
| Host regression tests added | 0 | **23 (all pass)** | — |

Both `Casper2_Flight.elf` and `Casper2_Ground.elf` build **clean**; all 23 host golden/characterization tests pass **identically before and after** (behaviour preserved).

---

## What changed, where, and why

### 1. Deleted an entire dead module — `casper_gyro_int` (−189 LOC)
`App/nav/casper_gyro_int.c/.h` deleted; removed from `Makefile` `C_SOURCES` and its `#include` in `flight_loop.c`. It was the old RK4 gyro integrator, fully superseded by `casper_attitude` — compiled into the ELF but **never called**. Its `quat_derivative`/RK4 logic also duplicated `casper_attitude.c`.

### 2. De-duplicated copy-pasted helpers
- **Little-endian serializers** `put_le16/24/32` were copy-pasted in three TUs (`tlm_manager.c`, `radio_manager.c`, `self_test.c` — one literally commented *"same as tlm_manager.c"*). Added `put_le24` to the shared `App/util/endian.h` and replaced all three local copies with `#include "endian.h"`. `cfg_manager.c` likewise had hand-rolled LE32 → now uses `put_le32`.
- **`mmc5983ma_init` / `_init_oneshot`** shared ~70 identical lines → extracted a `mmc5983ma_hw_init()` helper (register writes/delays/return semantics preserved exactly).
- **`logger_sanity` `emit()`** hand-rolled a `strlen` loop → now calls `strlen()`.

### 3. Removed dead functions (~32) — confirmed zero call sites
- **Nav:** `casper_quat_from_accel`, `casper_att_get_quaternion`, `casper_att_get_euler`, `casper_att_add_gate` + the always-false `mag_gated()` ignition-gate no-op.
- **Drivers:** `ms5611_read_raw`, `ms5611_get_pressure`, `lsm6dso32_read_raw`, `lsm6dso32_write_reg_ext`, `adxl372_read`, `adxl372_read_reg_ext`, `mmc5983ma_read_temp`, `max_m10m_send_valset_persist`, `w25q512jv_erase_chip`, `w25q512jv_test`.
- **Radio/telemetry/command/fsm:** `sx1276_set_payload_length`, `radio_send_gps` + `build_gps_packet` + unreachable GPS priority branch, `radio_is_active`, `crc32_hw_validate`, `tlm_get_seq`, `cfg_get_active`, `cac_test_mode_active`, `cac_test_mode_remaining_ms`, `flight_fsm_reset`, `check_antenna_up` wrapper, 5 unused `flight_logger` accessors, `diag_probe_init_dwt`. `self_test_run_all` was downgraded from public API to `static`.

### 4. Removed dead state, branches & includes
Unused struct fields (`casper_ekf_t.dt`, `casper_attitude_t.bias_sum/bias_count/gates[]/num_gates`, `casper_att_config_t.launch_accel_g`, `flight_logger_t.erase_pool`), unreachable enum `QSPI_POLLING` + its ISR branches, write-only flags/counters (`g_radio_dio3_flag`, `s_tx_error_count`, `s_gps_pending`, `diag_ned_z`, `last_baro_alt`), a preprocessor-unreachable `#if TEST_MODE != 2` block nested inside `#if TEST_MODE == 2`, a `#include <stdlib.h>` now guarded to the only build that uses `atoi`, the redundant `casper_gyro_int.h` include, a redundant double-bounds check in `cac_tick`, and unused macros (`PYRO_DEFAULT_FIRE_MS`, `FLASH_INDEX_END/SUMMARY_END`, `RADIO_MAX_CONSEC_CRC_ERRORS/MAX_TX_RETRIES`, `MAG_CAL_EXPECTED_MAG`).

---

## Deliberately NOT changed (safety vetoes by the Opus reviewer)

- **`flight_summary_t` fields** (`pad_tilt_deg`, `pyro_fire_tick[]`, `pyro_cont_at_launch[]`) — the struct is written to flash byte-for-byte with a CRC over `sizeof-4`; removing fields would corrupt the **on-flash wire format**. Kept as reserved.
- **`#ifdef HIL_MODE` code** in drivers/fsm/pyro — dead in the flight build *by design*; needed by `make hil`.
- **Calibration entry points** (`mag_cal_*`, `mag_val_*`, `temp_cal_*`) — only "dead" because they are reached via `#ifdef MAG_CAL`/`MAG_VAL`/`GYRO_TEMP_CAL` build variants. Left intact.
- **Protocol constants** (unused `NACK_ERR_*`, `FC_EVT_ORIGIN`, ground-only message types) and the **SX1276 register map** — wire/ABI surface; flagged for a future protocol pass rather than deleted.
- **`hamming_decode`** — kept as the matching SECDED decoder of a coherent ECC codec (and now test-covered).
- **EKF GPS update stubs** — the adversarial verifier *rejected* the "dead" claim: they are live no-op stubs actually called in the flight loop.

---

## Test harness added (`Software/test/`)
Host-compilable (MSYS2 gcc, run via `test/run.ps1`) golden/characterization suites pinning exact behaviour through the refactor: **cobs** (4), **hamming+CRC16** (4), **quat_pack** (2), **status_pack** (3), **casper_quat** (6), **endian incl. new put_le24** (3), and a full **casper_attitude** flight-path lock (pad-init → launch → 2000 RK4 steps with 10 Hz mag correction, exercising the removed gate path) — **23 tests, all green** on both the original and the optimized code.
