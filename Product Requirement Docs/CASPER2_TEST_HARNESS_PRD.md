# PRD: C.A.S.P.E.R.-2 Firmware Test Harness

**Rev 2.0 — February 2026**
**Author:** Ashwin Vishwanath
**Status:** Draft

---

## 1. Purpose

Define the requirements for a comprehensive, host-side unit and integration test harness for the C.A.S.P.E.R.-2 flight computer firmware. The harness runs on a development PC (not target hardware), catches regressions during development, validates adherence to project specifications and JSF AV C++ coding standards, and integrates into Claude Code's plan-mode verification stage.

### 1.1 Agent Discovery Principles

This PRD is consumed by a Claude Code agent working inside the C.A.S.P.E.R.-2 repository. Several sections contain **agent tasks** — things the agent must discover from the codebase rather than having prescribed. The agent must:

- **Search the repo for referenced documents** (msgset `.docx`, spec `.md` files, FSM source) rather than assuming file paths or contents. File names and locations may change.
- **Read CSV headers at runtime** to discover column names and data formats. Do not hardcode column indices.
- **Cross-reference code against spec documents** found in the repo. When the PRD says "matches spec," the agent finds the spec and extracts the expected value.
- **Document what it finds.** When discovering FSM states, CSV formats, or packet layouts, write a summary to `Tests/` (e.g., `Tests/CSVs/README.md`, `Tests/FSM_TRANSITIONS.md`) so the next developer (or agent) doesn't repeat the discovery.
- **Use Raven data as ground truth up to apogee + 5 seconds only.** All truth comparisons, error metrics, and assertions apply exclusively within this window. Data after apogee + 5s is ignored entirely.

### 1.2 Success Criteria

- All Tier 1 (pure math) modules achieve 100% function coverage and ≥90% branch coverage.
- All Tier 2 (protocol/FSM) modules achieve ≥80% function coverage.
- Tier 3 (flight loop integration) tests pass coarse end-to-end validation.
- Real flight data regression tests (CSV-driven) produce altitude/velocity outputs within defined tolerance of truth data.
- Spec compliance tests detect byte-layout, constant, and naming convention violations automatically.
- Full test suite builds and runs in under 60 seconds via `cmake --build . && ctest`.
- Claude Code invokes the suite during plan-mode verification. A `/test` slash command is available for manual runs.

---

## 2. Architecture

### 2.1 Directory Layout

```
Tests/
├── CMakeLists.txt              ← Top-level test build (standalone, not STM32)
├── unity/                      ← Unity test framework (vendored)
│   ├── unity.c
│   ├── unity.h
│   └── unity_internals.h
├── stubs/                      ← HAL + peripheral stubs
│   ├── stm32h7xx_hal.h         ← Minimal HAL type definitions
│   ├── stm32h7xx_hal_spi.h
│   ├── stm32h7xx_hal_i2c.h
│   ├── stm32h7xx_hal_adc.h
│   ├── stm32h7xx_hal_gpio.h
│   ├── stm32h7xx_hal_crc.h
│   ├── stm32h7xx_hal_tim.h
│   ├── stm32h7xx_hal_qspi.h
│   ├── arm_math_stub.h         ← CMSIS-DSP matrix stubs (real math, fake types)
│   └── stub_helpers.c          ← Shared spy/fake infrastructure
├── mocks/                      ← Configurable mock implementations
│   ├── mock_spi.c/h            ← SPI transaction recorder + canned responses
│   ├── mock_i2c.c/h
│   ├── mock_adc.c/h
│   ├── mock_gpio.c/h
│   └── mock_tick.c/h           ← HAL_GetTick() fake with manual advance
├── tier1_unit/                 ← Pure math unit tests
│   ├── test_casper_quat.c
│   ├── test_casper_ekf.c
│   ├── test_casper_attitude.c
│   ├── test_casper_gyro_int.c
│   ├── test_cobs.c
│   ├── test_crc32.c
│   ├── test_status_pack.c
│   ├── test_ms5611_math.c
│   └── test_tlm_pack.c
├── tier2_protocol/             ← Protocol and FSM tests
│   ├── test_cac_handler.c
│   ├── test_cmd_router.c
│   ├── test_pyro_manager.c
│   ├── test_flight_fsm.c
│   └── test_telemetry_framing.c
├── tier3_integration/          ← Coarse end-to-end tests
│   ├── test_flight_loop.c
│   └── test_nav_pipeline.c
├── regression/                 ← CSV-driven flight data replay
│   ├── test_ekf_regression.c
│   ├── test_attitude_regression.c
│   ├── csv_loader.c/h          ← Lightweight CSV parser
│   └── regression_helpers.c/h
├── compliance/                 ← Spec + standards adherence
│   ├── test_interface_spec.c   ← Packet sizes, msg_ids, byte offsets
│   ├── test_ekf_spec.c         ← EKF constants, dt, covariance P0
│   ├── test_sensor_spec.c      ← Register addresses, conversion factors
│   ├── test_pyro_spec.c        ← Channel map, safety interlock constants
│   ├── test_naming_convention.c← [Source]_[Category]_[Field] pattern check
│   └── test_jsf_static.c       ← Compile-time JSF AV subset checks
├── CSVs/                       ← Truth data (user-supplied)
│   ├── filtereddatafile.csv
│   └── raven_data.csv
└── scripts/
    ├── run_tests.sh            ← Build + run + report (slash command target)
    └── coverage.sh             ← gcov/lcov coverage report
```

### 2.2 Build System

The test harness uses a **standalone CMake build** that compiles firmware source files against host GCC with HAL stubs, entirely independent of the STM32CubeIDE project. The STM32 Makefile is not modified.

```cmake
# Tests/CMakeLists.txt (simplified)
cmake_minimum_required(VERSION 3.16)
project(casper2_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -DUNIT_TEST")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DARM_MATH_CM7 -fprofile-arcs -ftest-coverage")

# Unity
add_library(unity STATIC unity/unity.c)

# Source under test — pulled from ../Software/ by relative path
set(FIRMWARE_ROOT ${CMAKE_SOURCE_DIR}/../Software)

# Stub include paths replace real HAL headers
include_directories(
    ${CMAKE_SOURCE_DIR}/stubs
    ${CMAKE_SOURCE_DIR}/mocks
    ${CMAKE_SOURCE_DIR}/unity
    ${FIRMWARE_ROOT}/App/nav
    ${FIRMWARE_ROOT}/App/command
    ${FIRMWARE_ROOT}/App/pyro
    ${FIRMWARE_ROOT}/App/pack
    ${FIRMWARE_ROOT}/App/telemetry
    ${FIRMWARE_ROOT}/App/fsm
    ${FIRMWARE_ROOT}/App/flight
    ${FIRMWARE_ROOT}/App/drivers
    ${FIRMWARE_ROOT}/Core/Inc
)

# Example test target
add_executable(test_casper_ekf
    tier1_unit/test_casper_ekf.c
    ${FIRMWARE_ROOT}/App/nav/casper_ekf.c
    ${FIRMWARE_ROOT}/App/nav/casper_quat.c
    stubs/arm_math_stub.c
)
target_link_libraries(test_casper_ekf unity m)
add_test(NAME ekf COMMAND test_casper_ekf)

enable_testing()
```

**Build and run:**

```bash
cd Tests && mkdir -p build && cd build
cmake .. && cmake --build . -j$(nproc) && ctest --output-on-failure
```

### 2.3 HAL Stub Strategy

Stubs provide type definitions and function signatures that satisfy the compiler without any STM32 hardware. The approach is layered:

**Level 0 — Type stubs** (`stubs/stm32h7xx_hal.h`): Typedef `SPI_HandleTypeDef`, `GPIO_TypeDef`, etc. as opaque structs. Define `HAL_OK`, `HAL_ERROR`, `GPIO_PIN_SET`, `GPIO_PIN_RESET`, etc.

**Level 1 — Spy mocks** (`mocks/mock_spi.c`): Record all SPI transactions (register address, tx data, rx data) into a log array. Test code pre-loads canned responses and asserts expected register writes after the function under test returns.

**Level 2 — Tick fake** (`mocks/mock_tick.c`): `HAL_GetTick()` returns a manually-advanced counter so timeout and state machine tests run deterministically without real time.

**CMSIS-DSP stubs** (`stubs/arm_math_stub.c`): Implement `arm_mat_init_f32`, `arm_mat_mult_f32`, `arm_mat_add_f32` using plain C loops. These produce correct math results but without SIMD optimization — sufficient for validating EKF numerics on host.

### 2.4 Claude Code Integration

**Plan-mode verification:** Claude Code's verification step executes the test suite as a build check. The `CLAUDE.md` or equivalent config specifies:

```
verification:
  build: "cd Tests && mkdir -p build && cd build && cmake .. && cmake --build . -j$(nproc)"
  test: "cd Tests/build && ctest --output-on-failure"
```

If any test fails, Claude Code's verification fails and it must address the issue before proceeding.

**Slash command:** `/test` triggers `Tests/scripts/run_tests.sh`, which performs:

1. Clean build of the test harness via CMake.
2. Run all tests via `ctest`.
3. Print pass/fail summary with failing test details.
4. Optionally generate gcov coverage report.

---

## 3. Tier 1 — Pure Math Unit Tests

These tests have zero HAL dependencies and validate correctness of all mathematical and encoding functions.

### 3.1 `test_casper_quat.c`

**Source under test:** `App/nav/casper_quat.c`

| Test Case | Input | Expected Output | Tolerance |
|-----------|-------|-----------------|-----------|
| Identity multiply | q_id × q_id | q_id = [1,0,0,0] | 1e-7 |
| 90° Z rotation multiply | q_z90 × q_z90 | q_z180 | 1e-6 |
| Normalize unit quat | [1,0,0,0] | [1,0,0,0] unchanged | 1e-7 |
| Normalize non-unit | [2,0,0,0] | [1,0,0,0] | 1e-7 |
| Normalize near-zero | [1e-20,0,0,0] | Does not produce NaN/Inf | — |
| Rotmat identity | q_id → R | I₃ | 1e-7 |
| Rotmat 90° pitch | q_pitch90 → R | Known rotation matrix | 1e-6 |
| From accel level | [0,0,9.81] | roll=0, pitch=0 | 0.1° |
| From accel tilted | [0,4.9,8.5] | pitch≈0, roll≈30° | 0.5° |
| Euler round-trip | euler → quat → euler | Original angles ±0.01° | 0.01° |
| Euler gimbal lock | pitch=89.9° | No NaN, bounded output | — |
| Hamilton convention | i×j = k, j×k = i, k×i = j | Verify sign convention | 1e-7 |

**Spec compliance checks (EKF_SPEC §5):**
- Convention is `q[4] = [w, x, y, z]`, scalar-first.
- `casper_quat_to_rotmat` produces body-to-NED rotation matrix (row-major).

### 3.2 `test_casper_ekf.c`

**Source under test:** `App/nav/casper_ekf.c`

**Init tests:**
- After `casper_ekf_init()`, state vector `x[0..3]` = 0.
- P₀ diagonal matches EKF_SPEC §2.10: `[0.1, 0.001, 0.025, 0.75]`.
- `baro_ref` = 0, `baro_gated` = false.

**Predict tests:**
- Single predict step with zero accel (level, stationary): x unchanged, P grows by Q.
- Single predict step with known vertical accel (e.g., 10 m/s² up for 0.0012s): velocity increases by `10 * 0.0012`, altitude increases by `0.5 * 10 * 0.0012²`.
- Attitude quaternion properly rotates accel from body to NED before integration — test with a 45° tilt angle and verify only the NED-vertical component integrates into the state.
- `EKF_DT` = 0.0012s (833 Hz) — hardcoded constant matches EKF_SPEC §2.11.

**Update tests (baro):**
- Baro update with zero innovation: state unchanged, P reduced.
- Baro update with 10m innovation: altitude corrects toward measurement.
- Joseph form preserves P symmetry after 1000 sequential updates (check `P[i][j] == P[j][i]`).
- P diagonal remains positive after 10000 predict-update cycles (no negative variance).
- Innovation gate rejects measurement with |innovation| > 5σ (EKF_SPEC §2.5).
- `isfinite()` guard rejects NaN baro input (EKF_SPEC §2.6).
- Baro bias floor of 1e-4 maintained (EKF_SPEC §2.7).

**Mach gating tests (EKF_SPEC §2.8):**
- Below Mach 0.35: baro updates accepted.
- Above Mach 0.40: baro updates gated (dead-reckoning only).
- Between 0.35 and 0.40: hysteresis — state depends on direction of crossing.
- Speed of sound computation: `sqrt(1.4 * 287.058 * (288.15 - 0.0065 * alt))`.

**GPS stubs (EKF_SPEC §2.9):**
- GPS altitude update stub exists and compiles.
- GPS velocity update stub exists and compiles.
- H matrices match spec: `[1,0,0,0]` and `[0,1,0,0]`.
- R values: 100.0 m² (alt), 1.0 (m/s)² (vel).

### 3.3 `test_casper_attitude.c`

**Source under test:** `App/nav/casper_attitude.c`

**Static initialization (EKF_SPEC §3.2, Phase 1):**
- With level accel [0,0,9.81] and known mag vector: initial quaternion has roll≈0, pitch≈0, yaw matches tilt-compensated heading.
- Fallback: if < 500 mag samples after 10s timeout, yaw=0 and `mag_available=false`.
- `m_ref_ned` computed as `R(q) * mag_avg` and stored.

**Pad phase (Phase 2):**
- Gyro LPF cutoff = 50 Hz (EKF_SPEC §3.1): feed a 100 Hz signal, verify ≥3 dB attenuation. Feed a 10 Hz signal, verify <0.5 dB attenuation.
- Gyro bias: running average converges to injected constant bias within 5 seconds of samples.
- Gravity correction: with a static 5° tilt and zero gyro, quaternion converges toward gravity alignment.
- Mag correction on pad: with a known heading offset, quaternion converges toward correct yaw.

**Flight phase (Phase 3):**
- Launch detection at 3g threshold (EKF_SPEC §3.1 `launch_accel_g`).
- After launch: gravity correction disabled, mag correction gain reduces to `Kp_mag_flight`.
- RK4 propagation: integrate a constant 1 rad/s rotation for 1 second, verify final angle ≈ 57.3° (1 rad).

**Ignition gating (EKF_SPEC §3.5):**
- During configured gate window: mag corrections fully suppressed.
- Outside gate window: mag corrections active.
- Up to 4 gates configurable.

**Uncertainty tracking:**
- ARW growth during dead-reckoning (no corrections).
- Uncertainty reduction when mag corrections active.

### 3.4 `test_cobs.c`

| Test Case | Raw Payload | Expected Encoded |
|-----------|-------------|-----------------|
| Empty | `[]` | `[0x01, 0x00]` |
| No zeros | `[0x01, 0x02, 0x03]` | `[0x04, 0x01, 0x02, 0x03, 0x00]` |
| Single zero | `[0x00]` | `[0x01, 0x01, 0x00]` |
| Leading zero | `[0x00, 0x01]` | `[0x01, 0x02, 0x01, 0x00]` |
| 254 non-zero bytes | `[0x01]*254` | `[0xFF, 0x01*254, 0x00]` |
| 255 non-zero bytes | `[0x01]*255` | `[0xFF, 0x01*254, 0x02, 0x01, 0x00]` |
| Round-trip fuzz | Random 1-254 byte payloads | decode(encode(x)) == x |

**INTERFACE_SPEC §2.1 compliance:**
- Frame delimiter = 0x00.
- 0x00 never appears inside encoded payload.
- Max block length = 254 data bytes + 1 overhead byte.
- Back-to-back 0x00 delimiters silently ignored.
- Malformed frames silently discarded (no crash, no assertion).

### 3.5 `test_crc32.c`

**INTERFACE_SPEC §3 compliance:**

| Test | Input | Expected CRC-32 |
|------|-------|-----------------|
| Standard test vector | `"123456789"` (9 bytes) | `0xCBF43926` |
| Empty input | `""` (0 bytes) | `0x00000000` or per standard |
| Single byte 0x00 | `[0x00]` | Known reference |
| All 0xFF | `[0xFF]*256` | Known reference |

- Polynomial: `0x04C11DB7` (normal), `0xEDB88320` (reflected).
- Init = `0xFFFFFFFF`, final XOR = `0xFFFFFFFF`.
- Input/output reflected.
- Software implementation matches hardware CRC unit output (cross-validate against STM32 CRC reference values).

### 3.6 `test_status_pack.c`

**PYRO_SPEC §9 compliance:**

- Byte 0: CNT1-4 in bits [0:3], ARM1-4 in bits [4:7].
- Byte 1: FSM state in bits [4:7], FIRED in bit 3, ERROR in bit 2, reserved bits [0:1] = 0.
- Decode the spec example: `[0x15, 0x68]` → CH1=cont+armed, CH3=cont, FSM=APOGEE, FIRED=yes, ERROR=no.
- All 16 FSM states (0x0-0xB) encode/decode correctly.
- All 16 arm/continuity combinations per channel produce correct bits.

### 3.7 `test_ms5611_math.c`

**Source under test:** MS5611 calibration computation (extracted from `App/drivers/ms5611.c`)

- Known PROM coefficients + known D1/D2 raw values → expected pressure/temperature (use datasheet application note examples).
- Negative pressure clamped to ≥ 0.01 hPa (EKF_SPEC §2.6 defense).
- `ms5611_get_altitude()` with sea-level pressure 1013.25 hPa at known pressure → expected altitude (hypsometric formula).
- Temperature compensation: second-order correction below 20°C and below -15°C.

### 3.8 `test_tlm_pack.c`

**Agent task:** Before writing these tests, the agent must:
1. Search the repo for the telemetry message set document. It is a `.docx` file likely in the repo root — search for files containing "msgset", "message set", or "telemetry" in the filename.
2. Read the msgset doc to extract every packet type, its `msg_id`, byte count, field offsets, and encoding rules.
3. Find `tlm_types.h` (or equivalent) in the firmware source — this contains the `msg_id` constants, packed struct definitions, and event type codes.
4. Cross-reference the msgset doc against `tlm_types.h` to build the test assertions.

**Test requirements (once packet definitions are known):**

For every packet type defined in the telemetry msgset:
- Packed struct size matches documented byte count (if structs exist; if packets are built field-by-field, validate the total byte count of the builder output).
- `msg_id` at byte offset 0 matches defined constant.
- CRC-32 field position = last 4 bytes.
- Field byte offsets match msgset doc tables (spot-check critical fields: altitude, velocity, quaternion, status bitmap, etc.).
- Little-endian encoding verified for multi-byte fields.
- Quaternion smallest-three encoding/decoding round-trips correctly (if implemented).

---

## 4. Tier 2 — Protocol and FSM Tests

These tests require HAL stubs/mocks for timing, GPIO, and communication interfaces.

### 4.1 `test_cac_handler.c`

**Source under test:** `App/command/cac_handler.c`

**PYRO_SPEC §1-§2 compliance:**

**Happy path — ARM:**
1. Receive CMD_ARM (0x80) with valid magic (0xCA 0x5A), correct channel complement, valid CRC-32.
2. Assert ACK_ARM sent.
3. Receive CONFIRM within 5s.
4. Assert `pyro_mgr_set_arm()` called with correct channel.
5. Assert FC_EVT_ARM event emitted.

**Happy path — FIRE:**
1. Channel pre-armed, test mode active, continuity present.
2. Receive CMD_FIRE (0x81) with valid magic, channel complement, duration complement, valid CRC-32.
3. Assert ACK_FIRE sent.
4. Receive CONFIRM within 5s.
5. Assert `pyro_mgr_fire()` called with correct channel and duration.

**Rejection tests:**
- Wrong magic bytes → silent discard (no ACK, no NACK).
- Wrong channel complement → silent discard.
- Wrong duration complement → silent discard.
- Bad CRC-32 → silent discard.
- Channel out of range (>3) → NACK_ERR_BAD_STATE.
- No continuity → NACK_ERR_NO_CONTINUITY.
- Not in test mode (for FIRE) → NACK_ERR_NO_TESTMODE.
- Not armed (for FIRE) → NACK_ERR_NOT_ARMED.

**Timeout tests:**
- No CONFIRM within 5s → auto-cancel, pending cleared.
- ABORT received → cancel immediately.

**Nonce tests:**
- Same nonce replayed → rejected.
- Sequential nonces accepted.

### 4.2 `test_cmd_router.c`

**Source under test:** `App/command/cmd_router.c`

- Feed COBS-encoded frame byte-by-byte into `cmd_router_process()`.
- Assert correct msg_id dispatch (ARM → cac_handler, FIRE → cac_handler, HANDSHAKE → handshake handler).
- Partial frame + delimiter → frame dispatched.
- Corrupted frame (bad COBS) → silently discarded, no crash.
- Back-to-back frames → both dispatched.
- Ring buffer wraparound: feed more bytes than CDC_RING_SIZE (256), verify no corruption.
- Ring buffer full: verify graceful handling (drop oldest or reject new).

### 4.3 `test_pyro_manager.c`

**Source under test:** `App/pyro/pyro_manager.c`

**PYRO_SPEC §10 compliance (safety interlocks):**

- Cannot arm without continuity → returns error.
- Cannot fire without arm → returns error.
- Cannot fire without test mode (ground) or correct FSM state (flight) → returns error.
- Fire re-checks arm + continuity at execution time (double-check gate).
- Test mode fire limited to configured max duration.
- Auto-disarm after fire completes.

### 4.4 `test_flight_fsm.c`

**Source under test:** `App/fsm/flight_fsm.c`

**Agent task:** Before writing tests, the agent must:
1. Read `flight_fsm.c` and `flight_fsm.h` to extract the complete state enum, transition logic, and transition conditions.
2. Search for the telemetry message set document in the repo (likely `.docx` or `.md` in the repo root or `Software/` directory). The FSM states and their 4-bit codes are documented there.
3. If the FSM transition table is not fully documented in either source, derive it from the code and add a `FSM_TRANSITIONS.md` to `Tests/` documenting what was found.

**Test requirements (once states and transitions are known):**

- Every valid state transition: drive the FSM with synthetic inputs that trigger each transition. Assert the new state and that `FC_EVT_STATE` event is emitted.
- Every invalid transition: attempt transitions that should not be allowed (e.g., PAD → APOGEE direct). Assert state does not change.
- Sim flight profile: call `flight_fsm_sim_start()`, advance time through all waypoints, verify the FSM walks through the expected state sequence.
- Sim stop: verify `flight_fsm_sim_stop()` returns to PAD.
- `flight_fsm_force_state()`: verify it sets the state and emits an event.
- Apogee event: verify `FC_EVT_APOGEE` is emitted additionally when entering the apogee state.
- Mission timer: verify `flight_fsm_get_time_s()` returns 0 before launch, then monotonically increases.

---

## 5. Tier 3 — Integration Tests (Coarse)

These tests validate the data flow pipeline without hardware. They use mocked sensor reads and verify that the navigation stack produces reasonable outputs.

### 5.1 `test_flight_loop.c`

- Mock all sensors to return static pad data for 30 seconds (calibration period).
- Verify baro_ref is set at end of calibration.
- Verify EKF state = [0, 0, ~0, ~0] after calibration.
- Verify attitude quaternion converges to known orientation during pad phase.
- Switch to synthetic boost profile (50g for 2s): verify EKF altitude and velocity increase.
- Verify Mach gating activates when velocity exceeds threshold.

### 5.2 `test_nav_pipeline.c`

- Feed 100 identical IMU samples through attitude + EKF predict. Verify no drift exceeding tolerance.
- Feed known constant rotation through attitude estimator, then verify EKF uses rotated accel.
- Sensor-to-body frame remap: inject known sensor-frame values, verify body-frame output matches ORIENTATION_SPEC §2.1 cyclic permutation `body[i] = sensor[(i+2) % 3]`.
- Magnetometer negation: verify all three axes negated before calibration (ORIENTATION_SPEC §2.2).

---

## 6. Regression Tests (CSV-Driven)

### 6.1 Data Inventory

Four CSV files are provided in `Tests/CSVs/`. Their roles and contents are fixed and documented here.

#### 6.1.1 `raven_data.csv` — **PRIMARY TRUTH** (Standard Rate)

| Property | Value |
|----------|-------|
| Source | Featherweight Raven altimeter, same flight |
| Rate | 50 Hz (20 ms spacing) |
| Rows | ~10,255 |
| Time column | `FlightTime` (seconds, 0 = liftoff, negative = pre-launch) |
| Time range | -1.9s to +203.2s |
| Columns | 101 (many are pyro channel event flags — ignore) |

**Truth columns:**

| Column | Units | Description |
|--------|-------|-------------|
| `FlightTime` | s | Time relative to liftoff (0 = ignition) |
| `Velocity_Up` | ft/s | Vertical velocity (up positive) |
| `Velocity_DR` | ft/s | Downrange velocity |
| `Velocity_CR` | ft/s | Crossrange velocity |
| `Baro_Altitude_AGL` | ft | Barometric altitude above ground level |
| `Inertial_Altitude` | ft | Inertial nav altitude (NOTE: wraps/overflows after ~143s — treat with caution post-apogee) |
| `Tilt_Angle_(deg)` | deg | Tilt from vertical (0 = vertical, 90 = horizontal) |
| `Future_Angle_(deg)` | deg | Predicted future tilt angle |
| `Roll_Angle_(deg)` | deg | Roll about thrust axis |

**Event flags (binary 0/1):**

| Column | Meaning |
|--------|---------|
| `Liftoff` | 1 at and after liftoff (row 95, t=0s) |
| `Burnout_Coast` | 1 at and after motor burnout (t=4.98s) |
| `Apogee` | 1 at and after apogee detection (t=33.76s) |

**Flight profile from Raven truth:**

| Event | FlightTime | Velocity_Up | Altitude (ft/m) |
|-------|-----------|-------------|-----------------|
| Liftoff | 0.00s | — | 0 |
| Burnout | 4.98s | 1178 fps | — |
| Peak velocity | 4.58s | 1183 fps (360 m/s, Mach 1.05) | — |
| Apogee (vel=0) | 32.84s | 0 fps | 16905 ft (5153 m) |
| Apogee flag | 33.76s | -28 fps | 16892 ft (5149 m) |
| **Cutoff (apogee+5s)** | **38.76s** | — | — |

**UNIT CONVERSIONS REQUIRED:** All Raven altitude/velocity data is in Imperial units. The test code must convert to SI: `meters = feet × 0.3048`, `m/s = fps × 0.3048`.

#### 6.1.2 `raven_HR_data.csv` — **TRUTH** (High Rate, 6DOF IMU + Quaternion)

| Property | Value |
|----------|-------|
| Source | Featherweight Raven altimeter high-rate logger |
| Rate | 500 Hz (2 ms spacing) |
| Rows | ~102,640 |
| Time column | `Flight_Time_(s)` (seconds, 0 = liftoff) |
| Time range | -2.024s to +203.254s |

**Truth columns:**

| Column | Units | Description |
|--------|-------|-------------|
| `Flight_Time_(s)` | s | Time relative to liftoff |
| `Accel_X` | g | Acceleration along thrust axis (≈-1.0g on pad) |
| `Accel_Y` | g | Lateral acceleration |
| `Accel_Z` | g | Lateral acceleration |
| `Gyro_X` | deg/s | Angular rate about thrust axis (roll) |
| `Gyro_Y` | deg/s | Angular rate (pitch) |
| `Gyro_Z` | deg/s | Angular rate (yaw) |
| `Quat_1` | — | Quaternion w (scalar, ≈1.0 on pad) |
| `Quat_2` | — | Quaternion x |
| `Quat_3` | — | Quaternion y |
| `Quat_4` | — | Quaternion z |

**Raven body frame:** `+X = thrust axis` (up on pad). Gravity reads as `Accel_X ≈ -1.0g` on pad. Pad quaternion is identity `[1, 0, 0, 0]` — Raven body frame is aligned with its nav frame when vertical.

**FRAME REMAP REQUIRED:** Raven's body frame (+X = thrust) differs from CASPER-2's body frame (+Z = thrust per ORIENTATION_SPEC). Attitude comparison tests must apply a rotation between frames, or compare tilt magnitude / Euler angles rather than raw quaternions.

#### 6.1.3 `RawDataFile.csv` — **SENSOR INPUT** (Old Flight Computer)

| Property | Value |
|----------|-------|
| Source | Previous flight computer on same rocket, same flight |
| Rate | ~44 Hz (22.8 ms avg spacing) |
| Rows | 19,346 |
| Time column | `time` (microseconds, absolute — NOT relative to liftoff) |

**Sensor columns:**

| Column | Units | Description |
|--------|-------|-------------|
| `time` | µs | Absolute timestamp (liftoff ≈ 22959731 + 813.79×10⁶) |
| `ax`, `ay`, `az` | m/s² | Accelerometer (az ≈ -9.5 on pad → -Z = gravity) |
| `gx`, `gy`, `gz` | rad/s | Gyroscope |
| `mx`, `my`, `mz` | µT | Magnetometer |
| `baroAlt` | m | Barometric altitude AGL |

**Old FC body frame:** `+Y = thrust axis` (up on pad), `+Z = up` (out of sensor top). This differs from both CASPER-2 (+Z = thrust) and Raven (+X = thrust).

**Time alignment:** Old FC timestamps are absolute microseconds. Liftoff occurs at `time ≈ 836,747,731 µs` (row 15379). To align with Raven `FlightTime`: `t_flight = (time - 836747731) / 1e6`.

**FRAME REMAP REQUIRED:** Old FC sensor frame → CASPER-2 body frame requires axis remapping. The EKF regression test must handle this if using old FC sensor data as input.

#### 6.1.4 `FilteredDataFile.csv` — **DO NOT USE AS TRUTH**

| Property | Value |
|----------|-------|
| Source | Previous flight computer's EKF output |
| Status | **DIVERGED — unreliable** |

The old FC's EKF catastrophically diverged during flight. The filtered velocity (`vz ≈ -154 m/s` constant throughout flight, including during ascent) and position (`z` monotonically decreasing) are wrong. The `Altitude` column loosely tracks baro and is marginally usable, but all other filtered columns are garbage.

**Use only for:** Negative example of what EKF divergence looks like. The test harness may optionally include a "divergence detection" test that flags if CASPER-2's EKF exhibits similar symptoms (constant velocity, monotonic position regardless of baro).

### 6.2 CSV Loader

```c
typedef struct {
    int col_count;
    char **col_names;       /* from header row */
    int row_count;
    float **data;           /* data[row][col] */
} csv_table_t;

int csv_load(const char *path, csv_table_t *table);
int csv_find_col(const csv_table_t *table, const char *name);  /* -1 if not found */
void csv_free(csv_table_t *table);
```

The loader must handle: BOM (`\xEF\xBB\xBF`) in `FilteredDataFile.csv` and `RawDataFile.csv`, `\r\n` line endings, and trailing whitespace in column names (e.g., `"Apo_FER_H  ex"` in `raven_data.csv` — note the double space).

### 6.3 `test_ekf_regression.c`

**Input:** `RawDataFile.csv` (sensor data, remapped to CASPER-2 body frame)
**Truth:** `raven_data.csv` (altitude and velocity)

**Critical constraint: APOGEE + 5 SECOND CUTOFF.** All truth comparisons stop at `FlightTime = 38.76s` (apogee + 5s). Data after this point is ignored entirely.

**Test procedure:**
1. Load `RawDataFile.csv` and `raven_data.csv`.
2. Time-align: convert old FC `time` (µs absolute) to flight time using liftoff detection (first row where `|a| > 15 m/s²`). Raven `FlightTime` is already relative to liftoff.
3. Remap old FC sensor axes to CASPER-2 body frame (agent must determine the correct permutation from ORIENTATION_SPEC and old FC frame convention: old +Y=thrust → CASPER-2 +Z=thrust).
4. Convert Raven truth to SI: `alt_m = Baro_Altitude_AGL × 0.3048`, `vel_mps = Velocity_Up × 0.3048`.
5. Initialize EKF with default config (EKF_SPEC P₀, Q values).
6. Step through sensor rows: `casper_ekf_predict()` at each IMU sample, `casper_ekf_update_baro()` at each baro sample.
7. At each Raven truth timestamp where `FlightTime ≤ 38.76s`, interpolate EKF state and compare.
8. **Assert:**
   - Peak EKF altitude within ±5% of Raven truth apogee (5153 m).
   - EKF velocity zero-crossing within ±0.5s of Raven truth (32.84s).
   - No NaN or Inf in any state or covariance element.
   - P diagonal remains positive throughout.
   - Baro gating activates when EKF velocity exceeds Mach 0.4 (~136 m/s) and deactivates below Mach 0.35.

### 6.4 `test_attitude_regression.c`

**Input:** `RawDataFile.csv` (gyro + accel + mag, remapped)
**Truth:** `raven_HR_data.csv` (quaternion + Euler from Raven)

**Same apogee + 5s cutoff: `Flight_Time_(s) ≤ 38.76s`.**

**Test procedure:**
1. Load `RawDataFile.csv` and `raven_HR_data.csv`.
2. Time-align and remap sensor axes as in §6.3.
3. Convert Raven quaternion truth to CASPER-2 body frame convention (Raven +X = thrust → CASPER-2 +Z = thrust). Alternatively, compare tilt magnitude and roll angle rather than raw quaternions to avoid frame ambiguity.
4. Run CASPER-2 attitude pipeline: static init from accel+mag → pad gyro bias estimation → flight propagation.
5. At each Raven HR truth timestamp where `Flight_Time_(s) ≤ 38.76s`, compare.
6. **Assert:**
   - Pad phase: quaternion converges, tilt < 2° from vertical.
   - No quaternion norm drift > 1e-4 from unity at any timestep.
   - Tilt angle matches Raven `Tilt_Angle_(deg)` (from `raven_data.csv`, interpolated to HR timestamps) within ±5° during coast, ±10° during boost.
   - Roll angle matches Raven `Roll_Angle_(deg)` within ±15° (roll is poorly observable without GPS).

### 6.5 Divergence Canary Test

**Input:** Observe CASPER-2 EKF output during regression.
**Purpose:** Detect if CASPER-2's EKF shows the same divergence pattern as the old FC.

**Assert:**
- EKF velocity is NOT constant (std dev > 1 m/s) during any 10-second window within the flight.
- EKF altitude is NOT monotonically decreasing while baro altitude is increasing.
- EKF altitude error vs baro does not grow unboundedly (|EKF_alt - baro_alt| < 500m at all times within cutoff window).

---

## 7. Compliance Tests

### 7.1 Spec Constant Validation (`test_*_spec.c`)

These tests `#include` the firmware headers and assert that compile-time constants match the specification documents. Any spec change that isn't reflected in code (or vice versa) fails the build.

**EKF_SPEC compliance (`test_ekf_spec.c`):**

| Constant | Header Symbol | Expected Value | Spec Reference |
|----------|--------------|----------------|----------------|
| EKF dt | `EKF_DT` | 0.0012f | §2.11 |
| Mach gate on | `MACH_GATE_ON` | 0.40f | §2.8 |
| Mach gate off | `MACH_GATE_OFF` | 0.35f | §2.8 |
| P₀ altitude | `P0[0]` | 0.1f | §2.10 |
| P₀ velocity | `P0[1]` | 0.001f | §2.10 |
| P₀ accel bias | `P0[2]` | 0.025f | §2.10 |
| P₀ baro bias | `P0[3]` | 0.75f | §2.10 |
| GPS alt R | (stub) | 100.0f | §2.9 |
| GPS vel R | (stub) | 1.0f | §2.9 |
| Baro bias floor | — | 1e-4f | §2.7 |
| Launch accel threshold | `launch_accel_g` | 3.0f | §3.1 |
| Gyro LPF cutoff | `gyro_lpf_cutoff_hz` | 50.0f | §3.1 |
| Mag update rate | `mag_update_hz` | 10.0f | §3.1 |
| Static init mag samples | — | 500 | §3.2 |
| Static init timeout | — | 10.0s | §3.2 |
| Calibration duration | — | 30.0s | §3.2 |

**INTERFACE_SPEC compliance (`test_interface_spec.c`):**

**Agent task:** Find the INTERFACE_SPEC document (`.md` in the repo or project knowledge) and the telemetry message set document (`.docx` in the repo). Cross-reference both against the firmware headers to build assertions.

- Every `msg_id` constant in firmware headers matches the value documented in the msgset doc.
- Every packet struct `sizeof()` (or builder output size) matches the byte count in the msgset doc.
- CRC-32 parameters match spec (polynomial, init, reflect, final XOR).
- COBS parameters match spec (delimiter, max block length).
- Handshake byte matches spec.
- Status bitmap layout matches spec (bit positions for CNT, ARM, FSM, FIRED, ERROR).

**SENSOR_SPEC compliance (`test_sensor_spec.c`):**

| Constant | Expected | Spec Reference |
|----------|----------|----------------|
| LSM6DSO32 WHO_AM_I | 0x6C | §2.1 |
| ADXL372 DEVID | 0xFA | §3.3 |
| MS5611 reset command | 0x1E | §4.3 |
| ADXL372 sensitivity | 100 mg/LSB | §3.5 |
| LSM6DSO32 accel range | ±32g | §2.1 |
| LSM6DSO32 gyro range | ±2000 dps | §2.1 |
| PYRO continuity threshold | 8000 | HARDWARE_SPEC §8.1 |

**PYRO_SPEC compliance (`test_pyro_spec.c`):**

- CMD_ARM = 0x80, CMD_FIRE = 0x81.
- Magic bytes = 0xCA, 0x5A.
- CAC confirm timeout = 5s.
- Fire pin assignments match channel map table.

### 7.2 ORIENTATION_SPEC Compliance

Tested via `test_nav_pipeline.c`:
- Sensor-to-body remap is cyclic permutation: `body[i] = sensor[(i+2) % 3]`.
- Quaternion convention: scalar-first `[w, x, y, z]`.
- Body frame: +Z = nose (thrust axis, up on pad), +Y = starboard.
- Mag negation applied before calibration.
- Attitude estimator receives accel/gyro in body frame, mag in sensor frame (after negation + cal, before cyclic remap).

### 7.3 JSF AV C++ Static Checks (`test_jsf_static.c`)

Since the codebase is C (not C++), the applicable JSF AV subset is checked. Per the SKILL.md: "JSF AV C++ baseline, deviate when STM32 HAL requires it." Rules are tiered by severity.

#### 7.3.1 Severity Tiers

**HARD FAIL — test suite fails, blocks verification:**

These are safety-critical rules with no acceptable deviation in application code (`App/`).

| JSF Rule | Check | Method |
|----------|-------|--------|
| No dynamic allocation | `malloc`/`calloc`/`realloc`/`free` absent in `App/` | grep |
| No `setjmp`/`longjmp` | Absent in `App/` | grep |
| No `atoi`/`atof`/`atol` | Absent in `App/` | grep |
| No `goto` | Absent in `App/` | grep |
| Variables initialized | `-Wuninitialized -Werror` | Compiler flag |
| Include guards | Every `.h` in `App/` has `#ifndef`/`#define`/`#endif` | Script check |
| No recursion | Call graph acyclic in `App/` | `cflow` or script |

**SOFT WARN — printed as warnings, does not block verification:**

These are style and complexity rules where minor deviations are expected, especially in HAL-interfacing code or large FSM switch statements.

| JSF Rule | Check | Threshold | Method |
|----------|-------|-----------|--------|
| Functions ≤ 200 SLOC | Line count per function | Warn > 200, hard fail > 400 | Script |
| Cyclomatic complexity ≤ 20 | Complexity per function | Warn > 20, hard fail > 40 | Script |
| Fixed-width types | `int`/`long`/`short` used for data variables | Warn only (HAL callbacks require `int`) | grep |
| No bare `char` for data | `char` used outside string contexts | Warn only | grep |
| Single statement per line | Multiple `;` on one line | Warn only | grep |

**EXCLUDED — not checked:**

Rules that fundamentally conflict with STM32 HAL patterns and are blanket-excluded:

| Exclusion | Reason |
|-----------|--------|
| `Core/` and `Drivers/` directories | STM32Cube-generated code, not under project control |
| HAL callback signatures | HAL defines `void HAL_*_Callback(type *handle)` — cannot change parameter types or naming |
| `Error_Handler()` pattern | CubeMX generates this, project wraps it |
| Preprocessor macros for HAL config | `USB_MODE`, `ARM_MATH_CM7`, etc. — required by build system |
| Third-party headers (`unity.h`, CMSIS) | Not project code |

#### 7.3.2 Scan Scope

The scanner only checks files under `Software/App/` and `Software/Core/Src/casper_*.c`. Generated HAL code (`Core/Src/main.c` USER CODE regions are included, but CubeMX-generated regions are excluded via `/* USER CODE BEGIN */` / `/* USER CODE END */` markers).

```bash
# Scan scope
SCAN_DIRS="Software/App/ Software/Core/Src/casper_*.c"
EXCLUDE_PATTERNS="Drivers/ unity/ Tests/ stubs/ mocks/"
```

#### 7.3.3 Deviation Documentation

Per JSF AV: any `shall` rule deviation must be documented in the file that contains it. The scanner checks for this with a **light touch:**

**Rule:** If a HARD FAIL rule is violated in a scanned file, the test fails UNLESS a deviation comment exists in the same file within 10 lines of the violation:

```c
/* JSF-DEV: malloc used here for <reason> — approved <date> */
```

The scanner recognizes deviation markers matching the pattern:
```
JSF-DEV:
```

**Behavior:**
- HARD FAIL violation + no `JSF-DEV` comment nearby → **FAIL**
- HARD FAIL violation + `JSF-DEV` comment within 10 lines → **WARN** (logged, not blocking)
- SOFT WARN violation → **WARN** always (no deviation comment required)

This keeps the system honest without being pedantic about every HAL-forced `int` return type.

#### 7.3.4 Implementation

A shell script (`scripts/jsf_check.sh`) runs all checks and outputs a structured report:

```bash
#!/bin/bash
# scripts/jsf_check.sh
# Returns 0 if no undeviated HARD FAIL violations found
# Returns 1 if any HARD FAIL without JSF-DEV marker

HARD_FAIL=0
WARNINGS=0

# --- HARD FAIL checks ---
# Dynamic allocation
hits=$(grep -rn 'malloc\|calloc\|realloc\|\bfree(' $SCAN_DIRS $EXCLUDE_FLAGS)
for hit in $hits; do
    file=$(echo $hit | cut -d: -f1)
    line=$(echo $hit | cut -d: -f2)
    # Check for JSF-DEV within 10 lines
    if sed -n "$((line-10)),$((line+10))p" "$file" | grep -q 'JSF-DEV'; then
        echo "WARN: $hit (deviation documented)"
        WARNINGS=$((WARNINGS+1))
    else
        echo "FAIL: $hit (no deviation documented)"
        HARD_FAIL=$((HARD_FAIL+1))
    fi
done

# ... (repeat for each HARD FAIL rule)

# --- SOFT WARN checks ---
# Function length (warn only)
# ... (awk-based line counter per function)

echo "=== JSF Summary ==="
echo "Hard failures: $HARD_FAIL"
echo "Warnings: $WARNINGS"
exit $HARD_FAIL  # 0 = pass, >0 = fail
```

This script is registered as a ctest case:

```cmake
add_test(NAME jsf_compliance
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/jsf_check.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
)
```

---

## 8. Naming Convention Tests

**Msgset naming (casper-telemetry-msgset-v5 §2):**

All telemetry field constants, message IDs, and command identifiers in `tlm_types.h` and related headers must follow:

```
[Source]_[Category]_[Field]
```

Where Source ∈ {FC, GS, MC} and Category ∈ {MSG, TLM, GPS, ATT, PYR, PWR, FSM, SYS, EVT, LNK, PKT, DRV, CMD, CFG}.

**Test:** Parse all `#define` and `enum` values in `tlm_types.h`. For each, verify the prefix matches the pattern regex:

```
^(FC|GS|MC)_(MSG|TLM|GPS|ATT|PYR|PWR|FSM|SYS|EVT|LNK|PKT|DRV|CMD|CFG)_[A-Z0-9_]+$
```

**Severity:**
- Identifiers in `tlm_types.h` that don't match → **HARD FAIL** (these are wire-protocol names, must be correct).
- Identifiers in other `App/` headers that don't match → **WARN** (internal helpers, utility macros, and HAL wrappers may have different naming patterns).
- Identifiers prefixed with `CASPER_`, `PYRO_`, `CDC_`, `COBS_` → **EXCLUDED** (internal implementation constants, not protocol identifiers).

---

## 8.1 Deprecated Document Detection

**Purpose:** Prevent stale specs from being referenced by developers or tooling.

**Test:** Verify that `hardware-spec.md` (the deprecated early-design document) either:
- Does not exist in the repo, OR
- Contains a `DEPRECATED` marker in the first 5 lines.

If the file exists without a deprecation marker, emit a **WARN**. This is a soft check — it won't block CI but will remind you to clean it up.

The authoritative hardware reference is `HARDWARE_SPEC.md` (project knowledge).

---

## 9. Test Infrastructure Requirements

### 9.1 Unity Configuration

```c
// test_config.h — shared across all test files
#define UNITY_INCLUDE_FLOAT
#define UNITY_INCLUDE_DOUBLE
#define UNITY_FLOAT_PRECISION 1e-6f
#define UNITY_DOUBLE_PRECISION 1e-10

// Custom assertion for float arrays
#define TEST_ASSERT_FLOAT_ARRAY_WITHIN(delta, expected, actual, len) \
    do { for (int _i = 0; _i < (len); _i++) { \
        TEST_ASSERT_FLOAT_WITHIN((delta), (expected)[_i], (actual)[_i]); \
    } } while(0)

// Custom assertion for quaternion comparison (handles sign ambiguity: q == -q)
#define TEST_ASSERT_QUAT_EQUAL(expected, actual, tol) \
    do { \
        float _dot = (expected)[0]*(actual)[0] + (expected)[1]*(actual)[1] + \
                     (expected)[2]*(actual)[2] + (expected)[3]*(actual)[3]; \
        TEST_ASSERT_FLOAT_WITHIN((tol), 1.0f, fabsf(_dot)); \
    } while(0)

// Assert no NaN or Inf in array
#define TEST_ASSERT_ALL_FINITE(arr, len) \
    do { for (int _i = 0; _i < (len); _i++) { \
        TEST_ASSERT_TRUE_MESSAGE(isfinite((arr)[_i]), "NaN/Inf detected"); \
    } } while(0)
```

### 9.2 Tolerance Rationale

| Domain | Tolerance | Justification |
|--------|-----------|--------------|
| Quaternion math | 1e-7 | Single-precision float epsilon ≈ 1.2e-7 |
| Euler angles | 0.01° | Adequate for telemetry display (0.1° resolution) |
| EKF altitude (static) | 0.001m | Numerical precision at pad |
| EKF altitude (flight regression) | 5% of apogee | Accounts for model simplifications |
| CRC-32 | Exact | Must be bit-identical |
| Packet size | Exact | Must match spec byte count |
| Timing (apogee detection) | 0.5s | Accounts for filter phase lag |

### 9.3 Coverage Reporting

```bash
# Tests/scripts/coverage.sh
cd Tests/build
cmake .. -DCMAKE_C_FLAGS="--coverage"
cmake --build . -j$(nproc)
ctest --output-on-failure
lcov --capture --directory . --output-file coverage.info
lcov --remove coverage.info '*/stubs/*' '*/unity/*' --output-file filtered.info
genhtml filtered.info --output-directory coverage_report
echo "Report: Tests/build/coverage_report/index.html"
```

---

## 10. Execution Workflow

### 10.1 Developer (Manual)

```bash
cd Tests
./scripts/run_tests.sh          # Build + run all + summary
./scripts/run_tests.sh tier1    # Run only tier 1
./scripts/run_tests.sh coverage # Run all + generate coverage HTML
```

### 10.2 Claude Code (Automated)

During plan-mode verification:

```
1. cmake -S Tests -B Tests/build
2. cmake --build Tests/build -j$(nproc)    ← compile error = verification fail
3. cd Tests/build && ctest --output-on-failure  ← test failure = verification fail
```

Claude Code must not proceed past verification if any test fails. When fixing a bug or adding a feature, Claude Code should:

1. Run the full suite first to establish baseline.
2. Make changes.
3. Re-run the suite to confirm no regressions.
4. If adding new functionality, add corresponding tests before or during implementation.

### 10.3 Slash Command

The `/test` command in Claude Code maps to:

```bash
bash Tests/scripts/run_tests.sh
```

Output is the ctest summary: number of tests passed, failed, and details of any failures.

---

## 11. Implementation Stages

### Stage 1 — Foundation (Do First)

- [ ] Create `Tests/` directory structure.
- [ ] Vendor Unity framework.
- [ ] Create HAL stub headers (types only, enough to compile).
- [ ] Create `Tests/CMakeLists.txt` with one test target (`test_casper_quat`).
- [ ] Verify `cmake && make && ctest` works end-to-end.
- [ ] Create `run_tests.sh`.
- [ ] Configure Claude Code verification to run the suite.

### Stage 2 — Tier 1 Core

- [ ] `test_casper_quat.c` — full quaternion library coverage.
- [ ] `test_cobs.c` — encode/decode with all edge cases.
- [ ] `test_crc32.c` — standard test vectors.
- [ ] `test_status_pack.c` — bitmap pack/unpack.
- [ ] `test_ms5611_math.c` — calibration math.
- [ ] CMSIS-DSP stubs (`arm_math_stub.c`) — matrix multiply/add.

### Stage 3 — Tier 1 Navigation

- [ ] `test_casper_ekf.c` — init, predict, update, gating, numerics.
- [ ] `test_casper_attitude.c` — static init, phases, RK4, corrections.
- [ ] `test_casper_gyro_int.c` — legacy integrator.
- [ ] `test_tlm_pack.c` — packet serialization.

### Stage 4 — Tier 2 Protocol

- [ ] Mock infrastructure (`mock_spi.c`, `mock_tick.c`, etc.).
- [ ] `test_cac_handler.c` — full CAC state machine.
- [ ] `test_cmd_router.c` — COBS accumulation + dispatch.
- [ ] `test_pyro_manager.c` — safety gates.
- [ ] `test_flight_fsm.c` — all state transitions.

### Stage 5 — Compliance + Regression

- [ ] All `test_*_spec.c` compliance tests.
- [ ] `test_naming_convention.c`.
- [ ] `scripts/jsf_check.sh` static checks.
- [ ] CSV loader implementation (handle BOM, \r\n, column name whitespace).
- [ ] `test_ekf_regression.c` with Raven truth data (apogee+5s cutoff).
- [ ] `test_attitude_regression.c` with Raven HR quaternion truth.
- [ ] `test_divergence_canary.c` — detect old-FC-style divergence patterns.

### Stage 6 — Tier 3 Integration + Polish

- [ ] `test_flight_loop.c` — coarse pipeline test.
- [ ] `test_nav_pipeline.c` — frame remap + pipeline.
- [ ] Coverage reporting script.
- [ ] Final pass: verify all spec constants covered, all msg_ids tested.

---

## 12. Maintenance Rules

1. **Every new module gets tests before merge.** No new `.c` file in `App/` without a corresponding `test_*.c`.
2. **Spec changes require compliance test updates.** If EKF_SPEC changes a constant, `test_ekf_spec.c` must update in the same commit.
3. **Regression CSVs are immutable.** Truth data files in `Tests/CSVs/` are never modified. New flight data gets new files.
4. **Tests must not depend on execution order.** Each test file is a standalone executable. No shared mutable state between test files.
5. **No test should take > 5 seconds.** Regression tests processing large CSVs are the exception (up to 30s allowed).
6. **Stub/mock code never enters the firmware build.** The `Tests/stubs/` and `Tests/mocks/` directories are excluded from the STM32 Makefile.

---

## 13. Open Questions

1. ~~**CSV column format:**~~ **RESOLVED.** All four CSV files fully characterized in §6.1. Column names, units, sample rates, and frame conventions documented.

2. ~~**Truth data source for attitude:**~~ **RESOLVED.** `raven_HR_data.csv` provides quaternion truth at 500 Hz. `raven_data.csv` provides tilt/roll/future angles at 50 Hz. Cutoff at `FlightTime = 38.76s` (apogee + 5s).

3. ~~**JSF rules in `.claude/`:**~~ **RESOLVED.** All `.md` files in `.claude/` are now up to date. SKILL.md confirms JSF AV C++ baseline with HAL deviations. Compliance tests use tiered severity (§7.3).

4. ~~**Flight FSM state count:**~~ **AGENT TASK.** Agent reads `flight_fsm.c/h` and the msgset doc to extract states and transitions (§4.4). 12 states (0x0–0xB) are documented in the msgset. If transition logic is incomplete in code, agent creates `Tests/FSM_TRANSITIONS.md`.

5. ~~**Telemetry packet struct definitions:**~~ **AGENT TASK.** Agent finds `tlm_types.h` and the msgset `.docx` in the repo, cross-references them (§3.8, §7.1). Tests adapt to whether packets use packed structs or field-by-field builders.

6. **Sensor axis remap for regression tests.** The old FC sensor data (`RawDataFile.csv`) uses a different body frame (+Y = thrust) than CASPER-2 (+Z = thrust). The regression tests (§6.3, §6.4) must remap axes. The agent must derive the correct permutation from ORIENTATION_SPEC and the old FC convention (pad: `ax≈0.5, ay≈1.2, az≈-9.5` → gravity in -Z, slight tilt). Similarly, Raven HR uses +X = thrust. Both require frame transforms for quaternion comparison.
