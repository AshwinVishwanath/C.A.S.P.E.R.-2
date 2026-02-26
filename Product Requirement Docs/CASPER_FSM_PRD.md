# CASPER-2 FSM Transition Logic PRD

**Purpose:** Implementation instructions for sensor-driven flight state machine transitions, pyro auto-arm/auto-fire, and HIL test infrastructure on the C.A.S.P.E.R.-2 flight computer.

**Target:** Claude Code agents working in STM32CubeIDE C project.

**Hardware:** STM32H750VBT6, 432 MHz, 128KB flash, 512KB AXI SRAM.

**Repository:** https://github.com/AshwinVishwanath/C.A.S.P.E.R.-2

**Branch:** Create new branch `fsm-transitions` from current working branch.

**Authoritative spec:** FSM_TRANSITION_SPEC.md (attached to this PRD). That document defines all threshold values, transition conditions, struct definitions, and pseudocode. This PRD tells you **what to build, in what order, and how to test it.** The spec tells you **the exact logic.**

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
8. **All dwell timer comparisons use `fsm_get_tick()`, NEVER `HAL_GetTick()` directly.**

---

## LEVEL 1 — Scaffolding: Types, Dwell Timer, Virtual Clock

**Goal:** Create the foundational types and utilities that all subsequent levels depend on. No FSM logic yet.

**Agent scope:** Types, helper functions, and a host-side test harness.

### Files to CREATE

| File | Contents |
|------|----------|
| `App/fsm/fsm_types.h` | `fsm_input_t` struct, threshold `#define` constants, `dwell_timer_t` typedef, `PYRO_EXCLUDE_MASK` |
| `App/fsm/fsm_util.c` | `dwell_check()`, `compute_vert_accel()`, `check_antenna_up()`, `fsm_get_tick()`, `fsm_set_tick()` |
| `App/fsm/fsm_util.h` | Public API for above |
| `Tests/test_fsm_util.c` | Host-compilable unit tests (see below) |
| `Tests/Makefile` | Builds and runs host-side tests with `gcc` (no ARM, no HAL) |

### Files to MODIFY

None. Do not touch any existing files in Level 1.

### Struct: `fsm_input_t`

Copy exactly from FSM_TRANSITION_SPEC.md §2. All fields, all comments. Do not rename, reorder, or add fields.

### Threshold Constants

Copy exactly from FSM_TRANSITION_SPEC.md §4.0. All `#define` values. Place in `fsm_types.h`.

### `dwell_timer_t` and `dwell_check()`

Copy exactly from FSM_TRANSITION_SPEC.md §3. The `dwell_check()` function must use `fsm_get_tick()`, not `HAL_GetTick()`.

### Virtual Clock

Copy exactly from FSM_TRANSITION_SPEC.md §3.1. When `HIL_MODE` is defined, `fsm_get_tick()` returns a controllable virtual tick. When not defined, it calls `HAL_GetTick()`.

For the host-side test harness (Tests/), `HIL_MODE` is always defined, and `HAL_GetTick()` is stubbed.

### `compute_vert_accel()` and `check_antenna_up()`

Copy exactly from FSM_TRANSITION_SPEC.md §2.2 and §2.3. These depend on `quat_rotate_vec()` from the existing `casper_quat.c`. For the host test harness, provide a minimal stub of `quat_rotate_vec()` or copy the implementation into the test build.

### Test Harness

The test Makefile must compile `fsm_util.c`, `test_fsm_util.c`, and any required stubs with native `gcc` (NOT arm-none-eabi-gcc). Tests run on the host. No HAL, no CMSIS, no STM32 headers.

Provide a `HAL_stub.h` with:
```c
#define HIL_MODE
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef int int32_t;
typedef short int16_t;
typedef signed char int8_t;
#include <stdbool.h>
#include <math.h>
```

### Tests (Level 1)

```
TEST_DWELL_01: dwell_check returns false when condition is false
TEST_DWELL_02: dwell_check returns false when condition true but dwell not elapsed
TEST_DWELL_03: dwell_check returns true when condition sustained for required time
TEST_DWELL_04: dwell_check resets if condition drops out mid-dwell
TEST_DWELL_05: dwell_check with 0ms required time returns true immediately

TEST_TICK_01: fsm_set_tick / fsm_get_tick round-trips correctly
TEST_TICK_02: dwell_check uses fsm_get_tick, not wall clock

TEST_VERT_ACCEL_01: compute_vert_accel returns ~0g for stationary (accel = [0,0,9.81], q = identity)
TEST_VERT_ACCEL_02: compute_vert_accel returns ~2g for 2g upward thrust
TEST_VERT_ACCEL_03: compute_vert_accel returns ~-1g for free fall (accel = [0,0,0])

TEST_ANTENNA_01: check_antenna_up returns true for identity quaternion (upright)
TEST_ANTENNA_02: check_antenna_up returns false for 90° tilt
TEST_ANTENNA_03: check_antenna_up returns true for 25° tilt (within 30° threshold)
TEST_ANTENNA_04: check_antenna_up returns false for 35° tilt (outside threshold)
```

All tests print PASS/FAIL to stdout. Exit code 0 = all pass, nonzero = failure.

### DO NOT TOUCH

- `App/fsm/flight_fsm.c` — do not modify yet
- `App/fsm/flight_fsm.h` — do not modify yet
- `App/pyro/*` — do not modify
- `App/nav/*` — do not modify
- `Core/*` — do not modify

### DONE WHEN

1. `make -C Tests` compiles cleanly with `gcc` on host
2. All TEST_DWELL_*, TEST_TICK_*, TEST_VERT_ACCEL_*, TEST_ANTENNA_* pass
3. `fsm_types.h` contains all threshold constants matching FSM_TRANSITION_SPEC.md §4.0 exactly
4. `fsm_input_t` matches FSM_TRANSITION_SPEC.md §2 exactly
5. No dynamic allocation anywhere
6. No `HAL_GetTick()` calls in `fsm_util.c` — only `fsm_get_tick()`

---

## LEVEL 2 — FSM Core: State Transitions

**Goal:** Implement the real sensor-driven `flight_fsm_tick()` that replaces the current `(void)state;` TODO. This is the core flight logic.

**Agent scope:** FSM switch-case logic, dwell timers, transition conditions, event emissions.

### Files to MODIFY

| File | Change |
|------|--------|
| `App/fsm/flight_fsm.h` | Change `flight_fsm_tick()` signature to take `const fsm_input_t *in`. Add `flight_fsm_reset()` for test use. |
| `App/fsm/flight_fsm.c` | Replace sim-only logic in `flight_fsm_tick()` with sensor-driven transitions. Keep sim logic behind `if (s_sim_active)` guard — sim mode still works as before, sensor mode runs when sim is not active. |

### Files to CREATE

| File | Contents |
|------|----------|
| `Tests/test_fsm_transitions.c` | Host-compilable FSM transition tests |

### Implementation Rules

1. **`flight_fsm_tick()` must handle both sim and sensor modes.** If `s_sim_active`, run existing sim logic (unchanged). If not, run the new sensor-driven switch-case.
2. **Every state is a `case` in the switch.** No fall-through. Every case ends with `break`.
3. **`transition_to()` already exists.** Extend it to call `pyro_mgr_auto_arm_flight()` on BOOST entry (see §6.3 of spec). For Level 2 testing, stub this call — the actual pyro integration is Level 3.
4. **All static variables** listed in FSM_TRANSITION_SPEC.md §8 must be declared and cleared in `flight_fsm_init()` and `flight_fsm_reset()`.
5. **Peak tracking:** `s_peak_accel_g` resets on BOOST entry. `s_peak_alt_m` resets on COAST entry.
6. **Stage counting:** `s_stage_count` starts at 0, increments on each COAST → BOOST re-light.

### State-by-State Implementation

Implement transitions exactly as specified in FSM_TRANSITION_SPEC.md §4.1 through §4.8. The pseudocode in the spec is authoritative. Follow it literally.

**Evaluation order within COAST state:** Check sustain re-light FIRST, then apogee. If sustain triggers, `break` immediately — do not evaluate apogee.

**APOGEE state:** Check normal main deploy (altitude) FIRST, then drogue failure backup. If altitude triggers, `break` — do not evaluate drogue failure.

**LANDED → RECOVERY:** Simple timer check. Time in LANDED > `FSM_LANDED_TO_LOWPOWER_S` (300s = 300000ms).

### Stubs Required for Level 2

The FSM calls functions that don't exist yet in the test harness. Stub them:

```c
/* stubs for Level 2 testing */
void pyro_mgr_auto_arm_flight(void) { /* no-op stub */ }
int  pyro_mgr_auto_fire(uint8_t ch, uint16_t dur) { return 0; /* stub */ }
void tlm_queue_event(uint8_t type, uint16_t data) { /* record for test verification */ }
```

For test verification, `tlm_queue_event` should push events to a test-visible array so tests can assert which events were emitted.

### Tests (Level 2)

**Flight profile replay tests:** Construct `fsm_input_t` sequences that simulate flight profiles, advance `fsm_get_tick()` between calls, and verify state transitions.

```
TEST_FSM_LAUNCH_01: PAD stays in PAD when accel < threshold
TEST_FSM_LAUNCH_02: PAD stays in PAD when accel sustained but vel < 15 m/s
TEST_FSM_LAUNCH_03: PAD stays in PAD when antenna_up is false
TEST_FSM_LAUNCH_04: PAD → BOOST when all three conditions met and sustained 100ms
TEST_FSM_LAUNCH_05: PAD accel dwell resets if accel drops below threshold mid-dwell

TEST_FSM_BURNOUT_01: BOOST stays in BOOST while accel > 0g
TEST_FSM_BURNOUT_02: BOOST → COAST when accel < 0g sustained 100ms
TEST_FSM_BURNOUT_03: BOOST burnout dwell resets on brief accel spike

TEST_FSM_SUSTAIN_01: COAST → BOOST when accel > 3g sustained 100ms (sustain re-light)
TEST_FSM_SUSTAIN_02: Stage count increments on re-light
TEST_FSM_SUSTAIN_03: Peak accel resets on BOOST re-entry

TEST_FSM_APOGEE_01: COAST stays in COAST when vel > 0
TEST_FSM_APOGEE_02: COAST stays in COAST when vel <= 0 but flight_time < 5s
TEST_FSM_APOGEE_03: COAST → APOGEE when vel <= 0 for 25ms AND flight_time > 5s
TEST_FSM_APOGEE_04: Sustain re-light check takes priority over apogee check

TEST_FSM_MAIN_01: APOGEE → MAIN when alt <= main_deploy_alt_m
TEST_FSM_MAIN_02: APOGEE → MAIN via drogue failure (vert_vel < -threshold for time)
TEST_FSM_MAIN_03: Normal altitude deploy takes priority over drogue failure

TEST_FSM_LANDED_01: MAIN stays in MAIN when velocity oscillating (pendulum swing)
TEST_FSM_LANDED_02: MAIN → LANDED when |vel| < 1 AND |Δalt| < 2m for 3s
TEST_FSM_LANDED_03: Landing dwell resets if altitude drifts > 2m

TEST_FSM_RECOVERY_01: LANDED → RECOVERY after 300s

TEST_FSM_EVENTS_01: PAD → BOOST emits FC_EVT_STATE(BOOST)
TEST_FSM_EVENTS_02: BOOST → COAST emits FC_EVT_STATE(COAST) + FC_EVT_BURNOUT
TEST_FSM_EVENTS_03: COAST → APOGEE emits FC_EVT_STATE + FC_EVT_APOGEE + FC_EVT_PYRO
TEST_FSM_EVENTS_04: APOGEE → MAIN (drogue fail) emits FC_EVT_ERROR(ERR_DROGUE_FAIL)

TEST_FSM_FULL_FLIGHT_01: Complete single-stage flight profile
    PAD → BOOST → COAST → APOGEE → MAIN → LANDED → RECOVERY
    Verify every transition, every event, correct ordering.

TEST_FSM_FULL_FLIGHT_02: Two-stage flight profile
    PAD → BOOST → COAST → BOOST (sustain) → COAST → APOGEE → MAIN → LANDED
    Verify stage count = 1, both burnout events emitted.

TEST_FSM_FULL_FLIGHT_03: Drogue failure flight profile
    PAD → BOOST → COAST → APOGEE → MAIN (via drogue fail)
    Verify ERR_DROGUE_FAIL event, main fires early.
```

### DO NOT TOUCH

- `App/pyro/*` — stubs only, do not modify real pyro code
- `App/nav/*` — do not modify
- `Core/*` — do not modify
- Sim flight profile code in `flight_fsm.c` — preserve behind `if (s_sim_active)` guard
- `flight_fsm_sim_*()` functions — do not modify

### DONE WHEN

1. All TEST_FSM_* pass on host
2. `flight_fsm_tick()` handles all transitions from FSM_TRANSITION_SPEC.md §4.1–§4.8
3. Sim mode still works (existing `flight_fsm_sim_start()` / `flight_fsm_sim_stop()` unbroken)
4. Every transition emits correct events per FSM_TRANSITION_SPEC.md §9
5. `flight_fsm_reset()` clears all internal state (dwell timers, peaks, stage count)
6. No `HAL_GetTick()` calls — only `fsm_get_tick()`

---

## LEVEL 3 — Pyro Integration: Safety Invariants, Auto-Arm, Auto-Fire

**Goal:** Wire the FSM to the real pyro manager. Implement PAD lockout, channel exclusion, auto-arm, and auto-fire. This is the safety-critical level.

**Agent scope:** Pyro manager extensions and safety tests.

### Files to MODIFY

| File | Change |
|------|--------|
| `App/pyro/pyro_manager.c` | Add `pyro_mgr_auto_fire()`, `pyro_mgr_auto_arm_flight()`, `PYRO_PAD_LOCKOUT()` macro, `pyro_ch_excluded()` |
| `App/pyro/pyro_manager.h` | Expose new public functions |
| `App/fsm/flight_fsm.c` | Replace Level 2 stubs with real `pyro_mgr_auto_arm_flight()` and `pyro_mgr_auto_fire()` calls |

### Files to CREATE

| File | Contents |
|------|----------|
| `Tests/test_pyro_safety.c` | Host-compilable safety tests |

### Implementation

Follow FSM_TRANSITION_SPEC.md §6.1–§6.5 exactly.

**`PYRO_PAD_LOCKOUT()`:** Macro that checks `flight_fsm_get_state() == FSM_STATE_PAD` and returns -1. Must be the FIRST check in `pyro_mgr_auto_fire()`.

**`pyro_ch_excluded()`:** Reads from compile-time `PYRO_EXCLUDE_MASK`. Default 0x00 (no exclusions).

**`pyro_mgr_auto_arm_flight()`:** Called from `transition_to()` on BOOST entry. Arms all channels with continuity that are not excluded. Emits `FC_EVT_ARM` per channel.

**`pyro_mgr_auto_fire()`:** Check order (exact sequence, do not reorder):
1. `PYRO_PAD_LOCKOUT()`
2. `ch >= 4` → return -1
3. `pyro_ch_excluded(ch)` → return -1
4. `!s_armed[ch]` → return -1
5. `!pyro.continuity[ch]` → return -1
6. `pyro.firing[ch]` → return -1
7. Cap duration at `PYRO_MAX_FIRE_MS`
8. `casper_pyro_fire()` + emit `FC_EVT_PYRO`

### Tests (Level 3)

**SAFETY TESTS — THESE ARE THE MOST IMPORTANT TESTS IN THE ENTIRE SYSTEM.**

```
TEST_PAD_LOCK_01: pyro_mgr_auto_fire() returns -1 when FSM is in PAD
TEST_PAD_LOCK_02: pyro_mgr_auto_fire() returns -1 when FSM is in PAD even if channel is armed + has continuity
TEST_PAD_LOCK_03: pyro_mgr_auto_fire() succeeds when FSM is in BOOST and channel is armed + has continuity
TEST_PAD_LOCK_04: pyro_mgr_auto_fire() returns -1 when FSM is in PAD with every possible combination of arm/continuity/exclusion states

TEST_EXCLUDE_01: pyro_mgr_auto_arm_flight() does NOT arm excluded channels
TEST_EXCLUDE_02: pyro_mgr_auto_fire() returns -1 for excluded channel even if armed manually
TEST_EXCLUDE_03: pyro_mgr_auto_arm_flight() arms non-excluded channels with continuity
TEST_EXCLUDE_04: Excluded channels produce no FC_EVT_ARM events

TEST_AUTOARM_01: pyro_mgr_auto_arm_flight() arms channels with continuity
TEST_AUTOARM_02: pyro_mgr_auto_arm_flight() skips channels without continuity
TEST_AUTOARM_03: pyro_mgr_auto_arm_flight() is idempotent (already-armed channels unaffected)
TEST_AUTOARM_04: pyro_mgr_auto_arm_flight() emits FC_EVT_ARM for each newly armed channel
TEST_AUTOARM_05: Auto-arm is called on BOOST entry (verify via full PAD → BOOST transition)
TEST_AUTOARM_06: Auto-arm is called on sustain re-light BOOST re-entry

TEST_AUTOFIRE_01: pyro_mgr_auto_fire() succeeds for armed + continuity + non-PAD
TEST_AUTOFIRE_02: pyro_mgr_auto_fire() returns -1 for unarmed channel
TEST_AUTOFIRE_03: pyro_mgr_auto_fire() returns -1 for no-continuity channel
TEST_AUTOFIRE_04: pyro_mgr_auto_fire() returns -1 for already-firing channel
TEST_AUTOFIRE_05: pyro_mgr_auto_fire() caps duration at PYRO_MAX_FIRE_MS
TEST_AUTOFIRE_06: pyro_mgr_auto_fire() emits FC_EVT_PYRO on success

TEST_INTEGRATION_01: Full flight PAD → BOOST (auto-arm) → COAST → APOGEE (auto-fire apogee ch) → MAIN (auto-fire main ch) → LANDED
    Verify: channels auto-armed on BOOST, apogee pyro fired at APOGEE, main pyro fired at MAIN.

TEST_INTEGRATION_02: Same as above but apogee channel excluded via PYRO_EXCLUDE_MASK
    Verify: apogee channel NOT armed, NOT fired. Main channel still works.

TEST_INTEGRATION_03: Operator pre-arms channels on PAD, then launch
    Verify: auto-arm is idempotent, no duplicate FC_EVT_ARM events for already-armed channels.
```

### DO NOT TOUCH

- `Core/Src/casper_pyro.c` — hardware driver, do not modify
- `App/nav/*` — do not modify
- `App/command/cac_handler.c` — existing CAC fire path must still work unchanged

### DONE WHEN

1. All TEST_PAD_LOCK_* pass — **zero tolerance, every single one**
2. All TEST_EXCLUDE_*, TEST_AUTOARM_*, TEST_AUTOFIRE_* pass
3. All TEST_INTEGRATION_* pass
4. Existing CAC ARM/FIRE tests (if any) still pass — CAC path unbroken
5. `PYRO_PAD_LOCKOUT()` is the first check in `pyro_mgr_auto_fire()`
6. No direct `HAL_GPIO_WritePin()` calls on pyro pins outside of `casper_pyro.c`

---

## LEVEL 4 — Flight Loop Integration and HIL Interface

**Goal:** Wire `fsm_input_t` population into the real flight loop and create the USB command for HIL testing from MATLAB.

**Agent scope:** Flight loop modification, HIL command handler, integration test.

### Files to MODIFY

| File | Change |
|------|--------|
| `App/flight/flight_loop.c` | Add `fsm_input_t` population from EKF + attitude + config. Call `flight_fsm_tick(&fsm_in)` after sensor processing. |
| `App/flight/app_globals.h` | Add `extern` for flight config struct (pyro channels, deploy altitudes) |
| `App/command/cmd_router.c` | Add dispatch for new `CMD_HIL_INJECT` message |

### Files to CREATE

| File | Contents |
|------|----------|
| `App/command/hil_handler.c` | HIL inject handler: deserializes `fsm_input_t` + timestamp from USB, sets virtual clock, calls `flight_fsm_tick()` |
| `App/command/hil_handler.h` | Public API |

### `fsm_input_t` Population in Flight Loop

Follow FSM_TRANSITION_SPEC.md §2.1 exactly. The flight loop already has `ekf`, `att`, `accel_ms2` available as globals. Add:

```c
/* After sensor processing, before telemetry */
if (!flight_fsm_sim_active()) {
    fsm_input_t fsm_in = {0};
    fsm_in.alt_m          = ekf.x[0];
    fsm_in.vel_mps        = ekf.x[1];
    fsm_in.vert_accel_g   = compute_vert_accel(att.q, accel_ms2);
    fsm_in.antenna_up     = check_antenna_up(att.q);
    fsm_in.flight_time_s  = flight_fsm_get_time_s();
    fsm_in.main_deploy_alt_m    = g_flight_cfg.main_deploy_alt;
    fsm_in.drogue_fail_vel_mps  = g_flight_cfg.drogue_fail_vel;
    fsm_in.drogue_fail_time_s   = g_flight_cfg.drogue_fail_time;
    fsm_in.apogee_pyro_ch       = g_flight_cfg.apogee_pyro_ch;
    fsm_in.main_pyro_ch         = g_flight_cfg.main_pyro_ch;
    fsm_in.apogee_fire_dur_ms   = g_flight_cfg.apogee_fire_dur;
    fsm_in.main_fire_dur_ms     = g_flight_cfg.main_fire_dur;

    flight_fsm_tick(&fsm_in);
}
```

### HIL Command: `CMD_HIL_INJECT`

**msg_id:** `0xD1` (next available after `CMD_SIM_FLIGHT` at 0xD0)

**Purpose:** MATLAB sends a serialized `fsm_input_t` + virtual timestamp over USB. The FC deserializes it, sets the virtual clock, and runs one FSM tick. The FC responds with the current FSM state and pyro status via normal telemetry.

**Wire format (little-endian):**

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | msg_id | u8 | `0xD1` |
| 1–4 | virtual_tick_ms | u32 LE | Virtual clock value |
| 5–8 | alt_m | float LE | EKF altitude AGL |
| 9–12 | vel_mps | float LE | EKF vertical velocity |
| 13–16 | vert_accel_g | float LE | Vertical acceleration (g) |
| 17 | antenna_up | u8 | 0 = false, 1 = true |
| 18–21 | flight_time_s | float LE | Mission elapsed time |
| 22–25 | main_deploy_alt_m | float LE | Main deploy altitude |
| 26–29 | drogue_fail_vel_mps | float LE | Drogue fail threshold |
| 30–33 | drogue_fail_time_s | float LE | Drogue fail sustain time |
| 34 | apogee_pyro_ch | u8 | Apogee pyro channel (0-indexed) |
| 35 | main_pyro_ch | u8 | Main pyro channel (0-indexed) |
| 36–37 | apogee_fire_dur_ms | u16 LE | Apogee fire duration |
| 38–39 | main_fire_dur_ms | u16 LE | Main fire duration |
| 40–43 | crc32 | u32 LE | CRC-32 over bytes [0–39] |

**Total:** 44 bytes (+ COBS overhead)

**Handler:**
```c
void hil_handle_inject(const uint8_t *data, uint16_t len)
{
    if (len < 44) return;
    if (!verify_crc32(data, 40, &data[40])) return;

    /* Only accept in HIL_MODE builds */
    #ifndef HIL_MODE
    return;
    #endif

    fsm_input_t in = {0};
    uint32_t tick = read_u32_le(&data[1]);
    in.alt_m              = read_f32_le(&data[5]);
    in.vel_mps            = read_f32_le(&data[9]);
    in.vert_accel_g       = read_f32_le(&data[13]);
    in.antenna_up         = data[17] != 0;
    in.flight_time_s      = read_f32_le(&data[18]);
    in.main_deploy_alt_m  = read_f32_le(&data[22]);
    in.drogue_fail_vel_mps = read_f32_le(&data[26]);
    in.drogue_fail_time_s = read_f32_le(&data[30]);
    in.apogee_pyro_ch     = data[34];
    in.main_pyro_ch       = data[35];
    in.apogee_fire_dur_ms = read_u16_le(&data[36]);
    in.main_fire_dur_ms   = read_u16_le(&data[38]);

    fsm_set_tick(tick);
    flight_fsm_tick(&in);
    /* State is reported via normal telemetry on next tlm_tick() */
}
```

**CRITICAL:** `CMD_HIL_INJECT` is **gated behind `#ifdef HIL_MODE`**. It does not compile into flight firmware. A flight build with HIL commands would be a safety issue.

### Flight Config Struct

Create a minimal flight config struct for the parameters the FSM needs. This will eventually be populated by MC upload, but for now use compile-time defaults:

```c
typedef struct {
    float    main_deploy_alt;     /* m AGL, default 250.0 */
    float    drogue_fail_vel;     /* m/s, default 50.0    */
    float    drogue_fail_time;    /* s, default 3.0       */
    uint8_t  apogee_pyro_ch;     /* 0-indexed, default 0 */
    uint8_t  main_pyro_ch;       /* 0-indexed, default 1 */
    uint16_t apogee_fire_dur;    /* ms, default 1000     */
    uint16_t main_fire_dur;      /* ms, default 1000     */
} flight_cfg_t;

extern flight_cfg_t g_flight_cfg;
```

### Tests (Level 4)

```
TEST_HIL_01: CMD_HIL_INJECT with valid CRC is accepted, FSM state changes
TEST_HIL_02: CMD_HIL_INJECT with bad CRC is rejected (state unchanged)
TEST_HIL_03: CMD_HIL_INJECT with short packet is rejected
TEST_HIL_04: Virtual clock advances correctly across multiple HIL injections
TEST_HIL_05: Full flight replay via HIL injection sequence matches Level 2 test results

TEST_LOOP_01: flight_loop_tick() populates fsm_input_t correctly from EKF/attitude globals
TEST_LOOP_02: flight_loop_tick() does not call flight_fsm_tick() when sim is active
```

### DO NOT TOUCH

- `Core/Src/casper_pyro.c` — hardware driver
- Sensor driver files — do not modify
- Existing telemetry packing — do not modify

### DONE WHEN

1. All TEST_HIL_* pass
2. All TEST_LOOP_* pass
3. All Level 1, 2, 3 tests still pass (regression)
4. `CMD_HIL_INJECT` handler compiles only when `HIL_MODE` is defined
5. Normal flight build (without `HIL_MODE`) compiles cleanly with no HIL code
6. `build/Casper2.bin` size does not increase for non-HIL builds
7. Existing USB_MODE switching (CDC/MSC/data collection) still works

---

## CROSS-LEVEL VERIFICATION

After all 4 levels are complete, run the full test suite:

```bash
make -C Tests clean all
```

All tests from all levels must pass. Then verify the firmware builds:

```bash
# Normal flight build (no HIL)
cd Software && make clean && make -j8

# HIL build
cd Software && make clean && make -j8 EXTRA_CFLAGS=-DHIL_MODE
```

Both must compile with zero warnings (`-Wall -Werror`).

---

## REFERENCE: FSM_TRANSITION_SPEC.md

The full specification document (FSM_TRANSITION_SPEC.md) is the authoritative reference for all transition logic, threshold values, struct definitions, pseudocode, pyro safety model, and event emissions. When this PRD and the spec disagree, **the spec wins.**
