# CASPER-2 Flight-Critical Firmware PRD

**Purpose:** Implementation instructions for autonomous flight event detection, pyro deployment, and flight data logging on the C.A.S.P.E.R.-2 flight computer.

**Target:** Claude Code agents working in STM32CubeIDE C project.

**Hardware:** STM32H750VBT6, 432 MHz, 128KB flash, 512KB AXI SRAM, 128KB DTCMRAM.

**Repository:** https://github.com/AshwinVishwanath/C.A.S.P.E.R.-2

**CRITICAL: Fork from the `mc-testing` branch, NOT `main`.** The `main` branch does not contain the telemetry stack, command router, CAC handler, pyro manager, or FSM. All of that only exists on `mc-testing`.

---

## HOW TO USE THIS DOCUMENT

This PRD is divided into **6 levels**. Each level is a self-contained unit of work.

**RULES FOR AGENTS:**

1. **Complete one level fully before starting the next.** Do not read ahead.
2. **Every level has a "DONE WHEN" section.** You are not done until every criterion passes.
3. **Every level has a "DO NOT TOUCH" section.** Violating this is a blocking error.
4. **Write tests first, then implementation.** Every level includes test specifications.
5. **All code is C99, no dynamic allocation (malloc/free/calloc), no floating-point in ISRs.**
6. **All structs must be initialized to zero at declaration.**
7. **Every function that can fail returns int (0 = success, negative = error).**
8. **Use the existing FSM state defines from `tlm_types.h`.** Do NOT create a new enum. `fsm_state_t` is `typedef uint8_t`.
9. **Preserve all existing sim flight functionality.** `flight_fsm_sim_start/stop/active/get_state` must continue to work for MC bench testing.

**File locations:**
- New modules go in `Software/App/flight/` (source and headers)
- Test harnesses go in `Software/App/test/`
- Existing code is in `Software/Core/Src/`, `Software/Core/Inc/`, and `Software/App/`
- The Makefile at `Software/Makefile` must be updated to include new `.c` files

---

## REFERENCE: Existing Code You Interface With (READ ONLY)

These modules exist on the `mc-testing` branch and work. Do not modify them unless a level explicitly says to.

### FSM State Defines — `Software/App/telemetry/tlm_types.h`

**CRITICAL: Use these existing defines. Do NOT create a new enum or redefine values.**

```c
typedef uint8_t fsm_state_t;  // NOT an enum

#define FSM_STATE_PAD        0x0
#define FSM_STATE_BOOST      0x1
#define FSM_STATE_COAST      0x2   // single-stage coast, or first coast before staging
#define FSM_STATE_COAST_1    0x3   // alias — same as COAST for multi-stage pre-staging
#define FSM_STATE_SUSTAIN    0x4   // second motor burning
#define FSM_STATE_COAST_2    0x5   // coast after second motor
#define FSM_STATE_APOGEE     0x6
#define FSM_STATE_DROGUE     0x7
#define FSM_STATE_MAIN       0x8
#define FSM_STATE_RECOVERY   0x9   // exists in protocol, not used in this PRD
#define FSM_STATE_TUMBLE     0xA   // exists in protocol, not implemented yet
#define FSM_STATE_LANDED     0xB
```

These values are encoded in telemetry packets and decoded by Mission Control. Changing them breaks the protocol.

### Existing Telemetry State — `Software/App/telemetry/tlm_types.h`

```c
typedef struct {
    float alt_m;
    float vel_mps;
    float quat[4];
    float batt_v;
    float flight_time_s;
} fc_telem_state_t;

typedef struct {
    float pad_lat_deg;
    float pad_lon_deg;
    float pad_alt_m;
    float main_deploy_alt_m;
    float launch_accel_g;
    uint32_t config_hash;
} flight_config_t;  // Current version — will be expanded in Level 6
```

### Existing Flight FSM — `Software/App/fsm/flight_fsm.c`

The current FSM is sim-only (`flight_fsm_tick()` ignores sensor state). It MUST be replaced with real sensor-driven logic, BUT the following sim functions must be preserved for MC bench testing:

```c
void flight_fsm_sim_start(void);      // Start scripted sim flight (0xD0 command)
void flight_fsm_sim_stop(void);       // Stop sim, return to PAD
bool flight_fsm_sim_active(void);     // Is sim running?
void flight_fsm_sim_get_state(fc_telem_state_t *out);  // Get sim alt/vel/quat
```

When sim is active, the FSM transitions are driven by the sim script (existing behavior). When sim is NOT active, transitions are driven by real sensor data (new behavior from this PRD).

### EKF — `Software/Core/Src/casper_ekf.c`

### EKF — `Software/Core/Src/casper_ekf.c`

```c
typedef struct {
    float x[4];         // [altitude_m, velocity_mps, accel_bias, baro_bias]
    float P[4][4];      // covariance
    float baro_ref;     // ground-level baro altitude
    // ... internal matrices
} casper_ekf_t;

void casper_ekf_predict(casper_ekf_t *ekf, const casper_attitude_t *att,
                        const float accel_ms2[3], float dt);
void casper_ekf_update_baro(casper_ekf_t *ekf, float baro_alt_agl_m);
// ekf.x[0] = altitude AGL (m, positive up)
// ekf.x[1] = vertical velocity (m/s, positive up)
```

**NOTE:** GPS EKF updates (`casper_ekf_update_gps_alt/vel`) are currently called in main.c but should be REMOVED. GPS is for telemetry/recovery only, not EKF.

### Attitude — `Software/Core/Src/casper_attitude.c`

```c
typedef struct {
    float q[4];         // [w, x, y, z] body-to-NED quaternion
    bool  in_flight;
    // ... internal state
} casper_attitude_t;
```

### Pyro Manager — `Software/App/pyro/pyro_manager.c`

```c
int  pyro_mgr_set_arm(uint8_t channel, bool armed);  // 1-indexed (1-4)
int  pyro_mgr_fire(uint8_t channel, uint16_t duration_ms);  // 1-indexed
bool pyro_mgr_has_continuity(uint8_t channel);  // 1-indexed
uint8_t pyro_mgr_get_arm_bitmap(void);  // bits 0-3 = CH1-CH4
uint8_t pyro_mgr_get_cont_bitmap(void);
bool pyro_mgr_is_firing(void);
void pyro_mgr_disarm_all(void);
void pyro_mgr_set_test_mode(bool enable);
bool pyro_mgr_is_test_mode(void);
```

**IMPORTANT:** `pyro_mgr_fire()` already has internal preconditions: channel must be armed, must have continuity, and either test_mode is active OR `flight_fsm_get_state() != FSM_STATE_PAD`. The new pyro_logic layer should ARM channels when their evaluation window opens, then call `pyro_mgr_fire()` when trigger conditions are met.

### Existing Low-Level Pyro — `Software/Core/Src/casper_pyro.c`

```c
extern casper_pyro_t pyro;  // Global in main.c, accessed by pyro_manager via extern
// Contains: pyro.continuity[4], pyro.firing[4], etc.
```

### Command Stack — `Software/App/command/`

```c
// cmd_router.c — routes incoming CDC frames by message ID
void cmd_router_init(void);
void cmd_router_process(void);  // Call in superloop when CDC data available

// cac_handler.c — Command-Acknowledge-Confirm protocol for ARM/FIRE
void cac_init(void);
void cac_tick(void);  // Check confirm timeout, call in superloop

// cfg_manager.c — flight config upload/storage (currently basic, expanded in Level 6)
void cfg_manager_init(void);
const flight_config_t *cfg_get_active(void);
uint32_t cfg_get_active_hash(void);
```

### Telemetry — `Software/App/telemetry/`

```c
int  tlm_queue_event(uint8_t event_type, uint16_t event_data);
void tlm_tick(const fc_telem_state_t *tstate, const pyro_state_t *pstate, fsm_state_t fsm);
void tlm_send_gps(const fc_gps_state_t *gps);
void tlm_send_response(const uint8_t *data, int len);  // Send raw frame over CDC
// FC_EVT_STATE = 0x01, FC_EVT_PYRO = 0x02, FC_EVT_APOGEE = 0x03
// FC_EVT_BURNOUT = 0x06, FC_EVT_STAGING = 0x07
```

### Quaternion — `Software/Core/Src/casper_quat.c`

```c
void casper_quat_to_rotmat(const float q[4], float R[3][3]);  // body-to-NED, row-major
```

### Sensor Data (available in main loop)

```c
// IMU (833 Hz, interrupt-driven):
//   accel_ms2[3] — body frame, m/s²
//   gyro_rads[3] — body frame, rad/s
//   Body frame: +Z = nose (up on pad), +Y = starboard, +X = RH completion

// Barometer (~100 Hz, async):
//   baro_alt_agl — metres above ground level

// GPS (10 Hz, async):
//   gps.lat_deg, gps.lon_deg, gps.alt_msl_m, gps.vel_d_m_s
//   GPS is for telemetry/recovery ONLY. Not used in EKF.
```

---

## LEVEL 1: Flight Context and FSM Transitions

**Goal:** Create the central flight state struct and implement sensor-driven state transitions with a pure-C test harness. No pyro firing, no logging — just the state machine backbone.

### DO NOT TOUCH
- Any existing files in `Software/Core/`, `Software/App/pyro/`, `Software/App/command/`, `Software/App/telemetry/`, `Software/App/pack/`, `Software/App/diag/`
- Do not call `pyro_mgr_fire()` or any pyro functions
- Do not write to flash or SD card
- Do not modify main.c yet — this level is test-harness only

### Files to Create / Modify

**`Software/App/flight/flight_context.h`** — NEW file

**`Software/App/fsm/flight_fsm.h`** and **`Software/App/fsm/flight_fsm.c`** — REPLACE existing files. Must preserve all sim flight functions (sim_start, sim_stop, sim_active, sim_get_state) with identical behavior.

```c
#ifndef FLIGHT_CONTEXT_H
#define FLIGHT_CONTEXT_H

#include <stdint.h>
#include <stdbool.h>
#include "tlm_types.h"  // for fsm_state_t and FSM_STATE_* defines

// Do NOT redefine FSM states — use FSM_STATE_* from tlm_types.h

typedef struct {
    // FSM
    fsm_state_t state;
    uint32_t    state_entry_ms;
    uint32_t    mission_start_ms;
    bool        mission_started;

    // Motor tracking
    uint8_t     motor_count;           // burnouts detected so far
    uint8_t     motors_expected;       // 1 or 2, from config
    uint32_t    last_burnout_ms;

    // Detection flags
    bool        launch_detected;
    bool        apogee_detected;
    uint32_t    launch_timestamp_ms;
    uint32_t    apogee_timestamp_ms;
    uint32_t    landing_timestamp_ms;

    // Peak tracking
    float       max_altitude_m;
    float       max_velocity_mps;
    float       max_accel_mps2;
    float       peak_tilt_deg;         // max tilt from vertical since launch

    // Baro reference
    float       baro_ref_m;

    // Apogee voting state
    float       max_ekf_alt_m;         // independent EKF altitude peak tracker
    float       max_baro_alt_m;        // independent baro altitude peak tracker

    // Debounce timers (internal)
    uint32_t    launch_debounce_start_ms;
    bool        launch_debouncing;
    uint32_t    burnout_debounce_start_ms;
    bool        burnout_debouncing;
    uint32_t    apogee_debounce_start_ms;
    bool        apogee_debouncing;
    uint32_t    landing_debounce_start_ms;
    bool        landing_debouncing;
    float       landing_ref_alt_m;
} flight_context_t;

// Snapshot of sensor data passed to FSM each tick
typedef struct {
    float    accel_body_ms2[3];   // body frame accelerometer, m/s²
    float    ekf_alt_m;           // EKF altitude AGL
    float    ekf_vel_mps;         // EKF vertical velocity, +up
    float    baro_alt_agl_m;      // raw baro altitude AGL (independent of EKF)
    float    tilt_deg;            // current tilt from vertical, degrees
    uint32_t timestamp_ms;        // current time (HAL_GetTick or test injection)
    bool     baro_updated;        // true if baro has new data this tick
} sensor_input_t;

#endif
```

**`Software/App/fsm/flight_fsm.h`** (REPLACES existing file)

**NOTE:** This replaces the existing `flight_fsm.h`. The new version adds real sensor-driven transitions while preserving all sim flight functions. The file stays in `Software/App/fsm/`, not in `Software/App/flight/`.

```c
#ifndef APP_FSM_FLIGHT_FSM_H
#define APP_FSM_FLIGHT_FSM_H

#include "flight_context.h"
#include "tlm_types.h"

// Initialize flight context to PAD state.
// motors_expected: 1 for single stage, 2 for two stage.
void flight_fsm_init(uint8_t motors_expected);

// Advance the FSM by one tick. Call at sensor rate (e.g. 833 Hz for IMU ticks).
// When sim is active, uses sim script (existing behavior).
// When sim is NOT active, uses real sensor data from `input`.
// Returns the new state.
// Does NOT fire pyros — that is pyro_logic's job (Level 2).
fsm_state_t flight_fsm_tick(const sensor_input_t *input);

// Get read-only pointer to flight context (for telemetry, pyro_logic, etc.)
const flight_context_t *flight_fsm_get_context(void);

// Get current FSM state.
fsm_state_t flight_fsm_get_state(void);

// Get mission elapsed time in seconds. Returns 0.0 if not yet launched.
float flight_fsm_get_time_s(void);

// Force FSM to a specific state (for MC/debug-driven transitions).
void flight_fsm_force_state(fsm_state_t new_state);

// Notification from pyro_logic when a channel fires.
// Used for DROGUE→MAIN transition.
void flight_fsm_notify_pyro_event(uint8_t pyro_role, uint32_t now_ms);

// ── Sim flight (preserved from existing code) ──────────────────
void flight_fsm_sim_start(void);
void flight_fsm_sim_stop(void);
bool flight_fsm_sim_active(void);
void flight_fsm_sim_get_state(fc_telem_state_t *out);

// Utility: compute tilt from vertical in degrees from a quaternion.
float flight_fsm_tilt_from_quat(const float q[4]);

#endif
```

**`Software/App/flight/flight_fsm.c`**

Implement the following transition logic. Each transition uses a debounce pattern:

```
if (condition_met_this_tick) {
    if (!debouncing) {
        debouncing = true;
        debounce_start = now;
    } else if (now - debounce_start >= required_duration) {
        // TRANSITION
        debouncing = false;
    }
} else {
    debouncing = false;  // reset — condition must be CONTINUOUS
}
```

#### Transition: PAD → BOOST

| Parameter | Value |
|-----------|-------|
| Condition | `accel_body_ms2[2] > 39.2` (body Z = nose axis, 4g) |
| Debounce | 500 ms |
| Guard | `ekf_vel_mps > 0` |

On transition:
- Set `launch_detected = true`, `launch_timestamp_ms = timestamp_ms`
- Set `mission_started = true`, `mission_start_ms = timestamp_ms`
- Set `motor_count = 0` (first motor still burning)

#### Transition: BOOST → COAST

| Parameter | Value |
|-----------|-------|
| Condition | `accel_body_ms2[2] < 14.7` (< 1.5g on nose axis) |
| Debounce | 200 ms |
| Guard: min time in BOOST | 500 ms since state entry |
| Guard: velocity | `ekf_vel_mps > 10.0` |

On transition:
- Increment `motor_count` (first motor burned out)
- Set `last_burnout_ms = timestamp_ms`
- If `motors_expected == 1`: state = `FSM_STATE_COAST`
- If `motors_expected == 2`: state = `FSM_STATE_COAST_1` (different telemetry byte, MC knows staging is expected)

#### Transition: FSM_STATE_COAST_1 → FSM_STATE_SUSTAIN (multi-stage only)

| Parameter | Value |
|-----------|-------|
| Condition | `accel_body_ms2[2] > 29.4` (> 3g on nose axis) |
| Debounce | 200 ms |
| Guard | `motors_expected == 2 && motor_count == 1` |

On transition: state = `FSM_STATE_SUSTAIN`

Note: This transition is triggered by the physical event of the second motor igniting (which was commanded by pyro_logic firing an Ignition channel). The FSM detects the result, it does not command the ignition.

#### Transition: FSM_STATE_SUSTAIN → FSM_STATE_COAST_2

Same logic as BOOST → COAST:

| Parameter | Value |
|-----------|-------|
| Condition | `accel_body_ms2[2] < 14.7` |
| Debounce | 200 ms |
| Guard: min time in SUSTAIN | 500 ms |
| Guard: velocity | `ekf_vel_mps > 10.0` |

On transition:
- Increment `motor_count`
- Set `last_burnout_ms = timestamp_ms`
- State = `FSM_STATE_COAST_2`

#### Transition: COAST / COAST_1 / COAST_2 → APOGEE

**2-of-3 voting with debounce:**

| Vote | Condition |
|------|-----------|
| A: EKF velocity | `ekf_vel_mps < -1.0` |
| B: EKF altitude falling | `ekf_alt_m < max_ekf_alt_m - 2.0` |
| C: Baro altitude falling | `baro_alt_agl_m < max_baro_alt_m - 2.0` |

Continuously update `max_ekf_alt_m` and `max_baro_alt_m` as peak trackers.

| Parameter | Value |
|-----------|-------|
| Vote threshold | >= 2 of 3 votes true |
| Debounce | 300 ms |
| Guard: min time in coast | 2000 ms since entering COAST, COAST_1, or COAST_2 |
| Guard: max altitude | `max_ekf_alt_m > 20.0` |

On transition:
- Set `apogee_detected = true`, `apogee_timestamp_ms = timestamp_ms`

#### Transition: APOGEE → DROGUE

**Immediate.** The moment apogee is detected, transition to DROGUE. Pyro firing is handled by pyro_logic (Level 2), not here. The FSM just changes state.

On transition: state = `FSM_STATE_DROGUE` (same tick as APOGEE entry)

Implementation: When transitioning to APOGEE, immediately also transition to DROGUE in the same tick. APOGEE is a transient event state, not a dwell state.

#### Transition: DROGUE → MAIN

Handled by pyro_logic (Level 2). The FSM transitions to MAIN when pyro_logic reports that a Main-role channel has fired. For Level 1 testing, this transition is stubbed — it will only occur when driven by Level 2.

Provide a function for pyro_logic to call:

```c
void flight_fsm_notify_pyro_event(flight_context_t *ctx, uint8_t pyro_role, uint32_t now_ms);
// pyro_role: 0=Apogee, 2=Main, 4=Ignition
// When pyro_role == MAIN: transition DROGUE → MAIN
// When pyro_role == IGNITION: no FSM action (COAST→SUSTAIN detected via accel)
```

#### Transition: MAIN / DROGUE → LANDED

| Parameter | Value |
|-----------|-------|
| Condition | `fabsf(ekf_vel_mps) < 2.0 AND fabsf(ekf_alt_m - landing_ref_alt_m) < 1.0` |
| Debounce | 5000 ms |
| Guard: time since apogee | > 10000 ms |

`landing_ref_alt_m` is captured when the debounce timer starts, and reset if the condition breaks.

On transition:
- Set `landing_timestamp_ms = timestamp_ms`

#### Tilt Calculation

```c
float flight_fsm_tilt_from_quat(const float q[4]) {
    float qx = q[1], qy = q[2];
    float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
    // cos_tilt = R[2][2]. Body +Z (nose) in NED.
    // Nose up: cos_tilt = -1 → tilt = 0°
    // Nose horizontal: cos_tilt = 0 → tilt = 90°
    float tilt_rad = acosf(fmaxf(-1.0f, fminf(1.0f, -cos_tilt)));
    return tilt_rad * (180.0f / 3.14159265f);
}
```

#### Peak Tracking (every tick, after transition checks)

```c
if (ctx->mission_started) {
    if (input->ekf_alt_m > ctx->max_altitude_m) ctx->max_altitude_m = input->ekf_alt_m;
    if (input->ekf_vel_mps > ctx->max_velocity_mps) ctx->max_velocity_mps = input->ekf_vel_mps;
    float accel_mag = input->accel_body_ms2[2]; // nose axis
    if (accel_mag > ctx->max_accel_mps2) ctx->max_accel_mps2 = accel_mag;
    if (input->tilt_deg > ctx->peak_tilt_deg) ctx->peak_tilt_deg = input->tilt_deg;
    // Independent peak trackers for apogee voting
    if (input->ekf_alt_m > ctx->max_ekf_alt_m) ctx->max_ekf_alt_m = input->ekf_alt_m;
    if (input->baro_alt_agl_m > ctx->max_baro_alt_m) ctx->max_baro_alt_m = input->baro_alt_agl_m;
}
```

### Test Harness

**`Software/App/test/test_flight_fsm.c`**

Create a test harness that runs on the STM32 (or on a host PC with a minimal shim). It does NOT require real sensors. It feeds synthetic `sensor_input_t` data to `flight_fsm_tick()` and verifies state transitions.

The harness must include these test cases:

**Test 1: Normal single-stage flight**
```
Time 0-5s:     accel=0, vel=0, alt=0           → stays PAD
Time 5.0s:     accel=60 m/s² (6g), vel rising  → debounce starts
Time 5.5s:     still 6g, vel=30                 → PAD → BOOST
Time 5.5-8s:   accel=60, vel rising, alt rising → stays BOOST
Time 8.0s:     accel drops to 5 m/s²            → debounce starts
Time 8.2s:     still 5 m/s², vel=200            → BOOST → COAST
Time 8.2-20s:  vel decreasing, alt still rising → stays COAST
Time 20s:      vel=-1.5, alt<peak-2, baro<peak-2 → debounce starts
Time 20.3s:    still falling                    → COAST → APOGEE → DROGUE (same tick)
Time 20.3-60s: descending                       → stays DROGUE
Time 60-90s:   vel≈-5, alt≈50                   → stays DROGUE
Time 90s:      vel≈0, alt stable                → debounce starts
Time 95s:      still stable                     → DROGUE → LANDED
```
Verify: All transition timestamps recorded. Peak trackers correct. motor_count = 1.

**Test 2: Two-stage flight**
```
Same as Test 1 through BOOST → COAST at 8.2s.
Time 10s:      accel spikes to 40 m/s² (staging fires) → debounce starts
Time 10.2s:    still 40 m/s²                   → COAST → SUSTAIN
Time 10.2-13s: accel=40, vel rising             → stays SUSTAIN
Time 13s:      accel drops to 5 m/s²            → debounce starts
Time 13.2s:    still low                        → SUSTAIN → COAST_2
Continue with normal apogee/descent/landing.
```
Verify: motor_count = 2. SUSTAIN entered and exited correctly.

**Test 3: False launch rejection**
```
Time 0s:    accel spikes to 50 m/s² for 400ms, then drops to 0.
```
Verify: State stays PAD. 400ms < 500ms debounce, so no transition.

**Test 4: Apogee voting — single sensor disagreement**
```
In COAST state. EKF velocity says descending, EKF alt says falling, but baro says still rising.
```
Verify: 2-of-3 met → transitions to APOGEE. Baro disagreement does not block.

**Test 5: Apogee voting — only 1 sensor agrees**
```
In COAST state. Only EKF velocity says descending. EKF alt and baro both still rising.
```
Verify: 1-of-3 → does NOT transition. Stays in COAST.

**Test 6: Landing stability reset**
```
In DROGUE. Velocity drops to 0, alt stable for 4 seconds, then a gust bumps velocity to 3 m/s.
Then velocity returns to 0 and stays stable for 5 seconds.
```
Verify: First attempt resets at 4s. Second attempt completes at 5s. Total time > 9s, not 5s.

**Test 7: Guard — no apogee below 20m**
```
Short hop: launch, coast to 15m peak, descend.
```
Verify: COAST → APOGEE never fires. max_ekf_alt_m guard blocks it.

**Test 8: Staging fails — still reaches apogee**
```
Two-stage config. BOOST → COAST at 8s. No accel spike (ignition didn't fire).
Velocity decreases, eventually apogee at 18s.
```
Verify: Stays in COAST (never enters SUSTAIN). Apogee detected normally. motor_count = 1.

### DONE WHEN

- [ ] All 8 test cases pass
- [ ] `flight_context_t` is fully zero-initialized by `flight_fsm_init()`
- [ ] No dynamic memory allocation anywhere
- [ ] No calls to any hardware functions (GPIO, SPI, ADC, etc.)
- [ ] `flight_fsm_tick()` is a pure function of `(ctx, input)` — deterministic, no side effects beyond mutating `ctx`
- [ ] Compiles clean with `-Wall -Werror`

---

## LEVEL 2: Pyro Logic — Per-Channel Trigger Evaluation

**Goal:** Create the autonomous pyro firing logic that reads flight config, evaluates per-channel trigger conditions, and fires pyros based on FSM state and sensor data.

**Depends on:** Level 1 (flight_context.h, flight_fsm.h)

### DO NOT TOUCH
- `flight_fsm.c` — do not modify Level 1 code
- `Software/Core/Src/casper_pyro.c` — low-level hardware driver
- `Software/App/command/cac_handler.c` — CAC protocol
- Do not write to flash

### Files to Create

**`Software/App/flight/pyro_logic.h`**

```c
#ifndef PYRO_LOGIC_H
#define PYRO_LOGIC_H

#include "flight_context.h"
#include <stdint.h>
#include <stdbool.h>

#define PYRO_LOGIC_MAX_CHANNELS  4

typedef enum {
    PYRO_ROLE_APOGEE          = 0,
    PYRO_ROLE_APOGEE_BACKUP   = 1,
    PYRO_ROLE_MAIN            = 2,
    PYRO_ROLE_MAIN_BACKUP     = 3,
    PYRO_ROLE_IGNITION        = 4,
    PYRO_ROLE_IGNITION_BACKUP = 5,
    PYRO_ROLE_CUSTOM          = 6
} pyro_role_t;

typedef enum {
    ALT_SRC_EKF  = 0,
    ALT_SRC_BARO = 1
} altitude_src_t;

typedef enum {
    BACKUP_MODE_TIME   = 0,
    BACKUP_MODE_HEIGHT = 1
} backup_mode_t;

// Per-channel configuration (loaded from flight config upload)
typedef struct {
    pyro_role_t     role;
    uint8_t         hw_channel;              // 1-4 (matches pyro_mgr indexing)
    altitude_src_t  alt_source;

    // Trigger parameters
    float           deploy_alt_m;            // Main: fire below this altitude
    float           time_after_apogee_s;     // delay after apogee before firing
    float           fire_duration_s;         // pyro pulse width
    float           fire_delay_s;            // delay between trigger and fire
    float           min_velocity_mps;        // Ignition: minimum upward velocity
    float           min_altitude_m;          // Ignition: minimum altitude
    float           early_deploy_vel_mps;    // Main: fire early if descending faster
    uint8_t         after_motor_number;      // Ignition: which burnout to wait for
    uint8_t         max_ignition_angle_deg;  // Ignition: real-time tilt limit
    uint8_t         max_flight_angle_deg;    // Ignition: latching tilt limit
    bool            early_deploy_enabled;

    // Backup
    backup_mode_t   backup_mode;
    float           backup_value;            // seconds or metres depending on mode
} pyro_channel_config_t;

// Per-channel runtime state (managed by pyro_logic)
typedef struct {
    bool    fired;                    // has this channel fired (one-shot)
    bool    eval_enabled;             // FSM has opened evaluation window
    bool    trigger_met;              // preconditions currently satisfied
    uint32_t trigger_met_start_ms;    // when trigger was first continuously met
    bool    fire_delay_active;        // fire_delay countdown running
    uint32_t fire_delay_start_ms;
    bool    backup_active;            // backup timer running
    uint32_t backup_start_ms;
    bool    flight_angle_violated;    // LATCHING: peak tilt exceeded limit
    float   flight_angle_at_violation; // tilt when violation occurred
} pyro_channel_state_t;

// Output action from pyro_logic_tick — tells caller what to do
typedef struct {
    bool    fire;                     // true = fire this channel now
    uint8_t hw_channel;              // 1-4
    uint32_t duration_ms;
    uint8_t role;                    // for FSM notification
} pyro_action_t;

typedef struct {
    pyro_channel_config_t config[PYRO_LOGIC_MAX_CHANNELS];
    pyro_channel_state_t  state[PYRO_LOGIC_MAX_CHANNELS];
    uint8_t               num_channels;  // number of configured channels (0-4)
} pyro_logic_t;

// Initialize. Call after flight config is loaded.
void pyro_logic_init(pyro_logic_t *pl, const pyro_channel_config_t configs[],
                     uint8_t num_channels);

// Evaluate all channels. Call every sensor tick (833 Hz).
// Writes up to PYRO_LOGIC_MAX_CHANNELS actions to `actions_out`.
// Returns number of actions (channels that should fire this tick).
// Caller is responsible for actually calling pyro_mgr_fire() and
// flight_fsm_notify_pyro_event() based on returned actions.
int pyro_logic_tick(pyro_logic_t *pl, const flight_context_t *ctx,
                    const sensor_input_t *input, pyro_action_t actions_out[]);

#endif
```

**`Software/App/flight/pyro_logic.c`**

Implement the per-channel evaluation logic. For each channel, every tick:

#### Step 1: Determine if evaluation window is open

| Role | Window open when FSM state is: |
|------|-------------------------------|
| APOGEE | COAST, COAST_1, or COAST_2 |
| APOGEE_BACKUP | DROGUE (backup timer started by primary Apogee firing) |
| MAIN | DROGUE or MAIN |
| MAIN_BACKUP | MAIN (backup timer started by primary Main firing) |
| IGNITION | COAST_1 only, AND `motor_count == after_motor_number` |
| IGNITION_BACKUP | Same as IGNITION (but backup timer) |
| CUSTOM | Based on config: use same rules as closest matching role, OR always enabled in flight states |

If window is not open, skip to next channel.
If window was open and state has advanced past it (e.g., COAST_1 → APOGEE while Ignition was evaluating), **close the window permanently.** The channel will never fire.

#### Step 2: Evaluate trigger conditions (role-specific)

**APOGEE role:**
- Trigger: `ctx->apogee_detected == true`
- If `time_after_apogee_s > 0`: trigger after delay from `apogee_timestamp_ms`
- Otherwise: trigger immediately on apogee

**MAIN role:**
```c
float alt = get_channel_altitude(config, input);  // EKF or baro per config
bool altitude_trigger = (alt < config->deploy_alt_m);
bool early_trigger = (config->early_deploy_enabled &&
                      input->ekf_vel_mps < -(config->early_deploy_vel_mps));
// Early deploy guard: only after APOGEE state has been reached
bool past_apogee = (ctx->state >= FSM_STATE_APOGEE);
trigger = altitude_trigger || (early_trigger && past_apogee);
```

Guard: `timestamp_ms - ctx->apogee_timestamp_ms > 3000` (3s after apogee, avoid drogue transient)

**IGNITION role:**
```c
bool vel_ok  = (input->ekf_vel_mps > config->min_velocity_mps);
bool alt_ok  = (input->ekf_alt_m > config->min_altitude_m);
bool tilt_ok = (input->tilt_deg < config->max_ignition_angle_deg);
bool history_ok = (!state->flight_angle_violated);
trigger = vel_ok && alt_ok && tilt_ok && history_ok;
```

#### Step 3: Latching flight angle check (every tick for Ignition/Custom channels)

```c
if (ctx->mission_started && config->max_flight_angle_deg > 0) {
    if (input->tilt_deg > (float)config->max_flight_angle_deg) {
        if (!state->flight_angle_violated) {
            state->flight_angle_violated = true;
            state->flight_angle_at_violation = input->tilt_deg;
            // Caller should log this event
        }
    }
}
```

#### Step 4: Fire delay

If trigger is met and `fire_delay_s > 0`:
- Start delay timer on first trigger-met tick
- Fire after delay expires
- If trigger breaks during delay, reset timer

If `fire_delay_s == 0`: fire on the tick trigger is first met.

#### Step 5: Backup timer management

When a primary channel fires, start backup timers for corresponding backup channels:
- Apogee fires → start timer for all APOGEE_BACKUP channels
- Main fires → start timer for all MAIN_BACKUP channels

Backup channels fire unconditionally when their timer/altitude condition is met:
- `BACKUP_MODE_TIME`: fire when `now - backup_start_ms >= backup_value * 1000`
- `BACKUP_MODE_HEIGHT`: fire when `channel_altitude < backup_value`

#### Step 6: Produce action

If a channel should fire this tick: populate a `pyro_action_t` and include it in the output array. Set `state->fired = true` (one-shot: never fires again).

### Helper Function

```c
static float get_channel_altitude(const pyro_channel_config_t *config,
                                  const sensor_input_t *input) {
    if (config->alt_source == ALT_SRC_EKF) return input->ekf_alt_m;
    else return input->baro_alt_agl_m;
}
```

### Test Harness

**`Software/App/test/test_pyro_logic.c`**

Uses the same synthetic data approach as Level 1. Create a `flight_context_t`, manually set its state and fields, then call `pyro_logic_tick()` with crafted `sensor_input_t` data.

**Test 1: Apogee fires on detection**
```
Config: CH1 = Apogee, fire_duration = 1.0s, time_after_apogee = 0
Set ctx.state = FSM_STATE_COAST, ctx.apogee_detected = false.
Call tick → no action.
Set ctx.apogee_detected = true, ctx.state = FSM_STATE_DROGUE.
Call tick → action: fire CH1, 1000ms, role=APOGEE.
Call tick again → no action (already fired).
```

**Test 2: Apogee with delay**
```
Config: CH1 = Apogee, time_after_apogee = 2.0s
Set apogee_detected at T=20s.
Tick at T=21s → no action (1s < 2s delay).
Tick at T=22s → action: fire CH1.
```

**Test 3: Main deploys at altitude**
```
Config: CH2 = Main, deploy_alt = 300m, alt_source = EKF
Set ctx.state = FSM_STATE_DROGUE, apogee_timestamp = T-5s.
Input: ekf_alt = 350m → no action.
Input: ekf_alt = 290m → action: fire CH2.
```

**Test 4: Main early deploy**
```
Config: CH2 = Main, deploy_alt = 300m, early_deploy_enabled = true,
        early_deploy_vel = 40.0 m/s
Set ctx.state = FSM_STATE_DROGUE, apogee_timestamp = T-5s.
Input: ekf_alt = 2000m, ekf_vel = -50 m/s → action: fire CH2 (too fast).
```

**Test 5: Early deploy blocked before apogee**
```
Same config as Test 4, but ctx.state = FSM_STATE_COAST.
Input: vel = -50 → no action (not past apogee).
```

**Test 6: Ignition fires with all conditions met**
```
Config: CH3 = Ignition, after_motor = 1, min_vel = 50, min_alt = 500,
        max_ignition_angle = 15, max_flight_angle = 30
Set ctx.state = FSM_STATE_COAST, motor_count = 1.
Input: vel = 100, alt = 800, tilt = 5° → action: fire CH3.
```

**Test 7: Ignition blocked by real-time tilt**
```
Same config. Input: vel = 100, alt = 800, tilt = 20° (> 15).
→ no action. Tilt drops to 10° next tick → action fires.
```

**Test 8: Ignition blocked by latching flight angle**
```
Same config. At some earlier tick, tilt was 35° (> 30).
flight_angle_violated = true.
Now tilt = 5°, vel = 100, alt = 800.
→ no action. Permanently locked out.
```

**Test 9: Ignition window closes at apogee**
```
Config: CH3 = Ignition. ctx.state = FSM_STATE_COAST.
Ignition conditions never met.
ctx.state transitions to FSM_STATE_APOGEE then FSM_STATE_DROGUE.
Tick → eval_enabled = false, channel never fires.
```

**Test 10: Backup timer fires unconditionally**
```
Config: CH1 = Apogee, CH4 = Apogee Backup, backup_mode = TIME, backup_value = 3.0s
CH1 fires at T=20s → backup timer starts for CH4.
T=22s → no action (2s < 3s).
T=23s → action: fire CH4.
```

**Test 11: Backup height mode**
```
Config: CH2 = Main, CH4 = Main Backup, backup_mode = HEIGHT, backup_value = 200m
CH2 fires at T=40s. Altitude = 300m. Backup starts.
Alt = 250m → no action.
Alt = 190m → action: fire CH4.
```

### DONE WHEN

- [ ] All 11 test cases pass
- [ ] `pyro_logic_tick()` never directly calls `pyro_mgr_fire()` — it returns actions for the caller to execute
- [ ] Latching flight angle violation is permanent and logged
- [ ] Backup timers fire unconditionally (no continuity check on primary)
- [ ] Channels that have fired are never re-evaluated
- [ ] No dynamic memory allocation
- [ ] No hardware calls

---

## LEVEL 3: Flight Data Logging — Raw Binary to QSPI Flash

**Goal:** Implement flight data recording to QSPI flash using raw binary sequential writes with double-buffered non-blocking strategy and pre-launch ring buffer.

**Depends on:** Level 1 (flight_context.h for FSM state)

### DO NOT TOUCH
- `flight_fsm.c`, `pyro_logic.c` — do not modify Levels 1-2
- `Software/Core/Src/w25q512jv.c` — low-level flash driver (use its API, don't modify it)
- Do not add FATFS or any file system

### Files to Create

**`Software/App/flight/flight_log.h`**

```c
#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include "flight_context.h"
#include <stdint.h>
#include <stdbool.h>

// 32 bytes per log entry — 8 entries per flash page (256 bytes), 128 per sector (4KB)
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;              // 4 — microseconds since boot
    int16_t  accel_body_mg[3];          // 6 — body frame accel, milli-g
    int16_t  gyro_body_mdps[3];         // 6 — body frame gyro, milli-deg/s
    int16_t  ekf_alt_cm;               // 2 — EKF altitude, cm (±327m)
    int16_t  ekf_vel_cmps;             // 2 — EKF velocity, cm/s (±327 m/s)
    int16_t  baro_alt_cm;              // 2 — baro altitude AGL, cm
    uint16_t quat_packed[2];           // 4 — quaternion (smallest-three, 2 components, 
                                       //     12-bit each packed into 16-bit words; 
                                       //     3rd computed, drop index in upper 4 bits of [1])
    uint8_t  fsm_state;                // 1 — current FSM state
    uint8_t  flags;                    // 1 — bit0-3: pyro firing, bit4: baro_valid,
                                       //     bit5: gps_valid, bit6-7: log_rate
    int16_t  tilt_cdeg;               // 2 — tilt from vertical, centidegrees
    uint16_t padding;                  // 2 — alignment / future use
} flight_log_entry_t;                  // = 32 bytes total

_Static_assert(sizeof(flight_log_entry_t) == 32, "Log entry must be exactly 32 bytes");

// Summary event entry — variable length, stored in reserved flash region
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  msg_len;                  // length of message string (max 250)
    char     msg[];                    // null-terminated string, flexible array member
} flight_log_summary_entry_t;

// Log rate enum (stored in flags bits 6-7)
#define LOG_RATE_100HZ   0x00
#define LOG_RATE_50HZ    0x40
#define LOG_RATE_10HZ    0x80
#define LOG_RATE_STOPPED 0xC0

// Flash layout constants
#define FLASH_LOG_START_ADDR      0x00000000   // data log starts at beginning
#define FLASH_LOG_MAX_ADDR        0x03F00000   // ~63 MB for flight data
#define FLASH_SUMMARY_START_ADDR  0x03F00000   // last 1 MB for summary events
#define FLASH_SUMMARY_MAX_ADDR    0x04000000   // end of 64 MB
#define FLASH_SECTOR_SIZE         4096
#define FLASH_PAGE_SIZE           256
#define ENTRIES_PER_SECTOR        (FLASH_SECTOR_SIZE / sizeof(flight_log_entry_t))  // 128

// Ring buffer (pre-launch, in AXI SRAM)
#define RING_BUFFER_SECONDS       5
#define RING_BUFFER_HZ            100
#define RING_BUFFER_ENTRIES       (RING_BUFFER_SECONDS * RING_BUFFER_HZ)  // 500
#define RING_BUFFER_BYTES         (RING_BUFFER_ENTRIES * sizeof(flight_log_entry_t))  // 16000

// Initialize logging system. Must be called after w25q512jv_init().
int flight_log_init(void);

// Write one log entry to the appropriate destination based on FSM state.
// PAD state: writes to ring buffer in RAM.
// Flight states: writes to double buffer, flushes to flash when full.
// LANDED state: ignored (logging stopped).
void flight_log_write(const flight_log_entry_t *entry, fsm_state_t state);

// Call when launch is detected. Commits ring buffer contents to flash,
// then switches to direct flash logging.
void flight_log_commit_ring_buffer(void);

// Write a summary string event to the summary flash region.
void flight_log_summary(uint32_t timestamp_ms, const char *msg);

// Flush any remaining buffered data to flash. Call at landing.
void flight_log_flush(void);

// Return current write address (for telemetry / debug).
uint32_t flight_log_get_write_addr(void);

// Erase all flight log data. Erases the full flash chip.
// WARNING: Takes up to 400 seconds. Only call when USB-connected and idle.
int flight_log_erase_all(void);

#endif
```

**`Software/App/flight/flight_log.c`**

#### Architecture

```
Pre-launch (PAD state):
  flight_log_write() → ring_buffer[ring_index] = entry
  ring_index = (ring_index + 1) % RING_BUFFER_ENTRIES

Launch detected:
  flight_log_commit_ring_buffer()
    → erase first N sectors of flash
    → write ring buffer contents to flash sequentially
    → set flash_write_addr to next free address

In-flight (BOOST through MAIN):
  flight_log_write() → write_buf[buf_index++] = entry
  When buf_index == ENTRIES_PER_SECTOR (128):
    → swap buffers (double-buffer)
    → write full sector to flash at flash_write_addr
    → flash_write_addr += FLASH_SECTOR_SIZE
    → erase next sector ahead (wear leveling not needed — sequential writes)

Landing:
  flight_log_flush()
    → write any partial buffer to flash
    → flight_log_summary() with flight stats
```

#### Double Buffer Implementation

```c
// In AXI SRAM (place with __attribute__((section(".axi_sram"))) or linker config)
static flight_log_entry_t buf_a[ENTRIES_PER_SECTOR];  // 4096 bytes
static flight_log_entry_t buf_b[ENTRIES_PER_SECTOR];  // 4096 bytes

static flight_log_entry_t *active_buf = buf_a;   // sensor loop writes here
static flight_log_entry_t *flush_buf  = buf_b;   // flash write reads from here
static uint32_t buf_index = 0;                    // next write position in active_buf
static bool flush_pending = false;                // flush_buf has data to write
static uint32_t flash_write_addr = FLASH_LOG_START_ADDR;
```

When `buf_index` reaches `ENTRIES_PER_SECTOR`:
1. Swap `active_buf` and `flush_buf` pointers
2. Set `flush_pending = true`, reset `buf_index = 0`
3. In the next main loop idle period, write `flush_buf` to flash

The flash write itself (`w25q512jv_write()`) writes page-by-page (256 bytes = 8 entries). A full sector is 16 pages. This takes ~48ms total. The sensor loop continues writing to `active_buf` during this time.

**Critical:** If the sensor loop fills `active_buf` before the flash write completes, entries are dropped (not queued). Increment a `dropped_entries` counter for diagnostics. At 100 Hz logging and 48ms flush time, you get ~5 entries during a flush — well within the 128-entry buffer. This should never overflow in practice.

#### Adaptive Log Rate

The caller (main loop integration in Level 5) determines the log rate based on FSM state and velocity. `flight_log.c` itself does not decide rate — it logs whatever it receives. The rate is encoded in the `flags` field of each entry for post-flight analysis.

| FSM State | Condition | Rate | Decimation |
|-----------|-----------|------|------------|
| PAD | — | 100 Hz | log every IMU tick /8 (833/8 ≈ 104) |
| BOOST, COAST, COAST_2, SUSTAIN | — | 100 Hz | same |
| APOGEE (±5s of apogee) | — | 100 Hz | same |
| DROGUE | `|vel| > 15 m/s` | 50 Hz | every other log tick |
| DROGUE | `|vel| <= 15 m/s` | 10 Hz | every 10th log tick |
| MAIN | `|vel| > 15 m/s` | 50 Hz | re-escalate |
| MAIN | `|vel| <= 15 m/s` | 10 Hz | low rate |
| LANDED | — | 0 Hz | stopped |

#### Ring Buffer

```c
static flight_log_entry_t ring_buffer[RING_BUFFER_ENTRIES];  // 16 KB in AXI SRAM
static uint32_t ring_index = 0;
static bool ring_full = false;
```

Standard circular buffer. `flight_log_commit_ring_buffer()` writes entries in chronological order starting from the oldest entry.

#### Summary Log

Writes to the reserved flash region at the end of the chip. Each entry is a `flight_log_summary_entry_t` with a string message. Written sequentially from `FLASH_SUMMARY_START_ADDR`. Used for key events:
- "LAUNCH DETECTED"
- "BURNOUT motor=1"
- "APOGEE alt=5234m vel=0.3m/s"
- "CH1 FIRED role=APOGEE dur=1000ms"
- "FLIGHT ANGLE VIOLATED ch=3 tilt=35.2 limit=30"
- "LANDED alt=2.1m maxAlt=5234m maxVel=302m/s"

### Test Harness

**`Software/App/test/test_flight_log.c`**

For host-PC testing, create a mock `w25q512jv` that writes to a local file instead of real flash. For on-target testing, use the real flash driver.

**Test 1: Ring buffer wraps correctly**
```
Write 600 entries to ring buffer (capacity 500).
Verify: ring wraps. Oldest entry is #100, newest is #599.
Commit ring buffer. Verify: entries written to flash in order 100, 101, ..., 599.
```

**Test 2: Double buffer swap**
```
Write 128 entries (one full sector). Verify: flush_pending = true.
Call flush. Verify: 4096 bytes written to flash starting at expected address.
Write 128 more. Verify: written to second buffer, first still intact.
```

**Test 3: No drops under normal conditions**
```
Simulate 100 Hz logging for 10 seconds with 48ms flash writes.
Verify: dropped_entries == 0.
```

**Test 4: Summary log**
```
Write 5 summary strings. Read them back from summary flash region.
Verify: all strings intact, timestamps correct.
```

**Test 5: Log entry packing**
```
Create a known entry with specific values. Write to buffer, read back.
Verify: all fields decode correctly. sizeof == 32.
```

### DONE WHEN

- [ ] All 5 test cases pass
- [ ] `flight_log_entry_t` is exactly 32 bytes
- [ ] Ring buffer wraps correctly and commits in chronological order
- [ ] Double buffer swaps without blocking the caller
- [ ] No dynamic memory allocation
- [ ] Flash writes use `w25q512jv_write()` and `w25q512jv_erase_sector()` APIs only

---

## LEVEL 4: Post-Flight Readout

**Goal:** Implement USB-based flight log retrieval. Convert the raw binary flash contents to a host-readable format via USB CDC (streaming) or USB MSC (mount as drive).

**Depends on:** Level 3 (flight_log.h, flash layout)

### DO NOT TOUCH
- Levels 1-3 code
- `Software/Core/Src/usbd_cdc_if.c` — existing CDC implementation (use its API)

### Files to Create

**`Software/App/flight/flight_readout.h`**

```c
#ifndef FLIGHT_READOUT_H
#define FLIGHT_READOUT_H

#include <stdint.h>

// Stream raw binary log data over USB CDC.
// Sends a header (magic + entry count + entry size), then all entries sequentially.
// Blocking — call only when system is in LANDED or PAD state with USB connected.
int flight_readout_stream_raw(void);

// Stream summary log over USB CDC as null-terminated strings.
int flight_readout_stream_summary(void);

// Return total number of log entries stored in flash.
uint32_t flight_readout_get_entry_count(void);

#endif
```

**`Software/App/flight/flight_readout.c`**

The readout protocol is simple:

```
Raw stream format:
  [4 bytes] Magic: 0x43 0x41 0x53 0x50  ("CASP")
  [4 bytes] Entry count (u32 LE)
  [4 bytes] Entry size (u32 LE, should be 32)
  [4 bytes] CRC-32 of header (12 bytes above)
  [N × 32 bytes] Log entries, sequential
  [4 bytes] CRC-32 of all entry data

Summary stream format:
  [4 bytes] Magic: 0x53 0x55 0x4D 0x4D  ("SUMM")
  [4 bytes] Total size in bytes (u32 LE)
  [N bytes] Summary entries (length-prefixed strings)
  [4 bytes] CRC-32 of all summary data
```

**Host-side decoder:** Provide a Python script `tools/decode_flight_log.py` that:
1. Reads the raw binary stream from a serial port or file
2. Validates CRC
3. Decodes each 32-byte entry into human-readable CSV
4. Outputs `flight_data.csv` with columns: timestamp_us, ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps, ekf_alt_cm, ekf_vel_cmps, baro_alt_cm, fsm_state, flags, tilt_cdeg

### Test Harness

**Test 1: Round-trip encode/decode**
```
Create 100 known entries. Write to flash via flight_log. Read back via flight_readout.
Pipe through Python decoder. Verify: all values match originals.
```

**Test 2: CRC validation**
```
Corrupt one byte in the stream. Verify: Python decoder reports CRC failure.
```

### DONE WHEN

- [ ] Raw binary stream decodes to correct CSV via Python script
- [ ] CRC catches corruption
- [ ] Summary strings are readable
- [ ] Works over USB CDC at 115200 baud without data loss

---

## LEVEL 5: Main Loop Integration

**Goal:** Wire Levels 1-4 into the actual `main.c` superloop. This is where everything comes together with real hardware.

**Depends on:** All previous levels passing their tests.

### Files to Modify

**`Software/Core/Src/main.c`** — the main superloop

### Integration Architecture

```c
// Top of main.c — the flight dashboard
#include "flight_context.h"
#include "flight_fsm.h"
#include "pyro_logic.h"
#include "flight_log.h"

static flight_context_t flight;
static pyro_logic_t     pyro;
static flight_log_entry_t log_entry;

// Log rate decimation
static uint8_t  log_decimation_counter = 0;
static uint8_t  log_decimation_factor  = 8;  // 833/8 ≈ 104 Hz → ~100 Hz
```

**Inside `main()`, after all existing peripheral init:**

```c
// After existing sensor init, EKF init, attitude init, pyro init:
flight_fsm_init(&flight, config.motors_expected);  // 1 or 2 from config
pyro_logic_init(&pyro, config.pyro_channels, config.num_pyro_channels);
flight_log_init();
```

**Main loop structure:**

```c
while (1) {
    // === Layer 1: Always runs ===
    cmd_router_process();
    cac_tick();
    pyro_mgr_tick();

    // === Layer 2: IMU-driven (833 Hz) ===
    if (imu.data_ready) {
        imu.data_ready = false;

        // Read sensors
        float accel_ms2[3], gyro_rads[3];
        // ... existing sensor read + axis remap code ...

        // Read mag if available
        float *mag_ptr = NULL;
        float mag_cal_ut[3];
        if (mag.data_ready) { /* ... existing mag read ... */ mag_ptr = mag_cal_ut; }

        // Attitude update (existing)
        casper_att_update(&att, gyro_rads, accel_ms2, mag_ptr, dt);

        // EKF predict (existing)
        casper_ekf_predict(&ekf, &att, accel_ms2, dt);

        // Build sensor input for FSM
        sensor_input_t si = {0};
        si.accel_body_ms2[0] = accel_ms2[0];
        si.accel_body_ms2[1] = accel_ms2[1];
        si.accel_body_ms2[2] = accel_ms2[2];
        si.ekf_alt_m    = ekf.x[0];
        si.ekf_vel_mps  = ekf.x[1];
        si.baro_alt_agl_m = last_baro_alt_agl;  // updated async in Layer 3
        si.tilt_deg     = flight_fsm_tilt_from_quat(att.q);
        si.timestamp_ms = HAL_GetTick();
        si.baro_updated = false;  // set true in Layer 3 when baro updates

        // FSM tick
        fsm_state_t prev_state = flight.state;
        flight_fsm_tick(&flight, &si);

        // Pyro logic tick
        pyro_action_t actions[PYRO_LOGIC_MAX_CHANNELS];
        int num_actions = pyro_logic_tick(&pyro, &flight, &si, actions);
        for (int i = 0; i < num_actions; i++) {
            pyro_mgr_fire(actions[i].hw_channel, actions[i].duration_ms);
            flight_fsm_notify_pyro_event(&flight, actions[i].role, si.timestamp_ms);
            flight_log_summary(si.timestamp_ms, "PYRO FIRED ch=%d role=%d dur=%lu",
                               actions[i].hw_channel, actions[i].role, actions[i].duration_ms);
            tlm_queue_event(FC_EVT_PYRO, actions[i].hw_channel);
        }

        // On launch: commit ring buffer
        if (flight.state != FSM_STATE_PAD && prev_state == FSM_STATE_PAD) {
            flight_log_commit_ring_buffer();
        }

        // Peak tilt tracking (for telemetry)
        if (si.tilt_deg > flight.peak_tilt_deg) {
            flight.peak_tilt_deg = si.tilt_deg;
        }

        // === Data logging (decimated) ===
        log_decimation_counter++;
        uint8_t current_factor = get_log_decimation(flight.state, si.ekf_vel_mps);
        if (log_decimation_counter >= current_factor) {
            log_decimation_counter = 0;
            // Build log entry
            flight_log_entry_t entry = {0};
            entry.timestamp_us = /* DWT or TIM counter */;
            entry.accel_body_mg[0] = (int16_t)(accel_ms2[0] / 9.80665f * 1000.0f);
            entry.accel_body_mg[1] = (int16_t)(accel_ms2[1] / 9.80665f * 1000.0f);
            entry.accel_body_mg[2] = (int16_t)(accel_ms2[2] / 9.80665f * 1000.0f);
            entry.gyro_body_mdps[0] = (int16_t)(gyro_rads[0] * 180.0f / 3.14159f * 1000.0f);
            // ... etc
            entry.ekf_alt_cm  = (int16_t)(ekf.x[0] * 100.0f);
            entry.ekf_vel_cmps = (int16_t)(ekf.x[1] * 100.0f);
            entry.baro_alt_cm = (int16_t)(last_baro_alt_agl * 100.0f);
            entry.fsm_state   = (uint8_t)flight.state;
            entry.tilt_cdeg   = (int16_t)(si.tilt_deg * 100.0f);
            // ... pack quaternion, flags ...
            flight_log_write(&entry, flight.state);
        }

        // On landing: flush and write summary
        if (flight.state == FSM_STATE_LANDED && prev_state != FSM_STATE_LANDED) {
            flight_log_summary(si.timestamp_ms, "LANDED maxAlt=%.1f maxVel=%.1f peakTilt=%.1f",
                               flight.max_altitude_m, flight.max_velocity_mps, flight.peak_tilt_deg);
            flight_log_flush();
        }
    }

    // === Layer 3: Async sensor updates ===
    if (ms5611_tick(&baro) == 1) {
        float baro_alt = ms5611_get_altitude(&baro, 1013.25f) - flight.baro_ref_m;
        last_baro_alt_agl = baro_alt;
        casper_ekf_update_baro(&ekf, baro_alt);
        // baro_updated flag for next IMU tick
    }

    if (max_m10m_tick(&gps) == 1) {
        // GPS for telemetry only — not fed to EKF
    }

    // === Layer 4: Periodic telemetry (existing tlm_tick) ===
    tlm_tick();
}
```

**Log decimation helper:**

```c
static uint8_t get_log_decimation(fsm_state_t state, float vel_mps) {
    switch (state) {
        case FSM_STATE_PAD:
        case FSM_STATE_BOOST:
        case FSM_STATE_COAST:
        case FSM_STATE_SUSTAIN:
        case FSM_STATE_COAST_2:
        case FSM_STATE_APOGEE:
            return 8;   // 833/8 ≈ 104 Hz
        case FSM_STATE_DROGUE:
        case FSM_STATE_MAIN:
            if (fabsf(vel_mps) > 15.0f) return 16;   // ~52 Hz
            else return 83;                            // ~10 Hz
        case FSM_STATE_LANDED:
            return 255;  // effectively stopped
        default:
            return 8;
    }
}
```

### Test Procedure (requires hardware)

This is a bench integration test. You need the FC board, USB cable, and a way to observe LED behavior.

**Test A: Power-on → PAD state**
1. Power on FC
2. Verify: LED blinks at pad-rate pattern
3. Connect USB, verify handshake
4. Observe telemetry: FSM state = PAD, alt ≈ 0, vel ≈ 0

**Test B: Sim flight end-to-end**
1. Trigger SIM_FLIGHT via MC or USB command
2. Observe: FSM transitions through BOOST → COAST → APOGEE → DROGUE → LANDED
3. Verify: telemetry shows state changes at expected sim timestamps
4. After LANDED: verify flash contains log data via flight_readout

**Test C: Pyro test mode + firing**
1. Enter test mode via CAC
2. Arm CH1, verify continuity LED
3. Fire CH1, verify 50ms pulse (test mode cap)
4. Verify: FC_EVT_PYRO event in telemetry

**Test D: Log readout**
1. After a sim flight, connect USB
2. Run `tools/decode_flight_log.py`
3. Verify: CSV output contains all flight phases with correct timestamps

### DONE WHEN

- [ ] Sim flight produces correct FSM transitions in telemetry
- [ ] Flash log contains data from all flight phases
- [ ] Python decoder produces valid CSV from flash readout
- [ ] Summary log contains launch, apogee, pyro fire, and landing events
- [ ] No watchdog resets or hard faults during sim flight
- [ ] Pyro test mode still works (existing CAC functionality not broken)
- [ ] USB telemetry still works at 10 Hz during flight
- [ ] Build fits in 128 KB flash

---

## LEVEL 6: Flight Configuration Upload and Storage

**Goal:** Implement the flight config binary format parsing on the FC side, storing it in RAM for pyro_logic to consume. The MC uploads config via the existing 0xC1 CMD_UPLOAD path.

**Depends on:** Level 2 (pyro_channel_config_t), Level 5 (integration)

### DO NOT TOUCH
- Levels 1-5 code (except adding config load call to main.c init)

### Files to Create

**`Software/App/flight/flight_config.h`**

```c
#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

#include "pyro_logic.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t  config_version;        // must be 0x01
    uint8_t  motors_expected;       // 1 or 2

    // Per-channel configs (populated from binary payload)
    pyro_channel_config_t channels[PYRO_LOGIC_MAX_CHANNELS];
    uint8_t num_channels;

    // Pad location (for GPS delta computation)
    float    pad_lat_deg;
    float    pad_lon_deg;
    float    pad_alt_msl_m;

    // Pre-flight thresholds
    float    min_batt_v;
    float    min_integrity_pct;

    // Config integrity
    uint32_t config_hash;           // CRC-32 of serialized config
    bool     config_loaded;         // true after successful parse
} flight_config_t;

// Parse a 163-byte binary config payload into the struct.
// Returns 0 on success, -1 on CRC failure, -2 on version mismatch.
int flight_config_parse(flight_config_t *cfg, const uint8_t *data, uint16_t len);

// Return a default config (single-stage, CH1=Apogee, CH2=Main, no ignition).
void flight_config_default(flight_config_t *cfg);

// Get the config hash for handshake/ACK responses.
uint32_t flight_config_get_hash(const flight_config_t *cfg);

#endif
```

Implement `flight_config_parse()` to deserialize the binary format defined in INTERFACE_SPEC.md §15.1. Map each 32-byte per-channel block into a `pyro_channel_config_t`. Derive `motors_expected` from the number of IGNITION-role channels (0 ignition channels = 1 motor, 1+ = 2 motors).

### Test Harness

**Test 1: Parse valid config**
```
Construct a 163-byte buffer matching the spec. Parse it.
Verify: all fields match expected values. config_loaded = true.
```

**Test 2: CRC failure**
```
Flip one bit in the payload. Parse.
Verify: returns -1. config_loaded = false.
```

**Test 3: Default config**
```
Call flight_config_default(). Verify: single stage, CH1=Apogee, CH2=Main 300m.
```

**Test 4: motors_expected derivation**
```
Config with CH3 = Ignition role. Parse.
Verify: motors_expected = 2.
Config with no Ignition channels. Parse.
Verify: motors_expected = 1.
```

### DONE WHEN

- [ ] 163-byte config parses correctly
- [ ] CRC-32 validated
- [ ] `motors_expected` derived from Ignition channel presence
- [ ] Default config provides safe single-stage defaults
- [ ] Integrates with `pyro_logic_init()` and `flight_fsm_init()` in main.c

---

## APPENDIX A: Threshold Constants Reference

All magic numbers in one place. These should be `#define` constants in a shared header (`Software/App/flight/flight_constants.h`).

```c
// Launch detection
#define LAUNCH_ACCEL_THRESHOLD_MS2     39.2f    // 4g
#define LAUNCH_DEBOUNCE_MS             500

// Burnout detection
#define BURNOUT_ACCEL_THRESHOLD_MS2    14.7f    // 1.5g
#define BURNOUT_DEBOUNCE_MS            200
#define BURNOUT_MIN_BOOST_TIME_MS      500
#define BURNOUT_MIN_VELOCITY_MPS       10.0f

// Staging detection
#define STAGING_ACCEL_THRESHOLD_MS2    29.4f    // 3g
#define STAGING_DEBOUNCE_MS            200

// Apogee detection
#define APOGEE_VEL_THRESHOLD_MPS       -1.0f
#define APOGEE_ALT_MARGIN_M            2.0f
#define APOGEE_DEBOUNCE_MS             300
#define APOGEE_MIN_COAST_TIME_MS       2000
#define APOGEE_MIN_ALTITUDE_M          20.0f

// Landing detection
#define LANDING_VEL_THRESHOLD_MPS      2.0f
#define LANDING_ALT_STABLE_M           1.0f
#define LANDING_DEBOUNCE_MS            5000
#define LANDING_MIN_TIME_AFTER_APOGEE_MS 10000

// Main deploy safety
#define MAIN_DEPLOY_MIN_TIME_AFTER_APOGEE_MS 3000

// Logging
#define LOG_HIGH_VEL_THRESHOLD_MPS     15.0f
```

## APPENDIX B: File Tree

```
Software/
├── App/
│   ├── command/          # EXISTING — do not modify
│   │   ├── cac_handler.c/h
│   │   ├── cfg_manager.c/h      # Expanded in Level 6
│   │   └── cmd_router.c/h
│   ├── diag/             # EXISTING — do not modify
│   │   └── self_test.c/h
│   ├── flight/           # NEW — all new flight-critical code
│   │   ├── flight_constants.h
│   │   ├── flight_context.h
│   │   ├── pyro_logic.c/h
│   │   ├── flight_log.c/h
│   │   └── flight_readout.c/h
│   ├── fsm/              # EXISTING — flight_fsm REPLACED with real logic
│   │   ├── flight_fsm.c         # Rewritten (preserve sim functions)
│   │   └── flight_fsm.h         # Rewritten (new API + sim compat)
│   ├── pack/             # EXISTING — do not modify
│   │   ├── quat_pack.c/h
│   │   └── status_pack.c/h
│   ├── pyro/             # EXISTING — do not modify
│   │   └── pyro_manager.c/h
│   ├── telemetry/        # EXISTING — do not modify (except tlm_types.h if needed)
│   │   ├── cobs.c/h
│   │   ├── crc32_hw.c/h
│   │   ├── tlm_manager.c/h
│   │   └── tlm_types.h
│   └── test/             # NEW — test harnesses
│       ├── test_flight_fsm.c
│       ├── test_pyro_logic.c
│       ├── test_flight_log.c
│       └── test_flight_config.c
├── Core/
│   ├── Src/              # EXISTING — modify main.c only in Level 5
│   │   ├── main.c
│   │   ├── casper_ekf.c
│   │   ├── casper_attitude.c
│   │   ├── casper_pyro.c
│   │   ├── casper_quat.c
│   │   ├── ms5611.c
│   │   ├── lsm6dso32.c
│   │   ├── adxl372.c
│   │   ├── w25q512jv.c
│   │   ├── max_m10m.c
│   │   └── mmc5983ma.c
│   └── Inc/
├── Makefile              # UPDATE: add new .c files to C_SOURCES
└── tools/                # NEW — host-side utilities
    └── decode_flight_log.py
```

## APPENDIX C: main.c Modifications Checklist (Level 5)

When modifying `Software/Core/Src/main.c`:

1. **REMOVE** GPS EKF update calls (lines calling `casper_ekf_update_gps_alt/vel`)
2. **ADD** includes for `flight_context.h`, `pyro_logic.h`, `flight_log.h`
3. **ADD** `flight_fsm_init(motors_expected)` call after existing init sequence
4. **ADD** `pyro_logic_init()` call after FSM init
5. **ADD** `flight_log_init()` call after flash init
6. **MODIFY** the FSM tick section to build `sensor_input_t` and pass to new `flight_fsm_tick()`
7. **ADD** pyro_logic_tick() call after FSM tick, execute returned actions
8. **ADD** flight_log_write() call with decimation logic
9. **PRESERVE** all existing `#ifdef` paths (MAG_CAL, MAG_VAL, USB_MODE==2)
10. **PRESERVE** all existing CDC debug output under `#ifdef CDC_OUTPUT`
11. **PRESERVE** sim flight override (`if (flight_fsm_sim_active())`)
12. **UPDATE** `Software/Makefile` C_SOURCES to include all new `.c` files
