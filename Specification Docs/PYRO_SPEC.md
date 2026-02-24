# PYRO_SPEC.md — Pyro, FSM, and Command System

> FC-side implementation reference for the pyro firing system, flight state machine,
> command routing, and safety interlocks.
>
> **Companion docs:** INTERFACE_SPEC.md (packet byte layouts, MC-side protocol),
> HARDWARE_SPEC.md (GPIO/ADC pin assignments), SENSOR_SPEC.md (sensor drivers).

---

## 1  System Architecture

The pyro/command system is a four-layer stack:

```
   USB CDC (COBS frames from MC)
        │
   ┌────▼────────────────────┐
   │  cmd_router.c           │  Layer 1 — COBS decode + msg_id dispatch
   │  (ring buffer → decode  │
   │   → dispatch_frame)     │
   └────┬────────────────────┘
        │
   ┌────▼────────────────────┐
   │  cac_handler.c          │  Layer 2 — CAC protocol (CMD→ACK→CONFIRM)
   │  (magic/CRC/nonce       │            test mode, timeouts
   │   validation, ACK/NACK) │
   └────┬────────────────────┘
        │
   ┌────▼────────────────────┐
   │  pyro_manager.c         │  Layer 3 — Safety layer (arm, preconditions)
   │  (arm/fire gates,       │
   │   test mode cap, FSM)   │
   └────┬────────────────────┘
        │
   ┌────▼────────────────────┐
   │  casper_pyro.c          │  Layer 4 — Hardware driver (GPIO, ADC, LEDs)
   │  (fire pins, ADC reads, │
   │   auto-stop timer)      │
   └─────────────────────────┘
```

### Source Files

| Layer | File | Role |
|-------|------|------|
| 1 | `App/command/cmd_router.c` | COBS accumulation, decode, msg_id switch |
| 2 | `App/command/cac_handler.c/h` | CAC state machine, ARM/FIRE/TESTMODE/CONFIRM/ABORT |
| 2 | `App/fsm/flight_fsm.c/h` | Flight state machine, sim flight profile |
| 3 | `App/pyro/pyro_manager.c/h` | Safety gate layer over casper_pyro |
| 4 | `Core/Src/casper_pyro.c` / `Core/Inc/casper_pyro.h` | Raw hardware control |
| — | `App/pack/status_pack.c/h` | STATUS bitmap builder for telemetry |
| — | `App/telemetry/tlm_types.h` | All constants, message IDs, event types |

---

## 2  Pyro Hardware — casper_pyro

### 2.1  Channel Map

4 pyro channels, each with a fire MOSFET, continuity ADC, and indicator LED:

| Channel | Fire Pin | Fire GPIO | LED Pin | LED GPIO | ADC Channel | ADC Periph |
|---------|----------|-----------|---------|----------|-------------|------------|
| CH 1 | PD10 | `PY1_Pin` | PA10 | `CONT_YN_1_Pin` | ADC1_CH4 | hadc1 |
| CH 2 | PD9 | `PY2_Pin` | PB14 | `CONT_YN_2_Pin` | ADC1_CH3 | hadc1 |
| CH 3 | PD8 | `PY3_Pin` | PE8 | `CONT_YN_3_Pin` | ADC3_CH1 | hadc3 |
| CH 4 | PB15 | `PY4_Pin` | PE7 | `CONT_YN_4_Pin` | ADC2_CH10 | hadc2 |

> **Note:** Channels 1 and 2 share ADC1; channels 3 and 4 use ADC3 and ADC2 respectively.
> The `adc_idx` field in the HW lookup table selects which handle: 0=hadc1, 1=hadc2, 2=hadc3.

### 2.2  Continuity Sensing

ADC reads are **blocking** (poll-for-conversion) with a 2ms timeout per channel:

```c
/* ADC config for each read */
SamplingTime = ADC_SAMPLETIME_64CYCLES_5    /* longer sample for analog sense */
SingleDiff   = ADC_SINGLE_ENDED
```

Continuity threshold (16-bit ADC, 3.3V reference):
```c
#define PYRO_CONTINUITY_THRESHOLD  8000    /* tune empirically per e-match */
```

A reading above 8000 indicates the e-match circuit is closed (continuity present).
The `continuity[ch]` boolean array is updated every `casper_pyro_tick()` call.

### 2.3  ADC Calibration

All three ADC peripherals are calibrated during `casper_pyro_init()`:
```c
HAL_ADCEx_Calibration_Start(hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
HAL_ADCEx_Calibration_Start(hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
HAL_ADCEx_Calibration_Start(hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
```

> **STM32H7 requirement:** ADC calibration must run before the first conversion or
> readings will be incorrect. This is done once at init, not per-read.

### 2.4  casper_pyro_t Struct

```c
typedef struct {
    bool     continuity[4];          /* true if ADC > threshold */
    uint16_t adc_raw[4];             /* raw 16-bit ADC reading  */
    bool     firing[4];              /* true while fire pulse active */
    uint32_t fire_start_ms[4];       /* HAL_GetTick() at fire start */
    uint32_t fire_duration_ms[4];    /* requested pulse width */
    ADC_HandleTypeDef *hadc1, *hadc2, *hadc3;
} casper_pyro_t;
```

### 2.5  Driver API

| Function | Description |
|----------|-------------|
| `casper_pyro_init(p, hadc1, hadc2, hadc3)` | Zero struct, store handles, force all fire pins LOW, calibrate ADCs |
| `casper_pyro_fire(p, ch, duration_ms)` → `bool` | Set fire pin HIGH, start timer. Returns false if ch >= 4 |
| `casper_pyro_stop(p, ch)` | Set fire pin LOW, clear firing flag |
| `casper_pyro_stop_all(p)` | Stop all 4 channels |
| `casper_pyro_tick(p)` | (1) Read all ADCs → update continuity, (2) Update LEDs, (3) Auto-stop expired fires |

### 2.6  Fire Sequence (Hardware Level)

```
casper_pyro_fire(ch=0, dur=1000ms)
  → p->firing[0] = true
  → p->fire_start_ms[0] = HAL_GetTick()
  → p->fire_duration_ms[0] = 1000
  → HAL_GPIO_WritePin(PY1_Port, PY1_Pin, SET)    ← MOSFET ON

casper_pyro_tick() [called every ~100ms]:
  → if (now - fire_start_ms[0] >= 1000ms)
     → casper_pyro_stop(0)
       → HAL_GPIO_WritePin(PY1_Port, PY1_Pin, RESET)  ← MOSFET OFF
       → p->firing[0] = false
```

### 2.7  Continuity LED Behavior

Each tick updates the corresponding LED to reflect continuity:
- **LED ON** = continuity detected (e-match circuit intact)
- **LED OFF** = no continuity (open circuit or no e-match)

This runs regardless of arm/fire state — LEDs always show live continuity status.

---

## 3  Pyro Manager — Safety Layer

`pyro_manager.c` wraps `casper_pyro` with safety gates. It enforces all arming
and firing preconditions required by the PRD.

### 3.1  Private State

```c
static bool s_armed[4];     /* per-channel arm state */
static bool s_test_mode;    /* set by cac_handler via pyro_mgr_set_test_mode() */
```

### 3.2  Initialization

```c
pyro_mgr_init()
  → all s_armed[] = false
  → s_test_mode = false
```

Must be called **after** `casper_pyro_init()` since it references the global `pyro` struct.

### 3.3  Arming Preconditions

`pyro_mgr_set_arm(channel, armed)` — channel is **1-indexed** (1–4):

| Condition | Check |
|-----------|-------|
| Channel range | 1 ≤ channel ≤ 4 |
| Continuity (arm only) | `pyro.continuity[idx]` must be true |
| Disarm side-effect | Stops any active fire on that channel |

Returns 0 on success, -1 on precondition failure.

### 3.4  Firing Preconditions

`pyro_mgr_fire(channel, duration_ms)` — channel is **1-indexed**:

| Condition | Check | On Failure |
|-----------|-------|------------|
| Channel range | 1 ≤ channel ≤ 4 | return -1 |
| Armed | `s_armed[idx]` | return -1 |
| Continuity | `pyro.continuity[idx]` | return -1 |
| FSM/test gate | `s_test_mode OR fsm_state != PAD` | return -1 |
| Duration cap | `min(duration_ms, PYRO_MAX_FIRE_MS)` | clamped to 2000ms |
| Test mode cap | if test mode: `min(dur, 50)` | clamped to 50ms |

The full interlock chain: **armed AND continuity AND (test_mode OR past PAD state)**.

### 3.5  Test Mode Duration Cap

In test mode, fire duration is hard-capped to **50ms** regardless of what MC requests.
This prevents accidental sustained burns during bench testing:

```c
if (s_test_mode && dur > 50) {
    dur = 50;
}
```

Normal flight fires (FSM past PAD) use the full requested duration up to `PYRO_MAX_FIRE_MS` (2000ms).

### 3.6  Test Mode Lifecycle

```
pyro_mgr_set_test_mode(true)   ← called by cac_handler on 0x82 toggle ON
  → s_test_mode = true

pyro_mgr_set_test_mode(false)  ← called on toggle OFF or 60s timeout
  → s_test_mode = false
  → pyro_mgr_disarm_all()      ← disarms ALL channels + stops all fires
```

Exiting test mode automatically disarms everything as a safety measure.

### 3.7  Bitmap Getters

| Function | Returns |
|----------|---------|
| `pyro_mgr_get_arm_bitmap()` | Bits 0–3 = channels 1–4 armed |
| `pyro_mgr_get_cont_bitmap()` | Bits 0–3 = channels 1–4 continuity |
| `pyro_mgr_is_firing()` | true if any channel is currently firing |
| `pyro_mgr_is_test_mode()` | true if test mode active |

### 3.8  Full API Reference

| Function | Description |
|----------|-------------|
| `pyro_mgr_init()` | Reset all arm states, clear test mode |
| `pyro_mgr_tick()` | Delegates to `casper_pyro_tick()` for ADC/LED/auto-stop |
| `pyro_mgr_set_arm(ch, armed)` | Arm/disarm with precondition checks (1-indexed) |
| `pyro_mgr_has_continuity(ch)` | Query continuity for channel (1-indexed) |
| `pyro_mgr_fire(ch, dur_ms)` | Fire with full interlock chain (1-indexed) |
| `pyro_mgr_get_arm_bitmap()` | 4-bit arm bitmap |
| `pyro_mgr_get_cont_bitmap()` | 4-bit continuity bitmap |
| `pyro_mgr_is_firing()` | Any channel firing? |
| `pyro_mgr_disarm_all()` | Stop all fires + disarm all channels |
| `pyro_mgr_set_test_mode(en)` | Enable/disable test mode (disarms all on disable) |
| `pyro_mgr_is_test_mode()` | Query test mode state |

---

## 4  Flight State Machine

### 4.1  FSM States

12 states encoded as 4-bit values (fits in the STATUS bitmap):

| Value | Name | Description |
|-------|------|-------------|
| 0x0 | `FSM_STATE_PAD` | On launch pad, waiting for ignition |
| 0x1 | `FSM_STATE_BOOST` | Motor burning, accelerating |
| 0x2 | `FSM_STATE_COAST` | Post-burnout coast (single-stage) |
| 0x3 | `FSM_STATE_COAST_1` | First coast phase (multi-stage) |
| 0x4 | `FSM_STATE_SUSTAIN` | Sustainer motor phase (multi-stage) |
| 0x5 | `FSM_STATE_COAST_2` | Second coast phase (multi-stage) |
| 0x6 | `FSM_STATE_APOGEE` | Peak altitude detected |
| 0x7 | `FSM_STATE_DROGUE` | Drogue chute deployed, descending fast |
| 0x8 | `FSM_STATE_MAIN` | Main chute deployed, descending slow |
| 0x9 | `FSM_STATE_RECOVERY` | Near ground, final descent |
| 0xA | `FSM_STATE_TUMBLE` | Attitude unstable (safety deploy) |
| 0xB | `FSM_STATE_LANDED` | On ground, mission complete |

### 4.2  Internal State

```c
static fsm_state_t s_state;            /* current state */
static uint32_t    s_state_entry_ms;   /* HAL_GetTick() when entered current state */
static uint32_t    s_mission_start_ms; /* HAL_GetTick() at first non-PAD state */
static bool        s_mission_started;  /* true after first transition from PAD */
static bool        s_sim_active;       /* sim flight in progress */
static uint32_t    s_sim_start_ms;     /* sim start timestamp */
```

### 4.3  State Transitions

Every `flight_fsm_tick()` call checks if the current sim time has passed a waypoint
and transitions accordingly. The `transition_to()` helper:

```c
static void transition_to(fsm_state_t new_state)
{
    if (new_state == s_state) return;       /* no-op if same state */
    s_state = new_state;
    s_state_entry_ms = HAL_GetTick();
    tlm_queue_event(FC_EVT_STATE, (uint16_t)new_state);     /* always emit */
    if (new_state == FSM_STATE_APOGEE) {
        tlm_queue_event(FC_EVT_APOGEE, 0);                  /* apogee event */
    }
}
```

Events emitted on transition:
- **FC_EVT_STATE (0x01):** Every state change, data = new state value
- **FC_EVT_APOGEE (0x03):** Additionally emitted when entering APOGEE state

### 4.4  Sim Flight Profile

Time-scripted waypoints with linear interpolation between them:

| Time (s) | Altitude (m) | Velocity (m/s) | State |
|----------|-------------|----------------|-------|
| 0.0 | 0 | 0 | PAD |
| 0.5 | 5 | 50 | BOOST |
| 3.0 | 600 | 300 | BOOST |
| 3.5 | 750 | 250 | COAST |
| 12.0 | 5150 | 0 | APOGEE |
| 12.5 | 5100 | -20 | DROGUE |
| 45.0 | 300 | -8 | MAIN |
| 90.0 | 10 | -5 | RECOVERY |
| 180.0 | 0 | 0 | LANDED |

Sim ends 5 seconds after reaching LANDED.

### 4.5  Sim Telemetry Generation

`flight_fsm_sim_get_state()` fills an `fc_telem_state_t` with interpolated values:
- **alt_m, vel_mps:** Linear interpolation between bounding waypoints
- **flight_time_s:** Elapsed since sim start
- **quat[4]:** Slowly rotating yaw at 10 deg/s (visual indicator that sim is active)

### 4.6  FSM API

| Function | Description |
|----------|-------------|
| `flight_fsm_init()` | Start in PAD, clear mission timer |
| `flight_fsm_tick(state)` | Advance sim if active, return current state |
| `flight_fsm_get_state()` | Return current state |
| `flight_fsm_get_time_s()` | Mission elapsed time (0 if not launched) |
| `flight_fsm_force_state(st)` | Force to specific state (MC/debug), emits event |
| `flight_fsm_sim_start()` | Start scripted sim flight from PAD |
| `flight_fsm_sim_stop()` | Stop sim, return to PAD |
| `flight_fsm_sim_active()` | Is sim running? |
| `flight_fsm_sim_get_state(out)` | Get interpolated sim telemetry |

### 4.7  Sensor-Based Transitions (TODO)

Currently `flight_fsm_tick()` ignores the `state` parameter (sensor data). The
`fc_telem_state_t` input is reserved for real sensor-driven transitions:

```c
fsm_state_t flight_fsm_tick(const fc_telem_state_t *state)
{
    (void)state;  /* TODO: real sensor-based transitions */
    // ... sim-only logic for now
}
```

Future implementation will use EKF altitude/velocity + accelerometer data to detect
launch, burnout, apogee, and landing.

---

## 5  Command Router

### 5.1  Receive Path

```
USB CDC ISR
  → CDC_Receive_FS() writes to ring buffer (usbd_cdc_if.c)

Main loop (every iteration):
  → cmd_router_process()
    → cdc_ring_available() > 0?
      → read byte-by-byte
      → byte == 0x00 (COBS delimiter)?
        YES → cobs_decode(frame_buf) → dispatch_frame(decoded)
              → reset frame_pos
        NO  → accumulate in s_frame_buf[64]
              → overflow → reset (discard frame)
```

### 5.2  Frame Buffers

```c
#define FRAME_BUF_SIZE   64    /* max COBS-encoded frame */
#define DECODE_BUF_SIZE  64    /* max decoded payload */
```

### 5.3  Message Dispatch Table

`dispatch_frame()` switches on `decoded[0]` (msg_id):

| msg_id | Handler | Module |
|--------|---------|--------|
| 0x80 `CMD_ARM` | `cac_handle_arm()` | cac_handler.c |
| 0x81 `CMD_FIRE` | `cac_handle_fire()` | cac_handler.c |
| 0x82 `CMD_TESTMODE` | `cac_handle_testmode()` | cac_handler.c |
| 0x83 `CMD_POLL` | `cac_handle_config_poll()` | cac_handler.c |
| 0xF0 `CONFIRM` | `cac_handle_confirm()` | cac_handler.c |
| 0xF1 `ABORT` | `cac_handle_abort()` | cac_handler.c |
| 0xC0 `HANDSHAKE` | `cmd_handle_handshake()` | cmd_router.c (local) |
| 0xC1 `UPLOAD` | `cfg_handle_upload()` | cfg_manager.c |
| 0xC2 `DIAG` | `cmd_handle_diag()` | cmd_router.c → diag_runner |
| 0xC3 `READLOG` | `cfg_handle_readlog()` | cfg_manager.c |
| 0xC4 `ERASELOG` | `cfg_handle_eraselog()` | cfg_manager.c |
| 0xD0 `SIM_FLIGHT` | `cmd_handle_sim_flight()` | cmd_router.c → flight_fsm |
| other | silently discarded | — |

### 5.4  Handshake Response

`CMD_HANDSHAKE (0xC0)` is handled entirely in cmd_router.c. Response (13 bytes):

```
[0]=0xC0  [1]=proto_ver(5)  [2]=fw_major  [3]=fw_minor  [4]=fw_patch
[5-8]=config_hash_le32  [9-12]=CRC-32(0..8)
```

### 5.5  Sim Flight Toggle

`CMD_SIM_FLIGHT (0xD0)` toggles the simulated flight sequence:
- If sim is active → `flight_fsm_sim_stop()` (returns to PAD)
- If sim is inactive → `flight_fsm_sim_start()` (begins scripted profile)

No payload required. No CRC/magic validation (debug command).

---

## 6  CAC Protocol — FC Side

The Command-Acknowledge-Confirm protocol ensures safe pyro operations with
three-phase handshaking. See INTERFACE_SPEC.md for MC-side byte layouts.

### 6.1  CAC State Machine

```c
typedef enum { CAC_IDLE, CAC_AWAITING_CONFIRM } cac_phase_t;

static cac_phase_t  s_phase;
static uint16_t     s_pending_nonce;     /* nonce of pending cmd */
static pending_type_t s_pending_type;    /* PENDING_ARM or PENDING_FIRE */
static uint8_t      s_pending_channel;   /* 1-indexed channel */
static uint8_t      s_pending_action;    /* ARM: 0x01=arm, 0x00=disarm */
static uint16_t     s_pending_duration;  /* FIRE: duration in ms */
static uint32_t     s_confirm_deadline;  /* timeout timestamp */
```

### 6.2  Protocol Flow

```
MC                          FC
│                           │
├── CMD_ARM (0x80) ────────►│  Validate magic/CRC/complement
│                           │  Check preconditions
│                           │  Store pending, enter AWAITING_CONFIRM
│◄── ACK_ARM (0xA0) ───────┤  Send ACK with arm/cont bitmaps
│                           │
├── CONFIRM (0xF0) ────────►│  Validate magic/CRC/nonce
│                           │  Check not timed out
│                           │  Execute: pyro_mgr_set_arm()
│                           │  Emit FC_EVT_ARM event
│                           │  Return to IDLE
│                           │
│                           │  ── OR ──
│                           │
├── ABORT (0xF1) ──────────►│  Cancel pending, return to IDLE
│                           │
│                           │  ── OR ──
│                           │
│         (5s timeout) ─────│  Auto-cancel, return to IDLE
```

### 6.3  CMD_ARM Handler (0x80)

Validation chain:
1. `len >= SIZE_CMD_ARM (12)` — minimum packet size
2. `data[1]==0xCA, data[2]==0x5A` — magic bytes
3. `data[5] ^ data[7] == 0xFF` — channel complement check
4. `CRC-32(data[0..7]) == data[8..11]` — integrity check
5. **Nonce replay:** If same nonce and already AWAITING_CONFIRM for ARM → re-send ACK (idempotent)
6. `channel <= 3` — range check (0-indexed in wire format)
7. For ARM action: `pyro_mgr_has_continuity(ch+1)` — continuity required

On validation failure: `send_nack(nonce, error_code)`.

### 6.4  CMD_FIRE Handler (0x81)

Additional validations beyond ARM:
1. `data[6] ^ data[8] == 0xFF` — duration complement check
2. CRC-32 over bytes 0–8 (one more byte than ARM due to duration field)
3. **Test mode required:** `s_test_mode` must be true (NACK_ERR_NO_TESTMODE)
4. **Channel armed:** checked via arm bitmap (NACK_ERR_NOT_ARMED)
5. **Continuity:** must be present (NACK_ERR_NO_CONTINUITY)

Fire ACK flags byte: `bit0 = test_mode, bit1 = channel_armed`.

### 6.5  TESTMODE Handler (0x82)

Minimal packet — just the msg_id byte, no magic/CRC/nonce:

```c
void cac_handle_testmode(data, len)
  → FSM must be PAD (else NACK with ERR_BAD_STATE)
  → Toggle s_test_mode on/off
  → Propagate to pyro_mgr_set_test_mode()
  → Set 60s deadline (s_test_mode_deadline)
  → Emit FC_EVT_STATE event (so MC sees state change)
```

### 6.6  CONFIRM Handler (0xF0)

```
[0xF0][0xCA][0x5A][nonce:2][CRC-32:4] = 9 bytes
```

Validation:
1. `len >= 9`
2. Magic bytes
3. CRC-32 over bytes 0–4
4. `s_phase == CAC_AWAITING_CONFIRM`
5. Nonce matches `s_pending_nonce`
6. Not timed out (check `s_confirm_deadline`)

Execution on success:
- **PENDING_ARM:** `pyro_mgr_set_arm(channel, arm)` + emit `FC_EVT_ARM`
- **PENDING_FIRE:** `pyro_mgr_fire(channel, duration)` + emit `FC_EVT_PYRO`

### 6.7  ABORT Handler (0xF1)

Same 9-byte format as CONFIRM. If nonce matches pending, cancel and return to IDLE.
No event emitted, no NACK sent — silent cancellation.

### 6.8  CONFIG_POLL Handler (0x83)

Response (13 bytes):
```
[0]=0xA3  [1-2]=nonce  [3-6]=config_hash  [7]=proto_ver(5)  [8]=rsvd
[9-12]=CRC-32(0..8)
```

The config hash comes from `cfg_get_active_hash()`.

### 6.9  Timeout Management

`cac_tick()` runs every superloop iteration and handles two timeouts:

| Timeout | Duration | Constant | Action |
|---------|----------|----------|--------|
| Confirm timeout | 5 seconds | `CAC_CONFIRM_TIMEOUT_MS` | Cancel pending → IDLE |
| Test mode timeout | 60 seconds | `TEST_MODE_TIMEOUT_MS` | Disable test mode, disarm all, emit event |

### 6.10  Test Mode State (Dual Location)

Test mode state exists in **two** places, kept in sync:

| Location | Variable | Set By |
|----------|----------|--------|
| `cac_handler.c` | `static bool s_test_mode` | `cac_handle_testmode()`, `cac_tick()` timeout |
| `pyro_manager.c` | `static bool s_test_mode` | `pyro_mgr_set_test_mode()` (called by cac_handler) |

The CAC handler owns the toggle/timeout logic. The pyro manager only reads its
local copy for the fire precondition check and disarm-on-exit behavior.

---

## 7  NACK Error Codes

| Code | Define | Meaning |
|------|--------|---------|
| 0x01 | `NACK_ERR_CRC_FAIL` | CRC-32 validation failed |
| 0x02 | `NACK_ERR_BAD_STATE` | Wrong FSM state for this command |
| 0x03 | `NACK_ERR_NOT_ARMED` | Channel not armed (fire requires arm) |
| 0x04 | `NACK_ERR_NO_TESTMODE` | Test mode not active (fire requires it) |
| 0x05 | `NACK_ERR_NONCE_REUSE` | Nonce already used |
| 0x06 | `NACK_ERR_NO_CONTINUITY` | No continuity on requested channel |
| 0x07 | `NACK_ERR_LOW_BATTERY` | Battery too low for operation |
| 0x08 | `NACK_ERR_SELF_TEST` | Self-test failure |
| 0x09 | `NACK_ERR_CFG_TOO_LARGE` | Config upload too large |
| 0x0A | `NACK_ERR_FLASH_FAIL` | Flash operation failed |

NACK packet (10 bytes):
```
[0]=0xE0  [1-2]=nonce_le16  [3]=error_code  [4-5]=reserved(0)
[6-9]=CRC-32(0..5)
```

---

## 8  Event System

Events are queued via `tlm_queue_event(type, data)` and transmitted as FC_MSG_EVENT
packets (11 bytes). See INTERFACE_SPEC.md for packet format.

### 8.1  Event Types and Emission Points

| Type | Define | Data | Emitted By |
|------|--------|------|------------|
| 0x01 | `FC_EVT_STATE` | new FSM state | `transition_to()`, `cac_handle_testmode()` |
| 0x02 | `FC_EVT_PYRO` | `(channel << 8) \| 0x01` | `cac_handle_confirm()` on FIRE |
| 0x03 | `FC_EVT_APOGEE` | 0 | `transition_to(FSM_STATE_APOGEE)` |
| 0x04 | `FC_EVT_ERROR` | error code | (reserved for future use) |
| 0x05 | `FC_EVT_ORIGIN` | 0 | (reserved — GPS origin set) |
| 0x06 | `FC_EVT_BURNOUT` | 0 | (reserved — motor burnout detected) |
| 0x07 | `FC_EVT_STAGING` | 0 | (reserved — staging event) |
| 0x08 | `FC_EVT_ARM` | arm bitmap | `cac_handle_confirm()` on ARM |

### 8.2  FC_EVT_PYRO Data Encoding

The data field packs channel and action:
```
data = (channel << 8) | action
  channel: 1-indexed (1–4)
  action:  0x01 = fired
```

---

## 9  STATUS Bitmap

The 2-byte STATUS field is included in every FC_MSG_FAST packet (10 Hz).
Built by `status_pack_build()`.

### 9.1  Byte Layout

```
Byte 0: [ARM4][ARM3][ARM2][ARM1][CNT4][CNT3][CNT2][CNT1]
         b7    b6    b5    b4    b3    b2    b1    b0

Byte 1: [ST3][ST2][ST1][ST0][FIRED][ERROR][RSVD][RSVD]
         b7   b6   b5   b4    b3     b2    b1    b0
```

| Field | Bits | Description |
|-------|------|-------------|
| CNT1–CNT4 | byte0[0:3] | Continuity on channels 1–4 |
| ARM1–ARM4 | byte0[4:7] | Armed state on channels 1–4 |
| ST0–ST3 | byte1[4:7] | FSM state (4-bit, 0x0–0xB) |
| FIRED | byte1[3] | Any pyro channel fired this tick |
| ERROR | byte1[2] | System error flag |
| RSVD | byte1[0:1] | Reserved (always 0) |

### 9.2  Input: pyro_state_t

```c
typedef struct {
    bool armed[4];
    bool continuity[4];
    bool fired;               /* any channel actively firing */
    float cont_v[4];          /* continuity voltage (reserved for future) */
} pyro_state_t;
```

### 9.3  MC Decode Example

```
STATUS = [0x15, 0x68]

Byte 0 = 0x15 = 0b_0001_0101
  CNT: 0101 → CH1=yes, CH2=no, CH3=yes, CH4=no
  ARM: 0001 → CH1=armed, CH2-4=disarmed

Byte 1 = 0x68 = 0b_0110_1000
  FSM: 0110 → 0x6 = APOGEE
  FIRED: 1   → yes, a pyro is firing
  ERROR: 0   → no error
```

---

## 10  Safety Interlocks — Complete Chain

### 10.1  ARM Interlock

```
MC sends CMD_ARM (0x80)
  │
  ├─ Magic 0xCA 0x5A?              → silent discard if wrong
  ├─ Channel complement matches?    → silent discard if wrong
  ├─ CRC-32 valid?                  → silent discard if wrong
  ├─ Channel 0–3?                   → NACK_ERR_BAD_STATE
  ├─ Continuity present?            → NACK_ERR_NO_CONTINUITY
  │
  └─ OK → Store pending, send ACK_ARM
         → Wait for CONFIRM (5s timeout)
         → CONFIRM received?
           YES → pyro_mgr_set_arm(ch, arm)
                 emit FC_EVT_ARM
           NO  → auto-cancel (timeout or ABORT)
```

### 10.2  FIRE Interlock

```
MC sends CMD_FIRE (0x81)
  │
  ├─ Magic 0xCA 0x5A?              → silent discard
  ├─ Channel complement?            → silent discard
  ├─ Duration complement?           → silent discard
  ├─ CRC-32 valid?                  → silent discard
  ├─ Channel 0–3?                   → NACK_ERR_BAD_STATE
  ├─ Test mode active?              → NACK_ERR_NO_TESTMODE
  ├─ Channel armed?                 → NACK_ERR_NOT_ARMED
  ├─ Continuity present?            → NACK_ERR_NO_CONTINUITY
  │
  └─ OK → Store pending, send ACK_FIRE
         → Wait for CONFIRM (5s timeout)
         → CONFIRM received?
           YES → pyro_mgr_fire(ch, dur)
                 ├─ Recheck: armed?        → return -1
                 ├─ Recheck: continuity?   → return -1
                 ├─ Recheck: test/FSM?     → return -1
                 ├─ Cap duration (2000ms max, 50ms in test)
                 └─ casper_pyro_fire(ch, dur)
                    emit FC_EVT_PYRO
           NO  → auto-cancel
```

Note the **double-check**: preconditions are verified both at CMD_FIRE receipt
(in cac_handler) and again at CONFIRM execution (in pyro_manager). This guards
against state changes between CMD and CONFIRM (e.g., e-match disconnected).

### 10.3  Test Mode Interlock

```
MC sends CMD_TESTMODE (0x82)
  │
  ├─ FSM state == PAD?              → NACK_ERR_BAD_STATE if not
  │
  └─ OK → Toggle test mode on/off
         → Propagate to pyro_manager
         → If toggling OFF → disarm all channels
         → Set/clear 60s deadline
         → Emit FC_EVT_STATE
```

Test mode cannot be entered outside PAD state. The 60s auto-timeout in `cac_tick()`
ensures test mode doesn't persist indefinitely.

---

## 11  Superloop Integration

### 11.1  Initialization Order (main.c)

```c
casper_pyro_init(&pyro, &hadc1, &hadc2, &hadc3);   /* HW: force pins LOW, cal ADC */
pyro_mgr_init();                                     /* Safety: reset arm/test state */
flight_fsm_init();                                   /* FSM: start in PAD */
cac_init();                                          /* CAC: idle, no pending */
cmd_router_init();                                   /* Router: reset frame buffer */
tlm_init();                                          /* Telemetry: start periodic */
```

### 11.2  Superloop Tick Order

```c
while (1) {
    cmd_router_process();        /* 1. Process incoming USB commands */
    cac_tick();                  /* 2. Check confirm/test mode timeouts */
    flight_fsm_tick(&tstate);    /* 3. Advance FSM (sim or sensor) */
    pyro_mgr_tick();             /* 4. ADC reads, LED updates, auto-stop */
    tlm_tick();                  /* 5. Send periodic telemetry */
    // ... sensor reads, EKF, etc.
}
```

### 11.3  Module Ownership Rule

Per CLAUDE.md Rule 3: the superloop must **never** directly write to pyro pins.
All pyro GPIO control goes through `casper_pyro_fire()` / `casper_pyro_stop()`.
Direct `HAL_GPIO_WritePin()` on PY1–PY4 in the main loop would override
`casper_pyro_stop()` and create a safety hazard.

---

## 12  Constants Reference

### 12.1  Timing Constants

| Constant | Value | Source |
|----------|-------|--------|
| `CAC_CONFIRM_TIMEOUT_MS` | 5000 ms | tlm_types.h |
| `TEST_MODE_TIMEOUT_MS` | 60000 ms | tlm_types.h |
| `TLM_FAST_PERIOD_MS` | 100 ms | tlm_types.h (10 Hz) |
| `PYRO_MAX_FIRE_MS` | 2000 ms | tlm_types.h |
| `PYRO_DEFAULT_FIRE_MS` | 1000 ms | casper_pyro.h |
| Test mode fire cap | 50 ms | pyro_manager.c (hardcoded) |

### 12.2  Pyro Constants

| Constant | Value | Source |
|----------|-------|--------|
| `PYRO_NUM_CHANNELS` | 4 | casper_pyro.h |
| `PYRO_MGR_NUM_CHANNELS` | 4 | tlm_types.h |
| `PYRO_CONTINUITY_THRESHOLD` | 8000 | casper_pyro.h (16-bit ADC) |

### 12.3  Protocol Constants

| Constant | Value | Source |
|----------|-------|--------|
| `CAC_MAGIC_1` | 0xCA | tlm_types.h |
| `CAC_MAGIC_2` | 0x5A | tlm_types.h |
| `CAC_ACTION_ARM` | 0x01 | tlm_types.h |
| `CAC_ACTION_DISARM` | 0x00 | tlm_types.h |
| `PROTOCOL_VERSION` | 5 | tlm_types.h |

### 12.4  Packet Sizes

| Constant | Bytes | Byte Layout |
|----------|-------|-------------|
| `SIZE_CMD_ARM` | 12 | `[ID:1][MAG:2][NONCE:2][CH:1][ACT:1][~CH:1][CRC:4]` |
| `SIZE_CMD_FIRE` | 13 | `[ID:1][MAG:2][NONCE:2][CH:1][DUR:1][~CH:1][~DUR:1][CRC:4]` |
| `SIZE_CONFIRM` | 9 | `[ID:1][MAG:2][NONCE:2][CRC:4]` |
| `SIZE_NACK` | 10 | `[ID:1][NONCE:2][ERR:1][RSVD:2][CRC:4]` |
| `SIZE_ACK_ARM` | 12 | `[ID:1][NONCE:2][CH:1][ACT:1][ARM:1][CONT:1][RSVD:1][CRC:4]` |
| `SIZE_ACK_FIRE` | 13 | `[ID:1][NONCE:2][CH:1][DUR:1][FLAGS:1][CONT:1][RSVD:2][CRC:4]` |
| `SIZE_ACK_CFG` | 13 | `[ID:1][NONCE:2][HASH:4][VER:1][RSVD:1][CRC:4]` |
| `SIZE_HANDSHAKE_RESP` | 13 | `[ID:1][VER:1][FW:3][HASH:4][CRC:4]` |
| `SIZE_FC_MSG_EVENT` | 11 | `[ID:1][TYPE:1][DATA:2][TIME:2][RSVD:1][CRC:4]` |

---

## 13  Channel Indexing Convention

Two indexing conventions coexist:

| Context | Indexing | Example |
|---------|----------|---------|
| Wire protocol (packets) | 0-indexed | channel=0 means CH 1 |
| pyro_manager API | 1-indexed | channel=1 means CH 1 |
| casper_pyro API | 0-indexed | ch=0 means CH 1 |
| STATUS bitmap | bit position | bit0 = CH 1 |

The CAC handler converts: `uint8_t ch1 = channel + 1;` before calling pyro_manager.
