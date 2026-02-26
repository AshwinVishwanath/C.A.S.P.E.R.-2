# FSM_TRANSITION_SPEC.md — Sensor-Driven Flight State Machine

> Defines the transition conditions for C.A.S.P.E.R.-2's real-time flight state machine.
> Replaces the TODO in `flight_fsm_tick()` (PYRO_SPEC.md §4.7).
>
> **Companion docs:** PYRO_SPEC.md (FSM internals, pyro system, sim profile),
> EKF_SPEC.md (navigation stack), SENSOR_SPEC.md (sensor drivers),
> INTERFACE_SPEC.md (telemetry protocol, MC-side config).

---

## 1  Overview

The FSM consumes a `fsm_input_t` struct populated by the flight loop from EKF
outputs, attitude estimator state, and raw sensor data. All transitions are
evaluated every superloop iteration (~833 Hz when IMU-driven). Sustained-condition
checks use dwell timers against `HAL_GetTick()`.

### 1.1  Design Principles

1. **No single-sample transitions.** Every condition requires a sustained dwell
   time to reject sensor noise, buffeting, and transient spikes.
2. **Monotonic forward progression.** The FSM never moves backward except for
   the COAST → BOOST (sustain re-light) path. All other transitions are strictly
   forward in the state graph.
3. **Pyro decisions are FSM outputs.** The FSM sets fire request flags; the pyro
   manager executes them subject to its own safety gates (arm status, continuity).
4. **Configurable thresholds.** Deployment altitudes and drogue-fail parameters
   are user-configurable via MC upload. Detection thresholds are compile-time
   constants with sensible defaults.

---

## 2  FSM Input Struct

The existing `fc_telem_state_t` is insufficient for FSM decisions. A new
`fsm_input_t` struct is introduced, populated by the flight loop before each
`flight_fsm_tick()` call.

```c
typedef struct {
    /* From EKF */
    float alt_m;            /* Altitude AGL (m), from ekf.x[0]                */
    float vel_mps;          /* Vertical velocity (m/s), from ekf.x[1]         */
                            /*   positive = ascending, negative = descending   */

    /* From attitude estimator / IMU */
    float vert_accel_g;     /* Vertical acceleration (g), body-Z projected     */
                            /*   onto NED-down via attitude quaternion.         */
                            /*   Positive = upward accel (thrust), ~0 = free   */
                            /*   fall, negative = deceleration                 */
    bool  antenna_up;       /* True if rocket is upright (antenna pointing up) */
                            /*   Determined by Madgwick filter gravity vector: */
                            /*   body Z-axis within ±30° of NED up direction   */

    /* Timing */
    float flight_time_s;    /* Seconds since mission start (0 if not launched) */

    /* Configuration (from MC upload) */
    float main_deploy_alt_m;     /* Main chute deployment altitude AGL (m)     */
    float drogue_fail_vel_mps;   /* Drogue failure descent rate threshold (m/s)*/
                                 /*   Stored positive. Compared against EKF    */
                                 /*   vertical velocity (negative = descending)*/
                                 /*   Triggers if vel_mps < -this_value        */
    float drogue_fail_time_s;    /* Sustain time for drogue failure detect (s) */
    uint8_t apogee_pyro_ch;      /* Pyro channel for apogee/drogue (0-indexed) */
    uint8_t main_pyro_ch;        /* Pyro channel for main chute (0-indexed)    */
    uint16_t apogee_fire_dur_ms; /* Apogee pyro fire duration (ms)             */
    uint16_t main_fire_dur_ms;   /* Main pyro fire duration (ms)               */
} fsm_input_t;
```

### 2.1  Populating `fsm_input_t`

The flight loop builds this struct each iteration:

```
flight_loop_tick():
  ...
  fsm_in.alt_m          = ekf.x[0];
  fsm_in.vel_mps        = ekf.x[1];
  fsm_in.vert_accel_g   = compute_vert_accel(att.q, accel_ms2);
  fsm_in.antenna_up     = check_antenna_up(att.q);
  fsm_in.flight_time_s  = flight_fsm_get_time_s();
  fsm_in.main_deploy_alt_m    = cfg.main_deploy_alt;   /* from MC config */
  fsm_in.drogue_fail_vel_mps  = cfg.drogue_fail_vel;
  fsm_in.drogue_fail_time_s   = cfg.drogue_fail_time;
  fsm_in.apogee_pyro_ch       = cfg.apogee_pyro_ch;
  fsm_in.main_pyro_ch         = cfg.main_pyro_ch;
  fsm_in.apogee_fire_dur_ms   = cfg.apogee_fire_dur;
  fsm_in.main_fire_dur_ms     = cfg.main_fire_dur;

  flight_fsm_tick(&fsm_in);
```

### 2.2  `compute_vert_accel()`

Projects the body-frame accelerometer reading onto the NED vertical axis using
the current attitude quaternion:

```c
float compute_vert_accel(const float q[4], const float accel_body_ms2[3])
{
    /* Rotate body accel to NED frame */
    float a_ned[3];
    quat_rotate_vec(q, accel_body_ms2, a_ned);

    /* NED convention: Z-down is positive. Negate so upward accel = positive g. */
    /* Subtract gravity so stationary = 0g, not 1g.                            */
    return (-a_ned[2] - 9.80665f) / 9.80665f;
}
```

In this convention: sitting on pad ≈ 0g, boost thrust ≈ +Ng, free fall ≈ −1g,
deceleration during coast ≈ negative values.

> **Note:** This uses the LSM6DSO32 (±32g range) for normal flight. The
> ADXL372 (±200g) should be used if LSM6DSO32 saturates during boost.
> Sensor selection logic is outside the scope of this spec.

### 2.3  `check_antenna_up()`

Computes the angle between the rocket's body Z-axis and NED "up" direction:

```c
bool check_antenna_up(const float q[4])
{
    /* Body Z-axis [0,0,1] rotated to NED */
    float bz_ned[3];
    float body_z[3] = {0.0f, 0.0f, 1.0f};
    quat_rotate_vec(q, body_z, bz_ned);

    /* NED up = [0, 0, -1]. Dot product with body Z in NED: */
    float dot = -bz_ned[2];  /* cos(angle between body-Z and NED-up) */

    return dot > 0.866f;     /* < 30 degrees from vertical */
}
```

---

## 3  Dwell Timer Mechanism

All sustained-condition checks use a common pattern:

```c
typedef struct {
    uint32_t start_ms;      /* HAL_GetTick() when condition first met */
    bool     active;        /* condition currently sustained           */
} dwell_timer_t;

static inline bool dwell_check(dwell_timer_t *t, bool condition, uint32_t required_ms)
{
    if (!condition) {
        t->active = false;
        return false;
    }
    if (!t->active) {
        t->start_ms = HAL_GetTick();
        t->active = true;
    }
    return (HAL_GetTick() - t->start_ms) >= required_ms;
}
```

If the condition drops out at any point, the timer resets. The condition must be
continuously true for the full dwell period.

### 3.1  HIL Testing Override

For hardware-in-the-loop testing at faster-than-real-time rates, `HAL_GetTick()`
can be replaced with a virtual clock:

```c
#ifdef HIL_MODE
static uint32_t s_virtual_tick_ms = 0;
uint32_t fsm_get_tick(void) { return s_virtual_tick_ms; }
void     fsm_set_tick(uint32_t ms) { s_virtual_tick_ms = ms; }
#else
#define  fsm_get_tick() HAL_GetTick()
#endif
```

All dwell timers and mission elapsed time within the FSM use `fsm_get_tick()`
instead of `HAL_GetTick()` directly.

---

## 4  State Transition Definitions

### 4.0  Threshold Constants

```c
/* --- Launch detection (PAD → BOOST) --- */
#define FSM_LAUNCH_ACCEL_G          2.0f        /* Vertical accel threshold (g)   */
#define FSM_LAUNCH_ACCEL_DWELL_MS   100         /* Accel sustain time (ms)        */
#define FSM_LAUNCH_VEL_MPS          15.0f       /* EKF velocity threshold (m/s)   */

/* --- Burnout detection (BOOST → COAST) --- */
#define FSM_BURNOUT_ACCEL_G         0.0f        /* Accel below this = burnout     */
#define FSM_BURNOUT_DWELL_MS        100         /* Sustain time (ms)              */

/* --- Sustain re-light (COAST → BOOST) --- */
#define FSM_SUSTAIN_ACCEL_G         3.0f        /* Re-light accel threshold (g)   */
#define FSM_SUSTAIN_DWELL_MS        100         /* Sustain time (ms)              */

/* --- Apogee detection (COAST → APOGEE) --- */
#define FSM_APOGEE_VEL_MPS          0.0f        /* Velocity <= this = apogee      */
#define FSM_APOGEE_VEL_DWELL_MS     25          /* Sustain time (ms)              */
#define FSM_APOGEE_MIN_FLIGHT_S     5.0f        /* Minimum flight time gate (s)   */

/* --- Landing detection (MAIN → LANDED) --- */
#define FSM_LANDED_VEL_MPS          1.0f        /* |velocity| below this (m/s)    */
#define FSM_LANDED_ALT_DELTA_M      2.0f        /* Altitude change < this (m)     */
#define FSM_LANDED_DWELL_MS         3000        /* Sustain time (ms)              */

/* --- Low-power auto-timer (LANDED → LOW_POWER) --- */
#define FSM_LANDED_TO_LOWPOWER_S    300         /* 5 minutes after landing        */
```

### 4.1  PAD → BOOST

**Purpose:** Detect launch ignition.

**Conditions (all three must be true simultaneously):**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | `antenna_up == true` | Madgwick filter + gravity vector | Rocket is upright on pad. Prevents transition if knocked over. |
| 2 | `vert_accel_g > 2.0` sustained for 100 ms | LSM6DSO32 / ADXL372 via `compute_vert_accel()` | Motor ignition produces sustained upward acceleration. Dwell rejects shocks, pad handling, and wind gusts. |
| 3 | `vel_mps > 15.0` | EKF velocity estimate | Rocket has actually started moving. Velocity gate prevents accel-only false triggers from vibration or e-match ignition transients. |

**On transition:**
- Set `s_mission_start_ms = fsm_get_tick()`
- Set `s_mission_started = true`
- Emit `FC_EVT_STATE(BOOST)`
- Switch baro logging to burst mode
- Switch telemetry to high-rate TX
- Disable command reception (safety: no commands during flight)
- Switch ADXL372 to high-power mode
- Begin high-rate data logging (200–250 Hz)

**Internal state:**

```c
static dwell_timer_t s_launch_accel_dwell;
```

**Pseudocode:**

```c
case FSM_STATE_PAD:
    bool accel_sustained = dwell_check(&s_launch_accel_dwell,
                                        in->vert_accel_g > FSM_LAUNCH_ACCEL_G,
                                        FSM_LAUNCH_ACCEL_DWELL_MS);
    if (in->antenna_up && accel_sustained && in->vel_mps > FSM_LAUNCH_VEL_MPS) {
        transition_to(FSM_STATE_BOOST);
    }
    break;
```

### 4.2  BOOST → COAST

**Purpose:** Detect motor burnout.

**Condition:**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | `vert_accel_g < 0.0` sustained for 100 ms | Projected accel | Once thrust ceases, the rocket is in free fall (≈ −1g) plus aerodynamic drag. Accel drops below 0g. The 100 ms dwell rejects brief thrust oscillations near burnout. |

**On transition:**
- Emit `FC_EVT_STATE(COAST)`
- Emit `FC_EVT_BURNOUT` with peak acceleration recorded during BOOST
- Same operational settings as BOOST (high rate logging, TX, etc.)

**Internal state:**

```c
static dwell_timer_t s_burnout_dwell;
static float         s_peak_accel_g;    /* track peak during BOOST */
```

**Pseudocode:**

```c
case FSM_STATE_BOOST:
    /* Track peak acceleration for burnout event */
    if (in->vert_accel_g > s_peak_accel_g)
        s_peak_accel_g = in->vert_accel_g;

    if (dwell_check(&s_burnout_dwell,
                     in->vert_accel_g < FSM_BURNOUT_ACCEL_G,
                     FSM_BURNOUT_DWELL_MS)) {
        tlm_queue_event(FC_EVT_BURNOUT,
                        (uint16_t)(s_peak_accel_g * 1000.0f)); /* mg */
        transition_to(FSM_STATE_COAST);
    }
    break;
```

### 4.3  COAST → APOGEE

**Purpose:** Detect peak altitude (velocity zero-crossing).

**Conditions (all must be true):**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | `vel_mps <= 0.0` sustained for 25 ms | EKF velocity | Velocity crossing zero indicates apogee. Short dwell because apogee is a brief event and late detection costs altitude for drogue deployment. |
| 2 | `flight_time_s > 5.0` | Mission timer | Safety gate against false apogee detection from pad vibration, EKF transients during early boost, or brief negative velocity spikes. |

**On transition:**
- Emit `FC_EVT_STATE(APOGEE)`
- Emit `FC_EVT_APOGEE` with peak altitude
- **Fire apogee pyro channel:** Set fire request for `apogee_pyro_ch` with
  duration `apogee_fire_dur_ms` (via `pyro_mgr_auto_fire()`)
- Switch telemetry to high-range TX mode
- Begin flight software preparation for recovery mode

**Internal state:**

```c
static dwell_timer_t s_apogee_vel_dwell;
static float         s_peak_alt_m;      /* track peak during COAST */
```

**Pseudocode:**

```c
case FSM_STATE_COAST:
    /* Track peak altitude */
    if (in->alt_m > s_peak_alt_m)
        s_peak_alt_m = in->alt_m;

    /* Check for sustain re-light → back to BOOST */
    if (dwell_check(&s_sustain_relight_dwell,
                     in->vert_accel_g > FSM_SUSTAIN_ACCEL_G,
                     FSM_SUSTAIN_DWELL_MS)) {
        s_peak_accel_g = 0.0f;  /* reset peak tracking for new burn */
        transition_to(FSM_STATE_BOOST);
        break;
    }

    /* Check for apogee */
    if (in->flight_time_s > FSM_APOGEE_MIN_FLIGHT_S &&
        dwell_check(&s_apogee_vel_dwell,
                     in->vel_mps <= FSM_APOGEE_VEL_MPS,
                     FSM_APOGEE_VEL_DWELL_MS)) {
        tlm_queue_event(FC_EVT_APOGEE,
                        (uint16_t)(s_peak_alt_m / 10.0f)); /* decametres */
        pyro_mgr_auto_fire(in->apogee_pyro_ch, in->apogee_fire_dur_ms);
        transition_to(FSM_STATE_APOGEE);
    }
    break;
```

### 4.4  COAST → BOOST (Sustain Re-Light)

**Purpose:** Detect second-stage motor ignition during coast phase.

**Condition:**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | `vert_accel_g > 3.0` sustained for 100 ms | Projected accel | Sustained high-g indicates motor re-light, not aerodynamic transient. 3g threshold is well above drag deceleration. |

**On transition:**
- Emit `FC_EVT_STATE(BOOST)`
- Emit `FC_EVT_STAGING` with stage number
- Reset `s_peak_accel_g` for new burn phase tracking
- All operational settings remain as BOOST

**Note:** This transition is evaluated *before* the apogee check in the COAST
state handler (see §4.3 pseudocode). If sustain re-lights, apogee detection
is deferred until the subsequent coast phase.

**Internal state:**

```c
static dwell_timer_t s_sustain_relight_dwell;
static uint8_t       s_stage_count;     /* increments on each re-light */
```

### 4.5  APOGEE → MAIN (Normal Path)

**Purpose:** Deploy main chute at configured altitude.

**Condition:**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | `alt_m <= main_deploy_alt_m` | EKF altitude | User-configured deployment altitude. Typical: 150–300 m AGL. |

**On transition:**
- Emit `FC_EVT_STATE(MAIN)`
- **Fire main pyro channel:** Set fire request for `main_pyro_ch` with
  duration `main_fire_dur_ms` (via `pyro_mgr_auto_fire()`)
- Switch telemetry to high-range TX mode
- Switch to recovery mode settings
- Reduce data saving rate
- Enter lower power state

### 4.6  APOGEE → MAIN (Drogue Failure Backup)

**Purpose:** If drogue failed to deploy or open, fire main early to prevent
ballistic impact or main chute shredding at high speed.

**Condition:**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | `vel_mps < -drogue_fail_vel_mps` sustained for `drogue_fail_time_s` | EKF vertical velocity, MC config | If vertical descent rate exceeds the user-configured threshold for the configured duration, drogue is assumed to have failed. Uses EKF vertical velocity specifically (not total speed magnitude) because off-axis spin could corrupt a magnitude-based measurement — a spinning rocket under a functioning drogue could false-trigger on total speed while descending at a safe vertical rate. Firing main early risks higher opening shock but is preferable to a ballistic descent or deploying main at the normal altitude at excessive speed. |

Both parameters (`drogue_fail_vel_mps` and `drogue_fail_time_s`) are configured
from Mission Control during pre-flight setup. Typical values: 40–60 m/s threshold,
2–5 s sustain time. `drogue_fail_vel_mps` is stored as a positive value;
the comparison is against the negative (descending) EKF velocity.

**On transition:**
- Emit `FC_EVT_STATE(MAIN)`
- Emit `FC_EVT_ERROR` with error code `ERR_DROGUE_FAIL`
- **Fire main pyro channel** (same as normal path)
- Switch to ultra-high-power TX mode (maximize recovery beacon range)

**Internal state:**

```c
static dwell_timer_t s_drogue_fail_dwell;
```

**Pseudocode (APOGEE state handler):**

```c
case FSM_STATE_APOGEE:
    /* Normal path: altitude-based main deploy */
    if (in->alt_m <= in->main_deploy_alt_m) {
        pyro_mgr_auto_fire(in->main_pyro_ch, in->main_fire_dur_ms);
        transition_to(FSM_STATE_MAIN);
        break;
    }

    /* Backup path: drogue failure detection                              */
    /* vel_mps is negative when descending. drogue_fail_vel_mps is       */
    /* stored positive. Check: descending faster than threshold.          */
    float drogue_fail_dwell_ms = in->drogue_fail_time_s * 1000.0f;
    if (dwell_check(&s_drogue_fail_dwell,
                     in->vel_mps < -(in->drogue_fail_vel_mps),
                     (uint32_t)drogue_fail_dwell_ms)) {
        tlm_queue_event(FC_EVT_ERROR, ERR_DROGUE_FAIL);
        pyro_mgr_auto_fire(in->main_pyro_ch, in->main_fire_dur_ms);
        transition_to(FSM_STATE_MAIN);
    }
    break;
```

### 4.7  MAIN → LANDED

**Purpose:** Detect touchdown.

**Conditions (both must be true simultaneously, sustained):**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | `|vel_mps| < 1.0` sustained for 3 s | EKF vertical velocity (ZUPT will pull to 0) | Rocket has stopped moving vertically. ZUPT updates in the EKF will drive velocity to near-zero on landing, making this a reliable indicator. 3 s dwell rejects pendulum swings under chute. Uses absolute value because post-landing the velocity should oscillate around zero, not be consistently signed. |
| 2 | `|alt_m - alt_sample| < 2.0` sustained for 3 s | EKF altitude, sampled at dwell start | Altitude has stabilized within ±2 m. The altitude reference is captured when the dwell timer starts, so the check is against drift from that point, not absolute zero. |

**On transition:**
- Emit `FC_EVT_STATE(LANDED)`
- Switch telemetry to high-range TX mode (recovery beacons)
- Switch to recovery mode
- Stop data logging
- Disable EKF updates (save power)
- Start low-power auto-timer (`FSM_LANDED_TO_LOWPOWER_S`)
- Begin periodic recovery beacons at configurable interval

**Internal state:**

```c
static dwell_timer_t s_landed_dwell;
static float         s_landed_alt_ref;  /* altitude when dwell started */
```

**Pseudocode:**

```c
case FSM_STATE_MAIN:
{
    bool vel_ok = fabsf(in->vel_mps) < FSM_LANDED_VEL_MPS;

    /* Capture altitude reference when dwell begins */
    if (vel_ok && !s_landed_dwell.active) {
        s_landed_alt_ref = in->alt_m;
    }

    bool alt_ok = fabsf(in->alt_m - s_landed_alt_ref) < FSM_LANDED_ALT_DELTA_M;

    if (dwell_check(&s_landed_dwell,
                     vel_ok && alt_ok,
                     FSM_LANDED_DWELL_MS)) {
        transition_to(FSM_STATE_LANDED);
    }
    break;
}
```

### 4.8  LANDED → RECOVERY (Low-Power Mode)

**Purpose:** Conserve battery for extended recovery periods.

**Condition:**

| # | Condition | Source | Rationale |
|---|-----------|--------|-----------|
| 1 | Time in LANDED state > `FSM_LANDED_TO_LOWPOWER_S` | Auto-timer | After 5 minutes on the ground, transition to low-power recovery mode to extend battery life for recovery beacons. |

**On transition:**
- Emit `FC_EVT_STATE(RECOVERY)`
- Shut down IMU, barometer, magnetometer
- Disable EKF
- Reduce MCU clock (VOS3 / 200 MHz if voltage scaling implemented)
- GPS into PMREQ sleep (periodic wake for beacon position)
- Reduce LoRa TX to periodic beacons only (e.g., every 30 s)
- Battery LED remains active for visual location

> **Encoding:** Uses `FSM_STATE_RECOVERY (0x9)` from the existing 4-bit FSM enum.
> This state was previously reserved for single-deploy recovery; it is repurposed
> here as the post-landing low-power beacon mode.

---

## 5  State Transition Graph

```
                    ┌──────────────────────┐
                    │                      │
                    │  ┌───── BOOST ◄──────┘  (sustain re-light)
                    │  │   [auto-arm channels]
                    │  │        │
                    │  │        │ vert_accel < 0g, 100ms
                    │  │        ▼
  antenna_up AND    │  │      COAST
  accel > 2g 100ms  │  │        │
  AND vel > 15 m/s  │  │        ├──── vert_accel > 3g, 100ms ───┘
                    │  │        │
        PAD ────────┘  │        │ vel <= 0, 25ms AND flight_time > 5s
   [pyro lockout]      │        ▼
                       │      APOGEE ─── [fire apogee pyro]
                       │        │
                       │        ├──── alt <= main_deploy_alt (normal)
                       │        │
                       │        ├──── vert_vel < -drogue_fail_vel (backup)
                       │        │       for drogue_fail_time
                       │        ▼
                       │       MAIN ─── [fire main pyro]
                       │        │
                       │        │ |vel| < 1 m/s AND |Δalt| < 2m, 3s
                       │        ▼
                       │      LANDED
                       │        │
                       │        │ 5 min auto-timer
                       │        ▼
                       │    RECOVERY (0x9, low-power beacon mode)
```

---

## 6  Pyro Manager Integration

### 6.1  Pyro Safety Invariant

**PAD state is an absolute pyro lockout.** No channel may fire while the FSM is
in PAD state. This is the hardest safety invariant in the system. The only
exception is test mode, which has its own safety gates (PAD-only entry, 60 s
auto-timeout, 50 ms fire cap — see PYRO_SPEC.md §10.3).

This invariant is enforced at two levels:

1. **FSM level:** The FSM never calls `pyro_mgr_auto_fire()` from PAD state.
2. **pyro_manager level:** `pyro_mgr_auto_fire()` checks FSM state and refuses
   to fire if in PAD. Defense in depth — even a bug in the FSM cannot fire
   a channel on the pad.

```c
/* In pyro_manager.c */
#define PYRO_PAD_LOCKOUT() \
    do { if (flight_fsm_get_state() == FSM_STATE_PAD) return -1; } while(0)
```

### 6.2  Channel Exclusion

Channels can be hard-coded as excluded from auto-arm and auto-fire at compile
time. Excluded channels will never be armed or fired by the FSM, regardless of
continuity status.

```c
/* Compile-time channel exclusion mask.                           */
/* Set bit N to exclude channel N from auto-arm and auto-fire.    */
/* Example: 0x0C = exclude channels 2 and 3 (0-indexed).         */
/* TODO: Move to MC flight config upload (out of current scope).  */
#define PYRO_EXCLUDE_MASK   0x00    /* default: no channels excluded */

static inline bool pyro_ch_excluded(uint8_t ch)
{
    return (PYRO_EXCLUDE_MASK >> ch) & 1;
}
```

### 6.3  Auto-Arm on BOOST Entry

When the FSM transitions to BOOST (or re-enters BOOST via sustain re-light),
all channels that have continuity and are not excluded are automatically armed.
This prevents a missed pre-flight arming step from causing mission failure.

Channels already armed by the operator via CAC are unaffected (idempotent).

```c
void pyro_mgr_auto_arm_flight(void)
{
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (pyro_ch_excluded(ch)) continue;
        if (!pyro.continuity[ch]) continue;
        if (s_armed[ch]) continue;              /* already armed */

        s_armed[ch] = true;
        tlm_queue_event(FC_EVT_ARM,
                        ((uint16_t)ch << 8) | 0x01);   /* auto-arm event */
    }
}
```

This is called from `transition_to()` when entering BOOST:

```c
static void transition_to(fsm_state_t new_state)
{
    if (new_state == s_state) return;
    s_state = new_state;
    s_state_entry_ms = fsm_get_tick();
    tlm_queue_event(FC_EVT_STATE, (uint16_t)new_state);

    if (new_state == FSM_STATE_BOOST) {
        pyro_mgr_auto_arm_flight();
    }
    if (new_state == FSM_STATE_APOGEE) {
        tlm_queue_event(FC_EVT_APOGEE, 0);
    }
}
```

### 6.4  `pyro_mgr_auto_fire()`

FSM-triggered pyro fires. Unlike CAC-initiated fires which require the full
CMD → ACK → CONFIRM handshake, auto-fires are triggered by flight events and
bypass the CAC protocol. However, they **require** the channel to be armed.
ARM is the single source of truth for "is this channel allowed to fire."

```c
int pyro_mgr_auto_fire(uint8_t ch, uint16_t duration_ms)
{
    /* Hard safety: never fire on pad */
    PYRO_PAD_LOCKOUT();

    /* Preconditions */
    if (ch >= 4) return -1;
    if (pyro_ch_excluded(ch)) return -1;        /* hard-coded exclusion */
    if (!s_armed[ch]) return -1;                /* must be armed        */
    if (!pyro.continuity[ch]) return -1;        /* no e-match           */
    if (pyro.firing[ch]) return -1;             /* already firing       */

    /* Cap duration */
    if (duration_ms > PYRO_MAX_FIRE_MS) duration_ms = PYRO_MAX_FIRE_MS;

    casper_pyro_fire(&pyro, ch, duration_ms);
    tlm_queue_event(FC_EVT_PYRO,
                    ((uint16_t)ch << 8) | (duration_ms & 0xFF));
    return 0;
}
```

**Why ARM is required:** The ARM flag serves as the unified exclusion gate.
If a channel is hard-coded excluded, it is never auto-armed, so ARM remains
false, and auto-fire refuses. If an operator deliberately disarms a channel
pre-flight, auto-arm on BOOST re-arms it (unless excluded). The ARM flag is
the single place to check "should this channel fire."

### 6.5  Pre-Flight Arming

The normal pre-launch workflow is:

1. Operator ARMs apogee + main channels via CAC on the pad
2. MC displays armed + continuity status for verification
3. On launch detect, FSM auto-arms any channels with continuity that were missed

Auto-arm is a safety net, not the primary path. The pre-flight checklist should
always include explicit arming of all flight-critical channels.

---

## 7  `fc_telem_state_t` Struct Update

The existing `fc_telem_state_t` (used for sim telemetry and FSM) should be
updated to include the new fields, or the FSM should exclusively use `fsm_input_t`.

**Recommended approach:** Keep `fc_telem_state_t` for telemetry packing and
introduce `fsm_input_t` as the FSM-specific input. The flight loop populates
both from the same sensor data. This avoids coupling telemetry formatting to
FSM decision logic.

---

## 8  Internal State Summary

```c
/* Dwell timers */
static dwell_timer_t s_launch_accel_dwell;
static dwell_timer_t s_burnout_dwell;
static dwell_timer_t s_sustain_relight_dwell;
static dwell_timer_t s_apogee_vel_dwell;
static dwell_timer_t s_drogue_fail_dwell;
static dwell_timer_t s_landed_dwell;

/* Peak tracking */
static float   s_peak_accel_g;         /* reset on each BOOST entry */
static float   s_peak_alt_m;           /* reset on each COAST entry */

/* Landing detection */
static float   s_landed_alt_ref;       /* altitude when landing dwell started */

/* Multi-stage */
static uint8_t s_stage_count;          /* 0 = single stage, increments on re-light */

/* All cleared by flight_fsm_init() */
```

---

## 9  Event Emissions Summary

| Transition | Events Emitted |
|------------|---------------|
| PAD → BOOST | `FC_EVT_STATE(BOOST)` |
| BOOST → COAST | `FC_EVT_STATE(COAST)`, `FC_EVT_BURNOUT(peak_mg)` |
| COAST → BOOST (sustain) | `FC_EVT_STATE(BOOST)`, `FC_EVT_STAGING(stage_num)` |
| COAST → APOGEE | `FC_EVT_STATE(APOGEE)`, `FC_EVT_APOGEE(peak_dam)`, `FC_EVT_PYRO(apogee_ch)` |
| APOGEE → MAIN (normal) | `FC_EVT_STATE(MAIN)`, `FC_EVT_PYRO(main_ch)` |
| APOGEE → MAIN (drogue fail) | `FC_EVT_STATE(MAIN)`, `FC_EVT_ERROR(ERR_DROGUE_FAIL)`, `FC_EVT_PYRO(main_ch)` |
| MAIN → LANDED | `FC_EVT_STATE(LANDED)` |

---

## 10  Open Items

1. **TUMBLE state (0xA):** Currently unused. Could be implemented as a tilt-angle
   check during COAST (e.g., >60° from vertical for >500 ms → deploy drogue as
   safety measure). Deferred until needed.

2. **Backup pyro channel on continuity loss:** If an e-match breaks continuity
   between APOGEE and MAIN, the backup pyro channel (if configured) should be
   tried. Requires dual-channel configuration in MC. Out of current scope.

3. **GPS-aided landing detection:** ZUPT + EKF inertial stack is the primary
   landing detector. If GPS is available with a valid fix, GPS vertical velocity
   can serve as a secondary confirmation, but the inertial solution takes
   precedence. Requires GPS update implementation (EKF_SPEC.md §2.9).

4. **HIL virtual clock injection:** Define the USB command format for injecting
   `fsm_input_t` + virtual timestamp from MATLAB. Needs a new `msg_id` in the
   command dispatch table (PYRO_SPEC.md §5.3).
