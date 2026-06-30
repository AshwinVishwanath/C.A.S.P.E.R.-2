# T08 — Stripped ESKF MATLAB Port (4-State Vertical EKF)

## 1. Goal

Port the C.A.S.P.E.R.-2 firmware's 4-state vertical EKF from `Software/App/nav/casper_ekf.c` into a MATLAB Function block, with the Phase 0 stripping rules applied (`ARCHITECTURE.md` §7).

The port must match the firmware's numerical behavior exactly on the items listed as "kept" in `ARCHITECTURE.md` §7. The "stripped" items are deliberately simpler than the firmware. Phase 1 reintroduces them.

Get this right and the trust gate passes. Get the kept-items wrong and the gate fails in subtle ways that look like sensor model bugs — be careful.

## 2. Inputs

| Input | Source | Notes |
|---|---|---|
| Sensor params | T02 `Estimator` struct | All EKF constants live here |
| Firmware-frame sensor signals | T07 frame-switch output | Body accel in firmware frame |
| Truth Mach (for gate) | T01 truth bus | Used to drive `mach_gate_in` boolean |
| Truth attitude (for nav rotation) | T01 truth bus, frame-switched | Body accel rotated to nav via current attitude estimate |
| Baro altitude | T04 output | Drives baro update step |
| Baro valid bit | T04 output | Block updates if false |
| Time | sim clock | Predict step runs at 1/`Estimator.PredictDt_s` = 416 Hz |

## 3. Outputs

In `Software/Sim/build/T08_eskf_port/`:

| File | Purpose |
|---|---|
| `casper_eskf_predict.m` | MATLAB Function: predict step (state + covariance) |
| `casper_eskf_update_baro.m` | MATLAB Function: scalar baro update (Joseph form) |
| `casper_eskf_update_zupt.m` | MATLAB Function: zero-velocity update (gate-bypassed) |
| `casper_eskf_mach_gate.m` | MATLAB Function: Mach gate hysteresis + ungate scheduler |
| `casper_eskf_state.m` | Persistent-state container (state + covariance + ungate counter + last_mach_gate_state) |
| `build_eskf_block.m` | Simulink build script |
| `test_eskf_port.m` | Unit tests (against firmware C reference where possible) |
| `eskf_block.slx` (or library) | Subsystem artifact |
| `plots/eskf_states.png` | All 4 states vs truth over full trajectory |
| `plots/eskf_innovations.png` | Baro and ZUPT innovations with gate boundaries |
| `plots/eskf_covariance.png` | All P diagonal entries over time |
| `STATUS.md` | acceptance status |

## 4. State vector

```
x = [ alt_m         ]    (state 1: altitude, +up)
    [ vel_mps       ]    (state 2: vertical velocity, +up)
    [ accel_bias    ]    (state 3: accelerometer bias, m/s²)
    [ baro_bias     ]    (state 4: barometer bias, m)
```

All four states are scalars (vertical channel only). Position/velocity in horizontal axes are not estimated by the firmware EKF; the attitude estimator owns rotation, but horizontal velocity is unobserved without GPS-derived velocity, which is currently out of scope for the EKF (GPS feeds the FSM and downlink, not the EKF, in current firmware).

Initial state:
- `x(1) = baro_altitude_at_init` (the first baro reading)
- `x(2) = 0`
- `x(3) = 0`
- `x(4) = 0`

Initial covariance: diag([P0_ALT, P0_VEL, P0_ACCEL_BIAS, P0_BARO_BIAS]) = diag([0.1, 0.001, 0.025, 0.75]).

## 5. Predict step (`casper_eskf_predict.m`)

Runs at **fixed 416 Hz** in Phase 0 (`EKF_DT = 0.0024 s`). No adaptive dt; that is in the "stripped" list.

### 5.1 Inputs to the predict step

- Current state `x` (4×1, persistent)
- Current covariance `P` (4×4, persistent)
- Sensor input: vertical specific force in nav frame `accel_up_specific_mps2` (computed by upstream — see §5.2)

### 5.2 Computing the nav-frame vertical accel

In the firmware (see `casper_ekf.c` predict step comment), the sequence is:
1. Read accel in body frame: `accel_body_mps2`.
2. Rotate to nav (Z-up) frame using current attitude estimate (held in attitude estimator, T09): `accel_nav_mps2 = C_body_to_nav * accel_body_mps2`.
3. The Z-component of `accel_nav_mps2` is what feeds the predict step.

The firmware comment is explicit: `accel_nav_mps2[2]` reads `+9.81` on the pad (specific force opposing gravity). After subtracting `G_ACCEL`, what remains is the actual upward dynamic acceleration.

The Phase 0 sim does the same. The "current attitude" comes from T09's output running in parallel.

### 5.3 State propagation

```
a_up   = accel_nav_mps2_z - G - x(3)        % G = 9.80665 m/s²
x(1)   = x(1) + x(2) * dt + 0.5 * a_up * dt²
x(2)   = x(2) + a_up * dt
x(3)   = x(3)                                % bias states random-walk; deterministic in predict
x(4)   = x(4)
```

with `dt = 0.0024 s`.

### 5.4 Covariance propagation

```
Φ = [ 1   dt   -0.5*dt²   0 ;
      0    1   -dt        0 ;
      0    0    1         0 ;
      0    0    0         1 ]

Q = diag([ (ACCEL_VRW)² * dt³ / 3 ,             % alt-alt process noise
           (ACCEL_VRW)² * dt          ,         % vel-vel
           (ACCEL_BI_SIGMA)² * dt     ,         % accel-bias RW
           (BARO_BI_SIGMA)² * dt      ])        % baro-bias RW

P = Φ * P * Φ' + Q
P = 0.5 * (P + P')                              % force symmetry
```

The exact Q-matrix form must match the firmware. Read `casper_ekf.c` carefully to confirm whether the firmware uses the strict integrated-noise form (`(ACCEL_VRW)² * dt³ / 3` for the alt-alt component, off-diagonal alt-vel cross-coupling) or a diagonal-only approximation. **Adopt whatever the firmware does, verbatim**, even if a more "correct" form exists. Sim must match firmware.

### 5.5 P-floor

After every predict (and update) step:
```
P(4,4) = max(P(4,4), P_FLOOR_BARO_BIAS)        % = 0.01
```

This prevents the baro-bias state's variance from collapsing to zero during long stable phases.

## 6. Baro update (`casper_eskf_update_baro.m`)

Runs at the baro output rate (100 Hz nominal). Inputs: noisy `baro_altitude_m`, `mach_gate_active` flag from `casper_eskf_mach_gate.m`.

### 6.1 Innovation

```
H = [1, 0, 0, 1]                         % measurement = altitude + baro_bias
y_pred = x(1) + x(4)                     % predicted measurement
innov = baro_altitude_m - y_pred
S = H * P * H' + R                       % innovation variance
```

where R depends on Mach-gate state:
- Default: `R = R_BARO = 0.5`
- For the `N_UNGATE_STEPS = 10` baro updates immediately after gate disengages: `R = R_BARO_UNGATE = 50.0`

### 6.2 Innovation gate

```
if innov² > BARO_GATE_K2 * S      % BARO_GATE_K2 = 25 (5-sigma squared)
    reject this update, return without modifying x or P
end
```

Important: this is the innovation gate. ZUPT does NOT use this gate (bypassed via INFINITY — see §7).

### 6.3 Mach gate

If `mach_gate_active` is true (Mach > 0.40 with hysteresis), skip the baro update entirely. The Mach gate is *outside* the innovation gate — it prevents the update from running at all during transonic flight.

### 6.4 Joseph-form update

```
K = (P * H') / S                              % 4×1
I_KH = eye(4) - K * H
P_new = I_KH * P * I_KH' + K * R * K'         % Joseph form (numerically stable)
P_new = 0.5 * (P_new + P_new')                % force symmetry
x_new = x + K * innov
```

Joseph form is mandatory. Do not substitute the simpler `(I - KH) * P` form even though it's mathematically equivalent — it loses precision in finite arithmetic.

### 6.5 P-floor (post-update)

Apply `P(4,4) = max(P(4,4), 0.01)` after the update too.

## 7. ZUPT update (`casper_eskf_update_zupt.m`)

Triggered when external logic flags "vehicle is stationary". In firmware, this is gated by FSM == PAD. In Phase 0, the trigger is: `abs(accel_body_magnitude - G) < EKF_ZUPT_THRESHOLD` (0.3 m/s²).

### 7.1 Innovation

```
H = [0, 1, 0, 0]                              % measurement = velocity
y_pred = x(2)
innov = 0 - y_pred                            % pseudo-measurement: velocity = 0
S = H * P * H' + R_ZUPT                       % R_ZUPT = 6.15e-6
```

### 7.2 NO innovation gate

This is the critical firmware behavior to preserve. In the firmware:
```
casper_ekf_update(ekf, x, H, R, INFINITY)     // gate-K² = INFINITY
```

The gate threshold of `INFINITY` means the chi-squared check always passes. This is a hard-won fix from earlier debugging — gating ZUPT caused filter divergence during low-velocity boost-end phase.

**Phase 0 must preserve this**. Implement the ZUPT update by reusing the Joseph-form code path, but pass `gate_K2 = inf` so the rejection check never triggers.

### 7.3 Joseph-form update

Same as baro update, but with `H = [0, 1, 0, 0]` and `R = R_ZUPT`.

## 8. Mach gate (`casper_eskf_mach_gate.m`)

Hysteresis state machine:
```
if not currently_gated and mach > MACH_GATE_ON   (0.40)
    currently_gated = true
    ungate_counter = N_UNGATE_STEPS              (10)    % loaded when gate releases
elseif currently_gated and mach < MACH_GATE_OFF  (0.35)
    currently_gated = false
    % apply ungate inflation now:
    x(3) = 0                                     % reset accel bias to zero
    x(4) = 0                                     % reset baro bias to zero
    P(3,3) = P_UNGATE_ACCEL_BIAS                 (1.0)
    P(4,4) = P_UNGATE_BARO_BIAS                  (10.0)
end
```

When `currently_gated == false` but `ungate_counter > 0`, the next baro update uses inflated `R = R_BARO_UNGATE = 50.0`, and `ungate_counter` decrements. After 10 inflated updates, R returns to normal `R_BARO = 0.5`.

This block also outputs the gate state as a boolean signal for downstream logging and the validation block.

## 9. Persistent state

The MATLAB Function block must hold the following persistent variables:
- `x` (4×1)
- `P` (4×4)
- `mach_gate_active` (bool)
- `ungate_counter` (int)
- `initialized` (bool)
- `last_baro_for_init` (scalar)

On first call (`initialized == false`):
- Initialize `x` from current baro reading.
- Initialize `P = diag([P0_ALT, P0_VEL, P0_ACCEL_BIAS, P0_BARO_BIAS])`.
- Set `mach_gate_active = false`, `ungate_counter = 0`, `initialized = true`.

On Simulink subsystem reset (Initialize Function callback), clear all persistent variables so two runs with same seed produce identical state trajectories.

## 10. Block I/O

Subsystem inputs (`Simulink.Bus` `EkfInBus`):
- `accel_body_fw_mps2` (3×1) — firmware-frame body accel
- `attitude_quat_fw` (4×1) — current attitude estimate (Hamilton, body-fw to Zup)
- `baro_altitude_m` (scalar)
- `baro_valid` (bool)
- `baro_new_sample` (bool, pulses true at each baro 100 Hz sample)
- `truth_mach` (scalar, from truth bus; used by gate)
- `zupt_trigger` (bool) — from external comparator: `abs(|accel_body| - G) < threshold`
- `zupt_new_sample` (bool, pulses true at IMU rate when accel-magnitude check fires)

Subsystem outputs (`EkfOutBus`):
- `state_x` (4×1)
- `covariance_P` (4×4)
- `mach_gate_active` (bool)
- `ungate_counter` (int)
- `last_baro_innov` (scalar, for diagnostics)
- `last_baro_innov_var` (scalar, the `S` from §6.1)
- `last_zupt_innov` (scalar)

## 11. Acceptance criteria

1. **Static run on pad (5 s, no motion)**:
   - Final velocity state `|x(2)| < 0.01 m/s` (ZUPT converged).
   - Final altitude state `|x(1) - truth_alt| < 0.1 m`.
   - Final accel bias `|x(3)| < 0.05 m/s²`.
   - Final baro bias `|x(4)|` matches initial baro offset to within 0.3 m.
   - `P(4,4) >= P_FLOOR_BARO_BIAS = 0.01`.

2. **Mach gate timing**:
   - Gate fires within 0.25 s of `truth.mach` crossing 0.40 upward.
   - Gate releases within 0.5 s of `truth.mach` crossing 0.35 downward.
   - No baro updates fire while gated. Verify in `plots/eskf_innovations.png`.

3. **Ungate recovery**:
   - At gate-release, `x(3) := 0` and `x(4) := 0`.
   - `P(3,3) := 1.0` and `P(4,4) := 10.0` exactly.
   - Next 10 baro updates use `R = 50.0`.
   - `ungate_counter` returns to 0 after 10 updates.

4. **Full trajectory apogee**:
   - `max(state_x_history(:,1))` within ±10 m of truth apogee.

5. **Vertical velocity at burnout**:
   - State `x(2)` at burnout time within ±2 m/s of truth `vel_v_mps`.

6. **Joseph form numerical stability**:
   - After every update, `P` is positive semi-definite (`eig(P) >= -1e-9`).
   - `P` is symmetric to within `1e-12`.

7. **Determinism**:
   - Two runs with same seed produce byte-identical state history.

8. **Performance**:
   - Full 549 s sim of just T08 block (truth driving sensor inputs, no other blocks) runs in < 30 s wall-clock.

9. **No NaN/Inf**: ever.

10. **Comparison against firmware C reference** (optional but strongly recommended): generate identical inputs to a pad-only test, run firmware code in a unit-test harness, compare state trajectories. Must agree to within 1e-9 absolute (allowing for IEEE 754 operation-order ambiguity).

## 12. Anti-goals (preserve firmware behavior, do not "improve")

- Do NOT add an innovation gate on ZUPT updates.
- Do NOT add adaptive dt to the predict step.
- Do NOT add a gyro temperature compensation hook here (lives in T09 or stripped).
- Do NOT add measurement gating on ZUPT.
- Do NOT add velocity-state-direct GPS update (firmware does not).
- Do NOT add measurement preprocessing (median filter, etc.) here. Take the baro reading as given.
- Do NOT use `[I-KH]*P` instead of Joseph form, even though they're mathematically equivalent.
- Do NOT skip the P-floor on baro-bias variance.

## 13. Hand-off

T08's output feeds T10 validation. T10 compares `state_x` to truth altitude/velocity, and verifies the Mach-gate timing.

T08 is independent of T09 except for the rotation-to-nav step in §5.2 — the EKF needs the current attitude estimate to rotate body accel into nav frame. In the Simulink model, T09 (attitude) and T08 (EKF) run at different rates with the EKF predict consuming T09's latest attitude every predict step.

## 14. References to firmware

- `Software/App/nav/casper_ekf.c` — full reference implementation
- `Software/App/nav/casper_ekf.h` — public API and constants
- `Software/App/nav/casper_attitude.c` (for nav-rotation step)
- `FIRMWARE_CONSTANTS.md` §1 (this PRD's authoritative constants)
- `ARCHITECTURE.md` §7 (what's stripped, what's kept)

## 15. STATUS.md template

```
# T08 ESKF Port — STATUS

## Files created
- casper_eskf_predict.m
- casper_eskf_update_baro.m
- casper_eskf_update_zupt.m
- casper_eskf_mach_gate.m
- casper_eskf_state.m
- build_eskf_block.m
- test_eskf_port.m

## Acceptance criteria
| # | Criterion | Status | Notes |
|---|---|---|---|
| 1 | Static pad convergence | PASS | v=0.003 m/s, alt err 0.04 m |
| 2 | Mach gate timing | PASS | engaged 2.31 s, released 58.4 s |
| 3 | Ungate recovery | PASS | counters and inflation correct |
| 4 | Full trajectory apogee | PASS | error 4.2 m |
| 5 | Burnout velocity | PASS | error 0.8 m/s |
| 6 | Joseph form | PASS | min eig 2.1e-7, max asym 4e-15 |
| 7 | Determinism | PASS | byte-identical |
| ... | | | |

## Deviations from spec
- (any)

## Notes for T10/T11
- (anything downstream needs)
```
