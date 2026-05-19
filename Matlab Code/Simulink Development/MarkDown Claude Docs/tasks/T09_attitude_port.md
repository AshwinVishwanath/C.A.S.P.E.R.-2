# T09 — Stripped Attitude Estimator MATLAB Port (Mahony + RK4)

## 1. Goal

Port `Software/App/nav/casper_attitude.c` (and `casper_quat.c` quaternion math) to MATLAB / Simulink, with the Phase 0 stripping rules applied (`ARCHITECTURE.md` §7).

The attitude estimator is a hybrid Mahony complementary filter (on the pad) + RK4 gyro propagation with 10 Hz tilt-compensated mag correction (in flight). Phase 0 keeps the core algorithms exactly but removes the gyro temperature compensation, stationary EMA bias estimation, ignition-detection magnetometer gating, and FSM-driven mode switching.

## 2. Inputs

| Input | Source | Notes |
|---|---|---|
| Sensor params | T02 `Attitude` struct | All gains, ARW, etc. |
| IMU body accel (firmware frame) | T07 frame-switch output | Body-frame accel in m/s² |
| IMU body gyro (firmware frame) | T07 frame-switch output | Body-frame gyro in rad/s |
| Mag body field (firmware frame) | T07 frame-switch output | µT |
| Mag new-sample bit | T05 output | 100 Hz raw; only 10 Hz reaches correction (decimated) |
| Pad/flight mode bit | external | true = pad, false = flight. Phase 0 sets this from truth (truth velocity < 0.5 m/s) |
| Reference mag (Zup) | computed at init from launch site | 3×1 µT in firmware-frame Z-up |

## 3. Outputs

In `Software/Sim/build/T09_attitude_port/`:

| File | Purpose |
|---|---|
| `casper_quat_ops.m` | Hamilton quaternion ops (mult, conj, normalize, dcm) |
| `casper_attitude_static_init.m` | Pad-state accel/mag averaging, initial quat |
| `casper_attitude_predict_rk4.m` | RK4 gyro integration |
| `casper_attitude_mahony.m` | Pad mode: accel+mag complementary correction |
| `casper_attitude_mag_correct_flight.m` | Flight mode: 10 Hz tilt-compensated mag correction |
| `casper_attitude_gyro_lpf.m` | First-order IIR gyro LPF (50 Hz cutoff) |
| `build_attitude_block.m` | Simulink build script |
| `test_attitude_port.m` | Unit tests |
| `attitude_block.slx` (or library) | Subsystem artifact |
| `plots/attitude_pad_initialization.png` | Pad-state convergence |
| `plots/attitude_flight_tracking.png` | Truth vs estimate Euler angles |
| `plots/attitude_error_euler.png` | Error in degrees, full flight |
| `STATUS.md` | acceptance status |

## 4. Quaternion conventions

Hamilton, scalar-first `[w, x, y, z]`. Body-to-nav (firmware Z-up). Multiplication is Hamilton-product order: `q_a ⊗ q_b` means "first apply rotation `b`, then `a`".

Identity quaternion: `[1, 0, 0, 0]`.

Sign convention: enforce `q.w >= 0` after every operation. Flip sign of all four components if `q.w < 0`. This eliminates the double-cover ambiguity.

All quaternion operations live in `casper_quat_ops.m`:
- `q = quat_mult(a, b)`
- `q = quat_conj(a)`
- `q = quat_normalize(a)`
- `R = quat_to_dcm(a)`
- `q = dcm_to_quat(R)`
- `v_nav = quat_rotate_vec(q, v_body)`  (apply rotation `q` to body-frame vector)

Implement in raw arithmetic, no library calls. Match `casper_quat.c` operation order exactly.

## 5. Static initialization (`casper_attitude_static_init.m`)

Runs once at t=0, while vehicle is on the pad.

### 5.1 Accel-based tilt estimation

Average 100 IMU samples (~0.12 s at 833 Hz). The averaged accel `a_avg_body` points in the direction of gravity reaction (specific force, +up on pad). In firmware Z-up frame, gravity-reaction is `[0, 0, +G]`.

Compute initial roll/pitch (about firmware body X and Z, respectively) that aligns `a_avg_body` with the firmware-body's nose-up axis. In firmware, "nose-up" on the pad means body +Y = nav +Z (Z-up).

Construct the initial quaternion that rotates body-nose (Y_body) onto nav-up (Z_nav):
```
nose_body = [0, 1, 0]                             % firmware body Y = nose
nose_nav_target = a_avg_body / ||a_avg_body||     % unit vector along specific force, body frame
% the rotation from body-nose to nav-up via gravity-reaction
% in firmware body frame, gravity reaction reads ≈ [0, +G, 0] on pad → matches nose direction
% so the body→nav DCM has +Y_body mapping to +Z_nav
```

In the simplest form: if `a_avg_body` is exactly `[0, +G, 0]`, then `q_init` rotates body Y onto nav Z, which is a 90° rotation about body X (or equivalently, nav Y). Compute via Rodrigues formula from the cross product `nose_body × Z_nav_in_body`.

In practice, if the pad isn't perfectly vertical, the averaged accel will have small X and Z components that tilt the initial quaternion off vertical. That's correct behavior — the EKF/attitude will use this real tilt.

### 5.2 Mag-based yaw resolution

Average `STATIC_INIT_MAG_SAMPLES = 500` mag samples (~5 s at 100 Hz) or wait up to `STATIC_INIT_TIMEOUT_S = 10 s`.

The averaged mag `m_avg_body` is rotated through the partial `q_init` to get a nav-frame mag estimate. Compare to the expected nav-frame Earth field (computed from launch site lat/lon via `Mag.LaunchSite.Inclination_deg` and `Mag.LaunchSite.Declination_deg`). The yaw angle between the rotated and expected mag gives the initial yaw correction.

Apply the yaw rotation:
```
q_init = q_yaw_correction ⊗ q_init_tilt_only
```

After both corrections, `q_init` is the body-to-nav quaternion for the pad-stationary orientation.

### 5.3 Bias initialization

```
gyro_bias_init = mean of first 100 gyro samples     % body-frame, rad/s
gyro_bias = gyro_bias_init
```

Phase 0 freezes this at the static value. No online EMA update (stripped per `ARCHITECTURE.md` §7).

## 6. Gyro LPF (`casper_attitude_gyro_lpf.m`)

First-order IIR, cutoff 50 Hz, sample rate 833 Hz.

```
alpha = dt / (RC + dt)                                       % standard first-order coefficient
        where RC = 1 / (2 * pi * Attitude.GyroLPFCutoff_Hz)
gyro_filt[k] = alpha * gyro_raw[k] + (1 - alpha) * gyro_filt[k-1]
```

Initialized with the first raw gyro sample at static_init end.

Apply per-axis independently (3 parallel first-order filters).

## 7. Gyro propagation — RK4 (`casper_attitude_predict_rk4.m`)

Runs at IMU rate (833 Hz). Inputs: filtered, bias-corrected gyro `omega_body`.

### 7.1 RK4 integration

The quaternion derivative is `dq/dt = 0.5 * q ⊗ [0, omega_body]`. RK4 four-step:

```
dt = 1/833 s                                            % matches IMU rate
omega_corrected_k1 = omega_body - gyro_bias             % at start of step

k1 = 0.5 * q ⊗ [0, omega_body_at_start]
q_mid_1 = q + 0.5 * dt * k1
q_mid_1 = normalize(q_mid_1)
k2 = 0.5 * q_mid_1 ⊗ [0, omega_body_at_mid]            % use the same omega; could interpolate if 2x IMU avail
q_mid_2 = q + 0.5 * dt * k2
q_mid_2 = normalize(q_mid_2)
k3 = 0.5 * q_mid_2 ⊗ [0, omega_body_at_mid]
q_end_pre = q + dt * k3
q_end_pre = normalize(q_end_pre)
k4 = 0.5 * q_end_pre ⊗ [0, omega_body_at_end]

q_new = q + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
q_new = normalize(q_new)
```

In firmware, the same `omega_body` value is used for all four sub-steps (no interpolation between samples). Phase 0 sim matches.

### 7.2 Normalize after each step

Floating-point drift makes the quaternion lose unit norm. Renormalize after each RK4 step:
```
q_new = q_new / ||q_new||
if q_new.w < 0
    q_new = -q_new
end
```

## 8. Mahony complementary correction (`casper_attitude_mahony.m`)

Runs on the pad (when mode bit is "pad"). 833 Hz. Inputs: filtered/bias-corrected gyro `omega_body`, accel `a_body`, mag `m_body`, expected mag in nav.

### 8.1 Predicted gravity and mag in body

```
g_nav = [0, 0, G]                                       % nav-frame gravity (Z-up: pointing up)
m_nav = [Mag.ExpectedField_uT_NED]                       % rotated from launch-site Earth field
q = current quaternion
g_body_pred = quat_rotate_vec(quat_conj(q), g_nav)
m_body_pred = quat_rotate_vec(quat_conj(q), m_nav)
```

### 8.2 Error vectors

```
a_meas_norm = a_body / ||a_body||
m_meas_norm = m_body / ||m_body||

err_grav = cross(a_meas_norm, g_body_pred / ||g_body_pred||)
err_mag  = cross(m_meas_norm, m_body_pred / ||m_body_pred||)

err_total = Attitude.Mahony.Kp_grav * err_grav + Attitude.Mahony.Kp_mag_pad * err_mag
```

### 8.3 Apply correction

```
omega_correction = err_total
omega_corrected = omega_body - gyro_bias + omega_correction
```

Then run the RK4 integration with this corrected omega. The complementary filter steers the quaternion's pitch/roll using the accel reading and yaw using the mag reading, with gains `Kp_grav` and `Kp_mag_pad`.

### 8.4 Integral term (slow bias update)

```
gyro_bias_integral_update = Attitude.Mahony.Ki * err_total * dt
% Note: in Phase 0 stripped mode, this update is NOT applied (we keep gyro_bias static).
% In Phase 1, this would update gyro_bias online.
```

Phase 0 ignores this integral term. Phase 1 restores it.

## 9. Flight-mode mag correction (`casper_attitude_mag_correct_flight.m`)

Runs at 10 Hz (decimated). Tilt-compensated yaw-only correction.

### 9.1 Tilt-compensated mag

Project the measured mag onto the local horizontal plane (perpendicular to gravity), and project the expected nav-frame mag the same way. The angle between them is the yaw error.

```
g_body_pred = quat_rotate_vec(quat_conj(q), [0,0,G])
m_body_horizontal = m_body - dot(m_body, g_body_pred / ||g_body_pred||) * (g_body_pred / ||g_body_pred||)
m_nav_horizontal = m_nav_expected - dot(m_nav_expected, [0,0,1]) * [0,0,1]
m_nav_horizontal_in_body = quat_rotate_vec(quat_conj(q), m_nav_horizontal)

% Heading error: cross product gives yaw correction direction
err_yaw = cross(m_body_horizontal/||m_body_horizontal||, m_nav_horizontal_in_body/||m_nav_horizontal_in_body||)

% Only the gravity-aligned component is yaw-relevant
err_yaw_scalar = dot(err_yaw, g_body_pred / ||g_body_pred||)
```

### 9.2 Apply yaw correction

```
omega_correction = err_yaw_scalar * (g_body_pred / ||g_body_pred||) * Attitude.Mahony.Kp_mag_flight
omega_corrected = omega_body - gyro_bias + omega_correction
```

The correction only nudges the heading; pitch and roll come from the gyro integration alone in flight.

### 9.3 Heading uncertainty floor

```
heading_sigma = max(heading_sigma_state, HEADING_SIGMA_FLOOR)        % HEADING_SIGMA_FLOOR = 0.01 rad
```

In Phase 0, this is informational only (used for plotting). Phase 1 may use it for adaptive correction strength.

## 10. Mode switching

The "mode" (pad vs flight) is an external boolean to the block. In Phase 0:
- Pad mode: `|truth.vel_mps| < 0.5 m/s` (low velocity proxies for "on the pad").
- Flight mode: otherwise.

Mode transitions:
- Pad → flight: capture current `q` as `q_at_launch_snapshot` and store as a reference. Disable Mahony.
- Flight → pad: re-enable Mahony, reset integral terms.

In Phase 0, the trajectory only goes pad→flight once (at t≈0). The block handles the single transition cleanly.

## 11. Block I/O

Inputs (`AttitudeInBus`):
- `accel_body_fw_mps2` (3×1)
- `gyro_body_fw_rps` (3×1)
- `mag_body_fw_uT` (3×1)
- `mag_new_sample` (bool, 100 Hz from T05; only every 10th sample is consumed)
- `mode_pad` (bool)

Outputs (`AttitudeOutBus`):
- `quat_body_to_nav` (4×1, scalar-first Hamilton)
- `gyro_bias_rps` (3×1, frozen at static init in Phase 0)
- `heading_sigma_rad` (scalar)
- `init_complete` (bool)

## 12. Acceptance criteria

1. **Static init convergence**:
   - After 5 s of pad-mode operation, the recovered quaternion `q_est` matches truth `q_truth` to within 1° (3-axis Euler error).
   - The yaw correction from mag completes within `STATIC_INIT_TIMEOUT_S = 10 s`.

2. **Gyro-only propagation accuracy**:
   - With Mahony disabled and clean (no-noise) truth gyro fed in, integration over 60 s drifts < 0.5° about each axis.

3. **Quaternion unit norm**:
   - `||q_est||` remains in `[0.99999, 1.00001]` for the entire trajectory.

4. **Mahony correction operates on pad**:
   - With noisy IMU and constant truth pose, the estimator converges from a deliberately-perturbed initial quaternion (e.g., 5° tilt error) to truth within 5 s.

5. **Flight-mode mag correction operates at 10 Hz**:
   - Verify the correction is applied at every 10th raw mag sample (10 Hz with 100 Hz mag raw).
   - Yaw error during flight does not exceed 5° even with no roll observability.

6. **Heading sigma floor**:
   - `heading_sigma_rad >= 0.01` always.

7. **Truth tracking on full trajectory**:
   - Tilt RMS error during powered flight: < 1°.
   - Tilt RMS error during coast: < 2°.

8. **Determinism**:
   - Two runs with same seed produce byte-identical quaternion histories.

9. **No NaN/Inf**: ever.

10. **Performance**:
   - Block runs full 549 s sim in < 30 s wall-clock.

## 13. Anti-goals (preserve stripped scope)

- Do NOT apply gyro temperature compensation (stripped).
- Do NOT update gyro bias online via EMA (stripped).
- Do NOT gate mag corrections on motor-current-detected ignition (stripped — FSM not present).
- Do NOT use a flight FSM to drive mode switching beyond the simple `velocity < 0.5 m/s` test.
- Do NOT add an "accel-launch-detect" hook here. That's flight-loop logic, not attitude.
- Do NOT use the heading sigma to gate or weight corrections in Phase 0. Phase 1 may.
- Do NOT replace Hamilton convention with JPL convention. Match firmware.

## 14. Hand-off

T09's output `quat_body_to_nav` feeds T08's predict step (rotation of body accel to nav). The two blocks run in parallel inside the integration model.

T10 validation reads `quat_body_to_nav` and compares to `quat_truth_fw` (truth attitude after T07 frame switch). The error metric is the angle of `quat_truth_fw^-1 ⊗ quat_est`.

## 15. References to firmware

- `Software/App/nav/casper_attitude.c` — full reference
- `Software/App/nav/casper_attitude.h`
- `Software/App/nav/casper_quat.c`, `casper_quat.h` — quaternion ops
- `Software/App/cal/mag_cal.c` — mag calibration (informational)
- `FIRMWARE_CONSTANTS.md` §2 (this PRD's authoritative constants)
- `ARCHITECTURE.md` §7

## 16. STATUS.md template

```
# T09 Attitude Port — STATUS

## Files created
- casper_quat_ops.m
- casper_attitude_static_init.m
- casper_attitude_predict_rk4.m
- casper_attitude_mahony.m
- casper_attitude_mag_correct_flight.m
- casper_attitude_gyro_lpf.m
- build_attitude_block.m
- test_attitude_port.m

## Acceptance criteria
| # | Criterion | Status | Notes |
|---|---|---|---|
| 1 | Static init | PASS | converged to 0.04° in 1.2 s |
| 2 | Gyro-only drift | PASS | 0.12° / 60 s |
| 3 | Quaternion norm | PASS | max deviation 4e-13 |
| ... | | | |

## Stripped items
- gyro temp comp: not implemented (Phase 0 stripped)
- online bias EMA: not implemented (Phase 0 stripped)
- ignition-gated mag: not implemented (no FSM in Phase 0)

## Deviations from spec
- (any)
```
