# ARCHITECTURE.md — Locked Decisions

This file enumerates architectural decisions that are **closed for Phase 0**. Sub-agents must not re-litigate them. If a task file conflicts with this file, this file wins. Manager: if you believe a locked decision needs revisiting, halt and escalate to the user; do not unilaterally change anything here.

## 1. Approach Summary

- **Truth source (Phase 0)**: RasAero CSV (`Flight_Test.CSV`), force-posed. No 6-DOF EOM in Phase 0.
- **Sim nav frame**: Standard NED (X=North, Y=East, Z=Down), Aerospace Blockset native.
- **Sim body frame**: Standard aircraft body (X=forward/nose, Y=right/starboard, Z=down).
- **Firmware nav frame**: Z-up local-level. Despite firmware variable names containing "ned", gravity reads `+9.81` on the Z axis (Z is up, not down).
- **Firmware body frame**: Y=nose, X=starboard, Z=toward-operator. Non-standard.
- **HIL boundary**: One fixed rotation block between sim-frame and firmware-frame. NED↔Z-up = sign flip on Z; body permutation matrix is constant.
- **Estimator under test**: stripped MATLAB port of `casper_ekf.c` + `casper_attitude.c`. See §7 for what "stripped" means.
- **Acceptance criterion**: see `PHASE0_SPEC.md` §3 (trust gate).

## 2. MATLAB / Simulink Stack

Required toolboxes (user confirmed all are licensed):
- Simulink (R2024a or later)
- Aerospace Blockset — atmosphere, wind, 6-DOF (Phase 1+ only)
- Sensor Fusion and Tracking Toolbox — `imuSensor`, `magnetometer`, `gpsSensor` objects
- Signal Processing Toolbox — filters, PSD, spectrum analysis
- DSP System Toolbox — colored noise generators
- MATLAB Coder (optional, for later Phase 2+ HIL deployment)

Solver: fixed-step `ode4` (Runge-Kutta 4), dt = **1e-4 s** (10 kHz) at the top level. Rate transitions inside the model down-sample to per-sensor rates. No variable-step solvers anywhere — sim must be byte-deterministic across runs.

## 3. Frame Conventions

### 3.1 Sim-side conventions (Aerospace Blockset native)
- **Nav**: NED. Position +Z = down. Gravity vector in nav frame: `g_NED = [0, 0, +9.80665]`.
- **Body**: X-fwd, Y-right, Z-down. On a vertical rocket on the pad, body +X points up.
- **Quaternion**: scalar-first `[q0, q1, q2, q3]`, body-to-nav. This matches Aerospace Blockset's 6-DOF block convention. **q0 is scalar**, matching the firmware's `[w, x, y, z]` naming.

### 3.2 Firmware-side conventions (Z-up local-level)
- **Nav**: Z-up. Position +Z = up. Gravity vector: `g_Zup = [0, 0, +9.80665]` (yes, same sign — see `casper_ekf.c` predict step comment).
- **Body**: Y=nose, X=starboard, Z=toward operator. On the pad, body +Y points up. Pad accel reads `[0, +9.80665, 0]`.
- **Quaternion**: Hamilton `[w, x, y, z]`, body-to-nav. From `casper_quat.h`.

### 3.3 The HIL boundary transformation
Implemented once in `T07_frame_switch`. Components:

1. **Position/velocity Z sign**: `pos_firmware[2] = -pos_NED[2]`; same for velocity.
2. **Body axis permutation**: 
   - `body_FW = R_body * body_STD`
   - `R_body` is a constant 3×3 orthogonal matrix that swaps standard X-fwd to firmware Y-nose, and standard Y-right to firmware X-starboard, and standard Z-down to firmware Z-toward-operator.
   - **Exact form determined empirically at T07 implementation** by checking that the on-pad accelerometer reading (standard frame: `[+9.81, 0, 0]` measuring gravity reaction with X=forward=up; firmware frame: `[0, +9.81, 0]`) round-trips identically.
3. **Quaternion compose**: `q_firmware = q_align ⊗ q_std` where `q_align` is a constant Hamilton quaternion baked from `R_body` plus the nav-frame Z flip.

The sub-agent for T07 owns this; everyone else treats it as a black box.

## 4. Sample Rates (locked from firmware)

| Stream | Rate | Source firmware constant / location |
|---|---|---|
| Dynamics solver | 10 kHz | sim choice, `ode4` |
| IMU (LSM6DSO32) | 833 Hz | `lsm6dso32.c` CTRL1_XL = 0x74 |
| ADXL372 (high-g) | 800 Hz post-launch, 400 Hz pre-launch | `adxl372_fifo_init`, `flight_loop.c` FSM PAD→BOOST |
| Baro (MS5611) | ~100 Hz | `ms5611.c` non-blocking tick, OSR_4096 |
| Mag (MMC5983MA) raw | 100 Hz | `mmc5983ma.c` CTRL2 CM_100HZ |
| Mag at EKF | 10 Hz | `casper_attitude.c` `mag_update_hz` |
| GPS (MAX-M10M) | 10 Hz | `max_m10m.c` CFG_RATE_MEAS=100 |
| EKF predict | 416 Hz | `casper_ekf.h` `EKF_DT = 0.0024` |
| Radio TX cadence | 10 Hz (100 ms) | `radio_config.h` `RADIO_TX_PERIOD_MS` |
| Radio SF7 airtime | ~15 ms | LoRa `2^SF/BW + headers` |

Rate transitions: use **Simulink Rate Transition blocks** for explicit cross-rate signal passing. Do not rely on Simulink's auto-insertion. See `references/SIMULINK_PATTERNS.md` §4.

## 5. Sensor Hierarchy & Block Choices

Sub-agents must use stock blocks where available, MATLAB Function blocks elsewhere:

| Sensor | Stock block | MATLAB Function additions |
|---|---|---|
| LSM6DSO32 IMU | `imuSensor` (Sensor Fusion Tbx) | Saturation clip, temp coupling, axis remap |
| ADXL372 high-g | `imuSensor` configured high-g | Saturation clip, FIFO timing |
| MS5611 baro | None (use ISA + custom) | Full custom: COESA + noise + Mach-shock model |
| MMC5983MA mag | `magnetometer` (Sensor Fusion Tbx) | Hard/soft iron, ×-1 flip, AR(1) noise, **radio interference layer** |
| MAX-M10M GPS | `gpsSensor` (Sensor Fusion Tbx) | COCOM dropout, latency |
| Atmosphere | `COESA Atmosphere Model` (Aero Bkst) | — |
| Wind | `Dryden Wind Turbulence (Continuous)` (Aero Bkst) | Disabled for Phase 0 (force-posed) |

The radio-interference model layered on top of the magnetometer is the only Phase 0 custom sensor effect that has no calibration data. Use the **placeholder ±10 µT rectangular pulse** model defined in T05; flag it explicitly in the validation report so it does not get mistaken for a calibrated effect.

## 6. Determinism & Reproducibility

All randomness in the sim must be seedable:
- IMU `imuSensor`: set `RandomStream = 'mt19937ar with seed'` and `Seed = <int>`.
- Mag `magnetometer`: same.
- GPS `gpsSensor`: same.
- Any MATLAB Function block using `randn`/`rand`: use a local `RandStream` instance, seeded from a config parameter.
- Radio TX timing: deterministic schedule (no jitter in Phase 0).

A single config variable `Sim.Seed` (default `20260519`, today's date in YYYYMMDD) in `casper_sim_config.m` controls all seeds. Sub-agents must derive per-sensor seeds from `Sim.Seed` (e.g., `Sim.Seed + 1` for IMU, `+2` for mag, etc.) so that one config change re-seeds everything together.

Two consecutive runs with the same config must produce **identical** sensor streams (binary-identical .mat files). T11 must verify this.

## 7. Stripped Estimator Port — what "stripped" means

The Phase 0 estimator port is intentionally simpler than the flight firmware to make debugging tractable. Differences from `Software/App/nav/`:

### Stripped IN Phase 0 (use simple version):
- **Predict dt**: fixed `1/416 s = 0.0024 s`. No DWT cycle counter, no adaptive dt, no clamping.
- **No BOOST timeout**. The firmware's `FSM_BOOST_MAX_MS = 10000` safety net is gone; Phase 0 estimator runs in pure sensor-driven mode for the full trajectory.
- **No COAST timeout**. Same reason.
- **No gyro stationary EMA bias gate**. Bias is a static parameter set at init, no online update.
- **No gyro temperature compensation**. `GYRO_TC_SLOPE_*` not applied.
- **No baro Mach-shock detection in the EKF**. Mach gate still fires (R inflation), but no separate transonic baro-error model.
- **No flight FSM**. ZUPT fires whenever truth velocity < `EKF_ZUPT_THRESHOLD` (0.3 m/s), not gated on FSM state.
- **No flash logging side effects**, no telemetry packing, no radio TX scheduling.
- **No HIL_MODE branching**. The Phase 0 port is one path.

### Kept FROM firmware (must match exactly):
- 4-state EKF: `[altitude, velocity, accel_bias, baro_bias]`.
- Joseph-form scalar baro update.
- ZUPT bypasses innovation gates (R = `R_ZUPT`, gate = `INFINITY`).
- Mach gate at 0.40 on / 0.35 off.
- Un-gate recovery: bias reset to zero + P inflation for `N_UNGATE_STEPS = 10`.
- All process noise values (`ACCEL_VRW`, `ACCEL_BI_SIGMA`, `BARO_BI_SIGMA`).
- Initial covariance `P0` exactly as in `casper_ekf.c`.
- RK4 gyro propagation in the attitude port.
- Mahony complementary correction on pad (accel + mag).
- 10 Hz tilt-compensated mag correction in flight.
- Sign of `a_up = ned_accel[2] - G - accel_bias` (Z-up convention from firmware).

Phase 1 adds the stripped items back in one at a time, with regression tests against the stripped baseline.

## 8. Build & Output Layout

All build outputs land under `Software/Sim/build/<task_id>_<name>/`. Each sub-agent owns its own subdirectory and must not write to other tasks' directories.

```
Software/Sim/build/
├── T01_truth_pipeline/
│   ├── casper_rasaero_ingest.m
│   ├── casper_truth_resample.m
│   ├── build_truth_pipeline_block.m
│   ├── test_truth_pipeline.m
│   ├── truth_trajectory.mat
│   ├── plots/
│   └── STATUS.md
├── T02_sensor_params/
│   ├── casper_sensor_params.m
│   ├── verify_sensor_params.m
│   └── STATUS.md
├── ... (one per task) ...
└── T11_integration/
    ├── casper_sim_phase0.slx
    ├── casper_sim_config.m
    ├── run_phase0_trustgate.m
    ├── PHASE0_TRUSTGATE_REPORT.md
    └── plots/
```

Each task's `STATUS.md` summarizes outcome and any deviations.

## 9. Naming Conventions

- MATLAB functions: `casper_<purpose>_<verb>.m`, lowercase with underscores, max 40 chars.
- Simulink models: `casper_<purpose>.slx`.
- Build scripts (programmatic Simulink construction): `build_<purpose>.m`.
- Test scripts: `test_<purpose>.m`.
- Parameter structs: capitalized e.g. `Sim`, `IMU`, `Baro`, `Mag`, `GPS`, `Estimator`.
- All names are lowercase except parameter struct names and Simulink block/port labels.

## 10. Coding Standards

- MATLAB R2024a syntax. No `eval`, no dynamic struct field assembly via strings (use `setfield`/`getfield`).
- Every function has an H1 line and a docstring describing inputs, outputs, units, and source firmware reference if applicable.
- All physical quantities carry units in variable names: `alt_m`, `vel_mps`, `accel_g`, `mag_uT`, `time_s`, `time_ms`, `time_ds` (deci-seconds, matching telemetry encoding).
- All MATLAB Function blocks specify input/output types and sizes explicitly (no `(:,1)` ambiguity).
- All Simulink signals carry units via `set_param(line, 'Unit', '...')` where the unit is non-obvious.

## 11. What sub-agents may NOT do

- Modify any file under `Software/App/`, `Software/Drivers/`, or `Software/Core/`.
- Read or modify files outside `Software/Sim/` except as explicitly listed in their task's References.
- Make architectural changes (frame conventions, sample rates, library choices, estimator stripping rules). If something here needs to change, escalate to manager.
- Skip unit tests. Each task's acceptance criteria are mandatory.
- Use floating-point comparison without tolerance (`==` on doubles is a bug).
- Write code that has not been tested by an executable `test_*.m` script in the same directory.

## 12. Glossary

- **HIL**: Hardware-in-the-Loop. Phase 0 has no real hardware in the loop; the term refers to the *eventual* HIL boundary where this sim could feed a real STM32H750 via UART/USB.
- **Trust gate**: the acceptance test at the end of Phase 0 (see `PHASE0_SPEC.md` §3).
- **Force-posed**: vehicle's position/attitude/rates are read directly from the truth source rather than integrated from forces and moments. Phase 0 only.
- **Z-up local-level**: a navigation frame with Z pointing up (away from Earth), as distinct from NED (Z=down) or ENU (East-North-Up). The firmware uses Z-up despite naming its variables `ned_*`.
- **Y-nose body**: a body-fixed frame where the rocket's nose axis is +Y, not the standard +X. C.A.S.P.E.R.-2 firmware convention.

## 13. Out of Scope for Phase 0 (do not implement)

- Aero forward dynamics (6-DOF EOM with thrust, drag, lift) — Phase 1.
- Aero model augmentation (model-predicted measurements fed to EKF) — Phase 2.
- RTS smoother — Phase 3.
- Signed flight logs (HMAC) — Phase 3.
- Active control (servos) — Phase 4.
- Second IMU model (v2 board) — Phase 5 (depends on hardware availability).
- Real radio→mag interference calibration — pending bench data.
- Real flight log replay — pending hardware flight.
- Cross-platform builds (Linux MATLAB) — current target is whatever MATLAB the user has installed.
