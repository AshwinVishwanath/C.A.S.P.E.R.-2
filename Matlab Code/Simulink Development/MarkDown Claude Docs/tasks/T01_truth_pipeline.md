# T01 — Truth Trajectory Pipeline

## 1. Goal

Ingest the RasAero CSV `Flight_Test.CSV`, convert imperial→metric, resample onto a uniform 10 kHz grid, and produce a Simulink `TruthBus` signal that downstream sensor models consume. Phase 0 force-poses the vehicle along this truth; there is no 6-DOF EOM.

## 2. Inputs

| Input | Path | Notes |
|---|---|---|
| RasAero CSV | `<repo_root>/Flight_Test.CSV` | If not at repo root, search; if absent, halt and report. |
| Sim config | base workspace `Sim` struct (created by `casper_sim_config.m`) | At minimum `Sim.SolverDt = 1e-4` and `Sim.Seed`. |
| `casper_sensor_params.m` | not required; T01 is upstream of T02 — do not depend on it |

## 3. Outputs

All files land in `Software/Sim/build/T01_truth_pipeline/`:

| File | Purpose |
|---|---|
| `casper_rasaero_ingest.m` | Function: read CSV, convert units, return raw struct |
| `casper_truth_resample.m` | Function: interpolate raw struct onto uniform-dt time grid |
| `casper_truth_build_bus.m` | Function: package resampled struct into a `Simulink.Bus`-compatible signal |
| `build_truth_pipeline_block.m` | Script: programmatically construct the `truth_source` Simulink subsystem and save as a library block |
| `test_truth_pipeline.m` | Script: full unit test with plots and assertions |
| `truth_trajectory.mat` | Cached resampled struct (run by test script) |
| `plots/raw_vs_resampled.png` | Visual sanity check |
| `plots/truth_full_trajectory.png` | Full-flight summary plot |
| `STATUS.md` | Test result summary |

## 4. RasAero CSV schema

12,800 rows. First row is the header. Time grid is **non-uniform** (10 ms steps early, longer later). All linear quantities are imperial.

Columns (1-indexed):

| Col | Name | Unit | Use |
|---|---|---|---|
| 1 | Time (sec) | s | x-axis |
| 2 | Stage | string ("B" boost, "C" coast, etc.) | informational |
| 3 | Stage Time (sec) | s | ignore |
| 4 | Mach Number | — | populate `TruthBus.mach`; verify against derived value |
| 5 | Angle of Attack (deg) | deg | ignore in Phase 0 (vertical assumption) |
| 6 | CD | — | informational |
| 7 | CL | — | informational |
| 8 | Thrust (lb) | lb | informational |
| 9 | Weight (lb) | lb | informational |
| 10 | Drag (lb) | lb | informational |
| 11 | Lift (lb) | lb | informational |
| 12 | CG (in) | in | informational |
| 13 | CP (in) | in | informational |
| 14 | Stability Margin (cal) | — | informational |
| 15 | Accel (ft/sec²) | ft/s² | total accel magnitude — informational |
| 16 | **Accel-V (ft/sec²)** | ft/s² | **vertical accel — USE** |
| 17 | Accel-H (ft/sec²) | ft/s² | horizontal accel — Phase 0 zero |
| 18 | Velocity (ft/sec) | ft/s | total — informational |
| 19 | **Vel-V (ft/sec)** | ft/s | **vertical velocity — USE** |
| 20 | Vel-H (ft/sec) | ft/s | horizontal velocity — Phase 0 zero |
| 21 | **Pitch Attitude (deg)** | deg | **pitch attitude — USE** |
| 22 | Flight Path Angle (deg) | deg | sanity check vs Vel-V/Vel-H |
| 23 | **Altitude (ft)** | ft | **vertical position — USE** |
| 24 | Distance (ft) | ft | horizontal displacement — Phase 0 zero |

For Phase 0 we treat the trajectory as purely vertical:
- Horizontal velocity, accel, position: set to zero (override the CSV's non-zero horizontal values; they are sub-meter and within RasAero rounding).
- Roll: zero (no rotation about nose axis; later phases may revisit).
- Yaw: zero (no observability without mag; we are not testing mag yaw recovery in Phase 0).
- Pitch attitude: use directly from column 21.

## 5. Unit conversions

- 1 ft = 0.3048 m
- 1 ft/s = 0.3048 m/s
- 1 ft/s² = 0.3048 m/s²
- 1 lb (force) = 4.4482216 N (if Thrust/Drag/Weight ever used — informational only in Phase 0)
- 1 in = 0.0254 m (if CG/CP ever used — informational only)
- deg → rad: multiply by `pi/180`

## 6. Algorithm specification

### 6.1 `casper_rasaero_ingest.m`

**Signature**: `raw = casper_rasaero_ingest(csv_path)`

**Steps**:
1. `readtable(csv_path)` to load all columns.
2. Sanity: assert 24 columns, ≥ 12000 rows, first time = 0.
3. Convert each used column to metric (cols 16, 19, 21, 23 above).
4. Negate or keep sign per convention: in RasAero, altitude is positive-up (matches NED-Z-down → position-Z is negative; remember this for the next step).
5. Return a struct with fields:
   - `t_s` (Nx1, seconds)
   - `mach` (Nx1, dimensionless)
   - `alt_m` (Nx1, positive-up meters from launch)
   - `vel_v_mps` (Nx1, positive-up)
   - `accel_v_mps2` (Nx1, positive-up, net of gravity — this is what the IMU body Y-axis would NOT read, see §7 below)
   - `pitch_deg` (Nx1, deg from horizontal, +90 = vertical)
   - `stage` (Nx1, string)
   - `n_samples` (scalar)

### 6.2 `casper_truth_resample.m`

**Signature**: `truth = casper_truth_resample(raw, dt, t_max)`

**Steps**:
1. Build the uniform time grid: `t_uniform = 0 : dt : t_max`. Default `dt = 1e-4`, `t_max = 549`.
2. For each scalar field (`mach`, `alt_m`, `vel_v_mps`, `accel_v_mps2`, `pitch_deg`): use `interp1` with `'pchip'` (shape-preserving cubic Hermite) and `extrap = 0`. PCHIP avoids the ringing that `'spline'` introduces at sharp transitions like burnout.
3. For `stage`: use `interp1` with `'previous'` and cast back to a categorical or numeric stage ID.
4. Construct `accel_NED` and `pos_NED` and `vel_NED` as 3-vectors per sample (sim-side frame, NED):
   - `pos_NED = [0; 0; -alt_m]`  (Z is down in NED, so altitude up → Z is negative)
   - `vel_NED = [0; 0; -vel_v_mps]`
   - `accel_NED = [0; 0; -accel_v_mps2]`
5. Construct `attitude_quat_std` per sample: pitch about Y_body=right, then assemble Hamilton quaternion `[w, x, y, z]` representing the body-to-NED rotation. In standard aircraft body (X-fwd, Y-right, Z-down), the nose-up attitude has X pointing up = -Z_NED. A pitch of +90° (vertical) maps body-X to -Z_NED.
   - Build using `eul2quat([0, pitch_rad, 0], 'ZYX')` if you have the Robotics/Aerospace Toolbox, or hand-build using the half-angle formula. Output convention is scalar-first `[w, x, y, z]`. Verify with a known check: pitch=0 → quat=[1,0,0,0]; pitch=π/2 → quat=[cos(π/4), 0, sin(π/4), 0].
6. Compute body angular rates: `omega_body_std = d(quat)/dt` numerically differentiated, then converted to body-frame angular velocity vector. For purely-vertical pitch, only `omega_y` (pitch rate, in body frame after the rotation) is nonzero.
7. Compute atmospheric properties using Aerospace Blockset's `COESA Atmosphere Model` block — OR a direct ISA computation per altitude — to fill `air_density_kgm3`, `air_temp_K`, `air_pressure_pa`. Cache these as part of the truth struct so the baro sensor model (T04) doesn't have to recompute.
8. Return a struct matching the `TruthBus` definition in `SIMULINK_PATTERNS.md` §7.

### 6.3 `casper_truth_build_bus.m`

**Signature**: `bus_def = casper_truth_build_bus()`

Returns a `Simulink.Bus` object definition (cell array compatible with `Simulink.Bus.cellToObject`) for the TruthBus layout per `SIMULINK_PATTERNS.md` §7. Called from `casper_sim_config.m` at top-level setup time.

### 6.4 `build_truth_pipeline_block.m`

Programmatically construct a Simulink subsystem `truth_source` that:
- Reads `truth_trajectory.mat` via a `From File` block (or pre-loads via `truth_trajectory` workspace variable consumed by a `From Workspace` block).
- Outputs a `TruthBus`-typed signal at solver rate (10 kHz).

Save the subsystem as a library block in `casper_sim_lib.slx` (create the library if it doesn't exist; do not overwrite other blocks if it does).

### 6.5 `test_truth_pipeline.m`

Full sanity test. Must:
1. Call `casper_rasaero_ingest` on the input CSV.
2. Assert: raw struct has ≥ 12000 samples, max `alt_m` is between 30000 and 32000 (RasAero apogee ~31 km), max `vel_v_mps` is between 800 and 900 (peak ~850 m/s).
3. Call `casper_truth_resample` with `dt=1e-4`, `t_max=549`.
4. Assert: resampled struct has 5,490,001 samples (549/1e-4 + 1).
5. Assert: peak altitude in resampled matches raw within 0.5 m (interpolation should not change peaks meaningfully).
6. Generate plot `plots/raw_vs_resampled.png`: 3×1 subplot of alt/vel/accel-V showing raw circles + resampled line, full trajectory.
7. Generate plot `plots/truth_full_trajectory.png`: 5×1 subplot of alt/vel/accel-V/Mach/pitch over the full 549 s.
8. Save the resampled truth to `truth_trajectory.mat`.
9. Build the Simulink truth source block.
10. Smoke-test the block: run a 0.1 s simulation pulling truth, log the output, assert the first sample equals the raw CSV's t=0 values.

## 7. The sign of `accel_v_mps2` — read carefully

RasAero's `Accel-V` column is the **net vertical acceleration of the vehicle** (i.e., `d²(altitude)/dt²` in the world frame). It is NOT the accelerometer reading.

An accelerometer reads **specific force** = `(net acceleration in inertial) − gravity`. So if the vehicle is stationary, its net world-frame accel is 0, but the accelerometer reads `+g` on the upward axis (gravity reaction).

For Phase 0:
- `accel_NED` field in the truth bus is the **world-frame net acceleration**. For a stationary vehicle, this is `[0,0,0]`. During boost, this is `[0, 0, -100]` m/s² (negative Z = upward at 10g).
- The IMU sensor model (T03) is responsible for converting this to specific force in body frame: it computes `a_specific_body = R_body_from_NED * (a_NED - g_NED)`, where `g_NED = [0, 0, +9.80665]`. So a stationary vehicle gets `a_specific = R * (-g_NED)` = body-up gravity reaction. T03 will use truth attitude for `R_body_from_NED`.

Do not bake gravity into the `accel_NED` field. It is gravity-free (world-frame net accel only). T03 adds gravity reaction during sensor synthesis.

## 8. Acceptance criteria

The task is complete when **all** of the following pass:

- [ ] `casper_rasaero_ingest.m` reads `Flight_Test.CSV` without error.
- [ ] Raw struct has 12,800 ± 10 rows.
- [ ] Max altitude ∈ [30000, 32000] m.
- [ ] Max vertical velocity ∈ [800, 900] m/s.
- [ ] Resampled truth has 5,490,001 samples for `dt=1e-4, t_max=549`.
- [ ] Resampled peak altitude matches raw peak altitude within 0.5 m.
- [ ] Resampled peak velocity matches raw peak velocity within 0.5 m/s.
- [ ] Quaternion at t=0 equals `[1, 0, 0, 0]` (no rotation, since pad pitch RasAero reports may be 89°; verify the convention).
- [ ] `truth_trajectory.mat` exists and is < 200 MB.
- [ ] `build_truth_pipeline_block.m` runs without error and produces a library block.
- [ ] Library block, smoke-tested in a 0.1 s sim, emits expected initial values.
- [ ] Both plots exist and are visually sensible (apogee peak visible, no NaN spans).
- [ ] `STATUS.md` reports PASS with all checks.

## 9. Anti-goals — do NOT do any of these

- Do not interpolate the stage string with `'pchip'` (it's not numeric). Use `'previous'`.
- Do not assume CSV columns are in the order documented above — read by **column name** (`readtable` gives you column variable names; use them defensively).
- Do not silently drop rows. If the CSV has malformed lines, error out and report.
- Do not skip the unit conversion. RasAero is fully imperial.
- Do not "smooth" the truth trajectory beyond the resampling. Any low-pass filter introduces phase shift the EKF will see as systematic error.
- Do not bake gravity into `accel_NED` (see §7).
- Do not use `interp1` with `'linear'` on `accel_v_mps2` — the boost-to-coast transition is a discontinuity in accel that PCHIP handles better. Linear introduces phase smear that becomes a velocity bias.

## 10. Hand-off notes for downstream tasks

T03, T04, T05, T06 will consume `truth_trajectory.mat` directly (loaded into base workspace as `truth`). They expect every field listed in `SIMULINK_PATTERNS.md` §7. If you change the field names or types, downstream breaks.

T07 (frame switch) does **not** read truth; it operates on signals at runtime.

T10 (validation) loads `truth_trajectory.mat` for ground-truth comparison metrics.

## 11. Source firmware references

None (this task is sim-side only). RasAero is external to the firmware repo.

## 12. References

- `ARCHITECTURE.md` §3 (frame conventions), §4 (sample rates), §6 (determinism).
- `references/SIMULINK_PATTERNS.md` §7 (TruthBus definition), §2 (build pattern).
- `references/FIRMWARE_CONSTANTS.md` §9 (frame conventions consolidated).
- MATLAB docs: `readtable`, `interp1`, `Simulink.Bus`, `add_block`, `save_system`.
- RasAero documentation: external, not in repo. Column meanings inferred from header.
