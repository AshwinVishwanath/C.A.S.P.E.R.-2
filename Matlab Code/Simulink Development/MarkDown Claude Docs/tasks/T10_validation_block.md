# T10 — Validation Block & Trust-Gate Metrics

## 1. Goal

Build the validation block that compares estimator output against truth, computes the metrics defined in `PHASE0_SPEC.md` §3, and emits a structured pass/fail report plus diagnostic plots.

This is the block that says "Phase 0 PASS" or "Phase 0 FAIL". Get this right and the team has objective verdict on whether the sim is trustworthy.

## 2. Inputs

| Input | Source |
|---|---|
| Truth bus (sim frame) | T01 |
| Frame-switched truth (firmware frame) | T07 inverse-applied to truth |
| EKF state history | T08 output |
| Attitude estimate | T09 output |
| Mach gate state | T08 output |
| Sensor outputs (for snapshot plots) | T03, T04, T05, T06 |
| Run metadata | from `casper_sim_config.m` (seed, MATLAB version, timestamp) |

## 3. Outputs

In `Software/Sim/build/T10_validation_block/`:

| File | Purpose |
|---|---|
| `casper_metric_apogee.m` | Apogee error computation |
| `casper_metric_velocity.m` | Velocity error (RMS over phases) |
| `casper_metric_attitude.m` | Tilt RMS error |
| `casper_metric_mach_gate.m` | Mach gate timing checks |
| `casper_metric_determinism.m` | Two-run byte-compare |
| `casper_compute_all_metrics.m` | Aggregator |
| `casper_generate_plots.m` | Plot bundle generator |
| `casper_generate_report.m` | Writes `PHASE0_TRUSTGATE_REPORT.md` |
| `build_validation_block.m` | Simulink build script (logging-only subsystem) |
| `test_validation_block.m` | Unit test on synthetic data |
| `validation_block.slx` | Subsystem that logs all signals for downstream metric computation |
| `STATUS.md` | Acceptance status |

## 4. Metric specifications

Each metric function takes `Truth` and `Estimate` (both as structs with time-aligned arrays) and returns a `MetricResult` struct:

```
MetricResult:
  name        : string
  value       : scalar (or struct for multi-component metrics)
  threshold   : scalar (or struct)
  pass        : bool
  details     : string (one-line description of what was measured)
```

### 4.1 Apogee (`casper_metric_apogee.m`)

```
[truth_apogee, truth_idx] = max(Truth.alt_agl_m)
truth_apogee_time = Truth.time_s(truth_idx)

[est_apogee, est_idx] = max(Estimate.state_x(:,1))
est_apogee_time = Estimate.time_s(est_idx)

apogee_alt_err = abs(est_apogee - truth_apogee)
apogee_time_err = abs(est_apogee_time - truth_apogee_time)

pass_alt = apogee_alt_err <= 10                              % PHASE0_SPEC §3.1
pass_time = apogee_time_err <= 0.5
```

### 4.2 Velocity error (`casper_metric_velocity.m`)

Compute RMS in three windows:
- Powered flight: `t in [0, burnout_time]`
- Coast: `t in [burnout_time, apogee_time]`
- At burnout (single-point error): `est_vel - truth_vel` at the burnout sample

Burnout detection: first time `Truth.accel_NED(:,3)` ≥ `-G_ACCEL * 0.99` after t=2 (free fall begins).

```
v_truth_at_burnout = Truth.vel_v_mps(burnout_idx)            % from truth, +up
v_est_at_burnout   = Estimate.state_x(burnout_idx, 2)       % EKF velocity state

rms_powered = sqrt(mean( (Estimate.state_x(0..burnout, 2) - Truth.vel_v_mps(0..burnout)).^2 ))
rms_coast   = sqrt(mean( (Estimate.state_x(burnout..apogee, 2) - Truth.vel_v_mps(burnout..apogee)).^2 ))

pass = (rms_powered <= 5) && (abs(v_est_at_burnout - v_truth_at_burnout) <= 2) && (rms_coast <= 3)
```

### 4.3 Tilt RMS error (`casper_metric_attitude.m`)

For each time sample:
```
q_err = quat_mult(quat_conj(Truth.quat_fw(k,:)), Estimate.quat_fw(k,:))
% Ensure q_err.w >= 0
if q_err(1) < 0
    q_err = -q_err
end
tilt_angle_deg(k) = 2 * acosd(q_err(1))                     % full rotation error angle, degrees
```

Then:
```
rms_powered = sqrt(mean(tilt_angle_deg(0..burnout).^2))
rms_coast   = sqrt(mean(tilt_angle_deg(burnout..apogee).^2))

pass = (rms_powered <= 1) && (rms_coast <= 2)
```

### 4.4 Mach gate timing (`casper_metric_mach_gate.m`)

```
mach_truth = Truth.mach
gate_truth_engage_time = first time mach_truth crosses 0.40 upward
gate_truth_release_time = first time mach_truth crosses 0.35 downward

gate_est_engage_time = first time Estimate.mach_gate_active goes high
gate_est_release_time = first time Estimate.mach_gate_active goes low after engagement

engage_delay = gate_est_engage_time - gate_truth_engage_time     % positive: late
release_delay = gate_est_release_time - gate_truth_release_time

pass = (abs(engage_delay) <= 0.25) && (abs(release_delay) <= 0.5)
```

Also verify:
- No baro update fires while `mach_gate_active == true` (count baro innovations in that window; must be zero).
- Ungate inflation lasts exactly 10 baro updates after release (count baro updates where `R` reverted to nominal `R_BARO`; first 10 should still be `R_BARO_UNGATE = 50`).

### 4.5 Bias-state envelope (`casper_metric_bias.m`)

```
final_accel_bias = Estimate.state_x(end, 3)
final_baro_bias  = Estimate.state_x(end, 4)

accel_bias_3sigma = 3 * sqrt(P0_ACCEL_BIAS)                  = 0.474 m/s²
baro_bias_3sigma  = 3 * sqrt(P0_BARO_BIAS)                   = 2.598 m

pass = (abs(final_accel_bias) <= accel_bias_3sigma) && (abs(final_baro_bias) <= baro_bias_3sigma)
```

Also verify P-floor on baro bias: `P_history(:, 4, 4)` must all be `>= P_FLOOR_BARO_BIAS = 0.01`.

### 4.6 Determinism (`casper_metric_determinism.m`)

Run twice with identical config. Compute hashes:
```
hash1 = DataHash(sensor_streams_run1)
hash2 = DataHash(sensor_streams_run2)
pass = isequal(hash1, hash2)

hash3 = DataHash(estimate_run1)
hash4 = DataHash(estimate_run2)
pass = pass && isequal(hash3, hash4)
```

Use `DataHash` from File Exchange or implement a wrapper around `MD5` of `getByteStreamFromArray`.

### 4.7 Sanity (`casper_metric_sanity.m`)

Per `PHASE0_SPEC.md` §3.7:
- No NaN/Inf in any signal: scan all logged signals.
- Sim runtime < 5 min: measured wall clock.
- Sensor PSDs match spec to ±3 dB: compute PSD on pad-stationary segment for each sensor, compare to expected white-floor.

## 5. Aggregator (`casper_compute_all_metrics.m`)

Calls all individual metric functions, returns a `MetricBundle`:

```
MetricBundle:
  apogee        : MetricResult
  velocity      : MetricResult
  attitude      : MetricResult
  mach_gate     : MetricResult
  bias          : MetricResult
  determinism   : MetricResult
  sanity        : MetricResult
  overall_pass  : bool (AND of all)
```

## 6. Plot bundle (`casper_generate_plots.m`)

Generates PNG plots in `Software/Sim/build/T11_integration/plots/` (notably *T11's* plots dir — these are user-facing artifacts):

### 6.1 Required plots

1. `altitude_truth_vs_est.png`: full trajectory, two lines, with apogee markers.
2. `velocity_truth_vs_est.png`: full trajectory, two lines, with burnout markers.
3. `attitude_error_euler.png`: roll/pitch/yaw error vs time, all three on one figure.
4. `tilt_angle_error.png`: total tilt error in degrees, with 1° and 2° threshold bands shaded.
5. `mach_gate_state.png`: Mach number trace with gate-active boolean overlaid.
6. `eskf_innovations.png`: baro and ZUPT innovations with `±5σ` gate boundaries shaded; rejected updates marked.
7. `bias_states.png`: accel_bias and baro_bias over time with `±3σ` envelopes.
8. `covariance_diag.png`: log-scale plot of all 4 P diagonals over time.
9. `sensor_snapshot_pad.png`: 1-second window of all sensor outputs while on pad.
10. `sensor_snapshot_peak_mach.png`: 1-second window during peak Mach (~M=2.5 at t≈10 s).
11. `sensor_snapshot_drogue.png`: 1-second window during drogue descent (~t=70 s).
12. `mag_radio_interference_zoom.png`: 200 ms zoom showing 1-2 mag spikes during radio TX.

Style requirements:
- All plots have axes labels with units.
- Time axis is in seconds (not samples).
- Truth lines are dashed; estimate lines are solid.
- Color blind-friendly palette: blue (estimate), orange (truth), green (gate-active), red (failure).
- Threshold bands are shaded with 20% opacity.
- Title contains test ID and seed, e.g., "Apogee T11_trust_gate seed=20260519".
- All plots saved at 300 DPI, 8" × 5" default.

## 7. Report generation (`casper_generate_report.m`)

Writes `PHASE0_TRUSTGATE_REPORT.md` in `Software/Sim/build/T11_integration/`. Schema:

```markdown
# Phase 0 Trust Gate Report

**Verdict**: PASS | FAIL

**Run timestamp**: <ISO 8601>
**MATLAB version**: <ver>
**Simulink version**: <ver>
**Seed**: <Sim.Seed>
**Git commit**: <git rev-parse HEAD>

## Headline metrics

| Metric | Threshold | Actual | Status |
|---|---|---|---|
| Apogee altitude error | ≤ 10 m | X.X m | PASS/FAIL |
| Apogee time error | ≤ 0.5 s | X.X s | PASS/FAIL |
| Velocity at burnout | ≤ 2 m/s | X.X m/s | PASS/FAIL |
| Tilt RMS (powered) | ≤ 1° | X.X° | PASS/FAIL |
| Tilt RMS (coast) | ≤ 2° | X.X° | PASS/FAIL |
| Mach gate engage delay | ≤ 0.25 s | X.X s | PASS/FAIL |
| Mach gate release delay | ≤ 0.5 s | X.X s | PASS/FAIL |
| Determinism (two runs) | identical | <hash> | PASS/FAIL |

## Detailed metrics
(per-phase breakdowns, all individual MetricResult fields)

## Sanity checks
(NaN/Inf scan, runtime, PSD checks)

## Plots
(thumbnails/links to all PNG plots)

## Failures
(if any: failing metric, value vs threshold, suspected cause from PHASE0_SPEC §4 table)

## Deviations
(any unavoidable deviations from spec, justified)

## Discord update template
```
__phase 0 trust gate__: <PASS/FAIL>

apogee error: X.X m
burnout vel error: X.X m/s
tilt RMS, powered: X.XX°
mach gate behaved correctly: <yes/no>
determinism check: <PASS/FAIL>

<next step>
```
```

Use markdown tables and consistent units. The report must be copy-pasteable into a GitHub Issue or Discord thread.

## 8. Validation block in Simulink (`validation_block.slx`)

A subsystem inserted in the integration model that logs all the signals downstream metric computation needs. It does not compute metrics in real-time; it just logs to the workspace for offline computation in `casper_compute_all_metrics.m`.

Logged signals:
- `Truth.*` (full bus)
- `Estimate.state_x` (4-tuple, 416 Hz)
- `Estimate.state_P_diag` (4-tuple, 416 Hz)
- `Estimate.attitude_quat` (4-tuple, 833 Hz)
- `Estimate.mach_gate_active` (bool, 416 Hz)
- `Estimate.ungate_counter` (int, 416 Hz)
- `Estimate.baro_innov` (with timestamp, fires at baro rate)
- `Estimate.zupt_innov`
- All sensor outputs (IMU, ADXL, baro, mag, GPS) — captured for snapshot plots
- `RadioTX.active` (bool, for mag-spike visualization)

Use Simulink `To Workspace` blocks with `SaveFormat = 'StructureWithTime'` and unique workspace variable names per signal.

## 9. Acceptance criteria

1. **All metric functions implemented**: every metric in §4 is implemented and unit-tested on synthetic perfect-truth data (estimate == truth should return PASS for everything).
2. **Aggregator works**: `casper_compute_all_metrics` returns a complete `MetricBundle`.
3. **All plots generated**: 12 PNGs in the plots directory.
4. **Report file format**: `PHASE0_TRUSTGATE_REPORT.md` is valid markdown, opens in any viewer.
5. **Discord template populated**: report ends with a copy-paste-ready Discord block.
6. **Verdict correctness**:
   - On synthetic perfect-truth input → verdict = PASS, all metrics PASS.
   - On synthetic +20 m altitude offset → verdict = FAIL with apogee criterion flagged.
   - On synthetic +3 m/s burnout velocity error → verdict = FAIL with velocity criterion flagged.
   - On non-deterministic two-run input (manually inject) → verdict = FAIL with determinism flagged.
7. **Performance**: all metrics + all plots + report generation under 60 s wall-clock.
8. **Plot quality**: every plot is 300 DPI minimum, has labels with units, has a legend, has a title.
9. **No NaN/Inf**: the metrics handle missing data gracefully (skip with a NOTE in the report).

## 10. Anti-goals

- Do NOT change thresholds. They are in `PHASE0_SPEC.md` §3.
- Do NOT silently skip a metric if data is missing — flag it.
- Do NOT compute metrics in real-time inside the Simulink model. Log and post-process.
- Do NOT include sensor noise spec plots that aren't part of the gate (Phase 1 / advanced diagnostics live elsewhere).
- Do NOT include speed-of-sound or aero-related diagnostics in this block (Phase 1+).

## 11. Hand-off

T11 wires this validation block in and calls `casper_compute_all_metrics` + `casper_generate_plots` + `casper_generate_report` at the end of every sim run.

## 12. References

- `PHASE0_SPEC.md` §3 (the trust gate)
- `PHASE0_SPEC.md` §4 (failure-mode table)
- `PHASE0_SPEC.md` §7 (Discord template)
- `FIRMWARE_CONSTANTS.md` §1 (Estimator constants for thresholds)

## 13. STATUS.md template

```
# T10 Validation Block — STATUS

## Files created
- casper_metric_*.m × 6
- casper_compute_all_metrics.m
- casper_generate_plots.m
- casper_generate_report.m
- build_validation_block.m
- test_validation_block.m
- validation_block.slx

## Acceptance criteria
| # | Criterion | Status | Notes |
|---|---|---|---|
| 1 | All metric fns | PASS | 7/7 implemented |
| 2 | Aggregator | PASS | |
| 3 | Plots | PASS | 12/12 PNGs |
| 4 | Report format | PASS | valid markdown |
| 5 | Discord template | PASS | |
| 6 | Verdict on synthetic | PASS | 4/4 test cases correct |
| ... | | | |

## Deviations from spec
- (any)
```
