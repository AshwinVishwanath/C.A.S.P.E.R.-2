# PHASE0_SPEC.md — Phase 0 Mission & Acceptance Criteria

## 1. Mission Statement

Build a Simulink-native simulator that takes the RasAero CSV truth trajectory `Flight_Test.CSV`, generates simulated sensor outputs from the C.A.S.P.E.R.-2 sensor suite at the firmware's exact sample rates, runs them through a stripped MATLAB port of the firmware's ESKF + attitude estimator, and verifies the estimator recovers the truth trajectory within tight tolerances.

Phase 0 produces a **trust gate**: if the estimator-fed-by-sim recovers known truth, the sim is trustworthy for further work. If not, either the sensor models are wrong (likely) or the estimator port is wrong (less likely), and Phase 0 is incomplete until that's resolved.

## 2. Deliverables

Eleven task outputs combine into the Phase 0 deliverable bundle. Each task is owned by one sub-agent. The integration task (T11) wires them all together into a single runnable Simulink model.

| Task ID | Owner | Deliverable |
|---|---|---|
| T01 | Sonnet sub-agent A | Truth trajectory pipeline (`Flight_Test.CSV` → uniform-grid struct) |
| T02 | Sonnet sub-agent B | Parameter library mining firmware constants into MATLAB |
| T03 | Sonnet sub-agent C | LSM6DSO32 + ADXL372 sensor models |
| T04 | Sonnet sub-agent D | MS5611 baro sensor model |
| T05 | Sonnet sub-agent E | MMC5983MA mag model + radio-TX interference layer |
| T06 | Sonnet sub-agent F | MAX-M10M GPS sensor model |
| T07 | Sonnet sub-agent G | Sim↔firmware frame switch (HIL boundary) |
| T08 | Sonnet sub-agent H | Stripped ESKF MATLAB port |
| T09 | Sonnet sub-agent I | Stripped attitude estimator MATLAB port |
| T10 | Sonnet sub-agent J | Validation block + metrics dashboard |
| T11 | Sonnet sub-agent K (or Opus directly) | Top-level Simulink wiring + trust-gate execution |

The final artifact is `Software/Sim/build/T11_integration/casper_sim_phase0.slx` plus its companion run script `run_phase0_trustgate.m` and the auto-generated `PHASE0_TRUSTGATE_REPORT.md`.

## 3. Trust Gate — Acceptance Criteria

A Phase 0 build is **accepted** if and only if all of the following pass when running `Flight_Test.CSV` end-to-end through the assembled simulator with the default seed (`Sim.Seed = 20260519`):

### 3.1 Trajectory recovery
- **Apogee altitude error** ≤ 10 m, peak-vs-peak (compare max EKF-recovered altitude to max RasAero altitude).
- **Apogee time error** ≤ 0.5 s (compare time-of-max-altitude).
- **Burnout velocity error** ≤ 2 m/s RMS over the 100 ms window centered on burnout (defined as the time RasAero `Accel-V` first goes negative).
- **Attitude RMS error** ≤ 1° (3-axis) during powered flight (RasAero stage = "B"), where attitude error is the angle of `q_truth^-1 ⊗ q_estimate`.

### 3.2 Reproducibility
- Two runs with identical config produce **binary-identical** sensor stream `.mat` files.
- Two runs with identical config produce **bitwise-identical** estimator state outputs (to within `1e-12` absolute tolerance on doubles; the only allowed deviation comes from non-IEEE-deterministic operations like Simulink's `arm_mat_mult` analog, which there shouldn't be any of in MATLAB).

### 3.3 Sanity (always-on assertions)
- Stationary-on-pad simulated IMU reads `[0, +9.80665, 0]` body-frame accel (firmware convention) and `[0,0,0]` body-frame gyro at t=0.
- Stationary-on-pad simulated baro at t=0, after referencing, reads altitude within ±0.7 m (1σ ≈ √R_BARO).
- Stationary-on-pad simulated mag, with no radio TX active, reads magnitude `40.18 ± 1 µT` (matching `MAG_CAL_EXPECTED_MAG`).
- Radio TX events corrupt 1–2 mag samples each, peak deviation `±10 µT` (within ±15% of placeholder spec).
- ZUPT-driven velocity at t=0 converges to `|v| < 0.01 m/s` within 1 s.
- Mach gate fires at t ≈ 2.5 s (Mach 0.4 first crossing per RasAero) and stays active until t ≈ 60 s.

### 3.4 Outputs delivered
- `casper_sim_phase0.slx` exists, opens in Simulink, simulates without errors.
- `run_phase0_trustgate.m` executes start-to-finish from a fresh MATLAB session within 10 minutes.
- `PHASE0_TRUSTGATE_REPORT.md` is generated and contains all metrics above, plus a top-line PASS/FAIL.
- Plot bundle in `Software/Sim/build/T11_integration/plots/` containing at minimum:
  - Truth vs estimated altitude, velocity (full trajectory).
  - Attitude error (Euler degrees) over time.
  - EKF innovation residuals (baro, ZUPT) with gate boundaries.
  - Mach gate state vs Mach number.
  - Sensor stream snapshots: 1 s of IMU on pad, 1 s during peak-Mach, 1 s during drogue descent.

A build that fails any 3.1 or 3.3 criterion is **not accepted**. Diagnose, fix, re-run. The manager owns the diagnose/fix loop.

## 4. Definition of "fail" — what to do

| Failure mode | First place to look | Second place |
|---|---|---|
| Apogee off by >10 m, attitude OK | Baro model (Mach-shock or noise) or EKF Mach-gate logic | EKF ungate recovery (P inflation) |
| Apogee off by >100 m, attitude OK | Sign error in `a_up` (Z-up vs NED), or frame switch bug | Truth trajectory unit conversion (ft vs m) |
| Attitude RMS > 1°, altitude OK | Body-frame permutation in T07 | Mag interference too aggressive, or gyro bias init |
| Both apogee and attitude diverge | Frame switch is fundamentally broken | Truth pipeline interpolation |
| Burnout velocity error > 5 m/s | Truth accel pipeline (RasAero unit conversion, Accel-V sign) | EKF predict rate (must be exactly 416 Hz) |
| ZUPT divergence at t=0 | ZUPT threshold check wrong | Initial covariance P0 |
| Mag magnitude wrong by >5 µT | Hard/soft iron applied in wrong order | Sign flip ×-1 missed |
| Mag spike amplitude wildly off | Radio TX cadence/timing | Sample rate of injection point |

The manager owns the diagnostic loop. Each failure mode in this table should produce a specific re-dispatch to a sub-agent with the relevant task's diagnostic instructions.

## 5. Test plan (executed by T11)

### Test 1: stationary-on-pad (5 s)
- Override `Flight_Test.CSV` with constant-state input at `t=0` row.
- Expect: stationary IMU, baro converging to 0, mag stable except for TX spikes, ZUPT firing, EKF velocity → 0.
- Pass: §3.3 sanity checks.

### Test 2: full RasAero trajectory (549 s)
- Run end-to-end on the actual `Flight_Test.CSV`.
- Expect: §3.1 trajectory recovery and §3.3 sanity (modulated to in-flight values).
- Pass: §3.1 trajectory metrics.

### Test 3: reproducibility
- Run Test 2 twice with identical config.
- Diff sensor stream `.mat` files byte-by-byte.
- Diff estimator state outputs to `1e-12` tolerance.
- Pass: §3.2 reproducibility.

### Test 4: seed sensitivity (informational, not pass/fail)
- Run Test 2 with seeds `20260519`, `20260520`, `20260521`.
- Report apogee error spread across seeds.
- Use to characterize sensor noise contribution. (Documented in report; not gating.)

## 6. Out-of-scope for Phase 0

Listed in `ARCHITECTURE.md` §13. Do not implement these. If a sub-agent finds themselves needing one of these to pass their task, halt and escalate — the task spec is wrong.

## 7. Schedule expectations

This is an agent-driven build; manager should target completion in ≤ 50 sub-agent turn-counts total across all tasks. Indicative breakdown:

| Task | Expected complexity (turns) |
|---|---|
| T01 truth pipeline | 4 |
| T02 sensor params | 3 |
| T03 IMU model | 5 |
| T04 baro model | 6 (Mach-shock is the long pole) |
| T05 mag model | 6 (interference modeling) |
| T06 GPS model | 3 |
| T07 frame switch | 4 |
| T08 ESKF port | 6 |
| T09 attitude port | 6 |
| T10 validation | 4 |
| T11 integration | 8 (debugging loop) |

If the manager finds itself spending >10 turns on a single sub-agent task, halt and escalate.

## 8. Reporting

The manager produces one file at the end of Phase 0 attempt:

`Software/Sim/build/T11_integration/PHASE0_TRUSTGATE_REPORT.md`

Schema (mandatory headings):

```
# Phase 0 Trust Gate Report

## Verdict
PASS | FAIL | PARTIAL

## Summary metrics
- Apogee error: X.X m
- Apogee time error: X.X s
- Burnout velocity RMS error: X.X m/s
- Attitude RMS error: X.X deg
- Reproducibility check: PASS | FAIL

## Sanity checks (3.3)
[checklist of all sanity checks with pass/fail]

## Failures (if any)
[per-failure: failure mode, root cause if known, what was tried, remaining work]

## Deviations from spec
[any unavoidable deviations from this PHASE0_SPEC.md, with justification]

## Open items for Phase 1
[carry-forward issues, especially radio→mag calibration, real flight log replay]

## Run metadata
- MATLAB version
- Simulink version
- Toolbox versions
- Total runtime
- Seed used
- Git commit of firmware (for traceability)
```

If the verdict is anything other than PASS, the manager must enumerate every failure with a proposed next step before terminating.
