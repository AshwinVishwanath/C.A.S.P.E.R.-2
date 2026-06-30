# CASPER-2 Mahony Attitude Filter Hardening — MATLAB/Simulink Implementation Spec

**Target branch:** `Simulink-Auto-Build-2026-05-20`
**Authoritative spec references:** `EKF_SPEC.md`, `ORIENTATION_SPEC.md`, `SENSOR_SPEC.md`
**Validation reference:** Python ESKF dev tree in `/home/claude/eskf/` (separate handoff)

---

## 1. Why this exists

Python validation against synthetic IMU/baro/mag streams (Allan-derived noise from MATLAB analysis already in `EKF_SPEC.md` §2.3 and §3.3) revealed three independent failure modes in the current Mahony complementary filter. Combined, they inflate apogee altitude error from <1 m to >120 m. The 4-state EKF structure is not the limiter — the attitude estimator is. The fix is in attitude only; the EKF stays as it is.

This spec covers MATLAB/Simulink implementation of the three fixes plus scaffolding for the v2 kinematic-compensation upgrade. No EKF changes. No ESKF replacement. That is a separate workstream.

## 2. Findings from Python validation (the bar to clear)

Synthetic parametric flight, 60 s pad dwell, peak Mach 1.2, 28 s Mach-gated dead-reckoning window. Same sensor stream into every variant.

| Config | Alt RMSE | Apogee err | Tilt at launch |
|---|---|---|---|
| Current Mahony equivalent + ZUPT | 50.23 m | +123.4 m | 4.54° |
| + magnitude-gated accel only | 10.00 m | -12.5 m | 3.61° |
| + 60 s pad calibration | 1.18 m | +0.15 m | 0.04° |
| + Ki disabled (Mahony integral term) | **0.14 m** | **+0.03 m** | **0.03°** |
| Reference: 15-state ESKF on same data | 0.35 m | +0.03 m | (3D, not directly comparable) |

Conclusion: a properly-tuned Mahony + 4-state matches or beats a full ESKF on altitude accuracy for the synthetic test case. Real-flight validation is the next gate. Caveat: synthetic mag noise was `σ=50 µT, τ=160 ms` (placeholder). Real noise must come from the Simulink mag block in this branch.

## 3. Implementation deliverables

1. Mahony filter in the Simulink model with three configurable parameters defaulted to the validated values
2. Magnitude gate on the gravity correction with soft cosine window
3. Extended pad-phase calibration window with adjustable duration
4. Validation harness comparing Simulink output against the Python reference on the same synthetic stream
5. Feature-flagged scaffold for Approach B (kinematic compensation using EKF velocity feedforward) — not enabled by default

## 4. Agent team structure

### Roles

**Manager (Opus, single instance).** Owns plan, sequencing, reviews, branch hygiene, escalation handling. Reads this spec, decomposes work into level-bounded tasks, dispatches workers via the Task tool, reviews each level's output before authorising the next level. Does not write code directly except during escalation takeovers.

**Workers (Sonnet, ≤3 concurrent within a level).** Implement bounded tasks per the level definitions below. Each worker operates inside one level only and must not touch files or modules outside its level's scope. Workers commit their changes to a level-specific subbranch.

### Concurrency rules

- Workers within the same level may run in parallel only on disjoint files / model regions
- Levels execute strictly serially: L1 fully complete and manager-approved before L2 starts
- Each worker reads this spec + relevant authoritative `*_SPEC.md` files at task start

### Escalation protocol

A Sonnet worker is considered stuck when **any** of the following holds:
1. 20 minutes of wall-clock elapsed since task acceptance with no committed progress (no edits, no validated output)
2. Three or more failed approaches to the same problem
3. Encountered Simulink model element outside the stated scope of its task
4. Numerical output diverges from the Python reference by >2× the stated tolerance and cannot identify the cause within two diagnostic passes
5. Toolchain / compilation error not resolved within three attempts

**On escalation trigger**, the worker writes its current state to a `STUCK.md` note in its working directory (what it tried, why each attempt failed, what it suspects, what would help) and hands the task back to the Manager. The Manager reads the note, then either:
- **Takes over directly (Opus execution)** — if the blocker is architectural, requires global context, or needs judgement the worker lacked
- **Splits the task further** — if the worker was scope-overloaded
- **Reassigns to a different worker** — if it's a tooling/environment issue specific to the worker's session
- **Pauses and asks Ashwin** — if the blocker requires information not in any spec or repo

The Manager never lets a stuck worker grind. 20 minutes is the hard cap.

### Communication contract

- Workers post a single end-of-task summary (3–5 lines: what changed, where, how validated, any concerns flagged)
- Manager posts a level-completion summary (delta vs spec, deviations and why, list of open follow-ups)
- No chat-style back-and-forth between workers and Manager during a task — workers either finish or escalate

## 5. Levels

### Level 0 — Discovery (Manager-only, no workers)

Goal: orient on the existing Simulink model and locate the artefacts that subsequent levels will modify. No edits.

The Manager produces a `L0_DISCOVERY.md` containing:
- File path of the Simulink model containing the Mahony filter
- Subsystem / block path to the Mahony filter
- Current parameter values for `Kp_grav`, `Kp_mag_pad`, `Kp_mag_flight`, `Ki`, gyro bias estimation method, pad/flight transition logic
- File path and exact parameters of the magnetometer noise block (white σ per axis, any colored-noise model, hard/soft iron)
- File path of the baseline 4-state EKF subsystem
- Any existing test harness or run script

Definition of done: `L0_DISCOVERY.md` answers each bullet above with file path + line/block reference + parameter value. If a parameter is missing from the model, flag it explicitly as `NOT FOUND` rather than guessing. No L1 work starts until this is committed.

### Level 1 — Test harness and data pipeline

Goal: stand up a reproducible validation environment so that subsequent levels can be measured, not assumed. No filter changes in this level.

Tasks (parallel workers permitted, disjoint file scopes):

**T1.1 — Synthetic stream importer.** Build a MATLAB function `load_python_stream(csv_path)` that reads the CSV exports from the Python `/home/claude/eskf/` reference and returns a struct with `t, gyro(N,3), accel(N,3), mag(N,3), baro_alt(N), baro_valid(N), mag_valid(N), truth_q(N,4), truth_pos(N,3), truth_vel(N,3)`. Units: SI, Z-up, body-frame for sensors, nav-frame for truth. The Python side will emit a CSV per stream alongside a `manifest.json` describing column names and units. Tolerance: round-trip read should preserve values to within 1e-9 relative error.

**T1.2 — Reference flight log generator.** Python-side: write a script in the Python dev tree that runs `parametric_trajectory + synthesize` with `seed=42, fs=833, total_time=100, pad_dwell=62`, then exports the streams + truth to CSV+manifest under `validation/ref_flight_seed42/`. Do not modify the Python filter code; just export. This script lives in the Python repo for reproducibility but its output (the CSV bundle) is what L2/L3 consume.

**T1.3 — Comparison harness.** MATLAB function `compare_run(matlab_run, truth)` returning a struct with `alt_rmse, alt_max, apogee_err, vel_rmse, tilt_err_launch_deg, tilt_err_apogee_deg`. Window definitions: alt RMSE and vel RMSE computed over the boost-to-apogee interval. Tilt error computed as angle between body-X-in-nav of truth and estimate.

**T1.4 — Baseline run.** Run the current unmodified Mahony + 4-state through the harness on the L1.2 reference flight. Record results in `L1_BASELINE.md`. These are the numbers L2 must beat.

Acceptance: all four tasks committed to a `feature/mahony-l1` subbranch. Manager runs T1.4 independently on its own machine if possible; if the numbers don't roughly match the Python "Baseline + ZUPT" row from §2 (alt RMSE in the 30–60 m range), L1 is rejected and the divergence is diagnosed before L2 starts.

### Level 2 — Core Mahony patches

Goal: implement the three validated fixes as parametrised changes to the Simulink Mahony block, with the previous behaviour reachable by reverting parameters.

Tasks (these are serial within the same block; one worker, but Manager may split if needed):

**T2.1 — Magnitude gate on gravity correction.** Compute `|a_meas|` per step. Apply a soft window:

```
w(|a|) = 0.5 * (1 + cos(pi * clip((|a| - g) / window_half_width, -1, 1)))
```

where `g = 9.80665` and `window_half_width` defaults to `0.15 * g` (covers ~[0.85g, 1.15g]). Outside the window, `w = 0`. The Mahony gravity-correction gain becomes `Kp_grav_effective = Kp_grav * w`. This must apply in **both PAD and FLIGHT phases** — the boost-time gate is the whole point. Add `window_half_width` as a tunable parameter.

**T2.2 — Make `Ki` configurable and default to zero.** The current integral term accumulates spurious bias when the cross-product corrections are noisy. Disabling it removes that failure mode; the gyro-bias running-mean accumulator already covers the role Ki was nominally serving. Keep the integrator state in the model (for revert) but multiply the integral feedback by `Ki`, which defaults to 0.

**T2.3 — Make `Kp_mag_pad` and `Kp_mag_flight` configurable with low defaults.** Until the real mag noise floor from Simulink is folded back into the Python validation, default both to `0.0` (mag corrections disabled post-INIT). The INIT-phase yaw initialisation from averaged mag stays — single-sample mag corrections post-INIT do not. Expose `Kp_mag_pad` and `Kp_mag_flight` as separate parameters so they can be raised independently once mag SNR is characterised.

**T2.4 — Extended pad calibration window.** Add a parameter `pad_calib_duration_s` defaulted to `60.0`. The gyro bias estimator should accumulate for this duration before launch detection is armed. This is independent of the existing INIT phase (which is the static-accel + mag-yaw-init phase). Pad calibration runs through the entire INIT + PAD phase up to launch.

**T2.5 — Parameter block.** All four new parameters surface in a single Simulink mask or `.m` config file so they can be tuned without diving into the block diagram. Existing `Kp_grav` parameter stays where it is; new ones live alongside it.

Acceptance: configuration matrix table in `L2_CONFIG.md` showing the default values, the legacy values (for revert), and a one-line rationale for each. Code committed to `feature/mahony-l2`. No validation run yet — that's L3.

### Level 3 — Validation pass

Goal: prove the L2 changes hit the targets from §2 on the reference flight, with quantified tolerances.

**T3.1 — Run matrix.** Execute the model against the L1.2 reference flight under the following configurations:
- A: baseline (legacy values) — reproduces L1.4
- B: magnitude gate only (T2.1 enabled, T2.2/2.3/2.4 at legacy values)
- C: B + Ki=0
- D: C + `Kp_mag_pad=0, Kp_mag_flight=0`
- E (target): D + `pad_calib_duration_s=60`

For each, emit a comparison struct (T1.3 output) and append to `L3_RESULTS.md`. Acceptance per row:

| Config | Tolerance | Target |
|---|---|---|
| A | ±20% vs L1.4 | reproducibility check |
| B | alt_rmse ≤ 15 m | 5× improvement over A |
| C | alt_rmse ≤ 2 m | within 50% of Python config C |
| D | alt_rmse ≤ 2 m | confirms Kp_mag_pad effect or surfaces it as noise-dependent |
| **E** | **alt_rmse ≤ 0.5 m** | **clears 200× delta vs baseline** |

If E does not pass, the worker writes a `L3_DIVERGENCE.md` listing every numerical and structural difference between the MATLAB Simulink path and the Python reference (sample-rate handling, quaternion convention, gyro filtering cutoff, etc.). This is escalation material — do not try to fudge gains to make the number pass.

**T3.2 — Real-data dry run (if Ashwin has supplied a flight CSV by L3 start).** Re-run config E against any RasAero/OpenRocket-driven realistic flight in the repo. Record results — these don't have an acceptance bar yet, just a sanity check that the filter doesn't blow up on a different trajectory shape.

Acceptance: `L3_RESULTS.md` committed with all required entries. PR opened from `feature/mahony-l3` to the branch base.

### Level 4 — Approach B scaffolding (optional, do not enable)

Goal: stand up the kinematic-compensation upgrade as feature-flagged code that future work can switch on. Default OFF.

**T4.1 — Velocity feedforward port.** Wire the 4-state EKF's vertical velocity (and, where 3D becomes available, full nav velocity) into the Mahony block as an input. Numerically differentiate with a configurable low-pass cutoff (default 10 Hz). Surface `a_kin_nav_estimate` as an internal signal.

**T4.2 — Compensated gravity reference.** Add a new computation path:
```
g_body_meas_compensated = R(q)^T * a_kin_nav_estimate - f_body
```
Sign convention must match the existing Mahony correction. Cross-product with predicted `g_body` produces a kinematic-compensated correction error. Apply gain `Kp_grav_kin` (default 0).

**T4.3 — Safety gates.** Implement:
- Velocity-validity gate (disable compensation if estimated `vel_sigma > threshold`, e.g., during deep Mach-gating)
- Residual sanity gate (disable if `|g_body_meas_compensated|` deviates from `g` by more than 30%)
- Burnout transient gate (disable for 100 ms after detected accel discontinuity > 5g/sample)

**T4.4 — Documentation.** A `APPROACH_B_NOTES.md` describing how to enable it, the dependencies on EKF state, and the unresolved coupling-stability risk.

Acceptance: all of T4.1–T4.4 committed but `Kp_grav_kin = 0` by default. PR clearly marked "feature-flagged, do not enable for first flight." Manager reviews and merges only if Ashwin has explicitly authorised L4 to proceed — otherwise it stays as a draft PR.

## 6. Branch and PR conventions

- Manager creates branches: `feature/mahony-l1`, `feature/mahony-l2`, `feature/mahony-l3`, `feature/mahony-l4`
- Workers commit to the level branch they are operating in
- One PR per level, against the base `Simulink-Auto-Build-2026-05-20` branch
- PR title: `Mahony Lx: <one-line summary>` where x is the level
- PR description references this spec, lists tasks completed, and links the level's results doc
- L4 PR stays in draft until explicitly approved

## 7. Definition of done for the whole workstream

1. Default-parameter Simulink model achieves `alt_rmse ≤ 0.5 m` on the L1.2 reference flight (or escalation triggered and resolved)
2. All four parameter knobs (`Ki`, `Kp_mag_pad`, `Kp_mag_flight`, `pad_calib_duration_s`) are exposed and documented
3. Magnitude-gated correction works in both PAD and FLIGHT phases with a configurable window
4. L4 scaffolding exists, is off by default, has a clear enable path documented
5. PRs L1–L3 merged. L4 ready for review pending Ashwin sign-off.

## 8. Out of scope for this workstream

- ESKF15 implementation (separate workstream)
- EKF4 structural changes
- Flight code (C) port of the MATLAB changes — separate workstream once MATLAB is validated
- Real-flight data validation beyond the L3.2 dry run
- Magnetometer noise floor characterisation (separate task; the value is consumed here as an input, not produced here)

## 9. Open items the Manager must flag explicitly if not resolved before L2

- **Magnetometer noise floor.** L2.3 defaults `Kp_mag` parameters to 0 specifically because the Python validation used a placeholder noise level. If the real Simulink mag block has σ ≪ 50 µT, the right `Kp_mag` value may be nonzero. Manager should attempt to extract the value during L0 discovery and adjust the default if the value is known and supports a higher gain.
- **Gyro bias freeze-at-launch.** The Python port freezes gyro bias at launch detection. This may or may not match the current Simulink behaviour. L0 discovery must confirm what the existing model does. If the existing model freezes at launch, keep that behaviour. If it does not, the change is in scope but raise it as a separate item rather than silently introducing the freeze.
- **Mahony body-frame convention vs Z-up nav.** `EKF_SPEC.md` §3.2.1 contains an initial-attitude formula assuming body-Z up, which does not match the CASPER-2 nose-up convention. The L0 discovery must confirm whether the Simulink model uses the spec's formula or a corrected version. If the spec's formula, this is a separate bug that must be raised but is not auto-fixed in this workstream.

## 10. Reading list (in priority order for any new worker)

1. This document, top to bottom
2. `EKF_SPEC.md` §2 and §3 (state model and attitude estimator)
3. `ORIENTATION_SPEC.md` (frame conventions)
4. `SENSOR_SPEC.md` (LSM6DSO32, MS5611, MMC5983MA registers and rates)
5. `L0_DISCOVERY.md` (once produced)
6. Python reference code at `/home/claude/eskf/` (sources of truth for the numerical comparison)
