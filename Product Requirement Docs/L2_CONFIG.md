# L2 Config — Mahony Hardening Parameter Matrix

**Spec:** `MAHONY_HARDENING_PRD.md` Level 2 (§5.L2)
**Branch:** `Simulink-Auto-Build-2026-05-20`
**Date:** 2026-05-25

PRD §5 L2 acceptance: "configuration matrix table showing the default
values, the legacy values (for revert), and a one-line rationale for each."

---

## 1. Parameter matrix

All fields live on the base-workspace `Attitude` struct populated by
`Matlab Code/Simulink Development/params/casper_sensor_params.m`. The
runtime tuning hook is `casper.m`'s `tuning.Attitude_*` block.

| Parameter | New default | Legacy (revert) | Rationale |
|---|---|---|---|
| `Kp_Grav` | 10.0 | 10.0 | Unchanged. Base gravity-correction gain; gating is applied multiplicatively, not by lowering Kp. |
| `Kp_MagPad` | **0.0** | 0.0 | Was already 0 in firmware; L2.3 keeps it 0 until real-noise SNR justifies raising it. |
| `Kp_MagFlight` | **0.0** | 0.0 | Same as `Kp_MagPad`; tilt-comp mag correction stays exercised but inert. |
| `Ki` | **0.0** | 0.1 | L2.2: integral feedback was the dominant failure mode in the Python parametric study. Disabled by default; e_int state still accumulates so a future revert can re-enable. |
| `GravGate_WindowHalfWidth_g` | **0.15** | 100.0 (open) | L2.1 NEW. Cosine window half-width in multiples of g; w(±0.15g)=0, w(0)=1. Setting to a large value (≥100) reopens the gate fully → legacy behaviour. |
| `PadCalibDuration_s` | **60.0** | 0.0 | L2.4 NEW. Gyro-bias estimator keeps averaging while in pad mode until this duration elapses. Set to 0 to freeze bias immediately after INIT (= legacy). |
| `LaunchAccel_g` | 3.0 | 3.0 | Unchanged. Launch detection still threshold-driven; L2.4 gates the latch on `pad_calib_complete` (no change to the threshold itself). |
| `StaticInitSamples` | 500 | 500 | Unchanged. |
| `StaticInitTimeout_s` | 10.0 | 10.0 | Unchanged. |
| `HeadingSigmaFloor_rad` | 0.01 | 0.01 | Unchanged. |
| `GyroLpfCutoff_Hz` | 50.0 | 50.0 | Unchanged. |
| `MagUpdateRate_Hz` | 10.0 | 10.0 | Unchanged. |
| `GyroArw_radSqrtS` | `[6.08e-5; 4.92e-5; 6.73e-5]` | same | Unchanged. Allan-derived. |

### Where to flip a value

Tuning knobs are exposed in `casper.m` under the `--- TUNING SECTION ---`
block:

```matlab
tuning.Attitude_Kp_grav                    = [];   % keep firmware default
tuning.Attitude_Kp_mag_pad                 = [];
tuning.Attitude_Kp_mag_flight              = [];
tuning.Attitude_Ki                         = [];
tuning.Attitude_GravGate_WindowHalfWidth_g = [];   % NEW L2.1
tuning.Attitude_PadCalibDuration_s         = [];   % NEW L2.4
```

Leave a field `[]` to keep the `casper_sensor_params.m` default. Set it
to a number to override for the next `casper()` invocation.

---

## 2. Revert-to-legacy recipe

To reproduce pre-L2 behaviour exactly:

```matlab
casper('Profile','apogee');           % loads new defaults
A = evalin('base','Attitude');
A.Ki = 0.1;
A.GravGate_WindowHalfWidth_g = 100;   % open window -> w(|a|) ~ 1 everywhere
A.PadCalibDuration_s = 0;             % freeze bias immediately after INIT
assignin('base','Attitude',A);
```

The 4 PRD-L3 run-matrix configurations (A..E) map directly to subsets of
these knobs:

| Config | `Ki` | `GravGate_WindowHalfWidth_g` | `Kp_mag_*` | `PadCalibDuration_s` |
|---|---|---|---|---|
| A (baseline) | 0.1 | 100 | 0 | 0 |
| B (gate only) | 0.1 | 0.15 | 0 | 0 |
| C (B + Ki=0) | 0.0 | 0.15 | 0 | 0 |
| D (C + Kp_mag=0) | 0.0 | 0.15 | 0 | 0 |
| **E (target)** | **0.0** | **0.15** | **0** | **60.0** |

C and D are numerically identical with the current firmware defaults
(`Kp_MagPad = Kp_MagFlight = 0` even in legacy), so C and D differ only
on builds where the mag gains have been raised.

---

## 3. Mechanism touchpoints

| L2 task | File(s) changed | Behavioural change |
|---|---|---|
| L2.1 magnitude gate | `nav/attitude/casper_attitude_grav_correct.m` (NEW), `casper_attitude_mahony.m`, `casper_attitude_mag_correct_flight.m` | Effective `Kp_grav_eff = Kp_grav * w(|a|)` in BOTH PAD and FLIGHT. Window is the soft cosine bell from PRD §5 L2.1. |
| L2.2 Ki=0 default | `params/casper_sensor_params.m` | `Attitude.Ki = 0`. Integrator state retained. |
| L2.3 Kp_mag defaults | `params/casper_sensor_params.m` | No code change needed; firmware already 0. Comment annotates the L2.3 rationale. |
| L2.4 extended pad calibration | `casper_attitude_tick.m`, `casper_attitude_state_new.m`, `attitude_step_helper.m`, `casper_phase0_run.m` | Post-INIT bias keeps refining; `pad_calib_complete` latches at `mission_time_s ≥ PadCalibDuration_s`; launch detection is gated on this latch. |
| L2.5 single parameter block | `params/casper_sensor_params.m`, `casper.m` | All four new knobs surface together in both the source-of-truth params script and the runtime tuning hook. |

---

## 4. Prelaunch-pad handling

`PadCalibDuration_s` defaults to 60 s to match the PRD §2 validated config.
The attitude module disarms launch detection until the calibration window
closes, so the simulated prelaunch pad must cover at least
`PadCalibDuration_s` of stationary truth.

`casper_phase0_run.m` handles this automatically: if the caller does not
pass `PreLaunchPad_s` explicitly, the driver extends the prepended
stationary pad to `Attitude.PadCalibDuration_s + 2 s` (margin). The
extension is keyed off the live `Attitude` struct, so lowering
`PadCalibDuration_s` shortens the auto-pad consistently. Pin-truth runs
(`'PinTruth', true`) are exempt because the entire run is stationary.

If you do need an explicit pad length (e.g. for a byte-exact regression
against a historical run), pass it explicitly:

```matlab
casper_phase0_run(cfg, truth, 'PreLaunchPad_s', 5.0);   % override auto
```

The visual-model path (`casper_sim_phase0.slx`) consumes `truth_ts` from
the base workspace, which is generated upstream by `casper_load_truth_ts`
— that pipeline does NOT yet auto-extend, so visual-model runs against
short truth need either an explicit `PadCalibDuration_s` override
(tuning knob) or a longer hand-supplied truth_ts. Tracking item.

### Sizing the calibration window

The PRD's 60 s default is empirically derived from the Python parametric
study's `pad_dwell=62` — not first-principles. Bias-estimate std falls
as `ARW / √T_calib`; the apogee-error data from PRD §2 fits an
approximately `T_calib^(-2.5)` law between the two known points (10 s
INIT vs 60 s extended calib). Interpolated minima for the PRD L3 alt
RMSE targets:

| Target alt RMSE | Approx. minimum `PadCalibDuration_s` |
|---|---|
| 2 m | ~22 s |
| 1 m | ~28 s |
| 0.5 m (L3 config E) | ~36 s |
| 0.14 m (Python reference) | ~60 s |

These are interpolations from two data points. Confident reduction
below 60 s should be backed by Python parametric runs at T = 20, 30,
40, 50 s before lowering the default.

---

## 5. Verification

Unit-level: `nav/attitude/test_mahony_hardening.m` exercises
the four mechanisms in isolation. Run with:

```
matlab -batch test_mahony_hardening
```

Current status: 4/4 PASS on 2026-05-25.

System-level: L3 validation pass (`L3_RESULTS.md`) is a separate
deliverable per PRD §5 L3. Not in scope for this L2 milestone.
