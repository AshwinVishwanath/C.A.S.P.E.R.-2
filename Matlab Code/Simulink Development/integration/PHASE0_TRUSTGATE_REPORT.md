# Phase 0 Trust Gate Report

**Verdict**: FAIL

- **Run timestamp**: 2026-05-22T00:18:48Z UTC
- **MATLAB version**: 25.2.0.2998904 (R2025b)
- **Simulink version**: unknown
- **Seed**: 20260519
- **Git commit**: 98dfa0c
- **Test ID**: PHASE0_trustgate

## Headline metrics

| Metric | Threshold | Actual | Status |
|---|---|---|---|
| Apogee altitude error | <= 10 m | 80.13 m | FAIL |
| Apogee time error | <= 0.5 s | 0.033 s | PASS |
| Velocity at burnout | <= 2 m/s | 2.128 m/s | FAIL |
| Velocity RMS (powered) | <= 5 m/s | 1.013 m/s | PASS |
| Velocity RMS (coast) | <= 3 m/s | 1.703 m/s | PASS |
| Tilt RMS (powered) | <= 1 deg | 0.8956 deg | PASS |
| Tilt RMS (coast) | <= 2 deg | 0.8628 deg | PASS |
| Mach gate engage delay | <= 0.25 s | 0.000 s | PASS |
| Mach gate release delay | <= 0.5 s | -0.001 s | PASS |
| Final accel bias | <= 0.474 m/s^2 | -3.9052 m/s^2 | FAIL |
| Final baro bias | <= 2.598 m | 0.0000 m | PASS |
| Determinism (two runs) | identical | sensor:yes, est:yes | PASS |

## Detailed metrics

### apogee -- FAIL

- Details: truth_apogee=33615.42 m @ 81.098 s; est_apogee=33695.54 m @ 81.131 s; dAlt=80.126 m (<=10.0), dT=0.033 s (<=0.50)

### velocity -- FAIL

- Details: burnout @ 3.36 s; rms_pwr=1.013 m/s (<=5.0), burnout_err=2.128 m/s (<=2.0), rms_coast=1.703 m/s (<=3.0)

### attitude -- PASS

- Details: rms_pwr=0.8956 deg (<=1.0), rms_coast=0.8628 deg (<=2.0), max=2.9835 deg

### mach_gate -- PASS

- Details: engage truth=0.728 s, est=0.729 s, dT=0.000 s (<=0.25); release truth=71.413 s, est=71.412 s, dT=-0.001 s (<=0.50)

### bias -- FAIL

- Details: final ab=-3.9052 m/s^2 (<=0.474), final bb=0.0000 m (<=2.598), P_floor_min=0.0905 (>=0.01)

### determinism -- PASS

- Details: sensor[7b94629a vs 7b94629a] estimate[791a9b87 vs 791a9b87]

### sanity -- PASS

- Details: nan_signals={}; inf_signals={}; runtime=5.95 s (<=300)

## Sanity checks

- NaN signals: 
- Inf signals: 
- Wall-clock runtime: 5.95 s (budget 300 s) -> PASS

## Plots

- `altitude_truth_vs_est.png`
- `attitude_error_euler.png`
- `bias_states.png`
- `covariance_diag.png`
- `eskf_innovations.png`
- `mach_gate_state.png`
- `mag_radio_interference_zoom.png`
- `sensor_snapshot_drogue.png`
- `sensor_snapshot_pad.png`
- `sensor_snapshot_peak_mach.png`
- `tilt_angle_error.png`
- `velocity_truth_vs_est.png`

## Failures

- **apogee**: truth_apogee=33615.42 m @ 81.098 s; est_apogee=33695.54 m @ 81.131 s; dAlt=80.126 m (<=10.0), dT=0.033 s (<=0.50)
- **velocity**: burnout @ 3.36 s; rms_pwr=1.013 m/s (<=5.0), burnout_err=2.128 m/s (<=2.0), rms_coast=1.703 m/s (<=3.0)
- **bias**: final ab=-3.9052 m/s^2 (<=0.474), final bb=0.0000 m (<=2.598), P_floor_min=0.0905 (>=0.01)

## Deviations

_None recorded by this run (T10 captures any sub-agent-injected deviations in STATUS.md)._

## Discord update

```
__phase 0 trust gate__: FAIL

apogee error: 80.13 m
burnout vel error: 2.13 m/s
tilt RMS, powered: 0.896 deg
mach gate behaved correctly: yes
determinism check: PASS

next step: diagnose failing metric per PHASE0_SPEC.md S4.
```

