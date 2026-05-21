# Phase 0 Trust Gate Report

**Verdict**: PASS

- **Run timestamp**: 2026-05-20T20:39:24Z UTC
- **MATLAB version**: 25.2.0.2998904 (R2025b)
- **Simulink version**: unknown
- **Seed**: 20260519
- **Git commit**: unknown
- **Test ID**: T10_caseA

## Headline metrics

| Metric | Threshold | Actual | Status |
|---|---|---|---|
| Apogee altitude error | <= 10 m | 0.00 m | PASS |
| Apogee time error | <= 0.5 s | 0.000 s | PASS |
| Velocity at burnout | <= 2 m/s | 0.000 m/s | PASS |
| Velocity RMS (powered) | <= 5 m/s | 0.000 m/s | PASS |
| Velocity RMS (coast) | <= 3 m/s | 0.000 m/s | PASS |
| Tilt RMS (powered) | <= 1 deg | 0.0000 deg | PASS |
| Tilt RMS (coast) | <= 2 deg | 0.0000 deg | PASS |
| Mach gate engage delay | <= 0.25 s | 0.000 s | PASS |
| Mach gate release delay | <= 0.5 s | -0.001 s | PASS |
| Final accel bias | <= 0.474 m/s^2 | 0.0000 m/s^2 | PASS |
| Final baro bias | <= 2.598 m | 0.0000 m | PASS |
| Determinism (two runs) | identical | SKIPPED | SKIP |

## Detailed metrics

### apogee -- PASS

- Details: truth_apogee=609.86 m @ 12.197 s; est_apogee=609.86 m @ 12.197 s; dAlt=0.000 m (<=10.0), dT=0.000 s (<=0.50)

### velocity -- PASS

- Details: burnout @ 2.00 s; rms_pwr=0.000 m/s (<=5.0), burnout_err=0.000 m/s (<=2.0), rms_coast=0.000 m/s (<=3.0)

### attitude -- PASS

- Details: rms_pwr=0.0000 deg (<=1.0), rms_coast=0.0000 deg (<=2.0), max=0.0000 deg

### mach_gate -- PASS

- Details: engage truth=1.334 s, est=1.334 s, dT=0.000 s (<=0.25); release truth=4.501 s, est=4.500 s, dT=-0.001 s (<=0.50)

### bias -- PASS

- Details: final ab=0.0000 m/s^2 (<=0.474), final bb=0.0000 m (<=2.598), P_floor_min=0.7500 (>=0.01)

### determinism -- PASS

- Details: No two-run data supplied; metric skipped.

### sanity -- PASS

- Details: No signals supplied; sanity metric skipped.

## Sanity checks

- SKIPPED (no signals supplied).

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

_None._

## Deviations

_None recorded by this run (T10 captures any sub-agent-injected deviations in STATUS.md)._

## Discord update

```
__phase 0 trust gate__: PASS

apogee error: 0.00 m
burnout vel error: 0.00 m/s
tilt RMS, powered: 0.000 deg
mach gate behaved correctly: yes
determinism check: SKIPPED

next step: proceed to Phase 1.
```

