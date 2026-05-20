# T02 — Sensor Parameter Library — STATUS

**Result:** PASS
**Date:** 2026-05-20
**Firmware commit:** `80210b26318aec39f28818b6985ca2098afe2b9e` (branch `Simulink-Auto-Build-2026-05-20`)
**MATLAB:** R2025b (`25.2.0.2998904`)

## Verification summary

`verify_sensor_params.m` — **93 / 93 constants PASS, 0 FAIL.**

| Section | Count | Result |
|---|---:|---|
| EKF defines (`casper_ekf.c` / `casper_ekf.h`) | 20 | PASS |
| Attitude estimator (`casper_attitude.c`) | 8 | PASS |
| Attitude config (`main.c` `att_cfg` initializer) | 7 | PASS |
| Gyro temperature coefficients (`temp_cal_coeffs.h`) | 4 | PASS |
| Mag calibration (`mag_cal.c` arrays + `mag_cal.h`) | 13 | PASS |
| IMU driver constants (`lsm6dso32.c`) | 8 | PASS |
| Mag driver constants (`mmc5983ma.h`) | 2 | PASS |
| Radio config (`radio_config.[ch]`) | 12 | PASS |
| FSM thresholds (`fsm_types.h`) | 13 | PASS |
| Telemetry scales (`tlm_types.h`) | 5 | PASS |
| GPS rate (`max_m10m.c` CFG_RATE_MEAS) | 1 | PASS |

## Files produced

| File | Purpose |
|---|---|
| `casper_sensor_params.m` | Single-source-of-truth script. Populates `Sim`, `IMU`, `ADXL`, `Baro`, `Mag`, `GPS`, `Estimator`, `Attitude`, `GyroTempCal`, `Radio`, `FSM`, `Telemetry`, `Validation` in caller workspace. Every value annotated with firmware source. |
| `verify_sensor_params.m` | Function-form regression check. Re-reads firmware files at runtime, regex-extracts each constant, asserts MATLAB matches firmware (tolerance `1e-9 * max(1, |fw|)`). Errors out on any mismatch so `matlab -batch` returns nonzero. |
| `STATUS.md` | This file. |

## Acceptance checklist

- [x] `casper_sensor_params.m` executes cleanly in a fresh MATLAB session (no errors, no warnings).
- [x] After execution, all expected struct fields exist: `Sim`, `IMU`, `ADXL`, `Baro`, `Mag`, `GPS`, `Estimator`, `Attitude`, `GyroTempCal`, `Radio`, `FSM`, `Validation` (+ `Telemetry` extra).
- [x] `verify_sensor_params.m` reports zero mismatches with firmware source (93/93 PASS).
- [x] Every value referenced in `FIRMWARE_CONSTANTS.md` is present in the parameter file.
- [x] Every value carries a comment with its firmware source (filename and define/line).
- [x] No numerical value is hand-rounded — exact firmware values reproduced.

## Firmware-vs-doc discrepancies (firmware is canonical)

**Attitude (Mahony) gains: `main.c` overrides `FIRMWARE_CONSTANTS.md §2` "typical config" defaults.**

| Field | `FIRMWARE_CONSTANTS.md` (doc) | `Software/Core/Src/main.c` line 384-392 (canonical) | Used in `Attitude.*` |
|---|---:|---:|---:|
| `Kp_grav` | 1.0 | **10.0** | 10.0 |
| `Kp_mag_pad` | 0.5 | **0.0** | 0.0 |
| `Kp_mag_flight` | 2.0 | **0.0** | 0.0 |
| `Ki` | 0.0 | **0.1** | 0.1 |
| `gyro_lpf_cutoff_hz` | 50.0 | 50.0 | 50.0 (match) |
| `mag_update_hz` | 10.0 | 10.0 | 10.0 (match) |
| `launch_accel_g` | (not in doc; default in `.h` is 3) | 3.0 | 3.0 |

Per CLAUDE.md "Firmware is canonical" rule, `Attitude.Kp_*` and `Attitude.Ki` track the live `att_cfg` initializer values, not the doc defaults. The verification script grep-parses the `main.c` initializer directly so any future change in `main.c` will be caught. The doc note has been left untouched (T02 is read-only to `MarkDown Claude Docs/`). Manager: consider escalating an update to `FIRMWARE_CONSTANTS.md §2` so future readers see the canonical values.

**Implication for downstream tasks:**
- Mag corrections are currently disabled in flight (`Kp_mag_pad = Kp_mag_flight = 0`). The Phase 0 sim should mirror this — heading drifts open-loop from gyro alone once on the pad init quaternion is captured. T08 attitude port should NOT assume mag corrections are active.
- A non-zero integral gain (`Ki = 0.1`) is in use. The attitude estimator's `e_int` accumulator drives the bias-like correction; the stripped Phase 0 port (ARCHITECTURE.md §7) keeps this behavior since it doesn't fall under any of the explicitly stripped items.

## Deviations from spec

1. **Output path override** (per manager dispatch): used `Matlab Code/Simulink Development/build/T02_sensor_params/` instead of the original ARCHITECTURE.md §8 path `Software/Sim/build/T02_sensor_params/`. All downstream tasks land under this canonical Phase 0 build root.
2. **`Telemetry` struct added** (not in task §4 enumeration but referenced in `FIRMWARE_CONSTANTS.md §7`): scaling constants used by the telemetry encoding live in their own struct rather than being scattered across other structs. Downstream telemetry-related blocks should reference `Telemetry.*`.
3. **Profile-A airtime tolerance**: the `2^SF/BW * 30 symbols` physics estimate is `15.36 ms`, while `FIRMWARE_CONSTANTS.md §6` cites `≈15 ms` rounded. The verification script applies a ±1 ms sanity tolerance for this single approximation check (still under exact match for every other constant). Both values are kept in the parameter file: `Radio.ProfileA_Airtime_s = 0.015` (the rounded firmware-doc figure) and the verification computes the unrounded `2^SF/BW * 30` for sanity.

## How to re-run

```matlab
cd('Matlab Code/Simulink Development/build/T02_sensor_params')
casper_sensor_params       % loads structs into caller workspace
verify_sensor_params       % regression check
```

Or from PowerShell:

```powershell
matlab -batch "cd('<path>'); verify_sensor_params; exit"
```

Expected output ends with `[T02] verify_sensor_params: PASS (93/93)` and exit code 0.
