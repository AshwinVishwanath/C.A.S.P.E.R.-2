# T06 GPS Sensor Model — Visual (gpsSensor) Status

Round dispatched 2026-05-21 by Opus manager. Sonnet sub-agent built the
gpsSensor-backed `gps_block_visual.slx` alongside the legacy
`gps_block.slx`, mirroring the T03 IMU visual-PoC pattern proven in the
previous round.

Build dir: `E:/C.A.S.P.E.R/C.A.S.P.E.R Flight Software V2/Casper 2 flight firmware/C.A.S.P.E.R.-2/Matlab Code/Simulink Development/build/T06_gps_sensor_model`

**Overall: PASS** (26 / 26 quirks tests, 5 / 5 compile-test checks)

## Deliverables (visual round)

| File | Purpose |
|---|---|
| `casper_gps_step_visual.m` | Persistent `gpsSensor` System object wrapper. Pulls launch site + sigmas from base workspace `GPS` struct, seed from `Sim.Seed + 5`, returns `[lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps]`. |
| `casper_gps_quirks.m` | Firmware quirks: COCOM gate (truth-side v/alt thresholds), fix/sv schedule, last-valid hold, 1-sample (100 ms @ 10 Hz) latency FIFO, int32 NAV-PVT encoding (1e-7 deg, mm, mm/s). |
| `build_gps_block_visual.m` | Programmatic Simulink construction. Wires SensorInputBus → BusSelector → Rate Transitions @ `1/GPS.Rate_Hz` → `gpsSensor_step` MATLAB Fn → `PostQuirks` MATLAB Fn → 9 typed outports (matches GPSOutputBus order). |
| `gps_block_visual.slx` | Generated library with one subsystem `gps_visual_block`. |
| `test_gps_quirks.m` | 26 isolation unit tests for COCOM trigger / release / hold / int32 encoding / FIFO latency. |
| `test_gps_block_visual_compile.m` | Compile + 0.5 s sim of the visual subsystem on a stationary truth bus. Verifies lat/lon/alt are near London launch site (51.5074, -0.1278, 35 m), fix=3, sv=12, data_ready=1. |

## Test results

### `test_gps_quirks` — 26 PASS / 0 FAIL

| Group | Tests | Status |
|---|---|---|
| T1 (NAV-PVT int32 encoding) | 8 (lat/lon/alt/vN/vE/vD/fix/sv) | PASS |
| T2 (first-tick FIFO priming) | 3 (dr=false, lat=0, fix=0) | PASS |
| T3 (second-tick latency emission) | 1 (dr=true) | PASS |
| T4 (COCOM trigger) | 5 (dr/fix/sv during COCOM + pre-warmup) | PASS |
| T5 (COCOM release schedule) | 6 (immediate FIFO-buffered + re-acquire + return to 3D) | PASS |
| T6 (held value during COCOM) | 3 (lat/alt/vel pinned to pre-COCOM) | PASS |

### `test_gps_block_visual_compile` — 5 PASS / 0 FAIL

Pad-static (`pos_NED=[0;0;0]`, `vel_NED=[0;0;0]`) input over 5 ticks at 10 Hz.
After FIFO prime:

- `lat_deg7 = 515074028` → 51.5074028° (target 51.5074°)
- `lon_deg7 = -1278112`  → -0.1278112° (target -0.1278°)
- `alt_mm = 33538`       → 33.538 m (target 35 m ± 20 m given noise)
- `fix = 3`, `sv = 12`, `data_ready = 1`
- All samples finite

## Architecture notes

- **Stock block**: `gpsSensor` from Sensor Fusion Toolbox does flat-Earth
  reference-location → WGS84 lat/lon/alt conversion, applies per-axis
  white Gaussian noise (`HorizontalPositionAccuracy`, `VerticalPositionAccuracy`,
  `VelocityAccuracy`) and is fully deterministic via
  `RandomStream='mt19937ar with seed'` + `Seed=uint32(Sim.Seed+5)`. The
  T06 anti-goal "no colored noise" is honored by setting `DecayFactor=0`.
- **Quirks**: COCOM dropout, last-valid hold, latency, and int32 NAV-PVT
  encoding live in `casper_gps_quirks.m` (single MATLAB Function block in
  the subsystem). The spec explicitly notes that `gpsSensor` can't model
  COCOM, so this is the supported pattern.
- **Determinism**: seed is propagated via a Constant block holding
  `Sim.Seed + 5` and bound on each call. A reset_flag (also a Constant,
  `false` in normal use) triggers reconstruction of the persistent
  gpsSensor.
- **Rate**: 10 Hz native (`1/GPS.Rate_Hz`) on all three Rate Transitions.
  Harness drives the model at 1/10 s base step.
- **COCOM gate uses truth-side scalars** (`||vel_NED||`, `-pos_NED(3)`),
  not the noisy gpsSensor output, so a tail noise sample cannot
  spuriously toggle the gate.

## Hard constraints honored

- Did NOT modify Software/, T01 `casper_sim_lib.slx`, any other task dir,
  ARCHITECTURE.md, PHASE0_SPEC.md, FIRMWARE_CONSTANTS.md, or any spec.
- Did NOT commit or modify git state.
- All randomness comes from `Sim.Seed + 5` (gpsSensor `Seed` property).
- R2024a-compatible syntax; no `eval`; no dynamic struct field names from
  strings.
- All physical quantities carry unit suffixes (`lat_deg`, `alt_m`,
  `vn_mps`, `lat_deg7`, `alt_mm`, etc.).
- Hard `error()` on test failure; assertions throughout.

## Deviations / notes

- Default launch site (London 51.5074, -0.1278, 35 m) is injected into the
  base `GPS` struct by `build_gps_block_visual` before sim construction if
  the legacy `casper_gps_local_params` was not called first. The legacy
  `GPS_local` struct is left untouched.
- `gpsSensor` outputs absolute lat/lon/alt (WGS84 referenced to
  `ReferenceLocation`) rather than NED-delta degrees, which differs from
  the legacy `casper_gps_position_model.m` flat-earth math. The behavior is
  spec-equivalent because gpsSensor uses the same flat-Earth approximation
  internally given a single reference location.
- The compile test runs the visual block in isolation against a
  hand-built `SensorInputBus`; full integration into `casper_sim_phase0.slx`
  is T11's responsibility (not in scope here).
- `slprj/` build cache is generated by Simulink during the compile test
  and is not a source artifact.

## Files NOT touched

- `casper_gps_step.m`, `casper_gps_position_model.m`, `casper_gps_cocom_check.m`,
  `casper_gps_hold_lastvalid.m`, `casper_gps_latency.m`, `casper_gps_local_params.m`,
  `build_gps_block.m`, `gps_block.slx`, `test_gps_model.m`, `STATUS.md`,
  `plots/` — all legacy artifacts from the previous R1/R2 build.
