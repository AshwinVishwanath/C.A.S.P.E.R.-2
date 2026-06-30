# T04 — Baro Visual Block (atmoscoesa + MS5611 quirks)

Status: **PASS** — all unit + compile tests green.

This document covers the **visual** baro block (`baro_block_visual.slx`)
built on top of MATLAB's `atmoscoesa` truth path. It is the T03-pattern
re-implementation of T04 per plan
`lets-first-get-started-gentle-newt.md`. The legacy block
(`baro_block.slx`) and its `STATUS.md` are preserved verbatim alongside.

## Deliverables (all under `T04_baro_sensor_model/`)

| File | Purpose |
|---|---|
| `casper_baro_step.m`              | Stateful wrapper: atmoscoesa truth + Mach-shock + noise. Persistent RNG keyed off `Sim.Seed+2`. |
| `casper_baro_quirks.m`            | Pure firmware-quirk wrapper: 1-Pa quantize + MS5611 altitude decode. No state, no RNG. |
| `build_baro_block_visual.m`       | Programmatic Simulink construction of `baro_block_visual.slx`. |
| `baro_block_visual.slx`           | Generated library (one named subsystem: `baro_visual_block`). |
| `test_baro_quirks.m`              | 13 unit tests for the quirks wrapper. |
| `test_baro_block_visual_compile.m`| Compile + 0.1 s sim test of the visual subsystem in a tiny harness. |
| `STATUS_VISUAL.md`                | This file. |

## Block structure (visually auditable)

```
SensorInputBus
   -> Bus Selector (air_pressure_pa, air_temp_K, mach, vel_NED, air_density_kgm3)
   -> 5x Rate Transition to 1/Baro.Rate_Hz (100 Hz)
   -> MATLAB Function "baro_step"     [casper_baro_step]
        - persistent RNG keyed off (Sim.Seed + 2)
        - atmosphere truth (pass-through OR atmoscoesa per Baro_T04.ForceAtmosCOESA)
        - Mach-shock layer (calls casper_baro_mach_shock)
        - bias offset + bias drift random walk + white noise
        - clip to >= 1 Pa
        - emits press_pa_noisy, temp_C
   -> MATLAB Function "PostQuirks"    [casper_baro_quirks]
        - 1-Pa quantization
        - MS5611 altitude decode formula
   -> Outports: press_pa, alt_m, temp_C, data_ready
```

The Mach-shock function `casper_baro_mach_shock.m` is reused verbatim from
the legacy block. The truth-bus pass-through honours the spec note "T01
already runs COESA, do not re-compute"; the `ForceAtmosCOESA` switch in
the optional `Baro_T04` base-workspace struct lets a future regression
test exercise the `atmoscoesa` direct path explicitly.

## Bus contract

- **Input:** `SensorInputBus` (defined by `casper_build_unified_buses.m`).
  Bus Selector reads `air_pressure_pa, air_temp_K, mach, vel_NED, air_density_kgm3`.
- **Outputs (match `BaroOutputBus` field order):**
  - `press_pa`   (double)
  - `alt_m`      (double)
  - `temp_C`     (double)
  - `data_ready` (boolean, constant true at 100 Hz)

## Test results

### `test_baro_quirks` (run in MATLAB R2025b -batch)

```
[T04-quirks] tests starting...
  PASS T1a sea-level p quantized
  PASS T1b sea-level alt ~ 0 m
  PASS T2 1524 m round-trip
  PASS T3 10000 m round-trip
  PASS T4a quantize 101325.4 -> 101325
  PASS T4b quantize 101325.6 -> 101326
  PASS T5a quantize step=10 Pa
  PASS T6 quantize disabled = no-op
  PASS T7a clip negative -> 1 Pa
  PASS T7b alt finite after clip
  PASS T8a determinism (pressure)
  PASS T8b determinism (altitude)
  PASS T9 monotonic: lower P -> higher alt
[T04-quirks] 13 PASS / 0 FAIL
```

### `test_baro_block_visual_compile` (R2025b -batch, 0.1 s sim)

```
[compile] update OK.
[compile] sim OK.
[compile] press_pa data shape: [11 1]
[compile] last press_pa sample: 101327.000 Pa
[compile] last alt_m    sample: -0.1664 m
[compile] last temp_C   sample: 15.0000 degC
[compile] last data_ready sample: 1
[compile] PASS — all baro samples finite, in expected ranges.
```

At sea-level truth (101325 Pa, 288.15 K) the 0.1-s window produces a
quantized pressure of 101327 Pa (per-run bias offset + drift + noise are
well within the spec's ±20 Pa expected band) and the firmware-formula
altitude decode returns -0.17 m, consistent with the ~2 Pa positive
pressure perturbation.

## Determinism / seed scoping

All randomness derives from `Sim.Seed + 2` (per `casper_sim_config` seed
table). Reset semantics match T03: persistent state is rebuilt on
construction, on `reset_flag`, or on a seed change. The `RandStream` is
keyed in `baro_construct_local`, which mirrors `casper_imu_lsm_step.m`
verbatim in structure.

## Hard-constraint compliance

- [x] No edits to firmware (`Software/App/`, `Software/Drivers/`, `Software/Core/`).
- [x] No edits to `casper_sim_lib.slx` (T01) or other tasks' build dirs.
- [x] No git state changes.
- [x] No `eval`, no dynamic struct field names via strings (uses `setfield`-free `if isfield` pattern).
- [x] R2025b syntax (confirmed by `-batch` run); back-compatible to R2024a.
- [x] Physical quantities carry unit suffixes (`press_pa`, `alt_m`, `temp_C`).
- [x] Hard fail on errors (the compile test calls `error()` on every bad path).
- [x] Build artifacts: `.mat` / `.slxc` already gitignored; `.m` / `.slx` / `.md` tracked.

## Harness lesson (per T03)

The compile-test harness uses `FixedStep = 1/100` (== `1/Baro.Rate_Hz`)
so the harness base step is an integer multiple of the sensor rate. The
T03 finding ("`1/833` wasn't an integer multiple of 1 ms, so we set the
harness base step to match the sensor rate exactly") applies identically
here.

## Known limitations (Phase 0 scope)

- The atmosphere truth path defaults to **pass-through** (T01 already runs
  COESA). The `atmoscoesa`-direct mode is wired and selectable via
  `Baro_T04.ForceAtmosCOESA = true` in base workspace, but is not the
  default. The pass-through behaviour matches the legacy `baro_block.slx`
  exactly, which preserves the regression baseline.
- The Mach-shock model is phenomenological per T04 spec §5.2.
- `data_ready` is a constant `true` at 100 Hz (Phase 0 simplification per
  spec §5.5; no D1/D2 interleave modeled).
- The visual block does NOT yet generate the spec's three PNGs
  (`baro_pad_5s.png`, `baro_mach_window.png`, `baro_psd.png`). Those live
  with the legacy block's `STATUS.md` and `test_baro_model.m`. Adding a
  visual-block plot generator is a follow-up T11-integration concern, not
  T04 PoC scope.
