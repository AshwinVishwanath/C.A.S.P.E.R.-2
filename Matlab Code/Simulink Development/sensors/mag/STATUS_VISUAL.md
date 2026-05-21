# T05 Magnetometer Visual Subsystem — Status

Generated: 2026-05-21 (Opus takeover finished Sonnet's T05 visual conversion).

This document covers the **visual** rebuild only. The legacy hand-rolled
T05 model (math entirely inside MATLAB Function blocks) is documented in
`STATUS.md` and remains in place unchanged.

## Outcome

**Overall: PASS** — 22 PASS / 0 FAIL across both test scripts.

| Script | PASS | FAIL |
|---|---|---|
| `test_mag_quirks.m`            | 11 | 0 |
| `test_mag_block_visual_compile.m` | 11 | 0 |

## Visual subsystem — mag_block_visual.slx

One Simulink Library subsystem at `mag_block_visual/mag_visual_block`.

Wiring (left to right):
```
truth_bus (SensorInputBus)
   |
   Bus Selector  -> quat_std, pos_NED, time_s, omega_body_std
   Rate Transitions -> all @ 1/Mag.Rate_Hz = 1/100 s
   |
   MagFieldWorld (MATLAB Function -> casper_mag_field_world)   -> mag_NED_uT (3x1)
   |
   MagStep (MATLAB Function -> casper_mag_step)                -> mag_uT_body (3x1)
       wraps imuSensor('accel-gyro-mag'), seeded Sim.Seed+3
       handles NED->body rotation + white noise floor
   |
   MagQuirks (MATLAB Function -> casper_mag_quirks)            -> mag_uT_cal (3x1),
                                                                  raw18 (3x1 uint32)
       firmware-canonical inverse-cal + 18-bit encode + forward cal
   |
   RadioTxStep (MATLAB Function -> casper_radio_tx_step)        -> mag_uT_out (3x1),
                                                                  tx_active (bool)
       schedule = (mod(time_s, period) < airtime),
       spike applied if tx_active && Mag.RadioInterfActive,
       per-axis +/-1 sign drawn once from Sim.Seed+7
   |
   Outports (4, in MagOutputBus order):
       field_uT_body_std (3x1 double)
       raw18             (3x1 uint32)
       radio_active      (bool)  — schedule flag
       data_ready        (bool, tied true at Mag.Rate_Hz)
```

Every numerical parameter binds via base-workspace `evalin` to structs
created by `casper_sensor_params.m` (T02). Seeds derive from `Sim.Seed`
exclusively.

## Files delivered (changed vs Sonnet attempt)

| File | Status | Note |
|---|---|---|
| `casper_mag_step.m`        | **kept**           | Sonnet's imuSensor wrapper, untouched |
| `casper_mag_quirks.m`      | **kept**           | Inverse-cal + 18-bit + forward-cal, untouched |
| `casper_radio_tx_step.m`   | **kept**           | TX schedule + spike layer, untouched |
| `build_mag_block_visual.m` | **kept**           | Library construction, untouched |
| `mag_block_visual.slx`     | **regenerated**    | Rebuilt from scratch to confirm reproducibility |
| `test_mag_quirks.m`        | **kept**           | All 11 checks already passed |
| `test_mag_block_visual_compile.m` | **rewritten** | See "What changed" below |
| `STATUS_VISUAL.md`         | **new**            | This file |

Sonnet's debug probe scratch files (`debug_probe.txt`, `debug_probe2.txt`,
`debug_probe3.txt`, `debug_probe4.txt`, `debug_v.txt`) were deleted.

## What was actually wrong vs Sonnet's diagnosis

Sonnet reported (last debug message):
> "Only 1 element in the dataset. That's the field_uT_body_std. The other
>  3 outputs are NOT being captured. I bet this is because when the outport
>  sample times differ from the harness base rate (the bool ones might be
>  different rates), Simulink decides to skip them OR they get logged
>  separately."

This diagnosis was **wrong on both counts**:

1. **Logging always worked.** `numel(yout)` on a
   `Simulink.SimulationData.Dataset` returns 1 because the Dataset is a
   single-object handle, not an array. The correct accessors are
   `numElements(yout)` and `yout.getElement(k)` — which immediately reveal
   all 4 outport signals (`double [3 1 N]`, `uint32 [3 1 N]`,
   `logical [N 1]`, `logical [N 1]`). T06's `yout{k}` pattern works only
   because T06 implicitly relied on Dataset subsref overloading; either
   API is fine but Sonnet used neither correctly.

2. **The real blocker was the harness, not the block.** The harness drove
   `time_s` as a constant `0`. Inside the subsystem
   `casper_radio_tx_step` computes `tx_active = (mod(time_s, period) <
   airtime)`, which with `time_s=0` is permanently TRUE. With
   `Mag.RadioInterfActive=true` (the default), that meant the +/-10 uT
   radio spike was added to **every sample**, inflating |field| from
   ~40 uT to ~47.6 uT and tripping the magnitude assertion.

## Fix applied to the harness

`test_mag_block_visual_compile.m` was rewritten to:

* Use `numElements`/`getElement` to access all 4 outports correctly.
* Drive `time_s` from a Simulink `Digital Clock` block (sample time =
  base step = 1/100 s) instead of a constant, so the radio-TX schedule
  cycles correctly inside the subsystem.
* Run **two scenarios** in sequence so both invariants are exercised:
   - Scenario A: `Mag.RadioInterfActive = false` -> verify
     |field| ~ 40.18 +/- 2 uT (round-trip identity holds end-to-end
     including imuSensor white noise floor), `radio_active` still strobes
     the schedule (~5 active samples in 21).
   - Scenario B: `Mag.RadioInterfActive = true` -> verify per-axis
     |spike| ~ 10 uT during the TX-on windows.
* Rebuild the library between scenarios so parameter changes
  (`Mag.RadioInterfActive`) propagate.

The build script itself was unchanged — the subsystem was correct from
the start.

## Test summary

`test_mag_quirks.m` (math-only unit tests, MATLAB-level):
```
PASS C2  round-trip identity (max-abs-err = 1.882e-03 uT, limit 0.1)
PASS C2  round-trip <= 2 LSB (1.882e-03 vs 1.221e-02)
PASS C2b off-center round-trip (max-abs-err = 2.156e-03 uT)
PASS C5  raw18 in [0, 262143]
PASS C5  raw18 type/shape (3x1 uint32)
PASS C5  decoded mag on 1-LSB grid
PASS C6  tx_active duty = 0.1500 s/s (target 0.1500)
PASS C7  per-axis |spike|=[10.00 10.00 10.00] uT
PASS C7  axis_sign is +/-1 ([-1 +1 -1])
PASS C9  axis_sign deterministic under fixed seed
PASS C9  different seed yields valid +/-1 draw
```

`test_mag_block_visual_compile.m` (Simulink subsystem in a harness):
```
PASS yout has 4 signals                              (Scenario A)
PASS A.field_uT_body_std all finite (double, [3 1 21])
PASS A.raw18 is uint32 (shape [3 1 21])
PASS A.raw18 range in [0, 262143] (min 129847, max 140093)
PASS A.radio_active schedule count = 5 (expect 2..6 in 21 samples)
PASS A.data_ready all TRUE
PASS A.|field| within +/-2 uT of 40.18 (max dev 1.492)
PASS yout has 4 signals                              (Scenario B)
PASS B.field_uT_body_std all finite (double, [3 1 21])
PASS B.radio_active count = 5 (expect 2..6 in 21 samples)
PASS B.per-axis |spike| = [10.38 9.63 10.46] uT (target 10)
```

## Acceptance criteria mapping (T05 §7)

The C1..C11 criteria from `T05_mag_sensor_model.md` are owned by the
legacy `test_mag_model.m` against `mag_block.slx` (still PASS per
`STATUS.md`). The visual subsystem inherits:
- C2 (round-trip identity) — verified at the math level in `test_mag_quirks`
  and end-to-end in Scenario A.
- C5 (18-bit raw integer in range) — verified at the math level and
  end-to-end (Simulink raw18 outport).
- C6/C7 (TX schedule + +/-10 uT spike) — verified at the math level and
  end-to-end in Scenario B.
- C9 (reproducibility under fixed seed) — verified at the math level for
  `casper_radio_tx_step`; the imuSensor white-noise path also seeds from
  `Sim.Seed+3` and is reproducible by construction.

## Radio interference — placeholder

`Mag.RadioSpikeAmp_uT = 10.0` is an uncalibrated +/-10 uT rectangular
pulse stand-in (ARCHITECTURE.md sec.5, PHASE0_SPEC.md sec.6). Per-axis
sign vector under `Sim.Seed+7 = 20260526` is `[-1, +1, -1]` (same as the
legacy block's value, kept stable).

## Notes for T11 integration

* The subsystem inport is `SensorInputBus`; the outport is the 4 signals
  matching `MagOutputBus` (`field_uT_body_std`, `raw18`, `radio_active`,
  `data_ready`). T11 should add a `Bus Creator` of `MagOutputBus` after
  the subsystem if a single-bus output is needed downstream.
* `radio_active` reflects the **schedule**, independent of
  `Mag.RadioInterfActive`. Downstream consumers wanting "interference is
  being applied right now" should AND with `Mag.RadioInterfActive`.
* `time_s` MUST be a live time signal (Clock or the truth bus from T01),
  not a constant — see the "What was actually wrong" section above.
