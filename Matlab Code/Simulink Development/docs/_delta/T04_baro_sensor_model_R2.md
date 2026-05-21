# T04 Baro Sensor Model — Round 2 Delta Brief

Round 1 of T04 (sub-agent on 2026-05-20 20:49–20:56) failed 5 of 11 acceptance criteria. The agent died at report-back via socket error after the failures were already documented in STATUS.md. Round 2 must address the failures explicitly. This document is the canonical R2 brief.

Reference: round 1 STATUS.md lives at `Matlab Code/Simulink Development/build/T04_baro_sensor_model/STATUS.md` (will be overwritten by round 2 — read it first, then proceed).

## What's broken (must fix in R2)

### Failure 1 — AC1: pad mean pressure off by 8.7 kPa
- R1 result: pad_mean = 92,653.84 Pa, truth ground = 101,325.00 Pa, |delta| = 8671 Pa.
- Likely cause: pad altitude reference is being read as ~700 m (where 1013→925 mbar) instead of the truth 0 m (sea level), OR the pressure model is computed at the wrong altitude.
- Diagnostic: trace where the pad altitude is sourced. The truth bus `pos_NED(3)` on the pad should be 0 m (NED Z = down, on pad → Z = 0 nominally). Confirm via T01's truth_trajectory.mat first sample.
- Acceptance: re-run AC1 expecting |delta| < 20 Pa.

### Failure 2 — AC2: noise sigma is 6× too high
- R1 result: sigma_white = 38.45 Pa vs target 5.94 Pa (range [4, 8]).
- Likely cause: noise model is summing colored + white instead of using only white at the spec's level; or the noise scaling constant is being misapplied (e.g., dividing by something it shouldn't be, or RawCounts vs Pa unit mix-up).
- Diagnostic: read `Baro.NoiseSigma_Pa` (or equivalent) from `casper_sensor_params.m` and confirm the noise model writes Pa directly without an extra gain.
- Acceptance: sigma_white in [4, 8] Pa.

### Failure 3 — AC3: PSD not flat above 0.1 Hz; low-freq drift present
- R1 result: flat-band dB range = 92.1 dB (limit 25), drift_present = 1.
- Likely cause: random walk / bias drift component is too large and bleeding into the in-band noise. May share root cause with failure 2.
- Acceptance: PSD flat-band range < 25 dB; no detectable low-freq drift (per spec definition).

### Failure 4 — AC10: build script crashes on `DeterministicDataTransfer`
- R1 result: "RateTransition block does not have a parameter named 'DeterministicDataTransfer'".
- Root cause: parameter name is wrong for MATLAB R2025b. In R2025b, the parameter is `Integrity` (with values 'on'/'off'), and the related deterministic-transfer setting is `Deterministic` (not `DeterministicDataTransfer`).
- Fix: replace `DeterministicDataTransfer` with the correct parameter name. Test by running `build_baro_block.m` standalone — it must exit zero and produce `baro_block.slx`.
- Acceptance: `build_baro_block.m` runs without error and writes `baro_block.slx`.

### Failure 5 — AC11: baro_block doesn't exist in casper_sim_lib.slx
- R1 result: cascade of AC10. `baro_block.slx` was NOT produced (marked MISSING in R1 deliverables).
- Acceptance: 0.2 s sim of the Simulink baro_block produces sensible pressure output (~101 kPa on pad).

## What works (keep, don't regress)

These passed in R1 and should still pass in R2:
- AC4: Mach-shock toggle on/off at 0.6/0.4 (hysteresis correct).
- AC5: Mach-shock peak deviation magnitude (~433 Pa at M=1, ρ=0.05).
- AC6: 1 Pa quantization grid.
- AC7: 100 Hz output rate.
- AC8: reproducibility (bit-identical with same seed).
- AC9: three plots written.

R2 should not regress these. Keep the mach-shock implementation in `casper_baro_mach_shock.m` essentially as-is.

## R2 execution constraints

- Same hard constraints as the original dispatch (no firmware modifications, no other build dirs touched, no git operations).
- Start by reading the R1 STATUS.md so you understand prior failures concretely.
- You may overwrite R1 files. Do not delete the existing .m files until you've read and learned from them.
- Write R2's STATUS.md tagged as "Round 2" so the round history is visible.
- Hard target: 11/11 PASS. Anything less, document concretely.
