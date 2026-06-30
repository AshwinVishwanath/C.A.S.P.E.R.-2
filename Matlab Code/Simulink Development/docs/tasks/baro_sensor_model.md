# T04 — MS5611 Baro Sensor Model

## 1. Goal

Build a Simulink subsystem that simulates the MS5611 barometric pressure sensor used by C.A.S.P.E.R.-2. The output must match firmware behaviour bit-for-bit at the byte level: pressure in Pa, ~100 Hz effective rate via the firmware's non-blocking state machine, and characterized noise from the HOT/COLD validation dataset.

The block also implements a **transonic baro-error model** (Mach-shock effect) that introduces a systematic static-pressure error when the vehicle crosses Mach. This is what the EKF's Mach gate is designed to reject; the sim must produce the failure mode in order to validate the gate works.

## 2. Inputs

| Input | Source |
|---|---|
| Truth bus signal | T01 (altitude, mach, air_pressure_pa, air_temp_K) |
| Sensor parameters | T02 `Baro` struct |
| Random seed | `Sim.Seed + 2` |
| Sample rate | `Baro.Rate_Hz = 100` Hz nominal |

## 3. Outputs

In `Software/Sim/build/T04_baro_sensor_model/`:

| File | Purpose |
|---|---|
| `casper_baro_pressure_model.m` | MATLAB Function: ideal-atmosphere pressure from truth altitude |
| `casper_baro_mach_shock.m` | MATLAB Function: Mach-shock-induced static pressure error |
| `casper_baro_noise.m` | MATLAB Function: bias drift + white noise + quantization |
| `build_baro_block.m` | Simulink build script |
| `test_baro_model.m` | Unit test |
| `baro_block.slx` | Library subsystem |
| `plots/baro_pad_5s.png` | 5 s pad-mode noise visualization |
| `plots/baro_mach_window.png` | Mach window with shock-error spike |
| `plots/baro_psd.png` | PSD comparison vs firmware's measured baro_psd |
| `STATUS.md` | Result summary |

## 4. Signal flow

```
truth bus  ─►  Rate Transition (10 kHz → 100 Hz)  ─►  pressure_model
                                                          │
                                                          ▼
                                                     mach_shock_model
                                                       (active when Mach > 0.6)
                                                          │
                                                          ▼
                                                       baro_noise
                                                          │
                                                          ▼
                                                       baro_block out:
                                                          pressure_pa (scalar)
                                                          temp_C (scalar)
                                                          data_ready (bool)
```

The block does **not** convert pressure to altitude (the firmware does that via `ms5611_get_altitude`). The output is raw pressure in Pa, matching what the firmware reads from the sensor.

## 5. Pressure model spec

### 5.1 Ideal atmosphere pressure

Inputs: `pos_NED` (3×1, Z is down), `air_pressure_pa` (scalar, from truth bus — already computed by T01 via COESA).

Steps:
1. Take `air_pressure_pa` from truth bus directly. T01 has already computed COESA atmosphere for each altitude; do not re-compute.
2. Pass through unchanged as the clean pressure measurement.

Output: `pressure_clean_pa` (scalar).

### 5.2 Mach-shock effect

Inputs: `pressure_clean_pa`, `mach` (scalar from truth bus), `air_density_kgm3` (scalar).

Steps:
1. Compute dynamic pressure: `q = 0.5 * rho * V²`. From `vel_NED` and `air_density_kgm3`.
2. If `mach < 0.6`, return `pressure_clean_pa` unchanged.
3. Else, compute a Mach-dependent error factor:
   - For `0.6 ≤ M < 0.8` (subsonic, mild): error = `0.05 * q * (M - 0.6) / 0.2` (linear ramp-in from 0 to 5% q).
   - For `0.8 ≤ M < 1.2` (transonic, dramatic): error = `0.05 * q + 0.20 * q * (M - 0.8) / 0.4` (linear from 5% to 25% q).
   - For `1.2 ≤ M < 2.0` (supersonic, recovering): error = `0.25 * q - 0.15 * q * (M - 1.2) / 0.8` (linear from 25% down to 10% q).
   - For `M ≥ 2.0` (high supersonic, residual): error = `0.10 * q`.
4. Apply: `pressure_with_shock = pressure_clean_pa - error` (positive q means lower static pressure on the static port — sign matches typical rocket transonic baro behavior).

This is a **phenomenological** model, not a CFD result. It produces the qualitatively-correct behavior — large transonic error, biggest at M ≈ 1.0 — without claiming numerical accuracy. The job of this model is to give the EKF Mach gate something to reject. Phase 1 may revisit with a higher-fidelity model.

Note: this is one of the largest stated risks in the project memory. The Phase 0 acceptance criteria check that the gate **fires** in the right window, not that the pressure error magnitude is correct.

### 5.3 Noise

Apply, in order:

1. **Bias offset** (constant per run): `N(0, 5 Pa)` — small static bias.
2. **Bias drift (random walk)**: at each sample, add `N(0, BARO_BI_SIGMA² * dt)`. With `BARO_BI_SIGMA = 1e-3 m/√s` and pressure-to-altitude scaling near sea level being ~8.4 Pa/m, the equivalent pressure σ ≈ `0.0084 * sqrt(dt) Pa/sample`. Very slow drift.
3. **White noise per sample**: `N(0, sigma_white)` where `sigma_white = sqrt(R_BARO) * 8.4 Pa` ≈ 5.94 Pa (1σ ≈ 0.7 m altitude noise).
4. **Quantization**: round to nearest `1 Pa` (24-bit ADC at MS5611 sea-level resolution).

### 5.4 Temperature output

Compute: `temp_C = T_K - 273.15` where `T_K` comes from truth bus `air_temp_K`. No noise needed (firmware reads but barely uses it).

### 5.5 Data ready flag

The firmware's MS5611 state machine produces ~100 Hz effective rate via D1 (pressure) + D2 (temp) sequential conversions, ~9 ms each at OSR_4096. Total round-trip ~18 ms = ~55 Hz, but the firmware overlaps the two reads so effective rate is ~100 Hz.

For Phase 0, emit `data_ready = true` at exactly 100 Hz fixed rate. Phase 1 may model the actual interleaved D1/D2 state machine if it matters.

## 6. Acceptance criteria

- [ ] Stationary pad pressure output: mean ≈ `101325 Pa - <ground elevation offset>` to within ±20 Pa over 5 s (1σ noise ≈ 6 Pa → 5-s mean σ ≈ 0.26 Pa, so 20 Pa is generous).
- [ ] White-noise σ at pad: between 4 and 8 Pa per sample (target 5.94 Pa from `sqrt(R_BARO * scale²)`).
- [ ] PSD of pad-mode pressure (over 60 s) is approximately flat (white) above 0.1 Hz, with low-frequency drift visible below 0.05 Hz.
- [ ] Mach-shock active: when truth Mach crosses 0.6, pressure output deviates from clean pressure; when Mach crosses below 0.35 again, it returns to clean pressure.
- [ ] Mach-shock peak deviation occurs at M ≈ 1.0, magnitude on the order of 100–500 Pa (1–5 m altitude equivalent).
- [ ] Quantization visible: pressure values are integers (or 1-Pa increments).
- [ ] Output rate is exactly 100 Hz (verify time-between-samples).
- [ ] Reproducible: two runs with same seed produce identical pressure stream.
- [ ] Plots:
  - `baro_pad_5s.png`: 5 s pad data, pressure & noise around mean.
  - `baro_mach_window.png`: pressure trace 0–60 s with Mach gate ON/OFF markers, showing the shock effect inside the gate window.
  - `baro_psd.png`: PSD of pad pressure with annotations for white-noise floor.
- [ ] STATUS.md PASS.

## 7. Anti-goals

- Do not apply gravity to the pressure (gravity is in `air_pressure_pa` via COESA already).
- Do not produce negative pressure values. Clip to `[1, Inf]` Pa after noise.
- Do not assume sea-level launch site for the absolute pressure offset — the truth's `air_pressure_pa` already accounts for whatever altitude T01 is at.
- Do not couple the Mach-shock error to the random-noise seed — it's a deterministic function of truth.
- Do not skip the PSD plot; it's the regression check for noise correctness.
- Do not interpolate between the four Mach regimes; the kinks at 0.8, 1.2, 2.0 are intentional and easier to inspect.

## 8. Hand-off notes

T07 (frame switch) does **not** transform baro output (pressure is frame-independent scalar).

T08 (EKF port) consumes the pressure stream after converting to altitude via `44307.694 * (1 - (P/P0)^0.190284)` formula (same as firmware `ms5611_get_altitude`).

T10 (validation) needs the baro stream logged.

## 9. Source firmware references

- `Software/App/drivers/ms5611.c` — state machine, conversion timing, OSR
- `Software/App/drivers/ms5611.h` — opcodes, OSR enum

## 10. References

- `references/FIRMWARE_CONSTANTS.md` §1 (R_BARO, BARO_BI_SIGMA), §5.3 (MS5611 config)
- `references/SIMULINK_PATTERNS.md` §5 (MATLAB Function blocks)
- `ARCHITECTURE.md` §7 (stripped estimator — baro is not stripped, Mach gate is kept)
- Note: the Mach-shock model is **phenomenological** for Phase 0. Real CFD or wind-tunnel data would refine it; tag it in STATUS.md as a known low-fidelity model.
