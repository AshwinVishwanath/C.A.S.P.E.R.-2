# T05 — MMC5983MA Magnetometer Model + Radio Interference Layer

## 1. Goal

Build a Simulink subsystem that produces realistic MMC5983MA magnetometer outputs from the truth trajectory, including:
- Earth's magnetic field rotated to body frame using truth attitude.
- Hard-iron and soft-iron distortions (matching firmware calibration constants).
- The firmware's ×-1 axis sign flip.
- Colored noise (AR(1) with τ ≈ 160 ms — known firmware-measured correlation).
- **Radio-TX interference layer**: ±10 µT rectangular pulse during every TX event.

The radio interference is the headline feature of this task. The user has observed mag spikes coincident with radio TX in real hardware, but has not collected calibrated bench data yet. The Phase 0 sim uses a placeholder ±10 µT amplitude that will be re-calibrated against real bench data later.

## 2. Inputs

| Input | Source |
|---|---|
| Truth bus signal | T01 (quat_std for attitude rotation) |
| Sensor parameters | T02 `Mag` struct (hard/soft iron, sign flip, noise τ) |
| Earth mag field | configured or from WMM via `wrldmagm` (Aero Bkst); default constant `[B_north, B_east, B_down]` |
| Radio TX schedule | generated internally per `Mag.RadioTXPeriod_s` and `Mag.RadioTXAirtime_s` |
| Random seed | `Sim.Seed + 3` for noise, `Sim.Seed + 7` for radio interference |
| Sample rate | `Mag.Rate_Hz = 100` Hz |

## 3. Outputs

In `Software/Sim/build/T05_mag_sensor_model/`:

| File | Purpose |
|---|---|
| `casper_mag_field_world.m` | MATLAB Function: Earth mag field in NED frame at given altitude |
| `casper_mag_rotate_to_body.m` | MATLAB Function: rotate NED field to body frame via truth quat |
| `casper_mag_distort_field.m` | MATLAB Function: apply hard/soft iron + sign flip |
| `casper_mag_noise.m` | MATLAB Function: AR(1) colored noise + white noise + quantization |
| `casper_mag_radio_interference.m` | MATLAB Function: TX-windowed additive spike |
| `casper_radio_tx_schedule.m` | MATLAB Function: generate TX active/inactive boolean stream |
| `build_mag_block.m` | Simulink build script |
| `test_mag_model.m` | Unit test |
| `mag_block.slx` | Library subsystem |
| `plots/mag_pad_1s_no_tx.png` | Pad mag readings without radio interference |
| `plots/mag_pad_1s_with_tx.png` | Pad mag readings with TX events visible |
| `plots/mag_tx_event_zoom.png` | Zoomed view of a single TX event showing spike |
| `STATUS.md` | Result summary |

## 4. Signal flow

```
truth bus  ─►  Rate Transition (10 kHz → 100 Hz)  ─►  mag_field_world
                                                          │
                                                          ▼
                                                     mag_rotate_to_body
                                                          │
                                                          ▼
                                                     mag_distort_field
                                                       (×-1 + hard iron + soft iron inverse)
                                                          │
                                                          ▼
                                                     mag_noise
                                                       (AR(1) + white + quantize)
                                                          │
                                                          ▼
                                                     mag_radio_interference
                                                       (gated by radio_tx_schedule)
                                                          │
                                                          ▼
                                                     mag_block out:
                                                       mag_raw_18bit (3×1, uint32)
                                                       mag_uT_body (3×1, double)
                                                       data_ready (bool)
                                                       tx_active (bool, for debugging)
```

## 5. Mag model spec

### 5.1 Earth mag field (NED)

Inputs: `pos_NED` (3×1).

For Phase 0, use a constant Earth field:
- `B_north = 22.0 µT`
- `B_east = 0.5 µT`
- `B_down = 41.5 µT` (positive down — Northern hemisphere typical inclination)
- Total magnitude ≈ 47 µT.

Magnitude is intentionally different from `MAG_CAL_EXPECTED_MAG = 40.18 µT`. The expected-mag is **after** soft-iron correction (which reduces measured magnitude); the raw field is larger. The sim should produce a raw field magnitude in body frame that, after the firmware's calibration path applies hard+soft iron, ends up at 40.18 µT. Verify this round-trip in the unit test.

For Phase 1+, use Aerospace Blockset's `wrldmagm` block with WMM coefficients for the launch site latitude/longitude. Truth would need lat/lon, which RasAero doesn't provide; force it to a default (e.g., user's actual launch site).

### 5.2 Rotate to body frame

Inputs: `mag_NED` (3×1, from §5.1), `quat_std` (4×1, Hamilton scalar-first body-to-NED).

Steps:
1. Compute `R = quat_to_rotmat(quat_std)` (body-to-NED rotation matrix).
2. `mag_clean_body_std = R' * mag_NED` (NED-to-body rotation = transpose).

Output: 3×1 in standard aircraft body frame (X-fwd, Y-right, Z-down).

### 5.3 Apply firmware-side distortion (inverse of calibration)

The sim must produce **raw, uncalibrated** mag readings — what the sensor outputs before the firmware's `mag_cal_apply` runs. To do that, take the clean body field and apply the *inverse* of the firmware calibration:

1. **Soft iron inverse**: `mag_uncal_intermediate = inv(SoftIron) * mag_clean_body`.
2. **Add hard iron**: `mag_uncal_intermediate += HardIron`.
3. **Sign flip ×-1**: `mag_raw_body = -mag_uncal_intermediate` per axis.

This is the inverse pipeline of:
```
firmware: cal = soft_iron * (raw_frame_mapped - hard_iron),  raw_frame_mapped = -raw_sensor
```

Round-trip identity: applying the firmware calibration to the sim's output should recover the truth field within 0.1 µT. The unit test must verify this.

### 5.4 Noise

Apply, in order:

1. **AR(1) colored noise** with τ = 160 ms:
   - Time constant: `τ = 160 ms`, so `α = exp(-dt/τ) = exp(-0.01/0.16) ≈ 0.939` for `dt=10 ms`.
   - Update: `noise[k] = α * noise[k-1] + sqrt(1-α²) * sigma_white * randn`.
   - Per-axis independent. Use `persistent` state.
2. **Quantization**: round to nearest 18-bit LSB. Scale is `(MMC5983MA_18BIT_SCALE = 16384 counts/Gauss) / (100 uT/Gauss * 2) = 81.92 counts/µT`. So one LSB = 1/81.92 ≈ 0.0122 µT.
3. **No saturation needed** — Earth's field is far below ±8 Gauss range.

Persistent AR(1) state must reset cleanly via Initialize callback.

### 5.5 Output as 18-bit raw integer

The firmware reads MMC5983MA's 18-bit unsigned raw register value. Sim should emit both:
- `mag_uT_body` (3×1 double, post-noise µT — useful for plotting)
- `mag_raw_18bit` (3×1 uint32, integer in `[0, 2^18-1]`, matching firmware reads)

Conversion: `raw_18bit = round(mag_uT * 81.92 + 131072)`, clipped to `[0, 262143]`.

The firmware does the reverse: `mag_uT = (raw_18bit - 131072) / 81.92 / 100 * 100` ≈ same scale.

## 6. Radio interference model

### 6.1 TX schedule generator

Inputs: `time_s` (scalar), `Mag.RadioTXPeriod_s`, `Mag.RadioTXAirtime_s`.

A simple deterministic clock:
- TX events at `t = 0, 0.1, 0.2, ...` s.
- Each TX event lasts `Mag.RadioTXAirtime_s` = 0.015 s.
- Output `tx_active = true` when `mod(time_s, Mag.RadioTXPeriod_s) < Mag.RadioTXAirtime_s`.

No jitter in Phase 0. Real firmware has occasional retries and re-init that change cadence; Phase 1 may model these.

### 6.2 Interference layer

When `tx_active` is true, add to each mag axis:
- A constant `± Mag.RadioSpikeAmp_uT` = `± 10 µT` rectangular pulse.
- Sign: per-axis, drawn once at sim init from a uniform random ±1 sequence (seeded). Each axis's sign is constant across all TX events in the sim.
- Some axes may experience near-zero coupling (PCB layout dependent). For Phase 0, **assume all three axes are corrupted symmetrically** — refine in Phase 1 with bench data.

When `tx_active` is false, mag passes through unchanged.

The interference is added **after** the noise model (§5.4). Order matters: noise is sensor-intrinsic; interference is environmental.

### 6.3 Sample rate alignment

Mag is sampled at 100 Hz (every 10 ms). TX events last 15 ms, so each TX corrupts either 1 or 2 mag samples depending on phase alignment. The schedule starts at `t=0` and TX windows are `[0, 15ms]`, `[100ms, 115ms]`, etc. — mag samples at `[0, 10ms, 20ms, ...]` so the first two samples (at 0 and 10ms) are inside the first TX window, but the sample at 20ms is outside. Pattern repeats every 100 ms.

This 1-2 sample-per-TX hit rate is documented in the firmware behavior. Sim must reproduce it.

## 7. Acceptance criteria

- [ ] Stationary pad, no radio interference, no noise: mag output magnitude in body frame matches Earth field magnitude (47 µT here, assuming Northern hemisphere config).
- [ ] After firmware-side calibration (apply hard+soft+sign), the body-frame mag magnitude is `40.18 ± 0.5 µT` (matching `MAG_CAL_EXPECTED_MAG`). This is the round-trip identity check.
- [ ] AR(1) noise correlation: autocorrelation of the mag noise (excluding TX events) at lag 1 sample ≈ 0.94 (matches `exp(-10ms / 160ms)`).
- [ ] White noise PSD floor visible above 5 Hz; colored noise drops off below.
- [ ] Quantization: mag values are integer multiples of 1/81.92 ≈ 0.0122 µT (or, equivalently, `mag_raw_18bit` is an integer in [0, 2^18-1]).
- [ ] Radio TX schedule: `tx_active` is true for exactly 15 ms every 100 ms (1.5 samples per TX cycle at 100 Hz).
- [ ] During TX events, mag readings deviate by ±10 µT (± 1 µT tolerance after noise).
- [ ] Outside TX events, mag readings are within ±3 µT of the clean field (noise band).
- [ ] Reproducible: two runs with same seed produce identical mag stream.
- [ ] Plots:
  - `mag_pad_1s_no_tx.png`: 1 s of mag (`Mag.RadioInterfActive = false`) on pad showing baseline.
  - `mag_pad_1s_with_tx.png`: 1 s with TX on, spikes visible every 100 ms.
  - `mag_tx_event_zoom.png`: 100 ms zoom showing one TX event, with marker for TX-active window.
- [ ] STATUS.md PASS.

## 8. Anti-goals

- Do not assume Earth field is `[40.18, 0, 0]` — that's the post-calibration magnitude, not the raw field direction.
- Do not skip the sign flip ×-1. The firmware's mag_cal.c applies it; sim must produce raw values that, when ×-1 is applied, get into calibration's expected frame.
- Do not put the radio interference inside the noise function. Order matters: noise → interference, separately switchable.
- Do not jitter the TX schedule in Phase 0. Deterministic for reproducibility. Phase 1 can add jitter.
- Do not assume the spike is the same on every axis. Sign per axis is randomized once per run; document the seed and the resulting sign vector in STATUS.md.
- Do not exceed ±15 µT on the spike. The placeholder is ±10 µT; larger values are not justified without bench data.

## 9. Calibration plan (for Phase 1, not Phase 0)

The radio→mag interference model needs calibration against bench data:
- Place the board on a bench with mag sensor active and radio TX'ing.
- Log raw mag samples and radio TX timestamps for ≥ 30 s.
- Fit spike amplitude and per-axis coupling matrix from the data.
- Update `Mag.RadioSpikeAmp_uT` and add a new field `Mag.RadioPerAxisCoupling = [Cx; Cy; Cz]` (per-axis multipliers).
- Phase 1 task to be created.

For Phase 0: ±10 µT placeholder is sufficient to validate that the EKF doesn't blow up when mag is intermittently corrupted.

## 10. Hand-off notes

T07 (frame switch) converts mag output from sim-body to firmware-body convention. The mag block's output is in **standard aircraft body** frame (matches T03 IMU output).

T09 (attitude port) consumes mag (after T07 conversion). Its 10 Hz decimation logic is in the firmware; the Phase 0 attitude port replicates it.

T10 (validation) needs the mag stream + tx_active flag logged for plotting.

## 11. Source firmware references

- `Software/App/drivers/mmc5983ma.c` — sensor register layout, 18-bit decode
- `Software/App/drivers/mmc5983ma.h` — scale constants
- `Software/App/cal/mag_cal.c` — hard/soft iron values, axis flip, calibration function
- `Software/App/cal/mag_cal.h` — expected magnitude constant
- `Software/App/radio/radio_manager.c` — TX cadence behavior
- `Software/App/radio/radio_config.h` — `RADIO_TX_PERIOD_MS`

## 12. References

- `references/FIRMWARE_CONSTANTS.md` §4 (mag calibration), §5.4 (sensor config), §6 (radio TX timing)
- `references/SIMULINK_PATTERNS.md` §5 (MATLAB Function blocks), §8 (persistent state)
- `ARCHITECTURE.md` §5 (sensor block choices)
- Note in `PHASE0_SPEC.md` §6 about Phase 1 mag calibration carrying forward.
