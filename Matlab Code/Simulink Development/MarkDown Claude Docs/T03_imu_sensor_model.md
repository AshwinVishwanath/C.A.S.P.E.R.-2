# T03 — LSM6DSO32 + ADXL372 Sensor Models

## 1. Goal

Build Simulink blocks that produce realistic LSM6DSO32 (primary 6-DoF IMU) and ADXL372 (high-g shock accel) sensor outputs from the truth trajectory. Outputs must be at the firmware's exact byte format and at the firmware's exact sample rates.

These blocks live in the sim-side (NED + std-body) frame. They feed the frame-switch block (T07), which converts to firmware frame before the estimator port consumes them.

## 2. Inputs

| Input | Source |
|---|---|
| Truth bus signal | T01 output, runtime via Simulink bus |
| Sensor parameters | T02 `casper_sensor_params.m` (workspace structs `IMU`, `ADXL`, `Sim`) |
| Sample rates | `IMU.Rate_Hz = 833`, `ADXL.RatePostLaunch_Hz = 800`, `ADXL.RatePreLaunch_Hz = 400` |
| Random seed base | `Sim.Seed + 1` for LSM, `Sim.Seed + 4` for ADXL |

## 3. Outputs

In `Software/Sim/build/T03_imu_sensor_model/`:

| File | Purpose |
|---|---|
| `casper_imu_lsm_model.m` | MATLAB Function: clean LSM6DSO32 measurement from truth |
| `casper_imu_lsm_noise.m` | MATLAB Function: bias drift + ARW + cross-axis + saturation |
| `casper_imu_adxl_model.m` | MATLAB Function: clean ADXL372 measurement from truth |
| `casper_imu_adxl_noise.m` | MATLAB Function: noise + saturation + FIFO rate switch |
| `build_imu_block.m` | Simulink build script (constructs both subsystems) |
| `test_imu_model.m` | Unit test |
| `imu_block.slx` (or saved as library) | Persistent Simulink subsystems |
| `plots/imu_pad_1s.png` | Pad-mode 1 s waveform |
| `plots/imu_peak_mach_1s.png` | Trans/sonic 1 s window |
| `STATUS.md` | Result summary |

## 4. Signal flow

```
truth bus  ─►  Rate Transition (10 kHz → 833 Hz)  ─►  imu_lsm_model
                                                          │
                                                          ▼
                                                     imu_lsm_noise
                                                          │
                                                          ▼
                                                     imu_lsm_block out:
                                                       accel_g_body_std (3×1)
                                                       gyro_dps_body_std (3×1)
                                                       temp_C (scalar)
                                                       data_ready (bool)
```

And separately:

```
truth bus  ─►  Rate Transition (10 kHz → 400 or 800 Hz)  ─►  imu_adxl_model
                                                                │
                                                                ▼
                                                            imu_adxl_noise
                                                                │
                                                                ▼
                                                            adxl_block out:
                                                              accel_g_body_std (3×1)
                                                              fifo_active (bool)
```

The body frame here is **standard aircraft** (X-fwd, Y-right, Z-down). Frame conversion to firmware-frame happens downstream in T07.

## 5. LSM6DSO32 model spec

### 5.1 Clean measurement (no noise)

Inputs from truth bus: `accel_NED` (3×1), `vel_NED`, `pos_NED`, `quat_std` (Hamilton, body-to-NED, scalar-first), `omega_body_std` (3×1, body-frame angular rate in rad/s).

Steps:
1. Compute specific force in body frame:  
   `a_specific_body_std = R_body_from_NED * (a_NED - g_NED)`  
   where `g_NED = [0, 0, +9.80665]` (down is positive in NED), and `R_body_from_NED` is the inverse rotation derived from `quat_std`. Use the standard Hamilton rotation: `R = quat_to_rotmat(quat_std)` gives body-to-NED, so `R_body_from_NED = R'` (transpose).
2. Convert to g: `accel_g_body_std = a_specific_body_std / 9.80665`.
3. Output `gyro_dps_body_std = omega_body_std * 180 / pi`.
4. Output `temp_C = 25` (constant for Phase 0; firmware reads die temp, but truth doesn't model thermal environment yet).

### 5.2 Noise model

Apply, in order, to the clean output:

1. **Bias drift (random walk)**:
   - Initial bias: drawn once at sim start from `N(0, IMU.AccelBiasInit_mg)` for accel (typical 10 mg = 0.01 g per axis), `N(0, IMU.GyroBiasInit_dps)` for gyro (typical 0.5 dps per axis).
   - Random walk: at each sample, add `N(0, ACCEL_BI_SIGMA * sqrt(dt))` for accel bias, `N(0, GYRO_BI_SIGMA * sqrt(dt))` for gyro bias (slow drift).
2. **Angle/velocity random walk noise** (per-sample white noise):
   - Accel: `N(0, ACCEL_VRW / sqrt(dt))` per axis.
   - Gyro: per-axis `N(0, GyroArw[i] / sqrt(dt))` using `Attitude.GyroArw_radSqrtS`, then converted to dps.
3. **Cross-axis misalignment** (small constant rotation):
   - Apply once per axis: a 3×3 near-identity matrix with off-diagonals drawn from `N(0, IMU.CrossAxis_deg)` ≈ 0.3°. Convert to small-angle approximation.
   - This is constant per sim run, seeded from `Sim.Seed + 1`.
4. **Scale factor error** (small multiplicative bias):
   - Per-axis scale factor `1 + N(0, IMU.ScaleFactor_ppm)` ≈ 0.5% per axis. Constant per run.
5. **Quantization** to LSB:
   - Accel: round to nearest `IMU.AccelScale_gPerLSB` = 0.000976 g.
   - Gyro: round to nearest `IMU.GyroScale_dpsPerLSB` = 0.070 dps.
6. **Saturation**:
   - Accel: clip to ±32 g per axis.
   - Gyro: clip to ±2000 dps per axis.

Persistent state for bias drift is stored across calls via `persistent` variables. Initialize from seeded RNG on first call; reset between sim runs via subsystem Initialize callback (see `SIMULINK_PATTERNS.md` §8).

### 5.3 Data ready flag

The firmware reads the IMU when its DRDY interrupt fires at ~833 Hz. In the sim, emit `data_ready = true` once per output sample and `false` between. Downstream rate transitions can use it.

## 6. ADXL372 model spec

### 6.1 Clean measurement

Inputs: same as LSM (truth bus).

Steps:
1. Specific force in body frame: same formula as LSM6DSO32.
2. Convert to g: `accel_g_body_std`.

ADXL is accel-only (no gyro, no temp).

### 6.2 Noise model

1. **Bias offset**: small constant offset ≈ 0.01 g per axis, drawn at init.
2. **Quantization**: round to nearest `ADXL.Scale_gPerLSB = 0.1` g (12-bit @ ±200g).
3. **White noise**: per-axis `N(0, ADXL.Noise_g)` ≈ 0.05 g (datasheet typical at 200 Hz BW).
4. **Anti-aliasing filter effect**: the ADXL has a 200 Hz / 400 Hz LPF before ODR. Model as a first-order Butterworth at `ADXL.BandwidthPostLaunch_Hz` cutoff. Use `tf2ss` / `lsim` or a discrete IIR — pick whichever is more idiomatic.
5. **Saturation**: ±200 g per axis.

### 6.3 Rate switching

ADXL runs at 400 Hz pre-launch, 800 Hz post-launch. For Phase 0, "post-launch" means **truth altitude > 5 m above launch site**. Use a switched subsystem (or a Stateflow chart if simpler) that picks the rate based on truth altitude crossing.

In practice, since RasAero starts at zero altitude and goes up monotonically until apogee, the rate transition happens once around t ≈ 0.5 s. The transition is one-way (no return to 400 Hz).

The rate change affects only the sensor *output rate* in this Phase 0 model; the noise statistics scale appropriately (`σ_white / sqrt(dt)` becomes effectively `σ_white * sqrt(rate)`).

## 7. Acceptance criteria

The block is accepted when:

- [ ] On pad (truth state at t=0, vehicle vertical, stationary), the LSM6DSO32 output reads `accel_g_body_std ≈ [+1.000, 0, 0]` to within ±0.02 g (gravity reaction on body-X = forward = up for vertical aircraft body). Note this is **before** the frame switch — sim-body convention.
- [ ] On pad, gyro reads `gyro_dps_body_std ≈ [0, 0, 0]` to within ±0.5 dps per axis.
- [ ] During peak Mach (truth at t≈15s, ~100 m/s² accel), LSM accel output magnitude is `≈ 10.2 g` (1 g gravity + ~9.2 g vertical accel = ~10.2 g specific force on body-X).
- [ ] Quantization is visible: accel output values are integer multiples of 0.000976 g; gyro of 0.070 dps.
- [ ] Saturation: if peak accel were artificially boosted to 40 g, LSM output clips at 32 g. Verify with a synthetic test.
- [ ] ADXL pre-launch reads at 400 Hz (verify time-between-samples in logged output).
- [ ] ADXL post-launch reads at 800 Hz.
- [ ] Both blocks are deterministic: two runs with same seed produce identical output `.mat` files (byte-exact via `isequal`).
- [ ] Plot `imu_pad_1s.png` shows: top — accel three components hovering around `[+1g, 0, 0]` with visible noise; middle — gyro three components around zero with noise; bottom — magnitude (3D norm) near 1 g.
- [ ] Plot `imu_peak_mach_1s.png` shows accel magnitude rising as vehicle accelerates upward.
- [ ] All MATLAB Function blocks specify exact I/O types: `double(3,1)` for vectors, `double(1,1)` for scalars, `logical(1,1)` for flags.
- [ ] STATUS.md reports PASS.

## 8. Anti-goals

- Do not use `imuSensor` from Sensor Fusion Toolbox **unless** you can verify that its noise model parameters exactly match the firmware's ARW/VRW values from `casper_attitude.c`. If you use it, document the parameter mapping in STATUS.md. Manual MATLAB Function implementation is preferred for transparency.
- Do not apply gravity inside the truth bus's `accel_NED` field. Gravity is added in step 1 of §5.1 here.
- Do not skip cross-axis misalignment. Without it, the EKF will see perfect alignment and may converge faster than real flight allows.
- Do not use `rng(...)` inside MATLAB Function blocks — has global side effects. Use `persistent` `RandStream` instance.
- Do not assume body convention is firmware-frame (Y-nose). T03 outputs in **standard aircraft body** (X-fwd). T07 converts.
- Do not pre-compute and cache noise sequences — the sim must produce them via deterministic RNG so changing the seed actually changes the noise.

## 9. Hand-off notes

T07 (frame switch) consumes T03 outputs. Document the exact body-frame convention in STATUS.md so T07's implementer doesn't guess.

T08/T09 (estimator ports) ultimately consume these (after T07's frame switch).

T10 (validation) needs IMU output streams logged for plotting.

## 10. Source firmware references

- `Software/App/drivers/lsm6dso32.c` — sensor scale factors, register layout
- `Software/App/drivers/lsm6dso32.h` — sensor scale factors  
- `Software/App/drivers/adxl372.c` — sensor configuration, FIFO behavior
- `Software/App/drivers/adxl372.h` — register defines
- `Software/App/nav/casper_attitude.c` — gyro ARW values

## 11. References

- `references/FIRMWARE_CONSTANTS.md` §5.1 (LSM6DSO32), §5.2 (ADXL372)
- `references/SIMULINK_PATTERNS.md` §5 (MATLAB Function blocks), §8 (persistent state)
- `ARCHITECTURE.md` §5 (sensor block choices), §6 (reproducibility)
