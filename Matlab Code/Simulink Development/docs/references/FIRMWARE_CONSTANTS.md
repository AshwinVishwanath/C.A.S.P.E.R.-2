# FIRMWARE_CONSTANTS.md — Reference Table

Authoritative table of constants extracted from C.A.S.P.E.R.-2 firmware that the simulator must match. Every value below is cross-referenced to its firmware source. **Do not hand-edit this file**; if a value here disagrees with firmware, the firmware wins — fix the simulator and update this file with a citation diff.

This is a reference, not a parameter file. The deliverable from T02 is `casper_sensor_params.m`, which converts everything here into MATLAB struct form. Sub-agents pulling values must use `casper_sensor_params.m` for the runtime values; this file documents the *provenance* of those values.

## 1. Vertical EKF (`Software/App/nav/casper_ekf.c`)

| Constant | Value | Units | Source line | Notes |
|---|---|---|---|---|
| `G_ACCEL` | 9.80665 | m/s² | `casper_ekf.c` `#define G_ACCEL` | Standard gravity |
| `EKF_DT` | 0.0024 | s | `casper_ekf.c` `#define EKF_DT` | 416 Hz predict |
| `P0_ALT` | 0.1 | m² | `casper_ekf.c` `#define P0_ALT` | Initial P[0,0] |
| `P0_VEL` | 0.001 | (m/s)² | `casper_ekf.c` `#define P0_VEL` | Initial P[1,1] |
| `P0_ACCEL_BIAS` | 0.025 | (m/s²)² | `casper_ekf.c` `#define P0_ACCEL_BIAS` | Initial P[2,2] |
| `P0_BARO_BIAS` | 0.75 | m² | `casper_ekf.c` `#define P0_BARO_BIAS` | Initial P[3,3] |
| `ACCEL_VRW` | 2.162545e-3 | m/s/√s | `casper_ekf.c` `#define ACCEL_VRW` | Velocity random walk |
| `ACCEL_BI_SIGMA` | 1.953783e-4 | m/s²/√s | `casper_ekf.c` `#define ACCEL_BI_SIGMA` | Bias instability |
| `BARO_BI_SIGMA` | 1.0e-3 | m/√s | `casper_ekf.c` `#define BARO_BI_SIGMA` | Baro bias random walk |
| `R_BARO` | 0.5 | m² | `casper_ekf.c` `#define R_BARO` | Baro measurement noise variance, σ≈0.71 m |
| `R_ZUPT` | 6.15e-6 | (m/s)² | `casper_ekf.c` `#define R_ZUPT` | ZUPT measurement noise |
| `BARO_GATE_K2` | 25.0 | — | `casper_ekf.c` `#define BARO_GATE_K2` | 5-sigma² innovation gate (baro only) |
| `P_FLOOR_BARO_BIAS` | 0.01 | m² | `casper_ekf.c` `#define P_FLOOR_BARO_BIAS` | Min for P[3,3] post-update |
| `MACH_GATE_ON` | 0.40 | Mach | `casper_ekf.c` `#define MACH_GATE_ON` | Hysteresis upper threshold |
| `MACH_GATE_OFF` | 0.35 | Mach | `casper_ekf.c` `#define MACH_GATE_OFF` | Hysteresis lower threshold |
| `R_BARO_UNGATE` | 50.0 | m² | `casper_ekf.c` `#define R_BARO_UNGATE` | Inflated R for N steps post-gate |
| `N_UNGATE_STEPS` | 10 | — | `casper_ekf.c` `#define N_UNGATE_STEPS` | Recovery step count |
| `P_UNGATE_ACCEL_BIAS` | 1.0 | (m/s²)² | `casper_ekf.c` `#define P_UNGATE_ACCEL_BIAS` | Bias P reset value |
| `P_UNGATE_BARO_BIAS` | 10.0 | m² | `casper_ekf.c` `#define P_UNGATE_BARO_BIAS` | Baro bias P reset value |
| `EKF_ZUPT_THRESHOLD` | 0.3 | m/s² | `casper_ekf.h` `#define EKF_ZUPT_THRESHOLD` | Caller checks accel mag |

ZUPT gate is `INFINITY` — see `casper_ekf.c` `casper_ekf_update_zupt()`. The Phase 0 port must pass `Inf` (MATLAB) as the gate argument to its Joseph-form scalar update.

Baro update H vector: `[1, 0, 0, 1]` — alt and baro_bias contribute. See `casper_ekf.c` `casper_ekf_update_baro()`.

ZUPT update H vector: `[0, 1, 0, 0]` — velocity only.

State transition matrix Φ (4×4 row-major):
```
[ 1   dt   -dt²/2   0 ]
[ 0   1    -dt      0 ]
[ 0   0    1        0 ]
[ 0   0    0        1 ]
```

Process noise Q (4×4 row-major):
```
[ qa·dt³/3   qa·dt²/2   0       0      ]
[ qa·dt²/2   qa·dt      0       0      ]
[ 0          0          qab·dt  0      ]
[ 0          0          0       qbb·dt ]
```
where `qa = ACCEL_VRW²`, `qab = ACCEL_BI_SIGMA²`, `qbb = BARO_BI_SIGMA²`.

## 2. Attitude estimator (`Software/App/nav/casper_attitude.c`)

| Constant | Value | Units | Source | Notes |
|---|---|---|---|---|
| `STATIC_INIT_MAG_SAMPLES` | 500 | — | `casper_attitude.c` `#define` | Pad init: 500 mag samples |
| `STATIC_INIT_TIMEOUT_S` | 10.0 | s | same | Timeout fallback |
| `HEADING_SIGMA_FLOOR` | 0.01 | rad | same | ~0.6° lower bound |
| `BIAS_GYRO_THRESH` | 0.035 | rad/s | same | Stationary detection (~2 dps) |
| `BIAS_EMA_INV_TAU` | 0.2 | 1/s | same | EMA τ = 5 s |
| `gyro_arw[0]` (body X) | 6.08e-5 | rad/√s | `casper_att_init()` | LSM6DSO32 X-axis ARW |
| `gyro_arw[1]` (body Y) | 4.92e-5 | rad/√s | same | Y-axis (nose) |
| `gyro_arw[2]` (body Z) | 6.73e-5 | rad/√s | same | Z-axis |
| `expected mag magnitude` | 40.18 | µT | `mag_cal.h` `MAG_CAL_EXPECTED_MAG` | After hard/soft iron applied |
| `mag_update_hz` (flight) | 10.0 | Hz | `casper_att_config_t` default | Decimate 100 Hz raw to 10 Hz |
| `Kp_grav` (pad gain) | 10.0 | — | `Software/Core/Src/main.c:385` | Mahony gravity correction gain |
| `Kp_mag_pad` | 0.0 | — | `Software/Core/Src/main.c:386` | Mag correction on pad — **disabled** |
| `Kp_mag_flight` | 0.0 | — | `Software/Core/Src/main.c:387` | Mag correction in flight — **disabled** |
| `Ki` | 0.1 | — | `Software/Core/Src/main.c:388` | Integral gain |
| `gyro_lpf_cutoff_hz` | 50.0 | Hz | `Software/Core/Src/main.c:389` | IIR LPF on gyro |
| `launch_accel_g` | 3.0 | g | `Software/Core/Src/main.c:391` | Launch-detect accel threshold |

**Operational note** (corrected from earlier draft; T02 verification flagged the delta):
Both `Kp_mag_*` are currently zero in flight firmware, meaning mag corrections are disabled. After the static pad init averaging completes, heading propagates open-loop from the gyro (subject to bias drift). `Kp_grav = 10.0` is an order of magnitude higher than the earlier draft suggested. The Phase 0 attitude port (T09) must use these live values from `casper_att_config_t` in `main.c:384-392`, not the prior placeholder defaults.

## 3. Gyro temperature coefficients (`Software/App/nav/temp_cal_coeffs.h`)

| Constant | Value | Units | Notes |
|---|---|---|---|
| `GYRO_TC_T0` | 15.250 | °C | Reference temperature |
| `GYRO_TC_SLOPE_X` | −5.2144e-4 | rad/s/°C | R² = 0.970 (cold ramp) |
| `GYRO_TC_SLOPE_Y` | +2.2923e-4 | rad/s/°C | R² = 0.987 |
| `GYRO_TC_SLOPE_Z` | 0.0 | rad/s/°C | Cross-validation failed; zeroed |

**Phase 0 use**: do not apply gyro temp correction (stripped). Phase 1 adds it back. Sub-agent T02 still includes these constants in the parameter file for future use.

## 4. Magnetometer calibration (`Software/App/cal/mag_cal.c`)

| Constant | Value | Units |
|---|---|---|
| `mag_hard_iron` (mag_cal.c static array) | `[-9.082933, -23.520703, -18.099732]` | µT |
| `mag_soft_iron` (3×3 matrix) | row 0: `[0.777972, -0.017063, -0.010798]` | — |
|   | row 1: `[-0.017063, 0.806570, 0.014623]` | — |
|   | row 2: `[-0.010798, 0.014623, 0.784949]` | — |
| `MAG_CAL_EXPECTED_MAG` | 40.18 | µT |
| Frame mapping | `mx,my,mz = -raw_x, -raw_y, -raw_z` | — |

Application: `cal = soft_iron * (raw_frame_mapped - hard_iron)`.

For the **simulator**, apply the inverse to inject raw "uncalibrated" measurements:
- Start with `mag_clean_body` (the true field in body frame, µT, from truth attitude rotation).
- Apply ×-1 sign flip to get firmware-frame-mapped.
- *De-apply* hard iron: `mag_uncal = (soft_iron \ mag_cal) + hard_iron`.
- Output as the simulated `raw_mag` field.

The estimator's calibration path (in `mag_cal_apply()`) then re-applies the correct hard/soft iron and should recover `mag_clean_body`.

**Note for sub-agent T05**: this is a round-trip identity test. Verify it explicitly. If the round-trip fails by more than 0.1 µT, the matrix is being applied in the wrong order.

## 5. Sensor configurations

### 5.1 LSM6DSO32 IMU (`Software/App/drivers/lsm6dso32.c`)

| Setting | Value | Source |
|---|---|---|
| Accel range | ±32 g | CTRL1_XL = 0x74 |
| Accel scale | 0.000976 g/LSB | per datasheet for ±32g |
| Gyro range | ±2000 dps | CTRL2_G = 0x7C |
| Gyro scale | 0.070 dps/LSB | per datasheet |
| ODR | 833 Hz | both CTRL1_XL and CTRL2_G |
| Performance | High-perf accel + gyro | CTRL6_C = 0x00, CTRL7_G = 0x00 |
| Temp scale | 256 LSB/°C, offset 25°C | `OUT_TEMP_L` decode |
| INT2 | Accel data-ready | INT2_CTRL = 0x01 |

### 5.2 ADXL372 high-g accel (`Software/App/drivers/adxl372.c`)

| Setting | Value |
|---|---|
| Range | ±200 g |
| Scale | 0.1 g/LSB (12-bit, left-justified) |
| Filter | 200 Hz BW pre-launch, 400 Hz BW in FIFO mode |
| ODR | 400 Hz pre-launch, 800 Hz post-launch (FIFO stream) |
| Operating mode | Full-bandwidth measurement |

### 5.3 MS5611 baro (`Software/App/drivers/ms5611.c`)

| Setting | Value |
|---|---|
| OSR | 4096 |
| Conversion delay | 9.1 ms (per OSR_4096) |
| Pressure resolution | 1 mbar = 100 Pa (raw 24-bit) |
| Temperature resolution | 0.01 °C |
| Tick rate | non-blocking, ~100 Hz effective |
| Altitude formula | `44307.694 * (1 - (P_hPa / 1013.25)^0.190284)` |

### 5.4 MMC5983MA mag (`Software/App/drivers/mmc5983ma.c`)

| Setting | Value |
|---|---|
| Range | ±8 Gauss = ±800 µT |
| Resolution | 18-bit unsigned (offset 131072, scale 16384 counts/Gauss) |
| Bandwidth | 800 Hz |
| Mode | Continuous, 100 Hz |
| Auto SET/RESET | Enabled |

### 5.5 MAX-M10M GPS (`Software/App/drivers/max_m10m.c`)

| Setting | Value |
|---|---|
| Rate | 10 Hz NAV-PVT |
| Dynamic model | Airborne <4g (planned upgrade per memory) — currently default |
| Protocol | UBX-only over I²C |
| Latency | typ 100 ms |
| Position accuracy | 1.5 m CEP horizontal (open sky, multi-constellation when enabled) |
| Velocity accuracy | 0.05 m/s |

## 6. Radio TX timing (`Software/App/radio/radio_manager.c`, `radio_config.h`)

| Constant | Value |
|---|---|
| `RADIO_TX_PERIOD_MS` | 100 (10 Hz TX) |
| `RADIO_TX_TIMEOUT_MS` | 200 (max TX wait) |
| Profile A | SF7, BW 250 kHz, CR 4/5, sync 0x12, preamble 8 sym, +20 dBm, 868 MHz |
| Profile B | SF8, same BW/CR (switch above 20 km alt or 500 m/s vel) |
| Profile A airtime | ≈ 15 ms for ~17-byte payload |
| Profile B airtime | ≈ 28 ms |

**Symbol time** = `2^SF / BW`. SF7@250kHz → 128/250000 = 0.512 ms/symbol. ~30 symbols of overhead + payload → ~15 ms airtime.

## 7. Telemetry encoding scales (`Software/App/telemetry/tlm_types.h`)

These don't directly drive sim sensors, but the validation reports should produce values that, if telemetered, would match the firmware's binary stream:

| Scale | Value |
|---|---|
| `ALT_SCALE_M` | 0.01 m / LSB (u24, max 167.7 km) |
| `VEL_SCALE_DMS` | 0.1 m/s / LSB (i16) |
| `TIME_SCALE_100MS` | 0.1 s / LSB (u16) |
| `BATT_OFFSET_V` | 6.0 V |
| `BATT_STEP_V` | 0.012 V / LSB |

## 8. Flight FSM thresholds (`Software/App/fsm/fsm_types.h`)

Not used in Phase 0 (no FSM), but T10 validation may reference these for visualization (e.g., mark the predicted burnout in plots):

| Threshold | Value |
|---|---|
| `FSM_LAUNCH_ACCEL_G` | 2.0 g |
| `FSM_LAUNCH_ACCEL_DWELL_MS` | 100 ms |
| `FSM_LAUNCH_VEL_MPS` | 15.0 m/s |
| `FSM_BURNOUT_ACCEL_G` | 0.0 g |
| `FSM_BURNOUT_DWELL_MS` | 100 ms |
| `FSM_APOGEE_VEL_MPS` | 0.0 m/s |
| `FSM_APOGEE_VEL_DWELL_MS` | 100 ms |
| `FSM_APOGEE_MIN_FLIGHT_S` | 5.0 s |
| `FSM_LANDED_VEL_MPS` | 1.0 m/s |
| `FSM_LANDED_ALT_DELTA_M` | 2.0 m |
| `FSM_LANDED_DWELL_MS` | 3000 ms |

## 9. Frame conventions (consolidated)

See `ARCHITECTURE.md` §3 for the full discussion. Capsule:

- Firmware quaternion: Hamilton `[w, x, y, z]` scalar-first, body-to-nav.
- Firmware body: Y=nose, X=starboard, Z=toward operator.
- Firmware nav: Z-up local-level (despite variable names).
- Firmware gravity in nav: `g_nav = [0, 0, +9.80665]` (Z is up, gravity reads positive).
- Pad accel reading (firmware body): `[0, +9.80665, 0]` (measuring gravity reaction along Y=nose=up).
- Sim nav: NED. Sim body: X-fwd, Y-right, Z-down. Pad accel (sim body): `[+9.80665, 0, 0]`.

## 10. Sources

When in doubt, the firmware is canonical. Path is `Software/App/`. Sub-agents should `grep` for the constant name to find the exact line.

Last verified against firmware commit: **whatever `HEAD` is when you read this**. If you find a discrepancy, fix this file with a one-line note and a commit hash.
