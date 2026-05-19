# T02 — Sensor Parameter Library

## 1. Goal

Mine every sensor and estimator constant from the C.A.S.P.E.R.-2 firmware source and produce a single MATLAB file (`casper_sensor_params.m`) that defines them all as structured workspace variables. This is the **single source of truth** for runtime values used by every downstream sim block.

Sub-agents in T03–T11 must use these struct fields, not hand-coded numbers. If a downstream block has a magic number, it's a bug.

## 2. Inputs

| Input | Path |
|---|---|
| Firmware EKF | `Software/App/nav/casper_ekf.c`, `casper_ekf.h` |
| Firmware attitude | `Software/App/nav/casper_attitude.c`, `casper_attitude.h` |
| Gyro temp coeffs | `Software/App/nav/temp_cal_coeffs.h` |
| Mag calibration | `Software/App/cal/mag_cal.c`, `mag_cal.h` |
| IMU driver | `Software/App/drivers/lsm6dso32.c`, `lsm6dso32.h` |
| ADXL driver | `Software/App/drivers/adxl372.c`, `adxl372.h` |
| Baro driver | `Software/App/drivers/ms5611.c`, `ms5611.h` |
| Mag driver | `Software/App/drivers/mmc5983ma.c`, `mmc5983ma.h` |
| GPS driver | `Software/App/drivers/max_m10m.c`, `max_m10m.h` |
| Radio | `Software/App/radio/radio_config.c`, `radio_config.h`, `radio_manager.c` |
| FSM thresholds | `Software/App/fsm/fsm_types.h` |
| Telemetry scales | `Software/App/telemetry/tlm_types.h` |
| **Reference table** | `Software/Sim/references/FIRMWARE_CONSTANTS.md` |

## 3. Outputs

All files in `Software/Sim/build/T02_sensor_params/`:

| File | Purpose |
|---|---|
| `casper_sensor_params.m` | The parameter file. Sets `Sim`, `IMU`, `ADXL`, `Baro`, `Mag`, `GPS`, `Estimator`, `Attitude`, `Radio`, `FSM` structs in base workspace. |
| `verify_sensor_params.m` | Re-reads the firmware files at runtime and asserts each MATLAB parameter equals its firmware source. |
| `STATUS.md` | Test summary. |

## 4. File structure of `casper_sensor_params.m`

The file is a MATLAB script (not function) that, when executed, populates the base workspace. Layout:

```matlab
% casper_sensor_params.m
% Single source of truth for sim runtime values.
% Each value is annotated with its firmware source (file + line/define).
% Do not hand-edit. Regenerate via T02 if firmware changes.

% Generated: <timestamp>
% Firmware git ref: <git rev-parse HEAD output>

% ===== Sim global =====
Sim.SolverDt = 1e-4;        % s, 10 kHz solver
Sim.Seed     = 20260519;    % default seed
Sim.GravityMps2 = 9.80665;  % m/s², standard gravity (casper_ekf.c G_ACCEL)

% ===== IMU (LSM6DSO32) =====
IMU.Rate_Hz             = 833;         % CTRL1_XL=0x74, lsm6dso32.c
IMU.AccelRange_g        = 32;          % CTRL1_XL=0x74
IMU.AccelScale_gPerLSB  = 0.000976;    % datasheet for ±32g
IMU.GyroRange_dps       = 2000;        % CTRL2_G=0x7C
IMU.GyroScale_dpsPerLSB = 0.070;       % datasheet
IMU.TempScale_LSBperC   = 256;         % OUT_TEMP_L decode
IMU.TempOffset_C        = 25;          % zero point
% ... [continue for all IMU constants from FIRMWARE_CONSTANTS.md §5.1] ...

% ===== ADXL372 (high-G) =====
ADXL.Range_g                = 200;
ADXL.Scale_gPerLSB          = 0.1;
ADXL.RatePreLaunch_Hz       = 400;
ADXL.RatePostLaunch_Hz      = 800;
ADXL.BandwidthPreLaunch_Hz  = 200;
ADXL.BandwidthPostLaunch_Hz = 400;

% ===== Baro (MS5611) =====
Baro.Rate_Hz       = 100;        % nominal effective rate, ms5611.c non-blocking
Baro.OSR           = 4096;       % oversampling
Baro.PressureRes_Pa = 1;         % 24-bit ADC ≈ 1 Pa resolution
% ... etc ...

% ===== Mag (MMC5983MA) =====
Mag.Rate_Hz          = 100;
Mag.RangeGauss       = 8;
Mag.Bits             = 18;
Mag.HardIron_uT      = [-9.082933; -23.520703; -18.099732];   % mag_cal.c
Mag.SoftIron         = [ 0.777972, -0.017063, -0.010798; ...
                        -0.017063,  0.806570,  0.014623; ...
                        -0.010798,  0.014623,  0.784949];     % mag_cal.c
Mag.AxisFlipSign     = [-1; -1; -1];                          % mag_cal.c ×-1
Mag.ExpectedMag_uT   = 40.18;                                 % MAG_CAL_EXPECTED_MAG
Mag.NoiseTauSec      = 0.160;                                 % AR(1) correlation time (memory note)
Mag.NoiseStd_uT      = 0.5;                                   % per-axis 1σ (refine later)

% ===== Mag→Radio interference (placeholder) =====
Mag.RadioInterfActive    = true;
Mag.RadioSpikeAmp_uT     = 10.0;     % ±10 µT rectangular pulse (placeholder)
Mag.RadioTXAirtime_s     = 0.015;    % SF7 ~15 ms
Mag.RadioTXPeriod_s      = 0.100;    % 10 Hz

% ===== GPS (MAX-M10M) =====
GPS.Rate_Hz                  = 10;
GPS.PositionCEP_Horizontal_m = 1.5;
GPS.VelocityNoise_mps        = 0.05;
GPS.Latency_s                = 0.1;
GPS.COCOMVelThreshold_mps    = 500;
GPS.COCOMAltThreshold_m      = 18000;

% ===== Estimator (EKF) =====
Estimator.PredictRate_Hz   = 416;
Estimator.Dt               = 1/416;       % 0.0024 s, casper_ekf.c EKF_DT
Estimator.G                = 9.80665;
Estimator.P0_Alt           = 0.1;
Estimator.P0_Vel           = 0.001;
Estimator.P0_AccelBias     = 0.025;
Estimator.P0_BaroBias      = 0.75;
Estimator.AccelVRW         = 2.162545e-3;
Estimator.AccelBiSigma     = 1.953783e-4;
Estimator.BaroBiSigma      = 1.0e-3;
Estimator.R_Baro           = 0.5;
Estimator.R_Zupt           = 6.15e-6;
Estimator.BaroGateK2       = 25.0;
Estimator.PFloorBaroBias   = 0.01;
Estimator.MachGateOn       = 0.40;
Estimator.MachGateOff      = 0.35;
Estimator.R_BaroUngate     = 50.0;
Estimator.N_UngateSteps    = 10;
Estimator.P_UngateAccelBias = 1.0;
Estimator.P_UngateBaroBias  = 10.0;
Estimator.ZuptThreshold    = 0.3;        % m/s², caller-checked

% ===== Attitude estimator =====
Attitude.GyroArw_radSqrtS  = [6.08e-5; 4.92e-5; 6.73e-5];
Attitude.GyroLpfCutoff_Hz  = 50;
Attitude.MagUpdateRate_Hz  = 10;
Attitude.Kp_Grav           = 1.0;
Attitude.Kp_MagPad         = 0.5;
Attitude.Kp_MagFlight      = 2.0;
Attitude.Ki                = 0.0;
Attitude.StaticInitSamples = 500;
Attitude.StaticInitTimeout_s = 10.0;
Attitude.HeadingSigmaFloor_rad = 0.01;

% ===== Gyro temperature (informational only in Phase 0) =====
GyroTempCal.T0_C    = 15.250;
GyroTempCal.Slope_X = -5.2144e-4;
GyroTempCal.Slope_Y =  2.2923e-4;
GyroTempCal.Slope_Z = 0.0;

% ===== Radio =====
Radio.TX_Period_s   = 0.100;
Radio.TX_Timeout_s  = 0.200;
Radio.ProfileA_SF   = 7;
Radio.ProfileA_BW_Hz = 250000;
Radio.ProfileA_Airtime_s = 0.015;
Radio.ProfileB_SF   = 8;
Radio.ProfileB_BW_Hz = 250000;
Radio.ProfileB_Airtime_s = 0.028;

% ===== FSM thresholds (informational) =====
FSM.LaunchAccel_g       = 2.0;
FSM.LaunchDwell_ms      = 100;
% ... etc ...

% ===== Validation tolerances =====
Validation.ApogeeError_m       = 10.0;
Validation.AttitudeRmsError_deg = 1.0;
Validation.BurnoutVelRms_mps   = 2.0;

% Eof
```

## 5. `verify_sensor_params.m`

A unit-test-style script that:

1. Runs `casper_sensor_params.m` to populate workspace.
2. For each critical constant, re-reads the relevant firmware file at runtime and parses out the value.
3. Asserts MATLAB value matches firmware value (numerical equality, no tolerance for integer constants; `abs(a-b) < 1e-12 * max(1, abs(b))` for floats).
4. Reports any mismatches with file + extracted value + MATLAB value.

The grep/parse logic does not need to be sophisticated — a single regex per constant family is sufficient. For instance, EKF defines all use the pattern `#define\s+(\w+)\s+([-\d.eE+]+)f?`.

Constants to verify (mandatory list):
- All EKF process noise / measurement noise / gate thresholds.
- All gyro ARW values.
- Mag hard-iron vector (all three components).
- Mag soft-iron matrix (all nine components).
- Mag expected magnitude.
- Gyro temperature slopes (all three) and T0.
- IMU sample rate, range, scale.
- Radio TX period and SF7 airtime calculation.

This script is invoked by the manager as part of T02 acceptance and also during T11 integration as a regression check.

## 6. Acceptance criteria

- [ ] `casper_sensor_params.m` executes cleanly in a fresh MATLAB session (no errors, no warnings).
- [ ] After execution, all expected struct fields exist in workspace: `Sim`, `IMU`, `ADXL`, `Baro`, `Mag`, `GPS`, `Estimator`, `Attitude`, `GyroTempCal`, `Radio`, `FSM`, `Validation`.
- [ ] `verify_sensor_params.m` reports zero mismatches with firmware source.
- [ ] Every value referenced in `FIRMWARE_CONSTANTS.md` is present in the parameter file.
- [ ] Every value carries a comment with its firmware source (filename and define/line).
- [ ] No numerical value is hand-rounded — exact firmware values are reproduced.
- [ ] STATUS.md reports PASS with the list of constants verified.

## 7. Anti-goals

- Do not introduce new constants not in firmware (e.g., do not invent a "default Mahony gain" if firmware doesn't have one — escalate).
- Do not skip the `verify_sensor_params.m` step; it's the regression check that catches firmware drift.
- Do not store the firmware git hash inline if you don't actually compute it; if you can't get the git hash from inside MATLAB, leave a placeholder and document it.
- Do not change naming style (e.g., `Sim.solver_dt` instead of `Sim.SolverDt`). Lock to the pattern in §4.

## 8. Hand-off notes

This is the foundation file for all sensor and estimator sub-agents. Any inconsistency here will propagate. The manager runs `verify_sensor_params.m` first thing after T02 completes, and again at the start of T11.

If firmware changes between T02 completion and T11 execution, the verification will catch the drift and the manager will re-dispatch T02.

## 9. Source firmware references

See §2. All of `Software/App/` is fair game for read access; nothing in `Software/App/` may be modified.

## 10. References

- `references/FIRMWARE_CONSTANTS.md` — the comprehensive table this parameter file implements.
- `ARCHITECTURE.md` §9 (naming), §10 (coding standards).
- `references/SIMULINK_PATTERNS.md` §9 (parameter passing convention).
