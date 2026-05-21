function P = casper_imu_local_params()
%CASPER_IMU_LOCAL_PARAMS T03-local supplemental IMU/ADXL noise parameters.
%
% Synopsis:
%   P = casper_imu_local_params()
%
% T02's casper_sensor_params.m defines the firmware-traceable sensor
% structs (IMU, ADXL, Attitude, ...). The T03 spec calls out a small set
% of additional noise-model parameters (bias init, cross-axis, scale
% factor, ADXL noise/init) that are sim-side modelling choices rather
% than firmware constants, so they live here in the T03 build dir.
%
% Output struct fields:
%   .AccelBiasInit_g     scalar, 1-sigma initial accel bias per axis (g)
%   .GyroBiasInit_dps    scalar, 1-sigma initial gyro bias per axis (dps)
%   .CrossAxis_deg       scalar, 1-sigma cross-axis misalignment per axis (deg)
%   .ScaleFactor_ppm     scalar, 1-sigma per-axis scale-factor error (ppm)
%   .ADXL_BiasInit_g     scalar, 1-sigma ADXL initial bias per axis (g)
%   .ADXL_Noise_g        scalar, white noise 1-sigma at the ADXL bandwidth (g)
%   .ADXL_LPF_Order      scalar, anti-alias filter order (use 1)
%
% Source:
%   T03_imu_sensor_model.md §5.2, §6.2 (typical/nominal values cited).
%
% Notes:
%   These are deliberately conservative typical values, not bench-cal'd.

    P = struct();
    P.AccelBiasInit_g  = 0.010;     % 10 mg (T03 §5.2 step 1, "typical")
    P.GyroBiasInit_dps = 0.500;     % 0.5 dps (T03 §5.2 step 1, "typical")
    P.CrossAxis_deg    = 0.300;     % 0.3 deg (T03 §5.2 step 3, "typical")
    P.ScaleFactor_ppm  = 5000;      % 0.5% = 5000 ppm (T03 §5.2 step 4)
    P.ADXL_BiasInit_g  = 0.010;     % 10 mg (T03 §6.2 step 1)
    P.ADXL_Noise_g     = 0.050;     % 50 mg (T03 §6.2 step 3, datasheet typ)
    P.ADXL_LPF_Order   = 1;         % first-order Butterworth (T03 §6.2 step 4)
end
