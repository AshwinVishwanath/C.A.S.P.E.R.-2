function [accel_g_out, gyro_dps_out] = casper_imu_lsm_quirks( ...
        accel_mps2_in, gyro_radps_in, ...
        accel_scale_g_per_lsb, gyro_scale_dps_per_lsb, ...
        accel_range_g, gyro_range_dps)
%CASPER_IMU_LSM_QUIRKS Firmware-quirk wrapper for the LSM6DSO32 PostQuirks stage.
%
% Synopsis:
%   [a_g, g_dps] = casper_imu_lsm_quirks(a_mps2, g_radps, ...
%                       a_scale_g_per_lsb, g_scale_dps_per_lsb, ...
%                       a_range_g, g_range_dps)
%
% Purpose:
%   The visual-model T03 IMU subsystem uses imuSensor (Sensor Fusion Tbx)
%   for the random bits (bias, ARW/VRW, cross-axis, scale factor). imuSensor
%   outputs in SI units (m/s^2, rad/s) and has no notion of LSB quantization
%   or hardware saturation range. This wrapper covers the remaining
%   firmware-canonical steps after the imuSensor step():
%       1. Unit conversion to firmware-facing units (g, dps)
%       2. LSB quantization (round to nearest scale step)
%       3. Hardware-range saturation
%
%   The order matches the legacy casper_imu_lsm_noise.m steps 5 (quantize)
%   and 6 (saturate); the unit conversion is the new step that imuSensor
%   forces because of its SI-only outputs.
%
% Inputs:
%   accel_mps2_in       (3x1 double, m/s^2)  imuSensor accel output
%   gyro_radps_in       (3x1 double, rad/s)  imuSensor gyro output
%   accel_scale_g_per_lsb  (1x1 double, g/LSB)   typ 0.000976 g/LSB for +-32g
%   gyro_scale_dps_per_lsb (1x1 double, dps/LSB) typ 0.070 dps/LSB
%   accel_range_g       (1x1 double, g)      typ 32
%   gyro_range_dps      (1x1 double, dps)    typ 2000
%
% Outputs:
%   accel_g_out  (3x1 double, g)
%   gyro_dps_out (3x1 double, dps)
%
% This function is PURE (no persistent state, no randomness) so it is safe
% to use inside a MATLAB Function block at any sample rate. Determinism is
% guaranteed by construction.

    G0 = 9.80665;
    RAD2DEG = 180.0 / pi;

    % 1) Unit conversion
    a_g  = accel_mps2_in(:) / G0;
    g_dp = gyro_radps_in(:) * RAD2DEG;

    % 2) Quantization (round to nearest LSB step)
    a_g  = round(a_g  / accel_scale_g_per_lsb)  * accel_scale_g_per_lsb;
    g_dp = round(g_dp / gyro_scale_dps_per_lsb) * gyro_scale_dps_per_lsb;

    % 3) Saturation
    a_g  = min(max(a_g,  -accel_range_g),  accel_range_g);
    g_dp = min(max(g_dp, -gyro_range_dps), gyro_range_dps);

    accel_g_out  = a_g;
    gyro_dps_out = g_dp;
end
