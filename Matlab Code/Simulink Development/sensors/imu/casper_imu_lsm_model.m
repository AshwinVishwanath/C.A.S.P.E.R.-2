function [accel_g_body_std, gyro_dps_body_std, temp_C, data_ready] = ...
        casper_imu_lsm_model(accel_NED, quat_std, omega_body_std)
%CASPER_IMU_LSM_MODEL Clean LSM6DSO32 measurement from truth fields.
%
% Synopsis:
%   [accel_g_body_std, gyro_dps_body_std, temp_C, data_ready] = ...
%       casper_imu_lsm_model(accel_NED, quat_std, omega_body_std)
%
% Inputs (units annotated):
%   accel_NED       (3x1 double, m/s^2) world-frame NED acceleration
%                   (gravity-free; truth bus convention).
%   quat_std        (4x1 double)        Hamilton body-to-NED quaternion,
%                                       scalar-first [w x y z].
%   omega_body_std  (3x1 double, rad/s) body-frame angular rate.
%
% Outputs:
%   accel_g_body_std   (3x1 double, g)   specific-force in body frame,
%                                        in units of g (g0 = 9.80665).
%   gyro_dps_body_std  (3x1 double, dps) body-frame angular rate in deg/s.
%   temp_C             (1x1 double, deg C) Phase 0 constant = 25.
%   data_ready         (1x1 logical) always true at the LSM sample rate.
%
% Algorithm (T03 §5.1):
%   1. Build R_body_from_NED = (body-to-NED rotation)' from quat_std.
%   2. Specific force in body frame:
%         a_specific_body = R_body_from_NED * (a_NED - g_NED),
%      with g_NED = [0; 0; +9.80665] (Z-down).
%   3. accel_g_body_std = a_specific_body / 9.80665.
%   4. gyro_dps_body_std = omega_body_std * 180 / pi.
%   5. temp_C = 25 (firmware reads die temp; truth has no thermal model).
%
% Frame convention: STANDARD AIRCRAFT BODY (X-fwd, Y-right, Z-down).
% Conversion to firmware-frame (Y-nose) happens downstream in T07.
%
% Source firmware references:
%   - Software/App/drivers/lsm6dso32.c (scale factors, range)
%   - references/FIRMWARE_CONSTANTS.md §5.1.

    % --- Force fixed-size, real, double I/O for codegen / Simulink ---
    a_ned = double(accel_NED(:));        % 3x1
    q     = double(quat_std(:));         % 4x1
    w     = double(omega_body_std(:));   % 3x1

    g0      = 9.80665;
    g_ned   = [0.0; 0.0; g0];

    % --- Quaternion (Hamilton, scalar-first, unit-norm) -> rotation matrix ---
    % R_nb = body-to-NED rotation s.t. v_NED = R_nb * v_body.
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    n2 = qw*qw + qx*qx + qy*qy + qz*qz;
    if n2 <= 0
        n2 = 1.0;
    end
    s = 2.0 / n2;
    R_nb = [ ...
        1.0 - s*(qy*qy + qz*qz),   s*(qx*qy - qz*qw),       s*(qx*qz + qy*qw); ...
        s*(qx*qy + qz*qw),         1.0 - s*(qx*qx + qz*qz), s*(qy*qz - qx*qw); ...
        s*(qx*qz - qy*qw),         s*(qy*qz + qx*qw),       1.0 - s*(qx*qx + qy*qy)];

    R_bn = R_nb.';                       % NED-to-body
    a_specific_body = R_bn * (a_ned - g_ned);

    accel_g_body_std  = a_specific_body / g0;
    gyro_dps_body_std = w * (180.0 / pi);
    temp_C            = 25.0;
    data_ready        = true;
end
