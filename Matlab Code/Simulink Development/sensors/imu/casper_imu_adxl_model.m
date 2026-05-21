function [accel_g_body_std, fifo_active] = casper_imu_adxl_model( ...
        accel_NED, quat_std, alt_m, alt_launch_thresh_m)
%CASPER_IMU_ADXL_MODEL Clean ADXL372 high-g measurement from truth.
%
% Synopsis:
%   [accel_g_body_std, fifo_active] = casper_imu_adxl_model( ...
%       accel_NED, quat_std, alt_m, alt_launch_thresh_m)
%
% Inputs:
%   accel_NED            (3x1 double, m/s^2) world-frame NED accel (gravity-free)
%   quat_std             (4x1 double) Hamilton body-to-NED, scalar-first
%   alt_m                (1x1 double, m)  truth altitude above launch site
%                                          (positive = up). Used to detect
%                                          post-launch transition.
%   alt_launch_thresh_m  (1x1 double, m)  threshold for FIFO activation
%                                          (Phase 0: 5 m, per T03 §6.3)
%
% Outputs:
%   accel_g_body_std (3x1 double, g)   specific-force in body frame (g)
%   fifo_active      (1x1 logical)     true once alt > threshold (latched up)
%
% Frame convention: STANDARD AIRCRAFT BODY (X-fwd, Y-right, Z-down).
%
% Source firmware references:
%   - Software/App/drivers/adxl372.c (range, scale, FIFO mode)
%   - references/FIRMWARE_CONSTANTS.md §5.2.

    persistent fifo_latched;
    if isempty(fifo_latched)
        fifo_latched = false;
    end

    a_ned = double(accel_NED(:));
    q     = double(quat_std(:));

    g0    = 9.80665;
    g_ned = [0.0; 0.0; g0];

    % --- Quaternion -> NED-to-body rotation ---
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

    a_specific_body  = R_nb.' * (a_ned - g_ned);
    accel_g_body_std = a_specific_body / g0;

    % --- One-way FIFO latch (no return to 400 Hz) ---
    if ~fifo_latched && (alt_m > alt_launch_thresh_m)
        fifo_latched = true;
    end
    fifo_active = fifo_latched;
end
