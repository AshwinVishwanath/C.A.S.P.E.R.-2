function [accel_mps2_body, gyro_radps_body, rebuilt] = casper_imu_lsm_step( ...
        accel_nav_mps2, omega_body_radps, orientation_quat_wxyz, ...
        seed_base, reset_flag)
%CASPER_IMU_LSM_STEP Stateful wrapper around imuSensor for LSM6DSO32.
%
% Synopsis:
%   [a_mps2, g_radps, rebuilt] = casper_imu_lsm_step( ...
%       accel_nav_mps2, omega_body_radps, orientation_quat_wxyz, ...
%       seed_base, reset_flag)
%
% Inputs:
%   accel_nav_mps2          (1x3 row, m/s^2)  nav-frame inertial accel
%                                             (gravity-free; truth bus)
%   omega_body_radps        (1x3 row, rad/s)  body-frame angular rate
%   orientation_quat_wxyz   (1x4 row)         scalar-first body-to-nav quat
%   seed_base               (1x1 double)      RNG seed (typically Sim.Seed+1)
%   reset_flag              (1x1 logical)     true => re-construct imuSensor
%
% Outputs:
%   accel_mps2_body  (1x3 row, m/s^2) IMU accel measurement in body frame
%                                     (includes gravity per IMU convention)
%   gyro_radps_body  (1x3 row, rad/s) IMU gyro measurement in body frame
%   rebuilt          (1x1 logical)    true if the persistent IMU was rebuilt
%                                     this call (diagnostic)
%
% Behavior:
%   - First call (or reset): constructs a persistent imuSensor System object
%     with parameters drawn from base workspace structs (Sim, IMU, IMU_T03,
%     Estimator, Attitude), seeded from seed_base.
%   - Subsequent calls: calls step() on the persistent imuSensor.
%
% This function is intended for use inside a MATLAB Function block in the
% T03 IMU visual subsystem (imu_block_visual.slx). coder.extrinsic is used
% because imuSensor is a System object (not codegen-friendly inline).

    coder.extrinsic('lsm_construct_local', 'lsm_step_local');

    persistent imu;
    persistent last_seed;

    if isempty(imu)
        imu = lsm_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    elseif reset_flag || (seed_base ~= last_seed)
        imu = lsm_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    else
        rebuilt = false;
    end

    % Ensure row-vector shape for imuSensor (it expects Nx3 / Nx4).
    a_in = reshape(double(accel_nav_mps2),   1, 3);
    w_in = reshape(double(omega_body_radps), 1, 3);
    q_in = reshape(double(orientation_quat_wxyz), 1, 4);

    [accel_mps2_body, gyro_radps_body] = lsm_step_local(imu, a_in, w_in, q_in);
end


function imu = lsm_construct_local(seed_base)
% LSM_CONSTRUCT_LOCAL Build imuSensor with parameters from base workspace.
%   Pulls every parameter from base workspace structs so the configuration
%   stays consistent with casper_sensor_params (T02 output).

    Sim       = evalin('base', 'Sim');         %#ok<NASGU> reserved for future
    IMUp      = evalin('base', 'IMU');
    IMU_T03   = evalin('base', 'IMU_T03');
    Estimator = evalin('base', 'Estimator');
    Attitude  = evalin('base', 'Attitude');

    g0 = 9.80665;

    % Sensor Fusion Tbx noise inputs:
    %   NoiseDensity        = ARW/VRW in m/s^2/sqrt(Hz)  or rad/s/sqrt(Hz)
    %   BiasInstability     = bias instability sigma in m/s^2 or rad/s
    %   ConstantBias        = static bias offset (drawn from BiasInit at init)
    %   MisalignmentFactor  = small-angle misalignment (rad)
    %   ScaleFactor         = 1 + per-axis scale-factor error
    accel_white_density = Estimator.AccelVRW;                  % m/s^2/sqrt(Hz)
    accel_bi_sigma      = Estimator.AccelBiSigma * sqrt(IMUp.Rate_Hz);
    %   (BI in m/s^2; legacy stores m/s^2/sqrt(s) — scale by sqrt(rate))
    gyro_white_density  = mean(Attitude.GyroArw_radSqrtS);     % rad/s/sqrt(Hz)
    gyro_bi_sigma       = gyro_white_density / 10.0 * sqrt(IMUp.Rate_Hz);

    % Draw constant biases once at construction, using same seed offset.
    rs0 = RandStream('mt19937ar', 'Seed', uint32(mod(seed_base, 2^32)));
    accel_const_bias = IMU_T03.AccelBiasInit_g * g0 * randn(rs0, 1, 3);
    gyro_const_bias  = IMU_T03.GyroBiasInit_dps * (pi/180) * randn(rs0, 1, 3);

    imu = imuSensor('accel-gyro', ...
        'SampleRate',         IMUp.Rate_Hz, ...
        'RandomStream',       'mt19937ar with seed', ...
        'Seed',               uint32(mod(seed_base, 2^32)));

    imu.Accelerometer.NoiseDensity         = accel_white_density * ones(1, 3);
    imu.Accelerometer.BiasInstability      = accel_bi_sigma      * ones(1, 3);
    imu.Accelerometer.ConstantBias         = accel_const_bias;

    imu.Gyroscope.NoiseDensity             = gyro_white_density  * ones(1, 3);
    imu.Gyroscope.BiasInstability          = gyro_bi_sigma       * ones(1, 3);
    imu.Gyroscope.ConstantBias             = gyro_const_bias;
end


function [a_body_mps2, g_body_radps] = lsm_step_local(imu, a_nav_mps2, w_body_radps, q_wxyz)
% LSM_STEP_LOCAL Call imuSensor with orientation quaternion.
%   Wraps the orientation argument as a quaternion object (imuSensor expects
%   either an Nx4 matrix [w x y z] or a quaternion object).

    q_obj = quaternion(q_wxyz);
    [a_body_mps2, g_body_radps] = imu(a_nav_mps2, w_body_radps, q_obj);
end
