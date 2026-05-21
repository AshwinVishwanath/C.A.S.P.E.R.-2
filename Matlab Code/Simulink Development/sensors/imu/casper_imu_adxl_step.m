function [accel_mps2_body, rebuilt] = casper_imu_adxl_step( ...
        accel_nav_mps2, omega_body_radps, orientation_quat_wxyz, ...
        seed_base, reset_flag)
%CASPER_IMU_ADXL_STEP Stateful wrapper around imuSensor (high-g) for ADXL372.
%
% Mirror of casper_imu_lsm_step but configured for the ADXL372 noise/range.
% imuSensor doesn't differentiate "high-g" vs "regular" accelerometer
% intrinsically — we just plug ADXL noise parameters in.

    coder.extrinsic('adxl_construct_local', 'adxl_step_local');

    persistent imu;
    persistent last_seed;

    if isempty(imu)
        imu = adxl_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    elseif reset_flag || (seed_base ~= last_seed)
        imu = adxl_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    else
        rebuilt = false;
    end

    a_in = reshape(double(accel_nav_mps2),   1, 3);
    w_in = reshape(double(omega_body_radps), 1, 3);
    q_in = reshape(double(orientation_quat_wxyz), 1, 4);

    accel_mps2_body = adxl_step_local(imu, a_in, w_in, q_in);
end


function imu = adxl_construct_local(seed_base)
    ADXLp   = evalin('base', 'ADXL');
    IMU_T03 = evalin('base', 'IMU_T03');

    g0 = 9.80665;

    % ADXL372 noise: white density inferred from per-sample sigma at sensor BW.
    % Legacy convention: sigma_per_sample = noise_g * sqrt(2*BW*dt).
    %   one-sided PSD = noise_g^2 / BW (per Hz) -> NoiseDensity = noise_g / sqrt(BW)
    noise_density_g_sqrtHz   = IMU_T03.ADXL_Noise_g / sqrt(ADXLp.BandwidthPostLaunch_Hz);
    noise_density_mps2_sqrtHz = noise_density_g_sqrtHz * g0;

    rs0 = RandStream('mt19937ar', 'Seed', uint32(mod(seed_base, 2^32)));
    accel_const_bias = IMU_T03.ADXL_BiasInit_g * g0 * randn(rs0, 1, 3);

    imu = imuSensor('accel-gyro', ...
        'SampleRate',   ADXLp.RatePostLaunch_Hz, ...
        'RandomStream', 'mt19937ar with seed', ...
        'Seed',         uint32(mod(seed_base, 2^32)));

    imu.Accelerometer.NoiseDensity    = noise_density_mps2_sqrtHz * ones(1, 3);
    imu.Accelerometer.BiasInstability = 0 * ones(1, 3);
    imu.Accelerometer.ConstantBias    = accel_const_bias;

    % Gyro path is unused but must be initialized — zero everything.
    imu.Gyroscope.NoiseDensity    = 0 * ones(1, 3);
    imu.Gyroscope.BiasInstability = 0 * ones(1, 3);
    imu.Gyroscope.ConstantBias    = 0 * ones(1, 3);
end


function a_body_mps2 = adxl_step_local(imu, a_nav_mps2, w_body_radps, q_wxyz)
    q_obj = quaternion(q_wxyz);
    [a_body_mps2, ~] = imu(a_nav_mps2, w_body_radps, q_obj);
end
