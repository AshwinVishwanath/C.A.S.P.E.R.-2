function [accel_g_out, gyro_dps_out] = casper_imu_lsm_noise( ...
        accel_g_in, gyro_dps_in, dt, ...
        seed_base, ...
        accel_bias_init_g, gyro_bias_init_dps, ...
        cross_axis_deg, scale_factor_ppm, ...
        accel_vrw_mps2_sqrtHz, accel_bi_sigma_mps2_sqrtS, ...
        gyro_arw_radps_sqrtS, ...
        accel_scale_g_per_lsb, gyro_scale_dps_per_lsb, ...
        accel_range_g, gyro_range_dps, ...
        reset_flag)
%CASPER_IMU_LSM_NOISE Apply LSM6DSO32 noise/quantization/saturation chain.
%
% Synopsis:
%   [a_out_g, g_out_dps] = casper_imu_lsm_noise(a_in_g, g_in_dps, dt, ...)
%
% Inputs (units annotated):
%   accel_g_in                 (3x1 double, g)   clean accel input
%   gyro_dps_in                (3x1 double, dps) clean gyro input
%   dt                         (1x1 double, s)   sample period
%   seed_base                  (1x1 double)      RNG seed (Sim.Seed + 1)
%   accel_bias_init_g          (1x1 double, g)   1-sigma initial accel bias
%   gyro_bias_init_dps         (1x1 double, dps) 1-sigma initial gyro bias
%   cross_axis_deg             (1x1 double, deg) 1-sigma misalignment
%   scale_factor_ppm           (1x1 double, ppm) 1-sigma scale-factor error
%   accel_vrw_mps2_sqrtHz      (1x1 double, m/s/sqrt(s)) ACCEL_VRW
%   accel_bi_sigma_mps2_sqrtS  (1x1 double, m/s^2/sqrt(s)) ACCEL_BI_SIGMA
%   gyro_arw_radps_sqrtS       (3x1 double, rad/sqrt(s)) per-axis ARW
%   accel_scale_g_per_lsb      (1x1 double, g/LSB)
%   gyro_scale_dps_per_lsb     (1x1 double, dps/LSB)
%   accel_range_g              (1x1 double, g)   saturation +/- value
%   gyro_range_dps             (1x1 double, dps) saturation +/- value
%   reset_flag                 (1x1 logical) true => reinit persistent state
%
% Outputs:
%   accel_g_out  (3x1 double, g)   noisy + quantized + clipped accel
%   gyro_dps_out (3x1 double, dps) noisy + quantized + clipped gyro
%
% State:
%   Uses persistent RandStream so two runs with same seed are byte-identical.
%   Set reset_flag true on the first sample of each run (Simulink Initialize
%   Function on the parent subsystem) to reinit.
%
% Noise chain (T03 §5.2, in order):
%   1. Bias drift (random walk on accel_bias_g & gyro_bias_dps)
%   2. ARW/VRW white noise per sample
%   3. Cross-axis misalignment (constant 3x3 near-identity)
%   4. Scale factor (constant per-axis 1+eps)
%   5. Quantization to LSB
%   6. Saturation to range
%
% G_ACCEL = 9.80665 m/s^2 used to convert VRW (m/s/sqrt(s)) -> g/sqrt(s).
% Gravity gNED here is irrelevant; we operate on already-body-frame g values.

    persistent rs accel_bias_g gyro_bias_dps M_misalign s_accel s_gyro;

    g0 = 9.80665;

    need_init = isempty(rs) || (islogical(reset_flag) && reset_flag) ...
        || (isnumeric(reset_flag) && reset_flag ~= 0);

    if need_init
        % --- Deterministic per-block RandStream (NO global rng) ---
        rs = RandStream('mt19937ar', 'Seed', uint32(mod(seed_base, 2^32)));

        % --- Initial biases (drawn once) ---
        accel_bias_g  = accel_bias_init_g  * randn(rs, 3, 1);
        gyro_bias_dps = gyro_bias_init_dps * randn(rs, 3, 1);

        % --- Cross-axis misalignment: 3x3 near-identity ---
        % Off-diagonals are small angles (rad), I + skew is the small-angle
        % rotation approximation. Each off-diagonal pair is independent.
        ca_rad = cross_axis_deg * pi / 180.0;
        eps_xy = ca_rad * randn(rs);
        eps_xz = ca_rad * randn(rs);
        eps_yz = ca_rad * randn(rs);
        M_misalign = [ ...
            1.0,     eps_xy,  eps_xz; ...
           -eps_xy,  1.0,     eps_yz; ...
           -eps_xz, -eps_yz,  1.0];

        % --- Scale factor per axis (constant for run) ---
        sigma_sf = scale_factor_ppm * 1e-6;
        s_accel = 1.0 + sigma_sf * randn(rs, 3, 1);
        s_gyro  = 1.0 + sigma_sf * randn(rs, 3, 1);
    end

    % --- Per-sample driver checks ---
    if ~(dt > 0)
        error('casper_imu_lsm_noise:BadDt', 'dt must be > 0, got %g', dt);
    end
    sqrt_dt     = sqrt(dt);
    inv_sqrtdt  = 1.0 / sqrt_dt;

    % --- 1) Bias random walk ---
    % accel bias drifts in m/s^2; convert sigma to g for storage units.
    accel_bias_g  = accel_bias_g  + (accel_bi_sigma_mps2_sqrtS / g0) ...
                    * sqrt_dt * randn(rs, 3, 1);
    % Gyro bias random walk: scale ARW vector down by ~10x to model
    % bias instability (BI ~ ARW / 10 for typical MEMS). Convert rad->dps.
    gyro_bi_sigma_dps_sqrtS = (gyro_arw_radps_sqrtS / 10.0) * (180.0 / pi);
    gyro_bias_dps = gyro_bias_dps + gyro_bi_sigma_dps_sqrtS .* sqrt_dt ...
                    .* randn(rs, 3, 1);

    % --- 2) White (ARW / VRW) per sample ---
    accel_vrw_g_sqrtHz = accel_vrw_mps2_sqrtHz / g0;
    accel_white_g  = accel_vrw_g_sqrtHz * inv_sqrtdt * randn(rs, 3, 1);
    gyro_white_radps = gyro_arw_radps_sqrtS .* inv_sqrtdt .* randn(rs, 3, 1);
    gyro_white_dps   = gyro_white_radps * (180.0 / pi);

    % --- Compose clean + bias + white ---
    a_g  = accel_g_in(:)  + accel_bias_g  + accel_white_g;
    g_dp = gyro_dps_in(:) + gyro_bias_dps + gyro_white_dps;

    % --- 3) Cross-axis misalignment (constant) ---
    a_g  = M_misalign * a_g;
    g_dp = M_misalign * g_dp;

    % --- 4) Scale factor (per-axis) ---
    a_g  = s_accel .* a_g;
    g_dp = s_gyro  .* g_dp;

    % --- 5) Quantization ---
    a_g  = round(a_g  / accel_scale_g_per_lsb)  * accel_scale_g_per_lsb;
    g_dp = round(g_dp / gyro_scale_dps_per_lsb) * gyro_scale_dps_per_lsb;

    % --- 6) Saturation ---
    a_g  = min(max(a_g,  -accel_range_g),  accel_range_g);
    g_dp = min(max(g_dp, -gyro_range_dps), gyro_range_dps);

    accel_g_out  = a_g;
    gyro_dps_out = g_dp;
end
