function diag_visual_accel()
%DIAG_VISUAL_ACCEL Inspect what the visual model SEES at the IMU output
% block (post imu_unit_convert + FrameSwitch_Accel) on the pad.
%
% This bypasses the full sim and instead runs imuSensor directly with
% pad inputs to verify the sign/axis chain.

    fprintf('\n==== diag_visual_accel ====\n');

    % Ensure paths
    if ~exist('casper_sim_config', 'file')
        error('Run casper() first to set up base workspace.');
    end

    % Stationary truth — pull REAL pad quat from truth_ts (rocket nose-up)
    truth_ts = evalin('base', 'truth_ts');
    accel_NED  = truth_ts.accel_NED.Data(1, :);
    omega_body = truth_ts.omega_body_std.Data(1, :);
    quat_std   = truth_ts.quat_std.Data(1, :);
    fprintf('Truth pad q_std = [%.4f %.4f %.4f %.4f]\n', quat_std);

    % Clear persistent state then call imuSensor twice (first sample is noisy)
    clear casper_imu_lsm_step
    casper_imu_lsm_step(accel_NED, omega_body, quat_std, 20260519, true);  %#ok<*ASGLU>
    [a_b, g_b, ~] = casper_imu_lsm_step(accel_NED, omega_body, quat_std, 20260519, false);

    fprintf('imuSensor raw output on stationary pad with identity quat_std:\n');
    fprintf('  accel_body_std [m/s^2] = [%.4f %.4f %.4f]  (units: m/s^2)\n', a_b);
    fprintf('  gyro_body_std  [rad/s] = [%.4f %.4f %.4f]\n', g_b);

    % BUT! The visual model's IMU_LSM block outputs accel_g (g units),
    % and imu_unit_convert does `a_std = -accel_g * 9.80665`. Let's
    % mimic the chain:
    a_g_std = a_b / 9.80665;
    fprintf('\nIMU_LSM block output (accel in g): [%.4f %.4f %.4f]\n', a_g_std);

    a_mps2_std = -a_g_std * 9.80665;
    fprintf('After imu_unit_convert sign-flip (m/s^2 std-body):\n');
    fprintf('  [%.4f %.4f %.4f]\n', a_mps2_std);

    % FrameSwitch_Accel: vec_fw = [v(2); v(1); -v(3)]
    a_fw = [a_mps2_std(2); a_mps2_std(1); -a_mps2_std(3)];
    fprintf('After FrameSwitch_Accel (m/s^2 fw-body):\n');
    fprintf('  [%.4f %.4f %.4f]\n', a_fw);

    fprintf('\n----\n');
    fprintf('What attitude_step_helper expects on pad (legacy convention):\n');
    fprintf('  pad_accel_fw = [0, +%.4f, 0] (Y-nose = up)  -- per casper_frame_constants\n', 9.80665);
    fprintf('What EKF16 helper expects on pad:\n');
    fprintf('  pad_accel_zup = [0, 0, +%.4f] (Z = up)\n', 9.80665);

    % Compare with legacy direct casper_imu_lsm_model output:
    fprintf('\nLegacy casper_imu_lsm_model output on pad (truth quat):\n');
    [a_g_legacy, g_dps_legacy, temp_C, ~] = casper_imu_lsm_model(accel_NED(:), quat_std(:), omega_body(:));
    fprintf('  accel_g [g]      = [%.4f %.4f %.4f]\n', a_g_legacy);
    fprintf('  gyro_dps [dps]   = [%.4f %.4f %.4f]\n', g_dps_legacy);
    a_legacy_mps2_std = a_g_legacy * 9.80665;
    fprintf('  in m/s^2 (no sign flip) = [%.4f %.4f %.4f]\n', a_legacy_mps2_std);
    a_legacy_fw = [a_legacy_mps2_std(2); a_legacy_mps2_std(1); -a_legacy_mps2_std(3)];
    fprintf('  after frame_switch_body (fw) = [%.4f %.4f %.4f]\n', a_legacy_fw);

    fprintf('==== diag_visual_accel DONE ====\n\n');
end
