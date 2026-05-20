function test_imu_lsm_step_smoke()
%TEST_IMU_LSM_STEP_SMOKE Smoke test for casper_imu_lsm_step.
%   Confirms:
%   - imuSensor constructs from base workspace params
%   - First call returns 1x3 row vectors (accel m/s^2, gyro rad/s)
%   - Two calls with same seed produce identical output (determinism)
%   - reset_flag=true reconstructs the IMU

    % Add T11_integration to path then run config to populate base workspace.
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, '..', 'T11_integration'));
    casper_sim_config('Seed', 20260519, 'StopTime', 5.0);

    % Pad-style truth inputs: stationary, body aligned with NED Z-down,
    % so quat = identity, accel_NED = 0 (gravity-free truth bus convention),
    % omega = 0. IMU should read approximately [0, 0, +1g] body (gravity).
    accel_nav   = [0, 0, 0];
    omega_body  = [0, 0, 0];
    quat_ident  = [1, 0, 0, 0];
    seed_base   = 20260519 + 1;

    fprintf('[smoke] Call 1 (cold construct)...\n');
    [a1, g1, rebuilt1] = casper_imu_lsm_step(accel_nav, omega_body, quat_ident, seed_base, false);
    assert(rebuilt1 == true, 'first call should rebuild');
    fprintf('  accel = [%.4f, %.4f, %.4f] m/s^2\n', a1(1), a1(2), a1(3));
    fprintf('  gyro  = [%.4f, %.4f, %.4f] rad/s\n', g1(1), g1(2), g1(3));
    assert(isequal(size(a1), [1 3]), 'accel must be 1x3 row');
    assert(isequal(size(g1), [1 3]), 'gyro must be 1x3 row');

    fprintf('[smoke] Call 2 (no reset)...\n');
    [a2, g2, rebuilt2] = casper_imu_lsm_step(accel_nav, omega_body, quat_ident, seed_base, false);
    assert(rebuilt2 == false, 'second call should NOT rebuild');
    fprintf('  accel = [%.4f, %.4f, %.4f] m/s^2\n', a2(1), a2(2), a2(3));

    % Now reset and re-call: the stream should restart deterministically.
    fprintf('[smoke] Call 3 (reset)...\n');
    [a3, g3, rebuilt3] = casper_imu_lsm_step(accel_nav, omega_body, quat_ident, seed_base, true);
    assert(rebuilt3 == true, 'reset should rebuild');
    fprintf('  accel = [%.4f, %.4f, %.4f] m/s^2\n', a3(1), a3(2), a3(3));
    fprintf('[smoke] post-reset Call 4 should equal Call 2 (same seed sequence)...\n');
    [a4, ~, ~] = casper_imu_lsm_step(accel_nav, omega_body, quat_ident, seed_base, false);
    fprintf('  Call 1 = [%.4f, %.4f, %.4f]\n', a1(1), a1(2), a1(3));
    fprintf('  Call 3 = [%.4f, %.4f, %.4f]\n', a3(1), a3(2), a3(3));
    fprintf('  |Call1 - Call3| max = %.3e\n', max(abs(a1(:) - a3(:))));
    fprintf('  |Call2 - Call4| max = %.3e\n', max(abs(a2(:) - a4(:))));

    determinism_ok = (max(abs(a1(:) - a3(:))) < 1e-12) && ...
                     (max(abs(a2(:) - a4(:))) < 1e-12);
    if determinism_ok
        fprintf('[smoke] PASS — determinism verified at 1e-12\n');
    else
        fprintf('[smoke] FAIL — determinism violated\n');
        error('test_imu_lsm_step_smoke:NotDeterministic', 'Reset did not restore initial stream');
    end

    fprintf('[smoke] All checks PASS.\n');
end
