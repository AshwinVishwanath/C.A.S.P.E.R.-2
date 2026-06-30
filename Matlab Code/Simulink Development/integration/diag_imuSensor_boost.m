function diag_imuSensor_boost()
%DIAG_IMUSENSOR_BOOST  Test imuSensor with truth-like inputs during boost.
    here    = fileparts(mfilename('fullpath'));
    simroot = fileparts(here);
    addpath(here); addpath(simroot);
    addpath(fullfile(simroot, 'shared'));
    addpath(fullfile(simroot, 'truth'));
    addpath(fullfile(simroot, 'params'));
    addpath(fullfile(simroot, 'sensors', 'imu'));
    addpath(fullfile(simroot, 'sensors', 'baro'));

    casper_sim_config('Seed', 20260519, 'StopTime', 6.0);

    % Load truth and pick out values around boost
    S = load(fullfile(simroot, 'truth', 'truth_trajectory.mat'));
    truth = S.truth_trajectory;

    seed = 20260520;
    clear casper_imu_lsm_step

    fprintf('Testing imuSensor with truth signals around launch:\n');
    fprintf('  flight-t  | a_NED (m/s^2)        | w_body (rad/s)       | imuSensor returns\n');

    % Pick a series of truth indices
    targets = [0, 0.005, 0.01, 0.02, 0.05, 0.1, 0.3, 0.5, 1, 2, 3, 5];
    for k = 1:numel(targets)
        tt = targets(k);
        idx = find(truth.time_s >= tt, 1, 'first');
        if isempty(idx); continue; end
        a_nav = truth.accel_NED(idx, :);
        w_body = truth.omega_body_std(idx, :);
        q = truth.quat_std(idx, :);

        % Reset persistent on first call
        [a_imu, g_imu] = casper_imu_lsm_step(a_nav, w_body, q, seed, (k==1));

        fprintf('  t=%6.4f | a=[%8.3f %8.3f %8.3f] | w=[%8.4f %8.4f %8.4f] | imu_a=[%8.3f %8.3f %8.3f] imu_g=[%7.4f %7.4f %7.4f]\n', ...
            truth.time_s(idx), a_nav, w_body, a_imu, g_imu);
    end

    fprintf('\nNow same series via casper_imu_lsm_model (legacy):\n');
    for k = 1:numel(targets)
        tt = targets(k);
        idx = find(truth.time_s >= tt, 1, 'first');
        if isempty(idx); continue; end
        a_nav = truth.accel_NED(idx, :).';
        w_body = truth.omega_body_std(idx, :).';
        q = truth.quat_std(idx, :).';

        [a_g, g_dps, ~, ~] = casper_imu_lsm_model(a_nav, q, w_body);
        a_mps2 = a_g * 9.80665;
        g_radps = g_dps * pi/180;

        fprintf('  t=%6.4f | model_a=[%8.3f %8.3f %8.3f] m/s^2  model_g=[%7.4f %7.4f %7.4f] rad/s\n', ...
            truth.time_s(idx), a_mps2, g_radps);
    end
end
