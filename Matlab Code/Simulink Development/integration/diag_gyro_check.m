function diag_gyro_check()
%DIAG_GYRO_CHECK Compare visual vs legacy gyro signal.
    here    = fileparts(mfilename('fullpath'));
    simroot = fileparts(here);
    addpath(here); addpath(simroot);
    addpath(fullfile(simroot, 'shared'));
    addpath(fullfile(simroot, 'truth'));
    addpath(fullfile(simroot, 'params'));
    addpath(fullfile(simroot, 'sensors', 'imu'));
    addpath(fullfile(simroot, 'sensors', 'baro'));
    addpath(fullfile(simroot, 'sensors', 'mag'));
    addpath(fullfile(simroot, 'sensors', 'gps'));
    addpath(fullfile(simroot, 'nav', 'frame_switch'));
    addpath(fullfile(simroot, 'nav', 'eskf'));
    addpath(fullfile(simroot, 'nav', 'attitude'));
    addpath(fullfile(simroot, 'validation'));

    casper('Profile', 'apogee', 'StopTime', 6.0);
    build_casper_sim_phase0();

    mdl = 'casper_sim_phase0';
    if ~bdIsLoaded(mdl); load_system(mdl); end
    set_param(mdl, 'StopTime', '6.0');
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

    log_imu = simOut.get('log_sensor_imu');
    [t, v] = struct_with_time_(log_imu);
    fprintf('size of log_sensor_imu: %s\n', mat2str(size(v)));

    % log_sensor_imu is from IMU_LSM port 1 (accel_g std-body, 3-elem).
    % We need GYRO too. Let me check if there's a gyro log.
    fprintf('Available log signals: %s\n', strjoin(simOut.who, ', '));

    % Just to be safe, let's dump raw IMU_LSM port 1 and 2.
    % Sad -- only accel is logged. Let me also pull truth gyro.
    truth_ts = evalin('base', 'truth_ts');
    fprintf('\nTruth omega_body_std around launch:\n');
    for tm = [-1, -0.5, 0, 0.1, 0.5, 1, 2, 3, 5, 6]
        idx = find(truth_ts.omega_body_std.Time >= tm, 1, 'first');
        if isempty(idx); continue; end
        v = truth_ts.omega_body_std.Data(idx, :);
        fprintf('  t=%7.4f w=[%7.4f %7.4f %7.4f] rad/s\n', truth_ts.omega_body_std.Time(idx), v);
    end

    if bdIsLoaded(mdl); close_system(mdl, 0); end

    % --- Now directly test the imuSensor wrapper to see what gyro it returns
    fprintf('\n=== Direct imuSensor test (stationary, 10 ms, 5 sec)\n');
    cfg = casper_sim_config('Seed', 20260519, 'StopTime', 5.0);

    seed_base = cfg.Seeds.IMU;
    fprintf('IMU seed = %u\n', seed_base);

    clear casper_imu_lsm_step;
    % Pin truth: stationary nose-up
    a_nav = [0 0 0];
    w_body = [0 0 0];
    q_wxyz = [cos(pi/4) 0 sin(pi/4) 0];  % pitch=90 truth

    n = 100;
    accs = zeros(n,3); gyrs = zeros(n,3);
    for k = 1:n
        [a, g] = casper_imu_lsm_step(a_nav, w_body, q_wxyz, seed_base, k==1);
        accs(k,:) = a; gyrs(k,:) = g;
    end
    fprintf('imuSensor mean accel over 100 steps stationary:  [%7.4f %7.4f %7.4f] m/s^2\n', mean(accs,1));
    fprintf('imuSensor mean gyro  over 100 steps stationary:  [%7.4f %7.4f %7.4f] rad/s\n', mean(gyrs,1));
    fprintf('  (= [%7.4f %7.4f %7.4f] deg/s)\n', mean(gyrs,1) * 180/pi);

    % Compare to casper_imu_lsm_model
    [a_g, g_dps, ~, ~] = casper_imu_lsm_model([0;0;0], q_wxyz.', [0;0;0]);
    fprintf('\ncasper_imu_lsm_model output (no noise):\n');
    fprintf('  accel_g_body_std = [%7.4f %7.4f %7.4f] g\n', a_g);
    fprintf('  gyro_dps_body_std = [%7.4f %7.4f %7.4f] dps\n', g_dps);
end

function [t, v] = struct_with_time_(s)
    if isnumeric(s) || islogical(s); t = []; v = s; return; end
    if isfield(s, 'time') && isfield(s, 'signals')
        t = s.time(:);
        sigs = s.signals;
        cols = cell(numel(sigs), 1);
        for k = 1:numel(sigs)
            vv = sigs(k).values;
            if ndims(vv) == 3
                vv = squeeze(vv);
                if size(vv, 1) ~= numel(t) && size(vv, 2) == numel(t)
                    vv = vv.';
                end
            end
            if size(vv, 1) ~= numel(t) && size(vv, 2) == numel(t)
                vv = vv.';
            end
            cols{k} = vv;
        end
        v = horzcat(cols{:});
        return;
    end
    if isa(s, 'timeseries'); t = s.Time(:); v = s.Data; return; end
end
