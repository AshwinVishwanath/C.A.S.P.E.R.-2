function diag_padaccel()
%DIAG_PADACCEL  Compare what the visual model SEES at the attitude input vs legacy.
% Goal: confirm whether the body-fw accel signal at the ATTITUDE block is
% identical on both paths.

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

    % --- Build & casper setup -------------------------------------------
    casper('Profile', 'apogee', 'StopTime', 6.0);
    build_casper_sim_phase0();   % rebuild after attitude_step_helper change

    mdl = 'casper_sim_phase0';
    if ~bdIsLoaded(mdl); load_system(mdl); end
    set_param(mdl, 'StopTime', '1.0');

    % Add logging on FrameSwitch_Accel output AND on truth_ts
    % We'll use a temporary signal logging approach.

    % --- Tap log_sensor_imu (raw std-body accel from LSM block) ---------
    fprintf('  running short 1 s sim...\n');
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    fprintf('  sim done.\n');

    log_imu = simOut.get('log_sensor_imu');
    [t_imu, v_imu] = struct_with_time_(log_imu);
    fprintf('imu log shape: t=%s, v=%s\n', mat2str(size(t_imu)), mat2str(size(v_imu)));

    % Print first 5 samples
    fprintf('First 5 IMU log samples (after t=-5):\n');
    for k = 1:min(5, size(v_imu, 1))
        fprintf('  t=%7.4f: v=[', t_imu(k));
        fprintf(' %.4f', v_imu(k, :));
        fprintf(' ]\n');
    end

    % Print samples around launch (t=0)
    fprintf('IMU log samples around launch:\n');
    idx_close = find(abs(t_imu) < 0.01);
    for k = 1:numel(idx_close)
        fprintf('  t=%7.4f: v=[', t_imu(idx_close(k)));
        fprintf(' %.4f', v_imu(idx_close(k), :));
        fprintf(' ]\n');
    end

    % --- Compare to legacy --------------------------------------------
    cfg = casper_sim_config('Seed', 20260519, 'StopTime', 1.0);
    S = load(fullfile(simroot, 'truth', 'truth_trajectory.mat'));
    truth = S.truth_trajectory;
    fprintf('\nLegacy driver, PreLaunchPad=5, StopTime=1:\n');
    out_l = casper_phase0_run(cfg, truth, 'StopTime', 1.0);

    a_g_pad = out_l.Sensors.imu.accel_mps2(1:5, :) / 9.80665;
    fprintf('LEGACY first 5 IMU samples (std-body, g):\n');
    for k = 1:5
        fprintf('  t=%7.4f: a=[%.4f %.4f %.4f]\n', out_l.Sensors.imu.time_s(k), a_g_pad(k,:));
    end

    fprintf('\nLEGACY frame-switched (fw, m/s^2):\n');
    for k = 1:5
        v = out_l.Sensors.imu.accel_fw_mps2(k, :);
        fprintf('  t=%7.4f: a_fw=[%.4f %.4f %.4f]\n', out_l.Sensors.imu.time_s(k), v);
    end

    % --- truth_ts pos & accel directly ---------------------------------
    truth_ts = evalin('base', 'truth_ts');
    fprintf('\ntruth_ts.accel_NED first 5 samples:\n');
    for k = 1:5
        v = truth_ts.accel_NED.Data(k, :);
        fprintf('  t=%7.4f: accel_NED=[%.4f %.4f %.4f]\n', truth_ts.accel_NED.Time(k), v);
    end
    fprintf('truth_ts.quat_std first 5 samples:\n');
    for k = 1:5
        v = truth_ts.quat_std.Data(k, :);
        fprintf('  t=%7.4f: q=[%.4f %.4f %.4f %.4f]\n', truth_ts.quat_std.Time(k), v);
    end

    if bdIsLoaded(mdl); close_system(mdl, 0); end
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
