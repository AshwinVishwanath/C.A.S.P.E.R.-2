function diag_pad_handoff()
%DIAG_PAD_HANDOFF Diagnose A-E root causes for the 5 s pad-prepend bug.
%
% Confirms:
%   A. From-Workspace snapshot timing vs InitFcn ordering.
%   B. To-Workspace log time range (t starts at -5 or t=0).
%   C. static_init sample-counter behavior at t<0.
%   D. Persistent state init time vs sim start time.
%   E. Sensor emissions at sim_time = -4, -2, 0.

    here    = fileparts(mfilename('fullpath'));
    simroot = fileparts(here);
    addpath(here); addpath(simroot);
    addpath(fullfile(simroot, 'shared'));

    fprintf('==== diag_pad_handoff ====\n');

    % --- Setup with apogee profile but truncate to 12 s for fast iteration ---
    casper('Profile', 'apogee', 'StopTime', 12.0);

    % Check truth_ts BEFORE build/sim — should start at 0 (no pad yet)
    truth_ts_pre = evalin('base', 'truth_ts');
    t0_pre = truth_ts_pre.pos_NED.Time(1);
    tN_pre = truth_ts_pre.pos_NED.Time(end);
    fprintf('[pre-build] truth_ts.pos_NED Time span: [%.4f, %.4f] s (n=%d)\n', ...
        t0_pre, tN_pre, numel(truth_ts_pre.pos_NED.Time));

    % Build the model (sets StartTime=-5, StopTime=12, registers InitFcn)
    build_casper_sim_phase0();

    % Check truth_ts AFTER build (build should NOT prepend pad itself)
    truth_ts_post_build = evalin('base', 'truth_ts');
    t0_pb = truth_ts_post_build.pos_NED.Time(1);
    fprintf('[post-build] truth_ts.pos_NED Time(1) = %.4f s\n', t0_pb);

    mdl = 'casper_sim_phase0';
    if ~bdIsLoaded(mdl); load_system(mdl); end

    % Confirm InitFcn and StartTime are set as expected
    init_fcn = get_param(mdl, 'InitFcn');
    start_t  = get_param(mdl, 'StartTime');
    fprintf('[mdl] StartTime="%s"  InitFcn="%s"\n', start_t, init_fcn);

    % Override StopTime to 12 s
    set_param(mdl, 'StopTime', '12');

    % Run sim
    fprintf('[sim] running...\n');
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    fprintf('[sim] done.\n');

    % Check truth_ts after sim — InitFcn should have prepended pad
    truth_ts_post_sim = evalin('base', 'truth_ts');
    t0_post = truth_ts_post_sim.pos_NED.Time(1);
    fprintf('[post-sim ] truth_ts.pos_NED Time(1) = %.4f s\n', t0_post);

    if t0_post <= -4.9
        fprintf('  >> pad WAS prepended in base WS by InitFcn (good).\n');
    else
        fprintf('  >> pad WAS NOT prepended. ROOT CAUSE = InitFcn never ran or skipped.\n');
    end

    % --- Inspect logs --------------------------------------------------------
    % Test for B: log time range
    log_state = simOut.get('log_est_state_x');
    [t_est, v_est] = struct_with_time_(log_state);
    fprintf('\nlog_est_state_x time span: [%.4f, %.4f] s (n=%d)\n', ...
        t_est(1), t_est(end), numel(t_est));
    if t_est(1) >= -1e-6
        fprintf('  >> ROOT CAUSE B candidate: log starts at t=0 (not -5).\n');
    else
        fprintf('  >> log starts before t=0, so B is NOT the issue.\n');
    end

    % Show state at key times
    fprintf('\nEKF state [alt vel ab bb] at sim times:\n');
    for tm = [-5 -4 -3 -2 -1 -0.5 -0.1 -0.01 0 0.01 0.5 1 2 3 4 5 6 8 10]
        idx = find(t_est >= tm, 1, 'first');
        if isempty(idx), continue; end
        fprintf('  t=%7.3f (idx %d): [%9.3f %9.3f %9.4f %9.4f]\n', ...
            t_est(idx), idx, v_est(idx, :));
    end

    % Test for D: persistent state init
    log_quat = simOut.get('log_est_quat');
    [t_q, v_q] = struct_with_time_(log_quat);
    fprintf('\nlog_est_quat time span: [%.4f, %.4f]\n', t_q(1), t_q(end));
    fprintf('Quat at key sim times:\n');
    for tm = [-5 -4 -3 -2 -1 -0.5 -0.01 0 0.5 1 5 10]
        idx = find(t_q >= tm, 1, 'first');
        if isempty(idx), continue; end
        q = v_q(idx, :);
        fprintf('  t=%7.3f: q=[%7.4f %7.4f %7.4f %7.4f]  |q|=%.4f\n', ...
            t_q(idx), q, norm(q));
    end

    % Test for E: sensor raw IMU emissions at t<0 vs t>0
    log_imu = simOut.get('log_sensor_imu');
    [t_imu, v_imu] = struct_with_time_(log_imu);
    fprintf('\nlog_sensor_imu time span: [%.4f, %.4f] (n=%d)\n', ...
        t_imu(1), t_imu(end), numel(t_imu));
    fprintf('IMU log at key sim times (raw std-body accel in g):\n');
    for tm = [-5 -4 -3 -2 -1 -0.5 -0.01 0 0.5 1 3 5 8]
        idx = find(t_imu >= tm, 1, 'first');
        if isempty(idx), continue; end
        fprintf('  t=%7.3f: a=[%7.4f %7.4f %7.4f]\n', t_imu(idx), v_imu(idx, :));
    end

    % Test for E continued: baro
    log_baro = simOut.get('log_sensor_baro');
    [t_b, v_b] = struct_with_time_(log_baro);
    fprintf('\nBaro at key sim times:\n');
    for tm = [-5 -4 -1 -0.01 0 1 5 8]
        idx = find(t_b >= tm, 1, 'first');
        if isempty(idx), continue; end
        fprintf('  t=%7.3f: %.4f\n', t_b(idx), v_b(idx, 1));
    end

    fprintf('\n==== diag_pad_handoff complete ====\n');
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
    t = []; v = [];
end
