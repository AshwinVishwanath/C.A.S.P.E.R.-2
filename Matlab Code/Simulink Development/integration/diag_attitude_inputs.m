function diag_attitude_inputs()
%DIAG_ATTITUDE_INPUTS Compare body-fw accel & gyro entering attitude estimator.
% Both legacy and visual paths should see the same signal here.

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

    stop_time_s = 6.0;

    % --- Legacy path ---------------------------------------------------
    cfg = casper_sim_config('Seed', 20260519, 'StopTime', stop_time_s);
    S = load(fullfile(simroot, 'truth', 'truth_trajectory.mat'));
    truth = S.truth_trajectory;
    out_l = casper_phase0_run(cfg, truth, 'StopTime', stop_time_s);

    leg_imu = out_l.Sensors.imu;
    leg_t = leg_imu.time_s;
    leg_a_fw = leg_imu.accel_fw_mps2;
    leg_g_fw = leg_imu.gyro_fw_radps;

    fprintf('\nLEGACY a_fw_mps2 & g_fw_radps at flight-time:\n');
    fprintf('  %-10s %-30s %-30s\n', 'sim-t', 'a_fw_mps2', 'g_fw_radps');
    for tm = [-5, -2, -0.5, 0, 0.1, 0.5, 1, 2, 3, 5, 6]
        idx = find(leg_t >= tm, 1, 'first');
        if isempty(idx); idx = numel(leg_t); end
        fprintf('  t=%7.4f a=[%7.3f %7.3f %7.3f] g=[%7.4f %7.4f %7.4f]\n', ...
            leg_t(idx), leg_a_fw(idx,:), leg_g_fw(idx,:));
    end

    % --- Visual path ---------------------------------------------------
    casper('Profile', 'apogee', 'StopTime', stop_time_s);
    build_casper_sim_phase0();

    mdl = 'casper_sim_phase0';
    if ~bdIsLoaded(mdl); load_system(mdl); end
    set_param(mdl, 'StopTime', num2str(stop_time_s));
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

    % Visual logs raw std-body IMU only. We need to recompute frame-switched.
    log_imu = simOut.get('log_sensor_imu');
    [vt, vimu] = struct_with_time_(log_imu);   % accel_g std-body
    % Re-apply unit convert (with sign flip!) and frame switch to match
    % what attitude actually sees.
    vis_a_fw = zeros(size(vimu));
    for k = 1:size(vimu, 1)
        a_std = -double(vimu(k,:)).' * 9.80665;   % FLIP + g->m/s^2
        a_fw  = [a_std(2); a_std(1); -a_std(3)];
        vis_a_fw(k, :) = a_fw.';
    end

    fprintf('\nVISUAL a_fw_mps2 (post-flip-and-frameswitch) at sim-time:\n');
    fprintf('  %-10s %-30s\n', 'sim-t', 'a_fw_mps2');
    for tm = [-5, -2, -0.5, 0, 0.1, 0.5, 1, 2, 3, 5, 6]
        idx = find(vt >= tm, 1, 'first');
        if isempty(idx); idx = numel(vt); end
        fprintf('  t=%7.4f a=[%7.3f %7.3f %7.3f]\n', vt(idx), vis_a_fw(idx,:));
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
