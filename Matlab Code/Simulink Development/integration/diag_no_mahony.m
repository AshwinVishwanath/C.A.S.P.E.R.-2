function diag_no_mahony()
%DIAG_NO_MAHONY  Try with Mahony Kp_Grav forced to 0 to isolate the bug.
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

    stop_time_s = 10.0;
    casper('Profile', 'apogee', 'StopTime', stop_time_s);
    build_casper_sim_phase0();

    Att = evalin('base', 'Attitude');
    fprintf('Default Kp_Grav=%.3f, Kp_MagPad=%.3f, Ki=%.3f\n', ...
        Att.Kp_Grav, Att.Kp_MagPad, Att.Ki);
    Att.Kp_Grav = 0;
    Att.Ki = 0;
    assignin('base', 'Attitude', Att);
    fprintf('Setting Kp_Grav=0, Ki=0 for this run\n');

    % Force re-init of helper persistent so it picks up the new Att struct
    clear attitude_step_helper

    mdl = 'casper_sim_phase0';
    if ~bdIsLoaded(mdl); load_system(mdl); end
    set_param(mdl, 'StopTime', num2str(stop_time_s));
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

    log_quat = simOut.get('log_est_quat');
    [vt, vqfw] = struct_with_time_(log_quat);

    fprintf('\nVisual quat with Kp_Grav=0:\n');
    for tm = [-1, 0, 1, 2, 3, 4, 5, 5.5, 6, 7, 8, 10]
        idx = find(vt >= tm, 1, 'first');
        if isempty(idx); idx = numel(vt); end
        q = vqfw(idx, :); q = q / norm(q);
        fprintf('  t=%6.2f: q=[%7.4f %7.4f %7.4f %7.4f]\n', vt(idx), q);
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
                if size(vv, 1) ~= numel(t) && size(vv, 2) == numel(t); vv = vv.'; end
            end
            if size(vv, 1) ~= numel(t) && size(vv, 2) == numel(t); vv = vv.'; end
            cols{k} = vv;
        end
        v = horzcat(cols{:}); return;
    end
end
