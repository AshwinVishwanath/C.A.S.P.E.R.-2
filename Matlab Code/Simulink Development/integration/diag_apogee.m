function diag_apogee()
%DIAG_APOGEE Full apogee profile run + summary, post-fix.
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

    stop_time_s = 85.0;

    % Legacy
    cfg = casper_sim_config('Seed', 20260519, 'StopTime', stop_time_s);
    S = load(fullfile(simroot, 'truth', 'truth_trajectory.mat'));
    truth = S.truth_trajectory;
    out_l = casper_phase0_run(cfg, truth, 'StopTime', stop_time_s);

    % Visual
    casper('Profile', 'apogee', 'StopTime', stop_time_s);
    build_casper_sim_phase0();
    mdl = 'casper_sim_phase0';
    if ~bdIsLoaded(mdl); load_system(mdl); end
    set_param(mdl, 'StopTime', num2str(stop_time_s));
    simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

    log_x = simOut.get('log_est_state_x');
    log_q = simOut.get('log_est_quat');
    [vt_x, vx] = struct_with_time_(log_x);
    [vt_q, vq] = struct_with_time_(log_q);

    leg_t_truth = out_l.Truth.time_s;
    leg_alt_truth = out_l.Truth.alt_m;
    leg_velv_truth = out_l.Truth.vel_v_mps;
    leg_t_est = out_l.Estimate.time_s;
    leg_x = out_l.Estimate.state_x;
    leg_q = out_l.Estimate.quat_fw;
    leg_q_truth_fw = out_l.Truth.quat_fw;

    t_marks = [10, 20, 30, 50, 70, 81, 85];
    fprintf('\nApogee profile (StopTime=%.1f s):\n', stop_time_s);
    fprintf('%-22s', 'sim-time');
    for t = t_marks; fprintf(' %10.1fs', t); end; fprintf('\n');
    fprintf('%s\n', repmat('-', 1, 22 + 11*numel(t_marks)));

    fprintf('%-22s', 'TRUTH alt [m]:');
    for t = t_marks; v = interp_(leg_t_truth, leg_alt_truth, t); fprintf(' %11.2f', v); end; fprintf('\n');
    fprintf('%-22s', 'LEGACY alt [m]:');
    for t = t_marks; v = interp_(leg_t_est, leg_x(:,1), t); fprintf(' %11.2f', v); end; fprintf('\n');
    fprintf('%-22s', 'VISUAL alt [m]:');
    for t = t_marks; v = interp_(vt_x, vx(:,1), t); fprintf(' %11.2f', v); end; fprintf('\n');

    fprintf('%-22s', 'TRUTH vel_up [m/s]:');
    for t = t_marks; v = interp_(leg_t_truth, leg_velv_truth, t); fprintf(' %11.2f', v); end; fprintf('\n');
    fprintf('%-22s', 'LEGACY vel [m/s]:');
    for t = t_marks; v = interp_(leg_t_est, leg_x(:,2), t); fprintf(' %11.2f', v); end; fprintf('\n');
    fprintf('%-22s', 'VISUAL vel [m/s]:');
    for t = t_marks; v = interp_(vt_x, vx(:,2), t); fprintf(' %11.2f', v); end; fprintf('\n');

    fprintf('%-22s', 'LEGACY err vs truth:');
    for t = t_marks
        qt = interp_quat_(out_l.Truth.time_s, leg_q_truth_fw, t);
        qe = interp_quat_(leg_t_est, leg_q, t);
        e = quat_err_deg_(qt, qe);
        fprintf(' %11.2f', e);
    end; fprintf('\n');

    fprintf('%-22s', 'VISUAL err vs truth:');
    for t = t_marks
        qt = interp_quat_(out_l.Truth.time_s, leg_q_truth_fw, t);
        qe = interp_quat_(vt_q, vq, t);
        e = quat_err_deg_(qt, qe);
        fprintf(' %11.2f', e);
    end; fprintf('\n');

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

function y = interp_(t, x, tq)
    if isempty(t) || isempty(x); y = nan; return; end
    x = double(x(:)); t = double(t(:));
    if tq < t(1); y = x(1); return; end
    if tq > t(end); y = x(end); return; end
    y = interp1(t, x, tq, 'linear');
end

function q = interp_quat_(t, Q, tq)
    t = double(t(:));
    if size(Q, 2) ~= 4 && size(Q, 1) == 4; Q = Q.'; end
    if tq < t(1); q = Q(1,:).'; q = q/norm(q); return; end
    if tq > t(end); q = Q(end,:).'; q = q/norm(q); return; end
    q1 = interp1(t, Q(:,1), tq, 'linear');
    q2 = interp1(t, Q(:,2), tq, 'linear');
    q3 = interp1(t, Q(:,3), tq, 'linear');
    q4 = interp1(t, Q(:,4), tq, 'linear');
    q = [q1; q2; q3; q4]; q = q / norm(q);
end

function e = quat_err_deg_(qt, qe)
    qt = qt(:) / norm(qt); qe = qe(:) / norm(qe);
    qt_inv = [qt(1); -qt(2); -qt(3); -qt(4)];
    aw=qt_inv(1); ax=qt_inv(2); ay=qt_inv(3); az=qt_inv(4);
    bw=qe(1); bx=qe(2); by=qe(3); bz=qe(4);
    q = [aw*bw - ax*bx - ay*by - az*bz;
         aw*bx + ax*bw + ay*bz - az*by;
         aw*by - ax*bz + ay*bw + az*bx;
         aw*bz + ax*by - ay*bx + az*bw];
    w = min(1.0, abs(q(1)));
    e = 2 * acos(w) * 180 / pi;
end
