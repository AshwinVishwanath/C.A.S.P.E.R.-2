function diag_visual_vs_legacy()
%DIAG_VISUAL_VS_LEGACY Compare legacy MATLAB driver vs visual Simulink model.
%
% Runs both paths with identical seed/StopTime and prints a side-by-side
% table of attitude / altitude / velocity at t=5,15,30 s.
%
% Standalone helper for Phase 0 visual-model debug. Not part of regression.

    fprintf('\n==== diag_visual_vs_legacy ====\n');

    % --- Setup paths ----------------------------------------------------
    here    = fileparts(mfilename('fullpath'));
    simroot = fileparts(here);
    addpath(here);
    addpath(simroot);
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

    stop_time_s = 30.0;

    % --- 1. LEGACY DRIVER RUN ------------------------------------------
    fprintf('\n--- Legacy driver path ---\n');
    cfg = casper_sim_config('Seed', 20260519, 'StopTime', stop_time_s);

    % Load truth without rate-snapping (preserve native rates for legacy)
    S = load(fullfile(simroot, 'truth', 'truth_trajectory.mat'));
    truth = S.truth_trajectory;
    if isfield(truth, 'n_samples')
        N_full = truth.n_samples;
    else
        N_full = numel(truth.time_s);
    end
    fprintf('  truth loaded: n=%d, t_max=%.2f s\n', N_full, truth.time_s(end));

    out_legacy = casper_phase0_run(cfg, truth, 'StopTime', stop_time_s);

    % --- 2. VISUAL MODEL RUN -------------------------------------------
    fprintf('\n--- Visual model path ---\n');
    casper('Profile', 'apogee', 'StopTime', stop_time_s);

    % Make sure model is built fresh
    model_name = 'casper_sim_phase0';
    model_path = fullfile(here, [model_name '.slx']);
    if ~isfile(model_path)
        build_casper_sim_phase0();
    end

    if ~bdIsLoaded(model_name); load_system(model_path); end
    set_param(model_name, 'StopTime', num2str(stop_time_s));
    fprintf('  running visual sim (StopTime=%.1f s)...\n', stop_time_s);
    simOut = sim(model_name, 'ReturnWorkspaceOutputs', 'on');
    fprintf('  visual sim complete.\n');

    % --- 3. EXTRACT TIMESERIES ------------------------------------------
    % Legacy
    leg_t_est = out_legacy.Estimate.time_s;
    leg_x     = out_legacy.Estimate.state_x;
    leg_q_fw  = out_legacy.Estimate.quat_fw;
    leg_t_truth = out_legacy.Truth.time_s;
    leg_alt_truth = out_legacy.Truth.alt_m;
    leg_velv_truth = out_legacy.Truth.vel_v_mps;

    % Visual
    log_est_state_x = simOut.get('log_est_state_x');
    log_est_quat    = simOut.get('log_est_quat');
    [vis_t_est, vis_x]    = struct_with_time_(log_est_state_x);
    [vis_t_quat, vis_qfw] = struct_with_time_(log_est_quat);
    fprintf('  vis_t_est size: %s, vis_x size: %s\n', mat2str(size(vis_t_est)), mat2str(size(vis_x)));
    fprintf('  vis_t_quat size: %s, vis_qfw size: %s\n', mat2str(size(vis_t_quat)), mat2str(size(vis_qfw)));

    % --- Save snapshot EARLY before any analysis errors ---------------
    snap = struct( ...
        'leg_t_est',   leg_t_est, ...
        'leg_x',       leg_x, ...
        'leg_q_fw',    leg_q_fw, ...
        'leg_t_truth', leg_t_truth, ...
        'leg_alt_truth', leg_alt_truth, ...
        'leg_velv_truth', leg_velv_truth, ...
        'leg_q_truth_fw', out_legacy.Truth.quat_fw, ...
        'leg_init', out_legacy.Estimate.attitude_init_complete, ...
        'vis_t_est',   vis_t_est, ...
        'vis_x',       vis_x, ...
        'vis_t_quat',  vis_t_quat, ...
        'vis_qfw',     vis_qfw);
    if ~isfolder(fullfile(here,'data')); mkdir(fullfile(here,'data')); end
    save(fullfile(here, 'data', 'diag_snapshot.mat'), '-struct', 'snap');

    % --- 4. PRINT TABLE AT t = 5, 15, 30 s -----------------------------
    t_marks = [5, 15, 30];
    fprintf('\n%-22s %12s %12s %12s\n', 'Variable', 't=5s', 't=15s', 't=30s');
    fprintf('%s\n', repmat('-', 1, 64));

    fprintf('%-22s', 'TRUTH alt [m]:');
    for tm = t_marks; v = interp_(leg_t_truth, leg_alt_truth, tm); fprintf(' %12.3f', v); end; fprintf('\n');

    fprintf('%-22s', 'LEGACY alt [m]:');
    for tm = t_marks; v = interp_(leg_t_est, leg_x(:,1), tm); fprintf(' %12.3f', v); end; fprintf('\n');

    fprintf('%-22s', 'VISUAL alt [m]:');
    for tm = t_marks; v = interp_(vis_t_est, vis_x(:,1), tm); fprintf(' %12.3f', v); end; fprintf('\n');

    fprintf('%-22s', 'TRUTH vel_up [m/s]:');
    for tm = t_marks; v = interp_(leg_t_truth, leg_velv_truth, tm); fprintf(' %12.3f', v); end; fprintf('\n');

    fprintf('%-22s', 'LEGACY vel [m/s]:');
    for tm = t_marks; v = interp_(leg_t_est, leg_x(:,2), tm); fprintf(' %12.3f', v); end; fprintf('\n');

    fprintf('%-22s', 'VISUAL vel [m/s]:');
    for tm = t_marks; v = interp_(vis_t_est, vis_x(:,2), tm); fprintf(' %12.3f', v); end; fprintf('\n');

    % --- 5. ATTITUDE ERRORS --------------------------------------------
    % LEGACY: compute |angle(q_truth_fw^-1 * q_est_fw)| in deg
    leg_q_truth_fw = out_legacy.Truth.quat_fw;
    leg_t_qfw = out_legacy.Truth.time_s;

    fprintf('\n%-22s %12s %12s %12s\n', 'Attitude error', 't=5s', 't=15s', 't=30s');
    fprintf('%s\n', repmat('-', 1, 64));

    fprintf('%-22s', 'LEGACY |err| [deg]:');
    for tm = t_marks
        qt = interp_quat_(leg_t_qfw, leg_q_truth_fw, tm);
        qe = interp_quat_(leg_t_est, leg_q_fw, tm);
        e = quat_err_deg_(qt, qe);
        fprintf(' %12.3f', e);
    end; fprintf('\n');

    % VISUAL: est quat is quat_fw. To compare to truth, need to transform
    % truth_quat_std -> truth_quat_fw via casper_frame_switch_quat first.
    % First check: compare VISUAL vs LEGACY's est_quat at sample times.
    fprintf('%-22s', 'VISUAL |err vs truth_fw| [deg]:');
    % Build truth_quat_fw at t_marks
    for tm = t_marks
        qt_fw = interp_quat_(leg_t_qfw, leg_q_truth_fw, tm);
        qe = interp_quat_(vis_t_quat, vis_qfw, tm);
        e = quat_err_deg_(qt_fw, qe);
        fprintf(' %12.3f', e);
    end; fprintf('\n');

    % Also compute against truth_quat_std (what scope currently does)
    truth_ts = evalin('base', 'truth_ts');
    fprintf('%-22s', 'VISUAL |err vs std| [deg]:');
    for tm = t_marks
        qt_std = interp_truth_ts_(truth_ts.quat_std, tm);
        qe = interp_quat_(vis_t_quat, vis_qfw, tm);
        e = quat_err_deg_(qt_std, qe);
        fprintf(' %12.3f', e);
    end; fprintf('\n');

    % --- 6. INIT TIMING ------------------------------------------------
    fprintf('\n--- Init timing ---\n');
    leg_init = out_legacy.Estimate.attitude_init_complete;
    idx_init = find(leg_init, 1, 'first');
    if isempty(idx_init)
        fprintf('  LEGACY: init NEVER COMPLETED in %.1f s\n', stop_time_s);
    else
        fprintf('  LEGACY: init complete at t = %.3f s (sample %d)\n', leg_t_est(idx_init), idx_init);
    end

    fprintf('\n  snapshot saved to data/diag_snapshot.mat\n');

    if bdIsLoaded(model_name); close_system(model_name, 0); end

    fprintf('\n==== diag_visual_vs_legacy DONE ====\n');
end

% ===================================================================
function [t, v] = struct_with_time_(s)
    if isnumeric(s) || islogical(s)
        t = []; v = s; return;
    end
    if isfield(s, 'time') && isfield(s, 'signals')
        t = s.time(:);
        % To-Workspace 'StructureWithTime' for a vector signal stores values
        % in a single signals(1).values that is Nrows x Mcols (2D) OR
        % 1 x Mcols x Nrows (3D) depending on Simulink version. Flatten:
        sigs = s.signals;
        cols = cell(numel(sigs), 1);
        for k = 1:numel(sigs)
            vv = sigs(k).values;
            if ndims(vv) == 3
                % 1 x M x N  ->  N x M
                vv = squeeze(vv);
                if size(vv, 1) == numel(t) || (size(vv, 2) ~= numel(t))
                    % already N x M, leave as is
                else
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
    if isa(s, 'timeseries')
        t = s.Time(:); v = s.Data; return;
    end
    error('unknown struct format');
end

function y = interp_(t, x, tq)
    if isempty(t) || isempty(x); y = nan; return; end
    x = double(x(:));
    t = double(t(:));
    if tq < t(1); y = x(1); return; end
    if tq > t(end); y = x(end); return; end
    y = interp1(t, x, tq, 'linear');
end

function q = interp_quat_(t, Q, tq)
    if isempty(t) || isempty(Q); q = [1;0;0;0]; return; end
    t = double(t(:));
    if size(Q, 2) ~= 4 && size(Q, 1) == 4
        Q = Q.';
    end
    if tq < t(1); q = Q(1,:).'; q = q/norm(q); return; end
    if tq > t(end); q = Q(end,:).'; q = q/norm(q); return; end
    q1 = interp1(t, Q(:,1), tq, 'linear');
    q2 = interp1(t, Q(:,2), tq, 'linear');
    q3 = interp1(t, Q(:,3), tq, 'linear');
    q4 = interp1(t, Q(:,4), tq, 'linear');
    q = [q1; q2; q3; q4];
    q = q / norm(q);
end

function q = interp_truth_ts_(ts, tq)
    t = ts.Time(:);
    D = ts.Data;   % NxM
    if tq < t(1); q = D(1,:).'; q = q/norm(q); return; end
    if tq > t(end); q = D(end,:).'; q = q/norm(q); return; end
    q = interp1(t, D, tq, 'linear').';
    q = q / norm(q);
end

function e = quat_err_deg_(qt, qe)
    qt = qt(:) / norm(qt);
    qe = qe(:) / norm(qe);
    % q_err = q_truth^-1 * q_est. Hamilton, scalar-first.
    qt_inv = [qt(1); -qt(2); -qt(3); -qt(4)];
    q = quat_mul_(qt_inv, qe);
    w = min(1.0, abs(q(1)));
    e = 2 * acos(w) * 180 / pi;
end

function qo = quat_mul_(a, b)
    aw=a(1); ax=a(2); ay=a(3); az=a(4);
    bw=b(1); bx=b(2); by=b(3); bz=b(4);
    qo = [aw*bw - ax*bx - ay*by - az*bz;
          aw*bx + ax*bw + ay*bz - az*by;
          aw*by - ax*bz + ay*bw + az*bx;
          aw*bz + ax*by - ay*bx + az*bw];
end
