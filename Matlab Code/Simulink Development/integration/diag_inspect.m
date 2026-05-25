function diag_inspect()
%DIAG_INSPECT  Inspect the diag snapshot to understand attitude divergence.
    here = fileparts(mfilename('fullpath'));
    s = load(fullfile(here, 'data', 'diag_snapshot.mat'));

    % --- Time range of visual sim --------------------------------------
    fprintf('vis_t_est range: [%.3f, %.3f] s, n=%d\n', s.vis_t_est(1), s.vis_t_est(end), numel(s.vis_t_est));
    fprintf('vis_t_quat range: [%.3f, %.3f] s, n=%d\n', s.vis_t_quat(1), s.vis_t_quat(end), numel(s.vis_t_quat));
    fprintf('leg_t_est range: [%.3f, %.3f] s, n=%d\n', s.leg_t_est(1), s.leg_t_est(end), numel(s.leg_t_est));

    % --- Visual quat trajectory at key times ---------------------------
    fprintf('\nVisual quat_fw at sim-time:\n');
    for tm = [-5 -1 0 1 2 3 4 4.5 5 6 10 15 30]
        q = interp_quat_(s.vis_t_quat, s.vis_qfw, tm);
        fprintf('  t=%6.2f: q=[%7.4f %7.4f %7.4f %7.4f]  |q|=%.4f\n', tm, q, norm(q));
    end

    fprintf('\nLegacy quat_fw at sim-time:\n');
    for tm = [-5 -1 0 1 2 3 4 4.5 5 6 10 15 30]
        q = interp_quat_(s.leg_t_est, s.leg_q_fw, tm);
        fprintf('  t=%6.2f: q=[%7.4f %7.4f %7.4f %7.4f]\n', tm, q);
    end

    fprintf('\nLegacy truth_quat_fw at sim-time:\n');
    leg_t_truth_q = s.leg_t_truth;
    for tm = [-5 -1 0 1 2 3 4 4.5 5 6 10 15 30]
        q = interp_quat_(leg_t_truth_q, s.leg_q_truth_fw, tm);
        fprintf('  t=%6.2f: q=[%7.4f %7.4f %7.4f %7.4f]\n', tm, q);
    end

    % --- Visual state x trajectory -------------------------------------
    fprintf('\nVisual state_x [alt vel ab bb] at sim-time:\n');
    for tm = [-5 -2 -1 -0.001 0 1 2 3 4 5 10 15 30]
        idx = find(s.vis_t_est >= tm, 1, 'first');
        if isempty(idx); idx = numel(s.vis_t_est); end
        fprintf('  t=%6.2f (idx %d): [%9.3f %9.3f %9.4f %9.4f]\n', s.vis_t_est(idx), idx, s.vis_x(idx,:));
    end

    fprintf('\nLegacy state_x at sim-time:\n');
    for tm = [-5 -2 -1 -0.001 0 1 2 3 4 5 10 15 30]
        idx = find(s.leg_t_est >= tm, 1, 'first');
        if isempty(idx); idx = numel(s.leg_t_est); end
        fprintf('  t=%6.2f (idx %d): [%9.3f %9.3f %9.4f %9.4f]\n', s.leg_t_est(idx), idx, s.leg_x(idx,:));
    end

    % --- attitude_init_complete trace ----------------------------------
    if isfield(s, 'leg_init') && ~isempty(s.leg_init)
        idx = find(s.leg_init, 1, 'first');
        fprintf('\nLegacy attitude init: complete at t=%.3f (sample %d)\n', s.leg_t_est(idx), idx);
    end
end

function q = interp_quat_(t, Q, tq)
    t = double(t(:));
    if size(Q, 2) ~= 4 && size(Q, 1) == 4; Q = Q.'; end
    if tq < t(1); q = Q(1,:).'; q = q/norm(q); return; end
    if tq > t(end); q = Q(end,:).'; q = q/norm(q); return; end
    q = interp1(t, Q, tq, 'linear').';
    q = q / norm(q);
end
