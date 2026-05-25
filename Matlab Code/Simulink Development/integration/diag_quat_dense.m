function diag_quat_dense()
%DIAG_QUAT_DENSE Dense sampling of visual quat between t=5 and t=6.
    here = fileparts(mfilename('fullpath'));
    s = load(fullfile(here, 'data', 'diag_snapshot.mat'));

    fprintf('Visual quat_fw between t=5 and t=6 (50 ms granularity):\n');
    for tm = 4.9:0.05:6.5
        q = interp_quat_(s.vis_t_quat, s.vis_qfw, tm);
        fprintf('  t=%6.3f: q=[%7.4f %7.4f %7.4f %7.4f]  e=%7.3f deg\n', ...
            tm, q, 2*acosd(min(1, abs(q(1)))));
    end

    fprintf('\nLegacy quat_fw between t=5 and t=6 (50 ms granularity):\n');
    for tm = 4.9:0.05:6.5
        q = interp_quat_(s.leg_t_est, s.leg_q_fw, tm);
        fprintf('  t=%6.3f: q=[%7.4f %7.4f %7.4f %7.4f]\n', tm, q);
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
