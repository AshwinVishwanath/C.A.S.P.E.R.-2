function diag_round1()
%DIAG_ROUND1 Drill into the visual model logs after a sim() run.
%
% Prints:
%   - truth_ts time bounds and apogee time
%   - 4-state EKF: first 10 timestamps where state_x(1) != 0 (init moment)
%   - 16-state EKF: first 10 timestamps where init_armed flips true
%   - estimates sampled at t = -5, -3, 0, 5, 10, 20, 40, 60, 80
%   - attitude init_complete first-true timestamp (via 4-state init timing
%     proxy since attitude block doesn't log)

    fprintf('\n==== diag_round1: visual model log inspection ====\n\n');

    truth_ts = evalin('base', 'truth_ts');
    t_truth  = truth_ts.pos_NED.Time(:);
    alt_truth = -truth_ts.pos_NED.Data(:,3);
    vel_truth = -truth_ts.vel_NED.Data(:,3);
    [apo, idx_apo] = max(alt_truth);
    fprintf('TRUTH:\n');
    fprintf('  t range: %.3f .. %.3f s (n=%d)\n', t_truth(1), t_truth(end), numel(t_truth));
    fprintf('  apogee = %.1f m at t=%.3f s\n', apo, t_truth(idx_apo));
    fprintf('  vel at t=0: %.2f m/s, at t=5: %.2f m/s, at t=10: %.2f m/s\n', ...
        interp1(t_truth, vel_truth, 0, 'linear', NaN), ...
        interp1(t_truth, vel_truth, 5, 'linear', NaN), ...
        interp1(t_truth, vel_truth, 10, 'linear', NaN));
    fprintf('  alt  at t=0: %.2f m, at t=5: %.2f m, at t=10: %.2f m\n', ...
        interp1(t_truth, alt_truth, 0, 'linear', NaN), ...
        interp1(t_truth, alt_truth, 5, 'linear', NaN), ...
        interp1(t_truth, alt_truth, 10, 'linear', NaN));

    % --- 4-state logs ---
    fprintf('\n4-STATE EKF (log_est_state_x):\n');
    [t4, x4] = pull_swt_('log_est_state_x');
    if isempty(t4)
        fprintf('  (no log_est_state_x found)\n');
    else
        % First sample where alt becomes non-zero
        idx_init = find(abs(x4(:,1)) > 1e-3, 1, 'first');
        if isempty(idx_init)
            fprintf('  state_x(1) never left zero!\n');
        else
            fprintf('  first non-zero alt: t=%.3f s, alt=%.3f m\n', ...
                t4(idx_init), x4(idx_init,1));
        end
        for ts = [-5, -3, 0, 1, 2, 3, 4, 5, 10, 20, 40, 60, 80]
            idx = find(t4 >= ts, 1, 'first');
            if ~isempty(idx)
                fprintf('  t=%+6.1f: alt=%.2f vel=%.2f ab=%.3f bb=%.3f\n', ...
                    ts, x4(idx,1), x4(idx,2), x4(idx,3), x4(idx,4));
            end
        end
    end

    % --- 16-state logs ---
    fprintf('\n16-STATE EKF (log_est16_alt_up_m, vel_up_mps):\n');
    [t16, alt16] = pull_swt_('log_est16_alt_up_m');
    [~,   vel16] = pull_swt_('log_est16_vel_up_mps');
    [~,   bg16]  = pull_swt_('log_est16_bg');
    [~,   ba16]  = pull_swt_('log_est16_ba');
    [~,   q16]   = pull_swt_('log_est16_att_quat');
    if isempty(t16)
        fprintf('  (no log_est16_alt_up_m found)\n');
    else
        idx_init16 = find(abs(alt16) > 1e-3, 1, 'first');
        if isempty(idx_init16)
            fprintf('  alt16 never left zero!\n');
        else
            fprintf('  first non-zero alt16: t=%.3f s, alt=%.3f m\n', ...
                t16(idx_init16), alt16(idx_init16));
        end
        % Find when alt goes maximum (early apogee)
        [pk16, idx_pk16] = max(alt16);
        fprintf('  16-state PEAK alt = %.3f m at t=%.3f s\n', pk16, t16(idx_pk16));
        for ts = [-5, -3, 0, 1, 2, 3, 4, 5, 10, 20, 26, 40, 60, 80]
            idx = find(t16 >= ts, 1, 'first');
            if ~isempty(idx) && idx <= numel(t16)
                if size(q16,1) == numel(t16)
                    q_row = q16(idx,:);
                else
                    q_row = [NaN NaN NaN NaN];
                end
                bg_row = bg16(idx,:);
                ba_row = ba16(idx,:);
                fprintf('  t=%+6.1f: alt=%.2f vel=%.2f | bg=[%.4f %.4f %.4f] ba=[%.3f %.3f %.3f] q=[%.3f %.3f %.3f %.3f]\n', ...
                    ts, alt16(idx), vel16(idx), bg_row, ba_row, q_row);
            end
        end
    end

    fprintf('\n==== diag_round1 DONE ====\n\n');
end

function [t, d] = pull_swt_(name)
    t = []; d = [];
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', name)); return; end
    v = evalin('base', name);
    if isnumeric(v) || islogical(v)
        d = v; t = (0:size(v,1)-1)';
        return;
    end
    if isstruct(v) && isfield(v, 'time') && isfield(v, 'signals')
        t = v.time(:);
        sigs = v.signals;
        if numel(sigs) == 1
            d = double(sigs(1).values);
        else
            cols = cell(numel(sigs),1);
            for k = 1:numel(sigs)
                cols{k} = double(sigs(k).values);
            end
            try
                d = horzcat(cols{:});
            catch
                d = double(sigs(1).values);
            end
        end
        if size(d,1) ~= numel(t) && size(d,2) == numel(t)
            d = d.';
        end
    elseif isa(v, 'timeseries')
        t = v.Time(:); d = double(v.Data);
    end
end
