function result = casper_metric_mach_gate(Truth, Estimate)
%CASPER_METRIC_MACH_GATE Verify mach gate engage/release timing.
%
% Synopsis:
%   result = casper_metric_mach_gate(Truth, Estimate)
%
% Inputs:
%   Truth    : struct with fields
%       time_s (Nx1 double)
%       mach   (Nx1 double)
%   Estimate : struct with fields
%       time_s            (Mx1 double)
%       mach_gate_active  (Mx1 logical/double; 0 or 1)
%       (optional) baro_innov_time_s, baro_innov_R - for cross checks
%       (optional) baro_R_at_update - R used per baro update, for ungate-inflate check
%
% Outputs:
%   result : MetricResult struct
%       Thresholds:
%           engage_delay_s  <= 0.25
%           release_delay_s <= 0.5
%
% Logic:
%   Truth engage: first time mach crosses 0.40 upward (>= 0.40 from below).
%   Truth release: first time mach crosses 0.35 downward after engage.
%   Est engage:  first time mach_gate_active goes high.
%   Est release: first time mach_gate_active goes low after engage.

    arguments
        Truth    struct
        Estimate struct
    end

    MACH_ON  = 0.40;
    MACH_OFF = 0.35;

    % --- Truth: find first up-crossing of MACH_ON ---
    mach = Truth.mach(:);
    t_s  = Truth.time_s(:);
    above_on = mach >= MACH_ON;
    idx_engage_truth = find(above_on, 1, 'first');
    if isempty(idx_engage_truth)
        gate_truth_engage_time = NaN;
    else
        gate_truth_engage_time = t_s(idx_engage_truth);
    end

    % First down-crossing of MACH_OFF after engage
    if isnan(gate_truth_engage_time)
        gate_truth_release_time = NaN;
    else
        below_off_after = (1:numel(mach)).' > idx_engage_truth & mach < MACH_OFF;
        ii = find(below_off_after, 1, 'first');
        if isempty(ii)
            gate_truth_release_time = NaN;
        else
            gate_truth_release_time = t_s(ii);
        end
    end

    % --- Estimate: gate state transitions ---
    gate_active = logical(Estimate.mach_gate_active(:));
    et = Estimate.time_s(:);
    rising  = find(diff([false; gate_active]) > 0, 1, 'first');
    if isempty(rising)
        gate_est_engage_time = NaN;
    else
        gate_est_engage_time = et(rising);
    end

    if isempty(rising)
        gate_est_release_time = NaN;
    else
        falling = find(diff([gate_active; false]) < 0, 1, 'first');
        if isempty(falling)
            gate_est_release_time = NaN;
        else
            gate_est_release_time = et(falling);
        end
    end

    engage_delay_s  = gate_est_engage_time  - gate_truth_engage_time;
    release_delay_s = gate_est_release_time - gate_truth_release_time;

    thr = struct('engage_delay_s', 0.25, 'release_delay_s', 0.5);

    pass_engage  = ~isnan(engage_delay_s)  && abs(engage_delay_s)  <= thr.engage_delay_s;
    pass_release = ~isnan(release_delay_s) && abs(release_delay_s) <= thr.release_delay_s;
    pass_all = pass_engage && pass_release;

    val = struct( ...
        'gate_truth_engage_time_s',  gate_truth_engage_time, ...
        'gate_truth_release_time_s', gate_truth_release_time, ...
        'gate_est_engage_time_s',    gate_est_engage_time, ...
        'gate_est_release_time_s',   gate_est_release_time, ...
        'engage_delay_s',            engage_delay_s, ...
        'release_delay_s',           release_delay_s);

    details = sprintf( ...
        'engage truth=%.3f s, est=%.3f s, dT=%.3f s (<=%.2f); release truth=%.3f s, est=%.3f s, dT=%.3f s (<=%.2f)', ...
        gate_truth_engage_time, gate_est_engage_time, engage_delay_s, thr.engage_delay_s, ...
        gate_truth_release_time, gate_est_release_time, release_delay_s, thr.release_delay_s);

    result = struct( ...
        'name',      'mach_gate', ...
        'value',     val, ...
        'threshold', thr, ...
        'pass',      pass_all, ...
        'details',   details);
end
