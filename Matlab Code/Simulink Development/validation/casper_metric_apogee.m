function result = casper_metric_apogee(Truth, Estimate)
%CASPER_METRIC_APOGEE Compute apogee altitude and time error.
%
% Synopsis:
%   result = casper_metric_apogee(Truth, Estimate)
%
% Inputs:
%   Truth    : struct with fields
%       time_s    (Nx1 double, seconds)
%       alt_agl_m (Nx1 double, meters AGL up)
%   Estimate : struct with fields
%       time_s    (Mx1 double, seconds)
%       state_x   (Mx4 double; column 1 = altitude_m)
%
% Outputs:
%   result : MetricResult struct
%       name      : 'apogee'
%       value     : struct(apogee_alt_err_m, apogee_time_err_s,
%                          truth_apogee_m, est_apogee_m,
%                          truth_apogee_time_s, est_apogee_time_s)
%       threshold : struct(alt_err_m=10, time_err_s=0.5)
%       pass      : bool (PHASE0_SPEC §3.1)
%       details   : string
%
% Phase 0 acceptance thresholds (PHASE0_SPEC.md §3.1):
%   apogee_alt_err  <= 10 m
%   apogee_time_err <= 0.5 s

    arguments
        Truth    struct
        Estimate struct
    end

    [truth_apogee, truth_idx] = max(Truth.alt_agl_m);
    truth_apogee_time = Truth.time_s(truth_idx);

    [est_apogee, est_idx] = max(Estimate.state_x(:, 1));
    est_apogee_time = Estimate.time_s(est_idx);

    apogee_alt_err  = abs(est_apogee - truth_apogee);
    apogee_time_err = abs(est_apogee_time - truth_apogee_time);

    thr = struct('alt_err_m', 10.0, 'time_err_s', 0.5);

    pass_alt  = apogee_alt_err  <= thr.alt_err_m;
    pass_time = apogee_time_err <= thr.time_err_s;
    pass_all  = pass_alt && pass_time;

    val = struct( ...
        'apogee_alt_err_m',     apogee_alt_err, ...
        'apogee_time_err_s',    apogee_time_err, ...
        'truth_apogee_m',       truth_apogee, ...
        'est_apogee_m',         est_apogee, ...
        'truth_apogee_time_s',  truth_apogee_time, ...
        'est_apogee_time_s',    est_apogee_time);

    details = sprintf( ...
        'truth_apogee=%.2f m @ %.3f s; est_apogee=%.2f m @ %.3f s; dAlt=%.3f m (<=%.1f), dT=%.3f s (<=%.2f)', ...
        truth_apogee, truth_apogee_time, est_apogee, est_apogee_time, ...
        apogee_alt_err, thr.alt_err_m, apogee_time_err, thr.time_err_s);

    result = struct( ...
        'name',      'apogee', ...
        'value',     val, ...
        'threshold', thr, ...
        'pass',      pass_all, ...
        'details',   details);
end
