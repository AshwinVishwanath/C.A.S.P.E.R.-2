function result = casper_metric_bias(Estimate)
%CASPER_METRIC_BIAS Verify final bias states within 3-sigma envelopes.
%
% Synopsis:
%   result = casper_metric_bias(Estimate)
%
% Inputs:
%   Estimate : struct with fields
%       time_s           (Mx1 double, seconds)
%       state_x          (Mx4 double; col 3 = accel_bias_mps2, col 4 = baro_bias_m)
%       state_P_diag     (Mx4 double; col 4 = baro_bias P)   -- optional
%
% Outputs:
%   result : MetricResult struct
%       Thresholds:
%           |accel_bias_final| <= 3*sqrt(P0_ACCEL_BIAS) = 0.474 m/s^2
%           |baro_bias_final|  <= 3*sqrt(P0_BARO_BIAS)  = 2.598 m
%           P_history(:,4) all >= P_FLOOR_BARO_BIAS = 0.01 m^2

    arguments
        Estimate struct
    end

    P0_ACCEL_BIAS = 0.025;    % m^2/s^4 (FIRMWARE_CONSTANTS §1)
    P0_BARO_BIAS  = 0.75;     % m^2
    P_FLOOR_BARO_BIAS = 0.01; % m^2

    accel_bias_3sigma = 3 * sqrt(P0_ACCEL_BIAS);
    baro_bias_3sigma  = 3 * sqrt(P0_BARO_BIAS);

    final_accel_bias_mps2 = Estimate.state_x(end, 3);
    final_baro_bias_m     = Estimate.state_x(end, 4);

    pass_accel = abs(final_accel_bias_mps2) <= accel_bias_3sigma;
    pass_baro  = abs(final_baro_bias_m)     <= baro_bias_3sigma;

    p_floor_ok = true;
    p_floor_min = NaN;
    if isfield(Estimate, 'state_P_diag') && ~isempty(Estimate.state_P_diag)
        baro_p = Estimate.state_P_diag(:, 4);
        p_floor_min = min(baro_p);
        % Use a tiny tolerance for numerical fp comparisons
        p_floor_ok = p_floor_min >= P_FLOOR_BARO_BIAS - 1e-12;
    end

    pass_all = pass_accel && pass_baro && p_floor_ok;

    thr = struct( ...
        'accel_bias_3sigma_mps2', accel_bias_3sigma, ...
        'baro_bias_3sigma_m',     baro_bias_3sigma, ...
        'P_floor_baro_bias_m2',   P_FLOOR_BARO_BIAS);

    val = struct( ...
        'final_accel_bias_mps2', final_accel_bias_mps2, ...
        'final_baro_bias_m',     final_baro_bias_m, ...
        'p_floor_baro_bias_min_m2', p_floor_min, ...
        'p_floor_ok',            p_floor_ok);

    details = sprintf( ...
        'final ab=%.4f m/s^2 (<=%.3f), final bb=%.4f m (<=%.3f), P_floor_min=%.4f (>=%.2f)', ...
        final_accel_bias_mps2, accel_bias_3sigma, ...
        final_baro_bias_m,     baro_bias_3sigma, ...
        p_floor_min, P_FLOOR_BARO_BIAS);

    result = struct( ...
        'name',      'bias', ...
        'value',     val, ...
        'threshold', thr, ...
        'pass',      pass_all, ...
        'details',   details);
end
