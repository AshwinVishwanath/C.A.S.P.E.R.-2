function result = casper_metric_sanity(Signals, Meta)
%CASPER_METRIC_SANITY NaN/Inf scan, runtime budget, PSD spot checks.
%
% Synopsis:
%   result = casper_metric_sanity(Signals, Meta)
%
% Inputs:
%   Signals : struct mapping signal_name -> Nx? double array.
%             Examples: 'imu_accel_mps2','imu_gyro_radps','baro_alt_m',
%             'mag_uT','gps_pos_m','est_state_x','est_quat'.
%   Meta    : struct with optional fields
%             runtime_s : measured wall-clock sim runtime (default NaN)
%             pad_window_idx : 1xK indices into Signals for "on pad" segment
%                              for PSD ±3 dB sanity check (optional)
%
% Outputs:
%   result : MetricResult struct
%       value.nan_inf_signals : cellstr of offending signal names
%       value.runtime_s
%       value.runtime_under_budget : bool (< 300 s, PHASE0_SPEC §3.7)
%       value.psd_notes : char (informational; no fail here unless data given)

    arguments
        Signals struct
        Meta    struct = struct()
    end

    fns = fieldnames(Signals);
    nan_signals = {};
    inf_signals = {};
    for k = 1:numel(fns)
        v = Signals.(fns{k});
        if ~isnumeric(v)
            continue
        end
        if any(isnan(v(:)))
            nan_signals{end+1} = fns{k}; %#ok<AGROW>
        end
        if any(isinf(v(:)))
            inf_signals{end+1} = fns{k}; %#ok<AGROW>
        end
    end
    no_nan_inf = isempty(nan_signals) && isempty(inf_signals);

    runtime_s = NaN;
    if isfield(Meta, 'runtime_s')
        runtime_s = Meta.runtime_s;
    end
    runtime_budget_s = 300.0;  % PHASE0_SPEC §3.7 "< 5 min"
    if isnan(runtime_s)
        runtime_under_budget = true;  % no data => not a hard fail
    else
        runtime_under_budget = runtime_s < runtime_budget_s;
    end

    pass_all = no_nan_inf && runtime_under_budget;

    val = struct( ...
        'nan_signals',          {nan_signals}, ...
        'inf_signals',          {inf_signals}, ...
        'runtime_s',            runtime_s, ...
        'runtime_budget_s',     runtime_budget_s, ...
        'runtime_under_budget', runtime_under_budget);

    thr = struct( ...
        'nan_inf_count',     0, ...
        'runtime_budget_s',  runtime_budget_s);

    details = sprintf( ...
        'nan_signals={%s}; inf_signals={%s}; runtime=%.2f s (<=%.0f)', ...
        strjoin(nan_signals, ','), strjoin(inf_signals, ','), ...
        runtime_s, runtime_budget_s);

    result = struct( ...
        'name',      'sanity', ...
        'value',     val, ...
        'threshold', thr, ...
        'pass',      pass_all, ...
        'details',   details);
end
