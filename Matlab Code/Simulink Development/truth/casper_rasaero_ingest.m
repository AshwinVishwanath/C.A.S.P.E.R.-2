function raw = casper_rasaero_ingest(csv_path)
%CASPER_RASAERO_INGEST Read RasAero CSV and convert imperial->metric.
%
% Synopsis:
%   raw = casper_rasaero_ingest(csv_path)
%
% Inputs:
%   csv_path : char/string, absolute or relative path to a RasAero
%              Flight_Test.CSV (or "Flight Test.CSV") export.
%
% Outputs:
%   raw : struct with fields
%       t_s            (Nx1, seconds, non-uniform grid from CSV)
%       mach           (Nx1, dimensionless)
%       alt_m          (Nx1, positive-up meters from launch)
%       vel_v_mps      (Nx1, positive-up, vertical velocity)
%       accel_v_mps2   (Nx1, positive-up, net vertical accel, gravity-free)
%       pitch_deg      (Nx1, degrees from horizontal, +90 = vertical)
%       stage          (Nx1, string)
%       n_samples      (scalar)
%
% Notes:
%   - Looks up columns by *header name* so CSV column reorders do not break
%     this function (anti-goal #2 in T01 spec).
%   - Imperial -> metric: ft = 0.3048 m, ft/s = 0.3048 m/s, ft/s^2 = 0.3048 m/s^2.
%   - accel_v_mps2 is the *net* world-frame vertical acceleration as RasAero
%     reports it. It is gravity-free; do NOT bake gravity into this field.
%     The IMU sensor model (T03) adds gravity reaction during synthesis.
%   - Hard fail on malformed CSV (error()); does not silently drop rows.
%
% Source firmware reference:
%   None. RasAero is an external offline trajectory tool.

    arguments
        csv_path (1,:) char
    end

    if ~isfile(csv_path)
        error('casper_rasaero_ingest:FileNotFound', ...
            'CSV not found at: %s', csv_path);
    end

    % readtable preserves header names; turn on VariableNamingRule = preserve
    % so columns retain their RasAero header text (e.g. 'Time (sec)').
    opts = detectImportOptions(csv_path, 'VariableNamingRule', 'preserve');
    T = readtable(csv_path, opts);

    n_samples = height(T);
    n_cols    = width(T);

    if n_cols ~= 24
        error('casper_rasaero_ingest:BadSchema', ...
            'Expected 24 columns, got %d', n_cols);
    end
    if n_samples < 12000
        error('casper_rasaero_ingest:TooFewRows', ...
            'Expected >= 12000 rows, got %d', n_samples);
    end

    % Defensive column lookup by header name (anti-goal #2).
    col_t       = find_col(T, 'Time (sec)');
    col_stage   = find_col(T, 'Stage');
    col_mach    = find_col(T, 'Mach Number');
    col_accel_v = find_col(T, 'Accel-V (ft/sec^2)');
    col_vel_v   = find_col(T, 'Vel-V (ft/sec)');
    col_pitch   = find_col(T, 'Pitch Attitude (deg)');
    col_alt     = find_col(T, 'Altitude (ft)');

    t_s_in   = T{:, col_t};
    stage_in = string(T{:, col_stage});
    mach_in  = T{:, col_mach};

    if any(~isfinite(t_s_in)) || any(~isfinite(mach_in))
        error('casper_rasaero_ingest:MalformedRow', ...
            'Non-finite values in time or mach column.');
    end
    if abs(t_s_in(1)) > 1e-6
        error('casper_rasaero_ingest:NonZeroT0', ...
            'First time sample is %g, expected 0.', t_s_in(1));
    end

    FT_TO_M = 0.3048;  % exact conversion factor

    raw = struct();
    raw.t_s          = t_s_in(:);
    raw.mach         = mach_in(:);
    raw.alt_m        = T{:, col_alt}     * FT_TO_M;
    raw.vel_v_mps    = T{:, col_vel_v}   * FT_TO_M;
    raw.accel_v_mps2 = T{:, col_accel_v} * FT_TO_M;
    raw.pitch_deg    = T{:, col_pitch};
    raw.stage        = stage_in(:);
    raw.n_samples    = n_samples;

    % Final sanity: vectors must all match length.
    fn = {'t_s','mach','alt_m','vel_v_mps','accel_v_mps2','pitch_deg','stage'};
    for k = 1:numel(fn)
        if numel(raw.(fn{k})) ~= n_samples
            error('casper_rasaero_ingest:LengthMismatch', ...
                'Field %s has %d samples, expected %d', ...
                fn{k}, numel(raw.(fn{k})), n_samples);
        end
    end
end

function idx = find_col(T, header_name)
    names = T.Properties.VariableNames;
    idx = find(strcmp(names, header_name), 1, 'first');
    if isempty(idx)
        error('casper_rasaero_ingest:ColumnMissing', ...
            'CSV column "%s" not found. Available: %s', ...
            header_name, strjoin(names, ', '));
    end
end
