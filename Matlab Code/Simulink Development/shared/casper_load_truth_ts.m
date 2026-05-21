function truth_ts = casper_load_truth_ts(varargin)
%CASPER_LOAD_TRUTH_TS Populate base-workspace 'truth_ts' for the visual model.
%
% Synopsis:
%   truth_ts = casper_load_truth_ts()
%   truth_ts = casper_load_truth_ts('Regenerate', true)
%   truth_ts = casper_load_truth_ts('StopTime', 5.0)
%
% Behaviour:
%   - Looks for cached truth_trajectory.mat under truth/.
%     If missing, runs the truth build chain (rasaero_ingest → resample →
%     build_bus) to regenerate it (~30 s on a fresh machine).
%   - Builds a struct of MATLAB timeseries (one per SensorInputBus field)
%     and assigns it to base workspace as `truth_ts`.
%   - The visual model casper_sim_phase0.slx has From-Workspace blocks
%     that read truth_ts.<field>; without this call sim() errors with
%     "Variable 'truth_ts' does not exist."
%
% Inputs (name/value):
%   'Regenerate'  (default false) — force re-run of the truth build chain
%                                   even if truth_trajectory.mat exists
%   'StopTime'    (default = full trajectory) — truncate truth_ts to first
%                                   StopTime seconds (faster sim).
%
% Output:
%   truth_ts (struct of timeseries) — same struct also assigned to base.
%
% Memory note: the full 549 s trajectory at 10 kHz is ~324 MB. Pass a
% smaller StopTime (e.g. 5.0 for the pad-only smoke) to keep base WS small.

    p = inputParser();
    addParameter(p, 'Regenerate', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'StopTime',   [],    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
    parse(p, varargin{:});

    here = fileparts(mfilename('fullpath'));         % .../shared
    simroot = fileparts(here);                       % .../Simulink Development
    truth_dir = fullfile(simroot, 'truth');
    addpath(truth_dir);

    mat_path = fullfile(truth_dir, 'truth_trajectory.mat');

    if p.Results.Regenerate || ~isfile(mat_path)
        fprintf('[load_truth_ts] regenerating truth_trajectory.mat via truth build...\n');
        raw    = casper_rasaero_ingest();          %#ok<NASGU>  writes to truth dir
        truth  = casper_truth_resample();          % loads raw.mat, writes truth_trajectory.mat
    else
        fprintf('[load_truth_ts] loading cached %s\n', mat_path);
        S = load(mat_path);
        % truth_trajectory.mat top-level is 'truth_trajectory' (per T01).
        % Fall back to other common names just in case.
        if isfield(S, 'truth_trajectory')
            truth = S.truth_trajectory;
        elseif isfield(S, 'truth')
            truth = S.truth;
        else
            truth = S;
        end
    end

    % Optional truncation for short sims (the visual smoke uses 5 s only)
    if ~isempty(p.Results.StopTime)
        N = find(truth.time_s >= p.Results.StopTime, 1, 'first');
        if isempty(N)
            N = numel(truth.time_s);
        end
        fprintf('[load_truth_ts] truncating to first %d samples (%.2f s)\n', ...
            N, truth.time_s(N));
        truth = truncate_(truth, N);
    end

    t = truth.time_s(:);

    truth_ts = struct();
    truth_ts.pos_NED          = timeseries(truth.pos_NED,         t, 'Name', 'pos_NED');
    truth_ts.vel_NED          = timeseries(truth.vel_NED,         t, 'Name', 'vel_NED');
    truth_ts.accel_NED        = timeseries(truth.accel_NED,       t, 'Name', 'accel_NED');
    truth_ts.quat_std         = timeseries(truth.quat_std,        t, 'Name', 'quat_std');
    truth_ts.omega_body_std   = timeseries(truth.omega_body_std,  t, 'Name', 'omega_body_std');
    truth_ts.time_s           = timeseries(t,                     t, 'Name', 'time_s');
    truth_ts.mach             = timeseries(truth.mach(:),         t, 'Name', 'mach');
    truth_ts.air_density_kgm3 = timeseries(truth.air_density_kgm3(:), t, 'Name', 'air_density_kgm3');
    truth_ts.air_temp_K       = timeseries(truth.air_temp_K(:),   t, 'Name', 'air_temp_K');
    truth_ts.air_pressure_pa  = timeseries(truth.air_pressure_pa(:), t, 'Name', 'air_pressure_pa');

    assignin('base', 'truth_ts', truth_ts);

    fprintf('[load_truth_ts] truth_ts populated in base WS (%d samples, %.2f s span)\n', ...
        numel(t), t(end) - t(1));
end


function out = truncate_(truth, N)
    fns = fieldnames(truth);
    out = struct();
    for k = 1:numel(fns)
        f = fns{k};
        v = truth.(f);
        if isnumeric(v) && size(v, 1) >= N
            out.(f) = v(1:N, :);
        else
            out.(f) = v;  % scalars / metadata pass through
        end
    end
    if isfield(out, 'n_samples')
        out.n_samples = N;
    end
end
