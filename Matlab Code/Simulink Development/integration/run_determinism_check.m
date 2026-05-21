function out = run_determinism_check(varargin)
%RUN_DETERMINISM_CHECK Two-run byte-compare for sensor + estimator streams.
%
% Synopsis:
%   out = run_determinism_check()
%   out = run_determinism_check('Seed', N, 'StopTime', T)
%
% Runs casper_phase0_run twice with identical config and verifies:
%   - sensor_streams hash identical
%   - estimate hash identical
%
% Output struct: .pass, .sensor_match, .estimate_match, .hashes (4 chars).
%
% NOTE: requires casper_phase0_run to reinit all persistent stochastic
% helpers between runs. casper_phase0_run does this via per-helper
% `clear(...)` calls inside the function so consecutive calls are byte-
% deterministic given the same seed and truth.

    p = inputParser();
    addParameter(p, 'Seed',     20260519);
    addParameter(p, 'StopTime', 30.0);    % 30 s gives enough sensor diversity
    parse(p, varargin{:});

    fprintf('==== run_determinism_check ====\n');

    cfg = casper_sim_config('Seed', p.Results.Seed, ...
                            'StopTime', p.Results.StopTime);
    here = fileparts(mfilename('fullpath'));
    truth = load_truth_();

    fprintf('  run A...\n');
    out_a = casper_phase0_run(cfg, truth, ...
        'StopTime', p.Results.StopTime, 'PreLaunchPad_s', 0);
    fprintf('  run B...\n');
    out_b = casper_phase0_run(cfg, truth, ...
        'StopTime', p.Results.StopTime, 'PreLaunchPad_s', 0);

    Determ = struct('RunA', out_a.Determ, 'RunB', out_b.Determ);
    result = casper_metric_determinism(Determ.RunA, Determ.RunB);

    out = struct();
    out.pass            = result.pass;
    out.sensor_match    = result.value.sensor_match;
    out.estimate_match  = result.value.estimate_match;
    out.sensor_hash_a   = result.value.sensor_hash_a;
    out.sensor_hash_b   = result.value.sensor_hash_b;
    out.estimate_hash_a = result.value.estimate_hash_a;
    out.estimate_hash_b = result.value.estimate_hash_b;
    out.details         = result.details;

    fprintf('  %s\n', result.details);
    fprintf('  determinism: %s (sensors=%s, estimate=%s)\n', ...
        ternary_(out.pass, 'PASS', 'FAIL'), ...
        ternary_(out.sensor_match, 'match', 'differ'), ...
        ternary_(out.estimate_match, 'match', 'differ'));

    % Persist hashes to a log
    log_path = fullfile(here, 'logs', 'determinism_check.log');
    fid = fopen(log_path, 'w');
    if fid > 0
        fprintf(fid, 'Determinism check (Seed=%u, StopTime=%.1f s)\n\n', ...
            cfg.Seed, p.Results.StopTime);
        fprintf(fid, '%s\n', result.details);
        fprintf(fid, '\nsensor_hash_a   = %s\n', char(out.sensor_hash_a));
        fprintf(fid, 'sensor_hash_b   = %s\n', char(out.sensor_hash_b));
        fprintf(fid, 'estimate_hash_a = %s\n', char(out.estimate_hash_a));
        fprintf(fid, 'estimate_hash_b = %s\n', char(out.estimate_hash_b));
        fprintf(fid, '\nOverall: %s\n', ternary_(out.pass, 'PASS', 'FAIL'));
        fclose(fid);
    end
end

function truth = load_truth_()
    here = fileparts(mfilename('fullpath'));        % .../integration
    simroot = fileparts(here);                      % .../Simulink Development
    truth_mat = fullfile(simroot, 'truth', 'truth_trajectory.mat');
    if ~isfile(truth_mat)
        error('run_determinism_check:NoTruthMat', ...
            'truth_trajectory.mat missing at %s.', truth_mat);
    end
    S = load(truth_mat, 'truth_trajectory');
    truth = S.truth_trajectory;
end

function y = ternary_(c, a, b)
    if c; y = a; else; y = b; end
end
