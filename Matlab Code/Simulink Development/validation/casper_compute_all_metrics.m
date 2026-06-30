function bundle = casper_compute_all_metrics(Truth, Estimate, Determ, Sanity)
%CASPER_COMPUTE_ALL_METRICS Aggregate all PHASE0 trust-gate metrics.
%
% Synopsis:
%   bundle = casper_compute_all_metrics(Truth, Estimate, Determ, Sanity)
%
% Inputs:
%   Truth    : struct with at least
%       time_s, alt_agl_m, vel_v_mps, accel_NED, mach, quat_fw
%   Estimate : struct with at least
%       time_s, state_x (Mx4), state_P_diag (Mx4), quat_fw (Mx4),
%       mach_gate_active (Mx1)
%   Determ   : struct of two runs (RunA, RunB) for determinism check, or [].
%              If empty, the determinism metric is set to NaN/SKIP.
%   Sanity   : struct(Signals, Meta) for sanity scan, or [].
%
% Outputs:
%   bundle : struct with fields
%       apogee, velocity, attitude, mach_gate, bias, determinism, sanity
%       overall_pass : logical (AND of all individual passes, with SKIP=true)

    arguments
        Truth    struct
        Estimate struct
        Determ        = []
        Sanity        = []
    end

    bundle = struct();

    bundle.apogee    = casper_metric_apogee(Truth, Estimate);
    bundle.velocity  = casper_metric_velocity(Truth, Estimate);
    bundle.attitude  = casper_metric_attitude(Truth, Estimate);
    bundle.mach_gate = casper_metric_mach_gate(Truth, Estimate);
    bundle.bias      = casper_metric_bias(Estimate);

    if isempty(Determ)
        bundle.determinism = make_skip_result('determinism', ...
            'No two-run data supplied; metric skipped.');
    else
        bundle.determinism = casper_metric_determinism(Determ.RunA, Determ.RunB);
    end

    if isempty(Sanity)
        bundle.sanity = make_skip_result('sanity', ...
            'No signals supplied; sanity metric skipped.');
    else
        meta = struct();
        if isfield(Sanity, 'Meta')
            meta = Sanity.Meta;
        end
        bundle.sanity = casper_metric_sanity(Sanity.Signals, meta);
    end

    passes = [ ...
        bundle.apogee.pass,    bundle.velocity.pass,    bundle.attitude.pass, ...
        bundle.mach_gate.pass, bundle.bias.pass, ...
        bundle.determinism.pass, bundle.sanity.pass];

    bundle.overall_pass = all(passes);
end

function r = make_skip_result(name, msg)
    r = struct( ...
        'name',      name, ...
        'value',     struct('status', 'SKIP'), ...
        'threshold', struct('status', 'SKIP'), ...
        'pass',      true, ...      % SKIP does not fail the gate; flagged in report
        'details',   msg);
end
