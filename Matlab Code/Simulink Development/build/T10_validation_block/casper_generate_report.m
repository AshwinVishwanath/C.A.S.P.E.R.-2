function report_path = casper_generate_report(bundle, OutPath, RunMeta, PlotsDir)
%CASPER_GENERATE_REPORT Write PHASE0_TRUSTGATE_REPORT.md from a MetricBundle.
%
% Synopsis:
%   report_path = casper_generate_report(bundle, OutPath, RunMeta, PlotsDir)
%
% Inputs:
%   bundle   : MetricBundle from casper_compute_all_metrics
%   OutPath  : full path to the .md file to write
%   RunMeta  : struct with optional fields
%       timestamp     (datetime, default now)
%       matlab_ver    (char)
%       simulink_ver  (char)
%       seed          (numeric)
%       git_commit    (char)
%       test_id       (char)
%   PlotsDir : char path to where the plot bundle lives (linked into report).
%
% Outputs:
%   report_path : OutPath (echoed for convenience)

    arguments
        bundle   struct
        OutPath  char
        RunMeta  struct = struct()
        PlotsDir char = ''
    end

    % Fill metadata defaults
    if ~isfield(RunMeta, 'timestamp')
        RunMeta.timestamp = datetime('now','TimeZone','UTC');
    end
    if ~isfield(RunMeta, 'matlab_ver')
        RunMeta.matlab_ver = version();
    end
    if ~isfield(RunMeta, 'simulink_ver')
        try
            RunMeta.simulink_ver = simulinkversion();
        catch
            RunMeta.simulink_ver = 'unknown';
        end
    end
    if ~isfield(RunMeta, 'seed')
        RunMeta.seed = 0;
    end
    if ~isfield(RunMeta, 'git_commit')
        RunMeta.git_commit = 'unknown';
    end
    if ~isfield(RunMeta, 'test_id')
        RunMeta.test_id = 'PHASE0_trustgate';
    end

    verdict = ternary(bundle.overall_pass, 'PASS', 'FAIL');

    lines = {};
    lines{end+1} = '# Phase 0 Trust Gate Report';
    lines{end+1} = '';
    lines{end+1} = sprintf('**Verdict**: %s', verdict);
    lines{end+1} = '';
    lines{end+1} = sprintf('- **Run timestamp**: %s UTC', datestr(RunMeta.timestamp, 'yyyy-mm-ddTHH:MM:SSZ'));
    lines{end+1} = sprintf('- **MATLAB version**: %s', RunMeta.matlab_ver);
    lines{end+1} = sprintf('- **Simulink version**: %s', RunMeta.simulink_ver);
    lines{end+1} = sprintf('- **Seed**: %d', RunMeta.seed);
    lines{end+1} = sprintf('- **Git commit**: %s', RunMeta.git_commit);
    lines{end+1} = sprintf('- **Test ID**: %s', RunMeta.test_id);
    lines{end+1} = '';

    lines{end+1} = '## Headline metrics';
    lines{end+1} = '';
    lines{end+1} = '| Metric | Threshold | Actual | Status |';
    lines{end+1} = '|---|---|---|---|';
    apo = bundle.apogee.value;
    lines{end+1} = row('Apogee altitude error', '<= 10 m', ...
        sprintf('%.2f m', apo.apogee_alt_err_m), pf(apo.apogee_alt_err_m <= 10));
    lines{end+1} = row('Apogee time error', '<= 0.5 s', ...
        sprintf('%.3f s', apo.apogee_time_err_s), pf(apo.apogee_time_err_s <= 0.5));
    vel = bundle.velocity.value;
    lines{end+1} = row('Velocity at burnout', '<= 2 m/s', ...
        sprintf('%.3f m/s', vel.burnout_err_mps), pf(vel.burnout_err_mps <= 2));
    lines{end+1} = row('Velocity RMS (powered)', '<= 5 m/s', ...
        sprintf('%.3f m/s', vel.rms_powered_mps), pf(vel.rms_powered_mps <= 5));
    lines{end+1} = row('Velocity RMS (coast)', '<= 3 m/s', ...
        sprintf('%.3f m/s', vel.rms_coast_mps), pf(vel.rms_coast_mps <= 3));
    att = bundle.attitude.value;
    lines{end+1} = row('Tilt RMS (powered)', '<= 1 deg', ...
        sprintf('%.4f deg', att.rms_powered_deg), pf(att.rms_powered_deg <= 1));
    lines{end+1} = row('Tilt RMS (coast)', '<= 2 deg', ...
        sprintf('%.4f deg', att.rms_coast_deg), pf(att.rms_coast_deg <= 2));
    mg = bundle.mach_gate.value;
    if isfield(mg, 'engage_delay_s')
        lines{end+1} = row('Mach gate engage delay', '<= 0.25 s', ...
            sprintf('%.3f s', mg.engage_delay_s), pf(abs(mg.engage_delay_s) <= 0.25));
        lines{end+1} = row('Mach gate release delay', '<= 0.5 s', ...
            sprintf('%.3f s', mg.release_delay_s), pf(abs(mg.release_delay_s) <= 0.5));
    end
    bb = bundle.bias.value;
    if isstruct(bb) && isfield(bb,'final_accel_bias_mps2')
        lines{end+1} = row('Final accel bias', sprintf('<= %.3f m/s^2', bundle.bias.threshold.accel_bias_3sigma_mps2), ...
            sprintf('%.4f m/s^2', bb.final_accel_bias_mps2), ...
            pf(abs(bb.final_accel_bias_mps2) <= bundle.bias.threshold.accel_bias_3sigma_mps2));
        lines{end+1} = row('Final baro bias', sprintf('<= %.3f m', bundle.bias.threshold.baro_bias_3sigma_m), ...
            sprintf('%.4f m', bb.final_baro_bias_m), ...
            pf(abs(bb.final_baro_bias_m) <= bundle.bias.threshold.baro_bias_3sigma_m));
    end
    dt = bundle.determinism.value;
    if isstruct(dt) && isfield(dt, 'sensor_match')
        lines{end+1} = row('Determinism (two runs)', 'identical', ...
            sprintf('sensor:%s, est:%s', tf(dt.sensor_match), tf(dt.estimate_match)), ...
            pf(bundle.determinism.pass));
    else
        lines{end+1} = row('Determinism (two runs)', 'identical', 'SKIPPED', 'SKIP');
    end
    lines{end+1} = '';

    lines{end+1} = '## Detailed metrics';
    lines{end+1} = '';
    fn = fieldnames(bundle);
    for k = 1:numel(fn)
        if strcmp(fn{k}, 'overall_pass'); continue; end
        m = bundle.(fn{k});
        lines{end+1} = sprintf('### %s -- %s', m.name, pf(m.pass));
        lines{end+1} = '';
        lines{end+1} = sprintf('- Details: %s', m.details);
        lines{end+1} = '';
    end

    lines{end+1} = '## Sanity checks';
    lines{end+1} = '';
    if isstruct(bundle.sanity.value) && isfield(bundle.sanity.value, 'nan_signals')
        sv = bundle.sanity.value;
        lines{end+1} = sprintf('- NaN signals: %s', strjoin(sv.nan_signals, ', '));
        lines{end+1} = sprintf('- Inf signals: %s', strjoin(sv.inf_signals, ', '));
        lines{end+1} = sprintf('- Wall-clock runtime: %.2f s (budget %.0f s) -> %s', ...
            sv.runtime_s, sv.runtime_budget_s, pf(sv.runtime_under_budget));
    else
        lines{end+1} = '- SKIPPED (no signals supplied).';
    end
    lines{end+1} = '';

    if ~isempty(PlotsDir)
        lines{end+1} = '## Plots';
        lines{end+1} = '';
        plots = dir(fullfile(PlotsDir, '*.png'));
        for k = 1:numel(plots)
            relpath = fullfile(strrep(PlotsDir, fileparts(OutPath), '.'), plots(k).name);
            relpath = strrep(relpath, '\', '/');
            lines{end+1} = sprintf('- `%s`', plots(k).name);
        end
        lines{end+1} = '';
    end

    % Failures
    lines{end+1} = '## Failures';
    lines{end+1} = '';
    failed_any = false;
    for k = 1:numel(fn)
        if strcmp(fn{k},'overall_pass'); continue; end
        m = bundle.(fn{k});
        if ~m.pass
            failed_any = true;
            lines{end+1} = sprintf('- **%s**: %s', m.name, m.details);
        end
    end
    if ~failed_any
        lines{end+1} = '_None._';
    end
    lines{end+1} = '';

    % Deviations
    lines{end+1} = '## Deviations';
    lines{end+1} = '';
    lines{end+1} = '_None recorded by this run (T10 captures any sub-agent-injected deviations in STATUS.md)._';
    lines{end+1} = '';

    % Discord block
    discord_verdict = verdict;
    discord_lines = {};
    discord_lines{end+1} = '```';
    discord_lines{end+1} = sprintf('__phase 0 trust gate__: %s', discord_verdict);
    discord_lines{end+1} = '';
    discord_lines{end+1} = sprintf('apogee error: %.2f m', apo.apogee_alt_err_m);
    discord_lines{end+1} = sprintf('burnout vel error: %.2f m/s', vel.burnout_err_mps);
    discord_lines{end+1} = sprintf('tilt RMS, powered: %.3f deg', att.rms_powered_deg);
    discord_lines{end+1} = sprintf('mach gate behaved correctly: %s', tf(bundle.mach_gate.pass));
    if isstruct(dt) && isfield(dt, 'sensor_match')
        discord_lines{end+1} = sprintf('determinism check: %s', pf(bundle.determinism.pass));
    else
        discord_lines{end+1} = 'determinism check: SKIPPED';
    end
    discord_lines{end+1} = '';
    if bundle.overall_pass
        discord_lines{end+1} = 'next step: proceed to Phase 1.';
    else
        discord_lines{end+1} = 'next step: diagnose failing metric per PHASE0_SPEC.md S4.';
    end
    discord_lines{end+1} = '```';

    lines{end+1} = '## Discord update';
    lines{end+1} = '';
    for k = 1:numel(discord_lines)
        lines{end+1} = discord_lines{k};
    end
    lines{end+1} = '';

    % Write the file
    fid = fopen(OutPath, 'w');
    if fid < 0
        error('casper_generate_report:OpenFailed', 'Cannot write %s', OutPath);
    end
    for k = 1:numel(lines)
        fprintf(fid, '%s\n', lines{k});
    end
    fclose(fid);

    report_path = OutPath;
end

function s = pf(b)
    if b
        s = 'PASS';
    else
        s = 'FAIL';
    end
end

function s = tf(b)
    if b
        s = 'yes';
    else
        s = 'no';
    end
end

function r = row(name, thr, act, status)
    r = sprintf('| %s | %s | %s | %s |', name, thr, act, status);
end

function y = ternary(cond, a, b)
    if cond
        y = a;
    else
        y = b;
    end
end
