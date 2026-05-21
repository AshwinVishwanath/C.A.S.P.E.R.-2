function out = generate_ekf_comparison_report(varargin)
%GENERATE_EKF_COMPARISON_REPORT  Produce side-by-side EKF (4-state vs 16-state)
% comparison plots + markdown report from a finished sim() run.
%
% Synopsis:
%   generate_ekf_comparison_report()           % uses base WS logs
%   generate_ekf_comparison_report('OutDir', path)
%   out = generate_ekf_comparison_report(...)
%
% Assumes the visual model just finished running and the following
% variables exist in the base workspace:
%   - truth_ts                         (struct of timeseries)
%   - log_est_state_x                  (4-state EKF state, StructureWithTime)
%   - log_est_quat                     (firmware attitude quat)
%   - log_est_baro_innov               (4-state baro innovation)
%   - log_est16_pos_NED                (16-state pos NED)
%   - log_est16_vel_NED                (16-state vel NED)
%   - log_est16_att_quat               (16-state attitude quat, Zup->NED)
%   - log_est16_bg, log_est16_ba, log_est16_bb  (16-state biases)
%   - log_est16_sigma_pos/vel/att      (16-state 1-sigma)
%   - log_est16_alt_up_m, log_est16_vel_up_mps, log_est16_baro_gate_on
%
% Writes (relative to this file's directory):
%   plots/ekf_comparison/altitude.png
%   plots/ekf_comparison/velocity.png
%   plots/ekf_comparison/attitude_error.png
%   plots/ekf_comparison/biases_16state.png
%   plots/ekf_comparison/baro_innovation.png
%   EKF_COMPARISON_REPORT.md

    p = inputParser();
    addParameter(p, 'OutDir', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});

    here = fileparts(mfilename('fullpath'));
    if isempty(p.Results.OutDir)
        out_dir = here;
    else
        out_dir = char(p.Results.OutDir);
    end
    plots_dir = fullfile(out_dir, 'plots', 'ekf_comparison');
    if ~isfolder(plots_dir); mkdir(plots_dir); end

    fprintf('==== generate_ekf_comparison_report ====\n');

    % MATLAB -batch mode can hang on graphics rendering with the default
    % opengl renderer. Force painters (vector renderer) for speed/reliability.
    set(0, 'DefaultFigureRenderer', 'painters');
    set(0, 'DefaultFigureVisible', 'off');

    % --- Pull data from base WS ----------------------------------------------
    [t_truth, alt_truth, vel_truth] = pull_truth_();
    [t_e4,    alt_e4,    vel_e4]     = pull_e4_();
    [t_e16,   alt_e16,   vel_e16, ...
              bg16, ba16, bb16, ...
              att_quat16, sigma_pos16, sigma_vel16, ...
              baro_gate16, baro_innov_e4] = pull_e16_();
    truth_quat = pull_truth_quat_();

    % --- Downsample to keep plots responsive ---------------------------------
    % Truth often has 850k+ samples at 10 kHz; downsample to ~5 kHz for plots.
    [t_truth, alt_truth, vel_truth] = downsample_(t_truth, alt_truth, vel_truth, 2000);
    [t_e4,    alt_e4,    vel_e4]    = downsample_(t_e4,    alt_e4,    vel_e4,    2000);
    [t_e16,   alt_e16,   vel_e16]   = downsample_(t_e16,   alt_e16,   vel_e16,   2000);

    % --- Truth apogee + cutoff window ----------------------------------------
    [apogee_truth, idx_apo_t] = max(alt_truth);
    t_apo_truth = t_truth(idx_apo_t);
    t_cutoff = t_apo_truth + 5;
    fprintf('  Truth apogee: %.1f m at t=%.2f s\n', apogee_truth, t_apo_truth);

    % --- 4-state apogee within window ----------------------------------------
    [apogee_e4, idx_apo_e4, t_apo_e4] = bounded_max_(alt_e4, t_e4, t_cutoff);
    fprintf('  4-state apogee: %.1f m at t=%.2f s\n', apogee_e4, t_apo_e4);

    % --- 16-state apogee within window ---------------------------------------
    [apogee_e16, idx_apo_e16, t_apo_e16] = bounded_max_(alt_e16, t_e16, t_cutoff);
    fprintf('  16-state apogee: %.1f m at t=%.2f s\n', apogee_e16, t_apo_e16);

    % --- Find launch time from truth (first time vel > 1 m/s up) -------------
    idx_launch_truth = find(vel_truth > 1.0, 1, 'first');
    if isempty(idx_launch_truth)
        idx_launch_truth = 1;
    end
    t_launch = t_truth(idx_launch_truth);

    % --- Metrics per EKF -----------------------------------------------------
    M_e4  = compute_metrics_(alt_e4,  vel_e4,  t_e4,  ...
                             alt_truth, vel_truth, t_truth, ...
                             apogee_truth, t_apo_truth, ...
                             apogee_e4, t_apo_e4, ...
                             t_launch, t_cutoff);
    M_e16 = compute_metrics_(alt_e16, vel_e16, t_e16, ...
                             alt_truth, vel_truth, t_truth, ...
                             apogee_truth, t_apo_truth, ...
                             apogee_e16, t_apo_e16, ...
                             t_launch, t_cutoff);

    % --- Burnout velocity error (truth burnout at first peak of vel_truth) ---
    [vel_peak_truth, idx_vpeak_truth] = max(vel_truth);
    t_burnout = t_truth(idx_vpeak_truth);
    vel_e4_at_burnout  = interp1(t_e4,  vel_e4,  t_burnout, 'linear', NaN);
    vel_e16_at_burnout = interp1(t_e16, vel_e16, t_burnout, 'linear', NaN);

    % --- Final bias estimates (16-state, end of window) ---------------------
    idx_end16 = find(t_e16 <= t_cutoff, 1, 'last');
    if isempty(idx_end16), idx_end16 = numel(t_e16); end
    bg_final = bg16(idx_end16, :);
    ba_final = ba16(idx_end16, :);
    bb_final = bb16(idx_end16);

    skip_plots = false;
    if getenv('EKF_REPORT_SKIP_PLOTS')
        skip_plots = true;
        fprintf('  EKF_REPORT_SKIP_PLOTS set, skipping plot generation.\n');
    end

    if skip_plots
        % Skip the entire plot block; jump to markdown.
    else
    try
    % --- Plot 1: triple-compare altitude --------------------------------------
    fprintf('  Plotting altitude...\n');
    fig = figure('Name', 'EKF Comparison - Altitude', 'Visible', 'off');
    plot(t_truth, alt_truth, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Truth'); hold on;
    plot(t_e4,    alt_e4,    'b-', 'LineWidth', 1.0, 'DisplayName', '4-state EKF');
    plot(t_e16,   alt_e16,   'r--','LineWidth', 1.0, 'DisplayName', '16-state EKF');
    xline(t_launch,   '--g', 'Launch');
    xline(t_apo_truth,'--m', 'Apogee');
    xlim([max(0, t_launch-2), t_cutoff]);
    xlabel('Time [s]'); ylabel('Altitude [m]');
    title('Altitude triple-compare');
    legend('Location', 'best'); grid on;
    try_apply_style_(fig);
    print(fig, fullfile(plots_dir, 'altitude.png'), '-dpng', '-r100');
    close(fig);

    % --- Plot 2: triple-compare velocity --------------------------------------
    fprintf('  Plotting velocity...\n');
    fig = figure('Name', 'EKF Comparison - Velocity', 'Visible', 'off');
    plot(t_truth, vel_truth, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Truth'); hold on;
    plot(t_e4,    vel_e4,    'b-', 'LineWidth', 1.0, 'DisplayName', '4-state EKF');
    plot(t_e16,   vel_e16,   'r--','LineWidth', 1.0, 'DisplayName', '16-state EKF');
    xline(t_launch,   '--g', 'Launch');
    xline(t_apo_truth,'--m', 'Apogee');
    xlim([max(0, t_launch-2), t_cutoff]);
    xlabel('Time [s]'); ylabel('Vertical velocity [m/s]');
    title('Vertical velocity triple-compare');
    legend('Location', 'best'); grid on;
    try_apply_style_(fig);
    print(fig, fullfile(plots_dir, 'velocity.png'), '-dpng', '-r100');
    close(fig);

    % --- Plot 3: attitude error compare ---------------------------------------
    fprintf('  Plotting att error...\n');
    [t_aterr16, atterr16] = compute_att_err_(truth_quat, t_truth, ...
                                              att_quat16, t_e16);
    fig = figure('Name', 'EKF Comparison - Attitude Error', 'Visible', 'off');
    plot(t_aterr16, atterr16, 'r-', 'LineWidth', 1.0, 'DisplayName', '16-state att err');
    hold on;
    plot(t_e4, zeros(size(t_e4)), 'b-', 'LineWidth', 0.5, ...
        'DisplayName', '4-state (no att estimate)');
    xline(t_launch,   '--g', 'Launch');
    xline(t_apo_truth,'--m', 'Apogee');
    xlim([max(0, t_launch-2), t_cutoff]);
    xlabel('Time [s]'); ylabel('Attitude error [deg]');
    title('Attitude error magnitude');
    legend('Location', 'best'); grid on;
    try_apply_style_(fig);
    print(fig, fullfile(plots_dir, 'attitude_error.png'), '-dpng', '-r100');
    close(fig);

    % --- Plot 4: 16-state bias estimates --------------------------------------
    fprintf('  Plotting biases...\n');
    % Downsample bg/ba/bb to ~2000 points for fast rendering.
    N_orig = numel(t_e16);
    if N_orig > 2500
        step = ceil(N_orig / 2500);
        bg_ds = bg16(1:step:end, :);
        ba_ds = ba16(1:step:end, :);
        bb_ds = bb16(1:step:end);
        tb_ds = t_e16(1:step:end);
    else
        bg_ds = bg16; ba_ds = ba16; bb_ds = bb16; tb_ds = t_e16;
    end
    fig = figure('Name', 'EKF Comparison - 16-state Biases', 'Visible', 'off');
    subplot(3,1,1);
    plot(tb_ds, bg_ds(:,1)*1e3, 'r-', tb_ds, bg_ds(:,2)*1e3, 'g-', tb_ds, bg_ds(:,3)*1e3, 'b-');
    ylabel('Gyro bias [mrad/s]');
    legend({'x','y','z'}, 'Location', 'best'); grid on;
    title('16-state bias estimates');
    xlim([max(0, t_launch-2), t_cutoff]);

    subplot(3,1,2);
    plot(tb_ds, ba_ds(:,1), 'r-', tb_ds, ba_ds(:,2), 'g-', tb_ds, ba_ds(:,3), 'b-');
    ylabel('Accel bias [m/s^2]');
    legend({'x','y','z'}, 'Location', 'best'); grid on;
    xlim([max(0, t_launch-2), t_cutoff]);

    subplot(3,1,3);
    plot(tb_ds, bb_ds, 'k-');
    ylabel('Baro bias [m]'); xlabel('Time [s]'); grid on;
    xlim([max(0, t_launch-2), t_cutoff]);

    try_apply_style_(fig);
    print(fig, fullfile(plots_dir, 'biases_16state.png'), '-dpng', '-r100');
    close(fig);

    % --- Plot 5: baro innovation (4-state) + 16-state gate -------------------
    fprintf('  Plotting baro innov...\n');
    % Downsample to ~2000 points.
    [tbi, bi_ds, ~] = downsample_(t_e4, baro_innov_e4, baro_innov_e4, 2000);
    if numel(baro_gate16) > 2500
        step = ceil(numel(baro_gate16) / 2500);
        bg_ds = double(baro_gate16(1:step:end));
        tbg = t_e16(1:step:end);
    else
        bg_ds = double(baro_gate16);
        tbg = t_e16;
    end
    fig = figure('Name', 'EKF Comparison - Baro innov + gate', 'Visible', 'off');
    yyaxis left;
    plot(tbi, bi_ds, 'b-');
    ylabel('4-state baro innov [m]');
    yyaxis right;
    plot(tbg, bg_ds, 'r-');
    ylabel('16-state baro gate'); ylim([-0.1, 1.5]);
    xlim([max(0, t_launch-2), t_cutoff]);
    xline(t_launch,   '--g'); xline(t_apo_truth,'--m');
    xlabel('Time [s]');
    title('Baro innovation (4-state) + Mach gate (16-state)');
    grid on;
    try_apply_style_(fig);
    print(fig, fullfile(plots_dir, 'baro_innovation.png'), '-dpng', '-r100');
    close(fig);

    fprintf('  Plots written to %s\n', plots_dir);
    catch ME_plot
        fprintf('  [WARN] plot generation failed: %s\n', ME_plot.message);
        fprintf('  Continuing to markdown report only.\n');
    end
    end  % if ~skip_plots

    % --- Markdown report -----------------------------------------------------
    fprintf('  Writing markdown report...\n');
    md_path = fullfile(out_dir, 'EKF_COMPARISON_REPORT.md');
    write_markdown_(md_path, ...
        apogee_truth, t_apo_truth, ...
        apogee_e4,    t_apo_e4,    M_e4, ...
        apogee_e16,   t_apo_e16,   M_e16, ...
        vel_peak_truth, t_burnout, ...
        vel_e4_at_burnout, vel_e16_at_burnout, ...
        bg_final, ba_final, bb_final, ...
        plots_dir);
    fprintf('  Report: %s\n', md_path);

    out = struct( ...
        'apogee_truth_m',    apogee_truth, ...
        't_apo_truth_s',     t_apo_truth, ...
        'apogee_e4_m',       apogee_e4, ...
        'apogee_e16_m',      apogee_e16, ...
        'apogee_err_e4_pct', M_e4.apogee_err_pct, ...
        'apogee_err_e16_pct',M_e16.apogee_err_pct, ...
        'alt_rms_e4_m',      M_e4.alt_rms, ...
        'alt_rms_e16_m',     M_e16.alt_rms, ...
        'vel_rms_e4_mps',    M_e4.vel_rms, ...
        'vel_rms_e16_mps',   M_e16.vel_rms, ...
        'bg_final',          bg_final, ...
        'ba_final',          ba_final, ...
        'bb_final',          bb_final, ...
        'plots_dir',         plots_dir, ...
        'report_path',       md_path);
end


% =========================================================================
function [t, alt, vel] = pull_truth_()
    truth_ts = evalin('base', 'truth_ts');
    t   = truth_ts.pos_NED.Time(:);
    pos = truth_ts.pos_NED.Data;
    velNED = truth_ts.vel_NED.Data;
    alt = -pos(:, 3);
    vel = -velNED(:, 3);
end


% =========================================================================
function qt = pull_truth_quat_()
    truth_ts = evalin('base', 'truth_ts');
    qt = struct('t', truth_ts.quat_std.Time(:), ...
                'q', truth_ts.quat_std.Data);
end


% =========================================================================
function [t, alt, vel] = pull_e4_()
% 4-state ESKF logs: x = [alt_m; vel_mps; ab; bb]
    v = fetch_log_('log_est_state_x');
    [t, data] = unpack_swt_(v);
    if isempty(data)
        t = []; alt = []; vel = [];
        return;
    end
    alt = data(:, 1);
    vel = data(:, 2);
end


% =========================================================================
function [t, alt, vel, bg, ba, bb, att, sigma_pos, sigma_vel, ...
          baro_gate, baro_innov_e4] = pull_e16_()
    alt_v  = fetch_log_('log_est16_alt_up_m');
    vel_v  = fetch_log_('log_est16_vel_up_mps');
    bg_v   = fetch_log_('log_est16_bg');
    ba_v   = fetch_log_('log_est16_ba');
    bb_v   = fetch_log_('log_est16_bb');
    att_v  = fetch_log_('log_est16_att_quat');
    sp_v   = fetch_log_('log_est16_sigma_pos');
    sv_v   = fetch_log_('log_est16_sigma_vel');
    bg_on_v= fetch_log_('log_est16_baro_gate_on');

    [t, alt_data] = unpack_swt_(alt_v);
    [~, vel_data] = unpack_swt_(vel_v);
    [~, bg]       = unpack_swt_(bg_v);
    [~, ba]       = unpack_swt_(ba_v);
    [~, bb_data]  = unpack_swt_(bb_v);
    [~, att]      = unpack_swt_(att_v);
    [~, sp]       = unpack_swt_(sp_v);
    [~, sv]       = unpack_swt_(sv_v);
    [~, bg_on]    = unpack_swt_(bg_on_v);

    alt = alt_data(:);
    vel = vel_data(:);
    bb  = bb_data(:);
    sigma_pos = sp;
    sigma_vel = sv;
    baro_gate = bg_on(:);

    % 4-state baro innov for the gate plot
    try
        bi = fetch_log_('log_est_baro_innov');
        [~, bi_data] = unpack_swt_(bi);
        baro_innov_e4 = bi_data(:);
    catch
        baro_innov_e4 = nan(size(t));
    end
end


% =========================================================================
function v = fetch_log_(name)
% FETCH_LOG_ Resolve a logged signal by name. Looks in:
%   1. Base workspace directly (set when model has ReturnWorkspaceOutputs='off').
%   2. The most-recent Simulink.SimulationOutput in base workspace
%      (default modern Simulink behavior — sim() returns simOut/ans).
% Errors with a helpful message if not found anywhere.

    % Path 1: base workspace
    if evalin('base', sprintf('exist(''%s'', ''var'')', name))
        v = evalin('base', name);
        if ~isempty(v)
            return;
        end
    end

    % Path 2: SimulationOutput candidates
    candidates = {'simOut', 'sim_out', 'out', 'ans'};
    for k = 1:numel(candidates)
        c = candidates{k};
        if ~evalin('base', sprintf('exist(''%s'', ''var'')', c))
            continue;
        end
        obj = evalin('base', c);
        if isa(obj, 'Simulink.SimulationOutput')
            members = obj.who;
            if any(strcmp(members, name))
                v = obj.(name);
                return;
            end
        end
    end

    error('fetch_log_:NotFound', ...
        ['Logged signal ''%s'' not found in base workspace nor in any\n' ...
         'Simulink.SimulationOutput (looked for: simOut, sim_out, out, ans).\n' ...
         'Run `sim(''casper_sim_phase0'')` first. If sim() returned a\n' ...
         'SimulationOutput object, unpack it via:\n' ...
         '  simOut = sim(''casper_sim_phase0'');\n' ...
         '  fns = simOut.who;\n' ...
         '  for k=1:numel(fns), assignin(''base'', fns{k}, simOut.(fns{k})); end\n' ...
         'OR re-run `build_casper_sim_phase0` after this fix (sets\n' ...
         'ReturnWorkspaceOutputs=off so sim() writes directly to base WS).'], name);
end


% =========================================================================
function [t, data] = unpack_swt_(v)
% Unpack a StructureWithTime To-Workspace variable.
    t = []; data = [];
    if isempty(v); return; end
    if isnumeric(v) || islogical(v)
        data = v; t = (0:size(v,1)-1)';
        return;
    end
    if isstruct(v) && isfield(v, 'time') && isfield(v, 'signals')
        t = v.time(:);
        sigs = v.signals;
        if numel(sigs) == 1
            data = double(sigs(1).values);
        else
            cols = cell(numel(sigs), 1);
            for k = 1:numel(sigs)
                cols{k} = double(sigs(k).values);
            end
            try
                data = horzcat(cols{:});
            catch
                data = double(sigs(1).values);
            end
        end
        if size(data, 1) ~= numel(t) && size(data, 2) == numel(t)
            data = data';
        end
        return;
    end
    if isa(v, 'timeseries')
        t = v.Time(:);
        data = double(v.Data);
        return;
    end
end


% =========================================================================
function [pk, idx, t_pk] = bounded_max_(y, t, t_cut)
    keep = t <= t_cut;
    if ~any(keep)
        keep = true(size(t));
    end
    yk = y(keep);
    tk = t(keep);
    [pk, idx] = max(yk);
    t_pk = tk(idx);
end


% =========================================================================
function M = compute_metrics_(alt_est, vel_est, t_est, ...
                              alt_truth, vel_truth, t_truth, ...
                              apo_truth, t_apo_truth, ...
                              apo_est, t_apo_est, ...
                              t_launch, t_cutoff) %#ok<INUSL>
% Compute the standard metric bundle for one EKF.
    rng_t = t_est >= t_launch & t_est <= t_cutoff;
    if ~any(rng_t)
        rng_t = true(size(t_est));
    end
    alt_t_interp = interp1(t_truth, alt_truth, t_est, 'linear', NaN);
    vel_t_interp = interp1(t_truth, vel_truth, t_est, 'linear', NaN);
    alt_err = alt_est(rng_t) - alt_t_interp(rng_t);
    vel_err = vel_est(rng_t) - vel_t_interp(rng_t);
    alt_err = alt_err(isfinite(alt_err));
    vel_err = vel_err(isfinite(vel_err));

    M = struct();
    M.apogee_err_m   = apo_est - apo_truth;
    M.apogee_err_pct = 100 * (apo_est - apo_truth) / max(abs(apo_truth), 1);
    M.t_apo_err_s    = t_apo_est - t_apo_truth;
    if isempty(alt_err)
        M.alt_rms = NaN;
    else
        M.alt_rms = sqrt(mean(alt_err.^2));
    end
    if isempty(vel_err)
        M.vel_rms = NaN;
    else
        M.vel_rms = sqrt(mean(vel_err.^2));
    end
end


% =========================================================================
function [t_out, atterr_deg] = compute_att_err_(truth_q, t_truth, ...
                                                  e16_q,    t_e16) %#ok<INUSL>
% Attitude error magnitude between truth_quat (std-body -> NED) and
% 16-state att_quat (body-Zup -> NED). Vectorized — no for loop.
    if isempty(truth_q.q) || isempty(e16_q)
        t_out = t_e16; atterr_deg = nan(size(t_e16));
        return;
    end
    % Resample truth quat to e16 timeline (vectorized; interp1 columnwise).
    qt_re = interp1(truth_q.t, truth_q.q, t_e16, 'linear', NaN);    % Nx4

    % Normalize each row, ignoring NaNs.
    nt = sqrt(sum(qt_re.^2, 2));
    ne = sqrt(sum(e16_q.^2, 2));
    nt(nt < 1e-12) = NaN;
    ne(ne < 1e-12) = NaN;
    qt_n = qt_re ./ nt;
    qe_n = e16_q ./ ne;

    % q_err = qt^-1 * qe; scalar component:
    qt_inv = [qt_n(:,1), -qt_n(:,2), -qt_n(:,3), -qt_n(:,4)];
    w = qt_inv(:,1).*qe_n(:,1) - qt_inv(:,2).*qe_n(:,2) ...
      - qt_inv(:,3).*qe_n(:,3) - qt_inv(:,4).*qe_n(:,4);
    w_abs = min(1.0, abs(w));
    atterr_deg = 2 * acos(w_abs) * (180/pi);
    t_out = t_e16;
end


% =========================================================================
function try_apply_style_(fig)
    try
        casper_plot_style(fig);
    catch
        % style helper may not be on path in tests; skip silently
    end
end


% =========================================================================
function write_markdown_(md_path, ...
        apo_t, t_apo_t, apo_e4, t_apo_e4, M_e4, ...
        apo_e16, t_apo_e16, M_e16, ...
        vpk_t, t_burn, v_e4_burn, v_e16_burn, ...
        bg_final, ba_final, bb_final, plots_dir)

    fid = fopen(md_path, 'w');
    if fid < 0
        warning('generate_ekf_comparison_report:NoMD', ...
            'Could not open %s for writing.', md_path);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));

    p2 = strrep(relpath_(plots_dir, fileparts(md_path)), '\', '/');

    fprintf(fid, '# CASPER-2 Phase 0 EKF Comparison Report\n\n');
    fprintf(fid, 'Generated by `generate_ekf_comparison_report.m` after a `sim()` of `casper_sim_phase0.slx`.\n\n');
    fprintf(fid, '## Apogee\n\n');
    fprintf(fid, '| Metric | Truth | 4-state EKF | 16-state EKF |\n');
    fprintf(fid, '|---|---:|---:|---:|\n');
    fprintf(fid, '| Apogee altitude [m] | %.1f | %.1f | %.1f |\n', apo_t, apo_e4, apo_e16);
    fprintf(fid, '| Apogee time [s]    | %.2f | %.2f | %.2f |\n', t_apo_t, t_apo_e4, t_apo_e16);
    fprintf(fid, '| Apogee err [m]     |  -   | %+.1f | %+.1f |\n', M_e4.apogee_err_m, M_e16.apogee_err_m);
    fprintf(fid, '| Apogee err [%%]     |  -   | %+.2f | %+.2f |\n', M_e4.apogee_err_pct, M_e16.apogee_err_pct);
    fprintf(fid, '| Apogee time err [s]|  -   | %+.3f | %+.3f |\n', M_e4.t_apo_err_s, M_e16.t_apo_err_s);

    fprintf(fid, '\n## Burnout (peak truth velocity)\n\n');
    fprintf(fid, '| Metric | Truth | 4-state EKF | 16-state EKF |\n');
    fprintf(fid, '|---|---:|---:|---:|\n');
    fprintf(fid, '| Burnout time [s] | %.2f | - | - |\n', t_burn);
    fprintf(fid, '| Burnout velocity [m/s] | %.1f | %.1f | %.1f |\n', vpk_t, v_e4_burn, v_e16_burn);
    fprintf(fid, '| Burnout vel err [m/s]  | -    | %+.2f | %+.2f |\n', ...
        v_e4_burn - vpk_t, v_e16_burn - vpk_t);

    fprintf(fid, '\n## RMS errors (launch to apogee + 5 s)\n\n');
    fprintf(fid, '| Metric | 4-state EKF | 16-state EKF |\n');
    fprintf(fid, '|---|---:|---:|\n');
    fprintf(fid, '| Altitude RMS [m]    | %.2f | %.2f |\n', M_e4.alt_rms, M_e16.alt_rms);
    fprintf(fid, '| Velocity RMS [m/s]  | %.2f | %.2f |\n', M_e4.vel_rms, M_e16.vel_rms);

    fprintf(fid, '\n## 16-state final bias estimates (at apogee + 5 s)\n\n');
    fprintf(fid, '*Visual model injects zero true biases — expect estimates near zero.*\n\n');
    fprintf(fid, '| Bias | x | y | z |\n');
    fprintf(fid, '|---|---:|---:|---:|\n');
    fprintf(fid, '| Gyro bias [mrad/s]  | %+.4f | %+.4f | %+.4f |\n', bg_final*1e3);
    fprintf(fid, '| Accel bias [m/s^2]  | %+.4f | %+.4f | %+.4f |\n', ba_final);
    fprintf(fid, '| Baro bias [m]       | %+.4f |       |       |\n', bb_final);

    fprintf(fid, '\n## Plots\n\n');
    fprintf(fid, '- Triple-compare altitude:        `%s/altitude.png`\n', p2);
    fprintf(fid, '- Triple-compare vertical velocity: `%s/velocity.png`\n', p2);
    fprintf(fid, '- Attitude error magnitude:        `%s/attitude_error.png`\n', p2);
    fprintf(fid, '- 16-state bias estimates:        `%s/biases_16state.png`\n', p2);
    fprintf(fid, '- Baro innovation + gate:         `%s/baro_innovation.png`\n', p2);

    fprintf(fid, '\n## How to reproduce\n\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, 'cd "Matlab Code/Simulink Development"\n');
    fprintf(fid, 'casper                                  %% apogee profile\n');
    fprintf(fid, 'sim(''casper_sim_phase0'')\n');
    fprintf(fid, 'cd integration\n');
    fprintf(fid, 'generate_ekf_comparison_report\n');
    fprintf(fid, '```\n');

    fprintf(fid, '\n## Reference\n\n');
    fprintf(fid, '- 4-state EKF: `Matlab Code/Simulink Development/nav/eskf/`\n');
    fprintf(fid, '- 16-state EKF: `Matlab Code/Simulink Development/nav/eskf16/`\n');
    fprintf(fid, '- Algorithm: `Matlab Code/EKF Dev/EKF_Symbolic_Dev.m` + `EKF16Verify.m`\n');
end


% =========================================================================
function [t_d, y1_d, y2_d] = downsample_(t, y1, y2, target_n)
% Decimate (t, y1, y2) to roughly target_n points by uniform indexing.
    N = numel(t);
    if N <= target_n
        t_d = t; y1_d = y1; y2_d = y2;
        return;
    end
    step = max(1, ceil(N / target_n));
    idx = 1:step:N;
    t_d  = t(idx);
    if isvector(y1)
        y1_d = y1(idx);
    else
        y1_d = y1(idx, :);
    end
    if isvector(y2)
        y2_d = y2(idx);
    else
        y2_d = y2(idx, :);
    end
end


% =========================================================================
function r = relpath_(target, base)
    try
        r = char(java.io.File(base).toURI().relativize(java.io.File(target).toURI()).getPath());
    catch
        r = target;
    end
end
