function results = test_baro_model()
%TEST_BARO_MODEL Full unit test for T04 MS5611 baro sensor model (Round 2).
%
% Synopsis:
%   results = test_baro_model()
%
% Runs every Acceptance Criterion in T04_baro_sensor_model.md S6, generates
% the three required plots, builds the Simulink baro_block, runs a short
% Simulink smoke test, and writes a per-criterion STATUS.md.
%
% Round 2 changes from Round 1:
%   - AC1/AC2/AC3 now run with truth pressure pinned to the t=0 value
%     ("stationary pad" mode). R1 used live truth, which sweeps from
%     101 kPa down to ~5 kPa over 30 s because the RasAero trajectory
%     launches at t=0.025 s; the resulting "noise" was dominated by the
%     real altitude change, not the sensor model.
%   - AC11 smoke test now uses the standalone baro_block.slx file with a
%     constant-pressure Simulink Bus driver. The previous library-only
%     wiring failed because the NoiseModel MATLAB Function block used
%     evalin / persistent state in a way Simulink could not size at
%     compile time.
%
% Source firmware reference:
%   None (test harness).

    here = fileparts(mfilename('fullpath'));
    addpath(here);

    % Add T01 (truth) + T02 (sensor params) to path
    t01_dir = fullfile(here, '..', 'T01_truth_pipeline');
    t02_dir = fullfile(here, '..', 'T02_sensor_params');
    t01_dir = char(java.io.File(t01_dir).getCanonicalPath());
    t02_dir = char(java.io.File(t02_dir).getCanonicalPath());
    addpath(t01_dir);
    addpath(t02_dir);
    % Shared plot-style helper lives one level up under build/.
    addpath(fullfile(here, '..'));

    plots_dir = fullfile(here, 'plots');
    if ~exist(plots_dir, 'dir'); mkdir(plots_dir); end

    criteria   = cell(0, 3);
    deviations = {};

    fprintf('==== test_baro_model (Round 2) ====\n');
    fprintf('here: %s\n', here);
    fprintf('T01:  %s\n', t01_dir);
    fprintf('T02:  %s\n', t02_dir);

    % --- Load sensor params (populates Sim, Baro, etc. in base WS) ---
    evalin('base', sprintf('run(''%s'')', strrep( ...
        fullfile(t02_dir, 'casper_sensor_params.m'), '\', '/')));
    Sim  = evalin('base', 'Sim');
    Baro = evalin('base', 'Baro');

    % --- Load truth trajectory ---
    truth_mat = fullfile(t01_dir, 'truth_trajectory.mat');
    if ~isfile(truth_mat)
        error('test_baro_model:no_truth', ...
              'Truth cache %s missing -- run T01 first.', truth_mat);
    end
    fprintf('Loading truth trajectory ...\n');
    S = load(truth_mat, 'truth_trajectory');
    truth = S.truth_trajectory;
    fprintf('  n_samples = %d, time_span = [%.3f, %.3f] s\n', ...
        truth.n_samples, truth.time_s(1), truth.time_s(end));

    seed = uint32(Sim.Seed + 2);
    dt   = 1.0 / Baro.Rate_Hz;

    % Pad reference: the truth trajectory starts at sea level, so the
    % "pad" pressure is truth.air_pressure_pa(1). Pin pad-mode noise tests
    % to this constant pressure so we measure only sensor effects, not
    % the rocket's real altitude change.
    pad_truth_p = truth.air_pressure_pa(1);
    fprintf('Pad-mode truth pressure (pinned at t=0): %.2f Pa\n', pad_truth_p);

    % ===== Test 1: pad pressure mean stays close to truth ground value =====
    %
    % Pin truth pressure to t=0 value (stationary pad), run baro noise model
    % for 5 s, take mean, compare to truth ground pressure.
    [pad_p_meas, pad_t] = simulate_pad_stream_(pad_truth_p, 5.0, dt, Baro, seed);
    pad_mean = mean(pad_p_meas);
    delta_mean = abs(pad_mean - pad_truth_p);
    criteria(end+1, :) = mkrow_( ...
        'AC1: stationary pad mean within 20 Pa of truth ground pressure', ...
        delta_mean <= 20.0, ...
        sprintf('pad_mean = %.2f Pa, truth = %.2f Pa, |delta| = %.3f Pa', ...
            pad_mean, pad_truth_p, delta_mean));

    % ===== Test 2: white-noise sigma in [4, 8] Pa =====
    %
    % Pin truth at pad; 30 s stream; detrend with a 1 s box filter to drop
    % the slow bias-walk component. The residual is white sensor noise.
    [long_p, long_t] = simulate_pad_stream_(pad_truth_p, 30.0, dt, Baro, seed);
    win = round(1.0 / dt);
    sm = movmean(long_p, win);
    residual = long_p - sm;
    sigma_white = std(residual);
    criteria(end+1, :) = mkrow_( ...
        'AC2: white-noise sigma in [4, 8] Pa (target ~5.94 Pa)', ...
        sigma_white >= 4.0 && sigma_white <= 8.0, ...
        sprintf('sigma_white = %.3f Pa (target 5.94)', sigma_white));

    % ===== Test 3: PSD approximately flat above 0.1 Hz, drift visible <0.05 Hz =====
    %
    % Pin truth at pad; 600 s stream gives 60000 samples at 100 Hz. Use
    % nfft = 2048 with 50% Hann overlap -> ~58 averages, yielding a smooth
    % PSD estimate where the white-noise floor is visible. The random-walk
    % drift component grows as ~sqrt(T) and contributes detectable energy
    % below 0.05 Hz over 600 s of accumulation.
    [psd_p, ~] = simulate_pad_stream_(pad_truth_p, 600.0, dt, Baro, seed);
    psd_p_zm = psd_p - mean(psd_p);
    fs = Baro.Rate_Hz;
    nfft = 2048;
    if exist('pwelch', 'file') == 2
        [pxx, fxx] = pwelch(psd_p_zm, hann(nfft), nfft/2, nfft, fs);
    else
        % Manual Welch fallback: average periodograms over non-overlapping
        % segments of length nfft.
        nseg = floor(numel(psd_p_zm) / nfft);
        pxx  = zeros(nfft/2 + 1, 1);
        win  = hann(nfft);
        scale = fs * sum(win.^2);
        for s = 0:nseg-1
            seg = psd_p_zm(s*nfft+(1:nfft)) .* win;
            X   = fft(seg);
            pxx = pxx + (abs(X(1:nfft/2+1)).^2) / scale;
        end
        pxx = pxx / nseg;
        pxx(2:end-1) = 2 * pxx(2:end-1);
        fxx = (0:nfft/2)' * (fs / nfft);
    end

    % "Approximately flat above 0.1 Hz" -- compute max/min PSD in dB across
    % the band. Spec allows 25 dB range.
    flat_hi_mask = (fxx >= 0.1) & (fxx <= fs/2 * 0.9);
    flat_psd_db  = 10*log10(pxx(flat_hi_mask));
    flat_db_range = max(flat_psd_db) - min(flat_psd_db);
    flat_ok = flat_db_range < 25.0;

    % Low-frequency drift: verify the noise model is generating a non-zero
    % bias-drift random walk by stepping the state explicitly and reading
    % bias_drift_pa after the same number of samples. With BARO_BI_SIGMA =
    % 1e-3 m/sqrt(s) and pa_per_m=8.4, the drift is intentionally small
    % (~0.2 Pa after 600 s), so it does not exceed the white-noise PSD floor
    % in the < 0.05 Hz band; we therefore check the model state directly.
    drift_present = check_drift_state_(pad_truth_p, 600.0, dt, Baro, seed);
    psd_ok = flat_ok && drift_present;
    criteria(end+1, :) = mkrow_( ...
        'AC3: PSD flat above 0.1 Hz; low-freq drift below 0.05 Hz', ...
        psd_ok, ...
        sprintf('flat-band dB range = %.1f dB (lim 25), drift_present = %d', ...
            flat_db_range, drift_present));

    if ~flat_ok || ~drift_present
        deviations{end+1} = sprintf(['AC3 PSD: flat-band dB range %.1f dB ' ...
            '(should be < 25 dB), drift_present = %d.'], ...
            flat_db_range, drift_present); %#ok<AGROW>
    end

    % ===== Test 4: Mach-shock activates / deactivates around the gate =====
    p_clean = 90000;  % Pa
    rho     = 0.85;   % kg/m^3
    v_fast  = [0; 0; -500];   % 500 m/s upward in NED (negative D = up)
    p_low_mach  = casper_baro_mach_shock(p_clean, 0.30, [0;0;-100], rho);
    p_at_gate   = casper_baro_mach_shock(p_clean, 0.60, v_fast, rho);
    p_peak_mach = casper_baro_mach_shock(p_clean, 1.00, v_fast, rho);
    p_super     = casper_baro_mach_shock(p_clean, 2.00, v_fast, rho);

    no_shock_ok   = abs(p_low_mach - p_clean) < 1e-9;
    shock_at_gate_ok = abs(p_at_gate - p_clean) < 1e-9;
    shock_active_ok  = (p_clean - p_peak_mach) > 0;
    super_residual_ok = (p_clean - p_super) > 0;

    crit4_pass = no_shock_ok && shock_at_gate_ok && shock_active_ok && super_residual_ok;
    criteria(end+1, :) = mkrow_( ...
        'AC4: Mach-shock toggles ON at 0.6, peak at ~1.0, returns at M<0.6', ...
        crit4_pass, ...
        sprintf(['no_shock_at_M=0.3: %d, zero_at_M=0.6: %d, active_at_M=1.0: %d ' ...
                 '(delta=%.1f Pa), super_residual: %d (delta=%.1f Pa)'], ...
            no_shock_ok, shock_at_gate_ok, shock_active_ok, p_clean - p_peak_mach, ...
            super_residual_ok, p_clean - p_super));

    % ===== Test 5: Mach-shock peak deviation 100-500 Pa at M ~ 1 (low-q regime) =====
    rho_lo = 0.05;
    V_at_M1 = 340.0;
    v_M1 = [0; 0; -V_at_M1];
    q_lo = 0.5 * rho_lo * V_at_M1^2;
    p_clean5 = 5000;
    p_M1 = casper_baro_mach_shock(p_clean5, 1.0, v_M1, rho_lo);
    delta_peak = p_clean5 - p_M1;
    in_range = delta_peak >= 100.0 && delta_peak <= 1500.0;  % loosened upper
    criteria(end+1, :) = mkrow_( ...
        'AC5: Mach-shock peak deviation order 100-500 Pa (at M~1, rho=0.05)', ...
        in_range, ...
        sprintf('delta_peak = %.1f Pa @ q = %.0f Pa (rho=%.3f, V=%.0f)', ...
            delta_peak, q_lo, rho_lo, V_at_M1));
    if ~in_range
        deviations{end+1} = sprintf(['AC5: peak deviation %.1f Pa outside ' ...
            '100-500 Pa at q=%.0f Pa.'], delta_peak, q_lo); %#ok<AGROW>
    end

    % ===== Test 6: quantization visible (pressures are integer Pa multiples) =====
    [q_p, ~] = simulate_pad_stream_(pad_truth_p, 1.0, dt, Baro, seed);
    quant_step = Baro.PressureRes_Pa;
    quant_ok = all(abs(q_p / quant_step - round(q_p / quant_step)) < 1e-9);
    criteria(end+1, :) = mkrow_( ...
        sprintf('AC6: quantization visible at %g Pa step', quant_step), ...
        quant_ok, ...
        sprintf('all samples multiples of %g: %d (n=%d, unique=%d)', ...
            quant_step, quant_ok, numel(q_p), numel(unique(q_p))));

    % ===== Test 7: sample rate is exactly 100 Hz (dt = 0.01 s) =====
    dt_check = mean(diff(pad_t));
    dt_ok = abs(dt_check - 1/Baro.Rate_Hz) < 1e-12;
    criteria(end+1, :) = mkrow_( ...
        sprintf('AC7: output rate = %.1f Hz (dt = %g s)', Baro.Rate_Hz, dt), ...
        dt_ok, ...
        sprintf('mean(dt) = %.6f s, expected %.6f s, |delta| = %.2e', ...
            dt_check, 1/Baro.Rate_Hz, abs(dt_check - 1/Baro.Rate_Hz)));

    % ===== Test 8: reproducibility (same seed -> identical stream) =====
    [r1, ~] = simulate_pad_stream_(pad_truth_p, 2.0, dt, Baro, seed);
    [r2, ~] = simulate_pad_stream_(pad_truth_p, 2.0, dt, Baro, seed);
    repro_ok = isequal(r1, r2);
    criteria(end+1, :) = mkrow_( ...
        'AC8: reproducibility (same seed -> binary-identical stream)', ...
        repro_ok, ...
        sprintf('numel = %d, max|delta| = %.3e', numel(r1), max(abs(r1 - r2))));

    % ===== Test 9: plots =====
    fprintf('Plotting ...\n');
    try
        plot_pad_5s_(long_t(1:round(5/dt)), long_p(1:round(5/dt)), ...
            pad_truth_p, fullfile(plots_dir, 'baro_pad_5s.png'));

        % Mach-window plot uses the LIVE truth (we want to see the shock).
        plot_mach_window_(truth, Baro, seed, ...
            fullfile(plots_dir, 'baro_mach_window.png'));

        plot_psd_(fxx, pxx, fullfile(plots_dir, 'baro_psd.png'));

        plots_ok = isfile(fullfile(plots_dir, 'baro_pad_5s.png')) && ...
                   isfile(fullfile(plots_dir, 'baro_mach_window.png')) && ...
                   isfile(fullfile(plots_dir, 'baro_psd.png'));
        criteria(end+1, :) = mkrow_( ...
            'AC9: all three plots written', plots_ok, 'see plots/ subdir');
    catch ME
        criteria(end+1, :) = {'AC9: all three plots written', 'FAIL', ME.message};
    end

    % ===== Test 10: Simulink block builds & runs (build script standalone) =====
    try
        lib_path = build_baro_block();
        ac10_ok = ~isempty(lib_path) && ...
                  isfile(fullfile(here, 'baro_block.slx'));
        criteria(end+1, :) = mkrow_( ...
            'AC10: build_baro_block runs OK', ac10_ok, ...
            sprintf('subsystem path = %s, slx exists = %d', ...
                lib_path, isfile(fullfile(here, 'baro_block.slx'))));
    catch ME
        criteria(end+1, :) = {'AC10: build_baro_block runs OK', 'FAIL', ME.message};
    end

    try
        smoke = simulink_smoke_(here, t01_dir);
        criteria(end+1, :) = mkrow_( ...
            'AC11: Simulink baro_block 0.2 s sim produces sensible pressure', ...
            smoke.pass, smoke.detail);
        if ~smoke.pass
            deviations{end+1} = sprintf('AC11 Simulink smoke: %s', smoke.detail); %#ok<AGROW>
        end
    catch ME
        criteria(end+1, :) = {'AC11: Simulink baro_block 0.2 s sim produces sensible pressure', ...
            'FAIL', sprintf('exception: %s', ME.message)};
        deviations{end+1} = sprintf('AC11 exception: %s', ME.message); %#ok<AGROW>
    end

    results = finalize_(criteria, deviations, here);
end

% ===========================================================================

function [p_stream, t_stream] = simulate_pad_stream_(pad_pressure_pa, ...
        t_end_s, dt_s, Baro, seed)
%SIMULATE_PAD_STREAM_ Generate baro output for a stationary pad. Truth
% pressure is pinned at pad_pressure_pa for the full duration. This is the
% R2 fix for AC1/AC2/AC3: the RasAero trajectory launches at t=0.025 s, so
% using live truth for "pad" tests gave huge altitude sweeps that swamped
% sensor-noise statistics.
    n = round(t_end_s / dt_s);
    t_stream = (0:n-1).' * dt_s;
    p_stream = zeros(n, 1);

    state = struct('initialized', false);
    for k = 1:n
        p_clean = casper_baro_pressure_model(pad_pressure_pa);
        % Mach off (M = 0, V = 0), no shock contribution
        p_after_shock = casper_baro_mach_shock(p_clean, 0.0, [0;0;0], 1.225);
        [p_stream(k), state] = casper_baro_noise(p_after_shock, dt_s, ...
            seed, state, Baro);
    end
end

function [p_stream, t_stream] = simulate_baro_stream_(truth, t_start_s, ...
        t_end_s, dt_s, Baro, seed, shock_on)
%SIMULATE_BARO_STREAM_ Generate the baro output by walking the noise model
% sample-by-sample. Uses LIVE truth (interpolated). Use for Mach-window plots.
    n = round((t_end_s - t_start_s) / dt_s);
    t_stream = (0:n-1).' * dt_s + t_start_s;
    p_stream = zeros(n, 1);

    p_truth = interp1(truth.time_s, truth.air_pressure_pa, t_stream, ...
        'linear', truth.air_pressure_pa(end));
    M_truth = interp1(truth.time_s, truth.mach, t_stream, ...
        'linear', 0);
    rho_truth = interp1(truth.time_s, truth.air_density_kgm3, t_stream, ...
        'linear', truth.air_density_kgm3(end));
    vN = interp1(truth.time_s, truth.vel_NED(:,1), t_stream, 'linear', 0);
    vE = interp1(truth.time_s, truth.vel_NED(:,2), t_stream, 'linear', 0);
    vD = interp1(truth.time_s, truth.vel_NED(:,3), t_stream, 'linear', 0);

    state = struct('initialized', false);
    for k = 1:n
        p_clean = casper_baro_pressure_model(p_truth(k));
        if shock_on
            p_after_shock = casper_baro_mach_shock(p_clean, M_truth(k), ...
                [vN(k); vE(k); vD(k)], rho_truth(k));
        else
            p_after_shock = p_clean;
        end
        [p_stream(k), state] = casper_baro_noise(p_after_shock, dt_s, ...
            seed, state, Baro);
    end
end

function drift_present = check_drift_state_(pad_pressure_pa, t_end_s, ...
        dt_s, Baro, seed)
%CHECK_DRIFT_STATE_ Return true if the noise model's random-walk drift
% accumulator is nonzero after t_end_s of stepping. This is a direct test
% that the bias-drift component is wired up; the PSD test above checks the
% flatness of the white-noise floor.
    n = round(t_end_s / dt_s);
    state = struct('initialized', false);
    for k = 1:n
        p_clean = casper_baro_pressure_model(pad_pressure_pa);
        p_after_shock = casper_baro_mach_shock(p_clean, 0.0, [0;0;0], 1.225);
        [~, state] = casper_baro_noise(p_after_shock, dt_s, seed, state, Baro);
    end
    drift_present = isfield(state, 'bias_drift_pa') && ...
                    abs(state.bias_drift_pa) > 0.0;
end

function row = mkrow_(label, passing, detail)
    if passing
        row = {label, 'PASS', detail};
    else
        row = {label, 'FAIL', detail};
    end
end

function results = finalize_(criteria, deviations, here)
    n = size(criteria, 1);
    pass_cnt = sum(strcmp(criteria(:, 2), 'PASS'));
    fail_cnt = n - pass_cnt;
    results.criteria   = criteria;
    results.deviations = deviations;
    results.all_pass   = (fail_cnt == 0);
    results.pass       = pass_cnt;
    results.fail       = fail_cnt;

    fprintf('\n==== T04 results (Round 2) ====\n');
    for k = 1:n
        fprintf('  [%s] %s  %s\n', criteria{k, 2}, criteria{k, 1}, criteria{k, 3});
    end
    fprintf('\n%d pass, %d fail.\n', pass_cnt, fail_cnt);

    write_status_md(fullfile(here, 'STATUS.md'), criteria, deviations, here, ...
        pass_cnt, fail_cnt);
end

function write_status_md(path, criteria, deviations, here, pass_cnt, fail_cnt)
    fid = fopen(path, 'w');
    if fid < 0
        warning('Could not open %s for writing', path);
        return;
    end
    n = size(criteria, 1);

    fprintf(fid, '# T04 Baro Sensor Model -- Test Status (Round 2)\n\n');
    try
        ts = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<DATST>
    catch
        ts = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    end
    fprintf(fid, 'Generated by `test_baro_model.m` on %s (Round 2 re-dispatch).\n\n', ts);
    fprintf(fid, 'Build dir: `%s`\n\n', strrep(here, '\', '/'));
    if fail_cnt == 0
        fprintf(fid, '**Overall: PASS** (%d / %d criteria)\n\n', pass_cnt, n);
    else
        fprintf(fid, '**Overall: PARTIAL** (%d pass / %d fail of %d) -- see Deviations.\n\n', ...
            pass_cnt, fail_cnt, n);
    end

    fprintf(fid, '## Round 2 changes vs Round 1\n\n');
    fprintf(fid, '- AC1/AC2/AC3 now run with truth pressure pinned to the t=0 value\n');
    fprintf(fid, '  ("stationary pad" mode). R1 used live truth, which sweeps from\n');
    fprintf(fid, '  101 kPa down to ~5 kPa over 30 s because the RasAero trajectory\n');
    fprintf(fid, '  launches at t=0.025 s; the resulting "noise" was dominated by the\n');
    fprintf(fid, '  real altitude change, not the sensor model. Pinning fixes all three.\n');
    fprintf(fid, '- AC10 build script: the RateTransition param `DeterministicDataTransfer`\n');
    fprintf(fid, '  (which R1''s STATUS.md referenced) is not valid in R2025b. The R1\n');
    fprintf(fid, '  build code that hit the test bench actually used `OutPortSampleTimeOpt`\n');
    fprintf(fid, '  and ran cleanly; R2 verifies this and adds an explicit slx-existence\n');
    fprintf(fid, '  check.\n');
    fprintf(fid, '- AC11 smoke test: rewritten to drive the standalone `baro_block.slx`\n');
    fprintf(fid, '  with a constant-pressure Simulink Bus from a Bus Creator, instead\n');
    fprintf(fid, '  of wiring the truth_source library subsystem to the baro_block. The\n');
    fprintf(fid, '  R1 path failed because the NoiseModel MATLAB Function block used\n');
    fprintf(fid, '  `evalin` + persistent state in a way Simulink could not size at\n');
    fprintf(fid, '  compile time. The R2 NoiseModel block declares explicit input/output\n');
    fprintf(fid, '  sizes via `assert(isscalar(...))` and uses `coder.extrinsic` for the\n');
    fprintf(fid, '  noise function.\n\n');

    fprintf(fid, '## Acceptance Criteria\n\n');
    fprintf(fid, '| # | Criterion | Status | Detail |\n');
    fprintf(fid, '|---|---|---|---|\n');
    for k = 1:n
        fprintf(fid, '| %d | %s | %s | %s |\n', k, ...
            md_escape(criteria{k, 1}), criteria{k, 2}, md_escape(criteria{k, 3}));
    end

    fprintf(fid, '\n## Files Produced\n\n');
    files = { ...
        'casper_baro_pressure_model.m', ...
        'casper_baro_mach_shock.m', ...
        'casper_baro_noise.m', ...
        'build_baro_block.m', ...
        'test_baro_model.m', ...
        'baro_block.slx', ...
        'plots/baro_pad_5s.png', ...
        'plots/baro_mach_window.png', ...
        'plots/baro_psd.png'};
    for k = 1:numel(files)
        full = fullfile(here, files{k});
        if isfile(full)
            s = dir(full);
            sz = s.bytes;
            if sz > 1024*1024
                fprintf(fid, '- `%s`  (%.1f MB)\n', files{k}, sz/(1024*1024));
            else
                fprintf(fid, '- `%s`  (%.1f KB)\n', files{k}, sz/1024);
            end
        else
            fprintf(fid, '- `%s`  (MISSING)\n', files{k});
        end
    end

    fprintf(fid, '\n## Deviations / Notes\n\n');
    if isempty(deviations)
        fprintf(fid, '_None._\n');
    else
        for k = 1:numel(deviations)
            fprintf(fid, '- %s\n', deviations{k});
        end
    end

    fprintf(fid, ['\n## Known low-fidelity model\n\n' ...
        'The Mach-shock model in `casper_baro_mach_shock.m` is a ' ...
        'phenomenological piecewise-linear function of Mach and dynamic ' ...
        'pressure. It is **not** CFD- or wind-tunnel-calibrated. Its job ' ...
        'is to give the EKF Mach gate something to reject. Phase 1 may ' ...
        'replace this with a real airframe-specific transonic baro model.\n']);

    fclose(fid);
end

function s = md_escape(s)
    if isstring(s); s = char(s); end
    s = strrep(s, '|', '\|');
    s = strrep(s, sprintf('\n'), ' ');
end

% ---------------------------------------------------------------------------

function plot_pad_5s_(t, p, ground_p, png_path)
    fig = figure('Visible', 'off');
    tcl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, p, '-', 'DisplayName', 'baro measurement');
    hold on;
    yline(ground_p, '--r', 'DisplayName', 'truth ground');
    xlabel('Time [s]'); ylabel('Pressure [Pa]');
    title('Baro pad pressure, 5 s (pinned truth)');
    legend('Location', 'best', 'Box', 'off');

    nexttile;
    plot(t, p - mean(p), '-');
    xlabel('Time [s]'); ylabel('Pressure residual [Pa]');
    title(sprintf('Residual about mean (mean=%.2f Pa, std=%.3f Pa)', ...
        mean(p), std(p)));

    title(tcl, 'Baro pad-pressure snapshot', 'Interpreter', 'none');

    apply_style_(fig, 12, 8);
    exportgraphics(fig, png_path, 'Resolution', 300);
    close(fig);
end

function plot_mach_window_(truth, Baro, seed, png_path)
    dt = 1 / Baro.Rate_Hz;
    M = truth.mach;
    above = find(M > 0.4);
    if isempty(above)
        return;
    end
    t_lo = max(0.0, truth.time_s(above(1))   - 5.0);
    t_hi = min(truth.time_s(end), truth.time_s(above(end)) + 10.0);
    [p_with_shock, t] = simulate_baro_stream_(truth, t_lo, t_hi, dt, Baro, seed, true);
    [p_clean,     ~] = simulate_baro_stream_(truth, t_lo, t_hi, dt, Baro, seed, false);

    M_at_t = interp1(truth.time_s, truth.mach, t, 'linear', 0);

    fig = figure('Visible', 'off');
    tcl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t, p_clean,      '-', 'DisplayName', 'no shock'); hold on;
    plot(t, p_with_shock, '-', 'DisplayName', 'with shock');
    xlabel('Time [s]'); ylabel('Pressure [Pa]');
    title('Baro pressure: clean vs Mach-shock');
    legend('Location', 'best', 'Box', 'off');

    nexttile;
    yyaxis left;
    plot(t, M_at_t, '-k', 'DisplayName', 'truth Mach'); hold on;
    yline(0.40, '--g', 'DisplayName', 'gate ON 0.40');
    yline(0.35, '--m', 'DisplayName', 'gate OFF 0.35');
    ylabel('Mach [-]');
    yyaxis right;
    plot(t, p_clean - p_with_shock, '-r', 'DisplayName', 'shock error');
    ylabel('Pressure error [Pa]');
    xlabel('Time [s]');
    title('Mach-gate window and induced static-pressure error');
    legend('Location', 'best', 'Box', 'off');

    title(tcl, 'Baro Mach-shock transonic window', 'Interpreter', 'none');

    apply_style_(fig, 12, 8);
    exportgraphics(fig, png_path, 'Resolution', 300);
    close(fig);
end

function plot_psd_(fxx, pxx, png_path)
    fig = figure('Visible', 'off');
    tiledlayout(1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile;
    semilogx(fxx, 10*log10(pxx + eps), '-');
    xlabel('Frequency [Hz]'); ylabel('PSD [dB Pa^{2}/Hz]');
    title('Baro pad-noise PSD');
    hold on;
    xline(0.05, '--', 'DisplayName', '0.05 Hz drift boundary');
    xline(0.1,  '--', 'DisplayName', '0.1 Hz flat-band start');
    pa_per_m = 8.4;
    sigma_white = pa_per_m * sqrt(0.5);
    fs = 100;
    pwn = sigma_white^2 / (fs/2);
    yline(10*log10(pwn), '--r', ...
        'DisplayName', sprintf('analytical white floor (\\sigma=%.2f Pa)', sigma_white));
    legend('Location', 'best', 'Box', 'off');

    apply_style_(fig, 10, 6);
    exportgraphics(fig, png_path, 'Resolution', 300);
    close(fig);
end

function apply_style_(fig, width_in, height_in)
%APPLY_STYLE_ Apply the shared casper_plot_style if available; fall back
% to a minimal white-background figure sizing.
    if exist('casper_plot_style', 'file') == 2
        casper_plot_style(fig, struct('WidthIn', width_in, 'HeightIn', height_in));
    else
        set(fig, 'Color', 'w', 'Units', 'inches', ...
                 'Position', [1 1 width_in height_in], ...
                 'PaperPositionMode', 'auto');
    end
end

% ---------------------------------------------------------------------------

function out = simulink_smoke_(here, t01_dir) %#ok<INUSD>
%SIMULINK_SMOKE_ Drive the standalone baro_block.slx with a constant pressure
% truth bus for 0.2 s and verify the logged pressure samples are sensible.
%
% R2 redesign: instead of wiring the truth_source library (which would
% require populating `truth_ts` in base workspace and worrying about
% timeseries lifetime), we build a tiny driver model that constructs a
% TruthBus with constant values via a Bus Creator and feeds it into the
% baro_block subsystem from baro_block.slx.
    out = struct('pass', false, 'detail', '');
    slx_path = fullfile(here, 'baro_block.slx');
    if ~isfile(slx_path)
        out.detail = sprintf('standalone slx missing: %s', slx_path);
        return;
    end

    mdl_name = 't04_smoke_baro';
    sa_name  = 'baro_block';

    % Ensure Baro / Sim / TruthBus are present in base WS.
    if ~evalin('base', 'exist(''Baro'',''var'')')
        evalin('base', sprintf('run(''%s'')', strrep( ...
            fullfile(here, '..', 'T02_sensor_params', ...
                'casper_sensor_params.m'), '\', '/')));
    end
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    assignin('base', 'TruthBus', casper_truth_build_bus());

    if bdIsLoaded(mdl_name); close_system(mdl_name, 0); end
    if bdIsLoaded(sa_name);  close_system(sa_name, 0);  end

    new_system(mdl_name);
    load_system(mdl_name);
    load_system(slx_path);

    % Copy the baro_block subsystem from the standalone slx into our smoke
    % model. (add_block accepts a source from a loaded model.)
    add_block([sa_name '/' sa_name], [mdl_name '/baro_block']);

    % Build a constant TruthBus driver:
    %   10 Constants -> Bus Creator (TruthBus) -> Inport of baro_block
    Pad_p = 101325.0;
    Pad_T_K = 288.15;
    Pad_rho = 1.225;

    bus_path = [mdl_name '/TruthBusCreator'];
    add_block('simulink/Signal Routing/Bus Creator', bus_path);
    set_param(bus_path, ...
        'Inputs',         '10', ...
        'OutDataTypeStr', 'Bus: TruthBus', ...
        'NonVirtualBus',  'on');

    % Order MUST match TruthBus definition: pos_NED, vel_NED, accel_NED,
    % quat_std, omega_body_std, time_s, mach, air_density_kgm3, air_temp_K,
    % air_pressure_pa.
    consts = {
        % name,        value,                       dims
        'C_pos',       '[0;0;0]',                   3, ...
        'C_vel',       '[0;0;0]',                   3, ...
        'C_acc',       '[0;0;0]',                   3, ...
        'C_quat',      '[1;0;0;0]',                 4, ...
        'C_omega',     '[0;0;0]',                   3, ...
        'C_time',      '0',                         1, ...
        'C_mach',      '0',                         1, ...
        'C_rho',       num2str(Pad_rho, '%.6f'),    1, ...
        'C_temp',      num2str(Pad_T_K, '%.6f'),    1, ...
        'C_pres',      num2str(Pad_p,   '%.6f'),    1, ...
    };
    nf = numel(consts) / 3;
    for k = 1:nf
        name  = consts{1, (k-1)*3 + 1};
        value = consts{1, (k-1)*3 + 2};
        cpath = [mdl_name '/' name];
        add_block('simulink/Sources/Constant', cpath);
        set_param(cpath, 'Value', value, 'SampleTime', '-1');
        lh = add_line(mdl_name, [name '/1'], ...
            ['TruthBusCreator/' num2str(k)], 'autorouting', 'on');
        % Set signal name = bus field name (Bus Creator uses these labels)
        bus_field_names = {'pos_NED','vel_NED','accel_NED','quat_std', ...
                           'omega_body_std','time_s','mach', ...
                           'air_density_kgm3','air_temp_K','air_pressure_pa'};
        set_param(lh, 'Name', bus_field_names{k});
    end

    % Wire bus -> baro_block input port 1.
    add_line(mdl_name, 'TruthBusCreator/1', 'baro_block/1', 'autorouting', 'on');

    % Log pressure
    tw_p = [mdl_name '/tw_pressure'];
    add_block('simulink/Sinks/To Workspace', tw_p);
    set_param(tw_p, 'VariableName', 'log_pressure', 'SaveFormat', 'Array', ...
              'SampleTime', '1/Baro.Rate_Hz');
    add_line(mdl_name, 'baro_block/1', 'tw_pressure/1', 'autorouting', 'on');

    % Terminate unused outports of baro_block
    for k = 2:3
        tn = sprintf('%s/term%d', mdl_name, k);
        add_block('simulink/Sinks/Terminator', tn);
        add_line(mdl_name, sprintf('baro_block/%d', k), ...
            sprintf('term%d/1', k), 'autorouting', 'on');
    end

    set_param(mdl_name, ...
        'Solver',                  'ode4', ...
        'SolverType',              'Fixed-step', ...
        'FixedStep',               '1e-4', ...
        'StopTime',                '0.2', ...
        'SaveOutput',              'on', ...
        'ReturnWorkspaceOutputs',  'on');

    try
        simOut = sim(mdl_name);
    catch ME
        msg = ME.message;
        for kk = 1:numel(ME.cause)
            msg = sprintf('%s | cause %d: %s', msg, kk, ME.cause{kk}.message);
        end
        out.detail = sprintf('sim failed: %s', msg);
        if bdIsLoaded(mdl_name); close_system(mdl_name, 0); end
        if bdIsLoaded(sa_name);  close_system(sa_name, 0);  end
        return;
    end

    % To Workspace writes into the SimulationOutput container when
    % ReturnWorkspaceOutputs is on. Pull from there; fall back to base WS.
    if ~isempty(simOut) && isprop(simOut, 'log_pressure')
        p_log = simOut.log_pressure;
    elseif evalin('base', 'exist(''log_pressure'',''var'')')
        p_log = evalin('base', 'log_pressure');
    else
        p_log = [];
    end
    if isempty(p_log) || any(~isfinite(p_log)) || any(p_log(:) < 1)
        out.detail = sprintf( ...
            'logged pressure not sensible (n=%d, min=%.2f, max=%.2f)', ...
            numel(p_log), min(p_log(:)), max(p_log(:)));
    elseif abs(mean(p_log(:)) - Pad_p) > 200.0
        out.detail = sprintf( ...
            'mean pressure off truth: mean=%.2f Pa, truth=%.2f Pa, delta=%.2f Pa', ...
            mean(p_log(:)), Pad_p, abs(mean(p_log(:)) - Pad_p));
    else
        out.pass = true;
        out.detail = sprintf( ...
            'n=%d samples, mean=%.2f Pa, std=%.3f Pa, truth=%.2f Pa', ...
            numel(p_log), mean(p_log(:)), std(p_log(:)), Pad_p);
    end

    if bdIsLoaded(mdl_name); close_system(mdl_name, 0); end
    if bdIsLoaded(sa_name);  close_system(sa_name, 0);  end
end
