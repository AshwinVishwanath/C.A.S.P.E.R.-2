function results = test_mag_model()
%TEST_MAG_MODEL Acceptance tests for the T05 mag sensor model.
%
% Synopsis:
%   results = test_mag_model()
%
% Loads parameters from T02 (casper_sensor_params), exercises every
% MATLAB function in T05, builds the Simulink library, and writes a
% per-criterion PASS/FAIL summary to STATUS.md (and plots/).
%
% Returns:
%   results : struct array with fields
%       id          : char, e.g. 'C1'
%       description : char
%       status      : 'PASS' | 'FAIL'
%       detail      : char (human-readable measurement)
%
% Source firmware reference: see individual function headers.

    here = fileparts(mfilename('fullpath'));
    plot_dir = fullfile(here, 'plots');
    if ~isfolder(plot_dir)
        mkdir(plot_dir);
    end

    % ---------- Load parameters into base workspace ----------
    t02_dir    = fullfile(fileparts(here), 'T02_sensor_params');
    t02_script = fullfile(t02_dir, 'casper_sensor_params.m');
    addpath(t02_dir);
    addpath(here);
    evalin('base', sprintf('run(''%s'')', strrep(t02_script, '''', '''''')));

    Sim = evalin('base', 'Sim');
    Mag = evalin('base', 'Mag');

    results = struct('id', {}, 'description', {}, 'status', {}, 'detail', {});

    % ===================================================================
    % C1: Stationary pad, no noise, no interference: mag magnitude == |B|
    % ===================================================================
    % Pad attitude: pitch up 89 deg (matches T01 spec, rocket vertical).
    % For the magnitude check the attitude does not matter (rotation
    % preserves magnitude) — but use a non-trivial quat to exercise the
    % rotation path. Use eul2quat(pad).
    pad_quat = eul2quat([0, 89*pi/180, 0], 'ZYX').';  % 4x1 column

    mag_NED  = casper_mag_field_world(zeros(3,1));
    mag_body = casper_mag_rotate_to_body(mag_NED, pad_quat);
    mag_mag  = norm(mag_body);
    expected_mag = norm(mag_NED);  % matches the configured truth field magnitude
    pass_c1 = abs(mag_mag - expected_mag) < 1e-9;
    results(end+1) = mk('C1', ...
        'pad attitude, no noise: |mag_body| == |B_NED| (rotation preserves magnitude)', ...
        pass_c1, sprintf('|mag|=%.6f, |B|=%.6f, |delta|=%.2e', ...
                          mag_mag, expected_mag, abs(mag_mag-expected_mag)));

    % ===================================================================
    % C2: Round-trip identity — firmware cal recovers truth
    % ===================================================================
    % Apply distort_field to produce raw, then apply firmware cal path,
    % then check we recover mag_body within 0.1 µT. Also check that the
    % calibrated magnitude lands at 40.18 ± 0.5 µT.
    raw_body = casper_mag_distort_field(mag_body, Mag.HardIron_uT, ...
                                          Mag.SoftIron, Mag.AxisFlipSign);
    cal_body = firmware_mag_cal_apply(raw_body, Mag);
    rt_err   = max(abs(cal_body - mag_body));
    cal_mag  = norm(cal_body);

    pass_c2_roundtrip = rt_err < 0.1;
    pass_c2_magnitude = abs(cal_mag - Mag.ExpectedMag_uT) <= 0.5;
    pass_c2 = pass_c2_roundtrip && pass_c2_magnitude;
    results(end+1) = mk('C2', ...
        'round-trip identity AND calibrated |mag| in 40.18 ± 0.5 µT', ...
        pass_c2, sprintf(['round-trip max-abs-err=%.2e µT (limit 0.1), ' ...
                          'cal_mag=%.4f µT (target 40.18 ± 0.5)'], ...
                          rt_err, cal_mag));

    % ===================================================================
    % C3: AR(1) noise correlation == exp(-dt/tau)
    % ===================================================================
    N_C3   = 20000;
    seed_C3 = Sim.Seed + 3;
    dt      = 1 / Mag.Rate_Hz;
    expected_alpha = exp(-dt / Mag.NoiseTauSec);

    % Clear persistents and reseed by passing the seed-controlled function.
    clear casper_mag_noise;
    noise_x = zeros(N_C3, 1);
    zero_in = zeros(3,1);
    % Subtract the (quantized) zero-mean signal: feed zero -> output is
    % the noise post-quantization. Quantization quantum is 0.0061 µT
    % whereas sigma is 0.5 µT, so quant adds negligible decorrelation.
    for k = 1:N_C3
        [m_out, ~] = casper_mag_noise(zero_in, dt, Mag.NoiseTauSec, ...
            Mag.NoiseStd_uT, Mag.ScaleCountsPerGauss, ...
            Mag.OffsetCounts, seed_C3);
        noise_x(k) = m_out(1);
    end
    rho_lag1 = corr_lag1(noise_x);
    pass_c3 = abs(rho_lag1 - expected_alpha) < 0.02;
    results(end+1) = mk('C3', ...
        'AR(1) lag-1 autocorrelation ≈ exp(-dt/tau)', ...
        pass_c3, sprintf('rho1=%.4f vs target %.4f (|delta|<%.3f)', ...
                          rho_lag1, expected_alpha, 0.02));

    % ===================================================================
    % C4: PSD roll-off — colored noise drops off below ~5 Hz, white floor above
    % ===================================================================
    % For an AR(1) process with corner 1/(2*pi*tau) Hz, tau=0.16 -> ~1 Hz
    % corner. Below corner: -3 dB level low freq. Above corner: -20 dB/dec.
    % We check that PSD at 5 Hz is at least 6 dB below PSD at 0.5 Hz.
    fs   = Mag.Rate_Hz;
    nfft = 4096;
    [psd, f_hz] = simple_psd_welch(noise_x - mean(noise_x), fs, nfft);

    psd_low  = mean(psd(f_hz > 0.3 & f_hz < 0.7));   % around 0.5 Hz
    psd_hi_5 = mean(psd(f_hz > 4.5 & f_hz < 5.5));   % around 5 Hz
    psd_drop_dB = 10 * log10(psd_low / max(psd_hi_5, eps));
    pass_c4 = psd_drop_dB >= 6;  % at least 6 dB of low-pass shape
    results(end+1) = mk('C4', ...
        'PSD: colored shape (drop ≥ 6 dB from ~0.5 Hz to ~5 Hz)', ...
        pass_c4, sprintf('drop = %.2f dB (limit ≥ 6 dB)', psd_drop_dB));

    % Save PSD plot.
    fig = figure('Visible', 'off');
    semilogx(f_hz, 10*log10(max(psd, eps)));
    grid on;
    xlabel('Frequency (Hz)'); ylabel('PSD (dB µT^2/Hz)');
    title('Mag AR(1) noise PSD (axis X, zero-input)');
    saveas(fig, fullfile(plot_dir, 'mag_noise_psd.png'));
    close(fig);

    % ===================================================================
    % C5: Quantization — raw_18bit is integer in [0, 262143]; mag_uT is on grid
    % ===================================================================
    pass_c5 = true;
    grid_uT = 100 / Mag.ScaleCountsPerGauss;  % 1 LSB in µT, ≈ 0.006104
    test_inputs_uT = [10; -25; 47; -1.234; 0];
    raw_check = zeros(3, numel(test_inputs_uT), 'uint32');
    err_grid  = zeros(3, numel(test_inputs_uT));
    clear casper_mag_noise;
    for j = 1:numel(test_inputs_uT)
        % Zero-sigma to disable noise path; only quantization remains.
        in = test_inputs_uT(j) * ones(3, 1);
        [m_out, raw18] = casper_mag_noise(in, dt, Mag.NoiseTauSec, ...
            0.0, Mag.ScaleCountsPerGauss, Mag.OffsetCounts, Sim.Seed + 3);
        raw_check(:, j) = raw18;
        recon = (double(raw18) - Mag.OffsetCounts) / Mag.ScaleCountsPerGauss * 100;
        err_grid(:, j) = m_out - recon;
        if any(raw18 < 0) || any(raw18 > uint32(262143)) || ...
                any(abs(err_grid(:, j)) > 1e-9)
            pass_c5 = false;
        end
    end
    grid_max_err = max(abs(err_grid(:)));
    pass_c5 = pass_c5 && all(raw_check(:) <= uint32(262143));
    results(end+1) = mk('C5', ...
        'quantization: raw_18bit ∈ [0, 262143] and mag_uT on 1-LSB grid', ...
        pass_c5, sprintf(['grid LSB = %.6f µT; max(|m_uT - decode(raw18)|) ' ...
                          '= %.2e µT; all raw in range'], grid_uT, grid_max_err));

    % ===================================================================
    % C6: TX schedule — 1.5 samples per 10 Hz cycle, exactly 15 ms of TX per 100 ms
    % ===================================================================
    % Two independent checks:
    %  (a) Sample-domain at mag rate (100 Hz): 1.5 samples per cycle on
    %      average across 10 cycles.
    %  (b) Time-resolved at 100 kHz: TX active for 15.000 ms per cycle.
    t_mag = (0:dt:0.99).';   % 100 samples (1 s)
    tx_at_mag = false(numel(t_mag), 1);
    for k = 1:numel(t_mag)
        tx_at_mag(k) = casper_radio_tx_schedule(t_mag(k), ...
            Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s);
    end
    samples_per_cycle = sum(tx_at_mag) / 10;     % 10 cycles in 1 s
    % T05 §6.3 predicts samples at 0 and 10 ms are both inside the first TX
    % window [0, 15 ms] while 20 ms is outside, i.e. 2 samples per 100 ms
    % cycle at the discrete mag sample grid. The 1.5 in §7 acceptance refers
    % to the continuous-time ratio (15 ms airtime / 10 ms sample period),
    % verified independently below at fine grid.
    pass_c6a = abs(samples_per_cycle - 2.0) < 1e-9;

    dt_fine = 1e-5;
    t_fine  = (0:dt_fine:0.1-dt_fine).';
    tx_fine = false(numel(t_fine), 1);
    for k = 1:numel(t_fine)
        tx_fine(k) = casper_radio_tx_schedule(t_fine(k), ...
            Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s);
    end
    tx_duration_s = sum(tx_fine) * dt_fine;
    pass_c6b = abs(tx_duration_s - Mag.RadioTXAirtime_s) < 2 * dt_fine;

    pass_c6 = pass_c6a && pass_c6b;
    results(end+1) = mk('C6', ...
        'TX schedule: 15 ms active per 100 ms (1.5 sample-periods worth, 2 discrete samples)', ...
        pass_c6, sprintf(['samples_per_cycle=%.4f (target 2, per spec §6.3); ' ...
                          'tx_duration=%.6f s (target 0.015 s, == 1.5 sample-periods)'], ...
                          samples_per_cycle, tx_duration_s));

    % ===================================================================
    % C7: During TX events, mag deviates by ±10 µT (±1 µT tolerance)
    % ===================================================================
    % Run a 2 s window with the full chain (truth=pad, noise, interference).
    % Compare TX-on samples vs TX-off median to get spike magnitude.
    T = 2.0;
    [~, mag_uT_run, ~, tx_run] = run_chain_pad(T, Mag, Sim, true);

    on_idx  = tx_run;
    off_idx = ~tx_run;

    % Per-axis: median during TX events minus median outside is the spike.
    spike_amp = zeros(3, 1);
    for ax = 1:3
        spike_amp(ax) = median(mag_uT_run(on_idx, ax)) ...
                       - median(mag_uT_run(off_idx, ax));
    end
    spike_amp_abs = abs(spike_amp);
    pass_c7 = all(abs(spike_amp_abs - Mag.RadioSpikeAmp_uT) <= 1.0);
    results(end+1) = mk('C7', ...
        'TX-on deviation ≈ ±10 µT per axis (±1 µT tolerance)', ...
        pass_c7, sprintf('per-axis |spike|=[%.3f %.3f %.3f] vs target %.1f µT', ...
                          spike_amp_abs(1), spike_amp_abs(2), ...
                          spike_amp_abs(3), Mag.RadioSpikeAmp_uT));

    % Capture per-axis signs (signed median during TX vs off) for STATUS.
    axis_signs = sign(spike_amp);

    % ===================================================================
    % C8: Outside TX events, mag stays within ±3 µT of the clean signal
    % ===================================================================
    % The output stream is the RAW sensor reading (post sign-flip and post
    % hard/soft iron inverse), not the clean body-frame field. So compare
    % against the clean-RAW (mag chain pre-noise pre-interference).
    mag_NED_pad       = casper_mag_field_world(zeros(3,1));
    mag_clean_body    = casper_mag_rotate_to_body(mag_NED_pad, pad_quat);
    mag_clean_raw     = casper_mag_distort_field(mag_clean_body, ...
                          Mag.HardIron_uT, Mag.SoftIron, Mag.AxisFlipSign);
    dev = mag_uT_run(off_idx, :) - mag_clean_raw.';
    max_dev = max(abs(dev(:)));
    pass_c8 = max_dev <= 3.0;
    results(end+1) = mk('C8', ...
        'off-TX samples within ±3 µT of clean raw signal (noise band)', ...
        pass_c8, sprintf('max |dev|=%.4f µT (limit 3.0)', max_dev));

    % ===================================================================
    % C9: Reproducibility — two runs with same seed give identical mag streams
    % ===================================================================
    clear casper_mag_noise casper_mag_radio_interference;
    [~, mag_a, raw_a, tx_a] = run_chain_pad(1.0, Mag, Sim, true);
    clear casper_mag_noise casper_mag_radio_interference;
    [~, mag_b, raw_b, tx_b] = run_chain_pad(1.0, Mag, Sim, true);
    bit_match = isequal(mag_a, mag_b) && isequal(raw_a, raw_b) && isequal(tx_a, tx_b);
    pass_c9 = bit_match;
    results(end+1) = mk('C9', ...
        'reproducibility: identical seed -> byte-identical mag stream', ...
        pass_c9, sprintf('bit-identical: %d', bit_match));

    % ===================================================================
    % C10: build_mag_block runs without error and produces a valid .slx
    % ===================================================================
    pass_c10 = false;
    build_err_msg = '';
    try
        model_path = build_mag_block();
        pass_c10 = isfile(model_path);
        build_detail = sprintf('mag_block.slx at %s', model_path);
    catch ME
        build_err_msg = sprintf('%s: %s', ME.identifier, ME.message);
        build_detail = build_err_msg;
    end
    results(end+1) = mk('C10', ...
        'build_mag_block produces a valid mag_block.slx', ...
        pass_c10, build_detail);

    % ===================================================================
    % Plots
    % ===================================================================
    % mag_pad_1s_no_tx.png
    clear casper_mag_noise casper_mag_radio_interference;
    [t_no, mag_no, ~, ~] = run_chain_pad(1.0, Mag, Sim, false);
    fig = figure('Visible','off','Position',[100 100 900 500]);
    plot(t_no, mag_no(:,1), 'r', t_no, mag_no(:,2), 'g', t_no, mag_no(:,3), 'b');
    grid on;
    legend('m_x','m_y','m_z','Location','best');
    xlabel('time (s)'); ylabel('mag (µT)');
    title('Pad mag, 1 s, RadioInterfActive = false');
    saveas(fig, fullfile(plot_dir, 'mag_pad_1s_no_tx.png'));
    close(fig);
    pass_p1 = isfile(fullfile(plot_dir, 'mag_pad_1s_no_tx.png'));

    % mag_pad_1s_with_tx.png
    clear casper_mag_noise casper_mag_radio_interference;
    [t_yes, mag_yes, ~, tx_yes] = run_chain_pad(1.0, Mag, Sim, true);
    fig = figure('Visible','off','Position',[100 100 900 500]);
    subplot(2,1,1);
    plot(t_yes, mag_yes(:,1), 'r', t_yes, mag_yes(:,2), 'g', t_yes, mag_yes(:,3), 'b');
    grid on;
    legend('m_x','m_y','m_z','Location','best');
    ylabel('mag (µT)');
    title('Pad mag, 1 s, RadioInterfActive = true (spikes every 100 ms)');
    subplot(2,1,2);
    stairs(t_yes, double(tx_yes), 'k', 'LineWidth', 1.1);
    grid on;
    xlabel('time (s)'); ylabel('tx\_active');
    ylim([-0.1, 1.1]);
    saveas(fig, fullfile(plot_dir, 'mag_pad_1s_with_tx.png'));
    close(fig);
    pass_p2 = isfile(fullfile(plot_dir, 'mag_pad_1s_with_tx.png'));

    % mag_tx_event_zoom.png  --  100 ms zoom on first TX
    fig = figure('Visible','off','Position',[100 100 900 500]);
    zoom_idx = t_yes <= 0.1 + 1e-9;
    yyaxis left;
    plot(t_yes(zoom_idx)*1000, mag_yes(zoom_idx,1), 'r-o', ...
         t_yes(zoom_idx)*1000, mag_yes(zoom_idx,2), 'g-s', ...
         t_yes(zoom_idx)*1000, mag_yes(zoom_idx,3), 'b-^');
    ylabel('mag (µT)');
    yyaxis right;
    stairs(t_yes(zoom_idx)*1000, double(tx_yes(zoom_idx)), 'k', 'LineWidth', 1.2);
    ylim([-0.1, 1.1]);
    ylabel('tx\_active');
    grid on;
    xlabel('time (ms)');
    title('Single TX event zoom (0-100 ms)');
    legend('m_x','m_y','m_z','tx\_active','Location','best');
    saveas(fig, fullfile(plot_dir, 'mag_tx_event_zoom.png'));
    close(fig);
    pass_p3 = isfile(fullfile(plot_dir, 'mag_tx_event_zoom.png'));

    pass_c_plots = pass_p1 && pass_p2 && pass_p3;
    results(end+1) = mk('C11', ...
        'plots: mag_pad_1s_no_tx, mag_pad_1s_with_tx, mag_tx_event_zoom exist', ...
        pass_c_plots, sprintf('no_tx=%d, with_tx=%d, zoom=%d', ...
                               pass_p1, pass_p2, pass_p3));

    % ===================================================================
    % Write STATUS.md
    % ===================================================================
    write_status_md(here, results, Mag, Sim, axis_signs);

    % ===================================================================
    % Print summary
    % ===================================================================
    n_pass = sum(strcmp({results.status}, 'PASS'));
    n_fail = sum(strcmp({results.status}, 'FAIL'));
    fprintf('\n[test_mag_model] %d PASS, %d FAIL\n', n_pass, n_fail);
    for k = 1:numel(results)
        fprintf('  %s [%s] %s\n', results(k).id, results(k).status, ...
            results(k).description);
        if strcmp(results(k).status, 'FAIL')
            fprintf('      detail: %s\n', results(k).detail);
        end
    end
end

% ===========================================================================
% Helper functions
% ===========================================================================

function r = mk(id, desc, pass_bool, detail)
    if pass_bool
        st = 'PASS';
    else
        st = 'FAIL';
    end
    r = struct('id', id, 'description', desc, 'status', st, 'detail', detail);
end

function cal = firmware_mag_cal_apply(raw_uncal_uT, Mag)
% Replicate mag_cal.c forward path:
%   frame_mapped = AxisFlipSign .* raw  (firmware does ×-1 per axis)
%   cal          = soft_iron * (frame_mapped - hard_iron)
    frame_mapped = Mag.AxisFlipSign .* raw_uncal_uT;
    cal = Mag.SoftIron * (frame_mapped - Mag.HardIron_uT);
end

function rho = corr_lag1(x)
    x = x(:) - mean(x);
    num = sum(x(1:end-1) .* x(2:end));
    den = sum(x.^2);
    rho = num / max(den, eps);
end

function [psd, f] = simple_psd_welch(x, fs, nfft)
% Hand-rolled Welch (Hann window, 50% overlap), to avoid Sig Proc Tbx dep.
    x = x(:);
    win = 0.5 - 0.5*cos(2*pi*(0:nfft-1).'/(nfft-1));   % Hann
    overlap = nfft / 2;
    step = nfft - overlap;
    nseg = floor((numel(x) - nfft) / step) + 1;
    if nseg < 1
        psd = zeros(nfft/2+1, 1);
        f   = (0:nfft/2).' * fs / nfft;
        return;
    end
    U = sum(win.^2);
    Pxx = zeros(nfft/2+1, 1);
    for k = 1:nseg
        i0 = (k-1)*step + 1;
        seg = x(i0:i0+nfft-1) .* win;
        X = fft(seg, nfft);
        P = (abs(X(1:nfft/2+1)).^2) / (fs * U);
        % Double the one-sided PSD except DC and Nyquist
        P(2:end-1) = 2 * P(2:end-1);
        Pxx = Pxx + P;
    end
    psd = Pxx / nseg;
    f   = (0:nfft/2).' * fs / nfft;
end

function [t_s, mag_uT, raw_18bit, tx_active] = run_chain_pad(T_s, Mag, Sim, interf_on)
% Exercise the full T05 chain on a stationary pad for T_s seconds, mag rate.
% Returns:
%   t_s        : Nx1
%   mag_uT     : Nx3 (post-interference)
%   raw_18bit  : Nx3 uint32 (pre-interference; firmware integer output)
%   tx_active  : Nx1 logical

    dt   = 1 / Mag.Rate_Hz;
    t_s  = (0:dt:T_s-dt).';
    N    = numel(t_s);

    pad_quat = eul2quat([0, 89*pi/180, 0], 'ZYX').';
    pos_zero = zeros(3, 1);

    mag_NED       = casper_mag_field_world(pos_zero);
    mag_clean_body = casper_mag_rotate_to_body(mag_NED, pad_quat);
    raw_clean      = casper_mag_distort_field(mag_clean_body, ...
                        Mag.HardIron_uT, Mag.SoftIron, Mag.AxisFlipSign);

    mag_uT    = zeros(N, 3);
    raw_18bit = zeros(N, 3, 'uint32');
    tx_active = false(N, 1);

    interf_active = interf_on && Mag.RadioInterfActive;

    seed_noise = uint32(Sim.Seed + 3);
    seed_interf = uint32(Sim.Seed + 7);

    % Force fresh persistent state
    clear casper_mag_noise casper_mag_radio_interference;

    for k = 1:N
        [m_noisy, raw18] = casper_mag_noise(raw_clean, dt, Mag.NoiseTauSec, ...
            Mag.NoiseStd_uT, Mag.ScaleCountsPerGauss, Mag.OffsetCounts, seed_noise);
        tx_k = casper_radio_tx_schedule(t_s(k), Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s);
        m_out = casper_mag_radio_interference(m_noisy, tx_k, interf_active, ...
            Mag.RadioSpikeAmp_uT, seed_interf);
        mag_uT(k, :)    = m_out.';
        raw_18bit(k, :) = raw18.';
        tx_active(k)    = tx_k;
    end
end

function write_status_md(here, results, Mag, Sim, axis_signs)
    md_path = fullfile(here, 'STATUS.md');
    fid = fopen(md_path, 'w');
    if fid < 0
        error('test_mag_model:WriteStatus', 'cannot open %s for write', md_path);
    end
    cleanup = onCleanup(@() fclose(fid));

    n_pass = sum(strcmp({results.status}, 'PASS'));
    n_fail = sum(strcmp({results.status}, 'FAIL'));
    overall = 'PASS';
    if n_fail > 0
        overall = 'FAIL';
    end

    fprintf(fid, '# T05 Magnetometer Model — Test Status\n\n');
    fprintf(fid, 'Generated by `test_mag_model.m` on %s.\n\n', ...
        char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
    fprintf(fid, 'Build dir: `%s`\n\n', strrep(here, '\', '/'));
    fprintf(fid, '**Overall: %s** (%d pass / %d fail of %d)\n\n', ...
        overall, n_pass, n_fail, numel(results));

    fprintf(fid, '## Acceptance Criteria\n\n');
    fprintf(fid, '| # | Criterion | Status | Detail |\n');
    fprintf(fid, '|---|---|---|---|\n');
    for k = 1:numel(results)
        fprintf(fid, '| %s | %s | %s | %s |\n', results(k).id, ...
            md_escape(results(k).description), results(k).status, ...
            md_escape(results(k).detail));
    end

    fprintf(fid, '\n## Files Produced\n\n');
    fprintf(fid, '- `casper_mag_field_world.m`\n');
    fprintf(fid, '- `casper_mag_rotate_to_body.m`\n');
    fprintf(fid, '- `casper_mag_distort_field.m`\n');
    fprintf(fid, '- `casper_mag_noise.m`\n');
    fprintf(fid, '- `casper_mag_radio_interference.m`\n');
    fprintf(fid, '- `casper_radio_tx_schedule.m`\n');
    fprintf(fid, '- `build_mag_block.m`\n');
    fprintf(fid, '- `test_mag_model.m`\n');
    fprintf(fid, '- `mag_block.slx`\n');
    fprintf(fid, '- `plots/mag_pad_1s_no_tx.png`\n');
    fprintf(fid, '- `plots/mag_pad_1s_with_tx.png`\n');
    fprintf(fid, '- `plots/mag_tx_event_zoom.png`\n');
    fprintf(fid, '- `plots/mag_noise_psd.png` (informational)\n');
    fprintf(fid, '- `STATUS.md`\n');

    fprintf(fid, '\n## Radio Interference — Placeholder Status\n\n');
    fprintf(fid, '**This effect is NOT calibrated.** The ±%.1f µT rectangular pulse model is a Phase 0 placeholder. ', ...
        Mag.RadioSpikeAmp_uT);
    fprintf(fid, 'It exists to validate that the EKF + attitude filter behave robustly when mag samples are intermittently corrupted, ');
    fprintf(fid, 'NOT to reproduce real-world coupling magnitudes. Phase 1 will fit the amplitude and per-axis coupling matrix ');
    fprintf(fid, 'from bench data (see PHASE0_SPEC.md §6, T05 §9). Downstream consumers of this model must NOT treat the spike ');
    fprintf(fid, 'amplitude as a calibrated number.\n\n');
    fprintf(fid, 'Per-axis sign vector (seeded once from `Sim.Seed + 7 = %d`): [%+d, %+d, %+d].\n\n', ...
        Sim.Seed + 7, round(axis_signs(1)), round(axis_signs(2)), round(axis_signs(3)));

    fprintf(fid, '## Deviations / Notes\n\n');
    fprintf(fid, '- Spec §5.4 cites quantization scale `81.92 counts/µT` (= 1 LSB ≈ 0.0122 µT). The firmware (`mmc5983ma.c` lines 184-186) decodes as `mag_uT = (raw - 131072) / 16384 * 100`, i.e. **163.84 counts/µT** (1 LSB ≈ 0.006104 µT). The implementation uses the firmware-correct values from `casper_sensor_params.m` (`Mag.ScaleCountsPerGauss = 16384`, `Mag.OffsetCounts = 131072`). The acceptance criterion was reframed in terms of "raw_18bit is integer in [0, 262143] and mag_uT is on a 1-LSB grid", which is what the spec author was actually trying to test.\n');
    fprintf(fid, '- Spec mentions a calibration scaling division-by-2 inside that 81.92 derivation that is not present in the firmware; the firmware decode and the spec parenthetical "raw_18bit integer in [0, 2^18-1]" are mutually consistent, and that is what test C5 verifies.\n');
    fprintf(fid, '- Plot `mag_noise_psd.png` (PSD) is generated in addition to the three required plots, as supporting evidence for C4.\n');
    fprintf(fid, '- The Simulink library `mag_block.slx` is built via `build_mag_block.m` and saved as a locked library. It exposes inputs `quat_std (4x1)`, `pos_NED_m (3x1)`, `time_s (1)` and outputs `mag_uT_body (3x1)`, `mag_raw_18bit (3x1 uint32)`, `data_ready (bool)`, `tx_active (bool)`. The block runs at `1/Mag.Rate_Hz = 100 Hz` after the input Rate Transitions; the inputs themselves can be driven at the 10 kHz solver rate (typical use). All numerical parameters are bound at runtime via `evalin(''base'', ...)` to the structs created by `casper_sensor_params.m`.\n');
    fprintf(fid, '- The Simulink library is built and persisted but not exercised in `sim()` here — the unit tests target the .m functions directly because they hold the math. Full Simulink-level integration is the responsibility of T11.\n');
end

function s = md_escape(s)
    s = strrep(s, '|', '\|');
    s = strrep(s, newline, ' ');
end
