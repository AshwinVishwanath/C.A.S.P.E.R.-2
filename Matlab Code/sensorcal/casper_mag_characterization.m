%% C.A.S.P.E.R.-2 Magnetometer Noise + Radio/Flash EMI Characterization
%
%  Consumes the 4 CSV files produced by the MAG_NOISE firmware mode
%  (see Software/App/cal/mag_noise.c) and produces:
%
%    1. Allan deviation per axis  -> noise density, ARW, bias instability
%    2. PSD per axis with radio-TX windowing
%    3. EMI delta: mean/sigma shift inside TX bursts vs surrounding quiet
%    4. Reference-quat drift (static-bench sanity check)
%    5. Hard-iron drift over the 10-min capture
%
%  Output mag_noise_params.mat is consumed by:
%    Matlab Code/Simulink Development/sensors/mag/casper_mag_noise.m
%    Matlab Code/Simulink Development/sensors/mag/casper_mag_radio_interference.m
%
%  Expected CSV files (rename after each MSC pull):
%    run_A.csv  Radio OFF,                normal flash flushes
%    run_B.csv  Radio Profile A (SF7),    10 Hz TX
%    run_C.csv  Radio Profile B (SF8 hi), 10 Hz TX
%    run_D.csv  Radio OFF,                forced 50 ms flash bursts

clear; clc; close all;

%% ========================================================================
%                         USER CONFIGURATION
%  ========================================================================

% Per-run files (rename after each MSC pull).
%   .file  : mag samples (MAG_NOISE.CSV)
%   .evt   : radio TX events (RADIO_EVT.CSV)  — only populated for B/C
runs(1).id  = 'A'; runs(1).file = 'run_A.csv';     runs(1).evt = '';
runs(1).label = 'baseline (radio OFF)';
runs(2).id  = 'B'; runs(2).file = 'run_B.csv';     runs(2).evt = 'run_B_evt.csv';
runs(2).label = 'radio Profile A (SF7)';
runs(3).id  = 'C'; runs(3).file = 'run_C.csv';     runs(3).evt = 'run_C_evt.csv';
runs(3).label = 'radio Profile B (SF8, +20 dBm)';
runs(4).id  = 'D'; runs(4).file = 'run_D.csv';     runs(4).evt = '';
runs(4).label = 'flash burst (radio OFF)';

% Sensor nominal sample rate (MMC5983MA continuous mode)
MAG_FS_HZ = 100;
MAG_DT_S  = 1 / MAG_FS_HZ;

% Allan variance tau range — log-spaced from 1 sample up to ~T/10
ALLAN_MIN_M = 1;       % minimum cluster size (samples)
ALLAN_MAX_FRAC = 0.10; % max tau = MAX_FRAC * record length
ALLAN_NUM_TAUS = 40;   % number of points on the log-tau axis

% Output plot directory
plot_dir = 'plots_mag_noise';
if ~exist(plot_dir, 'dir'), mkdir(plot_dir); end

% Output .mat path (consumed by Simulink mag block)
PARAMS_OUT = 'mag_noise_params.mat';

%% ========================================================================
%                       SECTION 1: LOAD RUNS
%  ========================================================================
fprintf('================================================================\n');
fprintf('  C.A.S.P.E.R.-2 MAGNETOMETER NOISE CHARACTERIZATION\n');
fprintf('================================================================\n\n');

for k = 1:numel(runs)
    fname = runs(k).file;
    if ~exist(fname, 'file')
        warning('Run %s: file %s not found, skipping.', runs(k).id, fname);
        runs(k).data = [];
        continue;
    end

    fprintf('Loading run %s (%s)... ', runs(k).id, fname);
    T = readtable(fname, 'CommentStyle', '#');
    runs(k).data = T;

    % Sanity: report duration, sample count, mean sample interval
    n = height(T);
    t = T.t_ms / 1000;
    dt_mean = mean(diff(t));
    dt_std  = std(diff(t));
    fprintf('%d samples, %.1f s, dt = %.4f +/- %.4f s\n', ...
            n, t(end), dt_mean, dt_std);

    % Radio event file (sub-ms accurate TX start/end timestamps)
    runs(k).evt_data = [];
    if ~isempty(runs(k).evt) && exist(runs(k).evt, 'file')
        E = readtable(runs(k).evt, 'CommentStyle', '#');
        runs(k).evt_data = E;
        durs = E.duration_ms;
        fprintf('  + RADIO_EVT: %d TX events, mean duration = %.1f ms ' ...
                '(min %.1f, max %.1f), %d failed\n', ...
                height(E), mean(durs), min(durs), max(durs), sum(E.ok==0));
    end
end

%% ========================================================================
%                  SECTION 2: REFERENCE-QUAT DRIFT CHECK
%  ========================================================================
% On a static bench, the accel+gyro-only quaternion should drift slowly
% (gyro bias). A large drift means the bench moved -> data invalid.

fprintf('\n--- Reference attitude drift (bench-static sanity) ---\n');
for k = 1:numel(runs)
    if isempty(runs(k).data), continue; end
    T = runs(k).data;
    q0 = [T.qw(1), T.qx(1), T.qy(1), T.qz(1)];
    qN = [T.qw(end), T.qx(end), T.qy(end), T.qz(end)];
    % Angle between q0 and qN: 2*acos(|<q0,qN>|)
    dot_qq = abs(q0 * qN.');
    dot_qq = min(dot_qq, 1.0);
    drift_deg = 2 * acosd(dot_qq);
    fprintf('  Run %s: total quat drift = %.3f deg over %.1f min\n', ...
            runs(k).id, drift_deg, (T.t_ms(end)/1000)/60);
end

%% ========================================================================
%                  SECTION 3: ALLAN VARIANCE PER AXIS
%  ========================================================================
% Allan deviation:  sigma_A^2(tau) = 1/(2*(K-1)) * sum( (ybar[i+1] - ybar[i])^2 )
%   where ybar[i] is the mean of the i-th non-overlapping window of size m
%   and tau = m * dt.

fprintf('\n--- Allan deviation per axis ---\n');

axes_names = {'mx_ut','my_ut','mz_ut'};
colors = lines(numel(runs));

fig_allan = figure('Name','Allan deviation (mag)','Position',[100 100 1200 400]);
for ax_i = 1:3
    subplot(1,3,ax_i); hold on; grid on; box on;
    for k = 1:numel(runs)
        if isempty(runs(k).data), continue; end
        x = runs(k).data.(axes_names{ax_i});
        [tau, sigma] = allan_deviation(x, MAG_DT_S, ALLAN_MIN_M, ...
                                       ALLAN_MAX_FRAC, ALLAN_NUM_TAUS);
        loglog(tau, sigma, '-', 'Color', colors(k,:), ...
               'LineWidth', 1.5, 'DisplayName', sprintf('Run %s', runs(k).id));
        runs(k).allan.(axes_names{ax_i}) = struct('tau', tau, 'sigma', sigma);
    end
    set(gca,'XScale','log','YScale','log');
    xlabel('\tau (s)'); ylabel('\sigma_A(\tau) [\muT]');
    title(sprintf('Allan deviation: %s', strrep(axes_names{ax_i},'_','\_')));
    legend('Location','best');
end
saveas(fig_allan, fullfile(plot_dir, 'allan_deviation.png'));
fprintf('  Saved %s\n', fullfile(plot_dir, 'allan_deviation.png'));

%% Fit white-noise floor at tau ≈ 1 s for the noise density parameter
fprintf('\n--- Fitted white-noise density (sigma at tau = 1 s) ---\n');
for k = 1:numel(runs)
    if isempty(runs(k).data), continue; end
    sigma_at_1s = zeros(1,3);
    for ax_i = 1:3
        a = runs(k).allan.(axes_names{ax_i});
        [~, idx] = min(abs(a.tau - 1.0));
        sigma_at_1s(ax_i) = a.sigma(idx);
    end
    runs(k).sigma_1s = sigma_at_1s;
    fprintf('  Run %s: sigma(1s) = [%.4f, %.4f, %.4f] uT\n', ...
            runs(k).id, sigma_at_1s(1), sigma_at_1s(2), sigma_at_1s(3));
end

%% ========================================================================
%                  SECTION 4: PSD WITH TX OVERLAY
%  ========================================================================
fprintf('\n--- PSD per axis (Welch) ---\n');

fig_psd = figure('Name','Mag PSD','Position',[100 600 1200 400]);
for ax_i = 1:3
    subplot(1,3,ax_i); hold on; grid on; box on;
    for k = 1:numel(runs)
        if isempty(runs(k).data), continue; end
        x = runs(k).data.(axes_names{ax_i});
        x = x - mean(x);
        nfft = 2^nextpow2(min(2^14, numel(x)));
        [pxx, f] = pwelch(x, hann(nfft), nfft/2, nfft, MAG_FS_HZ);
        loglog(f, sqrt(pxx), '-', 'Color', colors(k,:), ...
               'LineWidth', 1.2, 'DisplayName', sprintf('Run %s', runs(k).id));
    end
    set(gca,'XScale','log','YScale','log');
    xlabel('Frequency (Hz)'); ylabel('Amplitude (\muT / \surdHz)');
    title(sprintf('PSD: %s', strrep(axes_names{ax_i},'_','\_')));
    legend('Location','best');
end
saveas(fig_psd, fullfile(plot_dir, 'psd_overlay.png'));
fprintf('  Saved %s\n', fullfile(plot_dir, 'psd_overlay.png'));

%% ========================================================================
%             SECTION 5: RADIO-TX EMI DELTA (Runs B & C)
%  ========================================================================
% Use the RADIO_EVT.CSV event timestamps (sub-ms accurate) when available,
% otherwise fall back to the sample-flagged radio_tx column (10 ms grid).
% For each TX burst, compare mag samples inside [start_ms, end_ms] against
% an equal-length quiet window straddling the burst. Output feeds
% casper_mag_radio_interference.m.

fprintf('\n--- Radio TX EMI delta (per axis) ---\n');

QUIET_PAD_MS = 200;  % gap on each side of the burst that must stay quiet

emi_results = struct();
for k = 1:numel(runs)
    if isempty(runs(k).data), continue; end
    T = runs(k).data;
    t_ms = T.t_ms;

    % Build (start_ms, end_ms) list either from events or sample flag
    have_events = ~isempty(runs(k).evt_data);
    if have_events
        E = runs(k).evt_data;
        ev_start = E.start_ms;
        ev_end   = E.end_ms;
    else
        if ~any(T.radio_tx > 0), continue; end
        tx = T.radio_tx > 0;
        d = diff([0; tx(:); 0]);
        s_idx = find(d > 0);
        e_idx = find(d < 0) - 1;
        ev_start = t_ms(s_idx);
        ev_end   = t_ms(e_idx);
    end

    if isempty(ev_start), continue; end
    fprintf('  Run %s: %d TX events (%s)\n', runs(k).id, numel(ev_start), ...
            ternary(have_events, 'event timestamps', 'sample flag'));

    for ax_i = 1:3
        x = T.(axes_names{ax_i});

        tx_means    = nan(numel(ev_start),1);
        tx_stds     = nan(numel(ev_start),1);
        quiet_means = nan(numel(ev_start),1);
        quiet_stds  = nan(numel(ev_start),1);
        tx_lens_ms  = zeros(numel(ev_start),1);

        for j = 1:numel(ev_start)
            s = ev_start(j); e = ev_end(j); dur = e - s;
            tx_mask    = (t_ms >= s) & (t_ms <= e);
            quiet_mask = ((t_ms >= s - QUIET_PAD_MS - dur) & (t_ms < s)) | ...
                         ((t_ms > e) & (t_ms <= e + QUIET_PAD_MS + dur));
            if any(tx_mask) && any(quiet_mask)
                tx_means(j)    = mean(x(tx_mask));
                tx_stds(j)     = std(x(tx_mask));
                quiet_means(j) = mean(x(quiet_mask));
                quiet_stds(j)  = std(x(quiet_mask));
                tx_lens_ms(j)  = dur;
            end
        end

        delta_bias  = mean(tx_means  - quiet_means, 'omitnan');
        delta_sigma = mean(tx_stds   - quiet_stds,  'omitnan');

        emi_results(k).(axes_names{ax_i}).delta_bias_ut  = delta_bias;
        emi_results(k).(axes_names{ax_i}).delta_sigma_ut = delta_sigma;
        emi_results(k).(axes_names{ax_i}).n_events       = numel(ev_start);
        emi_results(k).(axes_names{ax_i}).mean_burst_ms  = mean(tx_lens_ms);

        fprintf('    %s: dBias=%+0.4f uT, dSigma=%+0.4f uT, ' ...
                'mean burst=%.1f ms\n', ...
                axes_names{ax_i}, delta_bias, delta_sigma, mean(tx_lens_ms));
    end
end

%% ========================================================================
%        SECTION 5b: TIME-DOMAIN PLOT WITH TX WINDOWS OVERLAID
%  ========================================================================
% Zoomed-in (first 10 s) view of the mag signal with TX bursts shown as
% shaded vertical patches. This is the visual you want to confirm that
% radio TX is the EMI source (or that it isn't).

for k = 1:numel(runs)
    if isempty(runs(k).data) || isempty(runs(k).evt_data), continue; end
    T = runs(k).data;
    E = runs(k).evt_data;
    t_s = T.t_ms / 1000;

    % Plot the first 10 seconds (or full record if shorter)
    t_max = min(10, t_s(end));
    sel = t_s <= t_max;

    fig = figure('Name', sprintf('Run %s TX overlay', runs(k).id), ...
                 'Position', [100 100 1200 600]);
    for ax_i = 1:3
        subplot(3,1,ax_i); hold on; grid on; box on;
        % TX window patches BEHIND the signal
        evt_mask = E.start_ms <= t_max*1000;
        for j = find(evt_mask).'
            xs = E.start_ms(j) / 1000;
            xe = E.end_ms(j) / 1000;
            patch([xs xe xe xs], ...
                  [min(T.(axes_names{ax_i})) min(T.(axes_names{ax_i})) ...
                   max(T.(axes_names{ax_i})) max(T.(axes_names{ax_i}))], ...
                  [1 0.85 0.85], 'EdgeColor', 'none', ...
                  'FaceAlpha', 0.4, 'HandleVisibility', 'off');
        end
        plot(t_s(sel), T.(axes_names{ax_i})(sel), 'b-', 'LineWidth', 1.0);
        xlabel('t (s)');
        ylabel(sprintf('%s [\\muT]', strrep(axes_names{ax_i},'_','\_')));
        title(sprintf('Run %s — %s (red = radio TX active)', ...
                      runs(k).id, runs(k).label));
        xlim([0 t_max]);
    end
    saveas(fig, fullfile(plot_dir, sprintf('tx_overlay_run_%s.png', runs(k).id)));
    fprintf('  Saved tx_overlay_run_%s.png\n', runs(k).id);
end

%% ========================================================================
%             SECTION 6: HARD-IRON STABILITY
%  ========================================================================
% Compare the mean field vector across the first vs last minute. A drift
% would indicate that the hard-iron calibration is unstable (thermal,
% mechanical, or aging).

fprintf('\n--- Hard-iron stability (first vs last minute) ---\n');
for k = 1:numel(runs)
    if isempty(runs(k).data), continue; end
    T = runs(k).data;
    t = T.t_ms / 1000;
    mask_first = t < 60;
    mask_last  = t > (t(end) - 60);
    if sum(mask_first) < 100 || sum(mask_last) < 100
        fprintf('  Run %s: <60s usable on each end, skipping\n', runs(k).id);
        continue;
    end
    b_first = [mean(T.mx_ut(mask_first)), mean(T.my_ut(mask_first)), mean(T.mz_ut(mask_first))];
    b_last  = [mean(T.mx_ut(mask_last)),  mean(T.my_ut(mask_last)),  mean(T.mz_ut(mask_last))];
    drift = b_last - b_first;
    fprintf('  Run %s: first->last delta = [%+0.3f, %+0.3f, %+0.3f] uT, |drift|=%.3f uT\n', ...
            runs(k).id, drift(1), drift(2), drift(3), norm(drift));
    runs(k).hard_iron_drift_ut = drift;
end

%% ========================================================================
%             SECTION 7: PACKAGE OUTPUT FOR SIMULINK MAG BLOCK
%  ========================================================================
% Pull baseline (Run A) sigma_1s as the white-noise density, average TX
% delta from Runs B/C as the radio interference parameters.

fprintf('\n--- Building mag_noise_params.mat ---\n');

% Pull Run A as the baseline (if present)
baseline = [];
for k = 1:numel(runs)
    if strcmp(runs(k).id, 'A') && ~isempty(runs(k).data)
        baseline = runs(k); break;
    end
end

mag_noise_params = struct();
if ~isempty(baseline) && isfield(baseline,'sigma_1s')
    % Noise density: sigma at tau=1s gives sigma per second of integration.
    % For Allan-white-noise: N (rad/sqrt(s)) = sigma_A(1) for unity-rate samples.
    mag_noise_params.sigma_ut       = baseline.sigma_1s;     % [3]
    mag_noise_params.noise_density_ut_per_sqrt_hz = ...
        baseline.sigma_1s / sqrt(MAG_FS_HZ);                 % [3]
else
    warning('No baseline (Run A) — sigma_ut left empty.');
    mag_noise_params.sigma_ut       = [NaN NaN NaN];
    mag_noise_params.noise_density_ut_per_sqrt_hz = [NaN NaN NaN];
end

% Radio interference: average Run B and Run C delta bias/sigma
mag_noise_params.tx_delta_bias_ut  = [NaN NaN NaN];
mag_noise_params.tx_delta_sigma_ut = [NaN NaN NaN];
emi_runs = []; tally = 0;
for k = 1:numel(runs)
    if (strcmp(runs(k).id,'B') || strcmp(runs(k).id,'C')) && ...
       k <= numel(emi_results) && ~isempty(fieldnames(emi_results(k)))
        emi_runs = [emi_runs, k]; %#ok<AGROW>
    end
end
if ~isempty(emi_runs)
    db = zeros(numel(emi_runs),3);
    ds = zeros(numel(emi_runs),3);
    for ii = 1:numel(emi_runs)
        k = emi_runs(ii);
        for ax_i = 1:3
            db(ii,ax_i) = emi_results(k).(axes_names{ax_i}).delta_bias_ut;
            ds(ii,ax_i) = emi_results(k).(axes_names{ax_i}).delta_sigma_ut;
        end
    end
    mag_noise_params.tx_delta_bias_ut  = mean(db, 1);
    mag_noise_params.tx_delta_sigma_ut = mean(ds, 1);
    tally = numel(emi_runs);
end
mag_noise_params.tx_delta_runs_used = tally;

% TX burst statistics (from RADIO_EVT.csv) — feed casper_radio_tx_schedule.m
tx_durations_ms = [];
for k = 1:numel(runs)
    if ~isempty(runs(k).evt_data)
        tx_durations_ms = [tx_durations_ms; runs(k).evt_data.duration_ms]; %#ok<AGROW>
    end
end
if ~isempty(tx_durations_ms)
    mag_noise_params.tx_burst_ms_mean   = mean(tx_durations_ms);
    mag_noise_params.tx_burst_ms_std    = std(tx_durations_ms);
    mag_noise_params.tx_burst_ms_min    = min(tx_durations_ms);
    mag_noise_params.tx_burst_ms_max    = max(tx_durations_ms);
    mag_noise_params.tx_burst_count     = numel(tx_durations_ms);
end

% Sample rate so the Simulink block knows what dt the params correspond to
mag_noise_params.sample_rate_hz = MAG_FS_HZ;

save(PARAMS_OUT, 'mag_noise_params');
fprintf('  Saved %s\n', PARAMS_OUT);
disp(mag_noise_params);

fprintf('\nDone.\n');

%% ========================================================================
%                          LOCAL FUNCTIONS
%  ========================================================================

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function [tau, sigma] = allan_deviation(x, dt, m_min, max_frac, n_taus)
    % Overlapping Allan deviation via the bin-averaging method.
    %   sigma_A(tau)^2 = 1/(2*(K-1)) * sum( (ybar[i+1] - ybar[i])^2 )
    % where ybar[i] = mean of i-th non-overlapping window of size m,
    % tau = m * dt, K = floor(N/m).
    N = numel(x);
    m_max = floor(N * max_frac);
    if m_max < m_min, m_max = m_min; end
    m_vals = unique(round(logspace(log10(m_min), log10(m_max), n_taus)));
    m_vals(m_vals < 1) = [];
    tau   = m_vals * dt;
    sigma = zeros(size(m_vals));
    for ii = 1:numel(m_vals)
        m = m_vals(ii);
        K = floor(N / m);
        if K < 2
            sigma(ii) = NaN; continue;
        end
        Y = reshape(x(1:K*m), m, K);
        ybar = mean(Y, 1);
        sigma(ii) = sqrt(0.5 * mean(diff(ybar).^2));
    end
end
