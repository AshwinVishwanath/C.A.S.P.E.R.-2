%% Casper Sensor Characterization — Combined Analysis
%  Run this script after collecting all 5 CSV files from the data
%  collection firmware. It performs, in order:
%    1. Data quality check (sample rates, dropouts, gravity, gyro bias)
%    2. Allan variance analysis (LSM6 gyro/accel, ADXL372 accel)
%       - LSM6: segments data around timing gaps, uses longest clean segment
%       - Gyro/Accel: removes outlier spikes via rmoutliers() before analysis
%    3. Barometer noise characterization (MS5611 at 3 OSR settings)
%
%  Inputs:
%    IMU.CSV                    — timestamp_us, ax, ay, az, gx, gy, gz, temp (raw int16)
%    ADXL.CSV                   — timestamp_us, ax, ay, az (12-bit signed, 100 mg/LSB)
%    BARO_HI.CSV                — timestamp_us, pressure, temperature (OSR 4096)
%    BARO_MD.CSV                — same (OSR 1024)
%    BARO_LO.CSV                — same (OSR 256)
%
%  Outputs:
%    noise_params.mat           — IMU noise parameters for Q matrix design
%    baro_noise_params.mat      — Baro noise parameters for R matrix design
%    Plots saved to plots/ subdirectory
%
%  Sensor configurations:
%    LSM6DSO32: ±32g (0.976 mg/LSB), ±2000 dps (70 mdps/LSB), 833 Hz
%    ADXL372:   ±200g (100 mg/LSB on 12-bit), 800 Hz
%    MS5611:    OSR 4096/1024/256, SPI

clear; clc; close all;

% Create plots output directory
plot_dir = 'plots';
if ~exist(plot_dir, 'dir'), mkdir(plot_dir); end

%% ========================================================================
%                         USER CONFIGURATION
%  ========================================================================

% File paths
lsm6_file       = 'IMU.CSV';
adxl_file       = 'ADXL.CSV';
baro_high_file  = 'BARO_HI.CSV';
baro_med_file   = 'BARO_MD.CSV';
baro_low_file   = 'BARO_LO.CSV';

% LSM6DSO32 conversion
LSM6_ACCEL_MPS2_PER_LSB = 0.976e-3 * 9.80665;     % ±32g range
LSM6_GYRO_RADS_PER_LSB  = 70e-3 * (pi / 180);      % ±2000 dps range
LSM6_TEMP_DEGC_PER_LSB  = 1 / 256;                  % 256 LSB/°C, 0 = 25°C
LSM6_TEMP_OFFSET_DEGC   = 25;
LSM6_FS_HZ = 833;

% ADXL372 conversion (firmware already applies >> 4 to get 12-bit values)
ADXL_ACCEL_MPS2_PER_LSB = 100e-3 * 9.80665;        % ±200g, 12-bit
ADXL_FS_HZ = 800;

% LSM6 gap detection threshold — samples with dt > this are gap boundaries
% NOTE: FATFS buffer flushes cause periodic ~5-10 ms gaps every ~117 samples.
% Set threshold above the flush duration to avoid fragmenting the data.
LSM6_GAP_THRESHOLD_MS = 50;  % Ignore FATFS flush gaps (~5-10 ms); only flag real dropouts

% Minimum segment length for Allan variance (samples)
MIN_SEGMENT_SAMPLES = 1000;  % ~1.2 s at 833 Hz

% Constants
GRAVITY_MPS2 = 9.80665;
PA_TO_M_SCALE = 1 / 12.01;  % Approximate Pa → m at sea level ISA

% Baro compensation flag
APPLY_BARO_COMPENSATION = true;

% MS5611 PROM coefficients (read from sensor via CDC at startup)
PROM.C1 = 42791;  PROM.C2 = 45898;  PROM.C3 = 25578;
PROM.C4 = 22915;  PROM.C5 = 34429;  PROM.C6 = 27495;

%% ========================================================================
%                       SECTION 1: LOAD ALL DATA
%  ========================================================================
fprintf('================================================================\n');
fprintf('  CASPER SENSOR CHARACTERIZATION\n');
fprintf('================================================================\n\n');

axis_labels = {'X', 'Y', 'Z'};

% --- LSM6DSO32 ---
fprintf('Loading LSM6DSO32 from %s...\n', lsm6_file);
lsm6_raw   = readmatrix(lsm6_file);
lsm6_t_s   = (lsm6_raw(:,1) - lsm6_raw(1,1)) * 1e-6;
lsm6_accel = lsm6_raw(:, 2:4) * LSM6_ACCEL_MPS2_PER_LSB;
lsm6_gyro  = lsm6_raw(:, 5:7) * LSM6_GYRO_RADS_PER_LSB;
lsm6_n     = size(lsm6_raw, 1);
lsm6_fs    = 1 / median(diff(lsm6_t_s));
fprintf('  Samples: %d | Duration: %.1f s | Fs: %.1f Hz (expected %d)\n', ...
    lsm6_n, lsm6_t_s(end), lsm6_fs, LSM6_FS_HZ);

% --- ADXL372 ---
fprintf('Loading ADXL372 from %s...\n', adxl_file);
adxl_raw    = readmatrix(adxl_file);
adxl_t_s    = (adxl_raw(:,1) - adxl_raw(1,1)) * 1e-6;
adxl_accel  = adxl_raw(:, 2:4) * ADXL_ACCEL_MPS2_PER_LSB;
adxl_n      = size(adxl_raw, 1);
adxl_fs     = 1 / median(diff(adxl_t_s));
fprintf('  Samples: %d | Duration: %.1f s | Fs: %.1f Hz (expected %d)\n', ...
    adxl_n, adxl_t_s(end), adxl_fs, ADXL_FS_HZ);

% --- Barometer (3 OSR settings) ---
baro_files = {baro_high_file, baro_med_file, baro_low_file};
baro_osr   = [4096, 1024, 256];
baro_labels = {'OSR 4096', 'OSR 1024', 'OSR 256'};
baro_colors = {'b', 'r', 'k'};
baro = struct();

for k = 1:3
    fprintf('Loading baro %s from %s...\n', baro_labels{k}, baro_files{k});
    raw = readmatrix(baro_files{k});
    baro(k).t_s   = (raw(:,1) - raw(1,1)) * 1e-6;
    baro(k).n     = size(raw, 1);
    baro(k).fs    = 1 / median(diff(baro(k).t_s));
    baro(k).osr   = baro_osr(k);

    if APPLY_BARO_COMPENSATION
        [baro(k).press_pa, baro(k).temp_c] = ms5611_compensate( ...
            raw(:,2), raw(:,3), PROM);
    else
        baro(k).press_pa = raw(:, 2);
        baro(k).temp_c   = raw(:, 3);
    end

    fprintf('  Samples: %d | Duration: %.1f s | Fs: %.1f Hz\n', ...
        baro(k).n, baro(k).t_s(end), baro(k).fs);
end

%% ========================================================================
%                    SECTION 2: DATA QUALITY CHECK
%  ========================================================================
fprintf('\n================================================================\n');
fprintf('  DATA QUALITY CHECK\n');
fprintf('================================================================\n');

% --- Sample rate regularity ---
fprintf('\n--- Sample Timing ---\n');

lsm6_dt = diff(lsm6_t_s);
adxl_dt = diff(adxl_t_s);

fprintf('LSM6 dt: mean=%.4f ms, σ=%.4f ms, min=%.4f ms, max=%.4f ms\n', ...
    mean(lsm6_dt)*1e3, std(lsm6_dt)*1e3, min(lsm6_dt)*1e3, max(lsm6_dt)*1e3);
fprintf('ADXL dt: mean=%.4f ms, σ=%.4f ms, min=%.4f ms, max=%.4f ms\n', ...
    mean(adxl_dt)*1e3, std(adxl_dt)*1e3, min(adxl_dt)*1e3, max(adxl_dt)*1e3);

for k = 1:3
    bdt = diff(baro(k).t_s);
    fprintf('Baro %s dt: mean=%.4f ms, σ=%.4f ms\n', ...
        baro_labels{k}, mean(bdt)*1e3, std(bdt)*1e3);
end

% --- Dropout detection ---
fprintf('\n--- Dropout Check ---\n');
lsm6_dropouts = sum(lsm6_dt > 2/LSM6_FS_HZ);
adxl_dropouts = sum(adxl_dt > 2/ADXL_FS_HZ);

if lsm6_dropouts > 0
    warning('LSM6: %d dropouts detected (dt > 2x expected)', lsm6_dropouts);
else
    fprintf('LSM6: No dropouts ✓\n');
end
if adxl_dropouts > 0
    warning('ADXL: %d dropouts detected (dt > 2x expected)', adxl_dropouts);
else
    fprintf('ADXL: No dropouts ✓\n');
end

% --- LSM6 gap analysis ---
fprintf('\n--- LSM6 Gap Analysis ---\n');
gap_threshold_s = LSM6_GAP_THRESHOLD_MS * 1e-3;
gap_indices = find(lsm6_dt > gap_threshold_s);
n_gaps = length(gap_indices);
fprintf('  Gap threshold: %.1f ms\n', LSM6_GAP_THRESHOLD_MS);
fprintf('  Gaps found: %d\n', n_gaps);

if n_gaps > 0
    gap_sizes_ms = lsm6_dt(gap_indices) * 1e3;
    fprintf('  Gap sizes: mean=%.1f ms, max=%.1f ms, min=%.1f ms\n', ...
        mean(gap_sizes_ms), max(gap_sizes_ms), min(gap_sizes_ms));
end

% Use all LSM6 data — periodic FATFS flush gaps are short and uniform,
% so they don't meaningfully affect Allan variance or noise analysis.
% The nominal sample rate is used for frequency-domain calculations.
lsm6_seg_start = 1;
lsm6_seg_end   = lsm6_n;
lsm6_seg_n     = lsm6_n;

fprintf('  Using ALL data for analysis: %d samples (%.2f s) at nominal %.0f Hz\n', ...
    lsm6_seg_n, lsm6_seg_n/LSM6_FS_HZ, LSM6_FS_HZ);

% Extract (full dataset)
lsm6_seg_accel = lsm6_accel;
lsm6_seg_gyro  = lsm6_gyro;
lsm6_seg_t     = lsm6_t_s;

% --- Gyro outlier removal (replace with local median to preserve length) ---
fprintf('\n--- Gyro Outlier Removal ---\n');
lsm6_seg_gyro_clean = lsm6_seg_gyro;
for ax = 1:3
    [cleaned, tf] = filloutliers(lsm6_seg_gyro(:,ax), 'linear', 'movmedian', 50);
    n_outliers = sum(tf);
    if n_outliers > 0
        fprintf('  Gyro %s: replaced %d outliers (%.3f%%)\n', ...
            axis_labels{ax}, n_outliers, n_outliers/lsm6_seg_n*100);
        lsm6_seg_gyro_clean(:,ax) = cleaned;
    else
        fprintf('  Gyro %s: no outliers\n', axis_labels{ax});
    end
end

% --- Accel outlier removal ---
fprintf('\n--- Accel Outlier Removal (LSM6 segment) ---\n');
lsm6_seg_accel_clean = lsm6_seg_accel;
for ax = 1:3
    [cleaned, tf] = filloutliers(lsm6_seg_accel(:,ax), 'linear', 'movmedian', 50);
    n_outliers = sum(tf);
    if n_outliers > 0
        fprintf('  Accel %s: replaced %d outliers (%.3f%%)\n', ...
            axis_labels{ax}, n_outliers, n_outliers/lsm6_seg_n*100);
        lsm6_seg_accel_clean(:,ax) = cleaned;
    else
        fprintf('  Accel %s: no outliers\n', axis_labels{ax});
    end
end

% --- Gravity vector ---
fprintf('\n--- Gravity Vector ---\n');

lsm6_a_mean = mean(lsm6_accel, 1);
lsm6_a_norm = norm(lsm6_a_mean);
fprintf('LSM6 mean: [%.4f, %.4f, %.4f] m/s² | ‖g‖ = %.4f m/s²\n', ...
    lsm6_a_mean, lsm6_a_norm);

adxl_a_mean = mean(adxl_accel, 1);
adxl_a_norm = norm(adxl_a_mean);
fprintf('ADXL mean: [%.4f, %.4f, %.4f] m/s² | ‖g‖ = %.4f m/s²\n', ...
    adxl_a_mean, adxl_a_norm);

if abs(lsm6_a_norm - GRAVITY_MPS2) / GRAVITY_MPS2 > 0.05
    warning('LSM6 gravity magnitude off by >5%%');
else
    fprintf('LSM6 gravity magnitude OK ✓\n');
end
if abs(adxl_a_norm - GRAVITY_MPS2) / GRAVITY_MPS2 > 0.10
    warning('ADXL gravity magnitude off by >10%% (expected for low-res at 1g)');
else
    fprintf('ADXL gravity magnitude OK ✓\n');
end

[~, gax_lsm6] = max(abs(lsm6_a_mean));
[~, gax_adxl] = max(abs(adxl_a_mean));
fprintf('LSM6 gravity axis: %s (%.2f m/s²)\n', axis_labels{gax_lsm6}, lsm6_a_mean(gax_lsm6));
fprintf('ADXL gravity axis: %s (%.2f m/s²)\n', axis_labels{gax_adxl}, adxl_a_mean(gax_adxl));

% --- Gyro static bias (from cleaned segment) ---
fprintf('\n--- Gyro Static Bias ---\n');
g_mean = mean(lsm6_seg_gyro_clean, 1);
g_std  = std(lsm6_seg_gyro_clean, 0, 1);
fprintf('Mean:  [%.6f, %.6f, %.6f] rad/s  ([%.4f, %.4f, %.4f] °/s)\n', ...
    g_mean, g_mean * 180/pi);
fprintf('σ:     [%.6f, %.6f, %.6f] rad/s\n', g_std);

if any(abs(g_mean) > 0.05)
    warning('Gyro bias high — board may not have been stationary');
else
    fprintf('Gyro bias OK ✓\n');
end

% --- Baro pressure sanity ---
fprintf('\n--- Baro Pressure ---\n');
for k = 1:3
    fprintf('%s: mean = %.1f Pa, σ = %.4f Pa\n', ...
        baro_labels{k}, mean(baro(k).press_pa), std(baro(k).press_pa));
end

% --- Quality check plots ---
figure('Name', 'QC: Sample Timing', 'Position', [50 50 1200 500]);
subplot(2,1,1);
plot(lsm6_t_s(1:end-1), lsm6_dt*1e3, 'b.', 'MarkerSize', 1); hold on;
yline(LSM6_GAP_THRESHOLD_MS, 'r--', 'Gap threshold', 'LineWidth', 1.5);
seg_t_start = lsm6_t_s(lsm6_seg_start);
seg_t_end   = lsm6_t_s(lsm6_seg_end);
yl = ylim;
patch([seg_t_start seg_t_end seg_t_end seg_t_start], ...
    [yl(1) yl(1) yl(2) yl(2)], 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
xlabel('Time (s)'); ylabel('dt (ms)');
title(sprintf('LSM6 Inter-sample Time (expected %.3f ms) — green = selected segment', ...
    1/LSM6_FS_HZ*1e3));
grid on; hold off;
subplot(2,1,2);
plot(adxl_t_s(1:end-1), adxl_dt*1e3, 'r.', 'MarkerSize', 1);
xlabel('Time (s)'); ylabel('dt (ms)');
title(sprintf('ADXL Inter-sample Time (expected %.3f ms)', 1/ADXL_FS_HZ*1e3));
grid on;
saveas(gcf, fullfile(plot_dir, 'qc_sample_timing.png'));

figure('Name', 'QC: Raw IMU', 'Position', [50 50 1400 900]);
subplot(4,1,1);
plot(lsm6_t_s, lsm6_accel);
xlabel('Time (s)'); ylabel('m/s²');
title('LSM6 Accelerometer'); legend('X','Y','Z'); grid on;
subplot(4,1,2);
plot(lsm6_t_s, lsm6_gyro * 180/pi);
xlabel('Time (s)'); ylabel('°/s');
title('LSM6 Gyroscope (raw)'); legend('X','Y','Z'); grid on;
subplot(4,1,3);
plot(lsm6_seg_t, lsm6_seg_gyro_clean * 180/pi);
xlabel('Time (s)'); ylabel('°/s');
title('LSM6 Gyroscope (cleaned segment — used for Allan variance)');
legend('X','Y','Z'); grid on;
subplot(4,1,4);
plot(adxl_t_s, adxl_accel);
xlabel('Time (s)'); ylabel('m/s²');
title('ADXL372 Accelerometer'); legend('X','Y','Z'); grid on;
saveas(gcf, fullfile(plot_dir, 'qc_raw_imu.png'));

figure('Name', 'QC: Raw Baro', 'Position', [50 50 1200 800]);
for k = 1:3
    subplot(3,1,k);
    plot(baro(k).t_s, baro(k).press_pa);
    xlabel('Time (s)'); ylabel('Pressure (Pa)');
    title(sprintf('MS5611 %s (Fs = %.1f Hz)', baro_labels{k}, baro(k).fs));
    grid on;
end
saveas(gcf, fullfile(plot_dir, 'qc_raw_baro.png'));

figure('Name', 'QC: Noise Histograms', 'Position', [50 50 1400 500]);
subplot(1,3,1);
histogram(lsm6_seg_accel_clean(:,gax_lsm6) - mean(lsm6_seg_accel_clean(:,gax_lsm6)), 100);
xlabel('m/s² (zero-mean)'); ylabel('Count');
title(sprintf('LSM6 Accel %s (cleaned)', axis_labels{gax_lsm6}));
subplot(1,3,2);
histogram(lsm6_seg_gyro_clean(:,1) - mean(lsm6_seg_gyro_clean(:,1)), 100);
xlabel('rad/s (zero-mean)'); ylabel('Count');
title('LSM6 Gyro X (cleaned)');
subplot(1,3,3);
histogram(baro(1).press_pa - mean(baro(1).press_pa), 100);
xlabel('Pa (zero-mean)'); ylabel('Count');
title('Baro OSR 4096');
saveas(gcf, fullfile(plot_dir, 'qc_noise_histograms.png'));

fprintf('\n--- Quality check complete. Review plots before proceeding. ---\n');
fprintf('Press any key to continue to Allan variance analysis...\n');
pause;

%% ========================================================================
%                 SECTION 3: ALLAN VARIANCE — IMU
%  ========================================================================
fprintf('\n================================================================\n');
fprintf('  ALLAN VARIANCE ANALYSIS\n');
fprintf('================================================================\n');

% Remove mean from cleaned segment data
lsm6_accel_zm = lsm6_seg_accel_clean - mean(lsm6_seg_accel_clean, 1);
lsm6_gyro_zm  = lsm6_seg_gyro_clean  - mean(lsm6_seg_gyro_clean, 1);
adxl_accel_zm = adxl_accel            - mean(adxl_accel, 1);

fprintf('\nUsing LSM6 segment: %d samples (%.2f s) at %.1f Hz\n', ...
    lsm6_seg_n, lsm6_seg_n/LSM6_FS_HZ, LSM6_FS_HZ);

% --- LSM6 Gyroscope ---
fprintf('\nLSM6 Gyroscope Allan Deviation:\n');

gyro_arw_rads = zeros(1, 3);
gyro_bi_rads  = zeros(1, 3);

figure('Name', 'Allan Dev: LSM6 Gyro', 'Position', [50 50 1400 450]);

for ax = 1:3
    [avar, tau] = allanvar(lsm6_gyro_zm(:,ax), 'octave', LSM6_FS_HZ);
    adev = sqrt(avar);

    gyro_arw_rads(ax) = 10^interp1(log10(tau), log10(adev), 0, 'linear', 'extrap');
    [gyro_bi_rads(ax), bi_idx] = min(adev);

    subplot(1,3,ax);
    loglog(tau, adev, 'b-', 'LineWidth', 1.5); hold on;
    loglog(1, gyro_arw_rads(ax), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    loglog(tau(bi_idx), gyro_bi_rads(ax), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
    tau_ref = tau(tau >= min(tau) & tau <= max(tau));
    loglog(tau_ref, gyro_arw_rads(ax)./sqrt(tau_ref), 'r--', 'LineWidth', 1);
    xlabel('\tau (s)'); ylabel('\sigma(\tau) (rad/s)');
    title(sprintf('Gyro %s', axis_labels{ax}));
    legend('ADEV','ARW@\tau=1','BI','ARW slope','Location','sw','FontSize',7);
    grid on; hold off;

    fprintf('  %s: ARW = %.4e rad/√s (%.4f °/√hr) | BI = %.4e rad/s (%.4f °/hr)\n', ...
        axis_labels{ax}, ...
        gyro_arw_rads(ax), gyro_arw_rads(ax)*(180/pi)*60, ...
        gyro_bi_rads(ax), gyro_bi_rads(ax)*(180/pi)*3600);
end
sgtitle('LSM6DSO32 Gyroscope — Allan Deviation (cleaned segment)');
saveas(gcf, fullfile(plot_dir, 'allan_lsm6_gyro.png'));

% --- LSM6 Accelerometer ---
fprintf('\nLSM6 Accelerometer Allan Deviation:\n');

lsm6_vrw_mps2 = zeros(1, 3);
lsm6_abi_mps2 = zeros(1, 3);

figure('Name', 'Allan Dev: LSM6 Accel', 'Position', [50 50 1400 450]);

for ax = 1:3
    [avar, tau] = allanvar(lsm6_accel_zm(:,ax), 'octave', LSM6_FS_HZ);
    adev = sqrt(avar);

    lsm6_vrw_mps2(ax) = 10^interp1(log10(tau), log10(adev), 0, 'linear', 'extrap');
    [lsm6_abi_mps2(ax), bi_idx] = min(adev);

    subplot(1,3,ax);
    loglog(tau, adev, 'b-', 'LineWidth', 1.5); hold on;
    loglog(1, lsm6_vrw_mps2(ax), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    loglog(tau(bi_idx), lsm6_abi_mps2(ax), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
    tau_ref = tau(tau >= min(tau) & tau <= max(tau));
    loglog(tau_ref, lsm6_vrw_mps2(ax)./sqrt(tau_ref), 'r--', 'LineWidth', 1);
    xlabel('\tau (s)'); ylabel('\sigma(\tau) (m/s²)');
    title(sprintf('Accel %s', axis_labels{ax}));
    legend('ADEV','VRW@\tau=1','BI','VRW slope','Location','sw','FontSize',7);
    grid on; hold off;

    fprintf('  %s: VRW = %.4e m/s/√s | BI = %.4e m/s² (%.4f mg)\n', ...
        axis_labels{ax}, ...
        lsm6_vrw_mps2(ax), lsm6_abi_mps2(ax), ...
        lsm6_abi_mps2(ax)/9.80665*1000);
end
sgtitle('LSM6DSO32 Accelerometer — Allan Deviation (cleaned segment)');
saveas(gcf, fullfile(plot_dir, 'allan_lsm6_accel.png'));

% --- ADXL372 Accelerometer ---
fprintf('\nADXL372 Accelerometer Allan Deviation:\n');

adxl_vrw_mps2 = zeros(1, 3);
adxl_abi_mps2 = zeros(1, 3);

figure('Name', 'Allan Dev: ADXL372', 'Position', [50 50 1400 450]);

for ax = 1:3
    [avar, tau] = allanvar(adxl_accel_zm(:,ax), 'octave', ADXL_FS_HZ);
    adev = sqrt(avar);

    adxl_vrw_mps2(ax) = 10^interp1(log10(tau), log10(adev), 0, 'linear', 'extrap');
    [adxl_abi_mps2(ax), bi_idx] = min(adev);

    subplot(1,3,ax);
    loglog(tau, adev, 'b-', 'LineWidth', 1.5); hold on;
    loglog(1, adxl_vrw_mps2(ax), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    loglog(tau(bi_idx), adxl_abi_mps2(ax), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
    tau_ref = tau(tau >= min(tau) & tau <= max(tau));
    loglog(tau_ref, adxl_vrw_mps2(ax)./sqrt(tau_ref), 'r--', 'LineWidth', 1);
    xlabel('\tau (s)'); ylabel('\sigma(\tau) (m/s²)');
    title(sprintf('Accel %s', axis_labels{ax}));
    legend('ADEV','VRW@\tau=1','BI','VRW slope','Location','sw','FontSize',7);
    grid on; hold off;

    fprintf('  ADXL %s: VRW = %.4e m/s/√s | BI = %.4e m/s² (%.4f mg)\n', ...
        axis_labels{ax}, ...
        adxl_vrw_mps2(ax), adxl_abi_mps2(ax), ...
        adxl_abi_mps2(ax)/9.80665*1000);
end
sgtitle('ADXL372 Accelerometer — Allan Deviation');
saveas(gcf, fullfile(plot_dir, 'allan_adxl372.png'));

%% ========================================================================
%              SECTION 4: BAROMETER NOISE CHARACTERIZATION
%  ========================================================================
fprintf('\n================================================================\n');
fprintf('  BAROMETER NOISE CHARACTERIZATION\n');
fprintf('================================================================\n');

figure('Name', 'Baro Allan Dev', 'Position', [50 50 1400 450]);
figure('Name', 'Baro PSD', 'Position', [50 50 1400 450]);

baro_noise = struct();

for k = 1:3
    press_zm = baro(k).press_pa - mean(baro(k).press_pa);
    alt_zm   = press_zm * (-PA_TO_M_SCALE);

    press_std = std(press_zm);
    alt_std   = std(alt_zm);

    % Allan deviation
    [avar, tau] = allanvar(press_zm, 'octave', baro(k).fs);
    adev = sqrt(avar);

    % PSD via Welch
    nfft = min(2^nextpow2(baro(k).n / 4), 2^14);
    [pxx, f_psd] = pwelch(press_zm, hanning(nfft), nfft/2, nfft, baro(k).fs);

    % Store
    baro_noise(k).osr          = baro(k).osr;
    baro_noise(k).fs_hz        = baro(k).fs;
    baro_noise(k).press_std_pa = press_std;
    baro_noise(k).alt_std_m    = alt_std;
    baro_noise(k).R_alt_m2     = alt_std^2;
    baro_noise(k).adev_tau     = tau;
    baro_noise(k).adev         = adev;

    fprintf('%s: σ_P = %.4f Pa | σ_Alt = %.4f m | R_alt = %.6f m²\n', ...
        baro_labels{k}, press_std, alt_std, alt_std^2);

    figure(findobj('Name', 'Baro Allan Dev'));
    subplot(1,3,k);
    loglog(tau, adev, [baro_colors{k} '-'], 'LineWidth', 1.5);
    xlabel('\tau (s)'); ylabel('\sigma(\tau) (Pa)');
    title(sprintf('%s', baro_labels{k})); grid on;

    figure(findobj('Name', 'Baro PSD'));
    subplot(1,3,k);
    loglog(f_psd, sqrt(pxx), [baro_colors{k} '-'], 'LineWidth', 1.0);
    xlabel('Frequency (Hz)'); ylabel('PSD (Pa/√Hz)');
    title(sprintf('%s', baro_labels{k})); grid on;
end

figure(findobj('Name', 'Baro Allan Dev'));
sgtitle('MS5611 Barometer — Allan Deviation');
saveas(gcf, fullfile(plot_dir, 'baro_allan_dev.png'));
figure(findobj('Name', 'Baro PSD'));
sgtitle('MS5611 Barometer — Power Spectral Density');
saveas(gcf, fullfile(plot_dir, 'baro_psd.png'));

figure('Name', 'Baro Allan Dev Overlay', 'Position', [50 50 800 600]);
hold on;
for k = 1:3
    loglog(baro_noise(k).adev_tau, baro_noise(k).adev, ...
        [baro_colors{k} '-'], 'LineWidth', 1.5, 'DisplayName', baro_labels{k});
end
xlabel('\tau (s)'); ylabel('\sigma(\tau) (Pa)');
title('MS5611 Allan Deviation — All OSR');
legend('Location','best'); grid on; hold off;
saveas(gcf, fullfile(plot_dir, 'baro_allan_overlay.png'));

%% ========================================================================
%                       SECTION 5: SUMMARY
%  ========================================================================
fprintf('\n================================================================\n');
fprintf('                 NOISE PARAMETER SUMMARY\n');
fprintf('================================================================\n');

fprintf('\nLSM6 data: full dataset (%d samples, %.2f s) with outlier removal\n', ...
    lsm6_seg_n, lsm6_seg_n/LSM6_FS_HZ);

fprintf('\nLSM6DSO32 Gyroscope (→ Q_gyro):\n');
fprintf('  %-5s  ARW (rad/√s)     ARW (°/√hr)     BI (rad/s)      BI (°/hr)\n', 'Axis');
for ax = 1:3
    fprintf('  %-5s  %.4e        %.4f           %.4e       %.4f\n', ...
        axis_labels{ax}, ...
        gyro_arw_rads(ax), gyro_arw_rads(ax)*(180/pi)*60, ...
        gyro_bi_rads(ax), gyro_bi_rads(ax)*(180/pi)*3600);
end

fprintf('\nLSM6DSO32 Accelerometer (→ Q_accel):\n');
fprintf('  %-5s  VRW (m/s/√s)    BI (m/s²)       BI (mg)\n', 'Axis');
for ax = 1:3
    fprintf('  %-5s  %.4e        %.4e       %.4f\n', ...
        axis_labels{ax}, lsm6_vrw_mps2(ax), lsm6_abi_mps2(ax), ...
        lsm6_abi_mps2(ax)/9.80665*1000);
end

fprintf('\nADXL372 Accelerometer (reference only):\n');
fprintf('  %-5s  VRW (m/s/√s)    BI (m/s²)       BI (mg)\n', 'Axis');
for ax = 1:3
    fprintf('  %-5s  %.4e        %.4e       %.4f\n', ...
        axis_labels{ax}, adxl_vrw_mps2(ax), adxl_abi_mps2(ax), ...
        adxl_abi_mps2(ax)/9.80665*1000);
end

fprintf('\nMS5611 Barometer (→ R_baro):\n');
fprintf('  %-10s  Fs (Hz)   σ_P (Pa)   σ_Alt (m)   R_alt (m²)\n', 'OSR');
for k = 1:3
    fprintf('  %-10s  %-8.1f  %-9.4f  %-10.4f  %.6f\n', ...
        baro_labels{k}, baro_noise(k).fs_hz, ...
        baro_noise(k).press_std_pa, baro_noise(k).alt_std_m, ...
        baro_noise(k).R_alt_m2);
end

%% ========================================================================
%                       SECTION 6: SAVE RESULTS
%  ========================================================================

noise_params = struct();
noise_params.lsm6_gyro_arw_rads  = gyro_arw_rads;
noise_params.lsm6_gyro_bi_rads   = gyro_bi_rads;
noise_params.lsm6_accel_vrw_mps2 = lsm6_vrw_mps2;
noise_params.lsm6_accel_bi_mps2  = lsm6_abi_mps2;
noise_params.adxl_accel_vrw_mps2 = adxl_vrw_mps2;
noise_params.adxl_accel_bi_mps2  = adxl_abi_mps2;
noise_params.lsm6_fs_hz          = LSM6_FS_HZ;
noise_params.adxl_fs_hz          = ADXL_FS_HZ;
noise_params.lsm6_segment_used   = 'all';
noise_params.lsm6_segment_samples = lsm6_seg_n;

baro_noise_params = struct();
for k = 1:3
    field = sprintf('osr_%d', baro_noise(k).osr);
    baro_noise_params.(field).fs_hz        = baro_noise(k).fs_hz;
    baro_noise_params.(field).press_std_pa = baro_noise(k).press_std_pa;
    baro_noise_params.(field).alt_std_m    = baro_noise(k).alt_std_m;
    baro_noise_params.(field).R_alt_m2     = baro_noise(k).R_alt_m2;
end

save('noise_params.mat', 'noise_params');
save('baro_noise_params.mat', 'baro_noise_params', 'baro_noise');

fprintf('\nSaved: noise_params.mat, baro_noise_params.mat\n');
fprintf('\n================================================================\n');
fprintf('  CHARACTERIZATION COMPLETE\n');
fprintf('  Next: use noise_params and baro_noise_params for EKF Q/R design\n');
fprintf('================================================================\n');

%% ========================================================================
%                    LOCAL FUNCTIONS
%  ========================================================================

function [press_pa, temp_c] = ms5611_compensate(d1, d2, prom)
% MS5611 second-order compensation per datasheet
% Input:  d1 = raw pressure ADC, d2 = raw temperature ADC, prom = struct
% Output: press_pa in Pa, temp_c in °C

    dT   = d2 - prom.C5 * 2^8;
    TEMP = 2000 + dT * prom.C6 / 2^23;

    OFF  = prom.C2 * 2^16 + (prom.C4 .* dT) / 2^7;
    SENS = prom.C1 * 2^15 + (prom.C3 .* dT) / 2^8;

    % Second-order compensation for T < 20°C
    mask_cold = TEMP < 2000;
    if any(mask_cold)
        T2   = dT.^2 / 2^31;
        OFF2 = 5 * (TEMP - 2000).^2 / 2;
        SENS2= 5 * (TEMP - 2000).^2 / 4;

        mask_vcold = TEMP < -1500;
        if any(mask_vcold)
            OFF2(mask_vcold)  = OFF2(mask_vcold)  + 7*(TEMP(mask_vcold)+1500).^2;
            SENS2(mask_vcold) = SENS2(mask_vcold) + 11*(TEMP(mask_vcold)+1500).^2/2;
        end

        TEMP(mask_cold) = TEMP(mask_cold) - T2(mask_cold);
        OFF(mask_cold)  = OFF(mask_cold)  - OFF2(mask_cold);
        SENS(mask_cold) = SENS(mask_cold) - SENS2(mask_cold);
    end

    press_pa = (d1 .* SENS / 2^21 - OFF) / 2^15 / 100;
    temp_c   = TEMP / 100;
end
