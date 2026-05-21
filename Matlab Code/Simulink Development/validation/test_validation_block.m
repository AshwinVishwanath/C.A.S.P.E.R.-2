function test_validation_block()
%TEST_VALIDATION_BLOCK Unit-test the T10 validation block + metrics suite.
%
% Synopsis:
%   test_validation_block()
%
% Exercises every metric in casper_compute_all_metrics on synthetic truth +
% estimator data, plus an error-injection sweep (perfect, +20 m apogee, +3 m/s
% burnout vel, non-deterministic two-run). Generates all 12 plots. Writes
% PHASE0_TRUSTGATE_REPORT.md. Writes STATUS.md with per-criterion PASS/FAIL.
%
% Runs end-to-end in < 60 s (PHASE0_SPEC §3.7 implicit performance budget).

    t_start = tic();

    here = fileparts(mfilename('fullpath'));
    addpath(here);

    plots_dir = fullfile(here, 'plots');
    if ~isfolder(plots_dir)
        mkdir(plots_dir);
    end

    test_results = struct();
    test_results.cases = {};

    fprintf('\n========================================================\n');
    fprintf('T10 validation_block test suite\n');
    fprintf('========================================================\n');

    % --------- BUILD VALIDATION BLOCK ---------
    fprintf('\n[1/8] Building validation_block.slx...\n');
    try
        model_path = build_validation_block();
        cases_add('build_validation_block', isfile(model_path), ...
            sprintf('built at %s', model_path));
    catch ME
        cases_add('build_validation_block', false, ...
            sprintf('FAIL: %s', ME.message));
    end

    % --------- GENERATE SYNTHETIC TRUTH ---------
    fprintf('\n[2/8] Generating synthetic truth trajectory...\n');
    Truth = make_synthetic_truth();
    fprintf('   truth: n=%d samples, apogee=%.1f m @ %.2f s\n', ...
        numel(Truth.time_s), max(Truth.alt_agl_m), ...
        Truth.time_s(find(Truth.alt_agl_m == max(Truth.alt_agl_m), 1)));

    % --------- CASE A: PERFECT ESTIMATE ---------
    fprintf('\n[3/8] Case A: perfect estimator (estimate == truth)...\n');
    EstA = make_perfect_estimate(Truth);
    bundleA = casper_compute_all_metrics(Truth, EstA, [], []);
    fprintf('   apogee_pass=%d, vel_pass=%d, att_pass=%d, mg_pass=%d, bias_pass=%d, overall=%d\n', ...
        bundleA.apogee.pass, bundleA.velocity.pass, bundleA.attitude.pass, ...
        bundleA.mach_gate.pass, bundleA.bias.pass, bundleA.overall_pass);
    cases_add('A: perfect -> PASS', bundleA.overall_pass, ...
        sprintf('apogee_err=%.3f m, vel_brn_err=%.3f m/s, tilt_pwr=%.4f deg', ...
        bundleA.apogee.value.apogee_alt_err_m, ...
        bundleA.velocity.value.burnout_err_mps, ...
        bundleA.attitude.value.rms_powered_deg));

    % --------- CASE B: +20 m APOGEE OFFSET ---------
    fprintf('\n[4/8] Case B: +20 m apogee offset...\n');
    EstB = EstA;
    EstB.state_x(:, 1) = EstA.state_x(:, 1) + 20.0;
    bundleB = casper_compute_all_metrics(Truth, EstB, [], []);
    expected_b_fail = ~bundleB.apogee.pass;
    cases_add('B: +20 m alt -> FAIL on apogee', expected_b_fail, ...
        sprintf('apogee_err=%.3f m (threshold 10)', bundleB.apogee.value.apogee_alt_err_m));

    % --------- CASE C: +3 m/s BURNOUT VELOCITY OFFSET ---------
    fprintf('\n[5/8] Case C: +3 m/s burnout velocity offset...\n');
    EstC = EstA;
    EstC.state_x(:, 2) = EstA.state_x(:, 2) + 3.0;
    bundleC = casper_compute_all_metrics(Truth, EstC, [], []);
    expected_c_fail = ~bundleC.velocity.pass;
    cases_add('C: +3 m/s vel -> FAIL on velocity', expected_c_fail, ...
        sprintf('burnout_err=%.3f m/s (threshold 2)', bundleC.velocity.value.burnout_err_mps));

    % --------- CASE D: DETERMINISM TESTS (PASS + FAIL) ---------
    fprintf('\n[6/8] Case D: determinism (matching + injected divergence)...\n');
    RunPass.RunA.sensor_streams = struct('imu', randn(100, 6), 'baro', randn(100,1));
    RunPass.RunB = RunPass.RunA;
    RunPass.RunA.estimate = EstA;
    RunPass.RunB.estimate = EstA;
    detPass = casper_metric_determinism(RunPass.RunA, RunPass.RunB);
    cases_add('D1: identical two-run -> PASS', detPass.pass, detPass.details);

    RunFail = RunPass;
    RunFail.RunB.sensor_streams.imu(1,1) = RunFail.RunB.sensor_streams.imu(1,1) + 1e-9;
    detFail = casper_metric_determinism(RunFail.RunA, RunFail.RunB);
    expected_dfail = ~detFail.pass;
    cases_add('D2: divergent two-run -> FAIL', expected_dfail, detFail.details);

    % --------- CASE E: SANITY (NaN injection) ---------
    fprintf('\n[7/8] Case E: sanity NaN/Inf scan...\n');
    SignalsClean = struct('imu', randn(100, 6), 'baro', randn(100,1));
    sanityClean = casper_metric_sanity(SignalsClean, struct('runtime_s', 12.3));
    cases_add('E1: clean signals -> PASS', sanityClean.pass, sanityClean.details);

    SignalsBad = SignalsClean;
    SignalsBad.imu(5,2) = NaN;
    sanityBad = casper_metric_sanity(SignalsBad, struct('runtime_s', 12.3));
    expected_e_fail = ~sanityBad.pass;
    cases_add('E2: NaN injected -> FAIL', expected_e_fail, sanityBad.details);

    % --------- GENERATE PLOTS (uses case A inputs + synthetic sensors) ---------
    fprintf('\n[8/8] Generating plot bundle...\n');
    Sensors = make_synthetic_sensors(Truth);
    RadioTX = make_synthetic_radio_tx(Truth);
    plot_info = casper_generate_plots(Truth, EstA, Sensors, RadioTX, plots_dir, ...
        struct('test_id', 'T10_unit_test', 'seed', 20260519));
    cases_add('plots: 12 PNGs generated', plot_info.n_generated == 12, ...
        sprintf('saved %d files', plot_info.n_generated));

    % Generate report for case A (PASS) and case B (FAIL) to verify both code paths
    report_path_pass = fullfile(here, 'TEST_REPORT_caseA_pass.md');
    casper_generate_report(bundleA, report_path_pass, ...
        struct('test_id','T10_caseA','seed', 20260519), plots_dir);
    cases_add('report A (PASS) generated', isfile(report_path_pass), ...
        sprintf('written to %s', report_path_pass));

    % Provide determinism + sanity for the FAIL report (case B with apogee err)
    bundleB_full = casper_compute_all_metrics(Truth, EstB, RunPass, ...
        struct('Signals', SignalsClean, 'Meta', struct('runtime_s', 25.0)));
    report_path_fail = fullfile(here, 'TEST_REPORT_caseB_fail.md');
    casper_generate_report(bundleB_full, report_path_fail, ...
        struct('test_id','T10_caseB','seed', 20260519), plots_dir);
    cases_add('report B (FAIL) generated', isfile(report_path_fail), ...
        sprintf('written to %s', report_path_fail));

    runtime_s = toc(t_start);
    fprintf('\nTotal test runtime: %.2f s\n', runtime_s);
    cases_add('runtime <= 60 s', runtime_s <= 60.0, ...
        sprintf('elapsed %.2f s', runtime_s));

    % --------- WRITE STATUS.md ---------
    write_status_md(here, runtime_s);

    % Final report
    n_total = numel(test_results.cases);
    n_pass  = sum(cellfun(@(c) c.pass, test_results.cases));
    fprintf('\n========================================================\n');
    fprintf('T10 test summary: %d / %d cases passed\n', n_pass, n_total);
    fprintf('========================================================\n');

    if n_pass < n_total
        error('test_validation_block:Fail','%d/%d test cases failed.', n_total - n_pass, n_total);
    end

    function cases_add(name, pass_flag, detail)
        c = struct('name', name, 'pass', logical(pass_flag), 'detail', detail);
        test_results.cases{end+1} = c;
        if pass_flag
            fprintf('   [PASS] %s\n', name);
        else
            fprintf('   [FAIL] %s -- %s\n', name, detail);
        end
    end

    function write_status_md(outdir, rt)
        fp = fullfile(outdir, 'STATUS.md');
        fid = fopen(fp, 'w');
        if fid < 0; error('Cannot open %s', fp); end
        fprintf(fid, '# T10 Validation Block -- STATUS\n\n');
        fprintf(fid, 'Generated by `test_validation_block.m` on %s.\n\n', datestr(now,31));
        fprintf(fid, 'Build dir: `%s`\n\n', strrep(outdir, '\','/'));
        npass = sum(cellfun(@(c) c.pass, test_results.cases));
        nall  = numel(test_results.cases);
        fprintf(fid, '**Overall: %s** (%d pass / %d fail of %d)\n\n', ...
            ternary(npass==nall,'PASS','PARTIAL'), npass, nall-npass, nall);
        fprintf(fid, 'Total test runtime: %.2f s\n\n', rt);

        % Files created
        fprintf(fid, '## Files created\n\n');
        files = { ...
            'casper_metric_apogee.m', ...
            'casper_metric_velocity.m', ...
            'casper_metric_attitude.m', ...
            'casper_metric_mach_gate.m', ...
            'casper_metric_bias.m', ...
            'casper_metric_determinism.m', ...
            'casper_metric_sanity.m', ...
            'casper_data_hash.m', ...
            'casper_compute_all_metrics.m', ...
            'casper_generate_plots.m', ...
            'casper_generate_report.m', ...
            'build_validation_block.m', ...
            'test_validation_block.m', ...
            'validation_block.slx', ...
            'TEST_REPORT_caseA_pass.md', ...
            'TEST_REPORT_caseB_fail.md'};
        for k = 1:numel(files)
            fp_ = fullfile(outdir, files{k});
            if isfile(fp_)
                d_info = dir(fp_);
                kb = d_info.bytes / 1024;
                fprintf(fid, '- `%s`  (%.1f KB)\n', files{k}, kb);
            else
                fprintf(fid, '- `%s`  (MISSING)\n', files{k});
            end
        end
        % Plots
        fprintf(fid, '- `plots/` (12 PNGs)\n');
        fprintf(fid, '\n');

        % Acceptance criteria table
        fprintf(fid, '## Acceptance criteria (per task spec S9)\n\n');
        fprintf(fid, '| # | Criterion | Status | Notes |\n');
        fprintf(fid, '|---|---|---|---|\n');
        for k = 1:numel(test_results.cases)
            c = test_results.cases{k};
            st = ternary(c.pass,'PASS','FAIL');
            fprintf(fid, '| %d | %s | %s | %s |\n', k, c.name, st, c.detail);
        end
        fprintf(fid, '\n');

        fprintf(fid, '## Deviations from spec\n\n');
        fprintf(fid, '- Plots dir is `T10_validation_block/plots/` (T10 owns its own build dir).\n');
        fprintf(fid, '  Spec text mentions writing plots to `T11_integration/plots/` but those are\n');
        fprintf(fid, '  user-facing outputs that T11 produces by *calling* this module. The unit-test\n');
        fprintf(fid, '  artifacts live alongside T10.\n');
        fprintf(fid, '- TEST_REPORT_caseA_pass.md and TEST_REPORT_caseB_fail.md are unit-test artifacts\n');
        fprintf(fid, '  demonstrating both PASS and FAIL code paths of casper_generate_report.\n');
        fprintf(fid, '  Production runs use the PHASE0_TRUSTGATE_REPORT.md filename per spec.\n');
        fprintf(fid, '\n');
        fclose(fid);
    end
end

% ===================================================================
% ---- helpers ----
% ===================================================================

function y = ternary(cond, a, b)
    if cond, y = a; else, y = b; end
end

function Truth = make_synthetic_truth()
% Build a 60 s vertical trajectory: thrust 0..2 s (50 m/s^2 up), then coast
% under gravity. Apogee around (50*2)*30 = ~not quite, build closed-form:
%   accel_up = 50 m/s^2 from t=0..2 s (powered)
%   accel_up = -G from t=2..apogee
%   vel(2) = 100 m/s, alt(2) = 100 m
%   apogee_time = 2 + 100/G ~ 2 + 10.2 = 12.2 s
%   apogee_alt  = 100 + 100^2 / (2*G) ~ 100 + 509.8 = 609.8 m
%
% Mach: build mach signal that crosses 0.40 at t=2.5 s and falls below 0.35
% at t=8 s (placeholder; designed to make mach gate test exercise both
% engage and release).

    G = 9.80665;
    dt = 1e-3;  % 1 kHz truth grid (manageable size)
    t = (0 : dt : 60)';
    N = numel(t);

    accel_v_up = zeros(N, 1);
    vel_v_up   = zeros(N, 1);
    alt        = zeros(N, 1);

    % Powered: 0..2 s, accel_up = 50
    pwr = t <= 2.0;
    accel_v_up(pwr) = 50.0;

    % Integrate
    for k = 2:N
        accel_now = accel_v_up(k);
        if t(k) > 2.0
            accel_now = accel_now - G;   % gravity only (coast/descent)
        end
        accel_v_up(k) = accel_now;
        vel_v_up(k)   = vel_v_up(k-1) + accel_v_up(k) * dt;
        alt(k)        = alt(k-1)      + vel_v_up(k)   * dt;
    end

    % Convert to NED (Z-down)
    accel_NED = [zeros(N,2), -accel_v_up];   % Z-down NED accel
    vel_NED   = [zeros(N,2), -vel_v_up];
    pos_NED   = [zeros(N,2), -alt];

    % Quaternion: nose up, no rotation -> just identity rotated by 90 deg
    % pitch up. For Phase 0 attitude in NED frame, eul2quat([0, pitch, 0], 'ZYX')
    % with pitch = 89 deg (nose up). Use a constant attitude.
    pitch_rad = repmat(89.0 * pi/180, N, 1);
    eul = [zeros(N,1), pitch_rad, zeros(N,1)];
    quat_std = eul2quat(eul, 'ZYX');   % Nx4 [w x y z]

    % Mach: ramp up linearly during powered, then decay
    mach = zeros(N, 1);
    mach(t <= 2.0) = (t(t <= 2.0) / 2.0) * 0.6;            % 0..0.6 during burn
    decay_mask = t > 2.0;
    td = t(decay_mask) - 2.0;
    mach(decay_mask) = max(0.6 - 0.1 * td, 0);  % linear decay to 0

    % Make sure 0.40 first crossing is during powered phase
    %  - at t=4/3 s: mach = (4/3)/2 * 0.6 = 0.4 -> engage @ ~1.333 s
    %  - release when mach drops below 0.35: 0.6 - 0.1*td = 0.35 => td = 2.5 => t = 4.5 s

    % alt_agl_m = altitude up
    alt_agl_m = alt;

    Truth = struct();
    Truth.time_s            = t;
    Truth.alt_agl_m         = alt_agl_m;
    Truth.vel_v_mps         = vel_v_up;
    Truth.accel_v_mps2      = accel_v_up;
    Truth.accel_NED         = accel_NED;
    Truth.vel_NED           = vel_NED;
    Truth.pos_NED           = pos_NED;
    Truth.quat_fw           = quat_std;        % the validation block treats this as firmware-frame
    Truth.quat_std          = quat_std;        % alias
    Truth.mach              = mach;
    Truth.air_density_kgm3  = ones(N,1) * 1.225;
    Truth.air_temp_K        = ones(N,1) * 288.15;
    Truth.air_pressure_pa   = ones(N,1) * 101325;
    Truth.omega_body_std    = zeros(N, 3);
end

function Est = make_perfect_estimate(Truth)
% Sample truth at 416 Hz, create state_x = [alt, vel, ab=0, bb=0].
    dt_est = 1/416;
    t_est = (0 : dt_est : Truth.time_s(end))';
    M = numel(t_est);
    alt = interp1(Truth.time_s, Truth.alt_agl_m, t_est, 'linear');
    vel = interp1(Truth.time_s, Truth.vel_v_mps, t_est, 'linear');
    state_x = [alt, vel, zeros(M,1), zeros(M,1)];

    % Covariance diagonals start at P0 values and decay slightly
    P0 = [0.1, 0.001, 0.025, 0.75];
    state_P_diag = ones(M, 1) * P0;
    % Enforce floor on baro-bias P
    state_P_diag(:, 4) = max(state_P_diag(:, 4), 0.01);

    quat_fw = zeros(M, 4);
    for ax = 1:4
        quat_fw(:, ax) = interp1(Truth.time_s, Truth.quat_fw(:, ax), t_est, 'linear');
    end
    nrm = sqrt(sum(quat_fw.^2, 2));
    nrm(nrm<eps) = 1;
    quat_fw = quat_fw ./ nrm;

    mach_est = interp1(Truth.time_s, Truth.mach, t_est, 'linear');
    mach_gate_active = zeros(M, 1);
    gate_on = false;
    for k = 1:M
        if ~gate_on && mach_est(k) >= 0.40
            gate_on = true;
        elseif gate_on && mach_est(k) < 0.35
            gate_on = false;
        end
        mach_gate_active(k) = double(gate_on);
    end

    Est = struct();
    Est.time_s           = t_est;
    Est.state_x          = state_x;
    Est.state_P_diag     = state_P_diag;
    Est.quat_fw          = quat_fw;
    Est.mach_gate_active = mach_gate_active;

    % Synthetic baro innovations near zero with 5-sigma boundary
    bt = (0 : 0.01 : Truth.time_s(end))';
    Est.baro_innov = struct( ...
        'time_s',   bt, ...
        'innov_m',  0.05 * randn(numel(bt), 1), ...
        'sigma_m',  ones(numel(bt), 1) * sqrt(0.5), ...
        'rejected', false(numel(bt), 1));
    Est.zupt_innov = struct( ...
        'time_s',     bt(1:50), ...
        'innov_mps',  0.001 * randn(50, 1));
end

function Sensors = make_synthetic_sensors(Truth)
% Build minimal sensor traces for the snapshot plots.
    t = Truth.time_s;
    % IMU at 833 Hz
    t_imu = (0 : 1/833 : t(end))';
    N = numel(t_imu);
    accel = zeros(N, 3);
    accel(:, 2) = 9.80665;   % pad accel along body Y (firmware convention)
    gyro = zeros(N, 3);
    Sensors.imu = struct('time_s', t_imu, 'accel_mps2', accel, 'gyro_radps', gyro);

    % ADXL at 800 Hz
    t_adxl = (0 : 1/800 : t(end))';
    Sensors.adxl = struct('time_s', t_adxl, 'accel_mps2', zeros(numel(t_adxl), 3));

    % Baro at 100 Hz
    t_baro = (0 : 1/100 : t(end))';
    alt = interp1(t, Truth.alt_agl_m, t_baro, 'linear');
    Sensors.baro = struct('time_s', t_baro, 'alt_m', alt);

    % Mag at 100 Hz
    t_mag = (0 : 1/100 : t(end))';
    mag = repmat([20, 30, -10], numel(t_mag), 1);
    Sensors.mag = struct('time_s', t_mag, 'uT', mag);

    % GPS at 10 Hz
    t_gps = (0 : 1/10 : t(end))';
    Sensors.gps = struct('time_s', t_gps, 'pos_m', zeros(numel(t_gps), 3));
end

function RadioTX = make_synthetic_radio_tx(Truth)
% TX events at 10 Hz from t=0
    t_tx = (0 : 0.1 : Truth.time_s(end))';
    active = false(numel(t_tx), 1);
    active(1:end) = true;     % all schedule slots
    RadioTX = struct('time_s', t_tx, 'active', active);
end
