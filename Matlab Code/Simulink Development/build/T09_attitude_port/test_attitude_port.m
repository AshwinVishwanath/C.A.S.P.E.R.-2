function results = test_attitude_port()
%TEST_ATTITUDE_PORT  Acceptance tests for T09 stripped attitude estimator.
%
%   Returns a results struct with per-criterion PASS/FAIL.  Writes
%   plots/*.png and STATUS.md side-effects.  Designed for `matlab -batch`.

    this_dir = fileparts(mfilename('fullpath'));
    plots_dir = fullfile(this_dir, 'plots');
    if ~exist(plots_dir, 'dir')
        mkdir(plots_dir);
    end

    % ── Load T02 sensor params into this function's workspace ───────────
    t02_dir = fullfile(this_dir, '..', 'T02_sensor_params');
    addpath(t02_dir);
    % Shared plot-style helper lives one level up under build/.
    addpath(fullfile(this_dir, '..'));
    run(fullfile(t02_dir, 'casper_sensor_params.m'));
    %#ok<*NODEF>  Sim/Attitude come from the script above

    % Attach a default nav-frame reference mag for the static init
    % (T09 spec §5.2): use launch-site Earth field. Without a T07-supplied
    % LaunchSite struct, use a representative mid-latitude Z-up field:
    %   horizontal ~24 uT, vertical-down ~40 uT (in NED) → Z-up has
    %   z = -(-40) = +40? Be careful: firmware nav is Z-UP, and gravity
    %   reads +9.81 along +Z in nav (per ARCHITECTURE.md §3.2).
    %   The mag field at mid-latitude has its vertical component pointing
    %   INTO the ground; in Z-up convention that is negative Z.
    %   For a self-consistent simulation we just need a 3-vector — the
    %   estimator uses it as a fixed reference, not for absolute heading.
    Attitude.M_ref_nav_uT = [24.0; 0; -40.0];   % uT, Z-up (vertical down)

    fprintf('\n=== T09 Acceptance Tests ===\n');

    results = struct();
    results.ac1_static_init  = ac1_static_init(Attitude, Sim, plots_dir);
    results.ac2_gyro_only    = ac2_gyro_only_drift(Attitude, Sim);
    results.ac3_quat_norm    = ac3_quat_norm(Attitude, Sim);
    results.ac4_mahony_conv  = ac4_mahony_convergence(Attitude, Sim, plots_dir);
    results.ac5_mag_decim    = ac5_mag_decimation(Attitude, Sim);
    results.ac6_heading_floor= ac6_heading_sigma_floor(Attitude, Sim);
    results.ac7_full_traj    = ac7_full_trajectory(Attitude, Sim, plots_dir);
    results.ac8_determinism  = ac8_determinism(Attitude, Sim);
    results.ac9_no_nan_inf   = ac9_no_nan_inf(Attitude, Sim);
    results.ac10_performance = ac10_performance(Attitude, Sim);

    % Summary
    fprintf('\n=== Summary ===\n');
    names = fieldnames(results);
    n_pass = 0; n_fail = 0;
    for i = 1:numel(names)
        r = results.(names{i});
        status = r.pass;
        if status
            n_pass = n_pass + 1;
            tag = 'PASS';
        else
            n_fail = n_fail + 1;
            tag = 'FAIL';
        end
        fprintf('  %-20s %s  %s\n', names{i}, tag, r.notes);
    end
    fprintf('TOTAL: %d passed, %d failed\n', n_pass, n_fail);

    write_status_md(this_dir, results, n_pass, n_fail);

    if n_fail > 0
        error('test_attitude_port:fail', '%d acceptance criteria failed', n_fail);
    end
end

% =============================================================================
% AC1: Static init convergence
% =============================================================================
function r = ac1_static_init(Att, Sim, plots_dir)
    fprintf('-- AC1: static init convergence\n');
    dt_s = 1/833;
    T = 6.0;   % run 6 s (more than 5 s)
    N = round(T/dt_s);

    % Seedable RNG
    rs = RandStream('mt19937ar','Seed', Sim.Seed);

    % Truth pose: small but nonzero tilt — emulate slightly tilted rocket.
    % In firmware-frame, nose-up means body +Y = nav +Z, so pad accel is
    % nominally [0, +g, 0]. Add a 1-deg tilt about body X to simulate
    % imperfect pad alignment.
    tilt_rad = deg2rad(1.0);
    g = 9.80665;
    a_pad = [0;
             g * cos(tilt_rad);
             g * sin(tilt_rad)];

    % Reference mag in nav, Z-up; we synthesize a body-frame measurement
    % using the truth quaternion that satisfies a_pad.
    ops = casper_quat_ops();
    q_truth = ops.from_accel(a_pad);
    R       = ops.to_dcm(q_truth);
    m_nav   = Att.M_ref_nav_uT(:);
    m_body  = R' * m_nav;

    accel_noise = 0.05;   % m/s^2 1-sigma
    mag_noise   = 0.5;    % uT
    gyro_noise  = 0.001;  % rad/s

    p = Att;
    st = casper_attitude_state_new();

    % mag arrives at 100 Hz: every floor(833/100) = ~8 ticks
    mag_period_ticks = round(833/100);

    quat_hist = zeros(4, N);
    init_t_s  = NaN;
    for k = 1:N
        accel = a_pad + accel_noise * randn(rs, 3, 1);
        gyro  =          gyro_noise  * randn(rs, 3, 1);
        mag_new = (mod(k-1, mag_period_ticks) == 0);
        if mag_new
            mag = m_body + mag_noise * randn(rs, 3, 1);
        else
            mag = zeros(3,1);
        end
        [att, st] = casper_attitude_tick(accel, gyro, mag, mag_new, true, dt_s, p, st);
        quat_hist(:,k) = att.quat_body_to_nav;
        if att.init_complete && isnan(init_t_s)
            init_t_s = k * dt_s;
        end
    end

    % Final attitude error (after 5 s of post-init operation)
    final_idx = N;
    q_est_final = quat_hist(:, final_idx);
    angle_err_rad = ops.angle_between(q_truth, q_est_final);
    angle_err_deg = rad2deg(angle_err_rad);

    init_within_timeout = ~isnan(init_t_s) && (init_t_s <= Att.StaticInitTimeout_s);
    err_under_1deg      = angle_err_deg < 1.0;
    pass = init_within_timeout && err_under_1deg;

    % Plot
    try
        fig = figure('Visible', 'off');
        tiledlayout(1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        nexttile;
        t = (1:N) * dt_s;
        eul_deg = zeros(3, N);
        for k = 1:N
            eul_deg(:,k) = ops.to_euler(quat_hist(:,k));
        end
        plot(t, eul_deg(1,:), 'DisplayName', 'bodyZ (yaw)'); hold on;
        plot(t, eul_deg(2,:), 'DisplayName', 'bodyY (roll)');
        plot(t, eul_deg(3,:), 'DisplayName', 'bodyX (pitch)');
        xlabel('Time [s]'); ylabel('Euler angle [deg]');
        title(sprintf('AC1 pad-init: final err = %.3f deg, init done at %.2f s', ...
            angle_err_deg, init_t_s));
        legend('Location', 'best', 'Box', 'off');
        apply_style_(fig, 10, 6);
        exportgraphics(fig, fullfile(plots_dir, 'attitude_pad_initialization.png'), ...
            'Resolution', 300);
        close(fig);
    catch ME
        warning('AC1 plot failed: %s', ME.message);
    end

    r = struct('pass', pass, 'notes', ...
        sprintf('final err %.3f deg (lim 1.0), init at %.2f s (lim %.1f)', ...
                angle_err_deg, init_t_s, Att.StaticInitTimeout_s));
end

% =============================================================================
% AC2: Gyro-only propagation accuracy (Mahony off, clean truth gyro)
% =============================================================================
function r = ac2_gyro_only_drift(Att, Sim)
    fprintf('-- AC2: gyro-only drift\n');
    dt_s = 1/833;
    T = 60.0;
    N = round(T/dt_s);

    % Disable Mahony by zeroing gains AND running in flight mode
    p = Att;
    p.Kp_Grav = 0;
    p.Kp_MagPad = 0;
    p.Kp_MagFlight = 0;
    p.Ki = 0;

    st = casper_attitude_state_new();

    % Bypass static init: hand-set state so we can drive immediately.
    st.init_complete = true;
    st.q_body_to_nav = [1;0;0;0];   % identity
    st.m_ref_nav_uT  = [1;0;0];
    st.mag_available = false;
    st.gyro_filt_radps = zeros(3,1);
    st.gyro_bias_radps = zeros(3,1);

    % Clean (no-noise) zero gyro → ideal q should stay [1;0;0;0]
    quat_hist = zeros(4, N);
    for k = 1:N
        [att, st] = casper_attitude_tick( ...
            zeros(3,1), zeros(3,1), zeros(3,1), false, false, dt_s, p, st);
        quat_hist(:,k) = att.quat_body_to_nav;
    end

    ops = casper_quat_ops();
    q_truth = [1;0;0;0];
    q_final = quat_hist(:,end);
    err_rad = ops.angle_between(q_truth, q_final);
    err_deg = rad2deg(err_rad);

    % per-axis decomposition
    eul_final = ops.to_euler(q_final);   % degrees
    max_axis  = max(abs(eul_final));

    pass = max_axis < 0.5;
    r = struct('pass', pass, ...
        'notes', sprintf('|q angle| = %.4f deg; max axis = %.4f deg (lim 0.5)', ...
                          err_deg, max_axis));
end

% =============================================================================
% AC3: Quaternion unit norm
% =============================================================================
function r = ac3_quat_norm(Att, Sim)
    fprintf('-- AC3: quaternion unit norm\n');
    dt_s = 1/833;
    T = 30.0;
    N = round(T/dt_s);

    p  = Att;
    st = casper_attitude_state_new();
    st.init_complete = true;
    st.q_body_to_nav = ops_normalize_4([0.5; 0.5; 0.5; 0.5]);
    st.m_ref_nav_uT  = [1;0;0];
    st.mag_available = false;
    st.gyro_filt_radps = zeros(3,1);
    st.gyro_bias_radps = zeros(3,1);

    rs = RandStream('mt19937ar','Seed',Sim.Seed+2);
    norms = zeros(1,N);
    for k = 1:N
        gyro = 0.05 * randn(rs,3,1) + [0.1; 0.2; -0.05];
        [att, st] = casper_attitude_tick( ...
            [0;9.81;0], gyro, zeros(3,1), false, false, dt_s, p, st);
        q = att.quat_body_to_nav;
        norms(k) = sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
    end

    max_dev = max(abs(norms - 1));
    pass = max_dev < 1e-5;
    r = struct('pass', pass, ...
        'notes', sprintf('max ||q||-1 = %.3e (lim 1e-5)', max_dev));
end

% =============================================================================
% AC4: Mahony correction from perturbed initial quaternion
% =============================================================================
function r = ac4_mahony_convergence(Att, Sim, plots_dir)
    fprintf('-- AC4: Mahony correction convergence\n');
    dt_s = 1/833;
    T = 8.0;
    N = round(T/dt_s);

    % Truth pose: nose-up on pad
    g = 9.80665;
    a_truth = [0; g; 0];
    ops = casper_quat_ops();
    q_truth = ops.from_accel(a_truth);

    p = Att;
    % Use the FIRMWARE pad gravity gain. Disable mag (Kp_MagPad = 0 already).
    p.Ki = 0;   % avoid integrator wind-up for this short test

    st = casper_attitude_state_new();
    % Skip static init; set perturbed initial quat (5 deg tilt about body X)
    pert_rad = deg2rad(5.0);
    R_pert = ops.to_dcm(ops.from_euler(pert_rad, 0, 0));
    q_init = ops.dcm_to_quat(ops.to_dcm(q_truth) * R_pert);

    st.init_complete   = true;
    st.q_body_to_nav   = q_init;
    st.m_ref_nav_uT    = Att.M_ref_nav_uT(:);
    st.mag_available   = true;
    st.gyro_filt_radps = zeros(3,1);
    st.gyro_bias_radps = zeros(3,1);

    rs = RandStream('mt19937ar','Seed', Sim.Seed+3);
    err_deg_hist = zeros(1, N);
    converge_t_s = NaN;
    for k = 1:N
        accel = a_truth + 0.05 * randn(rs,3,1);
        gyro  = 0.001 * randn(rs,3,1);
        [att, st] = casper_attitude_tick( ...
            accel, gyro, zeros(3,1), false, true, dt_s, p, st);
        e = rad2deg(ops.angle_between(q_truth, att.quat_body_to_nav));
        err_deg_hist(k) = e;
        if isnan(converge_t_s) && e < 0.5
            converge_t_s = k * dt_s;
        end
    end

    pass = ~isnan(converge_t_s) && (converge_t_s < 5.0);
    r = struct('pass', pass, ...
        'notes', sprintf('converged to <0.5 deg at %.2f s (lim 5.0)', converge_t_s));
end

% =============================================================================
% AC5: Flight-mode mag correction fires at 10 Hz
% =============================================================================
function r = ac5_mag_decimation(Att, Sim)
    fprintf('-- AC5: 10 Hz mag decimation in flight\n');
    dt_s = 1/833;
    T = 2.0;
    N = round(T/dt_s);

    % We need to force Kp_MagFlight > 0 so the path actually executes the
    % correction and resets the timer. With Kp_MagFlight = 0 the function
    % STILL fires the path (i.e., still reaches the "reset timer" code)
    % when valid mag arrives — the gain only scales omega/heading_sigma.
    % The path resets the timer when fired==true regardless of gain.
    p = Att;
    p.Kp_MagFlight = 0;   % keep firmware-canonical zero gain
    p.Ki = 0;

    st = casper_attitude_state_new();
    st.init_complete = true;
    st.q_body_to_nav = [1;0;0;0];
    st.m_ref_nav_uT  = Att.M_ref_nav_uT(:);
    st.mag_available = true;
    st.gyro_filt_radps = zeros(3,1);
    st.gyro_bias_radps = zeros(3,1);

    mag_period_ticks = round(833/100);   % ~8 ticks → 100 Hz raw mag
    fire_count = 0;
    fire_times_s = [];
    for k = 1:N
        mag_new = (mod(k-1, mag_period_ticks) == 0);
        if mag_new
            mag = [10; 0; -40];
        else
            mag = zeros(3,1);
        end
        [att, st] = casper_attitude_tick( ...
            [0;9.81;0], zeros(3,1), mag, mag_new, false, dt_s, p, st);
        if att.fired_mag_flight
            fire_count = fire_count + 1;
            fire_times_s(end+1) = k * dt_s; %#ok<AGROW>
        end
    end

    % Expected ≈ 10 Hz × 2 s = 20 fires (allow ±2 for boundary effects).
    expected_fires = round(p.MagUpdateRate_Hz * T);
    diff_count = abs(fire_count - expected_fires);

    % Verify gap consistency between fires: should be ~0.1 s
    if numel(fire_times_s) > 2
        gaps = diff(fire_times_s);
        gap_mean = mean(gaps);
        gap_ok = abs(gap_mean - 0.1) < 0.02;
    else
        gap_ok = false;
        gap_mean = NaN;
    end

    pass = (diff_count <= 2) && gap_ok;
    r = struct('pass', pass, ...
        'notes', sprintf('fires=%d (exp %d), gap_mean=%.4f s (target 0.100)', ...
                          fire_count, expected_fires, gap_mean));
end

% =============================================================================
% AC6: Heading sigma floor
% =============================================================================
function r = ac6_heading_sigma_floor(Att, Sim)
    fprintf('-- AC6: heading sigma floor\n');
    dt_s = 1/833;
    T = 5.0;
    N = round(T/dt_s);

    p  = Att;
    st = casper_attitude_state_new();
    st.init_complete = true;
    st.q_body_to_nav = [1;0;0;0];
    st.m_ref_nav_uT  = Att.M_ref_nav_uT(:);
    st.mag_available = true;
    st.gyro_filt_radps = zeros(3,1);
    st.gyro_bias_radps = zeros(3,1);

    sigma_hist = zeros(1,N);
    for k = 1:N
        [att, st] = casper_attitude_tick( ...
            [0;9.81;0], zeros(3,1), zeros(3,1), false, true, dt_s, p, st);
        sigma_hist(k) = att.heading_sigma_rad;
    end

    min_sigma = min(sigma_hist);
    pass = (min_sigma >= p.HeadingSigmaFloor_rad - 1e-12);
    r = struct('pass', pass, ...
        'notes', sprintf('min heading_sigma = %.6f (floor %.6f)', ...
                          min_sigma, p.HeadingSigmaFloor_rad));
end

% =============================================================================
% AC7: Full-trajectory truth tracking (synthetic flight profile)
% =============================================================================
function r = ac7_full_trajectory(Att, Sim, plots_dir)
    fprintf('-- AC7: truth tracking on synthetic trajectory\n');
    % We use a synthetic trajectory because T01 / T10 not in scope.
    %   - Pad 5 s (zero body rate, nose-up)
    %   - Powered 3 s: roll about body Y at +30 dps (nose-axis spin)
    %   - Coast 20 s: zero gyro, drift only
    dt_s = 1/833;
    T = 28.0;
    N = round(T/dt_s);

    ops = casper_quat_ops();
    g = 9.80665;
    a_truth_pad = [0; g; 0];
    q_true = ops.from_accel(a_truth_pad);

    % Build truth gyro & accel histories
    omega_true_radps = zeros(3, N);
    accel_true_mps2  = zeros(3, N);
    mode_pad_hist    = false(1, N);

    for k = 1:N
        t = (k-1) * dt_s;
        if t < 5
            omega_true_radps(:,k) = [0;0;0];
            accel_true_mps2(:,k)  = a_truth_pad;
            mode_pad_hist(k) = true;
        elseif t < 8
            omega_true_radps(:,k) = deg2rad([0; 30; 0]);   % nose-axis roll
            accel_true_mps2(:,k)  = a_truth_pad;            % stays nose-up
            mode_pad_hist(k) = false;
        else
            omega_true_radps(:,k) = [0;0;0];
            accel_true_mps2(:,k)  = a_truth_pad;
            mode_pad_hist(k) = false;
        end
    end

    % Integrate truth quaternion offline (same RK4) so we can compare
    q_truth_hist = zeros(4, N);
    q = q_true;
    for k = 1:N
        q = casper_attitude_predict_rk4(q, omega_true_radps(:,k), dt_s);
        q_truth_hist(:,k) = q;
    end

    % Synthesize body-frame mag using truth attitude
    m_nav = Att.M_ref_nav_uT(:);
    mag_period_ticks = round(833/100);

    p = Att;
    p.M_ref_nav_uT = m_nav;

    rs = RandStream('mt19937ar','Seed',Sim.Seed+7);
    st = casper_attitude_state_new();

    q_est_hist = zeros(4, N);
    sigma_hist = zeros(1, N);
    accel_noise = 0.05;
    gyro_noise  = 0.001;
    mag_noise   = 0.5;
    for k = 1:N
        R = ops.to_dcm(q_truth_hist(:,k));
        mag_body_clean = R' * m_nav;
        gyro_meas  = omega_true_radps(:,k) + gyro_noise * randn(rs,3,1);
        accel_meas = accel_true_mps2(:,k)  + accel_noise * randn(rs,3,1);
        mag_new = (mod(k-1, mag_period_ticks) == 0);
        if mag_new
            mag_meas = mag_body_clean + mag_noise * randn(rs,3,1);
        else
            mag_meas = zeros(3,1);
        end
        [att, st] = casper_attitude_tick( ...
            accel_meas, gyro_meas, mag_meas, mag_new, ...
            mode_pad_hist(k), dt_s, p, st);
        q_est_hist(:,k) = att.quat_body_to_nav;
        sigma_hist(k)   = att.heading_sigma_rad;
    end

    % Per-tick angle error (rad)
    err_rad = zeros(1, N);
    for k = 1:N
        err_rad(k) = ops.angle_between(q_truth_hist(:,k), q_est_hist(:,k));
    end
    err_deg = rad2deg(err_rad);

    % Tilt RMS during powered (5..8 s) and coast (10..end) excluding init period
    t = (1:N) * dt_s;
    idx_powered = (t > 5.5) & (t < 8.0);
    idx_coast   = (t > 10.0);
    rms_powered = sqrt(mean(err_deg(idx_powered).^2));
    rms_coast   = sqrt(mean(err_deg(idx_coast).^2));

    pass_powered = rms_powered < 1.0;
    pass_coast   = rms_coast   < 2.0;
    pass = pass_powered && pass_coast;

    % Plots
    try
        fig1 = figure('Visible', 'off');
        tcl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        eul_truth = zeros(3,N); eul_est = zeros(3,N);
        for k = 1:N
            eul_truth(:,k) = ops.to_euler(q_truth_hist(:,k));
            eul_est(:,k)   = ops.to_euler(q_est_hist(:,k));
        end

        nexttile;
        plot(t, eul_truth(1,:), '-', 'DisplayName', 'truth'); hold on;
        plot(t, eul_est(1,:),   '--', 'DisplayName', 'estimate');
        xlabel('Time [s]'); ylabel('Yaw [deg]');
        title('Yaw');
        legend('Location', 'best', 'Box', 'off');

        nexttile;
        plot(t, eul_truth(2,:), '-', 'DisplayName', 'truth'); hold on;
        plot(t, eul_est(2,:),   '--', 'DisplayName', 'estimate');
        xlabel('Time [s]'); ylabel('Roll [deg]');
        title('Roll');
        legend('Location', 'best', 'Box', 'off');

        nexttile;
        plot(t, eul_truth(3,:), '-', 'DisplayName', 'truth'); hold on;
        plot(t, eul_est(3,:),   '--', 'DisplayName', 'estimate');
        xlabel('Time [s]'); ylabel('Pitch [deg]');
        title('Pitch');
        legend('Location', 'best', 'Box', 'off');

        title(tcl, 'AC7 attitude tracking (truth vs estimate)', 'Interpreter', 'none');
        apply_style_(fig1, 12, 10);
        exportgraphics(fig1, fullfile(plots_dir, 'attitude_flight_tracking.png'), ...
            'Resolution', 300);
        close(fig1);

        fig2 = figure('Visible', 'off');
        tiledlayout(1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        nexttile;
        plot(t, err_deg, 'DisplayName', 'angle error'); hold on;
        yline(1.0, '--', 'DisplayName', 'powered limit (1 deg)');
        yline(2.0, '--', 'DisplayName', 'coast limit (2 deg)');
        xlabel('Time [s]'); ylabel('Angle error [deg]');
        title(sprintf('AC7 angle error: RMS powered = %.3f, coast = %.3f', ...
            rms_powered, rms_coast));
        legend('Location', 'best', 'Box', 'off');
        apply_style_(fig2, 10, 6);
        exportgraphics(fig2, fullfile(plots_dir, 'attitude_error_euler.png'), ...
            'Resolution', 300);
        close(fig2);
    catch ME
        warning('AC7 plot failed: %s', ME.message);
    end

    r = struct('pass', pass, ...
        'notes', sprintf('RMS powered %.3f deg (lim 1.0), RMS coast %.3f deg (lim 2.0)', ...
                          rms_powered, rms_coast));
end

% =============================================================================
% AC8: Determinism
% =============================================================================
function r = ac8_determinism(Att, Sim)
    fprintf('-- AC8: determinism (same seed → byte-identical quats)\n');
    [hist1, ~] = run_short(Att, Sim);
    [hist2, ~] = run_short(Att, Sim);
    delta = max(abs(hist1(:) - hist2(:)));
    pass = delta == 0;
    r = struct('pass', pass, ...
        'notes', sprintf('max |Δq| between runs = %.3e (must be 0)', delta));
end

function [hist, st] = run_short(Att, Sim)
    dt_s = 1/833;
    T = 2.0;
    N = round(T/dt_s);
    rs = RandStream('mt19937ar','Seed',Sim.Seed+8);
    st = casper_attitude_state_new();
    mag_period_ticks = round(833/100);
    hist = zeros(4, N);
    for k = 1:N
        accel = [0;9.81;0] + 0.05 * randn(rs,3,1);
        gyro  = 0.001 * randn(rs,3,1);
        mag_new = (mod(k-1, mag_period_ticks) == 0);
        if mag_new
            mag = Att.M_ref_nav_uT(:) + 0.5 * randn(rs,3,1);
        else
            mag = zeros(3,1);
        end
        [att, st] = casper_attitude_tick( ...
            accel, gyro, mag, mag_new, true, dt_s, Att, st);
        hist(:,k) = att.quat_body_to_nav;
    end
end

% =============================================================================
% AC9: No NaN/Inf
% =============================================================================
function r = ac9_no_nan_inf(Att, Sim)
    fprintf('-- AC9: no NaN/Inf in outputs\n');
    [hist, st] = run_short(Att, Sim);
    bad = any(~isfinite(hist(:))) || any(~isfinite(st.gyro_bias_radps)) ...
          || ~isfinite(st.heading_sigma_rad);
    pass = ~bad;
    r = struct('pass', pass, ...
        'notes', sprintf('all finite = %d', ~bad));
end

% =============================================================================
% AC10: Performance (60 s sim < 30 s wall-clock; we use 30 s as proxy)
% =============================================================================
function r = ac10_performance(Att, Sim)
    fprintf('-- AC10: performance\n');
    dt_s = 1/833;
    T = 30.0;
    N = round(T/dt_s);
    rs = RandStream('mt19937ar','Seed',Sim.Seed+10);
    st = casper_attitude_state_new();
    mag_period_ticks = round(833/100);
    tic;
    for k = 1:N
        accel = [0;9.81;0] + 0.05 * randn(rs,3,1);
        gyro  = 0.001 * randn(rs,3,1);
        mag_new = (mod(k-1, mag_period_ticks) == 0);
        if mag_new
            mag = Att.M_ref_nav_uT(:) + 0.5 * randn(rs,3,1);
        else
            mag = zeros(3,1);
        end
        [~, st] = casper_attitude_tick( ...
            accel, gyro, mag, mag_new, true, dt_s, Att, st);
    end
    wall = toc;
    % Scale: spec says full 549 s sim < 30 s wall-clock. We measured 30 s
    % of sim and apply linear scaling.
    proj_full = wall * (549/30);
    pass = proj_full < 30;
    r = struct('pass', pass, ...
        'notes', sprintf('measured %.2fs for 30s sim → projected %.2fs for 549s (lim 30s)', ...
            wall, proj_full));
end

% =============================================================================
% Misc utilities
% =============================================================================
function q = ops_normalize_4(q)
    n = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
    q = q / n;
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

function write_status_md(this_dir, results, n_pass, n_fail)
    f = fopen(fullfile(this_dir, 'STATUS.md'), 'w');
    if f < 0
        warning('Could not write STATUS.md'); return;
    end
    c = onCleanup(@() fclose(f));
    fprintf(f, '# T09 Attitude Port — STATUS\n\n');
    fprintf(f, '## Files created\n');
    files = { ...
        'casper_quat_ops.m', ...
        'casper_attitude_state_new.m', ...
        'casper_attitude_static_init.m', ...
        'casper_attitude_gyro_lpf.m', ...
        'casper_attitude_predict_rk4.m', ...
        'casper_attitude_mahony.m', ...
        'casper_attitude_mag_correct_flight.m', ...
        'casper_attitude_tick.m', ...
        'build_attitude_block.m', ...
        'test_attitude_port.m', ...
        'attitude_block.slx', ...
        'plots/attitude_pad_initialization.png', ...
        'plots/attitude_flight_tracking.png', ...
        'plots/attitude_error_euler.png'};
    for i = 1:numel(files)
        fprintf(f, '- %s\n', files{i});
    end
    fprintf(f, '\n## Acceptance criteria\n');
    fprintf(f, '| # | Criterion | Status | Notes |\n');
    fprintf(f, '|---|---|---|---|\n');
    names = fieldnames(results);
    pretty = { ...
        'Static init (final err <1 deg, init <=10s)', ...
        'Gyro-only drift over 60 s, <0.5 deg per axis', ...
        'Quaternion unit norm |q|-1| < 1e-5', ...
        'Mahony converges from 5 deg perturb within 5 s', ...
        'Flight mag correction fires at 10 Hz', ...
        'Heading sigma >= floor always', ...
        'Tilt RMS <1 deg (powered), <2 deg (coast)', ...
        'Determinism (same seed → byte-identical quats)', ...
        'No NaN/Inf', ...
        'Performance (<30s wall for 549s sim)'};
    for i = 1:numel(names)
        r = results.(names{i});
        if r.pass, st = 'PASS'; else, st = 'FAIL'; end
        fprintf(f, '| %d | %s | %s | %s |\n', i, pretty{i}, st, r.notes);
    end
    fprintf(f, '\n**Summary**: %d PASS, %d FAIL\n\n', n_pass, n_fail);
    fprintf(f, '## Stripped items (per ARCHITECTURE.md §7)\n');
    fprintf(f, '- gyro temperature compensation: not implemented (Phase 0 stripped)\n');
    fprintf(f, '- online gyro EMA bias gate: not implemented (Phase 0 stripped, bias is static-init only)\n');
    fprintf(f, '- ignition-gated mag corrections: not implemented (no FSM in Phase 0)\n');
    fprintf(f, '- flight FSM-driven mode switching: replaced with simple boolean ''mode_pad'' input\n\n');
    fprintf(f, '## Deviations from spec\n');
    fprintf(f, '- Flight-mode mag correction uses firmware''s full 3D cross product (R'' * m_ref vs m_meas) instead of the alternative tilt-projection formulation in spec §9.1. Reason: CLAUDE.md ''firmware is canonical''.\n');
    fprintf(f, '- Kp_MagPad and Kp_MagFlight come from T02 ``Attitude.Kp_*`` which mirror main.c live values (BOTH 0). Code path is exercised but the correction is a no-op for omega until firmware changes the gains.\n');
    fprintf(f, '- RK4 sub-steps do NOT renormalize between k1..k4 (matches firmware); only final aggregation is normalized.\n');
end
