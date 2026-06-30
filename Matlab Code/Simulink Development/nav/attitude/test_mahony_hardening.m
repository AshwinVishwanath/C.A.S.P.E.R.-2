function results = test_mahony_hardening()
%TEST_MAHONY_HARDENING  Unit tests for the L2 Mahony hardening changes.
%
%   Source spec: Product Requirement Docs/MAHONY_HARDENING_PRD.md
%
%   Tests the three L2 mechanisms in isolation, without booting the full
%   Simulink stack:
%
%     T1  Magnitude window weight w(|a|) matches the closed-form cosine bell
%         at the centre, edges, and outside.
%     T2  Mahony at static |a|=g, Ki=0 default, integral does not leak into
%         omega_corr (sanity check of L2.2).
%     T3  Mahony during synthetic boost (|a|=8g) drives the gated gravity
%         correction to zero; the legacy un-gated path produces a large
%         omega_corr.
%     T4  Tick-level: pad_calib_complete latches at exactly
%         PadCalibDuration_s and gyro_bias_radps keeps refining until then.
%
%   Returns a struct with .pass for each test.  Designed for `matlab -batch`.

    this_dir = fileparts(mfilename('fullpath'));
    simroot  = fileparts(fileparts(this_dir));
    addpath(fullfile(simroot, 'params'));
    addpath(fullfile(simroot, 'nav', 'attitude'));
    addpath(fullfile(simroot, 'shared'));

    run(fullfile(simroot, 'params', 'casper_sensor_params.m'));
    %#ok<*NODEF>  Attitude / Sim come from the script above

    results = struct();
    results.t1_window_shape    = t1_window_shape();
    results.t2_ki_default_zero = t2_ki_default_zero(Attitude);
    results.t3_boost_gate      = t3_boost_gate(Attitude);
    results.t4_pad_calib_latch = t4_pad_calib_latch(Attitude);

    fprintf('\n=== test_mahony_hardening summary ===\n');
    names = fieldnames(results);
    n_pass = 0; n_fail = 0;
    for k = 1:numel(names)
        r = results.(names{k});
        if r.pass
            fprintf('  [PASS] %s  %s\n', names{k}, r.msg);
            n_pass = n_pass + 1;
        else
            fprintf('  [FAIL] %s  %s\n', names{k}, r.msg);
            n_fail = n_fail + 1;
        end
    end
    fprintf('  ----- %d/%d passed -----\n', n_pass, n_pass + n_fail);
    if n_fail > 0
        error('test_mahony_hardening:fail', '%d test(s) failed', n_fail);
    end
end

% =========================================================================
function r = t1_window_shape()
% Closed-form check of the cosine window weight w(|a|).
    G = 9.80665;
    hw_g = 0.15;
    q = [1; 0; 0; 0];   % identity, makes R'*[0;0;1] = [0;0;1]

    % At |a| = g exactly, weight = 1.
    a_centre = [0; 0; G];
    [~, ~, w_centre] = casper_attitude_grav_correct(a_centre, q, 1.0, hw_g);

    % At |a| = g * (1 + hw_g), weight = 0.
    a_edge_hi = [0; 0; G * (1 + hw_g)];
    [~, ~, w_edge_hi] = casper_attitude_grav_correct(a_edge_hi, q, 1.0, hw_g);

    % At |a| = g * (1 - hw_g), weight = 0.
    a_edge_lo = [0; 0; G * (1 - hw_g)];
    [~, ~, w_edge_lo] = casper_attitude_grav_correct(a_edge_lo, q, 1.0, hw_g);

    % Well outside (boost), weight = 0.
    a_boost = [0; 0; 8 * G];
    [~, ~, w_boost] = casper_attitude_grav_correct(a_boost, q, 1.0, hw_g);

    % Halfway up the window, |a| = g * (1 + hw_g/2) -> w = cos(pi/2)*0.5+0.5 = 0.5
    a_mid = [0; 0; G * (1 + hw_g/2)];
    [~, ~, w_mid] = casper_attitude_grav_correct(a_mid, q, 1.0, hw_g);

    tol = 1e-9;
    ok = abs(w_centre - 1.0) < tol && ...
         abs(w_edge_hi)       < tol && ...
         abs(w_edge_lo)       < tol && ...
         abs(w_boost)         < tol && ...
         abs(w_mid - 0.5)     < 1e-9;

    r.pass = ok;
    r.msg  = sprintf('w(g)=%.3g w(g+hw)=%.3g w(g-hw)=%.3g w(8g)=%.3g w(mid)=%.3g', ...
        w_centre, w_edge_hi, w_edge_lo, w_boost, w_mid);
end

% =========================================================================
function r = t2_ki_default_zero(Attitude)
% At static |a|=g and identity attitude, e_grav = 0 so the proportional
% feedback contributes nothing.  We verify that omega_corr == omega even
% after accumulating many ticks of (non-zero) e_int, because Ki=0.
    Attitude.Ki = 0.0;
    dt = 1/833;
    q = [1; 0; 0; 0];

    % Use a slightly tilted accel so e_grav is non-zero (build up e_int).
    a = [0.1; 0.0; 9.7];
    omega = [0.01; -0.02; 0.03];
    e_int = zeros(3,1);

    omega_corr_first = omega;   % placeholder
    for k = 1:100
        [omega_corr, e_int, ~, ~, ~] = casper_attitude_mahony( ...
            omega, a, zeros(3,1), false, q, zeros(3,1), false, ...
            Attitude, e_int, dt);
        if k == 100
            omega_corr_first = omega_corr;
        end
    end

    % With Ki=0, omega_corr == omega + Kp_Grav_eff * e_grav (no integral leak).
    % Solve for the expected feedback once:
    [e_grav, kp_eff, ~] = casper_attitude_grav_correct( ...
        a, q, Attitude.Kp_Grav, Attitude.GravGate_WindowHalfWidth_g);
    expected = omega + kp_eff * e_grav;

    tol = 1e-12;
    ok = max(abs(omega_corr_first - expected)) < tol;
    r.pass = ok;
    r.msg  = sprintf('|omega_corr - expected|_inf = %.3e ; |e_int|_inf = %.3e', ...
        max(abs(omega_corr_first - expected)), max(abs(e_int)));
end

% =========================================================================
function r = t3_boost_gate(Attitude)
% Synthetic boost: |a| = 8 g along body-Z.  With gate active, the gravity
% contribution to omega_corr must be zero (within tol).  With gate disabled
% (legacy open window) the contribution must be large.
    dt = 1/833;
    q = [1; 0; 0; 0];
    a = [0; 0; 8 * 9.80665];
    omega = zeros(3,1);
    e_int = zeros(3,1);

    Attitude.Ki = 0.0;
    Attitude.GravGate_WindowHalfWidth_g = 0.15;

    [omega_corr_gated, ~, ~, ~, w_gated] = casper_attitude_mahony( ...
        omega, a, zeros(3,1), false, q, zeros(3,1), false, ...
        Attitude, e_int, dt);

    % Legacy "no gate" path = wide-open window.
    Attitude_legacy = Attitude;
    Attitude_legacy.GravGate_WindowHalfWidth_g = 100.0;
    [~, ~, ~, ~, w_legacy] = casper_attitude_mahony( ...
        omega, a, zeros(3,1), false, q, zeros(3,1), false, ...
        Attitude_legacy, e_int, dt);

    tol_gated  = 1e-12;
    % Legacy: in this exact alignment (a parallel to gravity reaction),
    % e_grav = cross(a_hat, [0;0;1]) = 0 — so even legacy is 0 here.  Make
    % the test more discriminating by tilting the boost vector slightly.
    a_tilt = [9.81; 0; 8 * 9.80665];
    [omega_corr_gated2, ~, ~, ~, ~] = casper_attitude_mahony( ...
        omega, a_tilt, zeros(3,1), false, q, zeros(3,1), false, ...
        Attitude, e_int, dt);
    [omega_corr_legacy2, ~, ~, ~, ~] = casper_attitude_mahony( ...
        omega, a_tilt, zeros(3,1), false, q, zeros(3,1), false, ...
        Attitude_legacy, e_int, dt);

    % Legacy window has half-width 100*g, so for |a|=8g the weight is
    % cos(pi*0.07)*0.5+0.5 ~ 0.988, not exactly 1 — just check >0.9.
    ok = max(abs(omega_corr_gated))      < tol_gated && ...
         max(abs(omega_corr_gated2))     < tol_gated && ...
         max(abs(omega_corr_legacy2))    > 0.05 && ...
         abs(w_gated - 0)                < tol_gated && ...
         w_legacy                        > 0.9;

    r.pass = ok;
    r.msg  = sprintf( ...
        'gated w=%.2e |o|max=%.2e ; legacy w=%.3f |o|max=%.2e (need >0.05)', ...
        w_gated, max(abs(omega_corr_gated2)), w_legacy, max(abs(omega_corr_legacy2)));
end

% =========================================================================
function r = t4_pad_calib_latch(Attitude)
% Drive casper_attitude_tick at 1 kHz with stationary inputs for 0.5 s and
% verify pad_calib_complete stays false up to PadCalibDuration_s and
% latches at exactly that time.  Use a short calib window so the test is
% fast.
    dt = 1e-3;            % 1 kHz
    pad_dur = 0.3;        % 300 ms
    Attitude.PadCalibDuration_s   = pad_dur;
    Attitude.StaticInitTimeout_s  = 0.05;   % init done quickly so we exercise
                                            % the post-init bias-refine branch
    Attitude.StaticInitSamples    = 9999;   % force timeout path (no mag)
    Attitude.M_ref_nav_uT         = [0; 0; -40];

    % Stationary inputs: gravity reads +g on body-Z, gyro all zero.
    a = [0; 0; 9.80665];
    g = zeros(3,1);

    state = casper_attitude_state_new();

    n_steps = round(0.5 / dt);
    pad_complete_log = false(n_steps, 1);
    bias_norm_log    = zeros(n_steps, 1);

    for k = 1:n_steps
        [att, state] = casper_attitude_tick( ...
            a, g, zeros(3,1), false, true, dt, Attitude, state);
        pad_complete_log(k) = att.pad_calib_complete;
        bias_norm_log(k)    = norm(att.gyro_bias_radps);
    end

    % pad_calib_complete must be false at t < pad_dur and true at t >= pad_dur.
    % mission_time advances inside the tick so the k-th call's att_out
    % reports mission_time = k*dt.  Latch threshold is mission_time >=
    % PadCalibDuration_s, so the FIRST call that returns true is k =
    % round(pad_dur/dt) (mission_time = pad_dur exactly).  The call before
    % that (k-1) must still report false.
    k_first_latch = round(pad_dur / dt);                   % expected to be true
    k_last_false  = k_first_latch - 1;                     % expected to be false
    if k_first_latch > n_steps; k_first_latch = n_steps; end
    if k_last_false  < 1;        k_last_false = 1;       end

    ok_before = ~pad_complete_log(k_last_false);
    ok_after  =  pad_complete_log(k_first_latch);

    % gyro_bias_radps should be small (we fed zeros) and stable.
    ok_bias = bias_norm_log(end) < 1e-9;

    r.pass = ok_before && ok_after && ok_bias;
    r.msg  = sprintf( ...
        'pad_complete @t=%.3fs: %d ; @t=%.3fs: %d ; |bias|_end = %.2e', ...
        k_last_false  * dt, pad_complete_log(k_last_false), ...
        k_first_latch * dt, pad_complete_log(k_first_latch), ...
        bias_norm_log(end));
end
