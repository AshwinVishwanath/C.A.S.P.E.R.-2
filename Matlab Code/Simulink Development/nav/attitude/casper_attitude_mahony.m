function [omega_corr_radps, e_int_out, e_grav_out, e_mag_out, ...
          grav_gate_out] = casper_attitude_mahony( ...
        omega_minus_bias_radps, accel_body_fw_mps2, mag_body_fw_uT, ...
        mag_new_sample, q_body_to_nav, m_ref_nav_uT, mag_available, ...
        params, e_int_in, dt_s)
%CASPER_ATTITUDE_MAHONY  Pad-phase complementary correction (accel + mag).
%
%   PRD reference: MAHONY_HARDENING_PRD.md L2.1 + L2.2 + L2.3.
%
%   Mirrors casper_attitude.c §4 PAD branch with the L2 hardening:
%     1) Compute e_grav = cross(a_hat, R' * [0;0;1])  via shared helper
%        casper_attitude_grav_correct, which also returns the magnitude-
%        gated effective gain Kp_grav_eff = Kp_grav * w(|a|).  The window
%        w(.) is the soft cosine bell centred at |a|=g (L2.1).
%     2) if mag: m_pred_hat = normalize(R' * m_ref_nav)
%                e_mag = cross(m_hat, m_pred_hat)
%     3) e_int  += (e_grav + e_mag) * dt          (unchanged for revert)
%     4) omega_corr = omega + Kp_grav_eff*e_grav + Kp_mag_pad*e_mag + Ki*e_int
%
%   Notes on L2 deviations from the legacy behaviour:
%     - The integral accumulator still uses the un-gated e_grav (the PRD
%       only redefines the proportional feedback gain as
%       Kp_grav_eff = Kp_grav*w).  With Ki=0 (the new default per L2.2)
%       this is irrelevant for omega_corr.  If a future revert raises Ki,
%       the integral path will be governed by the same e_grav as in the
%       legacy filter; we leave that to a follow-up workstream.
%     - Kp_mag_pad defaults to 0 per L2.3 until the real Simulink mag
%       noise floor is characterised against the Python validation set.
%
%   Inputs:
%     omega_minus_bias_radps (3x1) gyro after LPF + bias subtract, rad/s
%     accel_body_fw_mps2     (3x1) raw accel, body frame (firmware-frame), m/s^2
%     mag_body_fw_uT         (3x1) calibrated mag, body frame, uT
%                                  (ignored if mag_new_sample == false)
%     mag_new_sample         (bool) true if calibrated mag arrived this tick
%     q_body_to_nav          (4x1) current quaternion
%     m_ref_nav_uT           (3x1) reference mag in nav-frame (from static init)
%     mag_available          (bool) static init resolved mag
%     params                 struct: .Kp_Grav, .Kp_MagPad, .Ki,
%                                    .GravGate_WindowHalfWidth_g (default 0.15)
%     e_int_in               (3x1) accumulated integral error, rad/s (history)
%     dt_s                   (1x1) sample period, s
%
%   Outputs:
%     omega_corr_radps       (3x1) corrected omega for RK4 step, rad/s
%     e_int_out              (3x1) updated integral
%     e_grav_out             (3x1) gravity error (un-gated, for telemetry/debug)
%     e_mag_out              (3x1) mag error (zero if mag unavailable)
%     grav_gate_out          (1x1) magnitude-window weight in [0,1] this tick

    assert(numel(omega_minus_bias_radps) == 3);
    assert(numel(accel_body_fw_mps2)     == 3);
    assert(numel(q_body_to_nav)          == 4);
    assert(numel(m_ref_nav_uT)           == 3);
    assert(numel(e_int_in)               == 3);

    omega = omega_minus_bias_radps(:);
    q     = q_body_to_nav(:);
    mref  = m_ref_nav_uT(:);
    e_int = e_int_in(:);

    % --- Magnitude-gated gravity correction (L2.1) --------------------------
    if isfield(params, 'GravGate_WindowHalfWidth_g')
        win_hw_g = params.GravGate_WindowHalfWidth_g;
    else
        % Backward-compatible default: legacy behaviour (window wide open).
        win_hw_g = 100.0;
    end
    [e_grav, kp_grav_eff, gate_w] = casper_attitude_grav_correct( ...
        accel_body_fw_mps2, q, params.Kp_Grav, win_hw_g);

    % --- Mag error (un-gated; L2.3 default Kp_MagPad=0) ---------------------
    e_mag = zeros(3,1);
    if mag_new_sample && mag_available
        assert(numel(mag_body_fw_uT) == 3);
        m_hat       = unit3(mag_body_fw_uT(:));
        ops         = casper_quat_ops();
        R           = ops.to_dcm(q);
        m_pred      = R' * mref;
        m_pred_hat  = unit3(m_pred);
        e_mag       = cross3(m_hat, m_pred_hat);
    end

    % --- Integral (firmware uses raw error, no normalization; PRD note: we
    % keep this un-gated so a future Ki>0 revert reproduces legacy maths) ---
    e_int = e_int + (e_grav + e_mag) * dt_s;

    omega_corr_radps = omega ...
        + kp_grav_eff      * e_grav ...
        + params.Kp_MagPad * e_mag ...
        + params.Ki        * e_int;

    e_int_out     = e_int;
    e_grav_out    = e_grav;
    e_mag_out     = e_mag;
    grav_gate_out = gate_w;
end

% ---------------------------------------------------------------------------
function u = unit3(v)
    n = sqrt(v(1)*v(1) + v(2)*v(2) + v(3)*v(3));
    if n > 1.0e-10
        u = v / n;
    else
        u = zeros(3,1);
    end
end

% ---------------------------------------------------------------------------
function r = cross3(a, b)
    r = [a(2)*b(3) - a(3)*b(2);
         a(3)*b(1) - a(1)*b(3);
         a(1)*b(2) - a(2)*b(1)];
end
