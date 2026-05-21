function [omega_corr_radps, e_int_out, e_grav_out, e_mag_out] = casper_attitude_mahony( ...
        omega_minus_bias_radps, accel_body_fw_mps2, mag_body_fw_uT, ...
        mag_new_sample, q_body_to_nav, m_ref_nav_uT, mag_available, ...
        params, e_int_in, dt_s)
%CASPER_ATTITUDE_MAHONY  Pad-phase complementary correction (accel + mag).
%
%   Mirrors casper_attitude.c §4 PAD branch:
%     1) g_pred = R' * [0;0;1]            (gravity-reaction direction in body)
%     2) e_grav = cross(a_hat, g_pred)
%     3) if mag: m_pred_hat = normalize(R' * m_ref_nav)
%               e_mag = cross(m_hat, m_pred_hat)
%     4) e_int  += (e_grav + e_mag) * dt
%     5) omega_corr = omega + Kp_grav*e_grav + Kp_mag_pad*e_mag + Ki*e_int
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
%     params                 struct: .Kp_Grav, .Kp_MagPad, .Ki
%     e_int_in               (3x1) accumulated integral error, rad/s (history)
%     dt_s                   (1x1) sample period, s
%
%   Outputs:
%     omega_corr_radps       (3x1) corrected omega for RK4 step, rad/s
%     e_int_out              (3x1) updated integral
%     e_grav_out             (3x1) gravity error (for telemetry/debug)
%     e_mag_out              (3x1) mag error (zero if mag unavailable)

    assert(numel(omega_minus_bias_radps) == 3);
    assert(numel(accel_body_fw_mps2)     == 3);
    assert(numel(q_body_to_nav)          == 4);
    assert(numel(m_ref_nav_uT)           == 3);
    assert(numel(e_int_in)               == 3);

    omega = omega_minus_bias_radps(:);
    a     = accel_body_fw_mps2(:);
    q     = q_body_to_nav(:);
    mref  = m_ref_nav_uT(:);
    e_int = e_int_in(:);

    ops = casper_quat_ops();
    R   = ops.to_dcm(q);

    % Gravity error: e_grav = cross(a_hat, R' * [0;0;1])
    a_hat   = unit3(a);
    g_nav   = [0; 0; 1];
    g_pred  = R' * g_nav;
    e_grav  = cross3(a_hat, g_pred);

    % Mag error
    e_mag = zeros(3,1);
    if mag_new_sample && mag_available
        assert(numel(mag_body_fw_uT) == 3);
        m_hat       = unit3(mag_body_fw_uT(:));
        m_pred      = R' * mref;
        m_pred_hat  = unit3(m_pred);
        e_mag       = cross3(m_hat, m_pred_hat);
    end

    % Integral (firmware uses raw error, no normalization)
    e_int = e_int + (e_grav + e_mag) * dt_s;

    omega_corr_radps = omega ...
        + params.Kp_Grav  * e_grav ...
        + params.Kp_MagPad * e_mag ...
        + params.Ki * e_int;

    e_int_out  = e_int;
    e_grav_out = e_grav;
    e_mag_out  = e_mag;
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
