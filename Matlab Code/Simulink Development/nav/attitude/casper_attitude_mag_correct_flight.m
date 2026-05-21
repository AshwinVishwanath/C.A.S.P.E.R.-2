function [omega_corr_radps, e_int_out, fired, mag_timer_out, ...
          heading_sigma_out, e_mag_out] = casper_attitude_mag_correct_flight( ...
        omega_minus_bias_radps, mag_body_fw_uT, mag_new_sample, ...
        q_body_to_nav, m_ref_nav_uT, mag_available, params, e_int_in, ...
        mag_timer_in, heading_sigma_in, dt_s)
%CASPER_ATTITUDE_MAG_CORRECT_FLIGHT  10 Hz tilt-comp mag correction.
%
%   Mirrors casper_attitude.c §4 FLIGHT branch:
%     - mag_update_timer += dt every call.
%     - When timer >= 1/mag_update_hz (0.1 s for 10 Hz):
%         if mag_cal && mag_available && !gated:
%             - Build e_mag = cross(m_hat, normalize(R' * m_ref))
%             - e_int += e_mag * dt_corr  (dt_corr = timer value at fire)
%             - omega_corr = omega + Kp_mag_flight * e_mag + Ki * e_int
%             - heading_sigma decays toward HEADING_SIGMA_FLOOR
%             - reset timer
%         else: leave timer running (fires next tick with valid data)
%
%   NOTE on tilt-compensation: the firmware does NOT do an explicit yaw-only
%   projection. Its mag error is the full 3D cross product, which the gyro
%   integration naturally constrains. The spec §9.1 describes an alternative
%   formulation (project onto horizontal then take cross product). We match
%   FIRMWARE behavior here because CLAUDE.md / FIRMWARE_CONSTANTS.md make
%   firmware canonical. The spec's "tilt-compensated" language is satisfied
%   by the body-frame nature of the cross product (R' rotates the reference
%   into the body frame, which is implicitly tilt-compensated).
%
%   In Phase 0, Kp_MagFlight = 0 so the correction is a NO-OP for omega and
%   heading_sigma_floor. The path is exercised and ready for activation.
%
%   Inputs / outputs documented inline.

    assert(numel(omega_minus_bias_radps) == 3);
    assert(numel(q_body_to_nav)          == 4);
    assert(numel(m_ref_nav_uT)           == 3);
    assert(numel(e_int_in)               == 3);

    omega = omega_minus_bias_radps(:);
    q     = q_body_to_nav(:);
    mref  = m_ref_nav_uT(:);
    e_int = e_int_in(:);

    omega_corr_radps  = omega;        % default: no correction this tick
    e_int_out         = e_int;
    fired             = false;
    e_mag_out         = zeros(3,1);
    mag_timer_out     = mag_timer_in + dt_s;
    heading_sigma_out = heading_sigma_in;

    period_s = 1 / params.MagUpdateRate_Hz;

    if mag_timer_out < period_s
        return;                       % not yet time
    end

    % Timer has fired — attempt correction
    if ~mag_new_sample || ~mag_available
        % No fresh mag this tick — leave timer running, retry next tick.
        % (firmware: do NOT reset timer; fall-through to no-op)
        return;
    end

    assert(numel(mag_body_fw_uT) == 3);
    mag_body_fw_uT = mag_body_fw_uT(:);

    ops = casper_quat_ops();
    R   = ops.to_dcm(q);

    m_hat      = unit3(mag_body_fw_uT);
    m_pred     = R' * mref;
    m_pred_hat = unit3(m_pred);
    e_mag      = cross3(m_hat, m_pred_hat);

    dt_corr = mag_timer_out;
    e_int   = e_int + e_mag * dt_corr;

    omega_corr_radps = omega ...
        + params.Kp_MagFlight * e_mag ...
        + params.Ki * e_int;

    % Heading sigma decay (matches casper_attitude.c L299-302)
    alpha_corr = params.Kp_MagFlight * dt_corr;
    if alpha_corr > 1
        alpha_corr = 1;
    end
    heading_sigma_out = heading_sigma_in * (1 - alpha_corr) ...
                      + params.HeadingSigmaFloor_rad * alpha_corr;

    % Reset timer only on successful correction
    mag_timer_out = 0;
    fired         = true;
    e_int_out     = e_int;
    e_mag_out     = e_mag;
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
