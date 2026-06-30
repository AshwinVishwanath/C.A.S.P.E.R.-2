function [omega_corr_radps, e_int_out, fired, mag_timer_out, ...
          heading_sigma_out, e_mag_out, e_grav_out, grav_gate_out] = ...
              casper_attitude_mag_correct_flight( ...
        omega_minus_bias_radps, accel_body_fw_mps2, ...
        mag_body_fw_uT, mag_new_sample, ...
        q_body_to_nav, m_ref_nav_uT, mag_available, params, e_int_in, ...
        mag_timer_in, heading_sigma_in, dt_s)
%CASPER_ATTITUDE_MAG_CORRECT_FLIGHT  Flight-phase Mahony correction.
%
%   PRD reference: MAHONY_HARDENING_PRD.md L2.1 (gate also active in flight).
%
%   This routine now does TWO corrections per tick:
%
%     (1) Magnitude-gated gravity correction, EVERY tick.  Mirrors the PAD
%         Mahony's gravity term but with the cosine-window gate active so
%         that the boost-time |a| >> g transient does not drag the
%         attitude estimate.  Outside the magnitude window the gain is
%         driven to zero, leaving omega untouched (apart from the slower
%         mag tick).  This is the L2.1 "must apply in both PAD and FLIGHT
%         phases" requirement.
%
%     (2) 10 Hz tilt-comp mag correction (firmware-canonical):
%         - mag_update_timer += dt every call.
%         - When timer >= 1/mag_update_hz (0.1 s for 10 Hz):
%             if mag_cal && mag_available && !gated:
%                 - e_mag = cross(m_hat, normalize(R' * m_ref))
%                 - e_int += e_mag * dt_corr
%                 - omega_corr += Kp_mag_flight * e_mag
%                 - heading_sigma decays toward HEADING_SIGMA_FLOOR
%                 - reset timer
%             else: leave timer running (fires next tick with valid data)
%
%   The gravity-error contribution to e_int is added every tick (matches
%   PAD behaviour so the PAD->FLIGHT transition does not cause an integral
%   discontinuity).  Ki defaults to 0 (L2.2) so this has no effect on
%   omega_corr at default settings; the path is exercised and ready for
%   activation if Ki is raised.
%
%   In Phase 0 with Kp_MagFlight=0 the mag branch is a NO-OP for omega
%   and heading_sigma_floor (the timer still ticks and gravity correction
%   still runs).  The path is exercised and ready for activation.
%
%   Inputs / outputs documented inline.

    assert(numel(omega_minus_bias_radps) == 3);
    assert(numel(accel_body_fw_mps2)     == 3);
    assert(numel(q_body_to_nav)          == 4);
    assert(numel(m_ref_nav_uT)           == 3);
    assert(numel(e_int_in)               == 3);

    omega = omega_minus_bias_radps(:);
    q     = q_body_to_nav(:);
    mref  = m_ref_nav_uT(:);
    e_int = e_int_in(:);

    % --- (1) Magnitude-gated gravity correction (L2.1, every tick) ---------
    if isfield(params, 'GravGate_WindowHalfWidth_g')
        win_hw_g = params.GravGate_WindowHalfWidth_g;
    else
        win_hw_g = 100.0;   % legacy: open window
    end
    [e_grav, kp_grav_eff, grav_gate_out] = casper_attitude_grav_correct( ...
        accel_body_fw_mps2, q, params.Kp_Grav, win_hw_g);

    % Accumulate gravity contribution into the integral every tick (matches
    % PAD).  Mag contribution is added below only when the mag tick fires.
    e_int = e_int + e_grav * dt_s;

    omega_corr_radps  = omega + kp_grav_eff * e_grav;
    e_int_out         = e_int;
    fired             = false;
    e_mag_out         = zeros(3,1);
    e_grav_out        = e_grav;
    mag_timer_out     = mag_timer_in + dt_s;
    heading_sigma_out = heading_sigma_in;

    % --- (2) Timer-driven mag correction ------------------------------------
    period_s = 1 / params.MagUpdateRate_Hz;

    if mag_timer_out < period_s
        % Apply Ki integral feedback this tick (gravity-only integral).
        omega_corr_radps = omega_corr_radps + params.Ki * e_int_out;
        return;                       % not yet time for mag
    end

    % Timer has fired — attempt correction
    if ~mag_new_sample || ~mag_available
        % No fresh mag this tick — leave timer running, retry next tick.
        % (firmware: do NOT reset timer; fall-through to no-op)
        omega_corr_radps = omega_corr_radps + params.Ki * e_int_out;
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
        + kp_grav_eff           * e_grav ...
        + params.Kp_MagFlight   * e_mag ...
        + params.Ki             * e_int;

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
