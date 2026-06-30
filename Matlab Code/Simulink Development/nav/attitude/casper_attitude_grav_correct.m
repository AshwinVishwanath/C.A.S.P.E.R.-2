function [e_grav, kp_grav_eff, gate_w] = casper_attitude_grav_correct( ...
        accel_body_fw_mps2, q_body_to_nav, kp_grav, window_half_width_g)
%CASPER_ATTITUDE_GRAV_CORRECT  Gravity-reference Mahony error with magnitude gate.
%
%   Shared between PAD (Mahony, every tick) and FLIGHT (every tick, gated)
%   so the boost-time transient where |a_meas| >> g cannot drag the
%   attitude estimate. Implements the L2.1 cosine window from
%   MAHONY_HARDENING_PRD.md §5 (Level 2).
%
%   Window:
%     x = clip((|a| - g) / (window_half_width_g * g), -1, 1)
%     w = 0.5 * (1 + cos(pi * x))
%   so w = 1 at |a| = g and w = 0 at |a| outside (1 +/- window_half_width_g)*g.
%
%   kp_grav_eff = kp_grav * w.  The legacy un-gated behaviour is recovered
%   by setting window_half_width_g to a very large value (e.g. 100) so the
%   window is open for any realistic accel magnitude (w = 1 always).
%
%   Inputs:
%     accel_body_fw_mps2   (3x1) raw accel, body firmware-frame, m/s^2
%     q_body_to_nav        (4x1) Hamilton, scalar-first
%     kp_grav              (1x1) base gravity-correction gain, Hz-equivalent
%     window_half_width_g  (1x1) half-width of the cosine window, in
%                                multiples of g (0.15 = +/- 0.15 g)
%
%   Outputs:
%     e_grav        (3x1) Mahony gravity error vector, body frame
%                          (cross(a_hat, R' * [0;0;1]))
%     kp_grav_eff   (1x1) gated gain = kp_grav * gate_w
%     gate_w        (1x1) window weight, in [0,1]
%
%   Sign / frame convention matches casper_attitude_mahony.m:
%   nav-frame is Z-up so the gravity-reaction direction is +Z_nav, the
%   accel measurement is positive-up while stationary, and the cross
%   product points along the "rotate-down-to-gravity" axis.

    assert(numel(accel_body_fw_mps2) == 3, 'accel must be 3x1');
    assert(numel(q_body_to_nav)      == 4, 'q must be 4x1');

    a = accel_body_fw_mps2(:);
    q = q_body_to_nav(:);

    G_MPS2 = 9.80665;

    % --- Magnitude window ----------------------------------------------------
    a_mag = sqrt(a(1)*a(1) + a(2)*a(2) + a(3)*a(3));
    if window_half_width_g <= 0
        % Defensive: a non-positive window collapses to "open always".
        gate_w = 1.0;
    else
        hw_mps2 = window_half_width_g * G_MPS2;
        x = (a_mag - G_MPS2) / hw_mps2;
        if x >  1, x =  1; end
        if x < -1, x = -1; end
        gate_w = 0.5 * (1 + cos(pi * x));
    end
    kp_grav_eff = kp_grav * gate_w;

    % --- Mahony gravity error ------------------------------------------------
    n = sqrt(a(1)*a(1) + a(2)*a(2) + a(3)*a(3));
    if n > 1.0e-10
        a_hat = a / n;
    else
        a_hat = zeros(3,1);
    end

    R = quat_to_dcm_(q);
    g_pred = R' * [0; 0; 1];     % gravity-reaction in body frame

    e_grav = [a_hat(2)*g_pred(3) - a_hat(3)*g_pred(2);
              a_hat(3)*g_pred(1) - a_hat(1)*g_pred(3);
              a_hat(1)*g_pred(2) - a_hat(2)*g_pred(1)];
end

% ---------------------------------------------------------------------------
function R = quat_to_dcm_(q)
% Hamilton scalar-first body-to-nav DCM. Inlined to keep this helper free of
% library dependencies (callable from MATLAB Function blocks with %#codegen).
    w = q(1); x = q(2); y = q(3); z = q(4);
    n2 = w*w + x*x + y*y + z*z;
    if n2 <= 0
        n2 = 1;
    end
    s = 2.0 / n2;
    R = [ 1 - s*(y*y + z*z),  s*(x*y - z*w),       s*(x*z + y*w); ...
          s*(x*y + z*w),      1 - s*(x*x + z*z),   s*(y*z - x*w); ...
          s*(x*z - y*w),      s*(y*z + x*w),       1 - s*(x*x + y*y)];
end
