function q_new = casper_attitude_predict_rk4(q_prev, omega_body_radps, dt_s)
%CASPER_ATTITUDE_PREDICT_RK4  RK4 quaternion propagation, Hamilton.
%
%   Mirrors casper_attitude.c §5 (RK4 block) exactly. The same omega is
%   used for all four sub-steps (no inter-sample interpolation).
%
%   Inputs:
%     q_prev            (4x1) Hamilton quaternion, body-to-nav, scalar-first
%     omega_body_radps  (3x1) angular rate, body frame, rad/s (bias-corrected)
%     dt_s              (1x1) propagation step, s (1/833 in firmware)
%
%   Output:
%     q_new             (4x1) propagated quaternion, normalized, w>=0
%
%   Per spec §7.2: normalize and sign-fix after the final aggregation.
%   The firmware does NOT renormalize between k1..k4 substeps; spec text
%   §7.1 mentions normalize(q_mid_*) but firmware does not. We match
%   firmware (no per-substep normalize); the final normalize covers drift.

    assert(numel(q_prev) == 4,           'q_prev must be 4x1');
    assert(numel(omega_body_radps) == 3, 'omega must be 3x1');
    q = q_prev(:);
    omega = omega_body_radps(:);

    k1 = quat_derivative(q, omega);

    qt = q + k1 * dt_s * 0.5;
    k2 = quat_derivative(qt, omega);

    qt = q + k2 * dt_s * 0.5;
    k3 = quat_derivative(qt, omega);

    qt = q + k3 * dt_s;
    k4 = quat_derivative(qt, omega);

    q_new = q + (dt_s / 6) * (k1 + 2*k2 + 2*k3 + k4);

    % Final normalize + sign enforcement (matches firmware
    % casper_quat_normalize; we additionally enforce w>=0 per spec §4).
    n = sqrt(q_new(1)^2 + q_new(2)^2 + q_new(3)^2 + q_new(4)^2);
    if n > 1.0e-12
        q_new = q_new / n;
    else
        q_new = [1; 0; 0; 0];
    end
    if q_new(1) < 0
        q_new = -q_new;
    end
end

% ---------------------------------------------------------------------------
function qdot = quat_derivative(q, omega)
    % qdot = 0.5 * q (x) [0; omega]  (Hamilton).  Inlined for speed.
    q = q(:);
    aw = q(1); ax = q(2); ay = q(3); az = q(4);
    bw = 0;
    bx = omega(1); by = omega(2); bz = omega(3);
    qdot = zeros(4,1);
    qdot(1) = 0.5 * (aw*bw - ax*bx - ay*by - az*bz);
    qdot(2) = 0.5 * (aw*bx + ax*bw + ay*bz - az*by);
    qdot(3) = 0.5 * (aw*by - ax*bz + ay*bw + az*bx);
    qdot(4) = 0.5 * (aw*bz + ax*by - ay*bx + az*bw);
end
