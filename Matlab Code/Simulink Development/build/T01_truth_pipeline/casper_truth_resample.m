function truth = casper_truth_resample(raw, dt, t_max)
%CASPER_TRUTH_RESAMPLE Interpolate raw RasAero struct onto a uniform grid.
%
% Synopsis:
%   truth = casper_truth_resample(raw, dt, t_max)
%
% Inputs:
%   raw   : struct from casper_rasaero_ingest()
%   dt    : scalar, sample period in seconds (default 1e-4 = 10 kHz)
%   t_max : scalar, end time in seconds (default 549)
%
% Outputs:
%   truth : struct conforming to TruthBus (SIMULINK_PATTERNS.md §7), with fields
%       time_s            (Mx1)
%       mach              (Mx1)
%       pos_NED           (Mx3, meters, Z down so altitude = -pos_NED(:,3))
%       vel_NED           (Mx3, m/s)
%       accel_NED         (Mx3, m/s^2, world-frame net, gravity-free)
%       quat_std          (Mx4, scalar-first [w x y z], body-to-NED)
%       omega_body_std    (Mx3, body-frame angular velocity rad/s)
%       air_density_kgm3  (Mx1)
%       air_temp_K        (Mx1)
%       air_pressure_pa   (Mx1)
%       % auxiliary helpers (informational, not part of bus contract):
%       alt_m             (Mx1, altitude up, convenience)
%       vel_v_mps         (Mx1)
%       accel_v_mps2      (Mx1)
%       pitch_deg         (Mx1)
%       stage             (Mx1 string)
%       dt_s              (scalar)
%       n_samples         (scalar)
%
% Algorithm:
%   - PCHIP interpolation for all numeric fields (shape-preserving cubic
%     Hermite avoids spline ringing at burnout discontinuity, and avoids
%     linear-interp phase smear that would create a velocity bias).
%   - 'previous' interpolation for the stage string (anti-goal #1).
%   - extrap value 0 for samples past raw t-range.
%   - World-frame: NED (Z down). altitude_up -> pos_NED Z is negative.
%   - Phase 0 vertical assumption: horizontal pos/vel/accel are zero.
%   - Attitude: quat = eul2quat([0, pitch_rad, 0], 'ZYX'), scalar-first.
%   - omega_body = body-frame angular rate, computed by numerical
%     differentiation of the quaternion. For a purely-vertical pitch,
%     only the pitch-rate axis (Y-body) is nonzero.
%   - Atmosphere: atmosisa (ISA standard) at each altitude.
%
% Source firmware reference:
%   Frame: ARCHITECTURE.md §3.1 (NED + standard aircraft body).

    arguments
        raw   struct
        dt    (1,1) double = 1e-4
        t_max (1,1) double = 549
    end

    if dt <= 0
        error('casper_truth_resample:BadDt','dt must be positive.');
    end
    if t_max <= 0
        error('casper_truth_resample:BadTmax','t_max must be positive.');
    end

    t_uniform = (0:dt:t_max).';
    M = numel(t_uniform);

    % --- Numeric PCHIP interpolation ---
    mach         = interp1(raw.t_s, raw.mach,         t_uniform, 'pchip', 0);
    alt_m        = interp1(raw.t_s, raw.alt_m,        t_uniform, 'pchip', 0);
    vel_v_mps    = interp1(raw.t_s, raw.vel_v_mps,    t_uniform, 'pchip', 0);
    accel_v_mps2 = interp1(raw.t_s, raw.accel_v_mps2, t_uniform, 'pchip', 0);
    pitch_deg    = interp1(raw.t_s, raw.pitch_deg,    t_uniform, 'pchip', 0);

    % --- Stage string: 'previous' / nearest-prior ---
    % interp1 cannot operate on strings directly; map to integer codes first.
    [stage_codes_raw, code_map] = stage_to_codes(raw.stage);
    stage_codes = interp1(raw.t_s, double(stage_codes_raw), t_uniform, ...
        'previous', double(stage_codes_raw(1)));
    stage = codes_to_stage(stage_codes, code_map);

    % --- World-frame NED (Z down) ---
    zero_col  = zeros(M, 1);
    pos_NED   = [zero_col, zero_col, -alt_m];
    vel_NED   = [zero_col, zero_col, -vel_v_mps];
    accel_NED = [zero_col, zero_col, -accel_v_mps2];

    % --- Attitude quaternion (scalar-first, body-to-NED) ---
    % Per T01 §6.2 step 5: eul2quat([yaw=0, pitch_rad, roll=0], 'ZYX').
    % MATLAB returns N-by-4 with column 1 = scalar (w).
    pitch_rad = pitch_deg * pi / 180;
    eul = [zero_col, pitch_rad, zero_col];
    quat_std = eul2quat(eul, 'ZYX');   % M-by-4, [w x y z]

    % --- Body-frame angular velocity from d(quat)/dt ---
    % omega_body = 2 * quat_conj * dq/dt   (Hamilton convention)
    % For a single pitch axis the result is [0, omega_y, 0] in body frame.
    dq_dt = central_diff(quat_std, dt);
    omega_body_std = quat_rate_to_body_omega(quat_std, dq_dt);

    % --- Atmosphere via ISA (atmosisa is in Aerospace Toolbox) ---
    % atmosisa returns scalars/vectors of temperature(K), speed_sound(m/s),
    % pressure(Pa), density(kg/m^3). Clip altitude to non-negative for the
    % model (negative altitudes show up only as numerical artefacts past
    % the trajectory end).
    alt_for_isa = max(alt_m, 0);
    [air_temp_K, ~, air_pressure_pa, air_density_kgm3] = atmosisa(alt_for_isa);

    % --- Assemble truth struct (TruthBus + helpers) ---
    truth = struct();
    truth.time_s            = t_uniform;
    truth.mach              = mach(:);
    truth.pos_NED           = pos_NED;
    truth.vel_NED           = vel_NED;
    truth.accel_NED         = accel_NED;
    truth.quat_std          = quat_std;
    truth.omega_body_std    = omega_body_std;
    truth.air_density_kgm3  = air_density_kgm3(:);
    truth.air_temp_K        = air_temp_K(:);
    truth.air_pressure_pa   = air_pressure_pa(:);
    % helpers (informational, not in bus contract)
    truth.alt_m             = alt_m(:);
    truth.vel_v_mps         = vel_v_mps(:);
    truth.accel_v_mps2      = accel_v_mps2(:);
    truth.pitch_deg         = pitch_deg(:);
    truth.stage             = stage(:);
    truth.dt_s              = dt;
    truth.n_samples         = M;
end

% ---------------------------------------------------------------------------

function [codes, code_map] = stage_to_codes(stage_strs)
    code_map = unique(stage_strs, 'stable');   % preserves first-seen order
    codes = zeros(numel(stage_strs), 1);
    for k = 1:numel(code_map)
        codes(stage_strs == code_map(k)) = k;
    end
end

function s = codes_to_stage(codes, code_map)
    codes = round(codes);
    codes(codes < 1) = 1;
    codes(codes > numel(code_map)) = numel(code_map);
    s = code_map(codes);
end

function dq = central_diff(q, dt)
% Central difference for interior, forward/backward for endpoints.
    [M, K] = size(q);
    dq = zeros(M, K);
    if M < 2
        return;
    end
    dq(1, :)       = (q(2, :)   - q(1, :))     / dt;
    dq(end, :)     = (q(end, :) - q(end-1, :)) / dt;
    if M >= 3
        dq(2:end-1, :) = (q(3:end, :) - q(1:end-2, :)) / (2 * dt);
    end
end

function omega = quat_rate_to_body_omega(q, dq)
%QUAT_RATE_TO_BODY_OMEGA Convert (q, dq/dt) to body-frame angular rate.
%   Hamilton scalar-first convention. omega_body = 2 * conj(q) * dq/dt
%   (and take the vector part). Inputs are Mx4 with column 1 = scalar.
    M = size(q, 1);
    omega = zeros(M, 3);
    for k = 1:M
        qk  = q(k, :);
        dqk = dq(k, :);
        qc  = [qk(1), -qk(2), -qk(3), -qk(4)];
        % Hamilton product qc (x) dqk  -- result = 2 * [0; omega_body]
        w =  qc(1)*dqk(1) - qc(2)*dqk(2) - qc(3)*dqk(3) - qc(4)*dqk(4);
        x =  qc(1)*dqk(2) + qc(2)*dqk(1) + qc(3)*dqk(4) - qc(4)*dqk(3);
        y =  qc(1)*dqk(3) - qc(2)*dqk(4) + qc(3)*dqk(1) + qc(4)*dqk(2);
        z =  qc(1)*dqk(4) + qc(2)*dqk(3) - qc(3)*dqk(2) + qc(4)*dqk(1);
        % w should be ~0 for a unit quat with consistent dq; ignore.
        omega(k, :) = 2 * [x, y, z];
        %#ok<*NASGU> -- 'w' intentionally unused
    end
end
