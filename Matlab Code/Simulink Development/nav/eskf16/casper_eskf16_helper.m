function [pos_NED, vel_NED, att_quat, bg, ba, bb, ...
          sigma_pos, sigma_vel, sigma_att, baro_gate_on, ...
          baro_innov, init_armed] = casper_eskf16_helper( ...
              gyro_body_zup, accel_body_zup, baro_alt_up_m, ...
              baro_new, mag_body_zup, mag_new, ...
              q_init_fw, init_done, reset_flag, dt)
%CASPER_ESKF16_HELPER  Persistent-state 16-state error-state EKF wrapper
% for the Simulink visual model. Faithful port of EKF16Verify.m §7 main
% loop (Joseph-form scalar updates, attitude reset on injection, baro
% Mach gating with hysteresis, mag suppression during boost).
%
% FRAME CONVENTION
%   The EKF16 algorithm (per EKF_Symbolic_Dev.m + EKF16Verify.m) is
%   "body Z = up at pad, NED nav frame". On the pad the quaternion
%   q_pad = [0;0;1;0] maps body +Z to NED -Z (i.e. body Z aligns with
%   gravity-opposite). The accelerometer in this convention reads
%   specific force = a_true - g, so on the pad it reads [0;0;+g] in
%   body (+Z up against gravity).
%
%   The CASPER-2 visual model upstream uses "Y-nose body / Z-up nav"
%   (firmware convention). This helper expects the caller to ROTATE the
%   IMU body-fw measurements into the EKF16 body convention BEFORE
%   passing them in. See casper_eskf16_body_fw_to_zup for the const
%   rotation. The output attitude quat is therefore in EKF16's
%   "body-Zup -> NED" convention.
%
% Inputs:
%   gyro_body_zup   (3x1, rad/s) IMU gyro in body-Zup
%   accel_body_zup  (3x1, m/s^2) IMU specific force in body-Zup
%                                (on pad ~ [0;0;+g])
%   baro_alt_up_m   (1x1, m)     barometric altitude AGL (up positive)
%   baro_new        (logical)    true on baro data-ready ticks
%   mag_body_zup    (3x1, uT)    mag field in body-Zup
%   mag_new         (logical)    true on mag data-ready ticks
%   q_init_fw       (4x1)        attitude block's q_fw at init (one-shot)
%   init_done       (logical)    attitude init_complete (gates EKF)
%   reset_flag      (logical)    true forces persistent-state reset
%   dt              (1x1, s)     time step (caller passes 2 ms snapped)
%
% Outputs (all in NED nav frame, EKF16 body convention):
%   pos_NED     (3x1, m)
%   vel_NED     (3x1, m/s)
%   att_quat    (4x1)        body-Zup -> NED Hamilton scalar-first
%   bg          (3x1, rad/s) gyro bias estimate
%   ba          (3x1, m/s^2) accel bias estimate
%   bb          (1x1, m)     baro bias estimate
%   sigma_pos   (3x1, m)     1-sigma position uncertainty
%   sigma_vel   (3x1, m/s)
%   sigma_att   (3x1, deg)
%   baro_gate_on (logical)
%   baro_innov  (1x1, m)     last accepted baro innovation (NaN if none)
%   init_armed  (logical)    true once filter has been seeded
%
% Reference:
%   Matlab Code/EKF Dev/EKF16Verify.m (canonical algorithm)
%   Matlab Code/EKF Dev/EKF_Symbolic_Dev.m (F/Q/H derivations)

    persistent st params mag_ref_ned I16

    % ------------------------------------------------------------------
    % First-time init or explicit reset.
    % ------------------------------------------------------------------
    if isempty(st) || logical(reset_flag)
        params = build_default_params_();

        % Mag reference in NED, uT. Prefer the visual model's truth field
        % (casper_mag_field_world) so the mag update sees the same NED-frame
        % reference it would in flight. Fall back to the canonical
        % EKF16Verify [20, 0.5, 43] uT if the helper isn't on the path.
        if exist('casper_mag_field_world', 'file')
            try
                mag_ref_ned = casper_mag_field_world([0;0;0]);
                mag_ref_ned = mag_ref_ned(:);
            catch
                mag_ref_ned = [20.0; 0.5; 43.0];
            end
        else
            mag_ref_ned = [20.0; 0.5; 43.0];
        end

        I16 = eye(16);

        st = make_init_state_();
    end

    % ------------------------------------------------------------------
    % Pre-init hold: until attitude says go, keep returning zeros.
    % ------------------------------------------------------------------
    if ~logical(init_done) && ~st.armed
        % Hold the at-rest container so downstream scopes see flat lines.
        pos_NED      = st.p_ref;
        vel_NED      = st.v_ref;
        att_quat     = st.q_ref;
        bg           = st.bg_ref;
        ba           = st.ba_ref;
        bb           = st.bb_ref;
        sigma_pos    = sqrt(abs(diag(st.P(7:9,7:9))));
        sigma_vel    = sqrt(abs(diag(st.P(4:6,4:6))));
        sigma_att    = sqrt(abs(diag(st.P(1:3,1:3)))) * 180/pi;
        baro_gate_on = st.baro_gate_on;
        baro_innov   = NaN;
        init_armed   = false;
        return;
    end

    % ------------------------------------------------------------------
    % One-shot seed on rising edge of init_done.
    % ------------------------------------------------------------------
    if logical(init_done) && ~st.armed
        % Seed quaternion. q_init_fw arrives in the EKF16 body-Zup -> NED
        % convention (the Simulink block converts the firmware fw-quat to
        % Zup before passing it in via casper_eskf16_seed_from_fw). Callers
        % may also pass the canonical q_pad = [0;0;1;0] directly.
        q_seed = double(q_init_fw(:));
        nq = norm(q_seed);
        if nq < 1e-6 || ~all(isfinite(q_seed))
            % Fallback: visual-model body-Zup pad quat (body+X=East,
            % body+Y=North, body+Z=Up). See build_casper_sim_phase0.m
            % C_e16_qinit block comment for the derivation.
            q_seed = [0; sqrt(2)/2; sqrt(2)/2; 0];
        else
            q_seed = q_seed / nq;
        end
        st.q_ref = q_seed;
        st.armed = true;
    end

    % If we still have no init signal but were already armed (latched),
    % keep running — the legacy EKF behaviour is to never un-arm.
    if ~st.armed
        pos_NED      = st.p_ref;
        vel_NED      = st.v_ref;
        att_quat     = st.q_ref;
        bg           = st.bg_ref;
        ba           = st.ba_ref;
        bb           = st.bb_ref;
        sigma_pos    = sqrt(abs(diag(st.P(7:9,7:9))));
        sigma_vel    = sqrt(abs(diag(st.P(4:6,4:6))));
        sigma_att    = sqrt(abs(diag(st.P(1:3,1:3)))) * 180/pi;
        baro_gate_on = st.baro_gate_on;
        baro_innov   = NaN;
        init_armed   = false;
        return;
    end

    % ------------------------------------------------------------------
    % §7 main loop body (one tick).
    % ------------------------------------------------------------------
    dt_use = double(dt);
    if ~(dt_use > 0)
        dt_use = params.dt_ekf;
    end

    gyro_raw  = double(gyro_body_zup(:));
    accel_raw = double(accel_body_zup(:));

    % --- IMU bias correction (current best estimates) ---
    omega_corr = gyro_raw  - st.bg_ref;
    f_corr     = accel_raw - st.ba_ref;

    % --- Reference state propagation ---
    dq = [1; omega_corr * dt_use / 2];
    st.q_ref = quatmult_(st.q_ref, dq);
    nq = norm(st.q_ref);
    if nq > 1e-12
        st.q_ref = st.q_ref / nq;
    end
    R_bn = quat2Tbn_(st.q_ref);

    f_ned = R_bn * f_corr;
    st.v_ref = st.v_ref + (f_ned + [0; 0; params.gravity]) * dt_use;
    st.p_ref = st.p_ref + st.v_ref * dt_use;

    % --- EKF predict (hand-coded F + Q for speed, byte-equivalent to
    % the symbolic derivation in EKF_Symbolic_Dev.m) ---
    F_k = build_F_(R_bn, omega_corr, f_corr, dt_use);
    Q_k = build_Q_(R_bn, dt_use, params);

    st.P = F_k * st.P * F_k' + Q_k;
    st.P = 0.5 * (st.P + st.P');

    % --- ZUPT (pre-launch only, here gated by accel < 1.5 g threshold) ---
    % EKF16Verify treats pad as ph==0; in the visual model we don't have
    % a clean phase signal, so emulate "stationary if total accel magnitude
    % is close to gravity AND speed is small". This mirrors the legacy
    % 4-state ESKF's ZUPT trigger convention.
    speed = norm(st.v_ref);
    a_mag_dev = abs(norm(accel_raw) - params.gravity);
    do_zupt = (speed < 1.0) && (a_mag_dev < 1.5);

    if do_zupt
        for ax = 4:6
            innov = -st.v_ref(ax - 3);
            S = st.P(ax,ax) + params.R_zupt;
            K = st.P(:,ax) / S;
            st.x_err = st.x_err + K * innov;
            IKH = I16; IKH(:,ax) = IKH(:,ax) - K;
            st.P = IKH * st.P * IKH' + params.R_zupt * (K * K');
            st.P = 0.5 * (st.P + st.P');
        end
        [st.q_ref, st.v_ref, st.p_ref, st.bg_ref, st.ba_ref, st.bb_ref] = ...
            apply_correction_(st.q_ref, st.v_ref, st.p_ref, ...
                              st.bg_ref, st.ba_ref, st.bb_ref, st.x_err);
        st.x_err = zeros(16, 1);
    end

    % --- Baro update (Mach-gated with hysteresis) ---
    alt_up_est = -st.p_ref(3);
    vel_up_est = -st.v_ref(3);
    sos_est = isa_sos_(alt_up_est);
    mach_est = abs(vel_up_est) / max(sos_est, 1.0);

    if ~st.baro_gate_on && mach_est > params.mach_gate_on
        st.baro_gate_on = true;
    elseif st.baro_gate_on && mach_est < params.mach_gate_off
        st.baro_gate_on = false;
    end

    baro_innov = NaN;
    if logical(baro_new) && ~st.baro_gate_on
        z_baro = double(baro_alt_up_m);
        z_hat  = -st.p_ref(3) - st.bb_ref;
        innov  = z_baro - z_hat;

        H_b = zeros(1, 16); H_b(9) = -1; H_b(16) = -1;
        S = H_b * st.P * H_b' + params.R_baro;
        if innov^2 <= params.baro_gate_sigma^2 * S
            K = st.P * H_b' / S;
            st.x_err = st.x_err + K * innov;
            IKH = I16 - K * H_b;
            st.P = IKH * st.P * IKH' + params.R_baro * (K * K');
            st.P = 0.5 * (st.P + st.P');

            [st.q_ref, st.v_ref, st.p_ref, st.bg_ref, st.ba_ref, st.bb_ref] = ...
                apply_correction_(st.q_ref, st.v_ref, st.p_ref, ...
                                  st.bg_ref, st.ba_ref, st.bb_ref, st.x_err);
            st.x_err = zeros(16, 1);
            baro_innov = innov;
        end

        if st.P(16,16) < 0.01, st.P(16,16) = 0.01; end
    end

    % --- Mag update (gated during boost == high vertical accel) ---
    a_z_body = accel_raw(3);  % body-Zup so on-pad ~ +g; during boost >> g
    boost_active = (a_z_body > 2.0 * params.gravity);
    if logical(mag_new) && ~boost_active && all(isfinite(mag_body_zup))
        % H_MAG block on attitude states = skew(Tbn' * m_ref); zeros elsewhere
        m_pred_body = R_bn' * mag_ref_ned;
        H_m = zeros(3, 16);
        H_m(1:3, 1:3) = skew_(m_pred_body);

        z_mag = double(mag_body_zup(:));
        z_hat_mag = m_pred_body;

        for ax = 1:3
            innov = z_mag(ax) - z_hat_mag(ax);
            H_row = H_m(ax, :);
            S = H_row * st.P * H_row' + params.R_mag;
            if S <= 0, continue; end
            K = st.P * H_row' / S;
            st.x_err = st.x_err + K * innov;
            IKH = I16 - K * H_row;
            st.P = IKH * st.P * IKH' + params.R_mag * (K * K');
            st.P = 0.5 * (st.P + st.P');
        end

        [st.q_ref, st.v_ref, st.p_ref, st.bg_ref, st.ba_ref, st.bb_ref] = ...
            apply_correction_(st.q_ref, st.v_ref, st.p_ref, ...
                              st.bg_ref, st.ba_ref, st.bb_ref, st.x_err);
        st.x_err = zeros(16, 1);
    end

    % ------------------------------------------------------------------
    % Pack outputs.
    % ------------------------------------------------------------------
    pos_NED      = st.p_ref;
    vel_NED      = st.v_ref;
    att_quat     = st.q_ref;
    bg           = st.bg_ref;
    ba           = st.ba_ref;
    bb           = st.bb_ref;
    sigma_pos    = sqrt(abs(diag(st.P(7:9,7:9))));
    sigma_vel    = sqrt(abs(diag(st.P(4:6,4:6))));
    sigma_att    = sqrt(abs(diag(st.P(1:3,1:3)))) * 180/pi;
    baro_gate_on = st.baro_gate_on;
    init_armed   = st.armed;
end


% =========================================================================
function st = make_init_state_()
% Build the EKF16 initial state container. Mirrors EKF16Verify lines 318-336.
% q_ref seed is the visual-model body-Zup pad quat
%   [0; sqrt(2)/2; sqrt(2)/2; 0]
% (body+X=East, body+Y=North, body+Z=Up at pad with nose pointing up
% north). The caller's `q_init_fw` argument overrides this on the first
% armed tick, so this default only matters before the first attitude
% init_complete edge.
    st = struct();
    st.q_ref  = [0; sqrt(2)/2; sqrt(2)/2; 0];
    st.v_ref  = zeros(3, 1);       % NED
    st.p_ref  = zeros(3, 1);       % NED
    st.bg_ref = zeros(3, 1);       % gyro bias estimate
    st.ba_ref = zeros(3, 1);       % accel bias estimate
    st.bb_ref = 0;                 % baro bias estimate
    st.x_err  = zeros(16, 1);

    P = zeros(16);
    P(1,1) = 7.6e-03; P(2,2) = 7.6e-03; P(3,3) = 7.6e-03;
    P(4,4) = 1e-04;   P(5,5) = 1e-04;   P(6,6) = 1e-04;
    P(7,7) = 1e-02;   P(8,8) = 1e-02;   P(9,9) = 1e-02;
    P(10,10)=1e-06;   P(11,11)=1e-06;   P(12,12)=1e-06;
    P(13,13)=1e-02;   P(14,14)=1e-02;   P(15,15)=1e-02;
    P(16,16)=7.5e-01;
    st.P = P;

    st.baro_gate_on = false;
    st.armed        = false;
end


% =========================================================================
function p = build_default_params_()
% EKF16 noise + gate parameters. Mirrors EKF16Verify §4 + §7.
    p = struct();
    p.gyro_arw  = [6.11e-04, 4.73e-04, 4.59e-04];
    p.gyro_bi   = [7.94e-05, 4.02e-05, 3.45e-05];
    p.accel_vrw = 2.228e-03;
    p.accel_bi  = 2.146e-03;
    p.baro_bi   = 1.000e-03;
    p.R_baro    = 9.7e-05;
    p.R_mag     = 11.1;
    p.R_zupt    = 0.001;
    p.gravity   = 9.80665;
    p.dt_ekf    = 0.002;
    p.mach_gate_on   = 0.40;
    p.mach_gate_off  = 0.35;
    p.baro_gate_sigma = 5.0;
end


% =========================================================================
function R = quat2Tbn_(q)
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [w^2+x^2-y^2-z^2,  2*(x*y-w*z),      2*(x*z+w*y);
         2*(x*y+w*z),      w^2-x^2+y^2-z^2,  2*(y*z-w*x);
         2*(x*z-w*y),      2*(y*z+w*x),      w^2-x^2-y^2+z^2];
end


% =========================================================================
function r = quatmult_(a, b)
    r = [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4);
         a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
         a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2);
         a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
end


% =========================================================================
function [q,v,p,bg,ba,bb] = apply_correction_(q,v,p,bg,ba,bb, x)
% Attitude reset on injection (EKF16Verify lines 659-672).
    dth = x(1:3);
    q_err = [1; dth/2];
    q = quatmult_(q, q_err);
    nq = norm(q);
    if nq > 1e-12, q = q / nq; end

    v  = v  + x(4:6);
    p  = p  + x(7:9);
    bg = bg + x(10:12);
    ba = ba + x(13:15);
    bb = bb + x(16);
end


% =========================================================================
function s = isa_sos_(h_up_m)
% Standard-atmosphere speed of sound, altitude clipped at 0.
    T = max(216.65, 288.15 - 0.0065 * max(h_up_m, 0));
    s = sqrt(1.4 * 287.058 * T);
end


% =========================================================================
function S = skew_(v)
    S = [   0   -v(3)  v(2);
          v(3)    0   -v(1);
         -v(2)  v(1)    0   ];
end


% =========================================================================
function F = build_F_(R_bn, omega, f_body, dt)
% Hand-coded F matrix (16x16), byte-equivalent to EKF_Symbolic_Dev.m §4.
%
% Block structure (matches state vector indexing [att; vel; pos; bg; ba; bb]):
%   F(att,  att)  = I + (-skew(omega))*dt    (att_new = att + (-skew(w)*att - dbg + ng)*dt)
%   F(att,  bg)   = -I*dt
%   F(vel,  att)  = -R_bn * skew(f_body) * dt
%   F(vel,  ba)   = -R_bn * dt
%   F(pos,  vel)  =  I*dt
%   F(bg,   bg)   =  I
%   F(ba,   ba)   =  I
%   F(bb,   bb)   =  1
%   everything else 0.

    F = eye(16);
    I3 = eye(3);

    skew_w = [   0     -omega(3)  omega(2);
               omega(3)   0     -omega(1);
              -omega(2) omega(1)    0     ];

    skew_f = [   0     -f_body(3)  f_body(2);
               f_body(3)   0     -f_body(1);
              -f_body(2) f_body(1)    0     ];

    F(1:3,  1:3 )  = I3 - skew_w * dt;
    F(1:3,  10:12) = -I3 * dt;
    F(4:6,  1:3 )  = -R_bn * skew_f * dt;
    F(4:6,  13:15) = -R_bn * dt;
    F(7:9,  4:6 )  =  I3 * dt;
    % F(10:12, 10:12), F(13:15, 13:15), F(16,16) already = I from eye(16).
end


% =========================================================================
function Q = build_Q_(R_bn, dt, params)
% Hand-coded Q matrix (16x16), byte-equivalent to EKF_Symbolic_Dev.m §6.
%
%   Q = G_c * diag(sig_w.^2) * G_c' * dt   +   bias_random_walk * dt
% where:
%   G_c(1:3,  1:3) = I         (gyro white noise drives attitude error)
%   G_c(4:6,  4:6) = R_bn      (accel white noise drives velocity error in NED)
%   G_c is zero elsewhere (other states are bias random walks).

    Q = zeros(16);

    % Gyro white-noise contribution: Qatt = I * diag(sig_g.^2) * I' * dt
    Q_att = diag([params.gyro_arw(1)^2, ...
                  params.gyro_arw(2)^2, ...
                  params.gyro_arw(3)^2]) * dt;
    Q(1:3, 1:3) = Q_att;

    % Accel white-noise contribution: Qvel = R_bn * diag(sig_a.^2) * R_bn' * dt
    Q_vel = R_bn * diag([params.accel_vrw^2, ...
                         params.accel_vrw^2, ...
                         params.accel_vrw^2]) * R_bn' * dt;
    Q(4:6, 4:6) = Q_vel;

    % Bias random walks (diagonal)
    Q(10,10) = params.gyro_bi(1)^2  * dt;
    Q(11,11) = params.gyro_bi(2)^2  * dt;
    Q(12,12) = params.gyro_bi(3)^2  * dt;
    Q(13,13) = params.accel_bi^2    * dt;
    Q(14,14) = params.accel_bi^2    * dt;
    Q(15,15) = params.accel_bi^2    * dt;
    Q(16,16) = params.baro_bi^2     * dt;

    Q = 0.5 * (Q + Q');
end


