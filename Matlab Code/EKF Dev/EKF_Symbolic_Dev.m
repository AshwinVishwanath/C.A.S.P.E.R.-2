% >>>  EXPERIMENTAL — DEV BRANCH ONLY — NOT FLIGHT-CERTIFIED  <<<
% ============================================================
%  TIER:     HOST-SIDE
%  MODULE:   EKF16 Symbolic Derivation
%  SUMMARY:  MATLAB-only symbolic derivation of the candidate 16-state EKF.
% ============================================================
%% CASPER-2 16-State Error-State EKF — Symbolic Derivation & Verification
%
% Derives F, G, Q, and all measurement H matrices symbolically,
% then runs numerical verification to confirm correctness.
%
% NO code generation.
%
% Requires: Symbolic Math Toolbox (MATLAB 2025b)
%
% State vector (16 error states):
%   x[0:2]   = attitude error (rotation vector, body frame) [rad]
%   x[3:5]   = velocity error (NED) [m/s]
%   x[6:8]   = position error (NED) [m]
%   x[9:11]  = gyro bias error (body frame) [rad/s]
%   x[12:14] = accel bias error (body frame) [m/s^2]
%   x[15]    = baro bias error [m]
%
% Author: Ashwin Vishwanath / Claude
% Date:   2026-04-15

clear all; %#ok<CLALL>
close all;
clc;

fprintf('================================================================\n');
fprintf('  CASPER-2 16-STATE ERROR-STATE EKF — SYMBOLIC DERIVATION\n');
fprintf('================================================================\n\n');

%% =====================================================================
%  SECTION 1: Define Symbolic Variables
%  =====================================================================

fprintf('§1  Defining symbolic variables...\n');

syms q0 q1 q2 q3 'real'
syms wx wy wz 'real'
syms fx fy fz 'real'
syms ngx ngy ngz 'real'
syms nax nay naz 'real'
syms dthx dthy dthz 'real'
syms dvn dve dvd 'real'
syms dpn dpe dpd 'real'
syms dbgx dbgy dbgz 'real'
syms dbax dbay dbaz 'real'
syms dbb 'real'
syms dt 'real'
syms R_BARO R_MAG R_ZUPT 'real'
syms magN magE magD 'real'
syms sig_gx sig_gy sig_gz 'real'
syms sig_ax sig_ay sig_az 'real'
syms sig_gbx sig_gby sig_gbz 'real'
syms sig_abx sig_aby sig_abz 'real'
syms sig_bb 'real'

fprintf('  Done.\n\n');

%% =====================================================================
%  SECTION 2: Rotation Matrix
%  =====================================================================

fprintf('§2  Building rotation matrix...\n');

Tbn = [q0^2+q1^2-q2^2-q3^2,  2*(q1*q2-q0*q3),      2*(q1*q3+q0*q2);
       2*(q1*q2+q0*q3),      q0^2-q1^2+q2^2-q3^2,  2*(q2*q3-q0*q1);
       2*(q1*q3-q0*q2),      2*(q2*q3+q0*q1),      q0^2-q1^2-q2^2+q3^2];

fprintf('  Done.\n\n');

%% =====================================================================
%  SECTION 3: Error-State Prediction Equations
%  =====================================================================

fprintf('§3  Deriving error-state prediction equations...\n');

skew = @(v) [  0   -v(3)  v(2);
              v(3)   0   -v(1);
             -v(2)  v(1)   0  ];

omega  = [wx; wy; wz];
f_body = [fx; fy; fz];

dth = [dthx; dthy; dthz];
dv  = [dvn;  dve;  dvd];
dp  = [dpn;  dpe;  dpd];
dbg = [dbgx; dbgy; dbgz];
dba = [dbax; dbay; dbaz];
ng  = [ngx;  ngy;  ngz];
na  = [nax;  nay;  naz];

dth_new = dth + (-skew(omega)*dth - dbg + ng) * dt;
dv_new  = dv + (-Tbn * skew(f_body) * dth - Tbn * dba + Tbn * na) * dt;
dp_new  = dp + dv * dt;
dbg_new = dbg;
dba_new = dba;
dbb_new = dbb;

stateVector    = [dth; dv; dp; dbg; dba; dbb];
newStateVector = [dth_new; dv_new; dp_new; dbg_new; dba_new; dbb_new];
nStates = numel(stateVector);

fprintf('  %d states. Done.\n\n', nStates);

%% =====================================================================
%  SECTION 4: State Transition Matrix F
%  =====================================================================

fprintf('§4  Computing F...\n');

F = jacobian(newStateVector, stateVector);

zeroVars = {dthx, dthy, dthz, dvn, dve, dvd, dpn, dpe, dpd, ...
            dbgx, dbgy, dbgz, dbax, dbay, dbaz, dbb};
F = subs(F, zeroVars, num2cell(zeros(1,16)));

fprintf('  F: %dx%d\n', size(F));

% Structural checks
assert(isequal(F(7:9, 4:6), eye(3)*dt),  'F(pos,vel) wrong');
assert(isequal(F(1:3, 10:12), -eye(3)*dt), 'F(att,gbias) wrong');
assert(isequal(F(10:12,10:12), sym(eye(3))),  'F(gbias) wrong');
assert(isequal(F(13:15,13:15), sym(eye(3))),  'F(abias) wrong');
assert(isequal(F(16,16), sym(1)),              'F(bbias) wrong');
assert(isequal(F(7:9, 1:3), sym(zeros(3))), 'F(pos,att) != 0');
assert(isequal(F(10:16, 1:9), sym(zeros(7,9))), 'F(bias,state) wrong');
fprintf('  [PASS] All structural checks.\n\n');

%% =====================================================================
%  SECTION 5: Continuous-Time Noise Influence G_c
%  =====================================================================

fprintf('§5  Building G_c...\n');

G_c = sym(zeros(nStates, 6));
G_c(1:3, 1:3) = eye(3);
G_c(4:6, 4:6) = Tbn;

fprintf('  Done.\n\n');

%% =====================================================================
%  SECTION 6: Process Noise Q
%  =====================================================================

fprintf('§6  Computing Q...\n');

Qc_imu = diag([sig_gx^2, sig_gy^2, sig_gz^2, ...
               sig_ax^2, sig_ay^2, sig_az^2]);
Q_imu = G_c * Qc_imu * G_c' * dt;

Q_bias = sym(zeros(nStates));
Q_bias(10,10) = sig_gbx^2*dt; Q_bias(11,11) = sig_gby^2*dt; Q_bias(12,12) = sig_gbz^2*dt;
Q_bias(13,13) = sig_abx^2*dt; Q_bias(14,14) = sig_aby^2*dt; Q_bias(15,15) = sig_abz^2*dt;
Q_bias(16,16) = sig_bb^2*dt;

Q = simplify(Q_imu + Q_bias);

assert(isequal(simplify(Q(1,1)), sig_gx^2*dt), 'Q(att) wrong');
assert(isequal(Q(7:9,7:9), sym(zeros(3))),     'Q(pos) wrong');
fprintf('  [PASS] Q structure verified.\n\n');

%% =====================================================================
%  SECTION 7: Measurement Models
%  =====================================================================

fprintf('§7  Measurement models...\n');

% --- Baro ---
H_BARO = sym(zeros(1, nStates));
H_BARO(1,9) = 1; H_BARO(1,16) = 1;
fprintf('  H_BARO: pos_D + baro_bias\n');

% --- Mag (Priseborough) ---
m_ref = [magN; magE; magD];
m_pred_body = Tbn' * m_ref;
H_MAG = sym(zeros(3, nStates));
H_MAG(1:3, 1:3) = skew(m_pred_body);
fprintf('  H_MAG: skew(Tbn''*m_ref) on attitude block\n');

% --- ZUPT ---
H_ZUPT = sym(zeros(3, nStates));
H_ZUPT(1,4) = 1; H_ZUPT(2,5) = 1; H_ZUPT(3,6) = 1;
fprintf('  H_ZUPT: identity on velocity block\n');

% --- GPS stubs ---
H_GPS_POS = sym(zeros(3, nStates));
H_GPS_POS(1,7) = 1; H_GPS_POS(2,8) = 1; H_GPS_POS(3,9) = 1;
H_GPS_VEL = sym(zeros(3, nStates));
H_GPS_VEL(1,4) = 1; H_GPS_VEL(2,5) = 1; H_GPS_VEL(3,6) = 1;
fprintf('  H_GPS: stubs defined.\n\n');

%% =====================================================================
%  SECTION 8: Save Symbolic Workspace
%  =====================================================================

fprintf('§8  Saving symbolic workspace...\n');
save('casper_ekf16_symbolic.mat', ...
    'F', 'G_c', 'Q', 'Tbn', ...
    'H_BARO', 'H_MAG', 'H_ZUPT', 'H_GPS_POS', 'H_GPS_VEL', ...
    'stateVector', 'nStates', 'omega', 'f_body', 'm_ref', 'm_pred_body');
fprintf('  Saved casper_ekf16_symbolic.mat\n\n');

%% =====================================================================
%  SECTION 9: Print Symbolic F Blocks
%  =====================================================================

fprintf('§9  Symbolic F (non-zero blocks):\n\n');
fprintf('  F(att, att):\n');   disp(F(1:3, 1:3));
fprintf('  F(att, gbias):\n'); disp(F(1:3, 10:12));
fprintf('  F(vel, att):\n');   disp(F(4:6, 1:3));
fprintf('  F(vel, abias):\n'); disp(F(4:6, 13:15));
fprintf('  F(pos, vel):\n');   disp(F(7:9, 4:6));

%% =====================================================================
%  SECTION 10: Numerical Setup
%  =====================================================================

fprintf('================================================================\n');
fprintf('  NUMERICAL VERIFICATION\n');
fprintf('================================================================\n\n');

params.gyro_arw  = [6.11e-04, 4.73e-04, 4.59e-04];
params.gyro_bi   = [7.94e-05, 4.02e-05, 3.45e-05];
params.accel_vrw = 2.228e-03;
params.accel_bi  = 2.146e-03;
params.baro_bi   = 1.000e-03;
params.R_baro    = 9.7e-05;
params.R_mag     = 11.1;
params.R_zupt    = 0.001;
params.gravity   = 9.80665;
params.dt        = 0.002;

% Pad quaternion: nose up, heading north

% Nose up: body +Z → NED -Z (up). This is 180° about body Y.
% q = [cos(90°), 0, sin(90°), 0] = [0, 0, 1, 0]
q_pad = [0; 0; 1; 0];
q0n = q_pad(1); q1n = q_pad(2); q2n = q_pad(3); q3n = q_pad(4);

Tbn_num = double(subs(Tbn, {q0,q1,q2,q3}, {q0n,q1n,q2n,q3n}));
body_z_ned = Tbn_num * [0;0;1];
disp(body_z_ned)
assert(norm(body_z_ned - [0;0;-1]) < 1e-10, 'Pad quaternion wrong');
fprintf('  [PASS] Pad quaternion: body +Z → NED [%.1f, %.1f, %.1f]\n', body_z_ned);

% Common substitution targets for Q
Q_sub_vars = {q0, q1, q2, q3, dt, ...
              sig_gx, sig_gy, sig_gz, sig_ax, sig_ay, sig_az, ...
              sig_gbx, sig_gby, sig_gbz, sig_abx, sig_aby, sig_abz, sig_bb};
Q_sub_vals = {q0n, q1n, q2n, q3n, params.dt, ...
              params.gyro_arw(1), params.gyro_arw(2), params.gyro_arw(3), ...
              params.accel_vrw, params.accel_vrw, params.accel_vrw, ...
              params.gyro_bi(1), params.gyro_bi(2), params.gyro_bi(3), ...
              params.accel_bi, params.accel_bi, params.accel_bi, params.baro_bi};

% Helper to build numerical F for given omega and f
sub_vars_F = {q0, q1, q2, q3, wx, wy, wz, fx, fy, fz, dt};
make_F = @(w, f) double(subs(F, sub_vars_F, ...
    {q0n, q1n, q2n, q3n, w(1), w(2), w(3), f(1), f(2), f(3), params.dt}));

% --- Precompute F and Q for each flight phase ---
fprintf('\n  Precomputing per-phase F and Q...\n');

% Phase 1: Pad (stationary)
F_pad = make_F([0;0;0], [0; 0; params.gravity]);

% Phase 2: Boost (10g along body Z, no rotation)
F_boost = make_F([0;0;0], [0; 0; params.gravity + 100]);

% Phase 3: Coast (near-zero specific force, slight roll)
F_coast = make_F([0.01;0;0], [0; 0; 0.1]);

% Q is the same for all phases (same quaternion, same noise params)
Q_num = double(subs(Q, Q_sub_vars, Q_sub_vals));
Q_num = 0.5 * (Q_num + Q_num');

eig_Q = min(eig(Q_num));
assert(eig_Q >= -1e-20, 'Q has negative eigenvalues');
fprintf('  [PASS] Q pos-semidef (min eig = %.2e)\n', eig_Q);

% Numerical H_MAG (London field)
m_ref_ned_num = [20.0; 0.5; 43.0];
H_mag_num = double(subs(H_MAG, ...
    {q0, q1, q2, q3, magN, magE, magD}, ...
    {q0n, q1n, q2n, q3n, m_ref_ned_num(1), m_ref_ned_num(2), m_ref_ned_num(3)}));

% Numerical H_BARO
H_baro_num = zeros(1,16); H_baro_num(9) = 1; H_baro_num(16) = 1;

% P0 (PRD §7)
P0 = zeros(16);
P0(1,1) = 7.6e-03; P0(2,2) = 7.6e-03; P0(3,3) = 7.6e-03;
P0(4,4) = 1e-04;   P0(5,5) = 1e-04;   P0(6,6) = 1e-04;
P0(7,7) = 1e-02;   P0(8,8) = 1e-02;   P0(9,9) = 1e-02;
P0(10,10)=1e-06;   P0(11,11)=1e-06;   P0(12,12)=1e-06;
P0(13,13)=1e-02;   P0(14,14)=1e-02;   P0(15,15)=1e-02;
P0(16,16)=7.5e-01;

fprintf('  Precomputation done.\n\n');

%% =====================================================================
%  SECTION 11: Open-Loop Propagation (60s, No Measurements)
%  =====================================================================

fprintf('§11 Open-loop propagation (60s, no measurements)...\n');

N_ol = round(60 / params.dt);
P_ol = P0;
sigma_ol = zeros(N_ol, 16);

for k = 1:N_ol
    P_ol = F_pad * P_ol * F_pad' + Q_num;
    P_ol = 0.5 * (P_ol + P_ol');
    sigma_ol(k,:) = sqrt(abs(diag(P_ol)))';
end

fprintf('  [%s] Symmetric: %.2e\n', tf2str(max(abs(P_ol-P_ol'),[],'all') < 1e-10), ...
    max(abs(P_ol-P_ol'),[],'all'));
fprintf('  [%s] Pos-def: min eig = %.2e\n', tf2str(min(eig(P_ol)) > 0), min(eig(P_ol)));
fprintf('  att_Z: %.3f → %.3f deg\n', sqrt(P0(3,3))*180/pi, sigma_ol(end,3)*180/pi);
fprintf('  vel_D: %.4f → %.3f m/s\n\n', sqrt(P0(6,6)), sigma_ol(end,6));

%% =====================================================================
%  SECTION 12: Measurement Update Tests
%  =====================================================================

fprintf('§12 Measurement update tests (on 60s propagated P)...\n\n');

% Baro
P_b = joseph_update(P_ol, H_baro_num, params.R_baro);
fprintf('  BARO:   pos_D %.3f→%.4f m | bb %.3f→%.3f m | [%s]\n', ...
    sqrt(P_ol(9,9)), sqrt(P_b(9,9)), sqrt(P_ol(16,16)), sqrt(P_b(16,16)), ...
    tf2str(min(eig(P_b))>0));

% Mag
P_m = P_ol;
for ax = 1:3, P_m = joseph_update(P_m, H_mag_num(ax,:), params.R_mag); end
fprintf('  MAG:    att_X %.3f→%.3f deg | att_Z %.3f→%.3f deg | [%s]\n', ...
    sqrt(P_ol(1,1))*180/pi, sqrt(P_m(1,1))*180/pi, ...
    sqrt(P_ol(3,3))*180/pi, sqrt(P_m(3,3))*180/pi, ...
    tf2str(min(eig(P_m))>0));

% ZUPT
P_z = P_ol;
for ax = 1:3
    H_row = zeros(1,16); H_row(3+ax) = 1;
    P_z = joseph_update(P_z, H_row, params.R_zupt);
end
fprintf('  ZUPT:   vel_N %.4f→%.4f m/s | ba_X %.4f→%.4f m/s^2 | [%s]\n\n', ...
    sqrt(P_ol(4,4)), sqrt(P_z(4,4)), ...
    sqrt(P_ol(13,13)), sqrt(P_z(13,13)), ...
    tf2str(min(eig(P_z))>0));

%% =====================================================================
%  SECTION 13: Full Scenario — Pad + Boost + Coast
%  =====================================================================

fprintf('================================================================\n');
fprintf('  FULL SCENARIO: PAD (30s) + BOOST (5s) + COAST (60s)\n');
fprintf('================================================================\n\n');

T_total = 95;
N_total = round(T_total / params.dt);
dt_s = params.dt;

% Precompute phase boundaries (index)
k_launch  = round(30 / dt_s);
k_burnout = round(35 / dt_s);

% Precompute truth trajectory
vel_hist = zeros(N_total, 1);
alt_hist = zeros(N_total, 1);
phase_id = ones(N_total, 1);   % 1=pad, 2=boost, 3=coast
vel_t = 0; alt_t = 0;

for k = 1:N_total
    if k <= k_launch
        vel_t = 0; alt_t = 0;
        phase_id(k) = 1;
    elseif k <= k_burnout
        vel_t = vel_t + 100 * dt_s;
        alt_t = alt_t + vel_t * dt_s;
        phase_id(k) = 2;
    else
        vel_t = vel_t - params.gravity * dt_s;
        alt_t = alt_t + vel_t * dt_s;
        phase_id(k) = 3;
    end
    vel_hist(k) = vel_t;
    alt_hist(k) = alt_t;
end

% Precompute baro gate mask
baro_gated = abs(vel_hist) / 340 > 0.4;

% Precompute measurement schedule masks
baro_mask = false(N_total, 1);
baro_interval = round(0.01 / dt_s);
baro_mask(baro_interval:baro_interval:N_total) = true;
baro_mask = baro_mask & ~baro_gated;

mag_mask = false(N_total, 1);
mag_interval = round(0.10 / dt_s);
mag_mask(mag_interval:mag_interval:N_total) = true;
mag_mask = mag_mask & (phase_id ~= 2);  % gated during boost

zupt_mask = (phase_id == 1);

% Select F per phase (precomputed — no symbolic subs in loop)
F_per_phase = {F_pad, F_boost, F_coast};

% --- Main loop (pure numeric, no symbolic ops) ---
fprintf('  Running %d steps...', N_total);
tic;

P_sim = P0;
sigma_sim = zeros(N_total, 16);

I16 = eye(16);

for k = 1:N_total
    F_k = F_per_phase{phase_id(k)};
    
    % Predict
    P_sim = F_k * P_sim * F_k' + Q_num;
    P_sim = 0.5 * (P_sim + P_sim');
    
    % ZUPT (pad only, 3 sequential scalars)
    if zupt_mask(k)
        for ax = 4:6
            S = P_sim(ax,ax) + params.R_zupt;
            K = P_sim(:,ax) / S;
            IKH = I16;
            IKH(:,ax) = IKH(:,ax) - K;
            P_sim = IKH * P_sim * IKH' + (params.R_zupt * (K * K'));
            P_sim = 0.5 * (P_sim + P_sim');
        end
    end
    
    % Baro
    if baro_mask(k)
        % H = [0..0, 1(pos_D), 0..0, 1(bb)]
        PH = P_sim(:,9) + P_sim(:,16);
        S = PH(9) + PH(16) + params.R_baro;
        K = PH / S;
        IKH = I16 - K * H_baro_num;
        P_sim = IKH * P_sim * IKH' + (params.R_baro * (K * K'));
        P_sim = 0.5 * (P_sim + P_sim');
        if P_sim(16,16) < 0.01, P_sim(16,16) = 0.01; end
    end
    
    % Mag (3 sequential scalars, gated during boost)
    if mag_mask(k)
        for ax = 1:3
            H_row = H_mag_num(ax,:);
            PH = P_sim * H_row';
            S = H_row * PH + params.R_mag;
            K = PH / S;
            IKH = I16 - K * H_row;
            P_sim = IKH * P_sim * IKH' + (params.R_mag * (K * K'));
            P_sim = 0.5 * (P_sim + P_sim');
        end
    end
    
    sigma_sim(k,:) = sqrt(abs(diag(P_sim)))';
end

elapsed = toc;
fprintf(' done in %.2f s\n\n', elapsed);

% Final checks
eig_final = min(eig(P_sim));
fprintf('  [%s] Final P pos-def: min eig = %.2e\n\n', tf2str(eig_final>0), eig_final);

% Key results
t_sim = (1:N_total)' * dt_s;

fprintf('  End of pad (t=30s):\n');
fprintf('    Attitude: [%.3f, %.3f, %.3f] deg\n', sigma_sim(k_launch,1:3)*180/pi);
fprintf('    Velocity: [%.5f, %.5f, %.5f] m/s\n', sigma_sim(k_launch,4:6));
fprintf('    Accel bias: [%.4f, %.4f, %.4f] m/s^2\n', sigma_sim(k_launch,13:15));

fprintf('  End of boost (t=35s):\n');
fprintf('    Attitude: [%.3f, %.3f, %.3f] deg\n', sigma_sim(k_burnout,1:3)*180/pi);
fprintf('    Velocity: [%.3f, %.3f, %.3f] m/s\n', sigma_sim(k_burnout,4:6));

fprintf('  End of coast (t=95s):\n');
fprintf('    Attitude: [%.3f, %.3f, %.3f] deg\n', sigma_sim(end,1:3)*180/pi);
fprintf('    Velocity: [%.3f, %.3f, %.3f] m/s\n', sigma_sim(end,4:6));
fprintf('    Position: [%.3f, %.3f, %.3f] m\n', sigma_sim(end,7:9));

gate_on = find(baro_gated, 1);
if ~isempty(gate_on)
    gate_off = find(baro_gated & (1:N_total)' > k_burnout, 1, 'last');
    fprintf('\n  Baro gated: t=%.1fs → t=%.1fs (Mach > 0.4)\n', ...
        gate_on*dt_s, gate_off*dt_s);
end

%% =====================================================================
%  SECTION 14: Plots
%  =====================================================================

fprintf('\n§14 Plotting...\n');

figure('Name', 'CASPER-2 EKF16 Verification', 'Position', [100 50 1400 950]);

subplot(3,2,1);
plot(t_sim, sigma_sim(:,1)*180/pi, 'r', t_sim, sigma_sim(:,2)*180/pi, 'g', ...
     t_sim, sigma_sim(:,3)*180/pi, 'b', 'LineWidth', 1.2);
ylabel('\sigma (deg)'); title('Attitude Uncertainty');
legend('X','Y','Z','Location','best'); grid on;
xline(30,'--k','Launch'); xline(35,'--k','Burnout');

subplot(3,2,2);
plot(t_sim, sigma_sim(:,4), 'r', t_sim, sigma_sim(:,5), 'g', ...
     t_sim, sigma_sim(:,6), 'b', 'LineWidth', 1.2);
ylabel('\sigma (m/s)'); title('Velocity Uncertainty');
legend('N','E','D','Location','best'); grid on;
xline(30,'--k'); xline(35,'--k');

subplot(3,2,3);
plot(t_sim, sigma_sim(:,7), 'r', t_sim, sigma_sim(:,8), 'g', ...
     t_sim, sigma_sim(:,9), 'b', 'LineWidth', 1.2);
ylabel('\sigma (m)'); title('Position Uncertainty');
legend('N','E','D','Location','best'); grid on;
xline(30,'--k'); xline(35,'--k');

subplot(3,2,4);
plot(t_sim, sigma_sim(:,10)*180/pi, 'r', t_sim, sigma_sim(:,11)*180/pi, 'g', ...
     t_sim, sigma_sim(:,12)*180/pi, 'b', 'LineWidth', 1.2);
ylabel('\sigma (deg/s)'); title('Gyro Bias Uncertainty');
legend('X','Y','Z','Location','best'); grid on;
xline(30,'--k'); xline(35,'--k');

subplot(3,2,5);
plot(t_sim, sigma_sim(:,13), 'r', t_sim, sigma_sim(:,14), 'g', ...
     t_sim, sigma_sim(:,15), 'b', 'LineWidth', 1.2);
ylabel('\sigma (m/s^2)'); xlabel('Time (s)'); title('Accel Bias Uncertainty');
legend('X','Y','Z','Location','best'); grid on;
xline(30,'--k'); xline(35,'--k');

subplot(3,2,6);
yyaxis left;
plot(t_sim, sigma_sim(:,16), 'b-', 'LineWidth', 1.2);
ylabel('\sigma_{bb} (m)');
yyaxis right;
area(t_sim, baro_gated, 'FaceColor',[1 .8 .8],'EdgeColor','none','FaceAlpha',0.5);
ylabel('Baro Gated'); ylim([-0.1 1.5]);
xlabel('Time (s)'); title('Baro Bias + Gate');
grid on;

sgtitle('CASPER-2 EKF16 — Pad → Boost → Coast', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, 'ekf16_verification.png');

figure('Name', 'Truth', 'Position', [200 200 800 400]);
subplot(1,2,1); plot(t_sim, alt_hist, 'k', 'LineWidth', 1.2);
ylabel('Alt (m)'); xlabel('Time (s)'); title('Altitude'); grid on;
subplot(1,2,2); plot(t_sim, vel_hist, 'k', 'LineWidth', 1.2);
ylabel('Vel (m/s)'); xlabel('Time (s)'); title('Velocity'); grid on;
sgtitle('Truth Trajectory');
saveas(gcf, 'ekf16_truth.png');

fprintf('  Saved plots.\n');

%% =====================================================================

fprintf('\n================================================================\n');
fprintf('  COMPLETE. Workspace: casper_ekf16_symbolic.mat\n');
fprintf('================================================================\n');

%% Local Functions

function s = tf2str(c)
    if c, s = 'PASS'; else, s = 'FAIL'; end
end

function P_new = joseph_update(P, H_row, R)
    S = H_row * P * H_row' + R;
    K = (P * H_row') / S;
    IKH = eye(size(P)) - K * H_row;
    P_new = IKH * P * IKH' + K * R * K';
    P_new = 0.5 * (P_new + P_new');
end