%% CASPER-2 EKF16 — Closed-Loop Verification Simulation
%
% Generates a synthetic 1-stage HPR flight using the AeroTech O5500X
% thrust curve, synthesises realistic IMU/baro/mag sensor data, then
% runs the full 16-state error-state EKF and compares against truth.
%
% Requires: casper_ekf16_symbolic.mat (from casper_ekf16_derive.m)
%
% Author: Ashwin Vishwanath / Claude
% Date:   2026-04-16

clear all; %#ok<CLALL>
close all;
clc;

fprintf('================================================================\n');
fprintf('  CASPER-2 EKF16 — CLOSED-LOOP VERIFICATION\n');
fprintf('  Motor: AeroTech O5500X-PS\n');
fprintf('================================================================\n\n');

%% Load symbolic workspace
fprintf('Loading symbolic workspace...\n');
load('casper_ekf16_symbolic.mat');
fprintf('  Done.\n\n');

%% =====================================================================
%  SECTION 1: AeroTech O5500X-PS Thrust Curve
%  =====================================================================

fprintf('§1  Building thrust curve...\n');

% RASP data from thrustcurve.org (time in s, thrust in N)
thrust_time = [0.020; 0.063; 0.116; 0.206; 0.349; 0.578; 0.921;
               1.378; 1.834; 2.290; 2.746; 2.870; 3.202; 3.659;
               3.999; 4.190; 4.380; 4.487; 4.517];
thrust_N    = [97.5; 5878.5; 6147.0; 6321.0; 6313.8; 6286.8; 6281.5;
               6265.5; 6213.6; 6137.5; 6164.9; 6079.0; 4268.1; 2843.9;
               1794.0; 1242.3; 513.8; 77.8; 0.0];

% Motor specs
motor.prop_mass   = 9.779;    % kg
motor.total_mass  = 15.681;   % kg (loaded)
motor.case_mass   = motor.total_mass - motor.prop_mass;
motor.burn_time   = 4.517;    % s
motor.total_impulse = trapz(thrust_time, thrust_N);

fprintf('  Total impulse: %.0f Ns\n', motor.total_impulse);
fprintf('  Burn time: %.2f s\n', motor.burn_time);
fprintf('  Avg thrust: %.0f N\n', motor.total_impulse / motor.burn_time);

%% =====================================================================
%  SECTION 2: Rocket Parameters
%  =====================================================================

fprintf('\n§2  Rocket parameters...\n');

rocket.dry_mass   = 12.0;     % kg (airframe, avionics, recovery, no motor)
rocket.Cd         = 0.45;     % drag coefficient
rocket.diameter   = 0.098;    % m (98mm airframe)
rocket.area       = pi/4 * rocket.diameter^2;

rocket.pad_mass = rocket.dry_mass + motor.total_mass;
fprintf('  Pad mass: %.1f kg\n', rocket.pad_mass);
fprintf('  Dry mass (burnout): %.1f kg\n', rocket.dry_mass + motor.case_mass);

%% =====================================================================
%  SECTION 3: Truth Trajectory Generation (1-DOF vertical)
%  =====================================================================

fprintf('\n§3  Generating truth trajectory...\n');

g = 9.80665;
dt_truth = 0.001;   % 1 kHz truth rate
T_pad    = 30.0;     % pad time before launch
T_flight = 120.0;    % max flight time after launch
T_total  = T_pad + T_flight;
N_truth  = round(T_total / dt_truth);

t_truth   = (0:N_truth-1)' * dt_truth;
alt_truth = zeros(N_truth, 1);
vel_truth = zeros(N_truth, 1);
acc_truth = zeros(N_truth, 1);   % true vertical acceleration (up +)
mass_truth = zeros(N_truth, 1);
phase_truth = zeros(N_truth, 1); % 0=pad, 1=boost, 2=coast, 3=descent

% ISA atmosphere
isa_rho = @(h) 1.225 * exp(-max(h,0) / 8500);
isa_temp = @(h) max(216.65, 288.15 - 0.0065 * max(h,0));
isa_sos = @(h) sqrt(1.4 * 287.058 * isa_temp(h));

% Attitude: rocket stays vertical (simplification for 1-DOF)
% Body +Z = up = NED -Z

vel = 0; alt = 0;
mass = rocket.pad_mass;
launched = false;

for k = 1:N_truth
    t = t_truth(k);
    t_flight_local = t - T_pad;
    
    if t < T_pad
        % Pad
        phase_truth(k) = 0;
        mass_truth(k) = rocket.pad_mass;
        acc_truth(k) = 0;
        vel = 0; alt = 0;
        
    elseif t_flight_local <= motor.burn_time
        % Boost
        phase_truth(k) = 1;
        
        % Thrust (interpolate)
        thrust = interp1(thrust_time, thrust_N, t_flight_local, 'linear', 0);
        
        % Mass (linear burn)
        burn_frac = t_flight_local / motor.burn_time;
        mass = rocket.pad_mass - motor.prop_mass * burn_frac;
        mass_truth(k) = mass;
        
        % Drag
        rho = isa_rho(alt);
        drag = 0.5 * rho * vel * abs(vel) * rocket.Cd * rocket.area;
        
        % Acceleration (up positive)
        acc = (thrust - drag) / mass - g;
        acc_truth(k) = acc;
        
        vel = vel + acc * dt_truth;
        alt = alt + vel * dt_truth;
        
    else
        % Coast / descent
        if vel > 0
            phase_truth(k) = 2;  % coast (ascending)
        else
            phase_truth(k) = 3;  % descent
        end
        
        mass = rocket.dry_mass + motor.case_mass;
        mass_truth(k) = mass;
        
        rho = isa_rho(alt);
        drag = 0.5 * rho * vel * abs(vel) * rocket.Cd * rocket.area;
        
        acc = -drag / mass - g;
        acc_truth(k) = acc;
        
        vel = vel + acc * dt_truth;
        alt = alt + vel * dt_truth;
        
        % Ground clamp
        if alt <= 0 && vel < 0
            alt = 0; vel = 0; acc = 0;
            acc_truth(k) = 0;
        end
    end
    
    vel_truth(k) = vel;
    alt_truth(k) = alt;
end

% Find key events
[apogee_alt, idx_apogee] = max(alt_truth);
t_apogee = t_truth(idx_apogee);
peak_vel = max(vel_truth);
peak_mach = peak_vel / isa_sos(alt_truth(find(vel_truth == peak_vel, 1)));
peak_acc_g = max(acc_truth) / g;

fprintf('  Apogee: %.0f m at t=%.1f s (%.1f s after launch)\n', ...
    apogee_alt, t_apogee, t_apogee - T_pad);
fprintf('  Peak velocity: %.0f m/s (Mach %.2f)\n', peak_vel, peak_mach);
fprintf('  Peak acceleration: %.0f m/s^2 (%.1f g)\n', max(acc_truth), peak_acc_g);
fprintf('  Burnout at t=%.1f s, vel=%.0f m/s, alt=%.0f m\n', ...
    T_pad + motor.burn_time, ...
    vel_truth(round((T_pad + motor.burn_time)/dt_truth)), ...
    alt_truth(round((T_pad + motor.burn_time)/dt_truth)));

%% =====================================================================
%  SECTION 4: Sensor Noise Parameters
%  =====================================================================

fprintf('\n§4  Sensor parameters...\n');

params.gyro_arw  = [6.11e-04, 4.73e-04, 4.59e-04];
params.gyro_bi   = [7.94e-05, 4.02e-05, 3.45e-05];
params.accel_vrw = 2.228e-03;
params.accel_bi  = 2.146e-03;
params.baro_bi   = 1.000e-03;
params.R_baro    = 9.7e-05;
params.R_mag     = 11.1;
params.R_zupt    = 0.001;
params.gravity   = g;
params.dt_ekf    = 0.002;    % 500 Hz

% True sensor biases (injected, unknown to filter)
true_bias.gyro  = [1.2e-03; -0.8e-03; 0.5e-03];  % rad/s
true_bias.accel = [0.05; -0.03; 0.02];            % m/s^2
true_bias.baro  = 0.3;                             % m

% Mag reference (London, NED, uT)
mag_ref_ned = [20.0; 0.5; 43.0];

fprintf('  Done.\n');

%% =====================================================================
%  SECTION 5: Synthesise Sensor Data
%  =====================================================================

fprintf('\n§5  Synthesising sensor data...\n');

% We generate data at 500 Hz (EKF rate) for simplicity.
% In firmware, the strapdown accumulates at 833 Hz — here we skip that
% layer and feed directly at the EKF rate.

dt_sim = params.dt_ekf;
N_sim = round(T_total / dt_sim);
t_sim = (0:N_sim-1)' * dt_sim;

% Interpolate truth to sim rate
alt_sim = interp1(t_truth, alt_truth, t_sim, 'linear', 0);
vel_sim = interp1(t_truth, vel_truth, t_sim, 'linear', 0);
acc_sim = interp1(t_truth, acc_truth, t_sim, 'linear', 0);
phase_sim = interp1(t_truth, phase_truth, t_sim, 'nearest', 0);

% Pad quaternion: body +Z = up = NED -Z. 180 deg about Y.
q_true = [0; 0; 1; 0];  % [w,x,y,z] — constant (vertical flight)
Tbn_true = quat2Tbn(q_true);

% --- Synthesise IMU (body frame) ---
% True specific force: f_true = a_true + g (accelerometer reads specific force)
% In NED: f_ned = [0; 0; -(acc_true + g)]  (acc_true is up-positive, NED-down is positive)
% Wait: acc_truth is the true acceleration (up positive).
% In NED frame: a_ned = [0; 0; -acc_truth] (up = -D)
% Specific force = a_true - g_ned = [0;0;-acc] - [0;0;g] = [0;0;-acc-g]
% But accelerometer measures specific force: f = a_body - g_body
%   On pad: a=0, f = -g_body = [0;0;+g] (body Z = up, gravity pulls -Z)
%   In boost: a=acc upward, f_ned = [0;0;-acc-g]... wait let me think carefully.
%
% Specific force (what accelerometer reads) = true_accel - gravity
% In NED: sf_ned = [0;0;-acc_truth] - [0;0;g] = [0;0;-acc_truth - g]
% In body (Tbn' * sf_ned): since body is aligned with NED but Z-flipped:
%   body Z = -NED Z, so sf_body_z = acc_truth + g
%
% On pad: acc=0, sf_body_z = g = 9.81 ✓
% In boost at 10g: acc=98, sf_body_z = 98+9.81 = 107.8 ✓ (accel reads thrust+gravity)

sf_body_z = acc_sim + g;  % specific force along body Z (nose)

% Full 3-axis body specific force (vertical flight = only Z)
imu_accel_true = zeros(N_sim, 3);
imu_accel_true(:, 3) = sf_body_z;

% Gyro: zero (no rotation in vertical flight)
imu_gyro_true = zeros(N_sim, 3);

% Add bias + noise
rng(42, 'twister');  % reproducible

imu_accel_meas = imu_accel_true + true_bias.accel' + ...
    params.accel_vrw / sqrt(dt_sim) * randn(N_sim, 3);

imu_gyro_meas = imu_gyro_true + true_bias.gyro' + ...
    params.gyro_arw / sqrt(dt_sim) .* randn(N_sim, 3);

% --- Synthesise baro ---
% Baro measures altitude AGL + bias + noise
% ~100 Hz = every 5th EKF step
baro_interval = 5;
baro_alt_meas = alt_sim + true_bias.baro + ...
    sqrt(params.R_baro) * randn(N_sim, 1);

% Mach for gating
mach_sim = abs(vel_sim) ./ arrayfun(isa_sos, alt_sim);

% --- Synthesise mag (body frame) ---
% mag_body = Tbn' * mag_ref_ned + noise
% At constant attitude: mag_body is constant
mag_body_true = Tbn_true' * mag_ref_ned;
mag_interval = 50;  % ~10 Hz
mag_meas = mag_body_true' + sqrt(params.R_mag) * randn(N_sim, 3);

fprintf('  Generated %d timesteps (%.1f s) at %.0f Hz\n', N_sim, T_total, 1/dt_sim);

%% =====================================================================
%  SECTION 6: Precompute EKF Matrices (from symbolic)
%  =====================================================================

fprintf('\n§6  Building EKF matrix function handles...\n');

% Convert symbolic F to a MATLAB function
syms q0 q1 q2 q3 wx wy wz fx fy fz dt 'real'
F_func = matlabFunction(F, 'Vars', {q0,q1,q2,q3, wx,wy,wz, fx,fy,fz, dt});

% Convert symbolic Q to a MATLAB function
syms sig_gx sig_gy sig_gz sig_ax sig_ay sig_az 'real'
syms sig_gbx sig_gby sig_gbz sig_abx sig_aby sig_abz sig_bb 'real'
Q_func = matlabFunction(Q, 'Vars', ...
    {q0,q1,q2,q3, dt, ...
     sig_gx,sig_gy,sig_gz, sig_ax,sig_ay,sig_az, ...
     sig_gbx,sig_gby,sig_gbz, sig_abx,sig_aby,sig_abz, sig_bb});

% H_MAG function
syms magN magE magD 'real'
H_MAG_func = matlabFunction(H_MAG, 'Vars', {q0,q1,q2,q3, magN,magE,magD});

fprintf('  F_func, Q_func, H_MAG_func compiled.\n');

%% =====================================================================
%  SECTION 7: Run EKF
%  =====================================================================

fprintf('\n================================================================\n');
fprintf('  RUNNING EKF\n');
fprintf('================================================================\n\n');

% --- EKF state ---
% Reference state
q_ref = q_true;          % initialise from "Mahony" (perfect here)
v_ref = [0; 0; 0];       % NED
p_ref = [0; 0; 0];       % NED
bg_ref = [0; 0; 0];      % gyro bias estimate (unknown at start)
ba_ref = [0; 0; 0];      % accel bias estimate
bb_ref = 0;              % baro bias estimate

% Error state (always near zero)
x_err = zeros(16, 1);

% Covariance
P = zeros(16);
P(1,1) = 7.6e-03; P(2,2) = 7.6e-03; P(3,3) = 7.6e-03;
P(4,4) = 1e-04;   P(5,5) = 1e-04;   P(6,6) = 1e-04;
P(7,7) = 1e-02;   P(8,8) = 1e-02;   P(9,9) = 1e-02;
P(10,10)=1e-06;   P(11,11)=1e-06;   P(12,12)=1e-06;
P(13,13)=1e-02;   P(14,14)=1e-02;   P(15,15)=1e-02;
P(16,16)=7.5e-01;

I16 = eye(16);

% History
ekf_alt = zeros(N_sim, 1);  % estimated altitude (up positive)
ekf_vel = zeros(N_sim, 1);  % estimated vertical velocity (up positive)
ekf_att = zeros(N_sim, 4);  % quaternion
ekf_sigma_att = zeros(N_sim, 3);
ekf_sigma_vel = zeros(N_sim, 3);
ekf_sigma_pos = zeros(N_sim, 3);
ekf_ba = zeros(N_sim, 3);
ekf_bg = zeros(N_sim, 3);
ekf_bb = zeros(N_sim, 1);
innov_baro = NaN(N_sim, 1);

baro_gated_sim = false(N_sim, 1);
baro_gate_on = false;

% EKF parameters
mach_gate_on  = 0.40;
mach_gate_off = 0.35;
baro_gate_sigma = 5.0;

fprintf('  Running %d steps...', N_sim);
tic;

for k = 1:N_sim
    t = t_sim(k);
    ph = phase_sim(k);
    
    % === IMU input (bias-corrected with current estimate) ===
    gyro_raw = imu_gyro_meas(k, :)';
    accel_raw = imu_accel_meas(k, :)';
    
    omega_corr = gyro_raw - bg_ref;
    f_corr = accel_raw - ba_ref;
    
    % === Reference state propagation ===
    % Quaternion update
    dq = [1; omega_corr * dt_sim / 2];
    q_ref = quatmult(q_ref, dq);
    q_ref = q_ref / norm(q_ref);
    
    % Rotation matrix
    R_bn = quat2Tbn(q_ref);
    
    % Velocity: v += (R*f + g_ned) * dt
    f_ned = R_bn * f_corr;
    v_ref = v_ref + (f_ned + [0; 0; g]) * dt_sim;
    
    % Position: p += v * dt
    p_ref = p_ref + v_ref * dt_sim;
    
    % === EKF predict ===
    F_k = F_func(q_ref(1),q_ref(2),q_ref(3),q_ref(4), ...
                 omega_corr(1),omega_corr(2),omega_corr(3), ...
                 f_corr(1),f_corr(2),f_corr(3), dt_sim);
    
    Q_k = Q_func(q_ref(1),q_ref(2),q_ref(3),q_ref(4), dt_sim, ...
                 params.gyro_arw(1),params.gyro_arw(2),params.gyro_arw(3), ...
                 params.accel_vrw, params.accel_vrw, params.accel_vrw, ...
                 params.gyro_bi(1),params.gyro_bi(2),params.gyro_bi(3), ...
                 params.accel_bi, params.accel_bi, params.accel_bi, ...
                 params.baro_bi);
    Q_k = 0.5 * (Q_k + Q_k');
    
    P = F_k * P * F_k' + Q_k;
    P = 0.5 * (P + P');
    
    % === ZUPT (pad only) ===
    if ph == 0
        for ax = 4:6
            innov = -v_ref(ax - 3);  % z=0, z_hat=v_ref
            S = P(ax,ax) + params.R_zupt;
            K = P(:,ax) / S;
            x_err = x_err + K * innov;
            IKH = I16; IKH(:,ax) = IKH(:,ax) - K;
            P = IKH * P * IKH' + params.R_zupt * (K * K');
            P = 0.5 * (P + P');
        end
        % Apply error state correction
        [q_ref, v_ref, p_ref, bg_ref, ba_ref, bb_ref] = ...
            apply_correction(q_ref, v_ref, p_ref, bg_ref, ba_ref, bb_ref, x_err);
        x_err = zeros(16, 1);
    end
    
    % === Baro update ===
    % Mach gating with hysteresis
    mach_est = abs(-v_ref(3)) / isa_sos(-p_ref(3));  % v_up = -v_D, alt = -p_D
    if ~baro_gate_on && mach_est > mach_gate_on
        baro_gate_on = true;
    elseif baro_gate_on && mach_est < mach_gate_off
        baro_gate_on = false;
    end
    baro_gated_sim(k) = baro_gate_on;
    
    if mod(k, baro_interval) == 0 && ~baro_gate_on
        % Baro measurement: z = -(baro_alt) in NED-down
        z_baro = baro_alt_meas(k);         % baro reads altitude AGL (up positive)
        z_hat = -p_ref(3) - bb_ref;        % predicted: alt_up = -pos_D, baro_bias adds in up frame
        innov = z_baro - z_hat;
        
        H_b = zeros(1, 16); H_b(9) = -1; H_b(16) = -1;
        S = H_b * P * H_b' + params.R_baro;
        
        % Innovation gate
        if innov^2 <= baro_gate_sigma^2 * S
            K = P * H_b' / S;
            x_err = x_err + K * innov;
            IKH = I16 - K * H_b;
            P = IKH * P * IKH' + params.R_baro * (K * K');
            P = 0.5 * (P + P');
            
            % Apply correction
            [q_ref, v_ref, p_ref, bg_ref, ba_ref, bb_ref] = ...
                apply_correction(q_ref, v_ref, p_ref, bg_ref, ba_ref, bb_ref, x_err);
            x_err = zeros(16, 1);
            
            innov_baro(k) = innov;
        end
        
        % Baro bias floor
        if P(16,16) < 0.01, P(16,16) = 0.01; end
    end
    
    % === Mag update (~10 Hz, gated during boost) ===
    if mod(k, mag_interval) == 0 && ph ~= 1
        H_m = H_MAG_func(q_ref(1),q_ref(2),q_ref(3),q_ref(4), ...
                          mag_ref_ned(1),mag_ref_ned(2),mag_ref_ned(3));
        
        z_mag = mag_meas(k, :)';
        z_hat_mag = quat2Tbn(q_ref)' * mag_ref_ned;
        
        for ax = 1:3
            innov = z_mag(ax) - z_hat_mag(ax);
            H_row = H_m(ax, :);
            S = H_row * P * H_row' + params.R_mag;
            K = P * H_row' / S;
            x_err = x_err + K * innov;
            IKH = I16 - K * H_row;
            P = IKH * P * IKH' + params.R_mag * (K * K');
            P = 0.5 * (P + P');
        end
        
        % Apply correction + attitude reset
        [q_ref, v_ref, p_ref, bg_ref, ba_ref, bb_ref] = ...
            apply_correction(q_ref, v_ref, p_ref, bg_ref, ba_ref, bb_ref, x_err);
        x_err = zeros(16, 1);
    end
    
    % === Record ===
    ekf_alt(k) = -p_ref(3);         % NED-down to up-positive
    ekf_vel(k) = -v_ref(3);         % NED-down to up-positive
    ekf_att(k, :) = q_ref';
    ekf_sigma_att(k,:) = sqrt(abs(diag(P(1:3,1:3))))' * 180/pi;
    ekf_sigma_vel(k,:) = sqrt(abs(diag(P(4:6,4:6))))';
    ekf_sigma_pos(k,:) = sqrt(abs(diag(P(7:9,7:9))))';
    ekf_ba(k,:) = ba_ref';
    ekf_bg(k,:) = bg_ref';
    ekf_bb(k) = bb_ref;
end

elapsed = toc;
fprintf(' done in %.1f s\n\n', elapsed);

%% =====================================================================
%  SECTION 8: Error Analysis
%  =====================================================================

fprintf('================================================================\n');
fprintf('  ERROR ANALYSIS\n');
fprintf('================================================================\n\n');

% Only analyse up to apogee + 5s (PRD convention)
idx_apo_sim = find(vel_sim > 0 & [vel_sim(2:end); -1] <= 0 & t_sim > T_pad, 1);
if isempty(idx_apo_sim), idx_apo_sim = N_sim; end
t_apo_sim = t_sim(idx_apo_sim);
k_cutoff = find(t_sim <= t_apo_sim + 5, 1, 'last');

% Errors
alt_err = ekf_alt - alt_sim;
vel_err = ekf_vel - vel_sim;

% Apogee comparison
[ekf_apogee, idx_ekf_apo] = max(ekf_alt(1:k_cutoff));
ekf_apo_time = t_sim(idx_ekf_apo);

fprintf('  Truth apogee: %.0f m at t=%.2f s\n', alt_sim(idx_apo_sim), t_apo_sim);
fprintf('  EKF apogee:   %.0f m at t=%.2f s\n', ekf_apogee, ekf_apo_time);
fprintf('  Alt error:    %.1f m (%.2f%%)\n', ...
    ekf_apogee - alt_sim(idx_apo_sim), ...
    100*(ekf_apogee - alt_sim(idx_apo_sim))/alt_sim(idx_apo_sim));
fprintf('  Time error:   %.3f s\n', ekf_apo_time - t_apo_sim);

% RMS errors (launch to cutoff)
k_launch = find(t_sim >= T_pad, 1);
range = k_launch:k_cutoff;
alt_rms = rms(alt_err(range));
vel_rms = rms(vel_err(range));

fprintf('\n  RMS altitude error (launch to apo+5s): %.2f m\n', alt_rms);
fprintf('  RMS velocity error (launch to apo+5s): %.2f m/s\n', vel_rms);

% Peak errors
fprintf('  Peak alt error: %.1f m at t=%.1f s\n', ...
    max(abs(alt_err(range))), t_sim(k_launch-1+find(abs(alt_err(range))==max(abs(alt_err(range))),1)));
fprintf('  Peak vel error: %.2f m/s at t=%.1f s\n', ...
    max(abs(vel_err(range))), t_sim(k_launch-1+find(abs(vel_err(range))==max(abs(vel_err(range))),1)));

% Bias estimation
fprintf('\n  Bias estimates at apogee:\n');
fprintf('    Gyro bias:  [%.4f, %.4f, %.4f] mrad/s  (true: [%.4f, %.4f, %.4f])\n', ...
    ekf_bg(idx_apo_sim,:)*1e3, true_bias.gyro'*1e3);
fprintf('    Accel bias: [%.4f, %.4f, %.4f] m/s^2   (true: [%.4f, %.4f, %.4f])\n', ...
    ekf_ba(idx_apo_sim,:), true_bias.accel');
fprintf('    Baro bias:  %.4f m                      (true: %.4f)\n', ...
    ekf_bb(idx_apo_sim), true_bias.baro);

% Sanity checks
fprintf('\n  [%s] No NaN in P\n', tf2str(~any(isnan(P(:)))));
fprintf('  [%s] P positive definite: min eig = %.2e\n', ...
    tf2str(min(eig(P)) > 0), min(eig(P)));
fprintf('  [%s] Apogee within 5%%: %.2f%%\n', ...
    tf2str(abs(ekf_apogee - alt_sim(idx_apo_sim))/alt_sim(idx_apo_sim) < 0.05), ...
    100*abs(ekf_apogee - alt_sim(idx_apo_sim))/alt_sim(idx_apo_sim));
fprintf('  [%s] Apogee timing within 0.5s: %.3f s\n', ...
    tf2str(abs(ekf_apo_time - t_apo_sim) < 0.5), abs(ekf_apo_time - t_apo_sim));

%% =====================================================================
%  SECTION 9: Plots
%  =====================================================================

fprintf('\n§9  Plotting...\n');

figure('Name', 'EKF16 Closed-Loop Verification', 'Position', [50 50 1500 1000]);

% Altitude
subplot(3,2,1);
plot(t_sim, alt_sim, 'k', t_sim, ekf_alt, 'b', 'LineWidth', 1.2);
xline(T_pad,'--r','Launch'); xline(T_pad+motor.burn_time,'--r','Burnout');
xline(t_apo_sim,'--g','Apogee');
ylabel('Altitude (m)'); title('Altitude');
legend('Truth','EKF','Location','best'); grid on;
xlim([T_pad-2, t_apo_sim+10]);

% Velocity
subplot(3,2,2);
plot(t_sim, vel_sim, 'k', t_sim, ekf_vel, 'b', 'LineWidth', 1.2);
xline(T_pad,'--r'); xline(T_pad+motor.burn_time,'--r'); xline(t_apo_sim,'--g');
ylabel('Velocity (m/s)'); title('Velocity');
legend('Truth','EKF','Location','best'); grid on;
xlim([T_pad-2, t_apo_sim+10]);

% Altitude error
subplot(3,2,3);
plot(t_sim, alt_err, 'b', 'LineWidth', 1);
hold on;
plot(t_sim, 3*ekf_sigma_pos(:,3), 'r--', t_sim, -3*ekf_sigma_pos(:,3), 'r--');
xline(T_pad,'--r'); xline(T_pad+motor.burn_time,'--r'); xline(t_apo_sim,'--g');
ylabel('Error (m)'); title('Altitude Error vs 3\sigma');
legend('Error','\pm3\sigma','Location','best'); grid on;
xlim([T_pad-2, t_apo_sim+10]);

% Velocity error
subplot(3,2,4);
plot(t_sim, vel_err, 'b', 'LineWidth', 1);
hold on;
plot(t_sim, 3*ekf_sigma_vel(:,3), 'r--', t_sim, -3*ekf_sigma_vel(:,3), 'r--');
xline(T_pad,'--r'); xline(T_pad+motor.burn_time,'--r'); xline(t_apo_sim,'--g');
ylabel('Error (m/s)'); title('Velocity Error vs 3\sigma');
legend('Error','\pm3\sigma','Location','best'); grid on;
xlim([T_pad-2, t_apo_sim+10]);

% Accel bias estimation
subplot(3,2,5);
plot(t_sim, ekf_ba(:,1), 'r', t_sim, ekf_ba(:,2), 'g', t_sim, ekf_ba(:,3), 'b', 'LineWidth', 1.2);
yline(true_bias.accel(1), 'r--'); yline(true_bias.accel(2), 'g--'); yline(true_bias.accel(3), 'b--');
xlabel('Time (s)'); ylabel('Bias (m/s^2)'); title('Accel Bias Estimation');
legend('Est X','Est Y','Est Z','True X','True Y','True Z','Location','best'); grid on;

% Baro bias + gate
subplot(3,2,6);
yyaxis left;
plot(t_sim, ekf_bb, 'b-', 'LineWidth', 1.2);
yline(true_bias.baro, 'b--');
ylabel('Baro bias (m)');
yyaxis right;
area(t_sim, baro_gated_sim, 'FaceColor',[1 .8 .8],'EdgeColor','none','FaceAlpha',0.5);
ylabel('Baro Gated'); ylim([-0.1 1.5]);
xlabel('Time (s)'); title('Baro Bias + Gate');
grid on;

sgtitle(sprintf('EKF16 Closed-Loop — O5500X | Apogee: %.0f m | Error: %.1f m (%.2f%%)', ...
    apogee_alt, ekf_apogee - alt_sim(idx_apo_sim), ...
    100*(ekf_apogee - alt_sim(idx_apo_sim))/alt_sim(idx_apo_sim)), ...
    'FontSize', 13, 'FontWeight', 'bold');

saveas(gcf, 'ekf16_closedloop.png');
fprintf('  Saved ekf16_closedloop.png\n');

fprintf('\n================================================================\n');
fprintf('  VERIFICATION COMPLETE\n');
fprintf('================================================================\n');

%% =====================================================================
%  Helper Functions
%  =====================================================================

function R = quat2Tbn(q)
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [w^2+x^2-y^2-z^2,  2*(x*y-w*z),      2*(x*z+w*y);
         2*(x*y+w*z),      w^2-x^2+y^2-z^2,  2*(y*z-w*x);
         2*(x*z-w*y),      2*(y*z+w*x),      w^2-x^2-y^2+z^2];
end

function r = quatmult(a, b)
    r = [a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4);
         a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
         a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2);
         a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1)];
end

function [q,v,p,bg,ba,bb] = apply_correction(q,v,p,bg,ba,bb, x)
    % Attitude reset
    dth = x(1:3);
    q_err = [1; dth/2];
    q = quatmult(q, q_err);
    q = q / norm(q);
    
    % State corrections
    v  = v  + x(4:6);
    p  = p  + x(7:9);
    bg = bg + x(10:12);
    ba = ba + x(13:15);
    bb = bb + x(16);
end

function s = tf2str(c)
    if c, s = 'PASS'; else, s = 'FAIL'; end
end