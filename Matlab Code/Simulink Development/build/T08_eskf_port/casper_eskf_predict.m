function state = casper_eskf_predict(state, accel_nav_up_mps2, Estimator)
%CASPER_ESKF_PREDICT Stripped EKF predict step (4-state vertical).
%
% Synopsis:
%   state = casper_eskf_predict(state, accel_nav_up_mps2, Estimator)
%
% Inputs:
%   state              : casper_eskf_state struct
%   accel_nav_up_mps2  : (scalar) Z-component of body accel rotated into
%                        nav (Z-up) frame, m/s^2. On the pad this is +9.80665.
%   Estimator          : sensor params struct (T02)
%
% Outputs:
%   state              : updated container (x_vec, P_mat propagated)
%
% Algorithm (matches Software/App/nav/casper_ekf.c casper_ekf_predict()):
%
%   a_up = accel_nav_up_mps2 - G - x(3)
%   x(1) = x(1) + x(2)*dt + 0.5*a_up*dt^2
%   x(2) = x(2) + a_up*dt
%   x(3), x(4) unchanged (random walk states; deterministic in predict)
%
%   Phi = [ 1  dt  -dt^2/2  0
%           0   1  -dt       0
%           0   0   1        0
%           0   0   0        1 ]
%
%   Q = [ qa*dt^3/3   qa*dt^2/2   0       0
%         qa*dt^2/2   qa*dt       0       0
%         0           0           qab*dt  0
%         0           0           0       qbb*dt ]
%
%   P = Phi*P*Phi' + Q
%   P = 0.5*(P + P')   (symmetrize)
%
%   Mach gate hysteresis: thresholds applied to |x(2)|/a_sound where
%   a_sound = sqrt(1.4*287.058*T_K) and T_K = max(216.65, 288.15-0.0065*alt).
%
%   P-floor on P(4,4) >= P_FLOOR_BARO_BIAS applied after every step.
%
% Source firmware reference:
%   Software/App/nav/casper_ekf.c casper_ekf_predict() lines 241-288

    if ~state.initialized
        error('casper_eskf_predict:NotInitialized', ...
            'State must be initialized before predict. Call casper_eskf_state(''init'',...) first.');
    end

    dt = Estimator.Dt;
    G  = Estimator.G;

    % --- State propagation ---
    a_up_mps2 = accel_nav_up_mps2 - G - state.x_vec(3);

    x_new = state.x_vec;
    x_new(1) = state.x_vec(1) + state.x_vec(2) * dt + 0.5 * a_up_mps2 * dt * dt;
    x_new(2) = state.x_vec(2) + a_up_mps2 * dt;
    % x_new(3) and x_new(4) are unchanged (random walks)

    state.x_vec = x_new;

    % --- Covariance propagation ---
    dt2h = 0.5 * dt * dt;
    Phi = [ 1,  dt, -dt2h, 0;
            0,   1, -dt,   0;
            0,   0,   1,   0;
            0,   0,   0,   1 ];

    qa  = Estimator.AccelVRW * Estimator.AccelVRW;
    qab = Estimator.AccelBiSigma * Estimator.AccelBiSigma;
    qbb = Estimator.BaroBiSigma * Estimator.BaroBiSigma;

    Q = [ qa*dt^3/3,  qa*dt^2/2, 0,       0;
          qa*dt^2/2,  qa*dt,     0,       0;
          0,          0,         qab*dt,  0;
          0,          0,         0,       qbb*dt ];

    P_new = Phi * state.P_mat * Phi' + Q;
    P_new = 0.5 * (P_new + P_new');   % symmetrize

    % --- P-floor on baro bias ---
    if P_new(4, 4) < Estimator.PFloorBaroBias
        P_new(4, 4) = Estimator.PFloorBaroBias;
    end

    state.P_mat = P_new;

    % NaN/Inf safety
    if any(~isfinite(state.x_vec)) || any(~isfinite(state.P_mat(:)))
        error('casper_eskf_predict:NonFinite', ...
            'State or covariance went non-finite during predict step.');
    end
end
