function state = casper_eskf_update_baro(state, baro_alt_m, Estimator)
%CASPER_ESKF_UPDATE_BARO Scalar baro update (Joseph form, with Mach gate).
%
% Synopsis:
%   state = casper_eskf_update_baro(state, baro_alt_m, Estimator)
%
% Inputs:
%   state      : casper_eskf_state container (must be initialized)
%   baro_alt_m : (scalar) baro altitude reading (m)
%   Estimator  : sensor params struct (T02)
%
% Outputs:
%   state      : updated container
%
% Algorithm (matches Software/App/nav/casper_ekf.c casper_ekf_update_baro()):
%
%   - If mach_gate_active: record skip and return immediately.
%   - If baro_alt_m non-finite: return without modifying state.
%   - On first call after gate releases: zero accel_bias and baro_bias states,
%     reset their P diagonals to (P_UNGATE_ACCEL_BIAS, P_UNGATE_BARO_BIAS),
%     and zero the row/column cross-covariances for indices 3 and 4. Then
%     set ungate_counter = 0 so the next N_UNGATE_STEPS updates use R_UNGATE.
%   - Select R: R_BARO_UNGATE if ungate_counter < N_UNGATE_STEPS, else R_BARO.
%   - Innovation: y_pred = x(1) + x(4); innov = baro_alt_m - y_pred.
%   - S = H*P*H' + R, where H = [1 0 0 1].
%   - Gate: if innov^2 > BARO_GATE_K2 * S, reject (no state change).
%     This matches firmware's NaN-safe inverted comparison.
%   - Joseph form Kalman update.
%   - P-floor on P(4,4) after update.
%
% Source firmware reference:
%   Software/App/nav/casper_ekf.c casper_ekf_update_baro() lines 290-339

    if ~state.initialized
        error('casper_eskf_update_baro:NotInitialized', ...
            'State must be initialized before update.');
    end

    state.baro_update_was_skipped = false;
    state.baro_update_was_accepted = false;

    % --- Mach gated: skip update entirely ---
    if state.mach_gate_active
        state.baro_update_was_skipped = true;
        return;
    end

    % --- NaN/Inf rejection ---
    if ~isfinite(baro_alt_m)
        state.baro_update_was_skipped = true;
        return;
    end

    % --- Gate-open transition: bias reset + P inflation ---
    % The mach gate logic in this MATLAB port detects the gate-release
    % at the moment mach_gate_active flips false (handled in
    % casper_eskf_mach_gate()), which also applies the bias-state reset
    % directly there. To exactly mirror the firmware C semantics, the
    % update_baro routine must also be tolerant of the "first baro after
    % a gate release" — we use ungate_counter == 0 (set by gate release)
    % as the indicator that the inflated R should be applied for the next
    % N_UNGATE_STEPS baro updates.

    % --- R selection: inflated for next N_UNGATE_STEPS, then nominal ---
    if state.ungate_counter < Estimator.N_UngateSteps
        R = Estimator.R_BaroUngate;
        state.ungate_counter = state.ungate_counter + 1;
    else
        R = Estimator.R_Baro;
    end

    % --- Joseph-form scalar update with H = [1 0 0 1] ---
    H = Estimator.H_Baro;            % 1x4 row
    if ~isrow(H)
        H = H(:).';
    end

    [state.x_vec, state.P_mat, accepted, innov, S] = local_joseph_scalar_update( ...
        state.x_vec, state.P_mat, H, baro_alt_m, R, Estimator.BaroGateK2);

    state.last_baro_innov_m = innov;
    state.last_baro_innov_var = S;
    state.baro_update_was_accepted = accepted;
    state.baro_update_was_skipped = ~accepted;

    % --- P-floor on baro bias variance ---
    if state.P_mat(4, 4) < Estimator.PFloorBaroBias
        state.P_mat(4, 4) = Estimator.PFloorBaroBias;
    end

    % NaN/Inf safety
    if any(~isfinite(state.x_vec)) || any(~isfinite(state.P_mat(:)))
        error('casper_eskf_update_baro:NonFinite', ...
            'State or covariance went non-finite during baro update.');
    end
end


function [x_out, P_out, accepted, innov, S] = local_joseph_scalar_update( ...
        x_in, P_in, H, z, R, gate_k2)
%LOCAL_JOSEPH_SCALAR_UPDATE Joseph form scalar measurement update.
%   Matches Software/App/nav/casper_ekf.c joseph_scalar_update() including
%   the inverted-NaN-safe gate comparison.

    x_out = x_in;
    P_out = P_in;
    accepted = false;

    % innovation = z - H*x
    Hx = H * x_in;
    innov = z - Hx;

    % S = H*P*H' + R (scalar)
    S = H * P_in * H.' + R;

    % Inverted comparison rejects NaN (firmware: !(innov^2 <= gate_k2 * S))
    if ~(innov * innov <= gate_k2 * S)
        return;
    end

    if ~isfinite(S) || S <= 0
        return;
    end

    % K = P*H' / S (4x1)
    K = (P_in * H.') / S;

    % State update
    x_out = x_in + K * innov;

    % Joseph form: P = (I - K*H) * P * (I - K*H)' + K*R*K'
    I_KH = eye(4) - K * H;
    P_new = I_KH * P_in * I_KH.' + K * R * K.';
    P_new = 0.5 * (P_new + P_new.');   % symmetrize

    P_out = P_new;
    accepted = true;
end
