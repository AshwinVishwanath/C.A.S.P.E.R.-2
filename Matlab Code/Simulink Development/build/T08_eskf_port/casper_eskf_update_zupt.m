function state = casper_eskf_update_zupt(state, Estimator)
%CASPER_ESKF_UPDATE_ZUPT Zero-velocity pseudo-measurement (gate-bypassed).
%
% Synopsis:
%   state = casper_eskf_update_zupt(state, Estimator)
%
% Inputs:
%   state     : casper_eskf_state container (must be initialized)
%   Estimator : sensor params struct (T02)
%
% Outputs:
%   state     : updated container
%
% Algorithm (matches Software/App/nav/casper_ekf.c casper_ekf_update_zupt()):
%
%   - H = [0 1 0 0]
%   - z = 0 (pseudo-measurement: velocity is zero)
%   - R = R_ZUPT
%   - Gate K^2 = +Inf (firmware uses INFINITY) — NEVER reject ZUPT updates.
%   - Joseph-form scalar update.
%
% IMPORTANT: This update does NOT have a 5-sigma innovation gate. The firmware
% explicitly passes INFINITY because gating ZUPT caused filter divergence
% during low-velocity boost-end phase. See spec T08 sec 7.2.
%
% Source firmware reference:
%   Software/App/nav/casper_ekf.c casper_ekf_update_zupt() lines 341-348

    if ~state.initialized
        error('casper_eskf_update_zupt:NotInitialized', ...
            'State must be initialized before update.');
    end

    H = Estimator.H_Zupt;            % 1x4 row
    if ~isrow(H)
        H = H(:).';
    end

    R = Estimator.R_Zupt;
    gate_k2 = Inf;                   % bypass innovation gate
    z = 0;                           % zero-velocity pseudo measurement

    Hx = H * state.x_vec;
    innov = z - Hx;
    state.last_zupt_innov_mps = innov;

    S = H * state.P_mat * H.' + R;

    % Gate check: with gate_k2 = Inf, innov^2 <= Inf*S is always true (S>0).
    % Keep the form for parity with the C update path.
    if ~(innov * innov <= gate_k2 * S)
        return;   % unreachable for Inf gate, but matches firmware path
    end
    if ~isfinite(S) || S <= 0
        return;
    end

    K = (state.P_mat * H.') / S;

    state.x_vec = state.x_vec + K * innov;

    I_KH = eye(4) - K * H;
    P_new = I_KH * state.P_mat * I_KH.' + K * R * K.';
    P_new = 0.5 * (P_new + P_new.');

    % P-floor on baro bias (firmware applies symmetrize but not P-floor inside
    % joseph_scalar_update; the post-update P-floor lives only inside
    % update_baro. For ZUPT no P-floor is applied -- match exactly.)
    state.P_mat = P_new;

    if any(~isfinite(state.x_vec)) || any(~isfinite(state.P_mat(:)))
        error('casper_eskf_update_zupt:NonFinite', ...
            'State or covariance went non-finite during ZUPT update.');
    end
end
