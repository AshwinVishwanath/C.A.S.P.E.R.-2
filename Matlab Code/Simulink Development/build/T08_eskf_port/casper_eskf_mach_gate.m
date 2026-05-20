function state = casper_eskf_mach_gate(state, mach, Estimator)
%CASPER_ESKF_MACH_GATE Mach hysteresis state machine + un-gate inflation.
%
% Synopsis:
%   state = casper_eskf_mach_gate(state, mach, Estimator)
%
% Inputs:
%   state     : casper_eskf_state container
%   mach      : (scalar) current Mach number to test against hysteresis
%   Estimator : sensor params struct (T02)
%
% Outputs:
%   state     : updated container (mach_gate_active, ungate_counter, and
%               possibly x_vec and P_mat on gate-release transition)
%
% Algorithm (matches spec T08 sec 8 + firmware update_baro un-gate logic):
%
%   if not currently gated and mach > MACH_GATE_ON (0.40)
%       mach_gate_active = true
%   elseif currently gated and mach < MACH_GATE_OFF (0.35)
%       mach_gate_active = false
%       % Apply ungate inflation NOW:
%       x(3) = 0; x(4) = 0
%       P(3,3) = P_UNGATE_ACCEL_BIAS = 1.0
%       P(4,4) = P_UNGATE_BARO_BIAS  = 10.0
%       Zero cross-covariances for rows/cols 3 and 4
%       ungate_counter = 0  -> next N_UNGATE_STEPS baro updates use R_UNGATE
%   end
%
% The firmware applies the bias reset and P inflation in update_baro the
% first time it runs after a gate release. In Phase 0 this mach gate function
% applies them at the moment of the transition itself. The functional outcome
% is identical because update_baro will not run between gate-release and the
% next baro sample anyway, and we hold ungate_counter at zero so the next 10
% baro updates use inflated R.
%
% Source firmware reference:
%   Software/App/nav/casper_ekf.c casper_ekf_predict() mach hysteresis lines
%   277-288 and casper_ekf_update_baro() lines 302-320 for un-gate inflation.

    if ~state.initialized
        error('casper_eskf_mach_gate:NotInitialized', ...
            'State must be initialized before mach gate update.');
    end

    if ~isfinite(mach)
        % Defensive: treat non-finite mach as "no transition"
        return;
    end

    if ~state.mach_gate_active && mach > Estimator.MachGateOn
        % Gate engages -- no state perturbation; just suppress baro updates
        state.mach_gate_active = true;
    elseif state.mach_gate_active && mach < Estimator.MachGateOff
        % Gate releases -- apply bias reset and P inflation now.
        state.mach_gate_active = false;

        state.x_vec(3) = 0;
        state.x_vec(4) = 0;

        state.P_mat(3, 3) = Estimator.P_UngateAccelBias;
        state.P_mat(4, 4) = Estimator.P_UngateBaroBias;

        % Zero cross-covariances for rows/cols 3 and 4 (firmware lines 316-319)
        for j = 1:4
            if j ~= 3
                state.P_mat(3, j) = 0;
                state.P_mat(j, 3) = 0;
            end
            if j ~= 4
                state.P_mat(4, j) = 0;
                state.P_mat(j, 4) = 0;
            end
        end

        % Reset un-gate counter so next N_UNGATE_STEPS baro updates use R_UNGATE.
        state.ungate_counter = 0;
    end
    % Otherwise: no transition.
end
