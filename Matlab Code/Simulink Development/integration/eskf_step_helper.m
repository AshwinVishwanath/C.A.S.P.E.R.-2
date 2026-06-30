function [state_x, P_diag, mach_gate_active, ungate_counter, ...
          last_baro_innov, last_zupt_innov] = eskf_step_helper( ...
              accel_nav_up, baro_alt, baro_new, mach_in, zupt_trig, init_done)
%ESKF_STEP_HELPER Persistent-state wrapper around the T08 ESKF for the
% Simulink visual model.
%
% Synopsis:
%   [state_x, P_diag, mach_gate_active, ungate_counter, last_baro_innov, ...
%    last_zupt_innov] = eskf_step_helper(accel_nav_up, baro_alt, baro_new,
%                                        mach_in, zupt_trig, init_done)
%
% This helper exists because the eskf_step MATLAB Function block (inside
% casper_sim_phase0.slx/ESKF) cannot easily express persistent-state
% management on `struct` values when coder.extrinsic is in play (the parser
% can't statically infer the struct's shape). By moving the logic to a
% regular .m function and declaring the whole thing coder.extrinsic, the
% Simulink chart only sees fixed-size scalar/vector outputs.
%
% Behavior:
%   - On first call, allocates ESKF state via casper_eskf_state('zero', Est).
%   - While init_done is false, holds the held state and returns it (no
%     predict / update).
%   - On the first call with init_done && baro_new, re-initializes the
%     ESKF via casper_eskf_state('init', Est, baro_alt) so the alt seed
%     comes from the actual baro reading at attitude-init moment.
%   - Once "init_armed" is true, runs the full predict + mach gate +
%     conditional baro + conditional ZUPT update chain on each call.
%
% This mirrors the gating semantics used in casper_phase0_run.m's main
% loop and matches the EKF_SPEC v2.1 init-on-baro-after-attitude-init rule.

    persistent st Est inited init_armed

    if isempty(inited)
        Est = evalin('base', 'Estimator');
        st  = casper_eskf_state('zero', Est);
        inited = true;
        init_armed = false;
    end

    if ~logical(init_done)
        % Hold initial state until attitude says go.
        state_x = st.x_vec(:);
        P_diag  = diag(st.P_mat);
        mach_gate_active = double(st.mach_gate_active);
        ungate_counter   = double(st.ungate_counter);
        last_baro_innov  = double(0);
        last_zupt_innov  = double(0);
        return;
    end

    if ~init_armed && logical(baro_new)
        st = casper_eskf_state('init', Est, double(baro_alt));
        init_armed = true;
    end

    if init_armed
        st = casper_eskf_predict(st, double(accel_nav_up), Est);
        st = casper_eskf_mach_gate(st, double(mach_in), Est);
        if logical(baro_new)
            st = casper_eskf_update_baro(st, double(baro_alt), Est);
        end
        if logical(zupt_trig)
            st = casper_eskf_update_zupt(st, Est);
        end
    end

    state_x          = st.x_vec(:);
    P_diag           = diag(st.P_mat);
    mach_gate_active = double(st.mach_gate_active);
    ungate_counter   = double(st.ungate_counter);
    last_baro_innov  = st.last_baro_innov_m;
    last_zupt_innov  = st.last_zupt_innov_mps;
end
