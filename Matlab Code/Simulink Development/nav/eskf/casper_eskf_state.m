function state = casper_eskf_state(action, varargin)
%CASPER_ESKF_STATE Stripped 4-state vertical EKF state container.
%
% Synopsis:
%   state = casper_eskf_state('init', Estimator, baro_alt_init_m)
%   state = casper_eskf_state('zero', Estimator)
%
% Inputs:
%   action            : 'init' or 'zero'
%   Estimator         : struct from casper_sensor_params.m
%   baro_alt_init_m   : (init only) initial baro altitude reading (m)
%
% Outputs:
%   state : struct with fields
%       x_vec            (4x1 double) [alt_m; vel_mps; accel_bias_mps2; baro_bias_m]
%       P_mat            (4x4 double) covariance
%       mach_gate_active (1x1 logical)
%       ungate_counter   (1x1 double) integer count
%       initialized      (1x1 logical)
%       last_baro_for_init_m (1x1 double)
%       last_baro_innov_m    (1x1 double) diagnostics
%       last_baro_innov_var  (1x1 double) S
%       last_zupt_innov_mps  (1x1 double) diagnostics
%       baro_update_was_skipped (1x1 logical) diagnostics
%       baro_update_was_accepted (1x1 logical) diagnostics
%
% Source firmware reference:
%   Software/App/nav/casper_ekf.c casper_ekf_init()
%
% Notes:
%   - Phase 0 stripped port. See MarkDown Claude Docs/ARCHITECTURE.md sec 7.
%   - Initial covariance and reset semantics match firmware verbatim.
%   - 'init' seeds altitude state from the first baro reading; 'zero' returns
%     an uninitialized container that gets populated on first call.

    if nargin < 1
        error('casper_eskf_state:NoAction', 'action argument is required.');
    end

    state = struct();
    state.x_vec = zeros(4, 1);
    state.P_mat = zeros(4, 4);
    state.mach_gate_active = false;
    state.ungate_counter = 0;          % count of un-gate inflated updates already applied
    state.initialized = false;
    state.last_baro_for_init_m = 0;
    state.last_baro_innov_m = 0;
    state.last_baro_innov_var = 0;
    state.last_zupt_innov_mps = 0;
    state.baro_update_was_skipped = false;
    state.baro_update_was_accepted = false;

    switch lower(action)
        case 'zero'
            % Empty container, will initialize on first call.
            return;

        case 'init'
            if numel(varargin) < 2
                error('casper_eskf_state:BadArgs', ...
                    'init requires Estimator struct and baro_alt_init_m.');
            end
            Estimator = varargin{1};
            baro_alt_init_m = varargin{2};

            % --- State initialization per spec section 4 ---
            state.x_vec(1) = baro_alt_init_m;   % alt from first baro
            state.x_vec(2) = 0;                  % vel
            state.x_vec(3) = 0;                  % accel bias
            state.x_vec(4) = 0;                  % baro bias

            % --- Initial covariance: diag([P0_ALT, P0_VEL, P0_ACCEL_BIAS, P0_BARO_BIAS]) ---
            state.P_mat = diag([Estimator.P0_Alt, ...
                                Estimator.P0_Vel, ...
                                Estimator.P0_AccelBias, ...
                                Estimator.P0_BaroBias]);

            % --- Firmware sets ungate_count = N_UNGATE_STEPS at init,
            %     meaning "no recovery needed on first boot" — i.e.,
            %     R_baro_nominal is used immediately, not R_baro_ungate. ---
            state.ungate_counter = Estimator.N_UngateSteps;

            state.mach_gate_active = false;
            state.initialized = true;
            state.last_baro_for_init_m = baro_alt_init_m;

        otherwise
            error('casper_eskf_state:UnknownAction', ...
                'Unknown action: %s. Expected init or zero.', action);
    end
end
