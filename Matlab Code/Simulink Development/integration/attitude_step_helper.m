function [quat_fw, gyro_bias, heading_sigma, init_complete] = ...
            attitude_step_helper(accel, gyro, mag, mag_new_eff, mode_pad, dt)
%ATTITUDE_STEP_HELPER Persistent-state wrapper around T09 attitude tick.
%
% Synopsis:
%   [quat_fw, gyro_bias, heading_sigma, init_complete] = attitude_step_helper(
%       accel, gyro, mag, mag_new_eff, mode_pad, dt)
%
% Companion to eskf_step_helper.m. Hides the persistent attitude state
% struct from the MATLAB Function block's static parser, which can't
% statically infer struct shape through extrinsic calls.
%
% Behavior mirrors the attitude path in casper_phase0_run.m:
%   - On first call, allocate state via casper_attitude_state_new() and
%     pull params from base WS Attitude. Defaults M_ref_nav_uT if missing.
%   - Each call runs casper_attitude_tick with mag_new_eff already gated
%     by the caller (the ATTITUDE subsystem performs the
%     mag_data_ready AND init_complete_delayed gate via Unit Delay).
%
% Outputs:
%   quat_fw       : 4x1 body-to-nav (Zup) quaternion, scalar-first
%   gyro_bias     : 3x1 estimated gyro bias (rad/s)
%   heading_sigma : 1x1 heading uncertainty (rad)
%   init_complete : 1x1 logical, true once static init has converged

    persistent st p inited

    if isempty(inited)
        st = casper_attitude_state_new();
        p  = evalin('base', 'Attitude');
        if ~isfield(p, 'M_ref_nav_uT')
            p.M_ref_nav_uT = [0; 0; 50];
        end
        inited = true;
    end

    [att, st] = casper_attitude_tick( ...
        double(accel(:)), double(gyro(:)), double(mag(:)), ...
        logical(mag_new_eff), logical(mode_pad), double(dt), p, st);

    quat_fw       = att.quat_body_to_nav(:);
    gyro_bias     = att.gyro_bias_radps(:);
    heading_sigma = double(att.heading_sigma_rad);
    init_complete = logical(att.init_complete);
end
