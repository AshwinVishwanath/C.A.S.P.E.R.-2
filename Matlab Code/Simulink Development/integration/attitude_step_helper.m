function [quat_fw, gyro_bias, heading_sigma, init_complete] = ...
            attitude_step_helper(accel, gyro, mag, mag_new_eff, ~, dt)
%ATTITUDE_STEP_HELPER Persistent-state wrapper around T09 attitude tick.
%
% Synopsis:
%   [quat_fw, gyro_bias, heading_sigma, init_complete] = attitude_step_helper(
%       accel, gyro, mag, mag_new_eff, mode_pad_in, dt)
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
%   - mode_pad latch: starts true, flips to false ONCE when
%     |a_fw| > Attitude.LaunchAccel_g * 9.80665. Mirrors the legacy
%     driver casper_phase0_run.m lines 336-340. The mode_pad_in argument
%     is IGNORED so the wrapper owns the latch state itself (the visual
%     model previously pinned a constant `true`, which made the Mahony
%     gravity-correction try to align the boost thrust vector with the
%     gravity reference -- causing the attitude estimate to diverge by
%     ~80-180 deg as soon as boost began).
%   - mag gating: also matches the legacy driver -- mag samples are
%     suppressed until static-init completes (independent of any input
%     mag_new_eff value). This guarantees the init's tilt-compensated
%     heading sees the same byte-exact stream the legacy path saw.
%
% Outputs:
%   quat_fw       : 4x1 body-to-nav (Zup) quaternion, scalar-first
%   gyro_bias     : 3x1 estimated gyro bias (rad/s)
%   heading_sigma : 1x1 heading uncertainty (rad)
%   init_complete : 1x1 logical, true once static init has converged

    persistent st p inited mode_pad_latch launch_thresh_mps2 transient_ticks_left

    if isempty(inited)
        st = casper_attitude_state_new();
        p  = evalin('base', 'Attitude');
        if ~isfield(p, 'M_ref_nav_uT')
            p.M_ref_nav_uT = [0; 0; 50];
        end
        % Internal launch-detect latch (matches legacy driver line 248-249,
        % 336-340).
        mode_pad_latch       = true;
        if isfield(p, 'LaunchAccel_g')
            launch_thresh_mps2 = double(p.LaunchAccel_g) * 9.80665;
        else
            % Fallback default if Attitude struct lacks the field.
            launch_thresh_mps2 = 3.0 * 9.80665;
        end
        % --- Drop the first N transient ticks ---
        % The visual model's imu_block_visual.slx funnels truth_ts ->
        % Rate Transition -> imuSensor (Sensor Fusion Tbx, extrinsic
        % via casper_imu_lsm_step). At sim_start the rate-transition +
        % extrinsic-call initialization combination produces a small
        % handful of corrupt IMU samples (typically the very first sample
        % saturates at +/- 32 g on every axis, which corresponds to a
        % full-scale gyro reading too). If those samples are accumulated
        % by casper_attitude_static_init's gyro-bias averaging window
        % (first 100 samples, see casper_attitude_tick lines 51-54), the
        % computed bias offset is far from the true sensor bias. After
        % init completes, that wrong bias gets subtracted from every
        % subsequent steady-state gyro reading, producing a phantom
        % constant angular rate of ~0.3-0.5 rad/s and a quaternion that
        % drifts ~30 deg/s -- the symptom we are fixing.
        %
        % Fix: silently drop the first 50 visual ticks (50 ms at 1 kHz)
        % so the attitude estimator never sees the transient. The legacy
        % MATLAB driver doesn't have this problem because it calls
        % casper_imu_lsm_model directly (no rate-transition / extrinsic
        % handshake), but for the visual model this guard is essential.
        transient_ticks_left = 50;
        inited = true;
    end

    if transient_ticks_left > 0
        transient_ticks_left = transient_ticks_left - 1;
        quat_fw       = [1;0;0;0];
        gyro_bias     = zeros(3,1);
        heading_sigma = double(0);
        init_complete = false;
        return;
    end

    % The caller's mode_pad input (now anonymous via `~`) is intentionally
    % ignored -- we own this latch internally so the visual model can't
    % hard-pin it `true` and cause boost-time attitude divergence.
    a_fw = double(accel(:));
    if mode_pad_latch && norm(a_fw) > launch_thresh_mps2
        mode_pad_latch = false;
    end

    % Mirror legacy driver: until static-init completes, force mag_new_eff
    % false so the tilt-compensated heading path never sees a mag sample
    % whose nav-frame convention disagrees with T07's T_nav_quat. After
    % init completes we honor the caller's gating signal (the visual
    % model's mag_data_ready AND init_complete_delayed AND gate).
    mag_new_eff_eff = logical(mag_new_eff);
    if ~st.init_complete
        mag_new_eff_eff = false;
    end

    [att, st] = casper_attitude_tick( ...
        a_fw, double(gyro(:)), double(mag(:)), ...
        mag_new_eff_eff, mode_pad_latch, double(dt), p, st);

    quat_fw       = att.quat_body_to_nav(:);
    gyro_bias     = att.gyro_bias_radps(:);
    heading_sigma = double(att.heading_sigma_rad);
    init_complete = logical(att.init_complete);
end
