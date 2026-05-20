function [att_out, state_out] = casper_attitude_tick( ...
        accel_body_fw_mps2, gyro_body_fw_radps, mag_body_fw_uT, ...
        mag_new_sample, mode_pad, dt_s, params, state_in)
%CASPER_ATTITUDE_TICK  One IMU-rate update for the stripped attitude estimator.
%
%   Composes the per-tick behavior of casper_att_update() (firmware) plus
%   the one-time casper_att_static_init() handover.
%
%   Inputs:
%     accel_body_fw_mps2   (3x1) raw body accel, firmware frame, m/s^2
%     gyro_body_fw_radps   (3x1) raw body gyro, firmware frame, rad/s
%     mag_body_fw_uT       (3x1) calibrated mag, firmware frame, uT
%                                (only consumed when mag_new_sample == true)
%     mag_new_sample       (bool) raw mag new-sample flag (100 Hz typical)
%     mode_pad             (bool) true = pad (Mahony), false = flight (RK4 + 10Hz mag)
%     dt_s                 (1x1) tick period, s (use 1/833 to match firmware)
%     params               struct: Attitude.* fields from T02 (Kp_Grav, ...
%                                  Kp_MagPad, Kp_MagFlight, Ki, GyroLpfCutoff_Hz,
%                                  MagUpdateRate_Hz, HeadingSigmaFloor_rad,
%                                  GyroArw_radSqrtS [3x1], StaticInitSamples,
%                                  StaticInitTimeout_s)
%                          plus  .M_ref_nav_uT  (3x1, optional override; if
%                                  init's mag fails this is the fallback)
%
%     state_in             struct (call casper_attitude_state_new() to init)
%
%   Outputs:
%     att_out              struct:
%       .quat_body_to_nav  (4x1) Hamilton, w>=0, unit norm
%       .gyro_bias_radps   (3x1) frozen at static-init in Phase 0
%       .heading_sigma_rad (scalar)
%       .init_complete     (bool)
%       .e_mag             (3x1) most recent mag error vector (debug)
%       .e_grav            (3x1) most recent gravity error vector (debug)
%       .fired_mag_flight  (bool) true iff a flight-mode mag correction
%                                 actually applied this tick
%
%     state_out            struct with updated history (pass back next tick)

    % ── 1. Initialize state if first call ────────────────────────────────
    if isempty(state_in) || ~isstruct(state_in) || ...
       ~isfield(state_in, 'init_complete')
        state_in = casper_attitude_state_new();
    end
    state_out = state_in;

    % ── 2. Static-init phase ─────────────────────────────────────────────
    if ~state_in.init_complete
        % Accumulate gyro for static bias (Phase 0 spec §5.3).
        % Firmware does NOT do this; spec says to.
        if state_in.gyro_bias_count < 100
            state_out.gyro_bias_sum_radps = state_in.gyro_bias_sum_radps + gyro_body_fw_radps(:);
            state_out.gyro_bias_count     = state_in.gyro_bias_count + 1;
        end

        % Build local init params with optional M_ref_nav_uT
        init_p = struct( ...
            'StaticInitSamples',   params.StaticInitSamples, ...
            'StaticInitTimeout_s', params.StaticInitTimeout_s);
        if isfield(params, 'M_ref_nav_uT')
            init_p.M_ref_nav_uT = params.M_ref_nav_uT;
        else
            init_p.M_ref_nav_uT = zeros(3,1);
        end

        in_st = struct( ...
            'accel_sum_mps2',      state_out.init_accel_sum, ...
            'accel_count',         state_out.init_accel_count, ...
            'mag_sum_uT',          state_out.init_mag_sum, ...
            'mag_count',           state_out.init_mag_count, ...
            'init_elapsed_s',      state_out.init_elapsed_s);

        [done, init_out, out_st] = casper_attitude_static_init( ...
            accel_body_fw_mps2, mag_body_fw_uT, mag_new_sample, dt_s, ...
            init_p, in_st);

        state_out.init_accel_sum   = out_st.accel_sum_mps2;
        state_out.init_accel_count = out_st.accel_count;
        state_out.init_mag_sum     = out_st.mag_sum_uT;
        state_out.init_mag_count   = out_st.mag_count;
        state_out.init_elapsed_s   = out_st.init_elapsed_s;

        if done
            state_out.q_body_to_nav  = init_out.q_body_to_nav;
            state_out.m_ref_nav_uT   = init_out.m_ref_nav_uT;
            state_out.mag_available  = init_out.mag_available;
            state_out.init_complete  = true;

            if state_out.gyro_bias_count > 0
                state_out.gyro_bias_radps = ...
                    state_out.gyro_bias_sum_radps / state_out.gyro_bias_count;
            else
                state_out.gyro_bias_radps = zeros(3,1);
            end
            % Initialize gyro LPF to the first raw sample to avoid a
            % first-order transient.
            state_out.gyro_filt_radps = gyro_body_fw_radps(:);
        end

        att_out = pack_att_out(state_out, false, zeros(3,1), zeros(3,1));
        return;
    end

    % ── 3. Steady-state tick: LPF, bias subtract ─────────────────────────
    lpf_state = struct('gyro_filt', state_out.gyro_filt_radps);
    [gyro_filt, lpf_state] = casper_attitude_gyro_lpf( ...
        gyro_body_fw_radps, dt_s, params.GyroLpfCutoff_Hz, lpf_state);
    state_out.gyro_filt_radps = lpf_state.gyro_filt;

    omega_minus_bias = gyro_filt - state_out.gyro_bias_radps;

    % ── 4. Mode-dependent correction ─────────────────────────────────────
    e_grav = zeros(3,1);
    e_mag  = zeros(3,1);
    fired_mag_flight = false;

    if mode_pad
        [omega_corr, e_int_out, e_grav, e_mag] = casper_attitude_mahony( ...
            omega_minus_bias, accel_body_fw_mps2, mag_body_fw_uT, ...
            mag_new_sample, state_out.q_body_to_nav, ...
            state_out.m_ref_nav_uT, state_out.mag_available, ...
            params, state_out.e_int_radps, dt_s);
        state_out.e_int_radps = e_int_out;
        % Reset flight mag-decimation timer while on pad
        state_out.mag_timer_s = 0;
    else
        [omega_corr, e_int_out, fired_mag_flight, mag_timer_out, ...
            heading_sigma_out, e_mag] = casper_attitude_mag_correct_flight( ...
                omega_minus_bias, mag_body_fw_uT, mag_new_sample, ...
                state_out.q_body_to_nav, state_out.m_ref_nav_uT, ...
                state_out.mag_available, params, state_out.e_int_radps, ...
                state_out.mag_timer_s, state_out.heading_sigma_rad, dt_s);
        state_out.e_int_radps      = e_int_out;
        state_out.mag_timer_s      = mag_timer_out;
        state_out.heading_sigma_rad = heading_sigma_out;
    end

    % ── 5. RK4 propagation ───────────────────────────────────────────────
    state_out.q_body_to_nav = casper_attitude_predict_rk4( ...
        state_out.q_body_to_nav, omega_corr, dt_s);

    % ── 6. Uncertainty (ARW integration) ─────────────────────────────────
    arw = params.GyroArw_radSqrtS(:);
    state_out.att_sigma_rad = sqrt( ...
        state_out.att_sigma_rad.^2 + arw.^2 * dt_s);

    % Heading sigma growth (nose-axis Y ARW = arw(2))
    state_out.heading_sigma_rad = sqrt( ...
        state_out.heading_sigma_rad^2 + arw(2)^2 * dt_s);

    % Enforce heading sigma floor (acceptance criterion #6: always >= floor).
    if state_out.heading_sigma_rad < params.HeadingSigmaFloor_rad
        state_out.heading_sigma_rad = params.HeadingSigmaFloor_rad;
    end

    state_out.mission_time_s = state_out.mission_time_s + dt_s;

    att_out = pack_att_out(state_out, fired_mag_flight, e_grav, e_mag);
end

% ---------------------------------------------------------------------------
function att = pack_att_out(state, fired_mag_flight, e_grav, e_mag)
    att = struct( ...
        'quat_body_to_nav',   state.q_body_to_nav, ...
        'gyro_bias_radps',    state.gyro_bias_radps, ...
        'heading_sigma_rad',  state.heading_sigma_rad, ...
        'init_complete',      state.init_complete, ...
        'fired_mag_flight',   fired_mag_flight, ...
        'e_grav',             e_grav(:), ...
        'e_mag',              e_mag(:));
end
