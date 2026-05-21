function out = casper_phase0_run(cfg, truth, varargin)
%CASPER_PHASE0_RUN Single-run driver: truth -> sensors -> nav -> outputs.
%
% Synopsis:
%   out = casper_phase0_run(cfg, truth)
%   out = casper_phase0_run(cfg, truth, 'StopTime', T)
%   out = casper_phase0_run(cfg, truth, 'PinTruth', true)   % stationary pad mode
%
% Inputs:
%   cfg   : struct from casper_sim_config()
%   truth : struct from casper_truth_resample() (loaded from truth_trajectory.mat)
%
% Optional name/value:
%   'StopTime'  (s)   truncate the truth window to this duration (default cfg.StopTime_s)
%   'PinTruth'  bool  if true, pin all truth fields to t=0 for the entire run
%                     (stationary-on-pad smoke test). Default false.
%   'PreLaunchPad_s' (s) prepend N seconds of stationary pad data before t=0
%                        to let the EKF/attitude converge before launch
%                        (default 5.0; matches T08 AC4/AC5 convention).
%
% Outputs (out struct):
%   .Truth     : truth fields aligned to the run window
%   .Estimate  : EKF state history (time, x, P_diag, mach_gate, ungate ctr,
%                quat_fw, baro/zupt innov diagnostics)
%   .Sensors   : per-sensor logged streams (imu, adxl, baro, mag, gps)
%   .RadioTX   : time + active flag
%   .Determ    : struct with sensor_streams + estimate for the hash comparison
%   .runtime_s : wall-clock for this run
%
% Determinism: all stochastic chains use persistent state derived from
% cfg.Seeds.*; we explicitly clear those persistent vars at run start so
% two consecutive calls with identical cfg/truth produce byte-identical
% sensor + estimate outputs.
%
% Source firmware reference: composes all T01..T09 modules per
% ARCHITECTURE.md §3-§7 and T11_integration.md §4-§5.

    p = inputParser();
    addParameter(p, 'StopTime',       cfg.StopTime_s);
    addParameter(p, 'PinTruth',       false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'PreLaunchPad_s', 5.0,   @(x) isnumeric(x) && isscalar(x) && x >= 0);
    parse(p, varargin{:});
    stop_time_s       = double(p.Results.StopTime);
    pin_truth         = logical(p.Results.PinTruth);
    pre_launch_pad_s  = double(p.Results.PreLaunchPad_s);

    % --- Pull T02 params straight from base workspace (canonical source) ----
    Sim_       = evalin('base', 'Sim');
    IMU_p      = evalin('base', 'IMU');
    ADXL_p     = evalin('base', 'ADXL');
    Baro_p     = evalin('base', 'Baro');
    Mag_p      = evalin('base', 'Mag');
    GPS_p      = evalin('base', 'GPS');
    Estimator_ = evalin('base', 'Estimator');
    Attitude_  = evalin('base', 'Attitude');
    Radio_p    = evalin('base', 'Radio');

    % T03 supplemental noise params
    IMU_local = casper_imu_local_params();

    % T06 launch-site augmentation
    GPS_local = casper_gps_local_params(GPS_p, Sim_);

    % --- Truncate truth to the requested window -----------------------------
    dt_solver = cfg.SolverDt_s;          % 1e-4 (10 kHz)
    N_total   = min(truth.n_samples, 1 + round(stop_time_s / dt_solver));

    truth_run = truncate_truth_(truth, N_total, pin_truth);
    t_s_truth = truth_run.time_s;

    % Apply pre-launch pad window: prepend N_pad samples of stationary t=0
    % truth so EKF/attitude have time to converge. Time-stamps continue from
    % -pad_s up to 0 then the trajectory.
    if pre_launch_pad_s > 0 && ~pin_truth
        N_pad = round(pre_launch_pad_s / dt_solver);
        truth_run = prepend_pad_(truth_run, N_pad, dt_solver);
        N_total = N_total + N_pad;
    end

    % --- Per-sensor downsampled indices into the 10 kHz truth grid ----------
    imu_dt   = 1.0 / IMU_p.Rate_Hz;        % 1/833
    adxl_dt  = 1.0 / ADXL_p.RatePostLaunch_Hz; % 1/800
    baro_dt  = 1.0 / Baro_p.Rate_Hz;       % 1/100
    mag_dt   = 1.0 / Mag_p.Rate_Hz;        % 1/100
    gps_dt   = 1.0 / GPS_p.Rate_Hz;        % 1/10

    [imu_idx,  imu_t]  = sensor_indices_(truth_run.time_s, imu_dt);
    [adxl_idx, adxl_t] = sensor_indices_(truth_run.time_s, adxl_dt);
    [baro_idx, baro_t] = sensor_indices_(truth_run.time_s, baro_dt);
    [mag_idx,  mag_t ] = sensor_indices_(truth_run.time_s, mag_dt);
    [gps_idx,  gps_t]  = sensor_indices_(truth_run.time_s, gps_dt);

    % EKF predict: 416 Hz; we step it inside the IMU loop using its own time
    % accumulator.
    ekf_dt = Estimator_.Dt;                % 0.0024

    % --- Allocate logs ------------------------------------------------------
    N_imu  = numel(imu_idx);
    N_adxl = numel(adxl_idx);
    N_baro = numel(baro_idx);
    N_mag  = numel(mag_idx);
    N_gps  = numel(gps_idx);

    sensors_imu_accel_g  = zeros(N_imu, 3);
    sensors_imu_gyro_dps = zeros(N_imu, 3);
    sensors_imu_accel_fw_mps2 = zeros(N_imu, 3);
    sensors_imu_gyro_fw_radps = zeros(N_imu, 3);
    sensors_imu_temp_C   = zeros(N_imu, 1);

    sensors_adxl_g       = zeros(N_adxl, 3);

    sensors_baro_press_pa = zeros(N_baro, 1);
    sensors_baro_alt_m    = zeros(N_baro, 1);

    sensors_mag_uT       = zeros(N_mag, 3);
    sensors_mag_raw18    = zeros(N_mag, 3, 'uint32');
    sensors_mag_uT_fw    = zeros(N_mag, 3);
    sensors_radio_active = false(N_mag, 1);

    sensors_gps_lat_deg7  = zeros(N_gps, 1, 'int32');
    sensors_gps_lon_deg7  = zeros(N_gps, 1, 'int32');
    sensors_gps_alt_mm    = zeros(N_gps, 1, 'int32');
    sensors_gps_vel_n_mms = zeros(N_gps, 1, 'int32');
    sensors_gps_vel_e_mms = zeros(N_gps, 1, 'int32');
    sensors_gps_vel_d_mms = zeros(N_gps, 1, 'int32');
    sensors_gps_fix       = zeros(N_gps, 1, 'uint8');
    sensors_gps_sv        = zeros(N_gps, 1, 'uint8');

    % EKF/attitude state histories: log every IMU tick (after frame switch
    % and one predict step at 416 Hz cadence). Estimate is "the most recent
    % EKF state visible to the firmware loop", so we log at the IMU rate.
    est_time_s        = zeros(N_imu, 1);
    est_state_x       = zeros(N_imu, 4);
    est_state_P_diag  = zeros(N_imu, 4);
    est_mach_gate     = false(N_imu, 1);
    est_ungate_counter = zeros(N_imu, 1);
    est_quat_fw       = zeros(N_imu, 4);
    est_attitude_init = false(N_imu, 1);
    est_baro_innov_m  = nan(N_baro, 1);     % logged at baro rate
    est_baro_innov_S  = nan(N_baro, 1);
    est_baro_used     = false(N_baro, 1);
    est_baro_accepted = false(N_baro, 1);
    est_zupt_innov_mps = nan(N_imu, 1);
    est_zupt_fired    = false(N_imu, 1);

    % --- Reset persistent state in all stochastic helpers -------------------
    % Each helper auto-reinits when its seed value changes, so the cleanest
    % cross-run reset is to call `clear functions` once at run start. That
    % invalidates persistents in every helper function we use, including
    % RandStreams in casper_imu_lsm_noise, casper_imu_adxl_noise,
    % casper_baro_noise_step, casper_mag_noise, casper_mag_radio_interference,
    % casper_gps_position_model, casper_gps_cocom_check,
    % casper_gps_hold_lastvalid, and casper_gps_latency.
    %
    % We deliberately do NOT use clear functions here because it would unload
    % every loaded function in the caller workspace and slow subsequent runs.
    % Instead each helper supports a seed-change-triggered reinit (verified
    % in each T0X test script).  We still bump seed sensitivity by passing
    % the documented per-sensor seed.
    seed_imu        = cfg.Seeds.IMU;
    seed_adxl       = cfg.Seeds.ADXL;
    seed_baro       = cfg.Seeds.Baro;     %#ok<NASGU> (consumed by casper_baro_noise_step via base WS)
    seed_mag        = cfg.Seeds.Mag;
    seed_radio_int  = cfg.Seeds.RadioInt;
    seed_gps        = cfg.Seeds.GPS;      % consumed by casper_gps_step

    % Override base WS Sim.Seed so casper_baro_noise_step sees the canonical value
    Sim_.Seed = double(cfg.Seed);
    assignin('base', 'Sim', Sim_);

    % --- Force reset: bump persistent state by ensuring no prior state -----
    % Many helpers detect seed change via a `last_seed` persistent. We force
    % a guaranteed reset by issuing one dummy call with a sentinel seed
    % differing from the production seed, then immediately the real seed.
    % That is unnecessary in practice (seed_change auto-reinits), so we just
    % rely on the seed-change-driven reinit pattern.  But for determinism
    % across runs in the SAME MATLAB session we MUST clear functions or
    % otherwise reinit -- without it, run-2 inherits run-1's RandStream
    % pointer which advances continuously and produces different bytes.
    % Use `munlock` per-function then clear so each helper's persistent is
    % wiped; this is fast enough (<0.05 s) and reliable.
    helpers_with_state = { ...
        'casper_imu_lsm_noise', 'casper_imu_adxl_noise', ...
        'casper_baro_noise_step', 'casper_baro_noise', ...
        'casper_mag_noise', 'casper_mag_radio_interference', ...
        'casper_gps_position_model', 'casper_gps_cocom_check', ...
        'casper_gps_hold_lastvalid', 'casper_gps_latency'};
    for k = 1:numel(helpers_with_state)
        if exist(helpers_with_state{k}, 'file')
            try
                clear(helpers_with_state{k});
            catch
                % ignore
            end
        end
    end

    % --- Attitude state ----------------------------------------------------
    att_state = casper_attitude_state_new();
    att_params = Attitude_;
    % Provide a Z-up nav-frame reference mag in the SAME convention the
    % quaternion path uses (T07 T_nav_quat = [0 1 0; 1 0 0; 0 0 -1]).
    % The vector-path frame switch (casper_frame_switch_nav) uses
    % T_nav = diag(1,1,-1) which keeps X/Y identity; the quaternion path
    % uses T_nav_quat which swaps X<->Y. For the attitude estimator to be
    % self-consistent with quaternion-based predictions, the mag reference
    % MUST live in the same nav-frame convention as the attitude quat
    % (i.e. T_nav_quat).
    K_frame = casper_frame_constants();
    mag_ned_uT = casper_mag_field_world([0;0;0]);   % constant in Phase 0
    att_params.M_ref_nav_uT = K_frame.T_nav_quat * mag_ned_uT;

    % --- EKF state: initialize at first baro sample ------------------------
    % Match firmware: the EKF first runs predict with x_vec(1)=baro_alt_init.
    % We seed alt from the FIRST baro reading we generate so the init
    % accel/baro biases are zero relative to whatever the simulated sensor
    % produced (bias offsets and quant land in P0_BARO_BIAS bucket).
    eskf = casper_eskf_state('zero', Estimator_);
    eskf_initialized = false;

    % --- Persistent for baro noise (we drive it directly here, not via base
    % WS, to avoid base-workspace coupling)
    baro_noise_state = struct('initialized', false);

    % --- Main loop ---------------------------------------------------------
    t_run_start = tic;

    % Pre-compute "next baro sample" index pointer
    next_baro_k = 1;
    next_mag_k  = 1;
    next_gps_k  = 1;
    next_adxl_k = 1;

    % EKF predict scheduler: fire when imu_t crosses a 1/416 grid boundary.
    % The IMU loop dt is 1/833 ~ 2x EKF rate, so we trigger an EKF predict
    % on every other IMU sample (with phase tracking to stay exact).
    ekf_time_accum = 0.0;
    last_ekf_time  = -inf;

    % NED accel trapezoidal accumulator across IMU samples (matches T08 §10
    % "Trapezoidal NED accel accumulation").  We average two IMU body
    % accelerations rotated to nav-Zup, then feed at 416 Hz.
    accel_zup_prev = NaN;
    accel_zup_curr = NaN;

    % Stationary-pad mode flag (Mahony correction with Kp_grav=10) until
    % launch detect.  Use firmware-style threshold: |a| > LaunchAccel_g * g.
    mode_pad = true;
    launch_accel_thresh_mps2 = Attitude_.LaunchAccel_g * 9.80665;

    for k_imu = 1:N_imu
        idx_t = imu_idx(k_imu);
        t_now = imu_t(k_imu);

        % --- Truth at this IMU sample (already on truth_run grid) ---------
        a_NED  = truth_run.accel_NED(idx_t, :).';
        v_NED  = truth_run.vel_NED  (idx_t, :).';
        q_std  = truth_run.quat_std (idx_t, :).';
        w_body = truth_run.omega_body_std(idx_t, :).';

        % --- IMU clean model (T03) ----------------------------------------
        [a_g_std, g_dps_std, temp_C, ~] = casper_imu_lsm_model(a_NED, q_std, w_body);

        % --- IMU noise (T03) -----------------------------------------------
        reset_flag = (k_imu == 1);
        [a_g_std_n, g_dps_std_n] = casper_imu_lsm_noise( ...
            a_g_std, g_dps_std, imu_dt, ...
            seed_imu, ...
            IMU_local.AccelBiasInit_g,  IMU_local.GyroBiasInit_dps, ...
            IMU_local.CrossAxis_deg,    IMU_local.ScaleFactor_ppm, ...
            Estimator_.AccelVRW,        Estimator_.AccelBiSigma, ...
            Attitude_.GyroArw_radSqrtS, ...
            IMU_p.AccelScale_gPerLSB,   IMU_p.GyroScale_dpsPerLSB, ...
            IMU_p.AccelRange_g,         IMU_p.GyroRange_dps, ...
            reset_flag);

        sensors_imu_accel_g(k_imu, :)  = a_g_std_n.';
        sensors_imu_gyro_dps(k_imu, :) = g_dps_std_n.';
        sensors_imu_temp_C(k_imu)      = temp_C;

        % --- Frame switch (T07) std-body -> firmware-body -----------------
        a_mps2_std = a_g_std_n * 9.80665;
        g_rad_std  = g_dps_std_n * (pi/180);
        a_fw_mps2  = casper_frame_switch_body(a_mps2_std);
        g_fw_radps = casper_frame_switch_body(g_rad_std);
        sensors_imu_accel_fw_mps2(k_imu, :) = a_fw_mps2.';
        sensors_imu_gyro_fw_radps(k_imu, :) = g_fw_radps.';

        % --- Mag sensor (T05) when a 100 Hz sample lands ------------------
        mag_new_for_att = false;
        mag_fw_uT_now   = zeros(3,1);
        if next_mag_k <= N_mag && idx_t >= mag_idx(next_mag_k)
            km = next_mag_k;
            t_mag = mag_t(km);

            mag_ned    = casper_mag_field_world(truth_run.pos_NED(idx_t,:).');
            mag_clean_body = casper_mag_rotate_to_body(mag_ned, q_std);

            mag_raw_body = casper_mag_distort_field(mag_clean_body, ...
                Mag_p.HardIron_uT, Mag_p.SoftIron, Mag_p.AxisFlipSign);

            [mag_noisy_uT, mag_raw_18bit] = casper_mag_noise( ...
                mag_raw_body, mag_dt, ...
                Mag_p.NoiseTauSec, Mag_p.NoiseStd_uT, ...
                Mag_p.ScaleCountsPerGauss, Mag_p.OffsetCounts, ...
                seed_mag);

            tx_active = casper_radio_tx_schedule(t_mag, ...
                Mag_p.RadioTXPeriod_s, Mag_p.RadioTXAirtime_s);
            sensors_radio_active(km) = tx_active;

            mag_post_interf = casper_mag_radio_interference(mag_noisy_uT, ...
                tx_active, Mag_p.RadioInterfActive, Mag_p.RadioSpikeAmp_uT, ...
                seed_radio_int);

            % Apply firmware-side calibration so the attitude estimator
            % receives calibrated mag (axis flip + soft_iron*(raw - hard_iron))
            mag_cal_body = apply_mag_cal_(mag_post_interf, ...
                Mag_p.HardIron_uT, Mag_p.SoftIron, Mag_p.AxisFlipSign);

            % Frame switch std-body -> firmware-body for attitude consumption
            mag_fw_uT_now = casper_frame_switch_body(mag_cal_body);

            sensors_mag_uT(km, :)   = mag_post_interf.';
            sensors_mag_raw18(km,:) = mag_raw_18bit(:).';
            sensors_mag_uT_fw(km,:) = mag_fw_uT_now.';

            mag_new_for_att = true;
            next_mag_k = next_mag_k + 1;
        end

        % --- Attitude tick (T09) -----------------------------------------
        % Pad/flight mode latch: leave pad mode once |a| exceeds launch
        % threshold (firmware uses 3g).  Phase 0 stripped: no flight FSM,
        % so we use this simple latch and never go back to pad mode.
        if mode_pad
            if norm(a_fw_mps2) > launch_accel_thresh_mps2
                mode_pad = false;
            end
        end

        % Mag init path: the firmware's static_init uses a tilt-compensated
        % heading formula whose internal nav-frame convention disagrees with
        % T07's T_nav_quat (X<->Y swap). To keep truth_fw and estimate_fw
        % in the same nav frame we suppress mag samples until init completes
        % via the gravity-only timeout. This matches firmware behavior when
        % the mag sensor is unavailable.  Kp_MagFlight = 0 means subsequent
        % flight ticks ignore mag anyway, so this is a no-op in steady state.
        mag_for_init = mag_fw_uT_now;
        mag_new_eff  = mag_new_for_att;
        if ~att_state.init_complete
            mag_new_eff = false;
        end
        [att_out, att_state] = casper_attitude_tick( ...
            a_fw_mps2, g_fw_radps, mag_for_init, ...
            mag_new_eff, mode_pad, imu_dt, att_params, att_state);

        est_quat_fw(k_imu, :)      = att_out.quat_body_to_nav(:).';
        est_attitude_init(k_imu)   = att_out.init_complete;

        % --- ADXL sample (T03 ADXL noise) --------------------------------
        % Phase 0: log clean ADXL value; not used by EKF or attitude.
        if next_adxl_k <= N_adxl && idx_t >= adxl_idx(next_adxl_k)
            ka = next_adxl_k;
            a_NED_adxl = truth_run.accel_NED(idx_t, :).';
            q_now      = truth_run.quat_std (idx_t, :).';
            alt_for_adxl = truth_run.alt_m(idx_t);
            [adxl_g, ~] = casper_imu_adxl_model(a_NED_adxl, q_now, alt_for_adxl, 5.0);
            % Apply ADXL noise chain (post-launch ODR/BW)
            adxl_bw = ADXL_p.BandwidthPostLaunch_Hz;
            try
                [adxl_g_n] = casper_imu_adxl_noise(adxl_g, adxl_dt, ...
                    seed_adxl, IMU_local.ADXL_BiasInit_g, IMU_local.ADXL_Noise_g, ...
                    adxl_bw, ADXL_p.Scale_gPerLSB, ADXL_p.Range_g, (ka == 1));
            catch ME
                warning('casper_phase0_run:adxl', 'ADXL noise step %d failed: %s', ka, ME.message);
                adxl_g_n = adxl_g;
            end
            sensors_adxl_g(ka, :) = adxl_g_n.';
            next_adxl_k = next_adxl_k + 1;
        end

        % --- Baro sample (T04) -------------------------------------------
        baro_new_for_ekf = false;
        baro_alt_for_ekf = NaN;
        if next_baro_k <= N_baro && idx_t >= baro_idx(next_baro_k)
            kb = next_baro_k;
            t_b = baro_t(kb);

            p_clean = casper_baro_pressure_model(truth_run.air_pressure_pa(idx_t));
            p_shock = casper_baro_mach_shock(p_clean, truth_run.mach(idx_t), ...
                v_NED, truth_run.air_density_kgm3(idx_t));
            [p_meas, baro_noise_state] = casper_baro_noise(p_shock, baro_dt, ...
                cfg.Seeds.Baro, baro_noise_state, Baro_p);

            % MS5611 altitude formula (Pa -> m) per casper_sensor_params
            p_hpa = p_meas * 0.01;
            alt_m_baro = Baro_p.AltitudeCoeff * (1 - (p_hpa / Baro_p.SeaLevelRef_hPa)^Baro_p.AltitudeExponent);

            sensors_baro_press_pa(kb) = p_meas;
            sensors_baro_alt_m(kb)    = alt_m_baro;

            baro_new_for_ekf = true;
            baro_alt_for_ekf = alt_m_baro;
            next_baro_k = next_baro_k + 1;
        end

        % --- GPS sample (T06) -- logged only; no EKF GPS update in Phase 0 -
        if next_gps_k <= N_gps && idx_t >= gps_idx(next_gps_k)
            kg = next_gps_k;
            t_g = gps_t(kg);
            try
                [lat_d7, lon_d7, alt_mm, vn_mms, ve_mms, vd_mms, fix_t, n_sv, ~] = ...
                    casper_gps_step( ...
                        truth_run.pos_NED(idx_t,:).', truth_run.vel_NED(idx_t,:).', t_g, ...
                        GPS_local.LaunchLat_deg, GPS_local.LaunchLon_deg, GPS_local.LaunchAlt_m, ...
                        GPS_local.EarthRadius_m, ...
                        GPS_local.PositionSigmaHorizontal_m, GPS_local.PositionSigmaVertical_m, ...
                        GPS_local.VelocitySigma_mps, ...
                        GPS_local.COCOMVelThreshold_mps, GPS_local.COCOMAltThreshold_m, ...
                        GPS_local.ReacquireTime_s, GPS_local.Seed);
                sensors_gps_lat_deg7(kg)  = int32(lat_d7);
                sensors_gps_lon_deg7(kg)  = int32(lon_d7);
                sensors_gps_alt_mm(kg)    = int32(alt_mm);
                sensors_gps_vel_n_mms(kg) = int32(vn_mms);
                sensors_gps_vel_e_mms(kg) = int32(ve_mms);
                sensors_gps_vel_d_mms(kg) = int32(vd_mms);
                sensors_gps_fix(kg)       = uint8(fix_t);
                sensors_gps_sv(kg)        = uint8(n_sv);
            catch ME
                % Don't let GPS failure kill the run; log NaNs-equivalent zeros
                warning('casper_phase0_run:GPS', 'GPS step %d failed: %s', kg, ME.message);
            end
            next_gps_k = next_gps_k + 1;
        end

        % --- EKF (T08) ----------------------------------------------------
        % Initialize the EKF on the first baro sample AFTER attitude init has
        % completed. Initializing earlier corrupts accel_bias because the
        % attitude estimator is at identity quaternion before init, which
        % means body accel [0,+g,0] rotates to nav frame Z=0 and the EKF
        % integrates -1g down, biasing the state heavily before launch.
        if ~eskf_initialized && baro_new_for_ekf && att_out.init_complete
            eskf = casper_eskf_state('init', Estimator_, baro_alt_for_ekf);
            eskf_initialized = true;
        end

        if eskf_initialized
            % --- Rotate body accel to nav Zup (using attitude estimate)
            % a_nav = R(q) * a_body, then flip Z to get Zup
            qfw = att_out.quat_body_to_nav;
            a_nav_zup = rotate_quat_(qfw, a_fw_mps2);   % already in Zup since q is to-Zup

            % Trapezoidal accumulation across 2 IMU samples for the 416 Hz step
            accel_zup_prev = accel_zup_curr;
            accel_zup_curr = a_nav_zup(3);   % Z is up in firmware nav

            % EKF predict cadence: 416 Hz.  Trigger when elapsed time crosses
            % an ekf_dt boundary.  Use simple counter: predict every step
            % whose imu_t aligns to k*ekf_dt within imu_dt/2.
            ekf_time_accum = ekf_time_accum + imu_dt;
            do_predict = (ekf_time_accum >= ekf_dt - 1e-12);
            if do_predict
                ekf_time_accum = ekf_time_accum - ekf_dt;
                if ~isnan(accel_zup_prev)
                    a_avg = 0.5 * (accel_zup_prev + accel_zup_curr);
                else
                    a_avg = accel_zup_curr;
                end
                eskf = casper_eskf_predict(eskf, a_avg, Estimator_);
                last_ekf_time = t_now;
            end

            % Mach gate
            eskf = casper_eskf_mach_gate(eskf, truth_run.mach(idx_t), Estimator_);

            % ZUPT trigger (per architecture §7: truth velocity threshold)
            v_mag = norm(v_NED);
            if v_mag < Estimator_.ZuptThreshold
                eskf = casper_eskf_update_zupt(eskf, Estimator_);
                est_zupt_innov_mps(k_imu) = eskf.last_zupt_innov_mps;
                est_zupt_fired(k_imu) = true;
            end

            % Baro update when a new baro sample is available
            if baro_new_for_ekf
                eskf = casper_eskf_update_baro(eskf, baro_alt_for_ekf, Estimator_);
                kb = next_baro_k - 1;   % we just advanced it
                est_baro_innov_m(kb) = eskf.last_baro_innov_m;
                est_baro_innov_S(kb) = eskf.last_baro_innov_var;
                est_baro_used(kb)    = ~eskf.baro_update_was_skipped;
                est_baro_accepted(kb)= eskf.baro_update_was_accepted;
            end
        end

        % Log estimate state at IMU rate
        est_time_s(k_imu) = t_now;
        if eskf_initialized
            est_state_x(k_imu, :)      = eskf.x_vec(:).';
            est_state_P_diag(k_imu, :) = diag(eskf.P_mat).';
            est_mach_gate(k_imu)       = eskf.mach_gate_active;
            est_ungate_counter(k_imu)  = eskf.ungate_counter;
        end
    end

    runtime_s = toc(t_run_start);

    % --- Package outputs ----------------------------------------------------
    Truth = struct();
    Truth.time_s     = truth_run.time_s;
    Truth.pos_NED    = truth_run.pos_NED;
    Truth.vel_NED    = truth_run.vel_NED;
    Truth.accel_NED  = truth_run.accel_NED;
    Truth.quat_std   = truth_run.quat_std;
    Truth.omega_body_std = truth_run.omega_body_std;
    Truth.mach       = truth_run.mach;
    Truth.alt_m      = truth_run.alt_m;
    Truth.vel_v_mps  = truth_run.vel_v_mps;
    Truth.accel_v_mps2 = truth_run.accel_v_mps2;
    Truth.alt_agl_m  = truth_run.alt_m;            % alias for T10 metric API

    % Build truth quat in firmware frame at IMU times (for T10 attitude metric)
    Truth.quat_fw = zeros(size(truth_run.quat_std));
    for k = 1:size(truth_run.quat_std, 1)
        Truth.quat_fw(k, :) = casper_frame_switch_quat(truth_run.quat_std(k,:).').';
    end

    Estimate = struct();
    Estimate.time_s           = est_time_s;
    Estimate.state_x          = est_state_x;
    Estimate.state_P_diag     = est_state_P_diag;
    Estimate.quat_fw          = est_quat_fw;
    Estimate.mach_gate_active = est_mach_gate;
    Estimate.ungate_counter   = est_ungate_counter;
    Estimate.attitude_init_complete = est_attitude_init;
    % Innovations packaged in T10's expected sub-struct form
    Estimate.baro_innov = struct( ...
        'time_s', baro_t, ...
        'innov_m', est_baro_innov_m, ...
        'sigma_m', sqrt(max(est_baro_innov_S, 0)), ...
        'used',    est_baro_used, ...
        'rejected', est_baro_used & ~est_baro_accepted);
    Estimate.zupt_innov = struct( ...
        'time_s', imu_t, ...
        'innov_mps', est_zupt_innov_mps, ...
        'fired',     est_zupt_fired);

    Sensors = struct();
    Sensors.imu = struct( ...
        'time_s', imu_t, ...
        'accel_mps2', sensors_imu_accel_g * 9.80665, ...
        'gyro_radps', sensors_imu_gyro_dps * (pi/180), ...
        'accel_fw_mps2', sensors_imu_accel_fw_mps2, ...
        'gyro_fw_radps', sensors_imu_gyro_fw_radps, ...
        'temp_C',     sensors_imu_temp_C);
    Sensors.adxl = struct( ...
        'time_s', adxl_t, ...
        'accel_mps2', sensors_adxl_g * 9.80665);
    Sensors.baro = struct( ...
        'time_s', baro_t, ...
        'pressure_pa', sensors_baro_press_pa, ...
        'alt_m', sensors_baro_alt_m);
    Sensors.mag = struct( ...
        'time_s', mag_t, ...
        'uT', sensors_mag_uT, ...
        'uT_fw', sensors_mag_uT_fw, ...
        'raw_18bit', sensors_mag_raw18);
    Sensors.gps = struct( ...
        'time_s', gps_t, ...
        'lat_deg7', sensors_gps_lat_deg7, ...
        'lon_deg7', sensors_gps_lon_deg7, ...
        'alt_mm',   sensors_gps_alt_mm, ...
        'vel_n_mms', sensors_gps_vel_n_mms, ...
        'vel_e_mms', sensors_gps_vel_e_mms, ...
        'vel_d_mms', sensors_gps_vel_d_mms, ...
        'fix_type', sensors_gps_fix, ...
        'num_sv',   sensors_gps_sv);
    Sensors.gps.pos_m = double(sensors_gps_alt_mm) * 1e-3;   % placeholder for plots

    RadioTX = struct('time_s', mag_t, 'active', sensors_radio_active);

    % Determinism bundle: pack the raw sensor streams + estimator state into
    % a struct suitable for casper_data_hash.
    Determ = struct();
    Determ.sensor_streams = struct( ...
        'imu_accel_g',  sensors_imu_accel_g, ...
        'imu_gyro_dps', sensors_imu_gyro_dps, ...
        'baro_pa',      sensors_baro_press_pa, ...
        'mag_uT',       sensors_mag_uT, ...
        'gps_alt_mm',   sensors_gps_alt_mm, ...
        'radio_active', sensors_radio_active);
    Determ.estimate = struct( ...
        'state_x',  est_state_x, ...
        'P_diag',   est_state_P_diag, ...
        'quat_fw',  est_quat_fw, ...
        'mach_gate', est_mach_gate);

    out = struct( ...
        'Truth',     Truth, ...
        'Estimate',  Estimate, ...
        'Sensors',   Sensors, ...
        'RadioTX',   RadioTX, ...
        'Determ',    Determ, ...
        'runtime_s', runtime_s);

    fprintf('[T11] casper_phase0_run: wall %.2f s (N_imu=%d, N_baro=%d, N_mag=%d)\n', ...
        runtime_s, N_imu, N_baro, N_mag);
end

% =========================================================================
% Helpers
% =========================================================================

function sub = truncate_truth_(truth, N, pin)
%TRUNCATE_TRUTH_ Cut truth to N samples; optionally pin everything to t=0.
    sub = truth;
    field_2d = {'pos_NED','vel_NED','accel_NED','quat_std','omega_body_std'};
    field_1d = {'time_s','mach','air_density_kgm3','air_temp_K','air_pressure_pa', ...
                'alt_m','vel_v_mps','accel_v_mps2','pitch_deg','stage'};
    for k = 1:numel(field_2d)
        if isfield(sub, field_2d{k})
            x = sub.(field_2d{k});
            sub.(field_2d{k}) = x(1:N, :);
            if pin
                sub.(field_2d{k}) = repmat(x(1,:), N, 1);
            end
        end
    end
    for k = 1:numel(field_1d)
        if isfield(sub, field_1d{k})
            x = sub.(field_1d{k});
            sub.(field_1d{k}) = x(1:N);
            if pin && ~strcmp(field_1d{k}, 'time_s')
                sub.(field_1d{k}) = repmat(x(1), N, 1);
            end
        end
    end
    sub.n_samples = N;
end

function sub = prepend_pad_(truth, N_pad, dt)
%PREPEND_PAD_ Prepend N_pad samples of stationary pad data at t < 0.
    if N_pad <= 0
        sub = truth; return;
    end
    sub = truth;
    n_old = numel(truth.time_s);
    n_new = n_old + N_pad;

    new_t = ((-N_pad):(n_old - 1)).' * dt;

    field_2d = {'pos_NED','vel_NED','accel_NED','quat_std','omega_body_std'};
    field_1d = {'mach','air_density_kgm3','air_temp_K','air_pressure_pa', ...
                'alt_m','vel_v_mps','accel_v_mps2','pitch_deg','stage'};
    for k = 1:numel(field_2d)
        if isfield(truth, field_2d{k})
            x = truth.(field_2d{k});
            pad = repmat(x(1,:), N_pad, 1);
            sub.(field_2d{k}) = [pad; x];
        end
    end
    for k = 1:numel(field_1d)
        if isfield(truth, field_1d{k})
            x = truth.(field_1d{k});
            pad = repmat(x(1), N_pad, 1);
            sub.(field_1d{k}) = [pad; x];
        end
    end
    sub.time_s    = new_t;
    sub.n_samples = n_new;
end

function [idx_grid, t_grid] = sensor_indices_(t_truth, dt_sensor)
%SENSOR_INDICES_ Indices into the 10 kHz truth grid where sensor samples fire.
    t_start = t_truth(1);
    t_end   = t_truth(end);
    n = floor((t_end - t_start) / dt_sensor) + 1;
    t_grid = t_start + (0:n-1).' * dt_sensor;
    % Snap each sensor time to the nearest truth index (truth grid is exact 1e-4)
    idx_grid = round((t_grid - t_truth(1)) / (t_truth(2) - t_truth(1))) + 1;
    idx_grid(idx_grid < 1) = 1;
    idx_grid(idx_grid > numel(t_truth)) = numel(t_truth);
end

function v_nav = rotate_quat_(q, v_body)
%ROTATE_QUAT_ Hamilton scalar-first body-to-nav rotation: v_nav = R(q) * v_body.
    w = q(1); x = q(2); y = q(3); z = q(4);
    n2 = w*w + x*x + y*y + z*z;
    if n2 <= 0; n2 = 1; end
    s = 2.0 / n2;
    R = [ 1 - s*(y*y + z*z),  s*(x*y - z*w),       s*(x*z + y*w); ...
          s*(x*y + z*w),      1 - s*(x*x + z*z),   s*(y*z - x*w); ...
          s*(x*z - y*w),      s*(y*z + x*w),       1 - s*(x*x + y*y)];
    v_nav = R * v_body(:);
end

function mag_cal = apply_mag_cal_(mag_raw, hard_iron, soft_iron, axis_sign)
%APPLY_MAG_CAL_ Firmware-side mag_cal_apply: frame_mapped = sign.*raw,
%   cal = soft_iron * (frame_mapped - hard_iron).
    fm = axis_sign(:) .* mag_raw(:);
    mag_cal = soft_iron * (fm - hard_iron(:));
end
