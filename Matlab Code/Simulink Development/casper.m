function cfg = casper(varargin)
%CASPER  Top-level entry point for the C.A.S.P.E.R.-2 Simulink simulator.
%
% Usage:
%   casper()                          % default = 'apogee' profile (85 s)
%   casper('Profile', 'smoke')        % 5 s pad smoke
%   casper('Profile', 'apogee')       % 85 s ascent past apogee (default)
%   casper('Profile', 'full')         % 549 s full trajectory
%   casper('StopTime', 60.0)          % manual override of profile StopTime
%   casper('Seed', 20260520)
%   casper('Regenerate', true)        % force truth re-run from CSV
%   cfg = casper(...);                % returns the config struct
%
% Does (in order):
%   1. Adds every task folder under Matlab Code/Simulink Development/ to
%      the MATLAB path (shared/ + truth/ + params/ + sensors/{imu,baro,mag,gps}/
%      + nav/{frame_switch,attitude,eskf}/ + validation/ + integration/).
%   2. Calls casper_sim_config (sensor params + 6 unified Simulink.Bus
%      objects -> base workspace).
%   3. Applies visual-model overrides: GPS launch-site augmentation +
%      rate snap (IMU/ADXL 833 -> 1000 Hz, EKF 416 -> 500 Hz). Legacy
%      MATLAB driver path is unaffected.
%   4. Loads truth_trajectory.mat into base WS as truth_ts (struct of
%      timeseries). Regenerates from inputs/Flight_Test.CSV if missing
%      or if 'Regenerate', true.
%   5. Applies tuning-knob overrides (see TUNING SECTION below).
%   6. Prints a ready-to-go summary with next-step commands.
%
% After this returns, the base workspace has everything needed for:
%   open_system('casper_sim_phase0')   % visual inspection
%   sim('casper_sim_phase0')           % run
%   test_visual_model_compile          % automated 5 s smoke
%
% --- TUNING SECTION -------------------------------------------------------
% The struct `tuning` below is the single source for sim-side tuning knobs
% applied AFTER casper_sim_config and BEFORE the visual overrides. Edit
% any value here and re-run casper() to apply.
%
% Values feed into the matching base-workspace structs via:
%       <Struct>.<Field> = tuning.<Struct>_<Field>
%
% Only edit the lines below the "--- TUNING ---" banner inside the function
% body; everything else is plumbing. Leave a knob unset (= []) to keep the
% sensor-params default loaded by casper_sim_config.
%
% Most useful day-to-day tuning knobs:
%   IMU noise sigmas, baro Mach-shock magnitude, mag radio-interference
%   amplitude, GPS CEP, EKF process noise diag, EKF measurement noise
%   (R_baro, R_zupt), Mahony Kp_grav / Kp_mag_pad / Kp_mag_flight / Ki,
%   Mach-gate hysteresis.
% --------------------------------------------------------------------------

    p = inputParser();
    addParameter(p, 'Seed',       20260519, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'Profile',    'apogee', @(x) ischar(x) || isstring(x));   % 'smoke' | 'apogee' | 'full'
    addParameter(p, 'StopTime',   [],       @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
    addParameter(p, 'Regenerate', false,    @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Tuning',     '',       @(x) ischar(x) || isstring(x));   % preset hook ('', 'pad', 'flight')
    parse(p, varargin{:});

    fprintf('=== casper() top-level setup ===\n');

    % --- Resolve StopTime from Profile (with manual override) ------------
    profile_str = lower(strtrim(char(p.Results.Profile)));
    switch profile_str
        case 'smoke'
            profile_stop = 5.0;
        case 'apogee'
            profile_stop = 85.0;
        case 'full'
            profile_stop = 549.0;
        otherwise
            warning('casper:UnknownProfile', ...
                'Unknown profile "%s" (use "smoke", "apogee", or "full"). Falling back to apogee.', ...
                profile_str);
            profile_str = 'apogee';
            profile_stop = 85.0;
    end
    if ~isempty(p.Results.StopTime)
        stop_time_s = p.Results.StopTime;
        fprintf('[casper] profile=%s, StopTime override = %.1f s\n', profile_str, stop_time_s);
    else
        stop_time_s = profile_stop;
        fprintf('[casper] profile=%s, StopTime = %.1f s\n', profile_str, stop_time_s);
    end

    % --- 1. Paths --------------------------------------------------------
    simroot = fileparts(mfilename('fullpath'));
    add_simroot_paths_(simroot);

    % --- 2. Sensor params + buses (casper_sim_config) --------------------
    cfg = casper_sim_config('Seed', p.Results.Seed, 'StopTime', stop_time_s);

    % --- 3. Visual-model overrides (parallel to casper_setup_visual) -----
    % These are intentionally limited to the visual model path. The legacy
    % MATLAB driver (casper_phase0_run.m / run_phase0_trustgate.m) keeps
    % the firmware-canonical rates and its own GPS local-params helper.
    apply_gps_augmentation_();   % LaunchLat/Lon/Alt + ReacquireTime
    apply_rate_snap_();          % IMU 833->1000, ADXL 800->1000, EKF 416->500

    % --- 4. Truth timeseries ---------------------------------------------
    casper_load_truth_ts('Regenerate', p.Results.Regenerate, ...
                         'StopTime',   stop_time_s);

    % --- 5. Tuning overrides (single source of truth, edit in place) -----
    tuning = build_tuning_struct_();
    if ~isempty(p.Results.Tuning)
        preset = char(p.Results.Tuning);
        tuning = apply_preset_(tuning, preset);
    end
    apply_tuning_overrides_(tuning);

    % --- 6. Summary ------------------------------------------------------
    fprintf('\n=== ready to simulate ===\n');
    fprintf('  Profile  : %s\n', profile_str);
    fprintf('  Seed     : %u\n', cfg.Seed);
    fprintf('  StopTime : %.1f s\n', cfg.StopTime_s);
    fprintf('  Solver dt: %.0e s\n', cfg.SolverDt_s);
    fprintf('Next steps:\n');
    fprintf('  build_casper_sim_phase0                                %% (re)build .slx; saves StopTime=%.1f\n', stop_time_s);
    fprintf('  open_system(''casper_sim_phase0'')                      %% visual inspection\n');
    fprintf('  sim(''casper_sim_phase0'')                              %% run the simulation (saved StopTime)\n');
    fprintf('  test_visual_model_compile                              %% automated 5 s smoke\n');
    fprintf('  run_phase0_trustgate                                   %% canonical byte-exact regression\n');
    fprintf('\n');
    fprintf('After sim() completes, 8 scopes auto-open:\n');
    fprintf('  4 existing      : Altitude, VerticalVelocity, Euler, AttitudeError\n');
    fprintf('  4 NEW compare   : Compare_Altitude, Compare_VerticalVelocity,\n');
    fprintf('                    Compare_AttitudeError, Compare_Biases\n');
    fprintf('  cd integration; generate_ekf_comparison_report          %% side-by-side report\n');
end


% =========================================================================
function add_simroot_paths_(simroot)
% Add every functional subdir of Simulink Development to the MATLAB path.
    subdirs = { ...
        'shared', ...
        'truth', 'params', ...
        fullfile('sensors', 'imu'),  fullfile('sensors', 'baro'), ...
        fullfile('sensors', 'mag'),  fullfile('sensors', 'gps'), ...
        fullfile('nav', 'frame_switch'), fullfile('nav', 'eskf'), ...
        fullfile('nav', 'eskf16'), ...
        fullfile('nav', 'attitude'), ...
        'validation', 'integration'};
    for k = 1:numel(subdirs)
        d = fullfile(simroot, subdirs{k});
        if isfolder(d)
            addpath(d);
        end
    end
end


% =========================================================================
function apply_gps_augmentation_()
% Mirror of casper_setup_visual::apply_gps_augmentation_. The visual GPS
% block needs launch-site origin + reacquire-time fields directly on the
% GPS struct (the legacy driver uses casper_gps_local_params instead).
    GPSp = evalin('base', 'GPS');
    if ~isfield(GPSp, 'LaunchLat_deg'),   GPSp.LaunchLat_deg   = 51.5074;  end
    if ~isfield(GPSp, 'LaunchLon_deg'),   GPSp.LaunchLon_deg   = -0.1278;  end
    if ~isfield(GPSp, 'LaunchAlt_m'),     GPSp.LaunchAlt_m     = 35.0;     end
    if ~isfield(GPSp, 'ReacquireTime_s'), GPSp.ReacquireTime_s = 1.0;      end
    assignin('base', 'GPS', GPSp);
end


% =========================================================================
function apply_rate_snap_()
% Snap firmware-native sensor/estimator rates to clean integer multiples
% of the 1 kHz visual-model base step. casper_phase0_run.m keeps native rates.
    IMUp = evalin('base', 'IMU');
    if IMUp.Rate_Hz ~= 1000
        IMUp.Rate_Hz = 1000;
        assignin('base', 'IMU', IMUp);
    end

    ADXLp = evalin('base', 'ADXL');
    if ADXLp.RatePostLaunch_Hz ~= 1000
        ADXLp.RatePostLaunch_Hz = 1000;
        assignin('base', 'ADXL', ADXLp);
    end

    Estp = evalin('base', 'Estimator');
    if Estp.Dt ~= 2e-3
        Estp.Dt = 2e-3;
        assignin('base', 'Estimator', Estp);
    end
end


% =========================================================================
function tuning = build_tuning_struct_()
% Day-to-day tuning knobs. Leave a field = [] to keep the firmware default
% that casper_sim_config (via casper_sensor_params) loaded.
%
% Naming convention: <BaseWSStruct>_<Field> on the LHS maps to
% <BaseWSStruct>.<Field> in the base workspace (see apply_tuning_overrides_).

    tuning = struct();

    % --- IMU noise (LSM6DSO32) ------------------------------------------
    % Allan-variance bench numbers. White-noise sigmas for accel / gyro.
    tuning.IMU_AccelNoise_g       = [];     % e.g. 4.5e-4 (g, 1-sigma per axis)
    tuning.IMU_GyroNoise_dps      = [];     % e.g. 0.025  (dps, 1-sigma per axis)
    tuning.IMU_AccelBiasRW_g      = [];     % accel bias random-walk
    tuning.IMU_GyroBiasRW_dps     = [];     % gyro  bias random-walk

    % --- Barometer (MS5611) ---------------------------------------------
    % Mach-shock magnitude is the peak transient pressure offset (Pa) added
    % to the truth pressure inside the Mach window. Raise to stress baro
    % gating; lower for cleaner runs.
    tuning.Baro_NoiseStd_pa       = [];     % e.g. 6.0 Pa (1-sigma)
    tuning.Baro_MachShockMag_pa   = [];     % e.g. 200 Pa peak shock

    % --- Magnetometer (MMC5983MA + radio interference) -------------------
    % Radio interference amplitude is the rectangular-pulse magnitude
    % applied during the TX window (uT).
    tuning.Mag_NoiseStd_uT        = [];     % e.g. 0.5 uT
    tuning.Mag_RadioIntfAmp_uT    = [];     % e.g. 10.0 uT

    % --- GPS (MAX-M10M) -------------------------------------------------
    % CEP (Circular Error Probable, 50 percentile horizontal). Bigger CEP
    % degrades GPS-update quality in the ESKF.
    tuning.GPS_CEP_m              = [];     % e.g. 2.5 m

    % --- ESKF process / measurement noise --------------------------------
    % Q_diag is the 4-state process-noise diagonal (alt, vel, ab, bb).
    % R_baro is the baro innovation variance; R_zupt the ZUPT variance.
    tuning.Estimator_Q_diag       = [];     % e.g. [0 1e-4 1e-6 1e-8]
    tuning.Estimator_R_baro       = [];     % e.g. 0.5
    tuning.Estimator_R_zupt       = [];     % e.g. 6.15e-6
    tuning.Estimator_MachGateLo   = [];     % e.g. 0.35
    tuning.Estimator_MachGateHi   = [];     % e.g. 0.40

    % --- Mahony attitude gains ------------------------------------------
    % Kp_grav: gravity-correction gain (always on).
    % Kp_mag_pad: mag-correction gain while on pad (large).
    % Kp_mag_flight: mag-correction gain after launch (small).
    % Ki: bias-estimate integral gain.
    tuning.Attitude_Kp_grav       = [];
    tuning.Attitude_Kp_mag_pad    = [];
    tuning.Attitude_Kp_mag_flight = [];
    tuning.Attitude_Ki            = [];
end


% =========================================================================
function tuning = apply_preset_(tuning, preset)
% Optional preset hook. Extend as future presets crystallize.
    switch lower(strtrim(preset))
        case ''
            % no-op
        case 'pad'
            % Quieter sensors + larger Mahony mag gain (pad-friendly).
            tuning.IMU_AccelNoise_g    = 2.5e-4;
            tuning.IMU_GyroNoise_dps   = 0.015;
            tuning.Attitude_Kp_mag_pad = 0.50;
            fprintf('[casper] tuning preset: pad\n');
        case 'flight'
            % Nominal flight: keep firmware defaults except smaller mag gain.
            tuning.Attitude_Kp_mag_flight = 0.05;
            fprintf('[casper] tuning preset: flight\n');
        otherwise
            warning('casper:UnknownPreset', ...
                'Unknown tuning preset "%s" (use "pad", "flight", or "").', preset);
    end
end


% =========================================================================
function apply_tuning_overrides_(tuning)
% Walk the tuning struct; for each set field, override the matching field
% inside the corresponding base-workspace struct.
    fns = fieldnames(tuning);
    n_applied = 0;
    for k = 1:numel(fns)
        f = fns{k};
        v = tuning.(f);
        if isempty(v)
            continue;
        end
        us = find(f == '_', 1, 'first');
        if isempty(us)
            warning('casper:BadTuningField', ...
                'Tuning field "%s" missing <Struct>_<Field> separator; skipped.', f);
            continue;
        end
        struct_name = f(1:us-1);
        field_name  = f(us+1:end);
        if ~evalin('base', sprintf('exist(''%s'', ''var'')', struct_name))
            warning('casper:NoBaseStruct', ...
                'Base WS has no struct "%s" for tuning field "%s"; skipped.', struct_name, f);
            continue;
        end
        S = evalin('base', struct_name);
        S.(field_name) = v;
        assignin('base', struct_name, S);
        n_applied = n_applied + 1;
    end
    if n_applied > 0
        fprintf('[casper] applied %d tuning override(s).\n', n_applied);
    end
end
