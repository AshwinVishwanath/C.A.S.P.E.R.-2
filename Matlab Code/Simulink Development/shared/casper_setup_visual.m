function casper_setup_visual(varargin)
%CASPER_SETUP_VISUAL One-stop setup before opening / simulating the visual model.
%
% Synopsis:
%   casper_setup_visual()                            % default seed, 5 s smoke
%   casper_setup_visual('StopTime', 549.0)           % full trajectory
%   casper_setup_visual('Seed', 20260520)            % different seed
%   casper_setup_visual('Regenerate', true)          % force re-run T01
%
% Behaviour (in order):
%   1. Calls casper_sim_config — populates Sim, IMU, ADXL, IMU_T03, Baro,
%      Mag, GPS, Estimator, Attitude structs + all 6 unified Simulink.Bus
%      objects (SensorInputBus, IMUOutputBus, BaroOutputBus, MagOutputBus,
%      GPSOutputBus, EstimateBus) into base workspace.
%   2. Calls casper_load_truth_ts — populates truth_ts (struct of
%      timeseries) into base workspace. Cached on disk; regenerates if
%      missing or if 'Regenerate', true.
%   3. Prints next-step commands the user should run.
%
% After this returns the base workspace has everything the visual model
% needs:
%   - Bus objects for Bus Creator / Bus Selector resolution
%   - Parameter structs for Constant block references (Sim.Seed, IMU.Rate_Hz, ...)
%   - truth_ts for the From-Workspace blocks
%
% Then either:
%   open_system('casper_sim_phase0')    % visual inspection
%   sim('casper_sim_phase0')            % run the simulation
%
% Or use the convenience helper that does both:
%   test_visual_model_compile            % builds, opens, runs 5 s smoke

    p = inputParser();
    addParameter(p, 'Seed',       20260519, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'StopTime',   5.0,      @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'Regenerate', false,    @(x) islogical(x) || isnumeric(x));
    parse(p, varargin{:});

    fprintf('=== casper_setup_visual ===\n');
    cfg = casper_sim_config('Seed', p.Results.Seed, 'StopTime', p.Results.StopTime); %#ok<NASGU>

    % Visual-only overrides — these change base-WS values that the visual
    % model relies on, but they intentionally do NOT touch the legacy
    % casper_phase0_run.m / run_phase0_trustgate.m path (which keep
    % firmware-canonical rates and use casper_gps_local_params instead).
    apply_gps_augmentation_();   % LaunchLat/Lon/Alt + ReacquireTime
    apply_rate_snap_();          % IMU 833->1000, ADXL 800->1000, EKF 416->500

    casper_load_truth_ts('Regenerate', p.Results.Regenerate, ...
                         'StopTime',   p.Results.StopTime);

    fprintf('\n=== ready to simulate ===\n');
    fprintf('Inspect:   open_system(''casper_sim_phase0'')\n');
    fprintf('Build:     build_casper_sim_phase0  (rebuilds the .slx if you edited the builder)\n');
    fprintf('Simulate:  sim(''casper_sim_phase0'')\n');
    fprintf('Or smoke:  test_visual_model_compile\n');
end


% =========================================================================
function apply_gps_augmentation_()
% Mirror of T06 build_gps_block_visual::augment_gps_struct_().
% The visual GPS block needs launch-site origin + reacquire-time fields on
% the GPS struct itself (rather than a separate GPS_local). The legacy
% MATLAB driver uses casper_gps_local_params instead and does NOT need this.
    GPSp = evalin('base', 'GPS');
    if ~isfield(GPSp, 'LaunchLat_deg'),   GPSp.LaunchLat_deg   = 51.5074;  end
    if ~isfield(GPSp, 'LaunchLon_deg'),   GPSp.LaunchLon_deg   = -0.1278;  end
    if ~isfield(GPSp, 'LaunchAlt_m'),     GPSp.LaunchAlt_m     = 35.0;     end
    if ~isfield(GPSp, 'ReacquireTime_s'), GPSp.ReacquireTime_s = 1.0;      end
    assignin('base', 'GPS', GPSp);
end


% =========================================================================
function apply_rate_snap_()
% Mirror of build_casper_sim_phase0::snap_rates_for_visual_model_().
% Snap firmware-native sensor / estimator rates to clean integer multiples
% of the 1 kHz visual-model base step. Required because the visual .slx's
% Rate Transition / Constant blocks reference symbolic IMU.Rate_Hz,
% ADXL.RatePostLaunch_Hz, Estimator.Dt — and those base-WS values get
% reset to their firmware-canonical values by every casper_sim_config call.
%
% This snap is visual-only. casper_phase0_run.m keeps the native rates.
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
