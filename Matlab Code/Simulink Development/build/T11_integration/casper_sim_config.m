function cfg = casper_sim_config(varargin)
%CASPER_SIM_CONFIG Phase 0 top-level simulator configuration.
%
% Synopsis:
%   cfg = casper_sim_config()             % default seed, full 549 s horizon
%   cfg = casper_sim_config('Seed', N)
%   cfg = casper_sim_config('StopTime', t_s)
%
% Loads casper_sensor_params.m (T02) into the caller workspace, builds
% TruthBus, and assembles a Sim runtime config struct.  Sub-agents and
% the trust-gate driver must call this *once* at the start of every sim
% session (replaces ad-hoc setup boilerplate).
%
% Output struct cfg:
%   .Seed                 (uint32) global Sim.Seed; canonical = 20260519
%   .StopTime_s           (double) sim stop time
%   .SolverDt_s           (double) fixed-step solver size (1e-4)
%   .Estimator_Dt_s       (double) EKF predict step (1/416)
%   .Attitude_Dt_s        (double) attitude step (1/833)
%   .Baro_Dt_s            (double) baro step (1/100)
%   .Mag_Dt_s             (double) mag step (1/100)
%   .GPS_Dt_s             (double) GPS step (1/10)
%   .Seeds  : struct with per-sensor seeds derived from .Seed
%       .IMU       = Seed + 1
%       .Baro      = Seed + 2
%       .Mag       = Seed + 3
%       .ADXL      = Seed + 4
%       .GPS       = Seed + 5
%       .RadioInt  = Seed + 7
%       .Estimator = Seed + 9   (reserved; ESKF currently deterministic)
%       .Attitude  = Seed + 10  (reserved; attitude currently deterministic)
%   .Horizons : struct
%       .SmokeStop_s = 5.0
%       .FullStop_s  = 549.0
%   .Plots_OutDir   (char)  default plots/ directory under T11
%   .Logs_OutDir    (char)  default logs/ directory under T11
%   .Data_OutDir    (char)  default data/ directory under T11
%
% Side effects:
%   - Adds T01..T11 build dirs and T02 to MATLAB path so all helper
%     functions (sensor models, EKF, attitude, frame switch, metrics)
%     resolve by name without further user setup.
%   - Loads casper_sensor_params (T02) into base workspace.
%   - Defines TruthBus in base workspace via casper_truth_build_bus().
%
% Source firmware reference: none (sim-side wiring).

    p = inputParser();
    addParameter(p, 'Seed',     20260519, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'StopTime', 549.0,     @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, varargin{:});
    seed = uint32(p.Results.Seed);
    stop_time = double(p.Results.StopTime);

    % --- Paths ---------------------------------------------------------------
    here = fileparts(mfilename('fullpath'));
    build_root = fileparts(here);           % .../build
    repo_root  = fileparts(fileparts(fileparts(build_root)));  %#ok<NASGU>

    task_dirs = { ...
        'T01_truth_pipeline', 'T02_sensor_params', 'T03_imu_sensor_model', ...
        'T04_baro_sensor_model', 'T05_mag_sensor_model', 'T06_gps_sensor_model', ...
        'T07_frame_switch', 'T08_eskf_port', 'T09_attitude_port', ...
        'T10_validation_block', 'T11_integration'};
    for k = 1:numel(task_dirs)
        d = fullfile(build_root, task_dirs{k});
        if isfolder(d)
            addpath(d);
        end
    end

    % --- Load sensor params into base workspace (T02) -----------------------
    % casper_sensor_params is a *script* (not a function) and populates the
    % caller workspace.  We run it in this function workspace, then assign
    % each struct into base.
    evalin('caller', '');  %#ok<EVLC> no-op guard
    casper_sensor_params;  %#ok<NASGU> populates Sim, IMU, ADXL, Baro, Mag, GPS,
                           %          Estimator, Attitude, GyroTempCal, Radio,
                           %          FSM, Telemetry, Validation in this scope.

    % Override Sim.Seed and propagate.
    Sim.Seed = double(seed);  %#ok<NODEF> Sim is defined by the script above

    % Build TruthBus and place in base workspace (required by sensor blocks
    % even when the trust-gate driver calls the MATLAB functions directly).
    truth_bus_obj = casper_truth_build_bus();

    base_vars = struct( ...
        'Sim',         Sim, ...
        'IMU',         IMU, ...
        'ADXL',        ADXL, ...
        'Baro',        Baro, ...
        'Mag',         Mag, ...
        'GPS',         GPS, ...
        'Estimator',   Estimator, ...
        'Attitude',    Attitude, ...
        'GyroTempCal', GyroTempCal, ...
        'Radio',       Radio, ...
        'FSM',         FSM, ...
        'Telemetry',   Telemetry, ...
        'Validation',  Validation, ...
        'TruthBus',    truth_bus_obj);
    fns = fieldnames(base_vars);
    for k = 1:numel(fns)
        assignin('base', fns{k}, base_vars.(fns{k}));
    end

    % --- Build cfg struct ----------------------------------------------------
    cfg = struct();
    cfg.Seed          = uint32(seed);
    cfg.StopTime_s    = stop_time;
    cfg.SolverDt_s    = Sim.SolverDt;
    cfg.Estimator_Dt_s = Estimator.Dt;          % 1/416
    cfg.Attitude_Dt_s  = 1.0 / IMU.Rate_Hz;     % 1/833
    cfg.Baro_Dt_s      = 1.0 / Baro.Rate_Hz;    % 1/100
    cfg.Mag_Dt_s       = 1.0 / Mag.Rate_Hz;     % 1/100
    cfg.GPS_Dt_s       = 1.0 / GPS.Rate_Hz;     % 1/10

    cfg.Seeds = struct( ...
        'IMU',       double(seed) + 1, ...
        'Baro',      double(seed) + 2, ...
        'Mag',       double(seed) + 3, ...
        'ADXL',      double(seed) + 4, ...
        'GPS',       double(seed) + 5, ...
        'RadioInt',  double(seed) + 7, ...
        'Estimator', double(seed) + 9, ...
        'Attitude',  double(seed) + 10);

    cfg.Horizons = struct( ...
        'SmokeStop_s', 5.0, ...
        'FullStop_s',  549.0);

    cfg.Plots_OutDir = fullfile(here, 'plots');
    cfg.Logs_OutDir  = fullfile(here, 'logs');
    cfg.Data_OutDir  = fullfile(here, 'data');
    for d = {cfg.Plots_OutDir, cfg.Logs_OutDir, cfg.Data_OutDir}
        if ~isfolder(d{1}); mkdir(d{1}); end
    end

    assignin('base', 'SimCfg', cfg);

    fprintf('[T11] casper_sim_config: Seed=%u, StopTime=%.1f s\n', seed, stop_time);
end
