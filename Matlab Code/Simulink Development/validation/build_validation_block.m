function model_path = build_validation_block()
%BUILD_VALIDATION_BLOCK Programmatically construct validation_block.slx
%
% Synopsis:
%   model_path = build_validation_block()
%
% Produces a logging-only Simulink subsystem at:
%   <this_dir>/validation_block.slx
%
% The block consumes the truth bus, estimator outputs, sensor traces, and
% radio TX status; it logs them to the base workspace via To Workspace blocks
% (SaveFormat = 'StructureWithTime'). Metrics are computed offline by
% casper_compute_all_metrics(). The block does not compute metrics in real
% time (T10 §10 anti-goal).
%
% CONSTRAINT: This script must NOT modify any other task's library file
% (e.g. T01's casper_sim_lib.slx). All blocks live in this task's own
% validation_block.slx.
%
% Source firmware reference: none (post-processing logger only).

    here = fileparts(mfilename('fullpath'));
    model_name = 'validation_block';
    model_path = fullfile(here, [model_name '.slx']);

    % Close any open instance, delete on-disk
    if bdIsLoaded(model_name)
        bdclose(model_name);
    end
    if isfile(model_path)
        delete(model_path);
    end

    % Create new system
    new_system(model_name, 'Model');
    open_system(model_name);

    % --- Top-level model parameters (ARCHITECTURE.md §2) ---
    set_param(model_name, ...
        'Solver',          'FixedStepDiscrete', ...
        'FixedStep',       '1e-4', ...
        'StartTime',       '0', ...
        'StopTime',        '1', ...
        'SaveOutput',      'on', ...
        'SaveFormat',      'StructureWithTime');

    sys = model_name;

    % --- Inports (signals into this subsystem) ---
    add_block('built-in/Inport',  [sys '/TruthBus_in']);            set_param([sys '/TruthBus_in'],  'Port', '1');
    add_block('built-in/Inport',  [sys '/Estimate_state_x']);       set_param([sys '/Estimate_state_x'], 'Port', '2');
    add_block('built-in/Inport',  [sys '/Estimate_state_P_diag']);  set_param([sys '/Estimate_state_P_diag'], 'Port', '3');
    add_block('built-in/Inport',  [sys '/Estimate_attitude_quat']); set_param([sys '/Estimate_attitude_quat'], 'Port', '4');
    add_block('built-in/Inport',  [sys '/Estimate_mach_gate']);     set_param([sys '/Estimate_mach_gate'], 'Port', '5');
    add_block('built-in/Inport',  [sys '/Estimate_ungate_counter']); set_param([sys '/Estimate_ungate_counter'], 'Port', '6');
    add_block('built-in/Inport',  [sys '/Estimate_baro_innov']);    set_param([sys '/Estimate_baro_innov'], 'Port', '7');
    add_block('built-in/Inport',  [sys '/Estimate_zupt_innov']);    set_param([sys '/Estimate_zupt_innov'], 'Port', '8');
    add_block('built-in/Inport',  [sys '/Sensor_IMU']);             set_param([sys '/Sensor_IMU'], 'Port', '9');
    add_block('built-in/Inport',  [sys '/Sensor_ADXL']);            set_param([sys '/Sensor_ADXL'], 'Port', '10');
    add_block('built-in/Inport',  [sys '/Sensor_Baro']);            set_param([sys '/Sensor_Baro'], 'Port', '11');
    add_block('built-in/Inport',  [sys '/Sensor_Mag']);             set_param([sys '/Sensor_Mag'], 'Port', '12');
    add_block('built-in/Inport',  [sys '/Sensor_GPS']);             set_param([sys '/Sensor_GPS'], 'Port', '13');
    add_block('built-in/Inport',  [sys '/RadioTX_active']);         set_param([sys '/RadioTX_active'], 'Port', '14');

    % --- To Workspace loggers for each signal ---
    add_log(sys, 'TruthBus_in',           'log_truth_bus');
    add_log(sys, 'Estimate_state_x',      'log_est_state_x');
    add_log(sys, 'Estimate_state_P_diag', 'log_est_state_P_diag');
    add_log(sys, 'Estimate_attitude_quat','log_est_quat');
    add_log(sys, 'Estimate_mach_gate',    'log_est_mach_gate_active');
    add_log(sys, 'Estimate_ungate_counter','log_est_ungate_counter');
    add_log(sys, 'Estimate_baro_innov',   'log_est_baro_innov');
    add_log(sys, 'Estimate_zupt_innov',   'log_est_zupt_innov');
    add_log(sys, 'Sensor_IMU',            'log_sensor_imu');
    add_log(sys, 'Sensor_ADXL',           'log_sensor_adxl');
    add_log(sys, 'Sensor_Baro',           'log_sensor_baro');
    add_log(sys, 'Sensor_Mag',            'log_sensor_mag');
    add_log(sys, 'Sensor_GPS',            'log_sensor_gps');
    add_log(sys, 'RadioTX_active',        'log_radio_tx_active');

    % --- Terminator blocks (these signals are also passed through unchanged
    %     for downstream consumers; not needed here since this is a leaf
    %     logger subsystem). To Workspace handles the leaf consumption.

    try
        Simulink.BlockDiagram.arrangeSystem(sys);
    catch
        % arrangeSystem can fail on some MATLAB versions; non-critical.
    end

    save_system(sys, model_path);
    bdclose(sys);

    fprintf('[T10] validation_block.slx built at %s\n', model_path);
end

function add_log(sys, inport_name, wsvar_name)
% Add a To Workspace block named after the inport and wire it.
    blk = [sys '/log_' inport_name];
    add_block('simulink/Sinks/To Workspace', blk);
    set_param(blk, ...
        'VariableName',   wsvar_name, ...
        'SaveFormat',     'StructureWithTime', ...
        'SampleTime',     '-1');
    add_line(sys, [inport_name '/1'], ['log_' inport_name '/1'], 'autorouting','smart');
end
