function lib_path = build_imu_block_visual(varargin)
%BUILD_IMU_BLOCK_VISUAL Construct the visual T03 IMU library using imuSensor.
%
% Synopsis:
%   lib_path = build_imu_block_visual()
%   lib_path = build_imu_block_visual('LibPath', '/abs/path/imu_block_visual.slx')
%
% Builds imu_block_visual.slx containing two named subsystems:
%   - lsm_visual_block (truth_bus -> accel_g_body_std, gyro_dps_body_std,
%                                    temp_C, data_ready)
%   - adxl_visual_block (truth_bus -> accel_g_body_std, fifo_active)
%
% Each subsystem wires SensorInputBus through a Bus Selector, Rate
% Transitions, a MATLAB Function block that calls imuSensor (Sensor Fusion
% Tbx) via casper_imu_lsm_step / casper_imu_adxl_step, and a final MATLAB
% Function block that applies the firmware quirks (unit convert / LSB
% quantize / saturation / FIFO latch).
%
% Returns:
%   lib_path : char absolute path to the .slx library.
%
% Source references:
%   - ARCHITECTURE.md §5 (stock-block intent)
%   - casper_imu_lsm_step.m, casper_imu_lsm_quirks.m, casper_imu_adxl_quirks.m
%   - casper_build_unified_buses.m (SensorInputBus / IMUOutputBus defs)

    p = inputParser();
    addParameter(p, 'LibPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    lib_path = char(p.Results.LibPath);

    here = fileparts(mfilename('fullpath'));
    if isempty(lib_path)
        lib_path = fullfile(here, 'imu_block_visual.slx');
    end
    lib_path = char(java.io.File(lib_path).getCanonicalPath());

    lib_name = 'imu_block_visual';

    % Path setup: T03 build dir + T11 (for casper_sim_config) + T01 (for
    % TruthBus helper). Also load configs into base workspace so the
    % Constant blocks' parameter references (Sim.Seed, IMU.AccelScale...)
    % resolve when the model is loaded.
    addpath(here);
    addpath(fullfile(here, '..', 'T11_integration'));
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    addpath(fullfile(here, '..', 'T02_sensor_params'));

    % Ensure base workspace has Sim, IMU, ADXL, IMU_T03, TruthBus, and
    % the unified buses. casper_sim_config does all of that.
    casper_sim_config('Seed', 20260519, 'StopTime', 5.0);

    if bdIsLoaded(lib_name)
        close_system(lib_name, 0);
    end
    if isfile(lib_path)
        delete(lib_path);
    end
    new_system(lib_name, 'Library');
    load_system(lib_name);
    set_param(lib_name, 'Lock', 'off');

    build_lsm_visual_subsystem_(lib_name);
    build_adxl_visual_subsystem_(lib_name);

    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, lib_path);
    close_system(lib_name, 0);

    fprintf('[build_imu_block_visual] wrote %s\n', lib_path);
end


% =====================================================================
function build_lsm_visual_subsystem_(lib_name)
% LSM6DSO32 visual subsystem: truth_bus -> imuSensor -> quirks -> outputs.

    sub = [lib_name '/lsm_visual_block'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [80 60 600 380]);

    % Inport: truth_bus
    in = [sub '/truth_bus'];
    add_block('built-in/Inport', in);
    set_param(in, 'Position', [20 130 50 150], ...
        'OutDataTypeStr', 'Bus: SensorInputBus', 'BusObject', 'SensorInputBus');

    % Bus Selector
    bs = [sub '/BusSelect'];
    add_block('simulink/Signal Routing/Bus Selector', bs);
    set_param(bs, 'OutputSignals', 'accel_NED,quat_std,omega_body_std');
    set_param(bs, 'Position', [90 100 130 180]);
    add_line(sub, 'truth_bus/1', 'BusSelect/1', 'autorouting', 'on');

    % Rate Transitions to 1/833 Hz
    rt_dt = '1/IMU.Rate_Hz';
    rt_names = {'RT_accel', 'RT_quat', 'RT_omega'};
    for k = 1:3
        rp = [sub '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, 'OutPortSampleTime', rt_dt, ...
            'Position', [170 (90 + (k-1)*40) 220 (110 + (k-1)*40)]);
        add_line(sub, ['BusSelect/' num2str(k)], [rt_names{k} '/1'], 'autorouting', 'on');
    end

    % MATLAB Function: imuSensor step wrapper
    mf = [sub '/imuSensor_LSM_step'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf);
    set_param(mf, 'Position', [260 60 420 200]);
    set_matlab_fn_script_(mf, [ ...
        'function [a_mps2_body, g_radps_body] = fcn(' ...
            'accel_NED, quat_std, omega_body_std, seed_base, reset_flag)' newline ...
        'coder.extrinsic(''casper_imu_lsm_step'');' newline ...
        'a_mps2_body = zeros(1,3); g_radps_body = zeros(1,3);' newline ...
        '[a_mps2_body, g_radps_body, ~] = casper_imu_lsm_step(' ...
            'accel_NED(:).'', omega_body_std(:).'', quat_std(:).'', ' ...
            'double(seed_base), logical(reset_flag));' newline ...
        'end' newline]);

    add_line(sub, 'RT_accel/1', 'imuSensor_LSM_step/1', 'autorouting', 'on');
    add_line(sub, 'RT_quat/1',  'imuSensor_LSM_step/2', 'autorouting', 'on');
    add_line(sub, 'RT_omega/1', 'imuSensor_LSM_step/3', 'autorouting', 'on');

    % Constants for seed and reset
    c_seed = [sub '/C_Seed'];
    add_block('simulink/Sources/Constant', c_seed);
    set_param(c_seed, 'Value', 'Sim.Seed + 1', 'SampleTime', rt_dt, ...
        'Position', [170 220 220 240]);
    add_line(sub, 'C_Seed/1', 'imuSensor_LSM_step/4', 'autorouting', 'on');

    c_reset = [sub '/C_Reset'];
    add_block('simulink/Sources/Constant', c_reset);
    set_param(c_reset, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [170 250 220 270]);
    add_line(sub, 'C_Reset/1', 'imuSensor_LSM_step/5', 'autorouting', 'on');

    % MATLAB Function: PostQuirks (unit convert + quantize + saturate)
    pq = [sub '/PostQuirks'];
    add_block('simulink/User-Defined Functions/MATLAB Function', pq);
    set_param(pq, 'Position', [460 60 620 200]);
    set_matlab_fn_script_(pq, [ ...
        'function [accel_g, gyro_dps] = fcn(' ...
            'a_mps2, g_radps, a_lsb, g_lsb, a_range, g_range)' newline ...
        '%#codegen' newline ...
        '[accel_g, gyro_dps] = casper_imu_lsm_quirks(' ...
            'a_mps2(:), g_radps(:), a_lsb, g_lsb, a_range, g_range);' newline ...
        'end' newline]);

    add_line(sub, 'imuSensor_LSM_step/1', 'PostQuirks/1', 'autorouting', 'on');
    add_line(sub, 'imuSensor_LSM_step/2', 'PostQuirks/2', 'autorouting', 'on');

    % Quirks constants
    quirk_consts = { ...
        {'C_AccelLSB',   'IMU.AccelScale_gPerLSB',     3}, ...
        {'C_GyroLSB',    'IMU.GyroScale_dpsPerLSB',    4}, ...
        {'C_AccelRange', 'IMU.AccelRange_g',           5}, ...
        {'C_GyroRange',  'IMU.GyroRange_dps',          6} };
    y0 = 290;
    for k = 1:numel(quirk_consts)
        nm = quirk_consts{k}{1};
        val = quirk_consts{k}{2};
        port = quirk_consts{k}{3};
        cp = [sub '/' nm];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
            'Position', [310 (y0 + (k-1)*25) 380 (y0 + (k-1)*25 + 20)]);
        add_line(sub, [nm '/1'], ['PostQuirks/' num2str(port)], 'autorouting', 'on');
    end

    % Temperature + data_ready constants
    c_temp = [sub '/C_Temp'];
    add_block('simulink/Sources/Constant', c_temp);
    set_param(c_temp, 'Value', '25.0', 'SampleTime', rt_dt, ...
        'Position', [460 230 510 250]);

    c_dr = [sub '/C_DataReady'];
    add_block('simulink/Sources/Constant', c_dr);
    set_param(c_dr, 'Value', 'true', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [460 260 510 280]);

    % Outports
    outs = { ...
        {'accel_g_body_std',  'PostQuirks/1'}, ...
        {'gyro_dps_body_std', 'PostQuirks/2'}, ...
        {'temp_C',            'C_Temp/1'}, ...
        {'data_ready',        'C_DataReady/1'} };
    for k = 1:numel(outs)
        op = [sub '/' outs{k}{1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [660 (60 + (k-1)*45) 690 (80 + (k-1)*45)]);
        add_line(sub, outs{k}{2}, [outs{k}{1} '/1'], 'autorouting', 'on');
    end
end


% =====================================================================
function build_adxl_visual_subsystem_(lib_name)
% ADXL372 visual subsystem: truth_bus -> imuSensor (high-g) -> quirks
% (LPF + quantize + saturate + FIFO latch) -> outputs.

    sub = [lib_name '/adxl_visual_block'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [80 420 600 720]);

    in = [sub '/truth_bus'];
    add_block('built-in/Inport', in);
    set_param(in, 'Position', [20 130 50 150], ...
        'OutDataTypeStr', 'Bus: SensorInputBus', 'BusObject', 'SensorInputBus');

    bs = [sub '/BusSelect'];
    add_block('simulink/Signal Routing/Bus Selector', bs);
    set_param(bs, 'OutputSignals', 'accel_NED,quat_std,omega_body_std,pos_NED');
    set_param(bs, 'Position', [90 90 130 200]);
    add_line(sub, 'truth_bus/1', 'BusSelect/1', 'autorouting', 'on');

    rt_dt = '1/ADXL.RatePostLaunch_Hz';
    rt_names = {'RT_accel', 'RT_quat', 'RT_omega', 'RT_pos'};
    for k = 1:4
        rp = [sub '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, 'OutPortSampleTime', rt_dt, ...
            'Position', [170 (80 + (k-1)*40) 220 (100 + (k-1)*40)]);
        add_line(sub, ['BusSelect/' num2str(k)], [rt_names{k} '/1'], 'autorouting', 'on');
    end

    % Altitude scalar (alt = -pos_NED(3))
    sel = [sub '/AltSelect'];
    add_block('simulink/Signal Routing/Selector', sel);
    set_param(sel, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', ...
        'Indices', '3', 'InputPortWidth', '3', ...
        'Position', [240 200 280 220]);
    add_line(sub, 'RT_pos/1', 'AltSelect/1', 'autorouting', 'on');

    gain = [sub '/NegGain'];
    add_block('simulink/Math Operations/Gain', gain);
    set_param(gain, 'Gain', '-1', 'Position', [300 200 340 220]);
    add_line(sub, 'AltSelect/1', 'NegGain/1', 'autorouting', 'on');

    % imuSensor wrapper (ADXL: uses Sim.Seed+4)
    mf = [sub '/imuSensor_ADXL_step'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf);
    set_param(mf, 'Position', [260 60 420 200]);
    set_matlab_fn_script_(mf, [ ...
        'function a_mps2_body = fcn(' ...
            'accel_NED, quat_std, omega_body_std, seed_base, reset_flag)' newline ...
        'coder.extrinsic(''casper_imu_adxl_step'');' newline ...
        'a_mps2_body = zeros(1,3);' newline ...
        'a_mps2_body = casper_imu_adxl_step(' ...
            'accel_NED(:).'', omega_body_std(:).'', quat_std(:).'', ' ...
            'double(seed_base), logical(reset_flag));' newline ...
        'end' newline]);

    add_line(sub, 'RT_accel/1', 'imuSensor_ADXL_step/1', 'autorouting', 'on');
    add_line(sub, 'RT_quat/1',  'imuSensor_ADXL_step/2', 'autorouting', 'on');
    add_line(sub, 'RT_omega/1', 'imuSensor_ADXL_step/3', 'autorouting', 'on');

    c_seed = [sub '/C_Seed'];
    add_block('simulink/Sources/Constant', c_seed);
    set_param(c_seed, 'Value', 'Sim.Seed + 4', 'SampleTime', rt_dt, ...
        'Position', [170 250 220 270]);
    add_line(sub, 'C_Seed/1', 'imuSensor_ADXL_step/4', 'autorouting', 'on');

    c_reset = [sub '/C_Reset'];
    add_block('simulink/Sources/Constant', c_reset);
    set_param(c_reset, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [170 280 220 300]);
    add_line(sub, 'C_Reset/1', 'imuSensor_ADXL_step/5', 'autorouting', 'on');

    % PostQuirks (LPF + quantize + saturate + FIFO latch)
    pq = [sub '/PostQuirks'];
    add_block('simulink/User-Defined Functions/MATLAB Function', pq);
    set_param(pq, 'Position', [460 60 620 240]);
    set_matlab_fn_script_(pq, [ ...
        'function [accel_g, fifo_active] = fcn(' ...
            'a_mps2, alt_m, lpf_prev, dt, bw_hz, lsb_g, range_g, ' ...
            'fifo_prev, fifo_thresh)' newline ...
        '%#codegen' newline ...
        '[accel_g, ~, fifo_active] = casper_imu_adxl_quirks(' ...
            'a_mps2(:), lpf_prev(:), alt_m, dt, bw_hz, lsb_g, range_g, ' ...
            'logical(fifo_prev), fifo_thresh);' newline ...
        'end' newline]);

    add_line(sub, 'imuSensor_ADXL_step/1', 'PostQuirks/1', 'autorouting', 'on');
    add_line(sub, 'NegGain/1',              'PostQuirks/2', 'autorouting', 'on');

    % LPF previous state (Memory block to break the loop on lpf_prev)
    mem_lpf = [sub '/Mem_LPF'];
    add_block('simulink/Discrete/Memory', mem_lpf);
    set_param(mem_lpf, 'InitialCondition', '[0; 0; 0]', ...
        'Position', [460 260 510 280]);
    add_line(sub, 'Mem_LPF/1', 'PostQuirks/3', 'autorouting', 'on');
    % The quirks function doesn't expose lpf_state_out via the PostQuirks
    % MATLAB-Function signature above (we discard it). For a true persistent
    % LPF you'd need to add an outport and feed it back into Mem_LPF — left
    % as a follow-up; PoC just exercises the wiring at this level.

    quirk_consts = { ...
        {'C_dt',       '1/ADXL.RatePostLaunch_Hz',     4}, ...
        {'C_bw',       'ADXL.BandwidthPostLaunch_Hz',  5}, ...
        {'C_lsb',      'ADXL.Scale_gPerLSB',           6}, ...
        {'C_range',    'ADXL.Range_g',                 7}, ...
        {'C_FifoPrev', 'false',                        8}, ...
        {'C_FifoThr',  '5.0',                          9} };
    y0 = 310;
    for k = 1:numel(quirk_consts)
        nm = quirk_consts{k}{1};
        val = quirk_consts{k}{2};
        port = quirk_consts{k}{3};
        cp = [sub '/' nm];
        add_block('simulink/Sources/Constant', cp);
        is_bool = strcmp(val, 'true') || strcmp(val, 'false');
        if is_bool
            set_param(cp, 'Value', val, 'OutDataTypeStr', 'boolean', ...
                'SampleTime', rt_dt, ...
                'Position', [310 (y0 + (k-1)*25) 380 (y0 + (k-1)*25 + 20)]);
        else
            set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
                'Position', [310 (y0 + (k-1)*25) 380 (y0 + (k-1)*25 + 20)]);
        end
        add_line(sub, [nm '/1'], ['PostQuirks/' num2str(port)], 'autorouting', 'on');
    end

    % Outports
    outs = { ...
        {'accel_g_body_std', 'PostQuirks/1'}, ...
        {'fifo_active',      'PostQuirks/2'} };
    for k = 1:numel(outs)
        op = [sub '/' outs{k}{1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [660 (60 + (k-1)*45) 690 (80 + (k-1)*45)]);
        add_line(sub, outs{k}{2}, [outs{k}{1} '/1'], 'autorouting', 'on');
    end
end


% =====================================================================
function set_matlab_fn_script_(block_path, src)
% SET_MATLAB_FN_SCRIPT_ Install the script body of a MATLAB Function block.
    sf = sfroot;
    blk = sf.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(blk)
        error('build_imu_block_visual:NoChart', ...
            'Could not find MATLAB Function chart at %s', block_path);
    end
    blk.Script = src;
end
