function lib_path = build_imu_block(varargin)
%BUILD_IMU_BLOCK Construct LSM6DSO32 and ADXL372 sensor subsystems.
%
% Synopsis:
%   lib_path = build_imu_block()
%   lib_path = build_imu_block('LibPath', '/abs/path/casper_sim_lib.slx')
%
% Adds two subsystems to the T01 sim library casper_sim_lib.slx:
%   - imu_lsm_block  (truth_bus in -> accel_g_body_std, gyro_dps_body_std,
%                                    temp_C, data_ready)
%   - imu_adxl_block (truth_bus in -> accel_g_body_std, fifo_active)
%
% The subsystems are rate-paced via Rate Transition blocks. Sample times:
%   - LSM6DSO32  : 1 / IMU.Rate_Hz (833 Hz)
%   - ADXL372    : 1 / ADXL.RatePostLaunch_Hz (800 Hz)   -- Phase 0 keeps
%     a fixed sensor sample rate at 800 Hz; the FIFO/active flag still
%     exposes the pre/post-launch state, but rate switching is deferred
%     to a downstream gating block per T07 (the spec allows either a
%     switched subsystem or a Stateflow chart).
%
% Returns:
%   lib_path : char, absolute path to the .slx library.
%
% Source firmware references:
%   - Software/App/drivers/lsm6dso32.c, adxl372.c
%   - references/FIRMWARE_CONSTANTS.md §5.1, §5.2.

    p = inputParser();
    addParameter(p, 'LibPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    lib_path = char(p.Results.LibPath);

    here = fileparts(mfilename('fullpath'));

    if isempty(lib_path)
        % Default: write our own library inside the T03 build dir, so we
        % don't modify any other task's directory (hard constraint).
        % Manager / T11 integration can incorporate this library directly.
        lib_path = fullfile(here, 'imu_block.slx');
    end

    lib_path = char(java.io.File(lib_path).getCanonicalPath());

    lib_name = 'imu_block';

    % --- Make sure the imu dir is on the MATLAB path so MATLAB Function
    % blocks can resolve casper_imu_lsm_model / *_noise / etc. ---
    addpath(here);

    % --- Ensure TruthBus exists in base workspace ---
    simroot   = fileparts(fileparts(here));            % .../Simulink Development
    truth_dir = fullfile(simroot, 'truth');
    if exist(fullfile(truth_dir, 'casper_truth_build_bus.m'), 'file')
        addpath(truth_dir);
        bus_obj = casper_truth_build_bus();
        assignin('base', 'TruthBus', bus_obj);
    end

    % --- Close lib if open ---
    if bdIsLoaded(lib_name)
        close_system(lib_name, 0);
    end

    % --- Open or create the library ---
    if isfile(lib_path)
        load_system(lib_path);
    else
        new_system(lib_name, 'Library');
        load_system(lib_name);
    end
    set_param(lib_name, 'Lock', 'off');

    % Wipe any prior versions so this script is idempotent.
    for sub = {'imu_lsm_block', 'imu_adxl_block'}
        full = [lib_name '/' sub{1}];
        if getSimulinkBlockHandle(full) ~= -1
            delete_block(full);
        end
    end

    % --- LSM6DSO32 subsystem ---
    build_lsm_subsystem(lib_name);

    % --- ADXL372 subsystem ---
    build_adxl_subsystem(lib_name);

    % Lock & save.
    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, lib_path);
    close_system(lib_name, 0);

    fprintf('[build_imu_block] wrote %s\n', lib_path);
end


% =====================================================================
function build_lsm_subsystem(lib_name)
%BUILD_LSM_SUBSYSTEM Construct imu_lsm_block under <lib_name>/.

    sub_path = [lib_name '/imu_lsm_block'];
    add_block('built-in/Subsystem', sub_path);
    set_param(sub_path, 'Position', [80 60 560 360]);

    % --- Inport: truth_bus ---
    in_path = [sub_path '/truth_bus'];
    add_block('built-in/Inport', in_path);
    set_param(in_path, 'Position', [20 130 50 150], ...
        'OutDataTypeStr', 'Bus: TruthBus', 'BusObject', 'TruthBus');

    % --- Bus Selector: extract accel_NED, quat_std, omega_body_std ---
    bs_path = [sub_path '/BusSelect'];
    add_block('simulink/Signal Routing/Bus Selector', bs_path);
    set_param(bs_path, 'OutputSignals', 'accel_NED,quat_std,omega_body_std');
    set_param(bs_path, 'Position', [90 100 130 180]);

    % --- Rate Transition: solver-rate -> 1/833 ---
    % Down-sample each selected line at the LSM rate.
    rt_dt = '1/833';
    rt_names = {'RT_accel', 'RT_quat', 'RT_omega'};
    for k = 1:3
        rp = [sub_path '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, 'OutPortSampleTime', rt_dt);
        set_param(rp, 'Position', [170 (90 + (k-1)*40) 220 (110 + (k-1)*40)]);
        add_line(sub_path, ['BusSelect/' num2str(k)], [rt_names{k} '/1'], 'autorouting', 'on');
    end

    % --- MATLAB Function: clean LSM model ---
    mdl_path = [sub_path '/lsm_model'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mdl_path);
    set_param(mdl_path, 'Position', [260 80 360 200]);
    set_lsm_model_script(mdl_path);

    % wire model inputs
    add_line(sub_path, 'RT_accel/1', 'lsm_model/1', 'autorouting', 'on');
    add_line(sub_path, 'RT_quat/1',  'lsm_model/2', 'autorouting', 'on');
    add_line(sub_path, 'RT_omega/1', 'lsm_model/3', 'autorouting', 'on');

    % --- MATLAB Function: noise ---
    noi_path = [sub_path '/lsm_noise'];
    add_block('simulink/User-Defined Functions/MATLAB Function', noi_path);
    set_param(noi_path, 'Position', [400 60 540 240]);
    set_lsm_noise_script(noi_path);

    % --- Reset pulse: constant 'false' (single-shot reset is wired via
    %     subsystem Initialize callback by clearing persistents; here the
    %     per-sample reset flag is just 'false'). ---
    rst_path = [sub_path '/ResetFalse'];
    add_block('simulink/Sources/Constant', rst_path);
    set_param(rst_path, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [260 220 290 240]);

    % Wire clean-model outputs into noise function:
    %   noise inputs (per casper_imu_lsm_noise signature):
    %   1 accel_g_in    (from lsm_model/1)
    %   2 gyro_dps_in   (from lsm_model/2)
    %   3 dt            (Constant)
    %   4 seed_base
    %   5..14 parameter constants
    %   15 reset_flag
    %
    % Connect first two from lsm_model.
    add_line(sub_path, 'lsm_model/1', 'lsm_noise/1', 'autorouting', 'on');
    add_line(sub_path, 'lsm_model/2', 'lsm_noise/2', 'autorouting', 'on');

    % Constants for the remaining noise inputs ---------------------------
    const_specs = {
        % name,                value,                          port
        {'C_dt',               '1/IMU.Rate_Hz',                3 }, ...
        {'C_seed',             'Sim.Seed + 1',                 4 }, ...
        {'C_abi',              'IMU_T03.AccelBiasInit_g',      5 }, ...
        {'C_gbi',              'IMU_T03.GyroBiasInit_dps',     6 }, ...
        {'C_cax',              'IMU_T03.CrossAxis_deg',        7 }, ...
        {'C_sfp',              'IMU_T03.ScaleFactor_ppm',      8 }, ...
        {'C_vrw',              'Estimator.AccelVRW',           9 }, ...
        {'C_abi_sig',          'Estimator.AccelBiSigma',       10}, ...
        {'C_garw',             'Attitude.GyroArw_radSqrtS',    11}, ...
        {'C_aglsb',            'IMU.AccelScale_gPerLSB',       12}, ...
        {'C_gdpslsb',          'IMU.GyroScale_dpsPerLSB',      13}, ...
        {'C_arange',           'IMU.AccelRange_g',             14}, ...
        {'C_grange',           'IMU.GyroRange_dps',            15}, ...
    };

    y0 = 260;
    dy = 30;
    for k = 1:numel(const_specs)
        nm   = const_specs{k}{1};
        val  = const_specs{k}{2};
        port = const_specs{k}{3};
        cp   = [sub_path '/' nm];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
            'Position', [260 (y0 + (k-1)*dy) 320 (y0 + (k-1)*dy + 20)]);
        add_line(sub_path, [nm '/1'], ['lsm_noise/' num2str(port)], 'autorouting', 'on');
    end
    % reset flag (port 16)
    add_line(sub_path, 'ResetFalse/1', 'lsm_noise/16', 'autorouting', 'on');

    % --- Outports ---
    out_specs = {
        % name,             port_index_on_noise/model
        {'accel_g_body_std',   'lsm_noise/1'}, ...
        {'gyro_dps_body_std',  'lsm_noise/2'}, ...
        {'temp_C',             'lsm_model/3'}, ...
        {'data_ready',         'lsm_model/4'} ...
    };
    for k = 1:numel(out_specs)
        op = [sub_path '/' out_specs{k}{1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [600 (60 + (k-1)*40) 630 (80 + (k-1)*40)]);
        add_line(sub_path, out_specs{k}{2}, [out_specs{k}{1} '/1'], 'autorouting', 'on');
    end
end


% =====================================================================
function build_adxl_subsystem(lib_name)
%BUILD_ADXL_SUBSYSTEM Construct imu_adxl_block under <lib_name>/.

    sub_path = [lib_name '/imu_adxl_block'];
    add_block('built-in/Subsystem', sub_path);
    set_param(sub_path, 'Position', [80 420 560 700]);

    % --- Inport: truth_bus ---
    in_path = [sub_path '/truth_bus'];
    add_block('built-in/Inport', in_path);
    set_param(in_path, 'Position', [20 130 50 150], ...
        'OutDataTypeStr', 'Bus: TruthBus', 'BusObject', 'TruthBus');

    % --- Bus Selector: accel_NED, quat_std, pos_NED ---
    bs_path = [sub_path '/BusSelect'];
    add_block('simulink/Signal Routing/Bus Selector', bs_path);
    set_param(bs_path, 'OutputSignals', 'accel_NED,quat_std,pos_NED');
    set_param(bs_path, 'Position', [90 100 130 180]);

    % --- Rate Transition (post-launch nominal: 1/800) ---
    rt_dt = '1/ADXL.RatePostLaunch_Hz';
    rt_names = {'RT_accel', 'RT_quat', 'RT_pos'};
    for k = 1:3
        rp = [sub_path '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, 'OutPortSampleTime', rt_dt);
        set_param(rp, 'Position', [170 (90 + (k-1)*40) 220 (110 + (k-1)*40)]);
        add_line(sub_path, ['BusSelect/' num2str(k)], [rt_names{k} '/1'], 'autorouting', 'on');
    end

    % --- Convert pos_NED -> altitude (alt_m = -pos_NED(3)) ---
    sel_path = [sub_path '/AltSelect'];
    add_block('simulink/Signal Routing/Selector', sel_path);
    set_param(sel_path, ...
        'NumberOfDimensions', '1', ...
        'IndexOptions',       'Index vector (dialog)', ...
        'Indices',            '3', ...
        'InputPortWidth',     '3', ...
        'Position',           [240 (90 + 2*40) 280 (110 + 2*40)]);
    add_line(sub_path, 'RT_pos/1', 'AltSelect/1', 'autorouting', 'on');

    gain_path = [sub_path '/NegGain'];
    add_block('simulink/Math Operations/Gain', gain_path);
    set_param(gain_path, 'Gain', '-1', ...
        'Position', [300 (90 + 2*40) 340 (110 + 2*40)]);
    add_line(sub_path, 'AltSelect/1', 'NegGain/1', 'autorouting', 'on');

    % --- MATLAB Function: ADXL clean model ---
    mdl_path = [sub_path '/adxl_model'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mdl_path);
    set_param(mdl_path, 'Position', [380 80 480 200]);
    set_adxl_model_script(mdl_path);

    % wire model inputs
    add_line(sub_path, 'RT_accel/1', 'adxl_model/1', 'autorouting', 'on');
    add_line(sub_path, 'RT_quat/1',  'adxl_model/2', 'autorouting', 'on');
    add_line(sub_path, 'NegGain/1',  'adxl_model/3', 'autorouting', 'on');

    % alt_launch_thresh constant
    th_path = [sub_path '/C_AltThresh'];
    add_block('simulink/Sources/Constant', th_path);
    set_param(th_path, 'Value', '5.0', 'SampleTime', rt_dt, ...
        'Position', [300 220 360 240]);
    add_line(sub_path, 'C_AltThresh/1', 'adxl_model/4', 'autorouting', 'on');

    % --- MATLAB Function: noise ---
    noi_path = [sub_path '/adxl_noise'];
    add_block('simulink/User-Defined Functions/MATLAB Function', noi_path);
    set_param(noi_path, 'Position', [520 60 660 240]);
    set_adxl_noise_script(noi_path);

    rst_path = [sub_path '/ResetFalse'];
    add_block('simulink/Sources/Constant', rst_path);
    set_param(rst_path, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [380 220 410 240]);

    add_line(sub_path, 'adxl_model/1', 'adxl_noise/1', 'autorouting', 'on');

    const_specs = {
        {'C_dt',         '1/ADXL.RatePostLaunch_Hz',           2 }, ...
        {'C_seed',       'Sim.Seed + 4',                       3 }, ...
        {'C_bias',       'IMU_T03.ADXL_BiasInit_g',            4 }, ...
        {'C_noise',      'IMU_T03.ADXL_Noise_g',               5 }, ...
        {'C_bw',         'ADXL.BandwidthPostLaunch_Hz',        6 }, ...
        {'C_lsb',        'ADXL.Scale_gPerLSB',                 7 }, ...
        {'C_range',      'ADXL.Range_g',                       8 }, ...
    };
    y0 = 260;
    dy = 30;
    for k = 1:numel(const_specs)
        nm   = const_specs{k}{1};
        val  = const_specs{k}{2};
        port = const_specs{k}{3};
        cp   = [sub_path '/' nm];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
            'Position', [380 (y0 + (k-1)*dy) 440 (y0 + (k-1)*dy + 20)]);
        add_line(sub_path, [nm '/1'], ['adxl_noise/' num2str(port)], 'autorouting', 'on');
    end
    add_line(sub_path, 'ResetFalse/1', 'adxl_noise/9', 'autorouting', 'on');

    out_specs = {
        {'accel_g_body_std', 'adxl_noise/1'}, ...
        {'fifo_active',      'adxl_model/2'} ...
    };
    for k = 1:numel(out_specs)
        op = [sub_path '/' out_specs{k}{1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [720 (60 + (k-1)*40) 750 (80 + (k-1)*40)]);
        add_line(sub_path, out_specs{k}{2}, [out_specs{k}{1} '/1'], 'autorouting', 'on');
    end
end


% =====================================================================
function set_lsm_model_script(block_path)
    src = [ ...
        'function [accel_g_body_std, gyro_dps_body_std, temp_C, data_ready] = fcn(accel_NED, quat_std, omega_body_std)' newline ...
        '%#codegen' newline ...
        '[accel_g_body_std, gyro_dps_body_std, temp_C, data_ready] = casper_imu_lsm_model(accel_NED, quat_std, omega_body_std);' newline ...
        'end' newline];
    set_matlab_fcn_script(block_path, src);
end

function set_lsm_noise_script(block_path)
    src = [ ...
        'function [accel_g_out, gyro_dps_out] = fcn(accel_g_in, gyro_dps_in, dt, seed_base, abi_g, gbi_dps, cax_deg, sf_ppm, accel_vrw, accel_bi_sig, gyro_arw, a_lsb, g_lsb, a_rng, g_rng, reset_flag)' newline ...
        '%#codegen' newline ...
        '[accel_g_out, gyro_dps_out] = casper_imu_lsm_noise(accel_g_in, gyro_dps_in, dt, seed_base, abi_g, gbi_dps, cax_deg, sf_ppm, accel_vrw, accel_bi_sig, gyro_arw, a_lsb, g_lsb, a_rng, g_rng, reset_flag);' newline ...
        'end' newline];
    set_matlab_fcn_script(block_path, src);
end

function set_adxl_model_script(block_path)
    src = [ ...
        'function [accel_g_body_std, fifo_active] = fcn(accel_NED, quat_std, alt_m, alt_launch_thresh_m)' newline ...
        '%#codegen' newline ...
        '[accel_g_body_std, fifo_active] = casper_imu_adxl_model(accel_NED, quat_std, alt_m, alt_launch_thresh_m);' newline ...
        'end' newline];
    set_matlab_fcn_script(block_path, src);
end

function set_adxl_noise_script(block_path)
    src = [ ...
        'function accel_g_out = fcn(accel_g_in, dt, seed_base, bias_init_g, noise_g, bw_hz, lsb_g, range_g, reset_flag)' newline ...
        '%#codegen' newline ...
        'accel_g_out = casper_imu_adxl_noise(accel_g_in, dt, seed_base, bias_init_g, noise_g, bw_hz, lsb_g, range_g, reset_flag);' newline ...
        'end' newline];
    set_matlab_fcn_script(block_path, src);
end

function set_matlab_fcn_script(block_path, src_text)
%SET_MATLAB_FCN_SCRIPT Set the script inside a MATLAB Function block.
    sf  = sfroot;
    blk = sf.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(blk)
        error('build_imu_block:NoChart', ...
            'Could not find MATLAB Function chart at %s', block_path);
    end
    blk.Script = src_text;
end
