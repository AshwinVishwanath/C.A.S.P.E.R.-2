function lib_path = build_mag_block_visual(varargin)
%BUILD_MAG_BLOCK_VISUAL Construct the visual T05 mag library using imuSensor (mag mode).
%
% Synopsis:
%   lib_path = build_mag_block_visual()
%   lib_path = build_mag_block_visual('LibPath', '/abs/path/mag_block_visual.slx')
%
% Builds mag_block_visual.slx containing one named subsystem:
%   - mag_visual_block (truth_bus -> field_uT_body_std, raw18 (3x1 uint32),
%                                      radio_active, data_ready)
%
% Internal wiring:
%   truth_bus  (SensorInputBus)
%      |
%      v   Bus Selector  -> [quat_std, pos_NED, time_s, omega_body_std]
%      v   Rate Transitions  (10 kHz -> 1/Mag.Rate_Hz = 100 Hz)
%      v   MATLAB Function: MagFieldWorld     -> mag_NED_uT (3x1) [casper_mag_field_world]
%      v   MATLAB Function: MagStep            -> mag_uT_body (3x1)
%             (calls casper_mag_step, which wraps imuSensor('accel-gyro-mag'),
%              seeded from Sim.Seed + 3)
%      v   MATLAB Function: MagQuirks          -> [field_uT_body_std (3x1),
%                                                  raw18 (3x1 uint32)]
%             (inverse cal + 18-bit encode + forward cal,
%              calls casper_mag_quirks; firmware-correct per T05 STATUS C2/C5)
%      v   MATLAB Function: RadioTxStep        -> [field_uT_body_std (3x1, post-interf),
%                                                  radio_active (bool)]
%             (TX schedule + ±10 uT spike layer, calls casper_radio_tx_step,
%              seeded from Sim.Seed + 7; PLACEHOLDER per ARCHITECTURE §5 /
%              PHASE0_SPEC §6)
%      v   Bus Creator -> MagOutputBus
%
% All numerical parameters bind at runtime via base-workspace evalin() to
% the structs created by casper_sensor_params.m. Seeds derive from Sim.Seed.
%
% Returns:
%   lib_path : char absolute path to the .slx library.
%
% Source references:
%   ARCHITECTURE.md §5 (mag stock-block intent)
%   T05_mag_sensor_model.md §4–§6 (signal flow + radio interference spec)
%   casper_mag_step.m, casper_mag_quirks.m, casper_radio_tx_step.m
%   casper_build_unified_buses.m (SensorInputBus / MagOutputBus defs)
%   build_imu_block_visual.m (template; we mirror its pattern)

    p = inputParser();
    addParameter(p, 'LibPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    lib_path = char(p.Results.LibPath);

    here = fileparts(mfilename('fullpath'));
    if isempty(lib_path)
        lib_path = fullfile(here, 'mag_block_visual.slx');
    end
    lib_path = char(java.io.File(lib_path).getCanonicalPath());

    lib_name = 'mag_block_visual';

    % Path setup: T05 build dir + T11 (for casper_sim_config) + T02 (for params)
    % + T01 (for TruthBus helper).
    addpath(here);
    addpath(fullfile(here, '..', 'T11_integration'));
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    addpath(fullfile(here, '..', 'T02_sensor_params'));

    % Ensure base workspace has Sim, Mag, TruthBus, and the unified buses.
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

    build_mag_visual_subsystem_(lib_name);

    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, lib_path);
    close_system(lib_name, 0);

    fprintf('[build_mag_block_visual] wrote %s\n', lib_path);
end


% =====================================================================
function build_mag_visual_subsystem_(lib_name)
% MMC5983MA visual subsystem: truth_bus -> imuSensor mag -> quirks -> radio TX
% -> MagOutputBus.

    sub = [lib_name '/mag_visual_block'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [80 60 760 460]);

    rt_dt = '1/Mag.Rate_Hz';

    % --- Inport: truth_bus (SensorInputBus) -----------------------------
    in = [sub '/truth_bus'];
    add_block('built-in/Inport', in);
    set_param(in, 'Position', [20 150 50 170], ...
        'OutDataTypeStr', 'Bus: SensorInputBus', 'BusObject', 'SensorInputBus');

    % --- Bus Selector: quat_std, pos_NED, time_s, omega_body_std --------
    bs = [sub '/BusSelect'];
    add_block('simulink/Signal Routing/Bus Selector', bs);
    set_param(bs, 'OutputSignals', 'quat_std,pos_NED,time_s,omega_body_std');
    set_param(bs, 'Position', [90 100 130 220]);
    add_line(sub, 'truth_bus/1', 'BusSelect/1', 'autorouting', 'on');

    % --- Rate Transitions to 100 Hz -------------------------------------
    rt_names = {'RT_quat', 'RT_pos', 'RT_time', 'RT_omega'};
    for k = 1:4
        rp = [sub '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, 'OutPortSampleTime', rt_dt, ...
            'Position', [170 (90 + (k-1)*40) 220 (110 + (k-1)*40)]);
        add_line(sub, ['BusSelect/' num2str(k)], [rt_names{k} '/1'], 'autorouting', 'on');
    end

    % --- MATLAB Function: MagFieldWorld (NED earth field) --------------
    mf_world = [sub '/MagFieldWorld'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf_world);
    set_param(mf_world, 'Position', [260 80 380 130]);
    set_matlab_fn_script_(mf_world, [ ...
        'function mag_NED_uT = fcn(pos_NED_m)' newline ...
        '%#codegen' newline ...
        'mag_NED_uT = casper_mag_field_world(pos_NED_m);' newline ...
        'end' newline]);
    add_line(sub, 'RT_pos/1', 'MagFieldWorld/1', 'autorouting', 'on');

    % --- MATLAB Function: MagStep (imuSensor mag wrapper) --------------
    % Inputs: mag_NED_uT (3x1), omega_body_std (3x1), quat_std (4x1),
    %         seed_base (1), reset_flag (bool)
    % Output: mag_uT_body (3x1)  (calibrated, post-NED->body rotation,
    %                              plus white sensor noise)
    mf_step = [sub '/MagStep'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf_step);
    set_param(mf_step, 'Position', [420 50 580 200]);
    set_matlab_fn_script_(mf_step, [ ...
        'function mag_uT_body = fcn(' ...
            'mag_NED_uT, omega_body_std, quat_std, seed_base, reset_flag)' newline ...
        'coder.extrinsic(''casper_mag_step'');' newline ...
        'mag_uT_body = zeros(3,1);' newline ...
        'tmp_row = zeros(1,3);' newline ...
        '[tmp_row, ~] = casper_mag_step(' ...
            'mag_NED_uT(:).'', omega_body_std(:).'', quat_std(:).'', ' ...
            'double(seed_base), logical(reset_flag));' newline ...
        'mag_uT_body = tmp_row(:);' newline ...
        'end' newline]);

    add_line(sub, 'MagFieldWorld/1', 'MagStep/1', 'autorouting', 'on');
    add_line(sub, 'RT_omega/1',       'MagStep/2', 'autorouting', 'on');
    add_line(sub, 'RT_quat/1',        'MagStep/3', 'autorouting', 'on');

    % Constants for MagStep seed and reset
    c_seed_mag = [sub '/C_SeedMag'];
    add_block('simulink/Sources/Constant', c_seed_mag);
    set_param(c_seed_mag, 'Value', 'Sim.Seed + 3', 'SampleTime', rt_dt, ...
        'Position', [340 220 400 240]);
    add_line(sub, 'C_SeedMag/1', 'MagStep/4', 'autorouting', 'on');

    c_reset_mag = [sub '/C_ResetMag'];
    add_block('simulink/Sources/Constant', c_reset_mag);
    set_param(c_reset_mag, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [340 250 400 270]);
    add_line(sub, 'C_ResetMag/1', 'MagStep/5', 'autorouting', 'on');

    % --- MATLAB Function: MagQuirks (inverse cal + 18-bit + forward cal)
    % Inputs: mag_uT_in (3x1), hard_iron (3x1), soft_iron (3x3),
    %         axis_flip (3x1), scale_cpg (scalar), offset_counts (scalar)
    % Outputs: mag_uT_cal (3x1), raw18 (3x1 uint32)
    mf_quirks = [sub '/MagQuirks'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf_quirks);
    set_param(mf_quirks, 'Position', [620 40 800 220]);
    set_matlab_fn_script_(mf_quirks, [ ...
        'function [mag_uT_cal, raw18] = fcn(' ...
            'mag_uT_in, hard_iron, soft_iron, axis_flip, scale_cpg, offset_counts)' newline ...
        '%#codegen' newline ...
        '[mag_uT_cal, raw18] = casper_mag_quirks(' ...
            'mag_uT_in(:), hard_iron(:), soft_iron, axis_flip(:), ' ...
            'scale_cpg, offset_counts);' newline ...
        'end' newline]);

    add_line(sub, 'MagStep/1', 'MagQuirks/1', 'autorouting', 'on');

    % MagQuirks parameter constants (pull straight from Mag.*).
    quirk_consts = { ...
        {'C_HardIron',  'Mag.HardIron_uT',             2}, ...
        {'C_SoftIron',  'Mag.SoftIron',                3}, ...
        {'C_AxisFlip',  'Mag.AxisFlipSign',            4}, ...
        {'C_ScaleCpG',  'Mag.ScaleCountsPerGauss',     5}, ...
        {'C_OffsetCnt', 'Mag.OffsetCounts',            6} };
    y0 = 280;
    for k = 1:numel(quirk_consts)
        nm = quirk_consts{k}{1};
        val = quirk_consts{k}{2};
        port = quirk_consts{k}{3};
        cp = [sub '/' nm];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
            'Position', [480 (y0 + (k-1)*25) 580 (y0 + (k-1)*25 + 20)]);
        add_line(sub, [nm '/1'], ['MagQuirks/' num2str(port)], 'autorouting', 'on');
    end

    % --- MATLAB Function: RadioTxStep (TX schedule + spike layer) ------
    % Inputs: mag_uT_in (3x1), time_s, period, airtime, spike_amp,
    %         interf_active, seed_base, reset_flag
    % Outputs: mag_uT_out (3x1), tx_active (bool)
    mf_radio = [sub '/RadioTxStep'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf_radio);
    set_param(mf_radio, 'Position', [860 40 1040 240]);
    set_matlab_fn_script_(mf_radio, [ ...
        'function [mag_uT_out, tx_active] = fcn(' ...
            'mag_uT_in, time_s, period, airtime, spike_amp, ' ...
            'interf_active, seed_base, reset_flag)' newline ...
        'coder.extrinsic(''casper_radio_tx_step'');' newline ...
        'mag_uT_out = zeros(3,1);' newline ...
        'tx_active  = false;' newline ...
        '[mag_uT_out, tx_active, ~, ~] = casper_radio_tx_step(' ...
            'mag_uT_in(:), double(time_s), double(period), double(airtime), ' ...
            'double(spike_amp), logical(interf_active), ' ...
            'double(seed_base), logical(reset_flag));' newline ...
        'end' newline]);

    add_line(sub, 'MagQuirks/1', 'RadioTxStep/1', 'autorouting', 'on');
    add_line(sub, 'RT_time/1',   'RadioTxStep/2', 'autorouting', 'on');

    radio_consts = { ...
        {'C_TxPeriod',  'Mag.RadioTXPeriod_s',         3}, ...
        {'C_TxAirtime', 'Mag.RadioTXAirtime_s',        4}, ...
        {'C_SpikeAmp',  'Mag.RadioSpikeAmp_uT',        5}, ...
        {'C_InterfOn',  'Mag.RadioInterfActive',       6}, ...
        {'C_SeedRadio', 'Sim.Seed + 7',                7}, ...
        {'C_ResetRadio','false',                       8} };
    y0 = 280;
    for k = 1:numel(radio_consts)
        nm = radio_consts{k}{1};
        val = radio_consts{k}{2};
        port = radio_consts{k}{3};
        cp = [sub '/' nm];
        add_block('simulink/Sources/Constant', cp);
        is_bool = strcmp(val, 'true') || strcmp(val, 'false') ...
            || strcmp(val, 'Mag.RadioInterfActive');
        if is_bool
            set_param(cp, 'Value', val, 'OutDataTypeStr', 'boolean', ...
                'SampleTime', rt_dt, ...
                'Position', [720 (y0 + (k-1)*25) 820 (y0 + (k-1)*25 + 20)]);
        else
            set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
                'Position', [720 (y0 + (k-1)*25) 820 (y0 + (k-1)*25 + 20)]);
        end
        add_line(sub, [nm '/1'], ['RadioTxStep/' num2str(port)], 'autorouting', 'on');
    end

    % --- data_ready constant (mag is at-rate; tied true) ----------------
    c_dr = [sub '/C_DataReady'];
    add_block('simulink/Sources/Constant', c_dr);
    set_param(c_dr, 'Value', 'true', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [860 260 920 280]);

    % --- Outports -------------------------------------------------------
    outs = { ...
        {'field_uT_body_std', 'RadioTxStep/1'}, ...
        {'raw18',             'MagQuirks/2'}, ...
        {'radio_active',      'RadioTxStep/2'}, ...
        {'data_ready',        'C_DataReady/1'} };
    for k = 1:numel(outs)
        op = [sub '/' outs{k}{1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [1080 (60 + (k-1)*45) 1110 (80 + (k-1)*45)]);
        add_line(sub, outs{k}{2}, [outs{k}{1} '/1'], 'autorouting', 'on');
    end
end


% =====================================================================
function set_matlab_fn_script_(block_path, src)
% SET_MATLAB_FN_SCRIPT_ Install the script body of a MATLAB Function block.
    sf = sfroot;
    blk = sf.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(blk)
        error('build_mag_block_visual:NoChart', ...
            'Could not find MATLAB Function chart at %s', block_path);
    end
    blk.Script = src;
end
