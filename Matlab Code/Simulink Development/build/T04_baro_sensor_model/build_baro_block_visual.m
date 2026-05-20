function lib_path = build_baro_block_visual(varargin)
%BUILD_BARO_BLOCK_VISUAL Construct the visual T04 baro library using
%atmoscoesa truth + custom Mach-shock + noise + MS5611 quirks.
%
% Synopsis:
%   lib_path = build_baro_block_visual()
%   lib_path = build_baro_block_visual('LibPath', '/abs/path/baro_block_visual.slx')
%
% Builds baro_block_visual.slx containing one named subsystem:
%   - baro_visual_block (SensorInputBus ->
%                          press_pa, alt_m, temp_C, data_ready)
%
% Internal structure:
%   SensorInputBus
%       -> Bus Selector (air_pressure_pa, air_temp_K, mach, vel_NED,
%                         air_density_kgm3)
%       -> Rate Transitions to 1/Baro.Rate_Hz (100 Hz)
%       -> casper_baro_step    (atmoscoesa truth + Mach-shock + noise)
%       -> casper_baro_quirks  (1-Pa quantize + MS5611 altitude decode)
%       -> Outports
%
% The atmosphere path uses MATLAB's atmoscoesa under the hood (or falls
% through to the truth bus's pre-computed air_pressure_pa per the
% Baro_T04.ForceAtmosCOESA flag).
%
% Returns:
%   lib_path : char absolute path to the .slx library.
%
% Source references:
%   - ARCHITECTURE.md §5 (stock-block intent)
%   - casper_baro_step.m, casper_baro_quirks.m
%   - casper_baro_mach_shock.m (T04 spec §5.2; reused unchanged from legacy)
%   - casper_build_unified_buses.m (SensorInputBus / BaroOutputBus defs)

    p = inputParser();
    addParameter(p, 'LibPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    lib_path = char(p.Results.LibPath);

    here = fileparts(mfilename('fullpath'));
    if isempty(lib_path)
        lib_path = fullfile(here, 'baro_block_visual.slx');
    end
    lib_path = char(java.io.File(lib_path).getCanonicalPath());

    lib_name = 'baro_block_visual';

    % Path setup: T04 build dir + T11 (for casper_sim_config) + T01 +
    % T02 (for casper_sensor_params).
    addpath(here);
    addpath(fullfile(here, '..', 'T11_integration'));
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    addpath(fullfile(here, '..', 'T02_sensor_params'));

    % Ensure base workspace has Sim, Baro, SensorInputBus, BaroOutputBus.
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

    build_baro_visual_subsystem_(lib_name);

    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, lib_path);
    close_system(lib_name, 0);

    fprintf('[build_baro_block_visual] wrote %s\n', lib_path);
end


% =====================================================================
function build_baro_visual_subsystem_(lib_name)
% MS5611 baro visual subsystem:
%   SensorInputBus -> baro_step -> baro_quirks -> outputs.

    sub = [lib_name '/baro_visual_block'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [80 60 700 480]);

    % Inport: sensor_bus (SensorInputBus)
    in = [sub '/sensor_bus'];
    add_block('built-in/Inport', in);
    set_param(in, 'Position', [20 200 50 220], ...
        'OutDataTypeStr', 'Bus: SensorInputBus', ...
        'BusObject',      'SensorInputBus', ...
        'UseBusObject',   'on');

    % Bus Selector: pull out the 5 fields the baro block uses.
    bs = [sub '/BusSelect'];
    add_block('simulink/Signal Routing/Bus Selector', bs);
    set_param(bs, 'OutputSignals', ...
        'air_pressure_pa,air_temp_K,mach,vel_NED,air_density_kgm3');
    set_param(bs, 'Position', [90 80 130 320]);
    add_line(sub, 'sensor_bus/1', 'BusSelect/1', 'autorouting', 'on');

    % Rate Transitions to 1/Baro.Rate_Hz (100 Hz). One per selected signal.
    rt_dt = '1/Baro.Rate_Hz';
    rt_names = {'RT_p_truth', 'RT_T_K', 'RT_mach', 'RT_vel', 'RT_rho'};
    for k = 1:5
        rp = [sub '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, 'OutPortSampleTime', rt_dt, ...
            'Position', [170 (70 + (k-1)*45) 220 (90 + (k-1)*45)]);
        add_line(sub, ['BusSelect/' num2str(k)], [rt_names{k} '/1'], ...
            'autorouting', 'on');
    end

    % MATLAB Function: casper_baro_step wrapper (atmoscoesa + Mach + noise)
    mf = [sub '/baro_step'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf);
    set_param(mf, 'Position', [280 60 460 260]);
    set_matlab_fn_script_(mf, [ ...
        'function [p_pa_noisy, t_C] = fcn(' ...
            'p_truth_pa, T_K, mach, vel_NED, rho_kgm3, ' ...
            'seed_base, reset_flag)' newline ...
        'coder.extrinsic(''casper_baro_step'');' newline ...
        'p_pa_noisy = double(0);' newline ...
        't_C        = double(0);' newline ...
        '[p_pa_noisy, t_C, ~] = casper_baro_step(' ...
            'p_truth_pa, T_K, mach, vel_NED(:), rho_kgm3, ' ...
            'double(seed_base), logical(reset_flag));' newline ...
        'end' newline]);

    add_line(sub, 'RT_p_truth/1', 'baro_step/1', 'autorouting', 'on');
    add_line(sub, 'RT_T_K/1',     'baro_step/2', 'autorouting', 'on');
    add_line(sub, 'RT_mach/1',    'baro_step/3', 'autorouting', 'on');
    add_line(sub, 'RT_vel/1',     'baro_step/4', 'autorouting', 'on');
    add_line(sub, 'RT_rho/1',     'baro_step/5', 'autorouting', 'on');

    % Constants for seed and reset (baro uses Sim.Seed + 2)
    c_seed = [sub '/C_Seed'];
    add_block('simulink/Sources/Constant', c_seed);
    set_param(c_seed, 'Value', 'Sim.Seed + 2', 'SampleTime', rt_dt, ...
        'Position', [170 320 220 340]);
    add_line(sub, 'C_Seed/1', 'baro_step/6', 'autorouting', 'on');

    c_reset = [sub '/C_Reset'];
    add_block('simulink/Sources/Constant', c_reset);
    set_param(c_reset, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [170 350 220 370]);
    add_line(sub, 'C_Reset/1', 'baro_step/7', 'autorouting', 'on');

    % MATLAB Function: PostQuirks (1-Pa quantize + altitude decode)
    pq = [sub '/PostQuirks'];
    add_block('simulink/User-Defined Functions/MATLAB Function', pq);
    set_param(pq, 'Position', [500 60 660 240]);
    set_matlab_fn_script_(pq, [ ...
        'function [press_pa, alt_m] = fcn(' ...
            'p_pa_noisy, sea_level_hpa, alt_coeff_m, alt_exp, p_res_pa)' newline ...
        '%#codegen' newline ...
        '[press_pa, alt_m] = casper_baro_quirks(' ...
            'p_pa_noisy, sea_level_hpa, alt_coeff_m, alt_exp, p_res_pa);' newline ...
        'end' newline]);

    add_line(sub, 'baro_step/1', 'PostQuirks/1', 'autorouting', 'on');

    % Quirks constants (firmware MS5611 decode formula parameters)
    quirk_consts = { ...
        {'C_SeaLevel', 'Baro.SeaLevelRef_hPa',  2}, ...
        {'C_AltCoeff', 'Baro.AltitudeCoeff',    3}, ...
        {'C_AltExp',   'Baro.AltitudeExponent', 4}, ...
        {'C_PRes',     'Baro.PressureRes_Pa',   5} };
    y0 = 270;
    for k = 1:numel(quirk_consts)
        nm = quirk_consts{k}{1};
        val = quirk_consts{k}{2};
        port = quirk_consts{k}{3};
        cp = [sub '/' nm];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
            'Position', [350 (y0 + (k-1)*25) 420 (y0 + (k-1)*25 + 20)]);
        add_line(sub, [nm '/1'], ['PostQuirks/' num2str(port)], ...
            'autorouting', 'on');
    end

    % Temperature passes through from baro_step (no quirk needed)
    % data_ready: Constant true at the baro rate
    c_dr = [sub '/C_DataReady'];
    add_block('simulink/Sources/Constant', c_dr);
    set_param(c_dr, 'Value', 'true', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [500 280 550 300]);

    % Outports (match BaroOutputBus field order: press_pa, alt_m, temp_C, data_ready)
    outs = { ...
        {'press_pa',   'PostQuirks/1'}, ...
        {'alt_m',      'PostQuirks/2'}, ...
        {'temp_C',     'baro_step/2'}, ...
        {'data_ready', 'C_DataReady/1'} };
    for k = 1:numel(outs)
        op = [sub '/' outs{k}{1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [710 (60 + (k-1)*45) 740 (80 + (k-1)*45)]);
        add_line(sub, outs{k}{2}, [outs{k}{1} '/1'], 'autorouting', 'on');
    end
end


% =====================================================================
function set_matlab_fn_script_(block_path, src)
% SET_MATLAB_FN_SCRIPT_ Install the script body of a MATLAB Function block.
    sf = sfroot;
    blk = sf.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(blk)
        error('build_baro_block_visual:NoChart', ...
            'Could not find MATLAB Function chart at %s', block_path);
    end
    blk.Script = src;
end
