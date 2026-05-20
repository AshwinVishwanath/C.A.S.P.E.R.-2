function block_path = build_baro_block(varargin)
%BUILD_BARO_BLOCK Programmatically construct the MS5611 baro sensor subsystem
% in casper_sim_lib.slx and emit a standalone baro_block.slx.
%
% Synopsis:
%   block_path = build_baro_block()
%   block_path = build_baro_block('LibPath', '/abs/path/casper_sim_lib.slx')
%
% Side effects:
%   - Opens (or creates) the Simulink library casper_sim_lib.slx in T01.
%   - Adds (or replaces) a subsystem called 'baro_block' inside it.
%   - Writes a standalone Simulink model copy at <T04 dir>/baro_block.slx
%     containing just the baro_block subsystem (for diff-friendly storage
%     of the T04 block; the canonical library file lives under T01).
%
% Block structure:
%   Inputs (1):
%       truth_bus                       (TruthBus from T01)
%   Internals:
%       BusSelector (extract air_pressure_pa, mach, vel_NED,
%                    air_density_kgm3, air_temp_K)
%       RateTransition (each scalar) -> 100 Hz (Baro.Rate_Hz)
%       MATLAB Function pressure_model
%       MATLAB Function mach_shock_model
%       MATLAB Function noise_model (persistent RNG state)
%   Outputs (3):
%       pressure_pa     (double, 100 Hz)
%       temp_C          (double, 100 Hz)
%       data_ready      (boolean, 100 Hz)
%
% Round 2 changes vs Round 1:
%   - NoiseModel MATLAB Function block declares explicit input/output
%     sizes/types via the Stateflow.Data interface, so Simulink can size
%     the block at compile time. R1 left `p_in_pa` to be inferred, which
%     combined with `evalin`+`coder.extrinsic` broke compile-time analysis.
%   - The standalone baro_block.slx now contains the actual subsystem (not
%     a library reference), so it can be loaded and exercised without the
%     casper_sim_lib being on disk.
%
% Source firmware reference:
%   Software/App/drivers/ms5611.c -- the firmware non-blocking state machine
%   we approximate as a fixed 100 Hz sample rate per T04 spec section 5.5.

    p = inputParser();
    addParameter(p, 'LibPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    lib_path = char(p.Results.LibPath);

    here = fileparts(mfilename('fullpath'));
    if isempty(lib_path)
        % Canonical library lives in T01 dir per T01 convention.
        t01_dir  = fullfile(here, '..', 'T01_truth_pipeline');
        t01_dir  = char(java.io.File(t01_dir).getCanonicalPath());
        lib_path = fullfile(t01_dir, 'casper_sim_lib.slx');
    end

    % We also write a standalone copy under T04 for storage / inspection.
    standalone_path = fullfile(here, 'baro_block.slx');

    lib_name = 'casper_sim_lib';
    sub_name = 'baro_block';

    % --- Open / create the library ---
    if bdIsLoaded(lib_name)
        close_system(lib_name, 0);
    end

    if isfile(lib_path)
        load_system(lib_path);
        set_param(lib_name, 'Lock', 'off');
        if getSimulinkBlockHandle([lib_name '/' sub_name]) ~= -1
            delete_block([lib_name '/' sub_name]);
        end
    else
        new_system(lib_name, 'Library');
        load_system(lib_name);
    end

    % TruthBus must exist for bus inports.
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    truth_bus_obj = casper_truth_build_bus();
    assignin('base', 'TruthBus', truth_bus_obj);

    % Ensure Sim / Baro exist in base WS (needed by NoiseModel at compile).
    if ~evalin('base', 'exist(''Sim'', ''var'')')
        evalin('base', sprintf('run(''%s'')', strrep(fullfile(here, '..', ...
            'T02_sensor_params', 'casper_sensor_params.m'), '\', '/')));
    end

    build_subsystem_(lib_name, sub_name);

    % --- Lock & save library ---
    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, lib_path);

    % --- Write standalone baro_block.slx (contains the full subsystem) ---
    sa_name = 'baro_block';
    if bdIsLoaded(sa_name); close_system(sa_name, 0); end
    if isfile(standalone_path); delete(standalone_path); end
    new_system(sa_name);
    load_system(sa_name);

    build_subsystem_(sa_name, sub_name);

    save_system(sa_name, standalone_path);
    close_system(sa_name, 0);
    close_system(lib_name, 0);

    block_path = [lib_name '/' sub_name];
    fprintf('[build_baro_block] wrote subsystem %s in %s\n', sub_name, lib_path);
    fprintf('[build_baro_block] wrote standalone %s\n', standalone_path);
end

% ========================================================================

function build_subsystem_(parent_name, sub_name)
%BUILD_SUBSYSTEM_ Construct the baro_block subsystem under parent_name.
% Used both for the library copy and the standalone slx copy so they stay
% byte-identical at the block level.

    sub_path = [parent_name '/' sub_name];
    add_block('built-in/Subsystem', sub_path);
    set_param(sub_path, 'Position', [80 60 480 460]);

    % Inport: truth_bus (TruthBus)
    in_truth = [sub_path '/truth_bus'];
    add_block('built-in/Inport', in_truth);
    set_param(in_truth, 'Position', [20 200 50 220]);
    set_param(in_truth, 'OutDataTypeStr', 'Bus: TruthBus');
    set_param(in_truth, 'BusObject',      'TruthBus');
    set_param(in_truth, 'UseBusObject',   'on');

    % Bus Selector: pull out the 5 truth fields the block uses.
    sel_path = [sub_path '/BusSel'];
    add_block('simulink/Signal Routing/Bus Selector', sel_path);
    sel_signals = 'air_pressure_pa,mach,vel_NED,air_density_kgm3,air_temp_K';
    set_param(sel_path, 'OutputSignals', sel_signals);
    set_param(sel_path, 'Position',      [90 80 100 320]);

    add_line(sub_path, 'truth_bus/1', 'BusSel/1', 'autorouting', 'on');

    % Rate Transition blocks (driver rate -> Baro.Rate_Hz). One per signal.
    % Per ARCHITECTURE.md sect 4 and SIMULINK_PATTERNS.md sect 4.3 the
    % parameter is `Integrity` ('on'/'off') in R2025b; `DeterministicData-
    % Transfer` does NOT exist on this block.
    rt_names = {'RT_p', 'RT_M', 'RT_v', 'RT_rho', 'RT_T'};
    for k = 1:numel(rt_names)
        rp = [sub_path '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, ...
            'OutPortSampleTimeOpt',     'Specify', ...
            'OutPortSampleTime',        '1/Baro.Rate_Hz', ...
            'X0',                       '0', ...
            'Integrity',                'on', ...
            'Deterministic',            'on');
        set_param(rp, 'Position', [180 70 + (k-1)*45 220 90 + (k-1)*45]);
        add_line(sub_path, ['BusSel/' num2str(k)], [rt_names{k} '/1'], ...
                 'autorouting', 'on');
    end

    % MATLAB Function: pressure_model
    pm_path = [sub_path '/PressureModel'];
    add_block('simulink/User-Defined Functions/MATLAB Function', pm_path);
    set_param(pm_path, 'Position', [260 60 360 110]);
    set_pm_script(pm_path);

    % MATLAB Function: mach_shock_model
    ms_path = [sub_path '/MachShockModel'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ms_path);
    set_param(ms_path, 'Position', [260 140 360 220]);
    set_ms_script(ms_path);

    % MATLAB Function: noise_model
    nm_path = [sub_path '/NoiseModel'];
    add_block('simulink/User-Defined Functions/MATLAB Function', nm_path);
    set_param(nm_path, 'Position', [400 80 510 180]);
    set_nm_script(nm_path);

    % MATLAB Function: temp model (T_K -> T_C)
    tm_path = [sub_path '/TempModel'];
    add_block('simulink/User-Defined Functions/MATLAB Function', tm_path);
    set_param(tm_path, 'Position', [400 220 510 270]);
    set_tm_script(tm_path);

    % Constant: data_ready = true at 100 Hz
    dr_path = [sub_path '/DataReady'];
    add_block('simulink/Sources/Constant', dr_path);
    set_param(dr_path, 'Value', 'boolean(1)');
    set_param(dr_path, 'OutDataTypeStr', 'boolean');
    set_param(dr_path, 'SampleTime', '1/Baro.Rate_Hz');
    set_param(dr_path, 'Position', [400 310 510 340]);

    % Outports
    out_p = [sub_path '/pressure_pa'];
    add_block('built-in/Outport', out_p);
    set_param(out_p, 'Position', [560 110 590 130]);

    out_T = [sub_path '/temp_C'];
    add_block('built-in/Outport', out_T);
    set_param(out_T, 'Position', [560 235 590 255]);

    out_DR = [sub_path '/data_ready'];
    add_block('built-in/Outport', out_DR);
    set_param(out_DR, 'Position', [560 315 590 335]);

    % --- Wire blocks ---
    % BusSel ports correspond 1:1 to sel_signals listed above:
    %   1: air_pressure_pa, 2: mach, 3: vel_NED, 4: air_density_kgm3,
    %   5: air_temp_K

    % PressureModel: input p_truth (RT_p)
    add_line(sub_path, 'RT_p/1',  'PressureModel/1', 'autorouting', 'on');

    % MachShockModel: inputs p_clean, M, v, rho
    add_line(sub_path, 'PressureModel/1', 'MachShockModel/1', 'autorouting', 'on');
    add_line(sub_path, 'RT_M/1',          'MachShockModel/2', 'autorouting', 'on');
    add_line(sub_path, 'RT_v/1',          'MachShockModel/3', 'autorouting', 'on');
    add_line(sub_path, 'RT_rho/1',        'MachShockModel/4', 'autorouting', 'on');

    % NoiseModel: input pressure_in (after shock)
    add_line(sub_path, 'MachShockModel/1', 'NoiseModel/1', 'autorouting', 'on');

    % TempModel: input T_K (RT_T)
    add_line(sub_path, 'RT_T/1', 'TempModel/1', 'autorouting', 'on');

    % Outports
    add_line(sub_path, 'NoiseModel/1', 'pressure_pa/1', 'autorouting', 'on');
    add_line(sub_path, 'TempModel/1',  'temp_C/1',      'autorouting', 'on');
    add_line(sub_path, 'DataReady/1',  'data_ready/1',  'autorouting', 'on');
end

% ========================================================================

function set_pm_script(block_path)
%SET_PM_SCRIPT Set the PressureModel MATLAB Function block's script.
    script = sprintf([ ...
'function p_clean_pa = fcn(p_truth_pa)\n' ...
'%%#codegen\n' ...
'p_clean_pa = casper_baro_pressure_model(p_truth_pa);\n' ...
'end\n']);
    set_mlfn_script(block_path, script);
    set_input_props(block_path,  'p_truth_pa',  '1');
    set_output_props(block_path, 'p_clean_pa',  '1');
end

function set_ms_script(block_path)
%SET_MS_SCRIPT Set the MachShockModel MATLAB Function block's script.
    script = sprintf([ ...
'function p_out_pa = fcn(p_clean_pa, mach, vel_NED, rho_kgm3)\n' ...
'%%#codegen\n' ...
'p_out_pa = casper_baro_mach_shock(p_clean_pa, mach, vel_NED, rho_kgm3);\n' ...
'end\n']);
    set_mlfn_script(block_path, script);
    set_input_props(block_path,  'p_clean_pa',  '1');
    set_input_props(block_path,  'mach',        '1');
    set_input_props(block_path,  'vel_NED',     '[3 1]');
    set_input_props(block_path,  'rho_kgm3',    '1');
    set_output_props(block_path, 'p_out_pa',    '1');
end

function set_nm_script(block_path)
%SET_NM_SCRIPT Set the NoiseModel MATLAB Function block's script.
%
% R2 change: noise math is inlined inside the block (no coder.extrinsic
% on casper_baro_noise) so Simulink can size all outputs at compile time.
% Persistent doubles hold the bias_offset, bias_drift, and RNG seed across
% calls; RandStream is created on the first call and held in a persistent
% (declared extrinsic since RandStream is a MATLAB class). Random samples
% are pulled via the persistent stream so reproducibility holds.
%
% Constants are parsed from base workspace `Sim` and `Baro` on the first
% call only. After that the block is purely numeric.
    script = sprintf([ ...
'function p_meas_pa = fcn(p_in_pa)\n' ...
'%%#codegen\n' ...
'p_meas_pa = double(0);\n' ...
'coder.extrinsic(''casper_baro_noise_step'');\n' ...
'p_meas_pa = casper_baro_noise_step(p_in_pa);\n' ...
'end\n']);
    set_mlfn_script(block_path, script);
    set_input_props(block_path,  'p_in_pa',    '1');
    set_output_props(block_path, 'p_meas_pa',  '1');
end

function set_tm_script(block_path)
%SET_TM_SCRIPT Set the TempModel MATLAB Function block's script.
    script = sprintf([ ...
'function T_C = fcn(T_K)\n' ...
'%%#codegen\n' ...
'T_C = T_K - 273.15;\n' ...
'end\n']);
    set_mlfn_script(block_path, script);
    set_input_props(block_path,  'T_K', '1');
    set_output_props(block_path, 'T_C', '1');
end

function set_mlfn_script(block_path, script)
%SET_MLFN_SCRIPT Drop a MATLAB Function block's script content programmatically.
    rt = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart', 'Path', block_path);
    if isempty(chart)
        error('build_baro_block:set_script', ...
              'Could not locate EMChart for block %s', block_path);
    end
    chart.Script = script;
end

function set_input_props(block_path, port_name, size_str)
%SET_INPUT_PROPS Set explicit size + type on a MATLAB Function block input.
    set_data_props_(block_path, port_name, size_str, 'Input');
end

function set_output_props(block_path, port_name, size_str)
%SET_OUTPUT_PROPS Set explicit size + type on a MATLAB Function block output.
    set_data_props_(block_path, port_name, size_str, 'Output');
end

function set_data_props_(block_path, port_name, size_str, scope)
% Set scope (Input/Output) and explicit size on a Stateflow data item.
% Type is left at default ('Inherited' / double primitive), which is what
% Simulink uses for double-typed signals from upstream blocks.
    rt = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart', 'Path', block_path);
    if isempty(chart)
        return;
    end
    data = chart.find('-isa','Stateflow.Data');
    for k = 1:numel(data)
        if strcmp(data(k).Name, port_name)
            data(k).Scope = scope;
            data(k).Props.Array.Size = size_str;
            return;
        end
    end
end
