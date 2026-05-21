function model_path = build_mag_block(varargin)
%BUILD_MAG_BLOCK Programmatically construct the mag_block Simulink library.
%
% Synopsis:
%   model_path = build_mag_block()
%   model_path = build_mag_block('ModelPath', '/abs/path/mag_block.slx')
%
% Creates (or rebuilds) a self-contained Simulink library at
% <T05 build dir>/mag_block.slx containing one subsystem `mag_block`
% with the signal flow described in T05 §4:
%
%   truth_bus
%      │
%      ▼  Bus Selector (extract quat_std + time_s + pos_NED)
%      │
%      ▼  Rate Transition (10 kHz -> 100 Hz)
%      │
%      ▼  MATLAB Function: mag_field_world  -> NED [µT]
%      ▼  MATLAB Function: mag_rotate_to_body -> body [µT]
%      ▼  MATLAB Function: mag_distort_field   -> raw body [µT] (cal-inverse)
%      ▼  MATLAB Function: mag_noise            -> AR(1) + 18-bit quantize
%      ▼  MATLAB Function: mag_radio_interference (gated by tx_schedule)
%      │
%      ▼  Outports: mag_uT_body (3x1 double), mag_raw_18bit (3x1 uint32),
%                   data_ready (bool), tx_active (bool)
%
% All numerical parameters are bound at runtime to base-workspace variables
% Mag.* and Sim.* per SIMULINK_PATTERNS.md §9. Run
% `casper_sensor_params` before opening this model.
%
% The model is saved as a Library (locked). Downstream models reference
% the subsystem via library link.
%
% Outputs:
%   model_path : char, absolute path to the saved .slx library.
%
% Source firmware reference:
%   None (sim-side construction wrapper).

    p = inputParser();
    addParameter(p, 'ModelPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    model_path = char(p.Results.ModelPath);

    here = fileparts(mfilename('fullpath'));
    if isempty(model_path)
        model_path = fullfile(here, 'mag_block.slx');
    end

    lib_name = 'mag_block';
    sub_name = 'mag_block';

    if bdIsLoaded(lib_name)
        close_system(lib_name, 0);
    end

    if isfile(model_path)
        delete(model_path);
    end

    new_system(lib_name, 'Library');
    load_system(lib_name);

    sub_path = [lib_name '/' sub_name];
    add_block('built-in/Subsystem', sub_path);
    set_param(sub_path, 'Position', [80 60 540 460]);

    % ---------- subsystem inports ----------
    add_block('built-in/Inport',  [sub_path '/quat_std']);
    set_param([sub_path '/quat_std'], 'Position', [30  30  60  50],  'PortDimensions', '4');
    add_block('built-in/Inport',  [sub_path '/pos_NED_m']);
    set_param([sub_path '/pos_NED_m'], 'Position', [30  80  60 100],  'PortDimensions', '3');
    add_block('built-in/Inport',  [sub_path '/time_s']);
    set_param([sub_path '/time_s'], 'Position', [30 130  60 150],  'PortDimensions', '1');

    % ---------- rate transitions (10 kHz -> 100 Hz) ----------
    add_rate_transition(sub_path, 'RT_quat',   '1/Mag.Rate_Hz');
    set_param([sub_path '/RT_quat'],   'Position', [110  25 150  55]);
    add_rate_transition(sub_path, 'RT_pos',    '1/Mag.Rate_Hz');
    set_param([sub_path '/RT_pos'],    'Position', [110  75 150 105]);
    add_rate_transition(sub_path, 'RT_time',   '1/Mag.Rate_Hz');
    set_param([sub_path '/RT_time'],   'Position', [110 125 150 155]);

    % ---------- MATLAB Function: mag_field_world ----------
    fcn_field = [sub_path '/MagFieldWorld'];
    add_matlab_function(fcn_field, mag_field_world_script());
    set_param(fcn_field, 'Position', [180  60 280 110]);

    % ---------- MATLAB Function: mag_rotate_to_body ----------
    fcn_rot = [sub_path '/MagRotateToBody'];
    add_matlab_function(fcn_rot, mag_rotate_script());
    set_param(fcn_rot, 'Position', [300  40 410 110]);

    % ---------- MATLAB Function: mag_distort_field ----------
    fcn_dist = [sub_path '/MagDistortField'];
    add_matlab_function(fcn_dist, mag_distort_script());
    set_param(fcn_dist, 'Position', [430  20 560 130]);

    % ---------- MATLAB Function: mag_noise ----------
    fcn_noise = [sub_path '/MagNoise'];
    add_matlab_function(fcn_noise, mag_noise_script());
    set_param(fcn_noise, 'Position', [590  20 720 130]);

    % ---------- MATLAB Function: tx_schedule ----------
    fcn_tx = [sub_path '/RadioTxSchedule'];
    add_matlab_function(fcn_tx, tx_schedule_script());
    set_param(fcn_tx, 'Position', [180 150 280 200]);

    % ---------- MATLAB Function: mag_radio_interference ----------
    fcn_interf = [sub_path '/MagRadioInterference'];
    add_matlab_function(fcn_interf, mag_interference_script());
    set_param(fcn_interf, 'Position', [750  30 900 150]);

    % ---------- data_ready: constant true at mag rate ----------
    add_block('simulink/Sources/Constant', [sub_path '/DataReadyConst']);
    set_param([sub_path '/DataReadyConst'], ...
        'Value', 'boolean(1)', ...
        'OutDataTypeStr', 'boolean', ...
        'SampleTime', '1/Mag.Rate_Hz', ...
        'Position', [180 240 280 270]);

    % ---------- subsystem outports ----------
    add_block('built-in/Outport', [sub_path '/mag_uT_body']);
    set_param([sub_path '/mag_uT_body'],   'Position', [950  35 980  55],  'PortDimensions', '3');
    add_block('built-in/Outport', [sub_path '/mag_raw_18bit']);
    set_param([sub_path '/mag_raw_18bit'], 'Position', [950  85 980 105],  'PortDimensions', '3');
    add_block('built-in/Outport', [sub_path '/data_ready']);
    set_param([sub_path '/data_ready'],    'Position', [950 245 980 265],  'PortDimensions', '1');
    add_block('built-in/Outport', [sub_path '/tx_active']);
    set_param([sub_path '/tx_active'],     'Position', [950 175 980 195],  'PortDimensions', '1');

    % ---------- wire it up ----------
    add_line(sub_path, 'quat_std/1',  'RT_quat/1',  'autorouting', 'on');
    add_line(sub_path, 'pos_NED_m/1', 'RT_pos/1',   'autorouting', 'on');
    add_line(sub_path, 'time_s/1',    'RT_time/1',  'autorouting', 'on');

    add_line(sub_path, 'RT_pos/1',  'MagFieldWorld/1',     'autorouting', 'on');
    add_line(sub_path, 'MagFieldWorld/1', 'MagRotateToBody/1', 'autorouting', 'on');
    add_line(sub_path, 'RT_quat/1',       'MagRotateToBody/2', 'autorouting', 'on');

    add_line(sub_path, 'MagRotateToBody/1', 'MagDistortField/1', 'autorouting', 'on');

    add_line(sub_path, 'MagDistortField/1', 'MagNoise/1', 'autorouting', 'on');

    % MagNoise outputs: mag_noisy_uT (1), mag_raw_18bit (2).
    add_line(sub_path, 'MagNoise/1', 'MagRadioInterference/1', 'autorouting', 'on');

    add_line(sub_path, 'RT_time/1',          'RadioTxSchedule/1', 'autorouting', 'on');
    add_line(sub_path, 'RadioTxSchedule/1',  'MagRadioInterference/2', 'autorouting', 'on');

    add_line(sub_path, 'MagRadioInterference/1', 'mag_uT_body/1', 'autorouting', 'on');
    add_line(sub_path, 'MagNoise/2',             'mag_raw_18bit/1', 'autorouting', 'on');
    add_line(sub_path, 'DataReadyConst/1',       'data_ready/1', 'autorouting', 'on');
    add_line(sub_path, 'RadioTxSchedule/1',      'tx_active/1', 'autorouting', 'on');

    % Lock and save.
    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, model_path);
    % save_system already persists; close_system second arg = 0 means
    % discard further unsaved edits (none expected). This still emits a
    % warning if Simulink considers the model dirty (e.g. from auto-
    % positioning side-effects); force-close to be safe.
    close_system(lib_name, 1);

    fprintf('[build_mag_block] wrote %s\n', model_path);
end

% ---------------------------------------------------------------------------
% Helpers
% ---------------------------------------------------------------------------

function add_rate_transition(parent, name, sample_time_expr)
    path = [parent '/' name];
    add_block('simulink/Signal Attributes/Rate Transition', path);
    set_param(path, ...
        'OutPortSampleTime', sample_time_expr);
end

function add_matlab_function(path, script_text)
% Adds a MATLAB Function block and writes the supplied Script into it.
    add_block('simulink/User-Defined Functions/MATLAB Function', path);
    root = sfroot();
    chart = find(root, '-isa', 'Stateflow.EMChart', 'Path', path);
    if isempty(chart)
        error('build_mag_block:NoChart', ...
            'Could not locate Stateflow.EMChart for %s', path);
    end
    chart = chart(1);
    chart.Script = script_text;
end

% ---------- inline scripts for each MATLAB Function block ----------

function s = mag_field_world_script()
    s = sprintf([ ...
        'function mag_NED_uT = MagFieldWorld(pos_NED_m)\n' ...
        '%%#codegen\n' ...
        'mag_NED_uT = casper_mag_field_world(pos_NED_m);\n' ...
        'end\n']);
end

function s = mag_rotate_script()
    s = sprintf([ ...
        'function mag_body_uT = MagRotateToBody(mag_NED_uT, quat_std)\n' ...
        '%%#codegen\n' ...
        'mag_body_uT = casper_mag_rotate_to_body(mag_NED_uT, quat_std);\n' ...
        'end\n']);
end

function s = mag_distort_script()
    s = sprintf([ ...
        'function mag_raw_body_uT = MagDistortField(mag_clean_body_uT)\n' ...
        '%%#codegen\n' ...
        'coder.extrinsic(''evalin'');\n' ...
        'hard_iron = zeros(3,1);\n' ...
        'soft_iron = zeros(3,3);\n' ...
        'axis_flip = zeros(3,1);\n' ...
        'hard_iron = evalin(''base'', ''Mag.HardIron_uT'');\n' ...
        'soft_iron = evalin(''base'', ''Mag.SoftIron'');\n' ...
        'axis_flip = evalin(''base'', ''Mag.AxisFlipSign'');\n' ...
        'mag_raw_body_uT = casper_mag_distort_field(mag_clean_body_uT, ...\n' ...
        '    hard_iron, soft_iron, axis_flip);\n' ...
        'end\n']);
end

function s = mag_noise_script()
    s = sprintf([ ...
        'function [mag_noisy_uT, mag_raw_18bit] = MagNoise(mag_in_uT)\n' ...
        '%%#codegen\n' ...
        'coder.extrinsic(''evalin'');\n' ...
        'dt_s          = 0.01;\n' ...
        'tau_s         = 0.16;\n' ...
        'sigma_white   = 0.5;\n' ...
        'scale_cpg     = 16384;\n' ...
        'offset_counts = 131072;\n' ...
        'seed          = uint32(0);\n' ...
        'dt_s          = evalin(''base'', ''1/Mag.Rate_Hz'');\n' ...
        'tau_s         = evalin(''base'', ''Mag.NoiseTauSec'');\n' ...
        'sigma_white   = evalin(''base'', ''Mag.NoiseStd_uT'');\n' ...
        'scale_cpg     = evalin(''base'', ''Mag.ScaleCountsPerGauss'');\n' ...
        'offset_counts = evalin(''base'', ''Mag.OffsetCounts'');\n' ...
        'seed          = uint32(evalin(''base'', ''Sim.Seed + 3''));\n' ...
        'mag_noisy_uT  = zeros(3,1);\n' ...
        'mag_raw_18bit = uint32(zeros(3,1));\n' ...
        '[mag_noisy_uT, mag_raw_18bit] = casper_mag_noise(mag_in_uT, ...\n' ...
        '    dt_s, tau_s, sigma_white, scale_cpg, offset_counts, seed);\n' ...
        'end\n']);
end

function s = tx_schedule_script()
    s = sprintf([ ...
        'function tx_active = RadioTxSchedule(time_s)\n' ...
        '%%#codegen\n' ...
        'coder.extrinsic(''evalin'');\n' ...
        'tx_period_s  = 0.1;\n' ...
        'tx_airtime_s = 0.015;\n' ...
        'tx_period_s  = evalin(''base'', ''Mag.RadioTXPeriod_s'');\n' ...
        'tx_airtime_s = evalin(''base'', ''Mag.RadioTXAirtime_s'');\n' ...
        'tx_active = casper_radio_tx_schedule(time_s, tx_period_s, tx_airtime_s);\n' ...
        'end\n']);
end

function s = mag_interference_script()
    s = sprintf([ ...
        'function mag_out_uT = MagRadioInterference(mag_in_uT, tx_active)\n' ...
        '%%#codegen\n' ...
        'coder.extrinsic(''evalin'');\n' ...
        'interf_active = false;\n' ...
        'spike_amp_uT  = 10.0;\n' ...
        'seed          = uint32(0);\n' ...
        'interf_active = logical(evalin(''base'', ''Mag.RadioInterfActive''));\n' ...
        'spike_amp_uT  = evalin(''base'', ''Mag.RadioSpikeAmp_uT'');\n' ...
        'seed          = uint32(evalin(''base'', ''Sim.Seed + 7''));\n' ...
        'mag_out_uT = casper_mag_radio_interference(mag_in_uT, tx_active, ...\n' ...
        '    interf_active, spike_amp_uT, seed);\n' ...
        'end\n']);
end
