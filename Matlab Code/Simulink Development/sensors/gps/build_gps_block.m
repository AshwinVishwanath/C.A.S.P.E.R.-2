function model_path = build_gps_block(varargin)
%BUILD_GPS_BLOCK Programmatically construct the gps_block Simulink library.
%
% Synopsis:
%   model_path = build_gps_block()
%   model_path = build_gps_block('ModelPath', '/abs/path/gps_block.slx')
%
% Creates (or rebuilds) a self-contained Simulink library at
% <T06 build dir>/gps_block.slx containing one subsystem `gps_block` with
% the signal flow described in T06 §4:
%
%   truth_bus
%      |
%      v  Inports: pos_NED (3x1), vel_NED (3x1), time_s (1)
%      |
%      v  Rate Transition (10 kHz -> 10 Hz)
%      |
%      v  MATLAB Function: GpsStep (extrinsic call to casper_gps_step)
%      |
%      v  Outports: lat_deg7, lon_deg7, alt_msl_mm,
%                   vel_n_mm_s, vel_e_mm_s, vel_d_mm_s   (all int32)
%                   fix_type, num_sv                     (uint8)
%                   data_ready                           (bool)
%
% All numerical parameters are bound at runtime to base-workspace variables
% GPS_local.* and Sim.* per SIMULINK_PATTERNS.md §9. Run
% `casper_sensor_params` and `casper_gps_local_params` before opening this
% model.
%
% The model is saved as a Library (locked). Downstream models reference
% the subsystem via library link.
%
% Output:
%   model_path : char, absolute path to the saved .slx library.
%
% Source firmware reference:
%   None (sim-side construction wrapper for MAX-M10M NAV-PVT producer).

    p = inputParser();
    addParameter(p, 'ModelPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    model_path = char(p.Results.ModelPath);

    here = fileparts(mfilename('fullpath'));
    if isempty(model_path)
        model_path = fullfile(here, 'gps_block.slx');
    end

    lib_name = 'gps_block';
    sub_name = 'gps_block';

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
    set_param(sub_path, 'Position', [80 60 540 540]);

    % ---------- subsystem inports ----------
    add_block('built-in/Inport',  [sub_path '/pos_NED']);
    set_param([sub_path '/pos_NED'], 'Position', [30  30  60  50],  'PortDimensions', '3');
    add_block('built-in/Inport',  [sub_path '/vel_NED']);
    set_param([sub_path '/vel_NED'], 'Position', [30  80  60 100],  'PortDimensions', '3');
    add_block('built-in/Inport',  [sub_path '/time_s']);
    set_param([sub_path '/time_s'], 'Position', [30 130  60 150],  'PortDimensions', '1');

    % ---------- rate transitions (10 kHz -> 10 Hz) ----------
    add_rate_transition(sub_path, 'RT_pos',  '1/GPS.Rate_Hz');
    set_param([sub_path '/RT_pos'],  'Position', [110  25 150  55]);
    add_rate_transition(sub_path, 'RT_vel',  '1/GPS.Rate_Hz');
    set_param([sub_path '/RT_vel'],  'Position', [110  75 150 105]);
    add_rate_transition(sub_path, 'RT_time', '1/GPS.Rate_Hz');
    set_param([sub_path '/RT_time'], 'Position', [110 125 150 155]);

    % ---------- MATLAB Function: GpsStep (extrinsic wrapper) ----------
    fcn_path = [sub_path '/GpsStep'];
    add_matlab_function(fcn_path, gps_step_script());
    set_param(fcn_path, 'Position', [220  20 420 460]);

    % ---------- subsystem outports ----------
    outs = { ...
        'lat_deg7',   'int32', '1';
        'lon_deg7',   'int32', '1';
        'alt_msl_mm', 'int32', '1';
        'vel_n_mm_s', 'int32', '1';
        'vel_e_mm_s', 'int32', '1';
        'vel_d_mm_s', 'int32', '1';
        'fix_type',   'uint8', '1';
        'num_sv',     'uint8', '1';
        'data_ready', 'boolean', '1'};
    for k = 1:size(outs, 1)
        op = [sub_path '/' outs{k,1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [470  30+45*(k-1)  500  50+45*(k-1)], ...
            'PortDimensions', outs{k,3});
    end

    % ---------- wire it up ----------
    add_line(sub_path, 'pos_NED/1',  'RT_pos/1',  'autorouting', 'on');
    add_line(sub_path, 'vel_NED/1',  'RT_vel/1',  'autorouting', 'on');
    add_line(sub_path, 'time_s/1',   'RT_time/1', 'autorouting', 'on');

    add_line(sub_path, 'RT_pos/1',   'GpsStep/1', 'autorouting', 'on');
    add_line(sub_path, 'RT_vel/1',   'GpsStep/2', 'autorouting', 'on');
    add_line(sub_path, 'RT_time/1',  'GpsStep/3', 'autorouting', 'on');

    for k = 1:size(outs, 1)
        add_line(sub_path, ['GpsStep/' num2str(k)], [outs{k,1} '/1'], ...
            'autorouting', 'on');
    end

    % Lock and save.
    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, model_path);
    close_system(lib_name, 1);

    fprintf('[build_gps_block] wrote %s\n', model_path);
end

% ---------------------------------------------------------------------------
% Helpers
% ---------------------------------------------------------------------------

function add_rate_transition(parent, name, sample_time_expr)
    path = [parent '/' name];
    add_block('simulink/Signal Attributes/Rate Transition', path);
    set_param(path, ...
        'OutPortSampleTime', sample_time_expr, ...
        'Integrity',         'on');
end

function add_matlab_function(path, script_text)
% Adds a MATLAB Function block and writes the supplied Script into it.
    add_block('simulink/User-Defined Functions/MATLAB Function', path);
    root = sfroot();
    chart = find(root, '-isa', 'Stateflow.EMChart', 'Path', path);
    if isempty(chart)
        error('build_gps_block:NoChart', ...
            'Could not locate Stateflow.EMChart for %s', path);
    end
    chart = chart(1);
    chart.Script = script_text;
end

function s = gps_step_script()
% MATLAB Function block body: thin extrinsic wrapper around casper_gps_step.
% Parameters are pulled from the base workspace via evalin (matches T05
% pattern). Output types and sizes are declared explicitly so the block
% compiles even though the called function is extrinsic.
    lines = {
    'function [lat_deg7, lon_deg7, alt_msl_mm, ...'
    '          vel_n_mm_s, vel_e_mm_s, vel_d_mm_s, ...'
    '          fix_type, num_sv, data_ready] = GpsStep(pos_NED, vel_NED, time_s)'
    '%#codegen'
    'coder.extrinsic(''evalin'');'
    'coder.extrinsic(''casper_gps_step'');'
    ''
    '% Default-initialise outputs so the codegen-time types are known.'
    'lat_deg7    = int32(0);'
    'lon_deg7    = int32(0);'
    'alt_msl_mm  = int32(0);'
    'vel_n_mm_s  = int32(0);'
    'vel_e_mm_s  = int32(0);'
    'vel_d_mm_s  = int32(0);'
    'fix_type    = uint8(0);'
    'num_sv      = uint8(0);'
    'data_ready  = false;'
    ''
    '% Pull parameters from the base workspace at run time. Defaults are'
    '% only used at codegen for type inference; the evalin calls overwrite'
    '% them every simulation step.'
    'lat0_deg       = 0.0;'
    'lon0_deg       = 0.0;'
    'alt0_m         = 0.0;'
    'earth_radius_m = 6371000.0;'
    'sigma_h_m      = 0.0;'
    'sigma_v_m      = 0.0;'
    'sigma_vel_mps  = 0.0;'
    'vel_thresh_mps = 500.0;'
    'alt_thresh_m   = 18000.0;'
    'reacquire_s    = 1.0;'
    'seed           = uint32(0);'
    ''
    'lat0_deg       = evalin(''base'', ''GPS_local.LaunchLat_deg'');'
    'lon0_deg       = evalin(''base'', ''GPS_local.LaunchLon_deg'');'
    'alt0_m         = evalin(''base'', ''GPS_local.LaunchAlt_m'');'
    'earth_radius_m = evalin(''base'', ''GPS_local.EarthRadius_m'');'
    'sigma_h_m      = evalin(''base'', ''GPS_local.PositionSigmaHorizontal_m'');'
    'sigma_v_m      = evalin(''base'', ''GPS_local.PositionSigmaVertical_m'');'
    'sigma_vel_mps  = evalin(''base'', ''GPS_local.VelocitySigma_mps'');'
    'vel_thresh_mps = evalin(''base'', ''GPS_local.COCOMVelThreshold_mps'');'
    'alt_thresh_m   = evalin(''base'', ''GPS_local.COCOMAltThreshold_m'');'
    'reacquire_s    = evalin(''base'', ''GPS_local.ReacquireTime_s'');'
    'seed           = uint32(evalin(''base'', ''GPS_local.Seed''));'
    ''
    '[lat_deg7, lon_deg7, alt_msl_mm, ...'
    ' vel_n_mm_s, vel_e_mm_s, vel_d_mm_s, ...'
    ' fix_type, num_sv, data_ready] = casper_gps_step( ...'
    '    pos_NED, vel_NED, time_s, ...'
    '    lat0_deg, lon0_deg, alt0_m, earth_radius_m, ...'
    '    sigma_h_m, sigma_v_m, sigma_vel_mps, ...'
    '    vel_thresh_mps, alt_thresh_m, reacquire_s, seed);'
    'end'
    };
    s = strjoin(lines, sprintf('\n'));
end
