function lib_path = build_gps_block_visual(varargin)
%BUILD_GPS_BLOCK_VISUAL Construct the visual T06 GPS library using gpsSensor.
%
% Synopsis:
%   lib_path = build_gps_block_visual()
%   lib_path = build_gps_block_visual('LibPath', '/abs/path/gps_block_visual.slx')
%
% Builds gps_block_visual.slx containing one named subsystem:
%   - gps_visual_block (SensorInputBus -> lat_deg7 int32, lon_deg7 int32,
%                                         alt_mm int32, vel_n_mms int32,
%                                         vel_e_mms int32, vel_d_mms int32,
%                                         fix uint8, sv uint8, data_ready bool)
%
% The subsystem wires SensorInputBus through a Bus Selector, Rate Transitions
% to 1/GPS.Rate_Hz (= 10 Hz), a MATLAB Function block that calls gpsSensor
% (Sensor Fusion Tbx) via casper_gps_step_visual, and a final MATLAB Function
% block that applies the firmware quirks (COCOM gating, last-valid hold,
% 100 ms latency, int32 NAV-PVT encoding) via casper_gps_quirks.
%
% Returns:
%   lib_path : char absolute path to the .slx library.
%
% Source references:
%   - ARCHITECTURE.md §5 (stock-block intent)
%   - casper_gps_step_visual.m, casper_gps_quirks.m
%   - casper_build_unified_buses.m (SensorInputBus / GPSOutputBus defs)

    p = inputParser();
    addParameter(p, 'LibPath', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    lib_path = char(p.Results.LibPath);

    here = fileparts(mfilename('fullpath'));
    if isempty(lib_path)
        lib_path = fullfile(here, 'gps_block_visual.slx');
    end
    lib_path = char(java.io.File(lib_path).getCanonicalPath());

    lib_name = 'gps_block_visual';

    % Path setup: sensors/gps + shared (for casper_sim_config) + truth + params.
    addpath(here);
    simroot = fileparts(fileparts(here));     % .../Simulink Development
    addpath(fullfile(simroot, 'shared'));
    addpath(fullfile(simroot, 'truth'));
    addpath(fullfile(simroot, 'params'));

    % Ensure base workspace has Sim, GPS, SensorInputBus, GPSOutputBus, etc.
    % casper_sim_config also calls casper_build_unified_buses + (importantly)
    % loads the T02 sensor params. We additionally augment the GPS struct
    % here with the T06-local launch-site origin + per-axis sigmas, exactly
    % as the legacy block does.
    casper_sim_config('Seed', 20260519, 'StopTime', 5.0);
    augment_gps_struct_();

    if bdIsLoaded(lib_name)
        close_system(lib_name, 0);
    end
    if isfile(lib_path)
        delete(lib_path);
    end
    new_system(lib_name, 'Library');
    load_system(lib_name);
    set_param(lib_name, 'Lock', 'off');

    build_gps_visual_subsystem_(lib_name);

    set_param(lib_name, 'Lock', 'on');
    save_system(lib_name, lib_path);
    close_system(lib_name, 0);

    fprintf('[build_gps_block_visual] wrote %s\n', lib_path);
end


% =====================================================================
function augment_gps_struct_()
% Ensure the base-workspace GPS struct carries launch-site origin and
% derived sigma fields used by casper_gps_step_visual. Mirrors what
% casper_gps_local_params produced for the legacy block (which targets a
% separate GPS_local struct); we collapse that onto GPS itself so the
% gpsSensor constructor can find every field on a single struct.

    GPSp = evalin('base', 'GPS');
    if ~isfield(GPSp, 'LaunchLat_deg')
        GPSp.LaunchLat_deg = 51.5074;   % London placeholder
    end
    if ~isfield(GPSp, 'LaunchLon_deg')
        GPSp.LaunchLon_deg = -0.1278;
    end
    if ~isfield(GPSp, 'LaunchAlt_m')
        GPSp.LaunchAlt_m = 35.0;
    end
    if ~isfield(GPSp, 'ReacquireTime_s')
        GPSp.ReacquireTime_s = 1.0;
    end
    assignin('base', 'GPS', GPSp);
end


% =====================================================================
function build_gps_visual_subsystem_(lib_name)
% MAX-M10M visual subsystem: SensorInputBus -> gpsSensor -> quirks -> outputs.

    sub = [lib_name '/gps_visual_block'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [80 60 720 480]);

    % Inport: SensorInputBus
    in = [sub '/truth_bus'];
    add_block('built-in/Inport', in);
    set_param(in, 'Position', [20 140 50 160], ...
        'OutDataTypeStr', 'Bus: SensorInputBus', 'BusObject', 'SensorInputBus');

    % Bus Selector — extract pos_NED, vel_NED, time_s.
    bs = [sub '/BusSelect'];
    add_block('simulink/Signal Routing/Bus Selector', bs);
    set_param(bs, 'OutputSignals', 'pos_NED,vel_NED,time_s');
    set_param(bs, 'Position', [90 100 130 200]);
    add_line(sub, 'truth_bus/1', 'BusSelect/1', 'autorouting', 'on');

    % Rate Transitions to 1/GPS.Rate_Hz (= 10 Hz)
    rt_dt = '1/GPS.Rate_Hz';
    rt_names = {'RT_pos', 'RT_vel', 'RT_time'};
    for k = 1:3
        rp = [sub '/' rt_names{k}];
        add_block('simulink/Signal Attributes/Rate Transition', rp);
        set_param(rp, 'OutPortSampleTime', rt_dt, ...
            'Position', [170 (90 + (k-1)*40) 220 (110 + (k-1)*40)]);
        add_line(sub, ['BusSelect/' num2str(k)], [rt_names{k} '/1'], 'autorouting', 'on');
    end

    % MATLAB Function: gpsSensor step wrapper
    mf = [sub '/gpsSensor_step'];
    add_block('simulink/User-Defined Functions/MATLAB Function', mf);
    set_param(mf, 'Position', [260 60 440 220]);
    set_matlab_fn_script_(mf, [ ...
        'function [lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps] = fcn(' ...
            'pos_NED, vel_NED, seed_base, reset_flag)' newline ...
        'coder.extrinsic(''casper_gps_step_visual'');' newline ...
        'lat_deg = 0.0; lon_deg = 0.0; alt_m = 0.0;' newline ...
        'vn_mps  = 0.0; ve_mps  = 0.0; vd_mps  = 0.0;' newline ...
        '[lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps, ~] = ' ...
            'casper_gps_step_visual(' ...
            'pos_NED(:).'', vel_NED(:).'', ' ...
            'double(seed_base), logical(reset_flag));' newline ...
        'end' newline]);

    add_line(sub, 'RT_pos/1',  'gpsSensor_step/1', 'autorouting', 'on');
    add_line(sub, 'RT_vel/1',  'gpsSensor_step/2', 'autorouting', 'on');

    % Constants for seed and reset
    c_seed = [sub '/C_Seed'];
    add_block('simulink/Sources/Constant', c_seed);
    set_param(c_seed, 'Value', 'Sim.Seed + 5', 'SampleTime', rt_dt, ...
        'Position', [170 220 220 240]);
    add_line(sub, 'C_Seed/1', 'gpsSensor_step/3', 'autorouting', 'on');

    c_reset = [sub '/C_Reset'];
    add_block('simulink/Sources/Constant', c_reset);
    set_param(c_reset, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', rt_dt, 'Position', [170 250 220 270]);
    add_line(sub, 'C_Reset/1', 'gpsSensor_step/4', 'autorouting', 'on');

    % --- Truth-side scalars for COCOM gate ---
    % v_total = norm(vel_NED); alt_total = -pos_NED(3).
    % We compute both via small Math Function / Selector / Gain chain.

    % v_total via Math Function (magnitude^2 sqrt) using Sum-of-Squares + sqrt.
    sqr = [sub '/VelSquare'];
    add_block('simulink/Math Operations/Math Function', sqr);
    set_param(sqr, 'Operator', 'square', 'Position', [260 290 310 320]);
    add_line(sub, 'RT_vel/1', 'VelSquare/1', 'autorouting', 'on');

    sum_sq = [sub '/SumOfSq'];
    add_block('simulink/Math Operations/Sum of Elements', sum_sq);
    set_param(sum_sq, 'Position', [330 290 360 320]);
    add_line(sub, 'VelSquare/1', 'SumOfSq/1', 'autorouting', 'on');

    sqrt_blk = [sub '/Vmag'];
    add_block('simulink/Math Operations/Sqrt', sqrt_blk);
    set_param(sqrt_blk, 'Position', [380 290 410 320]);
    add_line(sub, 'SumOfSq/1', 'Vmag/1', 'autorouting', 'on');

    % alt_total = -pos_NED(3): Selector(3) -> Gain(-1).
    sel_z = [sub '/PosZSelect'];
    add_block('simulink/Signal Routing/Selector', sel_z);
    set_param(sel_z, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', ...
        'Indices', '3', 'InputPortWidth', '3', ...
        'Position', [260 340 310 370]);
    add_line(sub, 'RT_pos/1', 'PosZSelect/1', 'autorouting', 'on');

    neg = [sub '/NegZ'];
    add_block('simulink/Math Operations/Gain', neg);
    set_param(neg, 'Gain', '-1', 'Position', [330 340 380 370]);
    add_line(sub, 'PosZSelect/1', 'NegZ/1', 'autorouting', 'on');

    % --- MATLAB Function: PostQuirks (COCOM gate + hold + latency + int32) ---
    pq = [sub '/PostQuirks'];
    add_block('simulink/User-Defined Functions/MATLAB Function', pq);
    set_param(pq, 'Position', [490 40 690 440]);
    set_matlab_fn_script_(pq, [ ...
        'function [lat_deg7, lon_deg7, alt_mm, vel_n_mms, vel_e_mms, ' ...
            'vel_d_mms, fix, sv, data_ready] = fcn(' ...
            'lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps, ' ...
            'v_total, alt_total, time_s, ' ...
            'cocom_v, cocom_a, reacq_s)' newline ...
        'coder.extrinsic(''casper_gps_quirks'');' newline ...
        'lat_deg7 = int32(0); lon_deg7 = int32(0); alt_mm = int32(0);' newline ...
        'vel_n_mms = int32(0); vel_e_mms = int32(0); vel_d_mms = int32(0);' newline ...
        'fix = uint8(0); sv = uint8(0); data_ready = false;' newline ...
        '[lat_deg7, lon_deg7, alt_mm, vel_n_mms, vel_e_mms, vel_d_mms, ' ...
            'fix, sv, data_ready] = casper_gps_quirks(' ...
            'double(lat_deg), double(lon_deg), double(alt_m), ' ...
            'double(vn_mps), double(ve_mps), double(vd_mps), ' ...
            'double(v_total), double(alt_total), double(time_s), ' ...
            'double(cocom_v), double(cocom_a), double(reacq_s));' newline ...
        'end' newline]);

    % Wire gpsSensor outputs (6 scalars) to PostQuirks inputs 1..6.
    for k = 1:6
        add_line(sub, ['gpsSensor_step/' num2str(k)], ...
                      ['PostQuirks/' num2str(k)], 'autorouting', 'on');
    end
    % v_total (port 7), alt_total (port 8), time_s (port 9).
    add_line(sub, 'Vmag/1',    'PostQuirks/7', 'autorouting', 'on');
    add_line(sub, 'NegZ/1',    'PostQuirks/8', 'autorouting', 'on');
    add_line(sub, 'RT_time/1', 'PostQuirks/9', 'autorouting', 'on');

    % COCOM thresholds + reacquire constants on ports 10/11/12.
    quirk_consts = { ...
        {'C_CocomV',  'GPS.COCOMVelThreshold_mps', 10}, ...
        {'C_CocomA',  'GPS.COCOMAltThreshold_m',   11}, ...
        {'C_Reacq',   'GPS.ReacquireTime_s',       12} };
    y0 = 380;
    for k = 1:numel(quirk_consts)
        nm = quirk_consts{k}{1};
        val = quirk_consts{k}{2};
        port = quirk_consts{k}{3};
        cp = [sub '/' nm];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', rt_dt, ...
            'Position', [340 (y0 + (k-1)*25) 410 (y0 + (k-1)*25 + 20)]);
        add_line(sub, [nm '/1'], ['PostQuirks/' num2str(port)], 'autorouting', 'on');
    end

    % Outports — match GPSOutputBus field order in casper_build_unified_buses.
    outs = { ...
        {'lat_deg7',   'PostQuirks/1', 'int32'}, ...
        {'lon_deg7',   'PostQuirks/2', 'int32'}, ...
        {'alt_mm',     'PostQuirks/3', 'int32'}, ...
        {'vel_n_mms',  'PostQuirks/4', 'int32'}, ...
        {'vel_e_mms',  'PostQuirks/5', 'int32'}, ...
        {'vel_d_mms',  'PostQuirks/6', 'int32'}, ...
        {'fix',        'PostQuirks/7', 'uint8'}, ...
        {'sv',         'PostQuirks/8', 'uint8'}, ...
        {'data_ready', 'PostQuirks/9', 'boolean'} };
    for k = 1:numel(outs)
        op = [sub '/' outs{k}{1}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [720 (40 + (k-1)*45) 750 (60 + (k-1)*45)]);
        add_line(sub, outs{k}{2}, [outs{k}{1} '/1'], 'autorouting', 'on');
    end
end


% =====================================================================
function set_matlab_fn_script_(block_path, src)
% SET_MATLAB_FN_SCRIPT_ Install the script body of a MATLAB Function block.
    sf = sfroot;
    blk = sf.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(blk)
        error('build_gps_block_visual:NoChart', ...
            'Could not find MATLAB Function chart at %s', block_path);
    end
    blk.Script = src;
end
