function model_path = build_attitude_block()
%BUILD_ATTITUDE_BLOCK  Programmatic Simulink subsystem for the attitude port.
%
%   Constructs `attitude_block.slx` in the T09 build directory.  The model
%   contains a single MATLAB Function block that wraps casper_attitude_tick.
%   This artifact lives in T09's own dir and DOES NOT modify
%   `T01_truth_pipeline/casper_sim_lib.slx`.
%
%   The model is bus-friendly: scalar+vector inports for sim use, struct
%   state pass-through is handled inside the MATLAB Function via a data
%   store memory in T11 integration; for the standalone block we use a
%   simple "reset on first call" pattern.
%
%   Returns:
%     model_path  absolute path to the produced .slx

    this_dir = fileparts(mfilename('fullpath'));
    model_name = 'attitude_block';
    model_path = fullfile(this_dir, [model_name, '.slx']);

    % Close & remove existing
    if bdIsLoaded(model_name)
        bdclose(model_name);
    end
    if exist(model_path, 'file')
        delete(model_path);
    end

    new_system(model_name);
    set_param(model_name, 'Solver',          'FixedStepDiscrete');
    set_param(model_name, 'FixedStep',       num2str(1/833));
    set_param(model_name, 'StopTime',        '1.0');
    set_param(model_name, 'SaveFormat',      'Dataset');
    set_param(model_name, 'SaveOutput',      'on');
    set_param(model_name, 'StartTime',       '0');

    % ── Inports ─────────────────────────────────────────────────────────
    ports = {
        'accel_body_fw_mps2', 3;
        'gyro_body_fw_radps', 3;
        'mag_body_fw_uT',     3;
        'mag_new_sample',     1;
        'mode_pad',           1};
    for k = 1:size(ports,1)
        nm = ports{k,1};
        w  = ports{k,2};
        add_block('built-in/Inport', [model_name, '/', nm]);
        set_param([model_name,'/',nm], ...
            'Port', num2str(k), ...
            'PortDimensions', num2str(w), ...
            'SampleTime', num2str(1/833));
    end

    % ── MATLAB Function block ───────────────────────────────────────────
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [model_name, '/attitude_tick']);

    sf_script = strjoin({
        'function [quat, gyro_bias, heading_sigma, init_complete] = attitude_tick(accel, gyro, mag, mag_new, mode_pad)' ...
        '%#codegen' ...
        '    persistent st p' ...
        '    if isempty(st)' ...
        '        st = casper_attitude_state_new();' ...
        '        % Pull params from base workspace at first call' ...
        '        p = evalin(''base'', ''Attitude'');' ...
        '        if ~isfield(p, ''M_ref_nav_uT'')' ...
        '            p.M_ref_nav_uT = [0; 0; 50];   % default Z-up Earth field, uT' ...
        '        end' ...
        '    end' ...
        '    [att, st] = casper_attitude_tick(accel(:), gyro(:), mag(:), logical(mag_new), logical(mode_pad), 1/833, p, st);' ...
        '    quat          = att.quat_body_to_nav;' ...
        '    gyro_bias     = att.gyro_bias_radps;' ...
        '    heading_sigma = att.heading_sigma_rad;' ...
        '    init_complete = att.init_complete;' ...
        'end'}, sprintf('\n'));
    setMATLABFunctionScript([model_name,'/attitude_tick'], sf_script);

    % ── Outports ────────────────────────────────────────────────────────
    outs = {'quat', 'gyro_bias', 'heading_sigma', 'init_complete'};
    for k = 1:numel(outs)
        nm = outs{k};
        add_block('built-in/Outport', [model_name, '/', nm]);
        set_param([model_name,'/',nm], 'Port', num2str(k));
    end

    % ── Wiring ──────────────────────────────────────────────────────────
    for k = 1:size(ports,1)
        add_line(model_name, [ports{k,1}, '/1'], ['attitude_tick/', num2str(k)], ...
            'autorouting','on');
    end
    for k = 1:numel(outs)
        add_line(model_name, ['attitude_tick/', num2str(k)], [outs{k}, '/1'], ...
            'autorouting','on');
    end

    try
        Simulink.BlockDiagram.arrangeSystem(model_name);
    catch
        % ignore cosmetic failures
    end

    save_system(model_name, model_path);
    bdclose(model_name);
end

% ---------------------------------------------------------------------------
function setMATLABFunctionScript(block_path, script_text)
    % Set the script of a MATLAB Function block programmatically.
    root = sfroot();
    chart = root.find('-isa','Stateflow.EMChart','Path',block_path);
    if isempty(chart)
        error('build_attitude_block:noChart', ...
              'Could not find MATLAB Function block at %s', block_path);
    end
    chart.Script = script_text;
end
