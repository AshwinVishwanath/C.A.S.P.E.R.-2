function build_eskf_block(out_dir)
%BUILD_ESKF_BLOCK Programmatically construct a Simulink subsystem wrapping
% the stripped 4-state vertical ESKF.
%
% Synopsis:
%   build_eskf_block()                  % writes ./eskf_block.slx
%   build_eskf_block(out_dir)           % writes <out_dir>/eskf_block.slx
%
% Output:
%   eskf_block.slx -- subsystem with a single MATLAB Function block
%   running the predict + update steps. Used by T10/T11 wiring.
%
% Notes:
%   - This is a thin wrapper around the four casper_eskf_* MATLAB functions.
%   - This script ONLY writes into its own task build directory. It does not
%     touch T01_truth_pipeline/casper_sim_lib.slx (which would violate the
%     T08 hard constraints on not polluting the shared library).
%
% Source firmware reference:
%   Software/App/nav/casper_ekf.c (entire file)

    if nargin < 1 || isempty(out_dir)
        out_dir = fileparts(mfilename('fullpath'));
    end

    model_name = 'eskf_block';
    model_path = fullfile(out_dir, [model_name '.slx']);

    % Close any open instance, delete the existing file.
    if bdIsLoaded(model_name)
        bdclose(model_name);
    end
    if exist(model_path, 'file')
        delete(model_path);
    end

    new_system(model_name);
    set_param(model_name, 'Solver',         'FixedStepDiscrete', ...
                          'SolverType',     'Fixed-step', ...
                          'FixedStep',      '0.0024', ...
                          'StopTime',       '5');

    % --- Inports ---
    add_block('built-in/Inport',  [model_name '/accel_nav_up_mps2']);
    set_param([model_name '/accel_nav_up_mps2'], 'Port', '1', 'SampleTime', '0.0024');

    add_block('built-in/Inport',  [model_name '/baro_alt_m']);
    set_param([model_name '/baro_alt_m'], 'Port', '2', 'SampleTime', '0.01');

    add_block('built-in/Inport',  [model_name '/baro_new_sample']);
    set_param([model_name '/baro_new_sample'], 'Port', '3', 'SampleTime', '0.01');

    add_block('built-in/Inport',  [model_name '/mach']);
    set_param([model_name '/mach'], 'Port', '4', 'SampleTime', '0.0024');

    add_block('built-in/Inport',  [model_name '/zupt_trigger']);
    set_param([model_name '/zupt_trigger'], 'Port', '5', 'SampleTime', '0.0024');

    % --- MATLAB Function block ---
    fb_path = [model_name '/eskf_step'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fb_path);

    fcn_text = sprintf(['function [state_x, P_diag, mach_gate_active, ungate_counter, ' ...
        'last_baro_innov, last_zupt_innov] = eskf_step(accel_nav_up, baro_alt, baro_new, mach_in, zupt_trig)\n' ...
        '%% Persistent ESKF state container. Reset via subsystem Init callback.\n' ...
        'persistent st Est\n' ...
        'if isempty(st)\n' ...
        '    casper_sensor_params; %%#ok<NASGU>\n' ...
        '    Est = evalin(''base'', ''Estimator'');\n' ...
        '    st = casper_eskf_state(''init'', Est, baro_alt);\n' ...
        'end\n' ...
        '%% Predict (every call)\n' ...
        'st = casper_eskf_predict(st, accel_nav_up, Est);\n' ...
        '%% Mach gate (every call)\n' ...
        'st = casper_eskf_mach_gate(st, mach_in, Est);\n' ...
        '%% Baro update (when new baro sample)\n' ...
        'if baro_new\n' ...
        '    st = casper_eskf_update_baro(st, baro_alt, Est);\n' ...
        'end\n' ...
        '%% ZUPT update\n' ...
        'if zupt_trig\n' ...
        '    st = casper_eskf_update_zupt(st, Est);\n' ...
        'end\n' ...
        'state_x = st.x_vec;\n' ...
        'P_diag = diag(st.P_mat);\n' ...
        'mach_gate_active = double(st.mach_gate_active);\n' ...
        'ungate_counter = double(st.ungate_counter);\n' ...
        'last_baro_innov = st.last_baro_innov_m;\n' ...
        'last_zupt_innov = st.last_zupt_innov_mps;\n' ...
        'end\n']);

    % Set the MATLAB Function block script via the Stateflow API.
    sf_root = sfroot;
    chart_obj = sf_root.find('-isa', 'Stateflow.EMChart', 'Path', fb_path);
    if isempty(chart_obj)
        % Some Simulink versions return an array; try alternate query.
        chart_obj = find(sf_root, '-isa', 'Stateflow.EMChart');
        if ~isempty(chart_obj)
            % Filter to our block path
            paths = {chart_obj.Path};
            mask = strcmp(paths, fb_path);
            chart_obj = chart_obj(mask);
        end
    end
    if isempty(chart_obj)
        error('build_eskf_block:NoChart', ...
            'Could not find Stateflow chart for MATLAB Function block at %s', fb_path);
    end
    chart_obj.Script = fcn_text;

    % --- Outports ---
    add_block('built-in/Outport', [model_name '/state_x']);
    add_block('built-in/Outport', [model_name '/P_diag']);
    add_block('built-in/Outport', [model_name '/mach_gate_active']);
    add_block('built-in/Outport', [model_name '/ungate_counter']);
    add_block('built-in/Outport', [model_name '/last_baro_innov']);
    add_block('built-in/Outport', [model_name '/last_zupt_innov']);

    % --- Wire ---
    add_line(model_name, 'accel_nav_up_mps2/1', 'eskf_step/1');
    add_line(model_name, 'baro_alt_m/1',        'eskf_step/2');
    add_line(model_name, 'baro_new_sample/1',   'eskf_step/3');
    add_line(model_name, 'mach/1',              'eskf_step/4');
    add_line(model_name, 'zupt_trigger/1',      'eskf_step/5');

    add_line(model_name, 'eskf_step/1', 'state_x/1');
    add_line(model_name, 'eskf_step/2', 'P_diag/1');
    add_line(model_name, 'eskf_step/3', 'mach_gate_active/1');
    add_line(model_name, 'eskf_step/4', 'ungate_counter/1');
    add_line(model_name, 'eskf_step/5', 'last_baro_innov/1');
    add_line(model_name, 'eskf_step/6', 'last_zupt_innov/1');

    try
        Simulink.BlockDiagram.arrangeSystem(model_name);
    catch
        % Auto-arrange is cosmetic; ignore failures.
    end

    save_system(model_name, model_path);
    bdclose(model_name);
    fprintf('[T08] Saved %s\n', model_path);
end
