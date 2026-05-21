function result = test_visual_model_compile()
%TEST_VISUAL_MODEL_COMPILE Smoke-test casper_sim_phase0.slx compiles and runs.
%
% Synopsis:
%   result = test_visual_model_compile()
%
% Steps:
%   1. Build casper_sim_phase0.slx via build_casper_sim_phase0.m.
%   2. Populate base workspace with a 5 s stationary 'truth_ts' so the
%      truth_source's From Workspace blocks have something to read.
%   3. set_param('SimulationCommand', 'update') — verifies the diagram
%      updates without warnings/errors.
%   4. sim(...) for 5 s with PadOnly stationary truth.
%   5. Verify all logged To Workspace variables are finite.
%
% PASS criterion: model compiles AND 5 s sim completes AND all logged
% signals are finite.
%
% Output:
%   result : struct with .pass logical, .checks cell, .runtime_s

    here = fileparts(mfilename('fullpath'));
    addpath(here);

    fprintf('==== test_visual_model_compile ====\n');

    result = struct('pass', false, 'checks', {{}}, 'runtime_s', 0);

    % --- Step 1: build the model -----------------------------------------
    model_path = build_casper_sim_phase0();
    [~, mdl, ~] = fileparts(model_path);

    if ~isfile(model_path)
        error('test_visual_model_compile:NoFile', ...
            'Model file missing after build: %s', model_path);
    end
    fprintf('  [ok] model built: %s\n', model_path);
    result.checks{end+1} = {'model file exists', true, model_path};

    % --- Step 2: populate truth_ts in base WS (5 s stationary) ----------
    % casper_sim_config (called inside the builder) already set up the
    % buses and params. We need to add a struct of timeseries that the
    % From Workspace blocks read.
    stop_time_s = 5.0;
    dt_truth    = 1e-3;   % match the model's base step
    N = round(stop_time_s / dt_truth) + 1;
    t = (0:N-1)' * dt_truth;
    zero3 = zeros(N, 3);
    quat0 = repmat([1 0 0 0], N, 1);

    truth_ts = struct();
    truth_ts.pos_NED          = timeseries(zero3,              t, 'Name', 'pos_NED');
    truth_ts.vel_NED          = timeseries(zero3,              t, 'Name', 'vel_NED');
    truth_ts.accel_NED        = timeseries(zero3,              t, 'Name', 'accel_NED');
    truth_ts.quat_std         = timeseries(quat0,              t, 'Name', 'quat_std');
    truth_ts.omega_body_std   = timeseries(zero3,              t, 'Name', 'omega_body_std');
    truth_ts.time_s           = timeseries(t,                  t, 'Name', 'time_s');
    truth_ts.mach             = timeseries(zeros(N, 1),        t, 'Name', 'mach');
    truth_ts.air_density_kgm3 = timeseries(1.225 * ones(N, 1), t, 'Name', 'air_density_kgm3');
    truth_ts.air_temp_K       = timeseries(288.15 * ones(N, 1),t, 'Name', 'air_temp_K');
    truth_ts.air_pressure_pa  = timeseries(101325 * ones(N, 1),t, 'Name', 'air_pressure_pa');
    assignin('base', 'truth_ts', truth_ts);
    fprintf('  [ok] truth_ts populated in base WS (N=%d, %.1f s)\n', N, stop_time_s);

    % --- Step 3: update diagram (compile) --------------------------------
    if ~bdIsLoaded(mdl); load_system(model_path); end
    set_param(mdl, 'StopTime', num2str(stop_time_s));

    try
        fprintf('  [..] updating diagram (compile-only)...\n');
        set_param(mdl, 'SimulationCommand', 'update');
        fprintf('  [ok] update OK.\n');
        result.checks{end+1} = {'diagram updates without error', true, ''};
    catch ME
        fprintf('  [FAIL] update failed: %s\n', ME.message);
        dump_exception_(ME, 0);
        result.checks{end+1} = {'diagram updates without error', false, ME.message};
        result.pass = false;
        return;
    end

    % --- Step 4: run 5 s sim ---------------------------------------------
    t_start = tic;
    simOut = []; %#ok<NASGU> overwritten on success; placeholder for catch path
    try
        fprintf('  [..] running 5 s sim...\n');
        simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
        result.runtime_s = toc(t_start);
        fprintf('  [ok] sim OK (wall: %.2f s)\n', result.runtime_s);
        result.checks{end+1} = {'5 s sim completes', true, ...
            sprintf('wall=%.2f s', result.runtime_s)};
    catch ME
        result.runtime_s = toc(t_start);
        fprintf('  [FAIL] sim failed: %s\n', ME.message);
        result.checks{end+1} = {'5 s sim completes', false, ME.message};
        result.pass = false;
        if bdIsLoaded(mdl); close_system(mdl, 0); end
        return;
    end

    % --- Step 5: verify logged signals finite ----------------------------
    % To-Workspace blocks write into simOut (because the model has
    % ReturnWorkspaceOutputs='on'). Each variable is a struct-with-time.
    expected_logs = { ...
        'log_truth_bus', 'log_est_state_x', 'log_est_state_P_diag', ...
        'log_est_quat', 'log_est_mach_gate_active', 'log_est_ungate_counter', ...
        'log_est_baro_innov', 'log_est_zupt_innov', ...
        'log_sensor_imu', 'log_sensor_adxl', 'log_sensor_baro', ...
        'log_sensor_mag', 'log_sensor_gps', 'log_radio_tx_active'};
    all_finite = true;
    available_names = simOut.who;
    for k = 1:numel(expected_logs)
        nm = expected_logs{k};
        if ismember(nm, available_names)
            v = simOut.get(nm);
            data = extract_data_(v);
            if isempty(data)
                fprintf('  [WARN] %s exists but is empty\n', nm);
            else
                if isnumeric(data)
                    fin = all(isfinite(data(:)));
                else
                    fin = true;     % boolean / int -> trivially "finite"
                end
                if ~fin
                    all_finite = false;
                    fprintf('  [FAIL] %s has non-finite values\n', nm);
                else
                    fprintf('  [ok] %s finite (%s)\n', nm, mat2str(size(data)));
                end
            end
        else
            all_finite = false;
            fprintf('  [WARN] %s not found in simOut\n', nm);
        end
    end
    result.checks{end+1} = {'all logged signals finite', all_finite, ''};

    % --- Tally -----------------------------------------------------------
    pass = all(cellfun(@(c) c{2}, result.checks));
    result.pass = pass;

    fprintf('\n==== Result: %s ====\n', ternary_(pass, 'PASS', 'FAIL'));
    for k = 1:numel(result.checks)
        c = result.checks{k};
        fprintf('  [%s] %s -- %s\n', ternary_(c{2}, 'PASS', 'FAIL'), c{1}, c{3});
    end

    if bdIsLoaded(mdl); close_system(mdl, 0); end
end


% =====================================================================
function data = extract_data_(v)
% Extract the underlying numeric data from a To-Workspace logged var.
% StructureWithTime: v.signals(k).values (concatenated channels)
    data = [];
    if isnumeric(v) || islogical(v)
        data = v;
        return;
    end
    if isstruct(v) && isfield(v, 'signals')
        sigs = v.signals;
        all_vals = cell(numel(sigs), 1);
        for k = 1:numel(sigs)
            all_vals{k} = sigs(k).values(:);
        end
        try
            data = vertcat(all_vals{:});
        catch
            data = [];
        end
        return;
    end
    if isa(v, 'timeseries')
        data = v.Data;
        return;
    end
end


function s = ternary_(c, a, b)
    if c; s = a; else; s = b; end
end


function dump_exception_(e, depth)
% Recursively print an MException cause chain.
    prefix = repmat('  ', 1, depth + 2);
    fprintf('%sMSG: %s\n%sID:  %s\n', prefix, e.message, prefix, e.identifier);
    if isprop(e, 'cause') && ~isempty(e.cause)
        for k = 1:length(e.cause)
            fprintf('%s-- CAUSE %d --\n', prefix, k);
            dump_exception_(e.cause{k}, depth + 1);
        end
    end
end
