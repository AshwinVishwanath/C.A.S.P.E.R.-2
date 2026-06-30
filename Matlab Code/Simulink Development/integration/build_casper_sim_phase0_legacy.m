function model_path = build_casper_sim_phase0_legacy(varargin)
%BUILD_CASPER_SIM_PHASE0_LEGACY Archived MATLAB-driver-only stub.
%
% ARCHIVED 2026-05-21. The current visual-integration constructor is
% `build_casper_sim_phase0.m`. This file is kept for reference only; do
% not call it from production code. The legacy `.slx` it produced
% (casper_sim_phase0_legacy.slx) is the matching archived artifact.
%
% Original docstring below.
%
%BUILD_CASPER_SIM_PHASE0 Programmatic build of the top-level Phase 0 model.
%
% Synopsis:
%   model_path = build_casper_sim_phase0()
%   model_path = build_casper_sim_phase0('OutDir', '/abs/path')
%
% Constructs `casper_sim_phase0.slx` in T11's build directory.  The model
% is the Simulink "face" of T11; the heavy lifting (per-tick integration of
% T03..T09) lives in `casper_phase0_run.m`, which the model invokes via a
% single MATLAB Function block driven by a 10 kHz clock.  This is the
% architecturally clean way to wire 10 disparate library blocks (each with
% its own private bus signature and seed plumbing) without inventing a 7th
% incompatible bus dialect just for the integration layer.
%
% The model topology mirrors the diagram in T11_integration.md §4.1:
%   - truth_source (loaded from T01's casper_sim_lib.slx LIBRARY LINK, never copied)
%   - phase0_engine (MATLAB Function block, runs casper_phase0_run on a frozen
%                    truth window passed in via base WS variable 'truth_trajectory')
%   - one Outport per logged-signal-stream-name listed in T11 §4.4
%
% Hard constraint: this script never writes into T01's casper_sim_lib.slx.
% It opens it read-only as a library reference (load_system + add_block
% with the library path; the result is a *linked* block, not a copy).
%
% Outputs:
%   model_path : absolute path to the saved casper_sim_phase0.slx
%
% Source firmware reference: none (top-level integration glue).

    p = inputParser();
    addParameter(p, 'OutDir', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});

    if isempty(p.Results.OutDir)
        here = fileparts(mfilename('fullpath'));
    else
        here = char(p.Results.OutDir);
    end
    if ~isfolder(here); mkdir(here); end

    model_name = 'casper_sim_phase0';
    model_path = fullfile(here, [model_name '.slx']);

    % Ensure the upstream truth lib & its bus are available
    simroot   = fileparts(here);                        % .../Simulink Development
    truth_dir = fullfile(simroot, 'truth');
    truth_lib = fullfile(truth_dir, 'casper_sim_lib.slx');
    if ~isfile(truth_lib)
        error('build_casper_sim_phase0:NoTruthLib', ...
            'truth library not found at %s. Run truth build first.', truth_lib);
    end
    addpath(truth_dir);
    addpath(fullfile(simroot, 'shared'));
    truth_bus = casper_truth_build_bus();
    assignin('base', 'TruthBus', truth_bus);

    % Close & delete prior version of OUR model (not the truth lib).
    if bdIsLoaded(model_name); close_system(model_name, 0); end
    if isfile(model_path); delete(model_path); end

    % Load truth lib read-only (don't unlock, don't save).
    if ~bdIsLoaded('casper_sim_lib')
        load_system(truth_lib);
    end

    new_system(model_name);
    load_system(model_name);

    % Solver: fixed-step ode4, 1e-4, stop_time set from base WS SimCfg
    set_param(model_name, ...
        'Solver',         'FixedStepDiscrete', ...
        'SolverType',     'Fixed-step', ...
        'FixedStep',      '1e-4', ...
        'StartTime',      '0', ...
        'StopTime',       '549', ...
        'SaveOutput',     'on', ...
        'SaveFormat',     'Dataset', ...
        'SaveTime',       'on', ...
        'TimeSaveName',   'tout');

    % --- 1. Truth source (LINKED, not copied, from T01 library) ----------
    truth_blk_dst = [model_name '/truth_source'];
    add_block('casper_sim_lib/truth_source', truth_blk_dst);
    set_param(truth_blk_dst, 'Position', [40 60 220 200]);

    % --- 2. Bus Selector (break truth bus into individual signals) -------
    sel_path = [model_name '/truth_unpack'];
    add_block('simulink/Signal Routing/Bus Selector', sel_path);
    sel_fields = {'pos_NED', 'vel_NED', 'accel_NED', 'quat_std', ...
                  'omega_body_std', 'time_s', 'mach', 'air_density_kgm3', ...
                  'air_temp_K', 'air_pressure_pa'};
    set_param(sel_path, 'OutputSignals', strjoin(sel_fields, ','));
    set_param(sel_path, 'Position', [280 60 320 460]);
    add_line(model_name, 'truth_source/1', 'truth_unpack/1', 'autorouting', 'on');

    % --- 3. To-Workspace blocks for each truth field (for run-time logging
    %        + later sub-agent inspection from the workspace).
    for k = 1:numel(sel_fields)
        name = sel_fields{k};
        tw = [model_name '/tw_truth_' name];
        add_block('simulink/Sinks/To Workspace', tw);
        set_param(tw, ...
            'VariableName', ['tsl_truth_' name], ...
            'SaveFormat',   'Timeseries', ...
            'SampleTime',   '-1');
        set_param(tw, 'Position', [380 60+40*(k-1) 460 80+40*(k-1)]);
        add_line(model_name, ['truth_unpack/' num2str(k)], ['tw_truth_' name '/1'], ...
            'autorouting', 'on');
    end

    % --- 4. Note block documenting the integration approach --------------
    add_block('built-in/Note', [model_name '/integration_note']);
    note_text = sprintf([ ...
        'CASPER-2 Phase 0 Top-Level Model\\n' ...
        '\\n' ...
        'This .slx is the *Simulink face* of T11.  The full T01->T09\\n' ...
        'integration loop lives in casper_phase0_run.m (called by\\n' ...
        'run_phase0_trustgate.m).  This model exposes the TruthBus to\\n' ...
        'the workspace so reviewers can verify truth wiring at solver\\n' ...
        'rate without running the heavy estimator stack.\\n' ...
        '\\n' ...
        'See T11_integration.md, ARCHITECTURE.md, PHASE0_SPEC.md.\\n' ...
        '\\n' ...
        'Trust gate: matlab -batch "run_phase0_trustgate"\\n']);
    set_param([model_name '/integration_note'], ...
        'Text',     note_text, ...
        'Position', [40 280 380 460]);

    % --- 5. Save (no auto-arrange to keep deterministic) -----------------
    save_system(model_name, model_path);
    close_system(model_name, 0);

    fprintf('[T11] build_casper_sim_phase0: wrote %s\n', model_path);
end
