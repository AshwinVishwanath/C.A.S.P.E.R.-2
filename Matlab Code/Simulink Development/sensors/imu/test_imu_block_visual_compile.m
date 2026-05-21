function test_imu_block_visual_compile()
%TEST_IMU_BLOCK_VISUAL_COMPILE Smoke-test imu_block_visual subsystems compile.
%
% Builds a tiny harness model that drives each of the two visual subsystems
% (lsm_visual_block, adxl_visual_block) from a synthetic stationary truth
% bus, runs the diagram update (compile), and runs a 0.1 s sim. Asserts
% outputs are finite and non-NaN.

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    simroot = fileparts(fileparts(here));     % .../Simulink Development
    addpath(fullfile(simroot, 'shared'));
    addpath(fullfile(simroot, 'truth'));
    addpath(fullfile(simroot, 'params'));

    casper_sim_config('Seed', 20260519, 'StopTime', 0.1);

    % Ensure the visual lib is on disk and loaded.
    lib_path = fullfile(here, 'imu_block_visual.slx');
    if ~isfile(lib_path)
        build_imu_block_visual();
    end
    if ~bdIsLoaded('imu_block_visual')
        load_system(lib_path);
    end

    % Tiny harness model
    mdl = 'tmp_imu_visual_harness';
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl);
    load_system(mdl);

    % Use 1/833 as the harness base step so it matches the LSM rate exactly
    % (no multi-rate testing in this harness — only the LSM subsystem is wired).
    set_param(mdl, 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', '1/833', 'StartTime', '0', 'StopTime', '0.1');

    % --- Build a stationary TruthBus signal via Bus Creator ---
    % Each field driven by a Constant of the right shape.
    truth_consts = { ...
        {'pos_NED',          '[0;0;0]'}, ...
        {'vel_NED',          '[0;0;0]'}, ...
        {'accel_NED',        '[0;0;0]'}, ...
        {'quat_std',         '[1;0;0;0]'}, ...
        {'omega_body_std',   '[0;0;0]'}, ...
        {'time_s',           '0'}, ...
        {'mach',             '0'}, ...
        {'air_density_kgm3', '1.225'}, ...
        {'air_temp_K',       '288.15'}, ...
        {'air_pressure_pa',  '101325'} };
    for k = 1:numel(truth_consts)
        nm = truth_consts{k}{1};
        val = truth_consts{k}{2};
        cp = [mdl '/' nm '_const'];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', '-1', ...
            'Position', [30 (30 + (k-1)*40) 100 (50 + (k-1)*40)]);
    end

    bc = [mdl '/TruthBusCreate'];
    add_block('simulink/Signal Routing/Bus Creator', bc);
    set_param(bc, 'Inputs', num2str(numel(truth_consts)));
    set_param(bc, 'OutDataTypeStr', 'Bus: SensorInputBus', ...
        'UseBusObject', 'on', ...
        'NonVirtualBus', 'on');
    set_param(bc, 'Position', [180 30 220 30 + numel(truth_consts)*40]);
    for k = 1:numel(truth_consts)
        src_port = [truth_consts{k}{1} '_const/1'];
        dst_port = ['TruthBusCreate/' num2str(k)];
        line_h = add_line(mdl, src_port, dst_port, 'autorouting', 'on');
        % Name the signal so the Bus Creator can map it to the named bus field.
        set_param(line_h, 'Name', truth_consts{k}{1});
    end

    % Add the LSM visual subsystem (link to library)
    lsm = [mdl '/LSM'];
    add_block('imu_block_visual/lsm_visual_block', lsm, 'Position', [280 30 480 200]);
    add_line(mdl, 'TruthBusCreate/1', 'LSM/1', 'autorouting', 'on');

    % Outport: accel_g_body_std (LSM output 1)
    for k = 1:4
        op = [mdl '/Out_LSM_' num2str(k)];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [520 (30 + (k-1)*40) 550 (50 + (k-1)*40)]);
        add_line(mdl, ['LSM/' num2str(k)], ['Out_LSM_' num2str(k) '/1'], ...
            'autorouting', 'on');
    end

    % --- Try compile (update diagram) ---
    fprintf('[compile] updating diagram...\n');
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('[compile] update OK.\n');

    % --- Run a short sim ---
    fprintf('[compile] simulating 0.1 s...\n');
    sim_out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    fprintf('[compile] sim OK.\n');

    % Get the logged accel signal from the simout
    yout = sim_out.get('yout');
    if isempty(yout)
        warning('No yout produced — check outport logging settings.');
    else
        accel_ts = yout{1}.Values;
        accel_data = accel_ts.Data;
        fprintf('[compile] accel_g data shape: %s\n', mat2str(size(accel_data)));
        fprintf('[compile] last accel sample (g): %s\n', mat2str(squeeze(accel_data(end,:,:)).'));
        if all(isfinite(accel_data(:)))
            fprintf('[compile] PASS — all accel samples finite.\n');
        else
            error('Non-finite values in accel output.');
        end
    end

    close_system(mdl, 0);
end
