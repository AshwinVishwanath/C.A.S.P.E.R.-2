function test_baro_block_visual_compile()
%TEST_BARO_BLOCK_VISUAL_COMPILE Smoke-test baro_block_visual subsystem compiles.
%
% Builds a tiny harness model that drives the baro_visual_block subsystem
% from a synthetic stationary truth bus (sea-level pressure, M=0, T=288.15 K),
% runs the diagram update (compile), and runs a 0.1 s sim. Asserts outputs
% are finite, pressure is near 101325 Pa and altitude is near 0 m.
%
% Pattern matches test_imu_block_visual_compile (T03). The harness base step
% is set to 1/Baro.Rate_Hz = 1/100 s so the rate matches the sensor rate
% exactly (T03 lesson: a base step that is not an integer multiple of the
% sensor rate breaks the discrete solver).

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    addpath(fullfile(here, '..', 'T11_integration'));
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    addpath(fullfile(here, '..', 'T02_sensor_params'));

    casper_sim_config('Seed', 20260519, 'StopTime', 0.1);

    % Ensure the visual lib is on disk and loaded.
    lib_path = fullfile(here, 'baro_block_visual.slx');
    if ~isfile(lib_path)
        build_baro_block_visual();
    end
    if ~bdIsLoaded('baro_block_visual')
        load_system(lib_path);
    end

    % Tiny harness model
    mdl = 'tmp_baro_visual_harness';
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl);
    load_system(mdl);

    % Harness base step = 1/Baro.Rate_Hz = 1/100, matches the baro rate.
    set_param(mdl, 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', '1/100', 'StartTime', '0', 'StopTime', '0.1');

    % --- Build a stationary SensorInputBus via Bus Creator ---
    % Each field driven by a Constant of the right shape (matches T03 pattern).
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
        nm  = truth_consts{k}{1};
        val = truth_consts{k}{2};
        cp = [mdl '/' nm '_const'];
        add_block('simulink/Sources/Constant', cp);
        set_param(cp, 'Value', val, 'SampleTime', '-1', ...
            'Position', [30 (30 + (k-1)*40) 100 (50 + (k-1)*40)]);
    end

    bc = [mdl '/SensorBusCreate'];
    add_block('simulink/Signal Routing/Bus Creator', bc);
    set_param(bc, 'Inputs', num2str(numel(truth_consts)));
    set_param(bc, 'OutDataTypeStr', 'Bus: SensorInputBus', ...
        'UseBusObject',  'on', ...
        'NonVirtualBus', 'on');
    set_param(bc, 'Position', [180 30 220 30 + numel(truth_consts)*40]);
    for k = 1:numel(truth_consts)
        src_port = [truth_consts{k}{1} '_const/1'];
        dst_port = ['SensorBusCreate/' num2str(k)];
        line_h = add_line(mdl, src_port, dst_port, 'autorouting', 'on');
        % Name the signal so the Bus Creator maps it to the named bus field.
        set_param(line_h, 'Name', truth_consts{k}{1});
    end

    % Add the baro visual subsystem (link to library)
    baro = [mdl '/BARO'];
    add_block('baro_block_visual/baro_visual_block', baro, ...
        'Position', [280 30 480 200]);
    add_line(mdl, 'SensorBusCreate/1', 'BARO/1', 'autorouting', 'on');

    % Outports: press_pa, alt_m, temp_C, data_ready
    for k = 1:4
        op = [mdl '/Out_BARO_' num2str(k)];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [520 (30 + (k-1)*40) 550 (50 + (k-1)*40)]);
        add_line(mdl, ['BARO/' num2str(k)], ['Out_BARO_' num2str(k) '/1'], ...
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

    % Get the logged signals from the simout
    yout = sim_out.get('yout');
    if isempty(yout)
        error('No yout produced — check outport logging settings.');
    end

    % Extract each output signal
    press_ts = yout{1}.Values;
    alt_ts   = yout{2}.Values;
    temp_ts  = yout{3}.Values;
    dr_ts    = yout{4}.Values;

    press_data = press_ts.Data;
    alt_data   = alt_ts.Data;
    temp_data  = temp_ts.Data;
    dr_data    = dr_ts.Data;

    fprintf('[compile] press_pa data shape: %s\n', mat2str(size(press_data)));
    fprintf('[compile] last press_pa sample: %.3f Pa\n',  press_data(end));
    fprintf('[compile] last alt_m    sample: %.4f m\n',   alt_data(end));
    fprintf('[compile] last temp_C   sample: %.4f degC\n', temp_data(end));
    fprintf('[compile] last data_ready sample: %d\n',     dr_data(end));

    if ~all(isfinite(press_data(:))) || ~all(isfinite(alt_data(:))) || ...
            ~all(isfinite(temp_data(:)))
        error('Non-finite values in baro output.');
    end

    % Sanity range: at 101325 Pa truth + noise sigma ~6 Pa + per-run bias ~5 Pa,
    % a fresh 0.1-s window should land within ~50 Pa of sea level (generous).
    if abs(press_data(end) - 101325) > 200
        error('Sea-level pressure out of expected band: got %.1f Pa', ...
            press_data(end));
    end
    if abs(alt_data(end)) > 20
        error('Sea-level altitude out of expected band: got %.3f m', ...
            alt_data(end));
    end
    if abs(temp_data(end) - 15.0) > 0.1
        error('Sea-level temperature out of expected band: got %.3f degC', ...
            temp_data(end));
    end
    if ~all(dr_data(:) == 1)
        error('data_ready not constant true');
    end

    fprintf('[compile] PASS — all baro samples finite, in expected ranges.\n');

    close_system(mdl, 0);
end
