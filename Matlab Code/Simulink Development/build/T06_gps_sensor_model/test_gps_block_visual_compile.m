function test_gps_block_visual_compile()
%TEST_GPS_BLOCK_VISUAL_COMPILE Smoke-test gps_block_visual compiles + simulates.
%
% Builds a tiny harness model that drives the gps_visual_block subsystem
% from a synthetic stationary truth bus, runs the diagram update (compile),
% and runs a 0.5 s sim (5 ticks at 10 Hz). Asserts:
%   * model compiles cleanly
%   * lat output (after FIFO primes) is near the configured launch latitude
%   * alt output is reasonable (within ~10 m of launch altitude given noise)
%   * fix outputs are uint8 and not stuck on 0 after warm-up

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    addpath(fullfile(here, '..', 'T11_integration'));
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    addpath(fullfile(here, '..', 'T02_sensor_params'));

    % Use a 5-tick window: 0.5 s at 1/10 base step. That's enough to prime
    % the latency FIFO and emit a couple of valid samples.
    base_step_s = 1/10;
    stop_time_s = 0.5;

    casper_sim_config('Seed', 20260519, 'StopTime', stop_time_s);

    % Inject launch-site origin onto base GPS struct so the gpsSensor
    % constructor finds it (also done internally by build_gps_block_visual).
    GPSp = evalin('base', 'GPS');
    if ~isfield(GPSp, 'LaunchLat_deg')
        GPSp.LaunchLat_deg = 51.5074;
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

    % Clear persistent state from any prior test in this MATLAB session.
    clear casper_gps_quirks;
    clear casper_gps_step_visual;

    % Ensure the visual lib is on disk and loaded.
    lib_path = fullfile(here, 'gps_block_visual.slx');
    if ~isfile(lib_path)
        build_gps_block_visual();
    end
    if ~bdIsLoaded('gps_block_visual')
        load_system(lib_path);
    end

    % Tiny harness model.
    mdl = 'tmp_gps_visual_harness';
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl);
    load_system(mdl);

    set_param(mdl, 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', num2str(base_step_s), ...
        'StartTime', '0', 'StopTime', num2str(stop_time_s));

    % --- Build a stationary TruthBus signal via Bus Creator ---
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

    bc = [mdl '/TruthBusCreate'];
    add_block('simulink/Signal Routing/Bus Creator', bc);
    set_param(bc, 'Inputs', num2str(numel(truth_consts)));
    set_param(bc, 'OutDataTypeStr', 'Bus: SensorInputBus', ...
        'UseBusObject', 'on', 'NonVirtualBus', 'on');
    set_param(bc, 'Position', [180 30 220 30 + numel(truth_consts)*40]);
    for k = 1:numel(truth_consts)
        src_port = [truth_consts{k}{1} '_const/1'];
        dst_port = ['TruthBusCreate/' num2str(k)];
        line_h = add_line(mdl, src_port, dst_port, 'autorouting', 'on');
        set_param(line_h, 'Name', truth_consts{k}{1});
    end

    % Add the GPS visual subsystem (library link).
    gps = [mdl '/GPS'];
    add_block('gps_block_visual/gps_visual_block', gps, ...
        'Position', [280 30 480 460]);
    add_line(mdl, 'TruthBusCreate/1', 'GPS/1', 'autorouting', 'on');

    % Outports — one per GPSOutputBus field (9 outputs).
    out_names = {'lat_deg7','lon_deg7','alt_mm', ...
                 'vel_n_mms','vel_e_mms','vel_d_mms', ...
                 'fix','sv','data_ready'};
    for k = 1:numel(out_names)
        op = [mdl '/Out_GPS_' num2str(k)];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [520 (30 + (k-1)*40) 550 (50 + (k-1)*40)]);
        add_line(mdl, ['GPS/' num2str(k)], ['Out_GPS_' num2str(k) '/1'], ...
            'autorouting', 'on');
    end

    % --- Try compile (update diagram) ---
    fprintf('[compile] updating diagram...\n');
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('[compile] update OK.\n');

    % --- Run a short sim ---
    fprintf('[compile] simulating %.2f s...\n', stop_time_s);
    sim_out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    fprintf('[compile] sim OK.\n');

    yout = sim_out.get('yout');
    if isempty(yout)
        error('No yout produced — check outport logging settings.');
    end

    % Per-outport, log the last sample (when the FIFO is primed and a valid
    % sample is emerging).
    lat_data = yout{1}.Values.Data;
    lon_data = yout{2}.Values.Data;
    alt_data = yout{3}.Values.Data;
    fix_data = yout{7}.Values.Data;
    sv_data  = yout{8}.Values.Data;
    dr_data  = yout{9}.Values.Data;

    fprintf('[compile] lat_deg7 last sample: %d (= %.7f deg)\n', ...
        lat_data(end), double(lat_data(end)) / 1e7);
    fprintf('[compile] lon_deg7 last sample: %d (= %.7f deg)\n', ...
        lon_data(end), double(lon_data(end)) / 1e7);
    fprintf('[compile] alt_mm last sample:   %d (= %.3f m)\n', ...
        alt_data(end), double(alt_data(end)) / 1000);
    fprintf('[compile] fix last sample:      %d\n', fix_data(end));
    fprintf('[compile] sv  last sample:      %d\n', sv_data(end));
    fprintf('[compile] data_ready last:      %d\n', dr_data(end));

    % --- Sanity assertions on the post-prime sample ---
    lat_deg_obs = double(lat_data(end)) / 1e7;
    lon_deg_obs = double(lon_data(end)) / 1e7;
    alt_m_obs   = double(alt_data(end)) / 1000;
    expected_lat = GPSp.LaunchLat_deg;
    expected_lon = GPSp.LaunchLon_deg;
    expected_alt = GPSp.LaunchAlt_m;

    npass = 0; nfail = 0;
    [npass, nfail] = assert_near(lat_deg_obs, expected_lat, 0.001, ...
        'lat near launch site', npass, nfail);
    [npass, nfail] = assert_near(lon_deg_obs, expected_lon, 0.001, ...
        'lon near launch site', npass, nfail);
    [npass, nfail] = assert_near(alt_m_obs,   expected_alt, 20.0, ...
        'alt near launch altitude', npass, nfail);
    if fix_data(end) == 3 && sv_data(end) == 12 && dr_data(end) == 1
        fprintf('  PASS fix/sv/data_ready post-prime\n'); npass = npass + 1;
    else
        fprintf('  FAIL fix/sv/data_ready post-prime (fix=%d sv=%d dr=%d)\n', ...
            fix_data(end), sv_data(end), dr_data(end));
        nfail = nfail + 1;
    end
    if all(isfinite(double(lat_data(:)))) && all(isfinite(double(alt_data(:))))
        fprintf('  PASS all samples finite\n'); npass = npass + 1;
    else
        fprintf('  FAIL non-finite GPS output\n'); nfail = nfail + 1;
    end

    fprintf('[compile] %d PASS / %d FAIL\n', npass, nfail);
    close_system(mdl, 0);
    if nfail > 0
        error('test_gps_block_visual_compile:FAIL', '%d check(s) failed', nfail);
    end
end


function [np, nf] = assert_near(actual, expected, tol, label, np, nf)
    if abs(actual - expected) <= tol
        fprintf('  PASS %s  (|%.6g - %.6g| <= %.3g)\n', label, actual, expected, tol);
        np = np + 1;
    else
        fprintf('  FAIL %s  |%.6g - %.6g| = %.3g > %.3g\n', label, ...
            actual, expected, abs(actual - expected), tol);
        nf = nf + 1;
    end
end
