function model_path = build_casper_sim_phase0(varargin)
%BUILD_CASPER_SIM_PHASE0 Visual top-level Phase 0 Simulink model constructor.
%
% Synopsis:
%   model_path = build_casper_sim_phase0()
%   model_path = build_casper_sim_phase0('OutDir', '/abs/path')
%
% Rewrites casper_sim_phase0.slx as a visually-wired top-level model. The
% diagram replaces the legacy MATLAB-driver-only stub (archived as
% build_casper_sim_phase0_legacy.m / casper_sim_phase0_legacy.slx). The
% canonical byte-identical regression reference remains casper_phase0_run.m;
% this .slx is for visual auditability of the data flow.
%
% Block topology (left-to-right by data flow, top-to-bottom by rate):
%
%   Row 1 (truth source, 1 kHz base):
%     truth_source_inline subsystem  — From Workspace blocks read truth_ts
%                                       (struct-of-timeseries in base WS),
%                                       Bus Creator emits SensorInputBus.
%
%   Row 2 (sensor blocks, native rates):
%     lsm_visual_block   (1 kHz; SNAPPED from 833 Hz)   library link
%     adxl_visual_block  (1 kHz; SNAPPED from 800 Hz)   library link
%     baro_visual_block  (100 Hz native)                library link
%     mag_visual_block   (100 Hz native)                library link
%     gps_visual_block   (10 Hz native)                 library link
%
%   Row 3 (frame switches, T07):
%     frame_switch_body_accel : a_std (3x1) -> a_fw (3x1)   library link
%     frame_switch_body_gyro  : g_std (3x1) -> g_fw (3x1)   library link
%     frame_switch_body_mag   : m_std (3x1) -> m_fw (3x1)   library link
%       (each links the 'frame_switch' subsystem from
%        frame_switch_block.slx; we only use the body_switch outport).
%
%   Row 4 (attitude, 1 kHz):
%     attitude_subsystem  — wraps attitude_block's attitude_tick MATLAB Fn
%       with the mag-suppression gate: mag_new_eff = mag_data_ready AND
%       init_complete_delayed (Unit Delay Z-1 breaks algebraic loop).
%
%   Row 5 (EKF, 500 Hz; SNAPPED from 416 Hz):
%     eskf_subsystem      — wraps eskf_block's eskf_step MATLAB Fn with an
%       init-gate. Until attitude init_complete, the Switch holds the
%       initial state via Memory; after the Rising-Edge Detector, the EKF
%       runs each step. Inputs: accel_zup (from quat-rotated a_fw),
%       baro_alt, baro_new, mach, zupt_trigger.
%
%   Row 6 (validation):
%     validation_subsystem  — inline 14-inport logger (built from same
%       skeleton as T10 validation_block but inlined so we don't need a
%       Model reference; each inport feeds a To Workspace block).
%
% Rate strategy:
%   The Simulink fixed-step solver requires every sample time to be an
%   integer multiple of the base step. Base = 1e-3 s (1 kHz). To use this
%   base we SNAP:
%     IMU LSM   833 Hz -> 1000 Hz (1e-3 s)
%     IMU ADXL  800 Hz -> 1000 Hz (1e-3 s)
%     EKF       416 Hz ->  500 Hz (2e-3 s)
%   Baro/Mag/GPS keep native rates (100, 100, 10 Hz).
%
%   This snap is acceptable because this model is for VISUAL AUDITABILITY.
%   casper_phase0_run.m remains the canonical byte-exact regression driver
%   at its native 1e-4 s solver and exact 833/800/416 Hz sensor rates.
%
% Four semantic constraints (per plan §4):
%   1. Mag suppression until attitude init — handled by Unit Delay on
%      init_complete + AND gate on mag_data_ready inside attitude subsystem.
%   2. EKF init gated on attitude init — Switch+Memory pattern inside
%      eskf_subsystem. The Switch picks held-state until init_complete.
%   3. 11 s pre-pad window — handled by truth_ts content in base WS (no
%      diagram change needed).
%   4. Post-pad metric slicing — handled inside casper_metric_*.m offline
%      (no diagram change needed).
%
% Returns:
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

    % --- Paths & base-workspace setup ----------------------------------------
    simroot = fileparts(here);                 % .../Simulink Development
    addpath(here);
    addpath(fullfile(simroot, 'shared'));
    addpath(fullfile(simroot, 'truth'));
    addpath(fullfile(simroot, 'params'));
    addpath(fullfile(simroot, 'sensors', 'imu'));
    addpath(fullfile(simroot, 'sensors', 'baro'));
    addpath(fullfile(simroot, 'sensors', 'mag'));
    addpath(fullfile(simroot, 'sensors', 'gps'));
    addpath(fullfile(simroot, 'nav', 'frame_switch'));
    addpath(fullfile(simroot, 'nav', 'eskf'));
    addpath(fullfile(simroot, 'nav', 'eskf16'));
    addpath(fullfile(simroot, 'nav', 'attitude'));
    addpath(fullfile(simroot, 'validation'));

    % Populate Sim/IMU/ADXL/Baro/Mag/GPS/Estimator/Attitude/TruthBus + the
    % six unified Simulink.Bus objects (SensorInputBus/IMUOutputBus/etc.).
    % If the user already ran casper() (which populates SimCfg with their
    % chosen profile's StopTime), preserve that so the rebuilt .slx has
    % a saved StopTime matching the profile — otherwise `sim(model)`
    % without an explicit override uses a 5 s default and the EKFs barely
    % finish initializing.
    if evalin('base', 'exist(''SimCfg'', ''var'')')
        prior_cfg = evalin('base', 'SimCfg');
        if isstruct(prior_cfg) && isfield(prior_cfg, 'StopTime_s')
            casper_sim_config('Seed', double(prior_cfg.Seed), ...
                              'StopTime', prior_cfg.StopTime_s);
        else
            casper_sim_config('Seed', 20260519, 'StopTime', 5.0);
        end
    else
        casper_sim_config('Seed', 20260519, 'StopTime', 5.0);
    end

    % Augment GPS struct (mirrors T06's build_gps_block_visual augment).
    augment_gps_struct_();

    % --- RATE SNAP for the visual model ----------------------------------
    % The library blocks bind their internal Rate Transition periods to
    % 1/IMU.Rate_Hz (833), 1/ADXL.RatePostLaunch_Hz (800), and
    % Estimator.Dt (1/416) at compile time. With a 1 kHz base step, those
    % native rates aren't integer multiples of the base, which blocks the
    % discrete solver. We OVERRIDE the rates in the base workspace BEFORE
    % the model loads so the embedded expressions evaluate to snapped
    % integer-multiple periods.
    %
    %   IMU LSM   : 833 Hz -> 1000 Hz (1 ms)
    %   IMU ADXL  : 800 Hz -> 1000 Hz (1 ms)
    %   EKF       : 416 Hz ->  500 Hz (2 ms)
    %
    % The legacy MATLAB driver (casper_phase0_run.m) keeps the native rates
    % at its own 1e-4 solver and remains the canonical byte-exact regression
    % reference. The visual model is for AUDITABILITY only.
    snap_rates_for_visual_model_();

    % --- Load every library link target -------------------------------------
    % NOTE: T07 (frame_switch_block) is intentionally NOT a library link.
    % We inline the body-frame switch as a single MATLAB Function block
    % per channel; see build_imu_frame_switch_ / build_mag_frame_switch_.
    % The T07 logic itself (vec_fw = [v(2); v(1); -v(3)]) is byte-identical
    % to casper_frame_switch_body.m.
    libs = { ...
        'imu_block_visual',  fullfile(simroot, 'sensors', 'imu',  'imu_block_visual.slx'); ...
        'baro_block_visual', fullfile(simroot, 'sensors', 'baro', 'baro_block_visual.slx'); ...
        'mag_block_visual',  fullfile(simroot, 'sensors', 'mag',  'mag_block_visual.slx'); ...
        'gps_block_visual',  fullfile(simroot, 'sensors', 'gps',  'gps_block_visual.slx') };
    for k = 1:size(libs, 1)
        lib_name = libs{k, 1};
        lib_path = libs{k, 2};
        if ~isfile(lib_path)
            error('build_casper_sim_phase0:NoLib', ...
                'Library %s not found at %s. Build the corresponding TXX block first.', ...
                lib_name, lib_path);
        end
        if ~bdIsLoaded(lib_name)
            load_system(lib_path);
        end
    end

    % Ensure the eskf16_block.slx library exists; build it on demand so the
    % top-level integration model can hard-link to it.
    eskf16_lib_path = fullfile(simroot, 'nav', 'eskf16', 'eskf16_block.slx');
    if ~isfile(eskf16_lib_path)
        build_eskf16_block(fullfile(simroot, 'nav', 'eskf16'));
    end
    if ~bdIsLoaded('eskf16_block')
        load_system(eskf16_lib_path);
    end

    % Pre-cache the EKF16 symbolic F/Q/H handles in the base workspace so the
    % first persistent-state call inside the helper does not stall the sim.
    try
        casper_eskf16_load_symbolic();
    catch ME
        warning('build_casper_sim_phase0:EKF16SymbolicLoadFailed', ...
            'Failed to pre-load EKF16 symbolic handles: %s', ME.message);
    end

    % --- Close & delete prior model -----------------------------------------
    if bdIsLoaded(model_name); close_system(model_name, 0); end
    if isfile(model_path); delete(model_path); end

    new_system(model_name);
    load_system(model_name);

    % Solver: fixed-step discrete, base step 1 ms (1 kHz). Snaps documented.
    % StartTime is -PAD_S s so the prepended pre-launch pad (added by the
    % InitFcn callback below via casper_prelaunch_pad_truth_ts) is actually
    % swept by the solver -- otherwise From Workspace blocks would never
    % read the pad samples and attitude static-init would still run during
    % boost. The fix-it sequence is:
    %   1. casper.m / test scripts populate truth_ts in base WS starting at t=0
    %   2. InitFcn calls casper_prelaunch_pad_truth_ts which prepends PAD_S
    %      seconds of stationary pad data at t=-PAD_S..0.
    %   3. Solver runs from t=-PAD_S (pad) through t=StopTime (post-pad
    %      flight) and the attitude estimator gets a clean init window
    %      that completes BEFORE the rocket starts moving.
    %
    % PAD_S must be >= Attitude.StaticInitTimeout_s (currently 10 s) PLUS
    % a small margin, because static_init's mag-sample completion path is
    % suppressed during init (see attitude_step_helper line 103: mag_new
    % is forced false while init_complete is false) -- so static_init
    % always completes via the TIMEOUT branch, never the mag-count branch.
    % If PAD_S < 10 s, init slips into the boost window and both EKFs
    % miss the entire ascent (observed apogee under-shoot ~50-89%).
    %
    % The canonical run_phase0_trustgate uses PreLaunchPad_s=11.0 for the
    % same reason. We use 11 s here for byte-for-byte parity.
    PAD_S = 11.0;
    % Pull the user's chosen StopTime from SimCfg if available — falls back
    % to 85 s (apogee profile) which is the sensible default for a flight
    % long enough that the EKFs converge.
    if evalin('base', 'exist(''SimCfg'', ''var'')')
        stoptime_str = sprintf('%.6g', evalin('base', 'SimCfg.StopTime_s'));
    else
        stoptime_str = '85';
    end

    set_param(model_name, ...
        'Solver',                  'FixedStepDiscrete', ...
        'SolverType',              'Fixed-step', ...
        'FixedStep',               '1e-3', ...
        'StartTime',               sprintf('%.6g', -PAD_S), ...
        'StopTime',                stoptime_str, ...
        'SaveOutput',              'on', ...
        'SaveFormat',              'Dataset', ...
        'SaveTime',                'on', ...
        'TimeSaveName',            'tout', ...
        'ReturnWorkspaceOutputs',  'off', ...
        'InitFcn',                 sprintf('casper_prelaunch_pad_truth_ts(%.6g);', PAD_S));
    % ReturnWorkspaceOutputs='off' makes To-Workspace blocks write the
    % log_* variables directly into the base workspace at sim end (the
    % classic R2019b- behavior). With it 'on' (the modern default), sim()
    % returns a Simulink.SimulationOutput object containing the logs and
    % nothing lands in base WS — which breaks downstream callers like
    % `generate_ekf_comparison_report` that read from base via evalin.
    % (The report generator also has a SimulationOutput fallback for
    % users who have a pre-existing .slx without this flag.)

    % Allow inherited sample times to propagate from the constants without
    % shouting (default would mark them all 'inherit' which is fine here).
    try
        set_param(model_name, 'InheritedTsInSrcMsg', 'none');
    catch
    end

    % --- Row 1: Truth source (inline) ---------------------------------------
    build_truth_source_inline_(model_name);

    % --- Row 2: Sensor blocks (library links) -------------------------------
    add_sensor_block_(model_name, 'lsm_visual_block',  'IMU_LSM',  [560  40  760 200]);
    add_sensor_block_(model_name, 'adxl_visual_block', 'IMU_ADXL', [560 220  760 380]);
    add_sensor_block_(model_name, 'baro_visual_block', 'BARO',     [560 400  760 540]);
    add_sensor_block_(model_name, 'mag_visual_block',  'MAG',      [560 560  760 700]);
    add_sensor_block_(model_name, 'gps_visual_block',  'GPS',      [560 720  760 900]);

    % Wire the SensorInputBus from the inline truth source to each sensor.
    for nm = {'IMU_LSM', 'IMU_ADXL', 'BARO', 'MAG', 'GPS'}
        add_line(model_name, 'SensorBusCreate/1', [nm{1} '/1'], 'autorouting', 'on');
    end

    % --- Row 3: Frame switches (3 instances of frame_switch_block) ----------
    % Each instance only uses the body_switch port (port 2). We bus-select
    % the IMU accel/gyro and mag fields, run them through frame switches.
    % LSM IMU output: outport 1 = accel_g (g, std-body), 2 = gyro_dps.
    % Apply unit conversions inside small MATLAB Function blocks.
    build_imu_frame_switch_(model_name);

    % MAG output: outport 1 = field_uT (std-body).
    build_mag_frame_switch_(model_name);

    % --- Row 4: Attitude (with mag suppression gate) ------------------------
    build_attitude_subsystem_(model_name);

    % --- Row 5: EKF (with init gate) ----------------------------------------
    build_eskf_subsystem_(model_name);

    % --- Row 6: Validation (inline 14-inport logger) ------------------------
    build_validation_subsystem_(model_name);

    % --- Row 7: Real-time scope panels (truth vs estimate) ------------------
    % Adds 4 Simulink Scope blocks that auto-open at sim start so the user
    % can dynamically compare truth vs estimate during simulation. Taps off
    % existing signal lines without breaking any existing connections.
    build_scope_panels_(model_name);

    % --- Row 8: Parallel 16-state EKF + comparison scopes -------------------
    % Adds the new ESKF16 visual block in parallel with the existing 4-state
    % ESKF subsystem. Both estimators share the same sensor inputs and run
    % side-by-side; comparison scopes auto-open at sim start.
    build_eskf16_subsystem_(model_name);

    % --- Note + final save --------------------------------------------------
    add_top_level_note_(model_name);

    save_system(model_name, model_path);
    close_system(model_name, 0);

    fprintf('[build_casper_sim_phase0] wrote %s\n', model_path);
end


% =========================================================================
function snap_rates_for_visual_model_()
% Snap sensor and estimator rates in the base workspace to integer
% multiples of the 1 kHz base step. See header comment of the caller.
    IMU = evalin('base', 'IMU');
    IMU.Rate_Hz = 1000;
    assignin('base', 'IMU', IMU);

    ADXL = evalin('base', 'ADXL');
    ADXL.RatePostLaunch_Hz = 1000;
    assignin('base', 'ADXL', ADXL);

    Est = evalin('base', 'Estimator');
    Est.Dt = 2e-3;
    assignin('base', 'Estimator', Est);
end


% =========================================================================
function augment_gps_struct_()
% Ensure the base-workspace GPS struct has launch-site origin & reacquire
% time so gps_visual_block's internal step function resolves all fields.
% Mirrors what T06's build_gps_block_visual does.
    GPSp = evalin('base', 'GPS');
    if ~isfield(GPSp, 'LaunchLat_deg'); GPSp.LaunchLat_deg = 51.5074; end
    if ~isfield(GPSp, 'LaunchLon_deg'); GPSp.LaunchLon_deg = -0.1278; end
    if ~isfield(GPSp, 'LaunchAlt_m');   GPSp.LaunchAlt_m   = 35.0;    end
    if ~isfield(GPSp, 'ReacquireTime_s'); GPSp.ReacquireTime_s = 1.0; end
    assignin('base', 'GPS', GPSp);
end


% =========================================================================
function build_truth_source_inline_(model_name)
% Inline truth source: From Workspace blocks read 'truth_ts.<field>',
% Bus Creator packs into SensorInputBus. The struct-of-timeseries truth_ts
% must be present in the base workspace before sim time (test scripts and
% trust-gate drivers populate it from truth_trajectory.mat).
%
% This avoids the TruthBus/SensorInputBus type mismatch we'd get if we
% used the T01 library's truth_source block directly (T01's block emits
% TruthBus and we'd need a non-trivial bus converter in between).

    truth_fields = { ...
        'pos_NED', 3; ...
        'vel_NED', 3; ...
        'accel_NED', 3; ...
        'quat_std', 4; ...
        'omega_body_std', 3; ...
        'time_s', 1; ...
        'mach', 1; ...
        'air_density_kgm3', 1; ...
        'air_temp_K', 1; ...
        'air_pressure_pa', 1};

    nf = size(truth_fields, 1);
    y0 = 40;
    y_step = 45;

    % From Workspace blocks
    for k = 1:nf
        name = truth_fields{k, 1};
        fw = [model_name '/FW_' name];
        add_block('simulink/Sources/From Workspace', fw);
        set_param(fw, ...
            'VariableName',           ['truth_ts.' name], ...
            'SampleTime',             '-1', ...
            'OutputAfterFinalValue',  'Holding final value', ...
            'Interpolate',            'on', ...
            'ZeroCross',              'off');
        ypos = y0 + (k-1) * y_step;
        set_param(fw, 'Position', [40 ypos 180 ypos+30]);
    end

    % Bus Creator -> SensorInputBus
    bc = [model_name '/SensorBusCreate'];
    add_block('simulink/Signal Routing/Bus Creator', bc);
    set_param(bc, ...
        'Inputs',         num2str(nf), ...
        'OutDataTypeStr', 'Bus: SensorInputBus', ...
        'UseBusObject',   'on', ...
        'NonVirtualBus',  'on');
    set_param(bc, 'Position', [320 y0 360 y0 + nf*y_step]);

    for k = 1:nf
        name = truth_fields{k, 1};
        lh = add_line(model_name, ['FW_' name '/1'], ['SensorBusCreate/' num2str(k)], ...
            'autorouting', 'on');
        set_param(lh, 'Name', name);
    end

    % Add a labeled tap so the bus shows up in the diagram at the entrance
    % to the sensor row. (Bus Creator output line names the signal.)
    lh_tap = get_param([model_name '/SensorBusCreate'], 'LineHandles');
    if ~isempty(lh_tap.Outport) && lh_tap.Outport(1) ~= -1
        try
            set_param(lh_tap.Outport(1), 'Name', 'SensorInputBus');
        catch
        end
    end
end


% =========================================================================
function add_sensor_block_(model_name, ~, alias, pos)
%ADD_SENSOR_BLOCK_ Add a library-linked subsystem from one of the per-task
% sensor libraries (imu_block_visual, baro_block_visual, etc.).
%
% lib_name (2nd arg, currently unused) is kept in the signature for
% symmetry with future per-library expansions; alias dispatch alone covers
% the lookup today.
%
% Special case: the IMU library contains BOTH lsm_visual_block and
% adxl_visual_block as separate sibling subsystems.

    if strcmp(alias, 'IMU_ADXL')
        src = 'imu_block_visual/adxl_visual_block';
    elseif strcmp(alias, 'IMU_LSM')
        src = 'imu_block_visual/lsm_visual_block';
    elseif strcmp(alias, 'BARO')
        src = 'baro_block_visual/baro_visual_block';
    elseif strcmp(alias, 'MAG')
        src = 'mag_block_visual/mag_visual_block';
    elseif strcmp(alias, 'GPS')
        src = 'gps_block_visual/gps_visual_block';
    else
        error('add_sensor_block_:Unknown', 'Unknown alias %s', alias);
    end

    dest = [model_name '/' alias];
    add_block(src, dest);
    set_param(dest, 'Position', pos);
end


% =========================================================================
function build_imu_frame_switch_(model_name)
% Take LSM IMU output (accel_g, gyro_dps in std-body) and produce
% accel_fw_mps2 and gyro_fw_radps for the attitude block.
%
% Each path uses TWO small MATLAB Function blocks:
%   imu_unit_convert  — accel_g->m/s^2 and gyro_dps->rad/s
%   FrameSwitch_Accel — body-std -> body-fw via casper_frame_switch_body
%   FrameSwitch_Gyro  — body-std -> body-fw via casper_frame_switch_body
%
% We use inline MATLAB Function blocks (each calling the T07
% casper_frame_switch_body function) rather than linking the
% frame_switch_block library, because that library packages all three
% switches into one subsystem whose unused inports (vec_NED 3x1, q_std
% 4x1) cause Simulink's type-propagation to fail when stub-fed.

    % -- Unit convert (single MATLAB Fn for both accel and gyro) ----------
    %
    % IMPORTANT: the visual IMU block (imu_block_visual.slx) builds the
    % accel measurement via Sensor Fusion Toolbox's imuSensor, which
    % follows the industry convention:
    %     accel_meas = -R_b_from_n * (a_inertial - g_inertial)
    % i.e. the IMU returns the *negated* specific force, so on the pad
    % (a_inertial = 0) you get accel_meas = +g pointing in the body-axis
    % direction OPPOSITE the gravity vector (industry-standard: IMUs read
    % +g on the axis pointing UP against gravity).
    %
    % Meanwhile the legacy MATLAB driver (casper_phase0_run.m) uses
    % casper_imu_lsm_model() which does NOT apply that final negation; it
    % returns the raw specific force a_specific = R_b_from_n * (a - g).
    %
    % The downstream T07 frame switch and casper_attitude_tick / static_init
    % expect the legacy convention (on-pad accel = [0,+g,0] in body-fw,
    % asserted by run_pad_only_test.m line 51). Without compensation, the
    % visual model would feed accel = [0,-g,0] to the attitude estimator,
    % which makes static_init compute roll = -pi/2 instead of +pi/2 and
    % produces a quaternion 180 deg about X off from truth -- causing the
    % observed 80-180 deg attitude error and the downstream EKF altitude
    % "going negative" bug (rotated accel projects to nav-Z as -g instead
    % of +g, EKF integrates downward).
    %
    % Fix: flip the sign of the accel here so the rest of the chain sees
    % the same byte-exact convention as the legacy driver. Gyro is sign-
    % preserved (imuSensor's gyro convention already matches the legacy).
    uc = [model_name '/imu_unit_convert'];
    add_block('simulink/User-Defined Functions/MATLAB Function', uc);
    set_param(uc, 'Position', [800 40 980 160]);
    set_matlab_fn_script_(uc, [ ...
        'function [a_mps2_std, g_radps_std] = fcn(accel_g, gyro_dps)' newline ...
        '%#codegen' newline ...
        '% Sign-flip: convert imuSensor industry convention (+ g UP on pad)' newline ...
        '% to the legacy casper_imu_lsm_model convention (a_specific = a-g).' newline ...
        'a_mps2_std  = -double(accel_g(:)) * 9.80665;' newline ...
        'g_radps_std =  double(gyro_dps(:)) * (pi/180);' newline ...
        'end' newline]);

    add_line(model_name, 'IMU_LSM/1', 'imu_unit_convert/1', 'autorouting', 'on');
    add_line(model_name, 'IMU_LSM/2', 'imu_unit_convert/2', 'autorouting', 'on');

    % -- Inline frame switch for accel (body std -> body fw) --------------
    fs_a = [model_name '/FrameSwitch_Accel'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fs_a);
    set_param(fs_a, 'Position', [1020 30 1180 80]);
    set_matlab_fn_script_(fs_a, frame_switch_body_script_());
    add_line(model_name, 'imu_unit_convert/1', 'FrameSwitch_Accel/1', 'autorouting', 'on');

    % -- Inline frame switch for gyro -------------------------------------
    fs_g = [model_name '/FrameSwitch_Gyro'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fs_g);
    set_param(fs_g, 'Position', [1020 160 1180 210]);
    set_matlab_fn_script_(fs_g, frame_switch_body_script_());
    add_line(model_name, 'imu_unit_convert/2', 'FrameSwitch_Gyro/1', 'autorouting', 'on');
end


% =========================================================================
function build_mag_frame_switch_(model_name)
% Take MAG output (field_uT in std-body, port 1) -> mag_fw_uT for attitude.
% Inline MATLAB Function (same reason as build_imu_frame_switch_).
    fs_m = [model_name '/FrameSwitch_Mag'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fs_m);
    set_param(fs_m, 'Position', [1020 560 1180 610]);
    set_matlab_fn_script_(fs_m, frame_switch_body_script_());
    add_line(model_name, 'MAG/1', 'FrameSwitch_Mag/1', 'autorouting', 'on');
end


% =========================================================================
function src = frame_switch_body_script_()
%FRAME_SWITCH_BODY_SCRIPT_ MATLAB Function body for the body-std -> body-fw
% frame switch. Mirrors casper_frame_switch_body.m / T07's body_switch.

    src = [ ...
        'function vec_fw = fcn(vec_std)' newline ...
        '%#codegen' newline ...
        'v = double(vec_std(:));' newline ...
        'vec_fw = [ v(2); v(1); -v(3) ];' newline ...
        'end' newline];
end


% =========================================================================
function build_attitude_subsystem_(model_name)
% Wrap casper_attitude_tick in a MATLAB Function block. Apply the mag
% suppression gate via Unit Delay (Z^-1) on init_complete fed into a
% boolean AND with mag data_ready. The Unit Delay breaks the algebraic
% loop (init_complete is consumed AND produced by attitude in the same
% step in the firmware reference, so we must delay one step in Simulink).

    sub = [model_name '/ATTITUDE'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [1260 60 1480 260]);

    % Subsystem inports: a_fw (3), g_fw (3), mag_fw (3), mag_data_ready (1)
    add_block('built-in/Inport',  [sub '/a_fw_mps2']);   set_param([sub '/a_fw_mps2'],  'Port', '1', 'Position', [20  40  50  60]);
    add_block('built-in/Inport',  [sub '/g_fw_radps']);  set_param([sub '/g_fw_radps'], 'Port', '2', 'Position', [20  80  50 100]);
    add_block('built-in/Inport',  [sub '/mag_fw_uT']);   set_param([sub '/mag_fw_uT'],  'Port', '3', 'Position', [20 120  50 140]);
    add_block('built-in/Inport',  [sub '/mag_dr']);      set_param([sub '/mag_dr'],     'Port', '4', 'Position', [20 160  50 180]);

    % Unit delay for init_complete feedback (initial = false; breaks alg loop)
    ud = [sub '/InitCompleteDelay'];
    add_block('simulink/Discrete/Unit Delay', ud);
    set_param(ud, 'InitialCondition', '0', 'SampleTime', '-1', ...
        'Position', [80 220 130 250]);

    % AND gate: mag_new_eff = mag_dr AND init_complete_delayed
    and_blk = [sub '/MagGateAND'];
    add_block('simulink/Logic and Bit Operations/Logical Operator', and_blk);
    set_param(and_blk, 'Operator', 'AND', 'Inputs', '2', ...
        'Position', [160 160 200 200]);
    add_line(sub, 'mag_dr/1',           'MagGateAND/1', 'autorouting', 'on');
    add_line(sub, 'InitCompleteDelay/1','MagGateAND/2', 'autorouting', 'on');

    % MATLAB Function: attitude_tick wrapper (mirrors T09 build_attitude_block).
    fb = [sub '/attitude_tick'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fb);
    set_param(fb, 'Position', [240 40 420 200]);
    set_matlab_fn_script_(fb, [ ...
        'function [quat_fw, gyro_bias, heading_sigma, init_complete] = fcn(' ...
            'accel, gyro, mag, mag_new_eff, mode_pad, dt)' newline ...
        '%#codegen' newline ...
        'coder.extrinsic(''attitude_step_helper'');' newline ...
        '% Preallocate outputs so chart parser can infer types.' newline ...
        'quat_fw       = [1;0;0;0];' newline ...
        'gyro_bias     = zeros(3,1);' newline ...
        'heading_sigma = double(0);' newline ...
        'init_complete = false;' newline ...
        '[quat_fw, gyro_bias, heading_sigma, init_complete] = ' ...
            'attitude_step_helper(' ...
            'double(accel), double(gyro), double(mag), ' ...
            'logical(mag_new_eff), logical(mode_pad), double(dt));' newline ...
        'end' newline]);

    % mode_pad: hard-pinned true for now (visual model is pad-only smoke);
    % a flight-mode latch upgrade is straightforward but out of scope here.
    c_pad = [sub '/C_ModePad'];
    add_block('simulink/Sources/Constant', c_pad);
    set_param(c_pad, 'Value', 'true', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', '-1', 'Position', [160 215 200 235]);

    c_dt = [sub '/C_Dt'];
    add_block('simulink/Sources/Constant', c_dt);
    set_param(c_dt, 'Value', '1e-3', 'SampleTime', '-1', ...
        'Position', [160 245 200 265]);

    % Wire attitude_tick inputs (port order: accel, gyro, mag, mag_new_eff,
    %                                       mode_pad, dt)
    add_line(sub, 'a_fw_mps2/1',  'attitude_tick/1', 'autorouting', 'on');
    add_line(sub, 'g_fw_radps/1', 'attitude_tick/2', 'autorouting', 'on');
    add_line(sub, 'mag_fw_uT/1',  'attitude_tick/3', 'autorouting', 'on');
    add_line(sub, 'MagGateAND/1', 'attitude_tick/4', 'autorouting', 'on');
    add_line(sub, 'C_ModePad/1',  'attitude_tick/5', 'autorouting', 'on');
    add_line(sub, 'C_Dt/1',       'attitude_tick/6', 'autorouting', 'on');

    % Feed init_complete back into the Unit Delay (closes the suppression loop).
    add_line(sub, 'attitude_tick/4', 'InitCompleteDelay/1', 'autorouting', 'on');

    % Outports: quat_fw, gyro_bias, heading_sigma, init_complete
    outs = {'quat_fw', 'gyro_bias', 'heading_sigma', 'init_complete'};
    for k = 1:4
        op = [sub '/' outs{k}];
        add_block('built-in/Outport', op);
        set_param(op, 'Port', num2str(k), ...
            'Position', [460 (30 + (k-1)*40) 490 (50 + (k-1)*40)]);
        add_line(sub, ['attitude_tick/' num2str(k)], [outs{k} '/1'], ...
            'autorouting', 'on');
    end

    % Top-level wiring into ATTITUDE subsystem
    add_line(model_name, 'FrameSwitch_Accel/1', 'ATTITUDE/1', 'autorouting', 'on');
    add_line(model_name, 'FrameSwitch_Gyro/1',  'ATTITUDE/2', 'autorouting', 'on');
    add_line(model_name, 'FrameSwitch_Mag/1',   'ATTITUDE/3', 'autorouting', 'on');
    add_line(model_name, 'MAG/4',               'ATTITUDE/4', 'autorouting', 'on');
end


% =========================================================================
function build_eskf_subsystem_(model_name)
% Wrap the ESKF predict/update chain with an init gate.
%
% Pattern (per plan §4 fallback): use a Switch+Memory combo.
%   - Memory blocks hold initial state [alt; vel; accel_bias; baro_bias].
%   - The ESKF MATLAB Function runs every step, but its outputs are gated
%     by a Switch on init_complete: when init_complete is FALSE, the
%     Switch passes the Memory (held initial state); when TRUE, the
%     Switch passes the ESKF's just-computed state.
%   - This is semantically equivalent to "don't run EKF until init done"
%     because the held value never advances during the gated period and
%     gets re-initialized cleanly on the rising edge (the persistent
%     ESKF state inside the MATLAB Fn block is initialized from the first
%     baro sample at the rising edge; see fcn body).

    sub = [model_name '/ESKF'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [1540 380 1780 660]);

    % Inports: accel_zup (3), baro_alt (1), baro_dr (1), mach (1),
    %          zupt_trig (1), init_complete (1)
    in_names = { ...
        'accel_zup_mps2', '1'; ...
        'baro_alt_m',     '1'; ...
        'baro_new',       '1'; ...
        'mach',           '1'; ...
        'zupt_trig',      '1'; ...
        'init_complete',  '1'};
    for k = 1:size(in_names, 1)
        nm = in_names{k, 1};
        add_block('built-in/Inport', [sub '/' nm]);
        set_param([sub '/' nm], 'Port', num2str(k), ...
            'Position', [20 (40 + (k-1)*40) 50 (60 + (k-1)*40)]);
    end

    % Selector: accel_zup is a 3x1 nav-Zup vector; ESKF expects the up
    % component (index 3, since R_quat(q)*a_fw is already Zup, take .Z).
    sel_z = [sub '/SelZ'];
    add_block('simulink/Signal Routing/Selector', sel_z);
    set_param(sel_z, ...
        'NumberOfDimensions', '1', ...
        'IndexOptions',       'Index vector (dialog)', ...
        'Indices',            '3', ...
        'InputPortWidth',     '3', ...
        'Position',           [80 35 120 55]);
    add_line(sub, 'accel_zup_mps2/1', 'SelZ/1', 'autorouting', 'on');

    % MATLAB Function: ESKF step (mirrors T08 build_eskf_block).
    fb = [sub '/eskf_step'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fb);
    set_param(fb, 'Position', [160 30 360 280]);
    set_matlab_fn_script_(fb, [ ...
        'function [state_x, P_diag, mach_gate_active, ungate_counter, ' ...
            'last_baro_innov, last_zupt_innov] = fcn(' ...
            'accel_nav_up, baro_alt, baro_new, mach_in, zupt_trig, init_done)' newline ...
        '%#codegen' newline ...
        'coder.extrinsic(''eskf_step_helper'');' newline ...
        '% All outputs preallocated to fixed sizes so the chart parser can' newline ...
        '% statically infer types even with extrinsic call.' newline ...
        'state_x = zeros(4,1);' newline ...
        'P_diag  = zeros(4,1);' newline ...
        'mach_gate_active = double(0);' newline ...
        'ungate_counter   = double(0);' newline ...
        'last_baro_innov  = double(0);' newline ...
        'last_zupt_innov  = double(0);' newline ...
        '[state_x, P_diag, mach_gate_active, ungate_counter, ' ...
            'last_baro_innov, last_zupt_innov] = eskf_step_helper(' ...
            'double(accel_nav_up), double(baro_alt), logical(baro_new), ' ...
            'double(mach_in), logical(zupt_trig), logical(init_done));' newline ...
        'end' newline]);

    add_line(sub, 'SelZ/1',           'eskf_step/1', 'autorouting', 'on');
    add_line(sub, 'baro_alt_m/1',     'eskf_step/2', 'autorouting', 'on');
    add_line(sub, 'baro_new/1',       'eskf_step/3', 'autorouting', 'on');
    add_line(sub, 'mach/1',           'eskf_step/4', 'autorouting', 'on');
    add_line(sub, 'zupt_trig/1',      'eskf_step/5', 'autorouting', 'on');
    add_line(sub, 'init_complete/1',  'eskf_step/6', 'autorouting', 'on');

    % Outports
    eskf_outs = {'state_x','P_diag','mach_gate_active','ungate_counter','baro_innov','zupt_innov'};
    for k = 1:numel(eskf_outs)
        op = [sub '/' eskf_outs{k}];
        add_block('built-in/Outport', op);
        set_param(op, 'Port', num2str(k), ...
            'Position', [420 (30 + (k-1)*40) 450 (50 + (k-1)*40)]);
        add_line(sub, ['eskf_step/' num2str(k)], [eskf_outs{k} '/1'], 'autorouting', 'on');
    end

    % --- Top-level wiring into ESKF subsystem ----------------------------
    % accel_zup: rotate a_fw_mps2 by attitude quat (q_fw). Inline as MATLAB Fn.
    rot = [model_name '/RotateBodyToNavZup'];
    add_block('simulink/User-Defined Functions/MATLAB Function', rot);
    set_param(rot, 'Position', [1260 380 1440 480]);
    set_matlab_fn_script_(rot, [ ...
        'function a_nav_zup = fcn(a_body_fw, q_fw)' newline ...
        '%#codegen' newline ...
        'q = double(q_fw(:));' newline ...
        'a = double(a_body_fw(:));' newline ...
        'nq = sqrt(q(1)*q(1)+q(2)*q(2)+q(3)*q(3)+q(4)*q(4));' newline ...
        'if nq < 1e-12; nq = 1; end' newline ...
        'q = q / nq;' newline ...
        'w = q(1); x = q(2); y = q(3); z = q(4);' newline ...
        'R = [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w);' newline ...
        '     2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w);' newline ...
        '     2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)];' newline ...
        'a_nav_zup = R * a;' newline ...
        'end' newline]);

    add_line(model_name, 'FrameSwitch_Accel/1', 'RotateBodyToNavZup/1', 'autorouting', 'on');
    add_line(model_name, 'ATTITUDE/1',          'RotateBodyToNavZup/2', 'autorouting', 'on');

    % Rate transition to 2 ms (500 Hz - snapped from 416 Hz)
    rt_eskf_acc = [model_name '/RT_eskf_accel'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_eskf_acc);
    set_param(rt_eskf_acc, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 410 1510 440]);
    add_line(model_name, 'RotateBodyToNavZup/1', 'RT_eskf_accel/1', 'autorouting', 'on');

    rt_eskf_mach = [model_name '/RT_eskf_mach'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_eskf_mach);
    set_param(rt_eskf_mach, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 510 1510 540]);
    % Pull mach from SensorBusCreate via a Bus Selector
    bs_mach = [model_name '/BS_MachForEKF'];
    add_block('simulink/Signal Routing/Bus Selector', bs_mach);
    set_param(bs_mach, 'OutputSignals', 'mach', 'Position', [1260 510 1300 540]);
    add_line(model_name, 'SensorBusCreate/1', 'BS_MachForEKF/1', 'autorouting', 'on');
    add_line(model_name, 'BS_MachForEKF/1', 'RT_eskf_mach/1', 'autorouting', 'on');

    % ZUPT trigger: |vel_NED| < ZuptThreshold (compute from truth via Bus Sel)
    bs_vel = [model_name '/BS_VelForZupt'];
    add_block('simulink/Signal Routing/Bus Selector', bs_vel);
    set_param(bs_vel, 'OutputSignals', 'vel_NED', 'Position', [1260 580 1300 610]);
    add_line(model_name, 'SensorBusCreate/1', 'BS_VelForZupt/1', 'autorouting', 'on');

    zupt_fn = [model_name '/ZuptTrigger'];
    add_block('simulink/User-Defined Functions/MATLAB Function', zupt_fn);
    set_param(zupt_fn, 'Position', [1320 580 1450 640]);
    set_matlab_fn_script_(zupt_fn, [ ...
        'function trig = fcn(vel_NED, thresh)' newline ...
        '%#codegen' newline ...
        'v = double(vel_NED(:));' newline ...
        'trig = sqrt(v(1)^2 + v(2)^2 + v(3)^2) < double(thresh);' newline ...
        'end' newline]);
    c_zth = [model_name '/C_ZuptThresh'];
    add_block('simulink/Sources/Constant', c_zth);
    set_param(c_zth, 'Value', 'Estimator.ZuptThreshold', 'SampleTime', '-1', ...
        'Position', [1260 640 1300 660]);
    add_line(model_name, 'BS_VelForZupt/1', 'ZuptTrigger/1', 'autorouting', 'on');
    add_line(model_name, 'C_ZuptThresh/1',  'ZuptTrigger/2', 'autorouting', 'on');

    rt_eskf_zupt = [model_name '/RT_eskf_zupt'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_eskf_zupt);
    set_param(rt_eskf_zupt, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 600 1510 630]);
    add_line(model_name, 'ZuptTrigger/1', 'RT_eskf_zupt/1', 'autorouting', 'on');

    % Init complete from attitude (rate-transitioned to EKF rate)
    rt_eskf_init = [model_name '/RT_eskf_init'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_eskf_init);
    set_param(rt_eskf_init, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 240 1510 270]);
    add_line(model_name, 'ATTITUDE/4', 'RT_eskf_init/1', 'autorouting', 'on');

    % Wire ESKF subsystem inputs (rate-matched 2 ms domain).
    add_line(model_name, 'RT_eskf_accel/1', 'ESKF/1', 'autorouting', 'on');
    add_line(model_name, 'BARO/2',          'ESKF/2', 'autorouting', 'on');
    add_line(model_name, 'BARO/4',          'ESKF/3', 'autorouting', 'on');
    add_line(model_name, 'RT_eskf_mach/1',  'ESKF/4', 'autorouting', 'on');
    add_line(model_name, 'RT_eskf_zupt/1',  'ESKF/5', 'autorouting', 'on');
    add_line(model_name, 'RT_eskf_init/1',  'ESKF/6', 'autorouting', 'on');
end


% =========================================================================
function build_validation_subsystem_(model_name)
% Inline 14-channel logger. Each signal feeds a To Workspace block named
% identically to T10's validation_block convention so downstream metrics
% functions resolve unchanged.
%
% Channels (per T10 build_validation_block.m):
%   TruthBus_in            -> log_truth_bus
%   Estimate_state_x       -> log_est_state_x
%   Estimate_state_P_diag  -> log_est_state_P_diag
%   Estimate_attitude_quat -> log_est_quat
%   Estimate_mach_gate     -> log_est_mach_gate_active
%   Estimate_ungate_ctr    -> log_est_ungate_counter
%   Estimate_baro_innov    -> log_est_baro_innov
%   Estimate_zupt_innov    -> log_est_zupt_innov
%   Sensor_IMU             -> log_sensor_imu
%   Sensor_ADXL            -> log_sensor_adxl
%   Sensor_Baro            -> log_sensor_baro
%   Sensor_Mag             -> log_sensor_mag
%   Sensor_GPS             -> log_sensor_gps
%   RadioTX_active         -> log_radio_tx_active

    sub = [model_name '/VALIDATION'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [1820 60 2080 900]);

    % 14 inports + 14 To Workspace logs.
    chans = { ...
        'TruthBus_in',            'log_truth_bus'; ...
        'Estimate_state_x',       'log_est_state_x'; ...
        'Estimate_state_P_diag',  'log_est_state_P_diag'; ...
        'Estimate_attitude_quat', 'log_est_quat'; ...
        'Estimate_mach_gate',     'log_est_mach_gate_active'; ...
        'Estimate_ungate_counter','log_est_ungate_counter'; ...
        'Estimate_baro_innov',    'log_est_baro_innov'; ...
        'Estimate_zupt_innov',    'log_est_zupt_innov'; ...
        'Sensor_IMU',             'log_sensor_imu'; ...
        'Sensor_ADXL',            'log_sensor_adxl'; ...
        'Sensor_Baro',            'log_sensor_baro'; ...
        'Sensor_Mag',             'log_sensor_mag'; ...
        'Sensor_GPS',             'log_sensor_gps'; ...
        'RadioTX_active',         'log_radio_tx_active'};

    for k = 1:size(chans, 1)
        in_nm = chans{k, 1};
        ws_nm = chans{k, 2};
        add_block('built-in/Inport', [sub '/' in_nm]);
        set_param([sub '/' in_nm], 'Port', num2str(k), ...
            'Position', [20 (30 + (k-1)*50) 50 (50 + (k-1)*50)]);
        tw = [sub '/log_' in_nm];
        add_block('simulink/Sinks/To Workspace', tw);
        set_param(tw, ...
            'VariableName', ws_nm, ...
            'SaveFormat',   'StructureWithTime', ...
            'SampleTime',   '-1', ...
            'Position', [240 (30 + (k-1)*50) 340 (50 + (k-1)*50)]);

        if strcmp(in_nm, 'TruthBus_in')
            % SensorInputBus is non-virtual; To Workspace can't accept a
            % bus directly. Pipeline: non-virtual bus -> To Virtual Bus
            % -> Bus to Vector -> To Workspace.
            tvb = [sub '/TVB_TruthBus'];
            add_block('simulink/Signal Attributes/Signal Conversion', tvb);
            set_param(tvb, ...
                'ConversionOutput', 'Virtual bus', ...
                'Position', [80 (30 + (k-1)*50) 140 (50 + (k-1)*50)]);
            bv = [sub '/BV_TruthBus'];
            add_block('simulink/Signal Attributes/Bus to Vector', bv);
            set_param(bv, 'Position', [160 (30 + (k-1)*50) 220 (50 + (k-1)*50)]);
            add_line(sub, [in_nm '/1'],     'TVB_TruthBus/1',     'autorouting', 'on');
            add_line(sub, 'TVB_TruthBus/1', 'BV_TruthBus/1',      'autorouting', 'on');
            add_line(sub, 'BV_TruthBus/1',  ['log_' in_nm '/1'],  'autorouting', 'on');
        else
            add_line(sub, [in_nm '/1'], ['log_' in_nm '/1'], 'autorouting', 'on');
        end
    end

    % --- Wire top-level signals into VALIDATION inports ------------------
    % Truth bus -> TruthBus_in (port 1)
    add_line(model_name, 'SensorBusCreate/1', 'VALIDATION/1', 'autorouting', 'on');
    % Estimate channels (ports 2..8) come from ESKF + ATTITUDE
    add_line(model_name, 'ESKF/1', 'VALIDATION/2', 'autorouting', 'on'); % state_x
    add_line(model_name, 'ESKF/2', 'VALIDATION/3', 'autorouting', 'on'); % P_diag
    add_line(model_name, 'ATTITUDE/1', 'VALIDATION/4', 'autorouting', 'on'); % quat_fw
    add_line(model_name, 'ESKF/3', 'VALIDATION/5', 'autorouting', 'on'); % mach_gate
    add_line(model_name, 'ESKF/4', 'VALIDATION/6', 'autorouting', 'on'); % ungate ctr
    add_line(model_name, 'ESKF/5', 'VALIDATION/7', 'autorouting', 'on'); % baro_innov
    add_line(model_name, 'ESKF/6', 'VALIDATION/8', 'autorouting', 'on'); % zupt_innov

    % Sensor channels (ports 9..14): for IMU and ADXL we log the raw
    % (std-body) accel output of the visual sensor block. Baro logs
    % press_pa (port 1), Mag logs field_uT (port 1), GPS logs lat_deg7
    % (port 1) — single-channel reps; the full multi-port output is
    % already on the diagram for visual inspection.
    add_line(model_name, 'IMU_LSM/1',  'VALIDATION/9',  'autorouting', 'on');
    add_line(model_name, 'IMU_ADXL/1', 'VALIDATION/10', 'autorouting', 'on');
    add_line(model_name, 'BARO/1',     'VALIDATION/11', 'autorouting', 'on');
    add_line(model_name, 'MAG/1',      'VALIDATION/12', 'autorouting', 'on');
    add_line(model_name, 'GPS/1',      'VALIDATION/13', 'autorouting', 'on');
    add_line(model_name, 'MAG/3',      'VALIDATION/14', 'autorouting', 'on'); % radio_active
end


% =========================================================================
function add_top_level_note_(model_name)
    add_block('built-in/Note', [model_name '/topnote']);
    note_text = sprintf([ ...
        'CASPER-2 Phase 0 Visual Top-Level Model\n' ...
        '\n' ...
        'Built by build_casper_sim_phase0.m. Visual auditability of the\n' ...
        'truth -> sensors -> frame switch -> attitude -> EKF -> validation\n' ...
        'data flow. Library-linked subsystems for T03..T10; click any block.\n' ...
        '\n' ...
        'Rate snap: LSM/ADXL 833/800 -> 1000 Hz; EKF 416 -> 500 Hz.\n' ...
        'Canonical byte-exact regression remains casper_phase0_run.m.\n' ...
        '\n' ...
        'EKF parallel comparison:\n' ...
        '  - ESKF subsystem    : 4-state vertical EKF (original)\n' ...
        '  - ESKF16 subsystem  : 16-state error-state EKF (new)\n' ...
        '  - 4 compare scopes  : Altitude / VerticalVelocity / AttitudeError / Biases\n' ...
        '  - After sim: cd integration; generate_ekf_comparison_report\n' ...
        '\n' ...
        'See STATUS_VISUAL.md for the construction notes.']);
    set_param([model_name '/topnote'], ...
        'Text',     note_text, ...
        'Position', [40 920 600 1020]);
end


% =========================================================================
function build_scope_panels_(model_name)
%BUILD_SCOPE_PANELS_ Add 4 real-time Scope blocks comparing truth vs estimate.
%
% Scopes:
%   Scope_Altitude         (2 ch) : truth_alt vs est_alt              [m]
%   Scope_VerticalVelocity (2 ch) : truth_vel_up vs est_vel           [m/s]
%   Scope_Euler            (6 ch) : truth/est roll, pitch, yaw        [deg]
%   Scope_AttitudeError    (1 ch) : magnitude of quat angle diff      [deg]
%
% All scopes set OpenAtSimulationStart='on' so they pop up automatically.
%
% Plumbing strategy:
%   - Tap SensorBusCreate (TruthBus) via Bus Selectors to get pos_NED,
%     vel_NED, quat_std without touching the existing wiring.
%   - Tap ESKF/1 (state_x) via Selectors for alt (idx 1) and vel (idx 2).
%   - All quat->euler and quat-error math lives inside an EulerConvert
%     subsystem to keep the top diagram clean.

    % --- Bus Selectors at top level (taps existing SensorBusCreate line) ----
    bs_pos = [model_name '/BS_PosForScope'];
    add_block('simulink/Signal Routing/Bus Selector', bs_pos);
    set_param(bs_pos, 'OutputSignals', 'pos_NED', ...
        'Position', [1260 1060 1320 1090]);
    add_line(model_name, 'SensorBusCreate/1', 'BS_PosForScope/1', 'autorouting', 'on');

    bs_vel = [model_name '/BS_VelForScope'];
    add_block('simulink/Signal Routing/Bus Selector', bs_vel);
    set_param(bs_vel, 'OutputSignals', 'vel_NED', ...
        'Position', [1260 1110 1320 1140]);
    add_line(model_name, 'SensorBusCreate/1', 'BS_VelForScope/1', 'autorouting', 'on');

    bs_q = [model_name '/BS_QuatForScope'];
    add_block('simulink/Signal Routing/Bus Selector', bs_q);
    set_param(bs_q, 'OutputSignals', 'quat_std', ...
        'Position', [1260 1160 1320 1190]);
    add_line(model_name, 'SensorBusCreate/1', 'BS_QuatForScope/1', 'autorouting', 'on');

    % --- Selectors to peel scalars off pos_NED, vel_NED, state_x -----------
    % Truth altitude: -pos_NED(3) (NED Z-down -> up positive)
    sel_pz = [model_name '/SelPosZ'];
    add_block('simulink/Signal Routing/Selector', sel_pz);
    set_param(sel_pz, ...
        'NumberOfDimensions', '1', ...
        'IndexOptions',       'Index vector (dialog)', ...
        'Indices',            '3', ...
        'InputPortWidth',     '3', ...
        'Position', [1360 1060 1400 1090]);
    add_line(model_name, 'BS_PosForScope/1', 'SelPosZ/1', 'autorouting', 'on');

    gain_alt = [model_name '/GainAltFlip'];
    add_block('simulink/Math Operations/Gain', gain_alt);
    set_param(gain_alt, 'Gain', '-1', 'Position', [1420 1060 1450 1090]);
    add_line(model_name, 'SelPosZ/1', 'GainAltFlip/1', 'autorouting', 'on');

    % Truth vertical velocity: -vel_NED(3)
    sel_vz = [model_name '/SelVelZ'];
    add_block('simulink/Signal Routing/Selector', sel_vz);
    set_param(sel_vz, ...
        'NumberOfDimensions', '1', ...
        'IndexOptions',       'Index vector (dialog)', ...
        'Indices',            '3', ...
        'InputPortWidth',     '3', ...
        'Position', [1360 1110 1400 1140]);
    add_line(model_name, 'BS_VelForScope/1', 'SelVelZ/1', 'autorouting', 'on');

    gain_vel = [model_name '/GainVelFlip'];
    add_block('simulink/Math Operations/Gain', gain_vel);
    set_param(gain_vel, 'Gain', '-1', 'Position', [1420 1110 1450 1140]);
    add_line(model_name, 'SelVelZ/1', 'GainVelFlip/1', 'autorouting', 'on');

    % Estimate altitude: state_x(1); estimate vertical velocity: state_x(2)
    sel_eax = [model_name '/SelEstAlt'];
    add_block('simulink/Signal Routing/Selector', sel_eax);
    set_param(sel_eax, ...
        'NumberOfDimensions', '1', ...
        'IndexOptions',       'Index vector (dialog)', ...
        'Indices',            '1', ...
        'InputPortWidth',     '4', ...
        'Position', [1620 1060 1660 1090]);
    add_line(model_name, 'ESKF/1', 'SelEstAlt/1', 'autorouting', 'on');

    sel_evz = [model_name '/SelEstVel'];
    add_block('simulink/Signal Routing/Selector', sel_evz);
    set_param(sel_evz, ...
        'NumberOfDimensions', '1', ...
        'IndexOptions',       'Index vector (dialog)', ...
        'Indices',            '2', ...
        'InputPortWidth',     '4', ...
        'Position', [1620 1110 1660 1140]);
    add_line(model_name, 'ESKF/1', 'SelEstVel/1', 'autorouting', 'on');

    % --- Build the EulerConvert subsystem (quat->Euler, quat error) --------
    build_euler_convert_subsystem_(model_name);

    % Wire truth quat (3x1 -> quat is 4x1 in SensorInputBus) into EulerConvert
    % EulerConvert input ports: 1=truth_quat (4x1), 2=est_quat (4x1)
    add_line(model_name, 'BS_QuatForScope/1', 'EulerConvert/1', 'autorouting', 'on');
    add_line(model_name, 'ATTITUDE/1',        'EulerConvert/2', 'autorouting', 'on');

    % --- Scope_Altitude (2 channels: truth_alt, est_alt) -------------------
    sc_alt = [model_name '/Scope_Altitude'];
    add_block('simulink/Sinks/Scope', sc_alt);
    set_param(sc_alt, ...
        'NumInputPorts',          '2', ...
        'OpenAtSimulationStart',  'on', ...
        'Position',               [1720 1040 1780 1100]);
    set_scope_title_(sc_alt, 'Altitude - truth vs estimate [m]');
    add_line(model_name, 'GainAltFlip/1', 'Scope_Altitude/1', 'autorouting', 'on');
    add_line(model_name, 'SelEstAlt/1',   'Scope_Altitude/2', 'autorouting', 'on');

    % --- Scope_VerticalVelocity (2 channels) -------------------------------
    sc_vel = [model_name '/Scope_VerticalVelocity'];
    add_block('simulink/Sinks/Scope', sc_vel);
    set_param(sc_vel, ...
        'NumInputPorts',          '2', ...
        'OpenAtSimulationStart',  'on', ...
        'Position',               [1720 1110 1780 1170]);
    set_scope_title_(sc_vel, 'Vertical velocity - truth vs estimate [m/s]');
    add_line(model_name, 'GainVelFlip/1', 'Scope_VerticalVelocity/1', 'autorouting', 'on');
    add_line(model_name, 'SelEstVel/1',   'Scope_VerticalVelocity/2', 'autorouting', 'on');

    % --- Scope_Euler (6 channels: truth/est roll, pitch, yaw, grouped) -----
    % EulerConvert outport order: 1=truth_euler_deg (3x1), 2=est_euler_deg (3x1),
    %                             3=truth_roll, 4=est_roll,
    %                             5=truth_pitch, 6=est_pitch,
    %                             7=truth_yaw, 8=est_yaw,
    %                             9=att_error_deg (scalar)
    sc_euler = [model_name '/Scope_Euler'];
    add_block('simulink/Sinks/Scope', sc_euler);
    set_param(sc_euler, ...
        'NumInputPorts',          '6', ...
        'OpenAtSimulationStart',  'on', ...
        'Position',               [1720 1180 1780 1270]);
    set_scope_title_(sc_euler, 'Orientation Euler - truth vs estimate [deg]');
    % Channel order: truth_roll, est_roll, truth_pitch, est_pitch, truth_yaw, est_yaw
    add_line(model_name, 'EulerConvert/3', 'Scope_Euler/1', 'autorouting', 'on');
    add_line(model_name, 'EulerConvert/4', 'Scope_Euler/2', 'autorouting', 'on');
    add_line(model_name, 'EulerConvert/5', 'Scope_Euler/3', 'autorouting', 'on');
    add_line(model_name, 'EulerConvert/6', 'Scope_Euler/4', 'autorouting', 'on');
    add_line(model_name, 'EulerConvert/7', 'Scope_Euler/5', 'autorouting', 'on');
    add_line(model_name, 'EulerConvert/8', 'Scope_Euler/6', 'autorouting', 'on');

    % --- Scope_AttitudeError (1 channel) -----------------------------------
    sc_aerr = [model_name '/Scope_AttitudeError'];
    add_block('simulink/Sinks/Scope', sc_aerr);
    set_param(sc_aerr, ...
        'NumInputPorts',          '1', ...
        'OpenAtSimulationStart',  'on', ...
        'Position',               [1720 1280 1780 1330]);
    set_scope_title_(sc_aerr, 'Attitude error magnitude [deg]');
    add_line(model_name, 'EulerConvert/9', 'Scope_AttitudeError/1', 'autorouting', 'on');
end


% =========================================================================
function build_euler_convert_subsystem_(model_name)
%BUILD_EULER_CONVERT_SUBSYSTEM_ Convert truth/est quats to Euler + quat error.
%
% Inputs:
%   1 : truth_quat (4x1, Hamilton scalar-first)
%   2 : est_quat   (4x1, Hamilton scalar-first)
%
% Outputs:
%   1 : truth_euler_deg (3x1 [roll;pitch;yaw])
%   2 : est_euler_deg   (3x1 [roll;pitch;yaw])
%   3 : truth_roll_deg  (scalar)
%   4 : est_roll_deg    (scalar)
%   5 : truth_pitch_deg (scalar)
%   6 : est_pitch_deg   (scalar)
%   7 : truth_yaw_deg   (scalar)
%   8 : est_yaw_deg     (scalar)
%   9 : att_error_deg   (scalar, magnitude of quat angle diff)

    sub = [model_name '/EulerConvert'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [1480 1180 1660 1320]);

    % --- Inports ---------------------------------------------------------
    add_block('built-in/Inport', [sub '/truth_quat']);
    set_param([sub '/truth_quat'], 'Port', '1', 'Position', [20 40 50 60]);
    add_block('built-in/Inport', [sub '/est_quat']);
    set_param([sub '/est_quat'],   'Port', '2', 'Position', [20 100 50 120]);

    % --- MATLAB Function: quat -> Euler (deg) for both quats + error -----
    fb = [sub '/quat_to_euler_and_error'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fb);
    set_param(fb, 'Position', [100 30 320 200]);
    set_matlab_fn_script_(fb, [ ...
        'function [truth_eul_deg, est_eul_deg, att_err_deg] = fcn(qt, qe)' newline ...
        '%#codegen' newline ...
        '% Hamilton scalar-first quaternion -> Euler (ZYX: roll,pitch,yaw) in deg.' newline ...
        '% Also computes magnitude of quat angle diff in deg.' newline ...
        '%' newline ...
        '% IMPORTANT FRAME NOTE:' newline ...
        '%   qt (truth_quat) arrives in std-aircraft body -> NED-down convention' newline ...
        '%     (X-fwd, Z-down). This is the convention RasAero / casper_truth_resample' newline ...
        '%     emits via TruthBus.quat_std.' newline ...
        '%   qe (est_quat) is in firmware body -> firmware nav (Y-nose, Z-up)' newline ...
        '%     convention, as produced by casper_attitude_tick.' newline ...
        '%' newline ...
        '%   Comparing the two quats DIRECTLY (without frame switching truth) gives' newline ...
        '%   a 90-180 deg "error" purely from the convention mismatch -- even when' newline ...
        '%   the estimator is perfect. We therefore convert qt -> qt_fw using the' newline ...
        '%   exact T_nav_quat * C_std * R_body'' composition that' newline ...
        '%   casper_frame_switch_quat() applies. Both factors are proper rotations' newline ...
        '%   (det=+1) so the DCM round-trips cleanly through quaternion form.' newline ...
        'qt = double(qt(:));' newline ...
        'qe = double(qe(:));' newline ...
        'nt = sqrt(qt(1)^2+qt(2)^2+qt(3)^2+qt(4)^2);' newline ...
        'ne = sqrt(qe(1)^2+qe(2)^2+qe(3)^2+qe(4)^2);' newline ...
        'if nt < 1e-12; nt = 1; end' newline ...
        'if ne < 1e-12; ne = 1; end' newline ...
        'qt = qt / nt;' newline ...
        'qe = qe / ne;' newline ...
        '% --- Frame switch: qt (std) -> qt_fw (firmware) ---' newline ...
        'qt_fw = quat_std_to_fw_(qt);' newline ...
        '% --- Euler (ZYX) from Hamilton [w x y z] ---' newline ...
        '% Display the truth Euler in firmware-frame too so the Euler scope' newline ...
        '% lines up with the estimator output channel-for-channel.' newline ...
        'truth_eul_deg = quat_to_eul_zyx_(qt_fw) * (180/pi);' newline ...
        'est_eul_deg   = quat_to_eul_zyx_(qe)    * (180/pi);' newline ...
        '% --- Quat angle diff: q_err = qt_fw^-1 * qe, angle = 2*acos(|w|) ---' newline ...
        '% Conjugate of unit quat = inverse.' newline ...
        'qt_fw_inv = [qt_fw(1); -qt_fw(2); -qt_fw(3); -qt_fw(4)];' newline ...
        'qe_q      = quat_mul_(qt_fw_inv, qe);' newline ...
        'w_abs     = min(1.0, abs(qe_q(1)));' newline ...
        'att_err_deg = 2 * acos(w_abs) * (180/pi);' newline ...
        'end' newline ...
        '' newline ...
        'function q_fw = quat_std_to_fw_(q_std)' newline ...
        '%#codegen' newline ...
        '% std-body-to-NED quat -> firmware-body-to-Zup quat. Mirrors' newline ...
        '% casper_frame_switch_quat(): C_fw = T_nav_quat * C_std * R_body''.' newline ...
        'C_std = dcm_from_quat_(q_std);' newline ...
        'R_body     = [ 0 1 0; 1 0 0; 0 0 -1 ];' newline ...
        'T_nav_quat = [ 0 1 0; 1 0 0; 0 0 -1 ];' newline ...
        'C_fw = T_nav_quat * C_std * R_body.'';' newline ...
        'q_fw = quat_from_dcm_(C_fw);' newline ...
        'if q_fw(1) < 0; q_fw = -q_fw; end' newline ...
        'end' newline ...
        '' newline ...
        'function C = dcm_from_quat_(q)' newline ...
        '%#codegen' newline ...
        'w = q(1); x = q(2); y = q(3); z = q(4);' newline ...
        'C = [ 1 - 2*(y*y + z*z),  2*(x*y - z*w),       2*(x*z + y*w);' newline ...
        '      2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w);' newline ...
        '      2*(x*z - y*w),      2*(y*z + x*w),       1 - 2*(x*x + y*y) ];' newline ...
        'end' newline ...
        '' newline ...
        'function q = quat_from_dcm_(C)' newline ...
        '%#codegen' newline ...
        '% Shepperd''s algorithm. Mirrors casper_frame_switch_quat local_quat_from_dcm.' newline ...
        'tr = C(1,1) + C(2,2) + C(3,3);' newline ...
        'qw = 0; qx = 0; qy = 0; qz = 0;' newline ...
        'if tr > 0' newline ...
        '    s = sqrt(tr + 1.0) * 2.0;' newline ...
        '    qw = 0.25 * s;' newline ...
        '    qx = (C(3,2) - C(2,3)) / s;' newline ...
        '    qy = (C(1,3) - C(3,1)) / s;' newline ...
        '    qz = (C(2,1) - C(1,2)) / s;' newline ...
        'elseif (C(1,1) > C(2,2)) && (C(1,1) > C(3,3))' newline ...
        '    s = sqrt(1.0 + C(1,1) - C(2,2) - C(3,3)) * 2.0;' newline ...
        '    qw = (C(3,2) - C(2,3)) / s;' newline ...
        '    qx = 0.25 * s;' newline ...
        '    qy = (C(1,2) + C(2,1)) / s;' newline ...
        '    qz = (C(1,3) + C(3,1)) / s;' newline ...
        'elseif C(2,2) > C(3,3)' newline ...
        '    s = sqrt(1.0 + C(2,2) - C(1,1) - C(3,3)) * 2.0;' newline ...
        '    qw = (C(1,3) - C(3,1)) / s;' newline ...
        '    qx = (C(1,2) + C(2,1)) / s;' newline ...
        '    qy = 0.25 * s;' newline ...
        '    qz = (C(2,3) + C(3,2)) / s;' newline ...
        'else' newline ...
        '    s = sqrt(1.0 + C(3,3) - C(1,1) - C(2,2)) * 2.0;' newline ...
        '    qw = (C(2,1) - C(1,2)) / s;' newline ...
        '    qx = (C(1,3) + C(3,1)) / s;' newline ...
        '    qy = (C(2,3) + C(3,2)) / s;' newline ...
        '    qz = 0.25 * s;' newline ...
        'end' newline ...
        'q = [qw; qx; qy; qz];' newline ...
        'n = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);' newline ...
        'if n > 0; q = q / n; end' newline ...
        'end' newline ...
        '' newline ...
        'function eul = quat_to_eul_zyx_(q)' newline ...
        '%#codegen' newline ...
        '% q = [w x y z]; eul = [roll; pitch; yaw] (rad), ZYX intrinsic convention.' newline ...
        'w = q(1); x = q(2); y = q(3); z = q(4);' newline ...
        '% Roll (x-axis rotation)' newline ...
        'sinr_cosp = 2 * (w*x + y*z);' newline ...
        'cosr_cosp = 1 - 2 * (x*x + y*y);' newline ...
        'roll  = atan2(sinr_cosp, cosr_cosp);' newline ...
        '% Pitch (y-axis rotation), clamp for safety near +/-90deg' newline ...
        'sinp = 2 * (w*y - z*x);' newline ...
        'if sinp >  1; sinp =  1; end' newline ...
        'if sinp < -1; sinp = -1; end' newline ...
        'pitch = asin(sinp);' newline ...
        '% Yaw (z-axis rotation)' newline ...
        'siny_cosp = 2 * (w*z + x*y);' newline ...
        'cosy_cosp = 1 - 2 * (y*y + z*z);' newline ...
        'yaw   = atan2(siny_cosp, cosy_cosp);' newline ...
        'eul = [roll; pitch; yaw];' newline ...
        'end' newline ...
        '' newline ...
        'function qo = quat_mul_(a, b)' newline ...
        '%#codegen' newline ...
        '% Hamilton quaternion product, scalar-first.' newline ...
        'aw = a(1); ax = a(2); ay = a(3); az = a(4);' newline ...
        'bw = b(1); bx = b(2); by = b(3); bz = b(4);' newline ...
        'qo = [ aw*bw - ax*bx - ay*by - az*bz;' newline ...
        '       aw*bx + ax*bw + ay*bz - az*by;' newline ...
        '       aw*by - ax*bz + ay*bw + az*bx;' newline ...
        '       aw*bz + ax*by - ay*bx + az*bw ];' newline ...
        'end' newline]);

    add_line(sub, 'truth_quat/1', 'quat_to_euler_and_error/1', 'autorouting', 'on');
    add_line(sub, 'est_quat/1',   'quat_to_euler_and_error/2', 'autorouting', 'on');

    % --- Selectors: split each 3x1 Euler vector into 3 scalars -----------
    % Truth Euler split
    sel_tr = [sub '/SelTruthRoll'];
    add_block('simulink/Signal Routing/Selector', sel_tr);
    set_param(sel_tr, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '1', ...
        'InputPortWidth', '3', 'Position', [360 30 400 50]);
    add_line(sub, 'quat_to_euler_and_error/1', 'SelTruthRoll/1', 'autorouting', 'on');

    sel_tp = [sub '/SelTruthPitch'];
    add_block('simulink/Signal Routing/Selector', sel_tp);
    set_param(sel_tp, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '2', ...
        'InputPortWidth', '3', 'Position', [360 60 400 80]);
    add_line(sub, 'quat_to_euler_and_error/1', 'SelTruthPitch/1', 'autorouting', 'on');

    sel_ty = [sub '/SelTruthYaw'];
    add_block('simulink/Signal Routing/Selector', sel_ty);
    set_param(sel_ty, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '3', ...
        'InputPortWidth', '3', 'Position', [360 90 400 110]);
    add_line(sub, 'quat_to_euler_and_error/1', 'SelTruthYaw/1', 'autorouting', 'on');

    % Estimate Euler split
    sel_er = [sub '/SelEstRoll'];
    add_block('simulink/Signal Routing/Selector', sel_er);
    set_param(sel_er, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '1', ...
        'InputPortWidth', '3', 'Position', [360 120 400 140]);
    add_line(sub, 'quat_to_euler_and_error/2', 'SelEstRoll/1', 'autorouting', 'on');

    sel_ep = [sub '/SelEstPitch'];
    add_block('simulink/Signal Routing/Selector', sel_ep);
    set_param(sel_ep, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '2', ...
        'InputPortWidth', '3', 'Position', [360 150 400 170]);
    add_line(sub, 'quat_to_euler_and_error/2', 'SelEstPitch/1', 'autorouting', 'on');

    sel_ey = [sub '/SelEstYaw'];
    add_block('simulink/Signal Routing/Selector', sel_ey);
    set_param(sel_ey, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '3', ...
        'InputPortWidth', '3', 'Position', [360 180 400 200]);
    add_line(sub, 'quat_to_euler_and_error/2', 'SelEstYaw/1', 'autorouting', 'on');

    % --- Outports --------------------------------------------------------
    out_specs = { ...
        'truth_euler_deg', 'quat_to_euler_and_error', 1; ...
        'est_euler_deg',   'quat_to_euler_and_error', 2; ...
        'truth_roll_deg',  'SelTruthRoll',            1; ...
        'est_roll_deg',    'SelEstRoll',              1; ...
        'truth_pitch_deg', 'SelTruthPitch',           1; ...
        'est_pitch_deg',   'SelEstPitch',             1; ...
        'truth_yaw_deg',   'SelTruthYaw',             1; ...
        'est_yaw_deg',     'SelEstYaw',               1; ...
        'att_err_deg',     'quat_to_euler_and_error', 3};

    for k = 1:size(out_specs, 1)
        op_name = out_specs{k, 1};
        src_blk = out_specs{k, 2};
        src_prt = out_specs{k, 3};
        op = [sub '/' op_name];
        add_block('built-in/Outport', op);
        set_param(op, 'Port', num2str(k), ...
            'Position', [460 (30 + (k-1)*25) 490 (50 + (k-1)*25)]);
        add_line(sub, [src_blk '/' num2str(src_prt)], [op_name '/1'], ...
            'autorouting', 'on');
    end
end


% =========================================================================
function set_scope_title_(scope_path, title_str)
%SET_SCOPE_TITLE_ Set scope Y-axis title via the Scope Configuration object.
% Fails quietly on older R-releases that lack the Title parameter.
    try
        cfg = get_param(scope_path, 'ScopeConfiguration');
        cfg.Title = title_str;
    catch
        try
            set_param(scope_path, 'Title', title_str);
        catch
            % Not all releases honor scope titles via set_param; skip silently.
        end
    end
end


% =========================================================================
function set_matlab_fn_script_(block_path, src)
%SET_MATLAB_FN_SCRIPT_ Install the script body of a MATLAB Function block.
    sf = sfroot;
    chart = sf.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(chart)
        all_charts = sf.find('-isa', 'Stateflow.EMChart');
        for k = 1:numel(all_charts)
            if strcmp(all_charts(k).Path, block_path)
                chart = all_charts(k);
                break;
            end
        end
    end
    if isempty(chart)
        error('build_casper_sim_phase0:NoChart', ...
            'Could not find MATLAB Function chart at %s', block_path);
    end
    chart.Script = src;
end


% =========================================================================
function build_eskf16_subsystem_(model_name)
%BUILD_ESKF16_SUBSYSTEM_ Wire a parallel 16-state ESKF onto the visual model.
%
% This function:
%   1. Drops eskf16_block.slx/eskf16_visual_block as a library-linked block.
%   2. Wires the same body-fw gyro/accel/mag the 4-state ESKF receives.
%   3. Rate-transitions all inputs to the 2 ms EKF domain.
%   4. Adds To-Workspace logging blocks for the 16-state outputs.
%   5. Adds 4 NEW comparison scopes (Altitude, VerticalVelocity, AttitudeError, Biases).
%
% Lives at the top level so the existing ESKF subsystem and validation
% logger remain untouched. Outputs of the 16-state are logged separately
% under log_est16_* names so the report generator can pull them.

    % --- 1) Drop the library-linked block --------------------------------
    sub = [model_name '/ESKF16'];
    add_block('eskf16_block/eskf16_visual_block', sub);
    set_param(sub, 'Position', [1540 720 1780 1000]);

    % --- 2) Rate-transition all inputs to the 2 ms (500 Hz) EKF rate -----
    % Gyro at 1 ms -> 2 ms
    rt_gyro = [model_name '/RT_e16_gyro'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_gyro);
    set_param(rt_gyro, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 720 1510 750]);
    add_line(model_name, 'FrameSwitch_Gyro/1', 'RT_e16_gyro/1', 'autorouting', 'on');

    % Accel at 1 ms -> 2 ms (re-use the existing FrameSwitch_Accel output;
    % rate-transition independently so we don't fight RT_eskf_accel).
    rt_accel = [model_name '/RT_e16_accel'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_accel);
    set_param(rt_accel, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 760 1510 790]);
    add_line(model_name, 'FrameSwitch_Accel/1', 'RT_e16_accel/1', 'autorouting', 'on');

    % Mag (100 Hz native) -> 2 ms (the helper handles its own sample-decimation
    % via the mag_new flag, but the chart needs a properly rate-matched input)
    rt_mag = [model_name '/RT_e16_mag'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_mag);
    set_param(rt_mag, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 800 1510 830]);
    add_line(model_name, 'FrameSwitch_Mag/1', 'RT_e16_mag/1', 'autorouting', 'on');

    rt_magdr = [model_name '/RT_e16_magdr'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_magdr);
    set_param(rt_magdr, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 840 1510 870]);
    add_line(model_name, 'MAG/4', 'RT_e16_magdr/1', 'autorouting', 'on');

    % Baro alt + data-ready are already 100 Hz; rate-transition to 2 ms.
    rt_baroalt = [model_name '/RT_e16_baroalt'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_baroalt);
    set_param(rt_baroalt, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 880 1510 910]);
    add_line(model_name, 'BARO/2', 'RT_e16_baroalt/1', 'autorouting', 'on');

    rt_baronew = [model_name '/RT_e16_baronew'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_baronew);
    set_param(rt_baronew, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 920 1510 950]);
    add_line(model_name, 'BARO/4', 'RT_e16_baronew/1', 'autorouting', 'on');

    % Init done from attitude (1 ms -> 2 ms): re-use the existing rate
    % transition rt_eskf_init if you want shared timing, but it's safer to
    % use an independent RT so library-link timing is self-contained.
    rt_initdone = [model_name '/RT_e16_init'];
    add_block('simulink/Signal Attributes/Rate Transition', rt_initdone);
    set_param(rt_initdone, 'OutPortSampleTime', '2e-3', ...
        'Position', [1460 960 1510 990]);
    add_line(model_name, 'ATTITUDE/4', 'RT_e16_init/1', 'autorouting', 'on');

    % Quat init seed: canonical EKF16 pad quat [0;0;1;0]. The EKF16 algorithm
    % from EKF16Verify.m §7 always seeds with the body-Zup -> NED pad quat
    % (body +Z up, 180 deg about Y). Threading the attitude block's q_fw
    % through here would require a frame conversion (fw -> Zup); for the
    % visual model's 1-DOF vertical truth, the canonical pad quat is byte-
    % identical to what the attitude block would produce after conversion.
    c_qinit = [model_name '/C_e16_qinit'];
    add_block('simulink/Sources/Constant', c_qinit);
    set_param(c_qinit, 'Value', '[0;0;1;0]', 'SampleTime', '-1', ...
        'Position', [1460 1000 1510 1030]);

    % --- 3) Wire ESKF16 inports (8 ports) --------------------------------
    add_line(model_name, 'RT_e16_gyro/1',    'ESKF16/1', 'autorouting', 'on');
    add_line(model_name, 'RT_e16_accel/1',   'ESKF16/2', 'autorouting', 'on');
    add_line(model_name, 'RT_e16_baroalt/1', 'ESKF16/3', 'autorouting', 'on');
    add_line(model_name, 'RT_e16_baronew/1', 'ESKF16/4', 'autorouting', 'on');
    add_line(model_name, 'RT_e16_mag/1',     'ESKF16/5', 'autorouting', 'on');
    add_line(model_name, 'RT_e16_magdr/1',   'ESKF16/6', 'autorouting', 'on');
    add_line(model_name, 'C_e16_qinit/1',    'ESKF16/7', 'autorouting', 'on');
    add_line(model_name, 'RT_e16_init/1',    'ESKF16/8', 'autorouting', 'on');

    % --- 4) To-Workspace logging for the 16-state outputs ----------------
    % These names live alongside log_est_* used by the existing validation
    % subsystem. The report generator pulls log_est16_*.
    %
    % Outport order (from build_eskf16_block):
    %   1=pos_NED, 2=vel_NED, 3=att_quat, 4=bg, 5=ba, 6=bb,
    %   7=sigma_pos, 8=sigma_vel, 9=sigma_att,
    %   10=alt_up_m, 11=vel_up_mps, 12=baro_gate_on
    logs_e16 = { ...
        1,  'log_est16_pos_NED'; ...
        2,  'log_est16_vel_NED'; ...
        3,  'log_est16_att_quat'; ...
        4,  'log_est16_bg'; ...
        5,  'log_est16_ba'; ...
        6,  'log_est16_bb'; ...
        7,  'log_est16_sigma_pos'; ...
        8,  'log_est16_sigma_vel'; ...
        9,  'log_est16_sigma_att'; ...
        10, 'log_est16_alt_up_m'; ...
        11, 'log_est16_vel_up_mps'; ...
        12, 'log_est16_baro_gate_on'};

    for k = 1:size(logs_e16, 1)
        port = logs_e16{k, 1};
        name = logs_e16{k, 2};
        tw = [model_name '/' name];
        add_block('simulink/Sinks/To Workspace', tw);
        set_param(tw, ...
            'VariableName', name, ...
            'SaveFormat',   'StructureWithTime', ...
            'SampleTime',   '-1', ...
            'Position', [1820 (730 + (k-1)*25) 1920 (750 + (k-1)*25)]);
        add_line(model_name, ['ESKF16/' num2str(port)], [name '/1'], ...
                 'autorouting', 'on');
    end

    % --- 5) Comparison scopes (4 new) ------------------------------------
    build_eskf16_compare_scopes_(model_name);
end


% =========================================================================
function build_eskf16_compare_scopes_(model_name)
%BUILD_ESKF16_COMPARE_SCOPES_ Add 4 NEW auto-opening scopes for side-by-side
% comparison of the 4-state vs 16-state EKFs.
%
% Scopes:
%   Scope_Compare_Altitude         (3 ch: truth, 4-state, 16-state) [m]
%   Scope_Compare_VerticalVelocity (3 ch: truth, 4-state, 16-state) [m/s]
%   Scope_Compare_AttitudeError    (2 ch: 4-state att err, 16-state att err) [deg]
%   Scope_Compare_Biases           (6 ch: 16-state gyro/accel biases x/y/z) [SI]

    % --- Scope_Compare_Altitude: tap GainAltFlip (truth), SelEstAlt (4-state),
    %     ESKF16/10 (16-state) ------------------------------------------------
    sc_alt = [model_name '/Scope_Compare_Altitude'];
    add_block('simulink/Sinks/Scope', sc_alt);
    set_param(sc_alt, 'NumInputPorts', '3', 'OpenAtSimulationStart', 'on', ...
        'Position', [1960 720 2020 790]);
    set_scope_title_(sc_alt, 'Altitude compare - truth vs 4-state vs 16-state [m]');
    add_line(model_name, 'GainAltFlip/1', 'Scope_Compare_Altitude/1', 'autorouting', 'on');
    add_line(model_name, 'SelEstAlt/1',   'Scope_Compare_Altitude/2', 'autorouting', 'on');
    add_line(model_name, 'ESKF16/10',     'Scope_Compare_Altitude/3', 'autorouting', 'on');

    % --- Scope_Compare_VerticalVelocity ---------------------------------------
    sc_vel = [model_name '/Scope_Compare_VerticalVelocity'];
    add_block('simulink/Sinks/Scope', sc_vel);
    set_param(sc_vel, 'NumInputPorts', '3', 'OpenAtSimulationStart', 'on', ...
        'Position', [1960 800 2020 870]);
    set_scope_title_(sc_vel, 'Vertical velocity compare - truth vs 4-state vs 16-state [m/s]');
    add_line(model_name, 'GainVelFlip/1', 'Scope_Compare_VerticalVelocity/1', 'autorouting', 'on');
    add_line(model_name, 'SelEstVel/1',   'Scope_Compare_VerticalVelocity/2', 'autorouting', 'on');
    add_line(model_name, 'ESKF16/11',     'Scope_Compare_VerticalVelocity/3', 'autorouting', 'on');

    % --- Scope_Compare_AttitudeError ------------------------------------------
    % 4-state att error: EulerConvert/9 (already exists)
    % 16-state att error: build a small chart that computes quat-error magnitude
    %   between truth_quat and the 16-state att_quat after converting to firmware
    %   frame for fair comparison. For 1-DOF vertical truth this stays near 0
    %   if the 16-state stays aligned.
    fb_e16err = [model_name '/E16_AttErrCompute'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fb_e16err);
    set_param(fb_e16err, 'Position', [1820 1050 1980 1110]);
    set_matlab_fn_script_(fb_e16err, e16_atterr_script_());

    add_line(model_name, 'BS_QuatForScope/1', 'E16_AttErrCompute/1', 'autorouting', 'on');
    add_line(model_name, 'ESKF16/3',          'E16_AttErrCompute/2', 'autorouting', 'on');

    sc_aerr = [model_name '/Scope_Compare_AttitudeError'];
    add_block('simulink/Sinks/Scope', sc_aerr);
    set_param(sc_aerr, 'NumInputPorts', '2', 'OpenAtSimulationStart', 'on', ...
        'Position', [2040 1050 2100 1110]);
    set_scope_title_(sc_aerr, 'Attitude error compare - 4-state vs 16-state [deg]');
    add_line(model_name, 'EulerConvert/9',     'Scope_Compare_AttitudeError/1', 'autorouting', 'on');
    add_line(model_name, 'E16_AttErrCompute/1','Scope_Compare_AttitudeError/2', 'autorouting', 'on');

    % --- Scope_Compare_Biases (6 channels: gyro xyz, accel xyz from 16-state) ---
    % Split bg (port 4) and ba (port 5) into scalars.
    sel_bgx = [model_name '/SelE16_BgX'];
    add_block('simulink/Signal Routing/Selector', sel_bgx);
    set_param(sel_bgx, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '1', ...
        'InputPortWidth', '3', 'Position', [1820 1130 1860 1150]);
    add_line(model_name, 'ESKF16/4', 'SelE16_BgX/1', 'autorouting', 'on');

    sel_bgy = [model_name '/SelE16_BgY'];
    add_block('simulink/Signal Routing/Selector', sel_bgy);
    set_param(sel_bgy, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '2', ...
        'InputPortWidth', '3', 'Position', [1820 1160 1860 1180]);
    add_line(model_name, 'ESKF16/4', 'SelE16_BgY/1', 'autorouting', 'on');

    sel_bgz = [model_name '/SelE16_BgZ'];
    add_block('simulink/Signal Routing/Selector', sel_bgz);
    set_param(sel_bgz, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '3', ...
        'InputPortWidth', '3', 'Position', [1820 1190 1860 1210]);
    add_line(model_name, 'ESKF16/4', 'SelE16_BgZ/1', 'autorouting', 'on');

    sel_bax = [model_name '/SelE16_BaX'];
    add_block('simulink/Signal Routing/Selector', sel_bax);
    set_param(sel_bax, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '1', ...
        'InputPortWidth', '3', 'Position', [1820 1220 1860 1240]);
    add_line(model_name, 'ESKF16/5', 'SelE16_BaX/1', 'autorouting', 'on');

    sel_bay = [model_name '/SelE16_BaY'];
    add_block('simulink/Signal Routing/Selector', sel_bay);
    set_param(sel_bay, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '2', ...
        'InputPortWidth', '3', 'Position', [1820 1250 1860 1270]);
    add_line(model_name, 'ESKF16/5', 'SelE16_BaY/1', 'autorouting', 'on');

    sel_baz = [model_name '/SelE16_BaZ'];
    add_block('simulink/Signal Routing/Selector', sel_baz);
    set_param(sel_baz, 'NumberOfDimensions', '1', ...
        'IndexOptions', 'Index vector (dialog)', 'Indices', '3', ...
        'InputPortWidth', '3', 'Position', [1820 1280 1860 1300]);
    add_line(model_name, 'ESKF16/5', 'SelE16_BaZ/1', 'autorouting', 'on');

    sc_bias = [model_name '/Scope_Compare_Biases'];
    add_block('simulink/Sinks/Scope', sc_bias);
    set_param(sc_bias, 'NumInputPorts', '6', 'OpenAtSimulationStart', 'on', ...
        'Position', [1960 1170 2020 1260]);
    set_scope_title_(sc_bias, '16-state biases - gyro xyz (rad/s) + accel xyz (m/s^2)');
    add_line(model_name, 'SelE16_BgX/1', 'Scope_Compare_Biases/1', 'autorouting', 'on');
    add_line(model_name, 'SelE16_BgY/1', 'Scope_Compare_Biases/2', 'autorouting', 'on');
    add_line(model_name, 'SelE16_BgZ/1', 'Scope_Compare_Biases/3', 'autorouting', 'on');
    add_line(model_name, 'SelE16_BaX/1', 'Scope_Compare_Biases/4', 'autorouting', 'on');
    add_line(model_name, 'SelE16_BaY/1', 'Scope_Compare_Biases/5', 'autorouting', 'on');
    add_line(model_name, 'SelE16_BaZ/1', 'Scope_Compare_Biases/6', 'autorouting', 'on');
end


% =========================================================================
function src = e16_atterr_script_()
% Compute attitude error (deg) between truth_quat (std-body -> NED-down) and
% the 16-state att_quat (body-Zup -> NED). Both quats are normalized.
% For 1-DOF vertical truth the error stays small (boresight rotation
% indistinguishable from yaw drift). Returns att_err_deg (scalar).
    src = [ ...
        'function att_err_deg = fcn(qt_std, qe_zup_ned)' newline ...
        '%#codegen' newline ...
        'qt = double(qt_std(:));' newline ...
        'qe = double(qe_zup_ned(:));' newline ...
        'nt = sqrt(qt(1)^2+qt(2)^2+qt(3)^2+qt(4)^2);' newline ...
        'ne = sqrt(qe(1)^2+qe(2)^2+qe(3)^2+qe(4)^2);' newline ...
        'if nt < 1e-12; nt = 1; end' newline ...
        'if ne < 1e-12; ne = 1; end' newline ...
        'qt = qt / nt;' newline ...
        'qe = qe / ne;' newline ...
        '% q_err = qt^-1 * qe  -> angle = 2*acos(|w|)' newline ...
        'qt_inv = [qt(1); -qt(2); -qt(3); -qt(4)];' newline ...
        'aw = qt_inv(1); ax = qt_inv(2); ay = qt_inv(3); az = qt_inv(4);' newline ...
        'bw = qe(1);     bx = qe(2);     by = qe(3);     bz = qe(4);' newline ...
        'w  = aw*bw - ax*bx - ay*by - az*bz;' newline ...
        'w_abs = min(1.0, abs(w));' newline ...
        'att_err_deg = 2 * acos(w_abs) * (180/pi);' newline ...
        'end' newline];
end
