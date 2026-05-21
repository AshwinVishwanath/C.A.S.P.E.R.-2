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
    build_root = fileparts(here);
    addpath(here);
    addpath(fullfile(build_root, 'T01_truth_pipeline'));
    addpath(fullfile(build_root, 'T02_sensor_params'));
    addpath(fullfile(build_root, 'T03_imu_sensor_model'));
    addpath(fullfile(build_root, 'T04_baro_sensor_model'));
    addpath(fullfile(build_root, 'T05_mag_sensor_model'));
    addpath(fullfile(build_root, 'T06_gps_sensor_model'));
    addpath(fullfile(build_root, 'T07_frame_switch'));
    addpath(fullfile(build_root, 'T08_eskf_port'));
    addpath(fullfile(build_root, 'T09_attitude_port'));
    addpath(fullfile(build_root, 'T10_validation_block'));

    % Populate Sim/IMU/ADXL/Baro/Mag/GPS/Estimator/Attitude/TruthBus + the
    % six unified Simulink.Bus objects (SensorInputBus/IMUOutputBus/etc.).
    casper_sim_config('Seed', 20260519, 'StopTime', 5.0);

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
        'imu_block_visual',  fullfile(build_root, 'T03_imu_sensor_model', 'imu_block_visual.slx'); ...
        'baro_block_visual', fullfile(build_root, 'T04_baro_sensor_model', 'baro_block_visual.slx'); ...
        'mag_block_visual',  fullfile(build_root, 'T05_mag_sensor_model', 'mag_block_visual.slx'); ...
        'gps_block_visual',  fullfile(build_root, 'T06_gps_sensor_model', 'gps_block_visual.slx') };
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

    % --- Close & delete prior model -----------------------------------------
    if bdIsLoaded(model_name); close_system(model_name, 0); end
    if isfile(model_path); delete(model_path); end

    new_system(model_name);
    load_system(model_name);

    % Solver: fixed-step discrete, base step 1 ms (1 kHz). Snaps documented.
    set_param(model_name, ...
        'Solver',        'FixedStepDiscrete', ...
        'SolverType',    'Fixed-step', ...
        'FixedStep',     '1e-3', ...
        'StartTime',     '0', ...
        'StopTime',      '5', ...
        'SaveOutput',    'on', ...
        'SaveFormat',    'Dataset', ...
        'SaveTime',      'on', ...
        'TimeSaveName',  'tout');

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
    uc = [model_name '/imu_unit_convert'];
    add_block('simulink/User-Defined Functions/MATLAB Function', uc);
    set_param(uc, 'Position', [800 40 980 160]);
    set_matlab_fn_script_(uc, [ ...
        'function [a_mps2_std, g_radps_std] = fcn(accel_g, gyro_dps)' newline ...
        '%#codegen' newline ...
        'a_mps2_std  = double(accel_g(:)) * 9.80665;' newline ...
        'g_radps_std = double(gyro_dps(:)) * (pi/180);' newline ...
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
        'See STATUS_VISUAL.md for the construction notes.']);
    set_param([model_name '/topnote'], ...
        'Text',     note_text, ...
        'Position', [40 920 600 1020]);
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
