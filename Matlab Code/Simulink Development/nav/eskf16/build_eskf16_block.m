function model_path = build_eskf16_block(out_dir)
%BUILD_ESKF16_BLOCK Programmatically construct a Simulink library wrapping
% the persistent-state 16-state error-state EKF helper.
%
% Synopsis:
%   build_eskf16_block()                  % writes ./eskf16_block.slx
%   build_eskf16_block(out_dir)           % writes <out_dir>/eskf16_block.slx
%
% Output:
%   eskf16_block.slx -- library with one 'eskf16_visual_block' subsystem,
%                       used as a linked block by build_casper_sim_phase0.m.
%
% The subsystem has the following ports:
%   Inputs:
%     1  gyro_body_fw_radps   (3x1)  body-fw gyro from attitude chain
%     2  accel_body_fw_mps2   (3x1)  body-fw specific force
%     3  baro_alt_m           (1x1)  altitude AGL (up positive)
%     4  baro_new             (1x1)  baro data-ready flag
%     5  mag_body_fw_uT       (3x1)  body-fw mag field
%     6  mag_new              (1x1)  mag data-ready flag
%     7  q_init_fw            (4x1)  attitude block's q_fw (one-shot seed)
%     8  init_done            (1x1)  attitude init_complete
%   Outputs:
%     1  pos_NED              (3x1, m)
%     2  vel_NED              (3x1, m/s)
%     3  att_quat             (4x1)  body-Zup -> NED Hamilton quat
%     4  bg                   (3x1, rad/s)
%     5  ba                   (3x1, m/s^2)
%     6  bb                   (1x1, m)
%     7  sigma_pos            (3x1, m)
%     8  sigma_vel            (3x1, m/s)
%     9  sigma_att            (3x1, deg)
%     10 alt_up_m             (1x1, m)        scalar alt for scopes (-pos_NED(3))
%     11 vel_up_mps           (1x1, m/s)      scalar v_up for scopes (-vel_NED(3))
%     12 baro_gate_on         (1x1, logical)
%
% Reference: Matlab Code/EKF Dev/EKF_Symbolic_Dev.m + EKF16Verify.m

    if nargin < 1 || isempty(out_dir)
        out_dir = fileparts(mfilename('fullpath'));
    end

    here = fileparts(mfilename('fullpath'));
    addpath(here);

    model_name = 'eskf16_block';
    model_path = fullfile(out_dir, [model_name '.slx']);

    if bdIsLoaded(model_name)
        bdclose(model_name);
    end
    if exist(model_path, 'file')
        delete(model_path);
    end

    new_system(model_name);
    set_param(model_name, ...
        'Solver',         'FixedStepDiscrete', ...
        'SolverType',     'Fixed-step', ...
        'FixedStep',      '2e-3', ...
        'StopTime',       '5');

    % --- Top-level Subsystem container (this is the link target) ---------
    sub = [model_name '/eskf16_visual_block'];
    add_block('built-in/Subsystem', sub);
    set_param(sub, 'Position', [60 40 480 700]);

    in_specs = { ...
        'gyro_body_fw_radps', 1; ...
        'accel_body_fw_mps2', 2; ...
        'baro_alt_m',         3; ...
        'baro_new',           4; ...
        'mag_body_fw_uT',     5; ...
        'mag_new',            6; ...
        'q_init_fw',          7; ...
        'init_done',          8};

    y0 = 40; ystep = 50;
    for k = 1:size(in_specs, 1)
        nm = in_specs{k, 1};
        add_block('built-in/Inport', [sub '/' nm]);
        set_param([sub '/' nm], ...
            'Port',     num2str(in_specs{k, 2}), ...
            'Position', [20 (y0 + (k-1)*ystep) 50 (y0 + (k-1)*ystep + 20)]);
    end

    % --- Body-frame rotators: fw -> EKF16 body-Zup ----------------------
    % MATLAB Function: swap X<->Y, keep Z (mirrors casper_eskf16_body_fw_to_zup).
    rot_gyro = [sub '/RotGyroBodyZup'];
    add_block('simulink/User-Defined Functions/MATLAB Function', rot_gyro);
    set_param(rot_gyro, 'Position', [100 30 240 90]);
    set_matlab_fn_script_(rot_gyro, body_fw_to_zup_script_('gyro'));

    rot_accel = [sub '/RotAccelBodyZup'];
    add_block('simulink/User-Defined Functions/MATLAB Function', rot_accel);
    set_param(rot_accel, 'Position', [100 100 240 160]);
    set_matlab_fn_script_(rot_accel, body_fw_to_zup_script_('accel'));

    rot_mag = [sub '/RotMagBodyZup'];
    add_block('simulink/User-Defined Functions/MATLAB Function', rot_mag);
    set_param(rot_mag, 'Position', [100 240 240 290]);
    set_matlab_fn_script_(rot_mag, body_fw_to_zup_script_('mag'));

    add_line(sub, 'gyro_body_fw_radps/1', 'RotGyroBodyZup/1',  'autorouting', 'on');
    add_line(sub, 'accel_body_fw_mps2/1', 'RotAccelBodyZup/1', 'autorouting', 'on');
    add_line(sub, 'mag_body_fw_uT/1',     'RotMagBodyZup/1',   'autorouting', 'on');

    % --- Rising-edge detector on init_done (one-shot seed flag) ----------
    % Use Unit Delay + comparison: edge = init_done AND NOT prev_init_done
    ud_init = [sub '/InitDelay'];
    add_block('simulink/Discrete/Unit Delay', ud_init);
    set_param(ud_init, 'InitialCondition', '0', 'SampleTime', '-1', ...
        'Position', [100 440 140 470]);
    add_line(sub, 'init_done/1', 'InitDelay/1', 'autorouting', 'on');

    % --- Reset flag: Constant false (the helper auto-inits on first call) ----
    c_reset = [sub '/C_Reset'];
    add_block('simulink/Sources/Constant', c_reset);
    set_param(c_reset, 'Value', 'false', 'OutDataTypeStr', 'boolean', ...
        'SampleTime', '-1', 'Position', [100 510 140 530]);

    % --- Dt constant -----------------------------------------------------
    c_dt = [sub '/C_Dt'];
    add_block('simulink/Sources/Constant', c_dt);
    set_param(c_dt, 'Value', '2e-3', 'SampleTime', '-1', ...
        'Position', [100 555 140 575]);

    % --- MATLAB Function: helper wrapper ---------------------------------
    fb = [sub '/eskf16_step'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fb);
    set_param(fb, 'Position', [300 30 540 660]);
    set_matlab_fn_script_(fb, helper_chart_script_());

    % Wire inputs to helper
    add_line(sub, 'RotGyroBodyZup/1',  'eskf16_step/1', 'autorouting', 'on');
    add_line(sub, 'RotAccelBodyZup/1', 'eskf16_step/2', 'autorouting', 'on');
    add_line(sub, 'baro_alt_m/1',      'eskf16_step/3', 'autorouting', 'on');
    add_line(sub, 'baro_new/1',        'eskf16_step/4', 'autorouting', 'on');
    add_line(sub, 'RotMagBodyZup/1',   'eskf16_step/5', 'autorouting', 'on');
    add_line(sub, 'mag_new/1',         'eskf16_step/6', 'autorouting', 'on');
    add_line(sub, 'q_init_fw/1',       'eskf16_step/7', 'autorouting', 'on');
    add_line(sub, 'init_done/1',       'eskf16_step/8', 'autorouting', 'on');
    add_line(sub, 'C_Reset/1',         'eskf16_step/9', 'autorouting', 'on');
    add_line(sub, 'C_Dt/1',            'eskf16_step/10','autorouting', 'on');

    % --- Outports --------------------------------------------------------
    out_specs = { ...
        'pos_NED',       1; ...
        'vel_NED',       2; ...
        'att_quat',      3; ...
        'bg',            4; ...
        'ba',            5; ...
        'bb',            6; ...
        'sigma_pos',     7; ...
        'sigma_vel',     8; ...
        'sigma_att',     9; ...
        'alt_up_m',     10; ...
        'vel_up_mps',   11; ...
        'baro_gate_on', 12};

    for k = 1:size(out_specs, 1)
        op = [sub '/' out_specs{k, 1}];
        add_block('built-in/Outport', op);
        set_param(op, ...
            'Port',     num2str(out_specs{k, 2}), ...
            'Position', [610 (30 + (k-1)*45) 640 (50 + (k-1)*45)]);
        add_line(sub, ['eskf16_step/' num2str(out_specs{k,2})], ...
                 [out_specs{k, 1} '/1'], 'autorouting', 'on');
    end

    try
        Simulink.BlockDiagram.arrangeSystem(model_name);
    catch
        % Best-effort cosmetic.
    end

    save_system(model_name, model_path);
    bdclose(model_name);
    fprintf('[eskf16] Saved %s\n', model_path);
end


% =========================================================================
function src = body_fw_to_zup_script_(label) %#ok<INUSD>
% Body-fw -> body-Zup identity. The visual model's FrameSwitch_Accel
% already produces "+g on Z" on the pad (see casper_eskf16_body_fw_to_zup.m
% for the trace through imu_unit_convert + casper_frame_switch_body).
% Same convention as EKF16 -> no swap needed.
    src = [ ...
        'function v_zup = fcn(v_fw)' newline ...
        '%#codegen' newline ...
        'v_zup = double(v_fw(:));' newline ...
        'end' newline];
end


% =========================================================================
function src = helper_chart_script_()
% Chart body that delegates to casper_eskf16_helper as extrinsic. Outputs
% are pre-allocated so the parser can statically infer types.
    src = [ ...
        'function [pos_NED, vel_NED, att_quat, bg, ba, bb, ' ...
        'sigma_pos, sigma_vel, sigma_att, alt_up_m, vel_up_mps, baro_gate_on] = fcn(' ...
        'gyro_zup, accel_zup, baro_alt, baro_new, mag_zup, mag_new, q_init_fw, init_done, reset_flag, dt)' newline ...
        '%#codegen' newline ...
        'coder.extrinsic(''casper_eskf16_helper'');' newline ...
        'pos_NED      = zeros(3,1);' newline ...
        'vel_NED      = zeros(3,1);' newline ...
        'att_quat     = [0;0;1;0];' newline ...
        'bg           = zeros(3,1);' newline ...
        'ba           = zeros(3,1);' newline ...
        'bb           = double(0);' newline ...
        'sigma_pos    = zeros(3,1);' newline ...
        'sigma_vel    = zeros(3,1);' newline ...
        'sigma_att    = zeros(3,1);' newline ...
        'alt_up_m     = double(0);' newline ...
        'vel_up_mps   = double(0);' newline ...
        'baro_gate_on = false;' newline ...
        'baro_innov_unused = double(0);' newline ...
        'init_armed_unused = false;' newline ...
        '[pos_NED, vel_NED, att_quat, bg, ba, bb, ' ...
        'sigma_pos, sigma_vel, sigma_att, baro_gate_on, ' ...
        'baro_innov_unused, init_armed_unused] = casper_eskf16_helper(' ...
        'double(gyro_zup), double(accel_zup), double(baro_alt), logical(baro_new), ' ...
        'double(mag_zup), logical(mag_new), double(q_init_fw), logical(init_done), ' ...
        'logical(reset_flag), double(dt));' newline ...
        'alt_up_m   = -double(pos_NED(3));' newline ...
        'vel_up_mps = -double(vel_NED(3));' newline ...
        'end' newline];
end


% =========================================================================
function set_matlab_fn_script_(block_path, src)
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
        error('build_eskf16_block:NoChart', ...
            'Could not find MATLAB Function chart at %s', block_path);
    end
    chart.Script = src;
end
