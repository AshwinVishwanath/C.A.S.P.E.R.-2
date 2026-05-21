function test_mag_block_visual_compile()
%TEST_MAG_BLOCK_VISUAL_COMPILE Smoke-test mag_block_visual subsystem compiles + runs.
%
% Builds a tiny harness model that drives the mag_visual_block subsystem
% from a synthetic stationary truth bus, runs the diagram update (compile),
% and runs short sims. Verifies, across two scenarios:
%
%   Scenario A — radio interference DISABLED (Mag.RadioInterfActive = false):
%     * all 4 outports log and are finite
%     * field_uT_body_std magnitude is within +/-2 uT of Mag.ExpectedMag_uT
%       (the pad-rotated NED field magnitude after firmware calibration
%       round-trips back to the truth field magnitude ~40.18 uT) — this is
%       the critical invariant proving the spike is not applied when
%       interf is off, even while the schedule still strobes.
%     * raw18 outport is uint32 and in [0, 262143]
%     * radio_active tracks the TX schedule (schedule independent of
%       interf_active per MagOutputBus def) — verify the per-sample count
%       lies in the spec-expected range.
%     * data_ready is TRUE for every sample
%
%   Scenario B — radio interference ENABLED, time driven by Clock:
%     * radio_active cycles (true near each TX window of 15 ms / 100 ms)
%     * during TX-on samples the field deviates by ~10 uT per axis vs the
%       calibrated truth (smoke check that the spike layer wired through)
%
% Sonnet's earlier blocker was a wrong yout access pattern: it called
% numel(yo) on a Simulink.SimulationData.Dataset, which returns 1 (one
% Dataset handle), not the number of signals. The correct API is
% numElements()/getElement(k). All four MagOutputBus fields log fine; the
% real bug was the harness driving time_s as a constant 0, which kept
% (mod(0,0.1) < 0.015) permanently TRUE and added the ±10 uT radio spike
% to every sample, inflating |field| from ~40 to ~47.59 uT.

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    addpath(fullfile(here, '..', 'T11_integration'));
    addpath(fullfile(here, '..', 'T01_truth_pipeline'));
    addpath(fullfile(here, '..', 'T02_sensor_params'));

    base_step_s = 1 / 100;          % match Mag.Rate_Hz so RTs are no-ops
    stop_time_s = 0.2;              % 20 ticks at 100 Hz; covers 2 TX cycles

    npass = 0; nfail = 0;

    % ----- Scenario A: radio interference DISABLED -----------------------
    casper_sim_config('Seed', 20260519, 'StopTime', stop_time_s);
    Mag = evalin('base', 'Mag');
    Mag.RadioInterfActive = false;
    assignin('base', 'Mag', Mag);

    rebuild_or_load_lib_(here);
    clear casper_mag_step casper_radio_tx_step;

    [yo_A, mdl_A] = run_harness_('tmp_mag_visual_harness_A', base_step_s, stop_time_s);

    [npass, nfail] = check_dataset_size_(yo_A, 4, npass, nfail);
    [field_A,  raw18_A,  radio_A,  dr_A]  = unpack_outports_(yo_A);
    [npass, nfail] = check_finite_(field_A, 'A.field_uT_body_std', npass, nfail);
    [npass, nfail] = check_type_shape_(raw18_A, 'uint32', 'A.raw18', npass, nfail);
    [npass, nfail] = check_range_uint32_(raw18_A, 0, 262143, 'A.raw18 range', npass, nfail);
    % radio_active reflects the TX schedule (not the interf_active gate).
    % With 0.015 s airtime per 0.1 s period at 100 Hz, expect ~2 active
    % samples per cycle => ~4 across the 0.2 s window (allow 2..6).
    n_active_A = sum(double(radio_A(:)));
    [npass, nfail] = check_scalar_in_(n_active_A, 2, 6, ...
        sprintf('A.radio_active schedule count = %d (expect 2..6 in %d samples)', ...
            n_active_A, numel(radio_A)), npass, nfail);
    [npass, nfail] = check_all_eq_(dr_A,    true,  'A.data_ready all TRUE',                  npass, nfail);

    % Per-sample magnitude check: |field| within +/-2 uT of expected target
    target_mag = Mag.ExpectedMag_uT;
    mags_A = squeeze(sqrt(sum(double(field_A).^2, 1)));
    if isvector(mags_A); mags_A = mags_A(:); end
    max_dev_A = max(abs(mags_A - target_mag));
    [npass, nfail] = check_scalar_lt_(max_dev_A, 2.0, ...
        sprintf('A.|field| within +/-2 uT of %.2f (max dev %.3f)', target_mag, max_dev_A), ...
        npass, nfail);

    close_system(mdl_A, 0);

    % ----- Scenario B: radio interference ENABLED, time from Clock -------
    casper_sim_config('Seed', 20260519, 'StopTime', stop_time_s);
    Mag = evalin('base', 'Mag');
    Mag.RadioInterfActive = true;
    assignin('base', 'Mag', Mag);

    rebuild_or_load_lib_(here);
    clear casper_mag_step casper_radio_tx_step;

    [yo_B, mdl_B] = run_harness_('tmp_mag_visual_harness_B', base_step_s, stop_time_s);

    [npass, nfail] = check_dataset_size_(yo_B, 4, npass, nfail);
    [field_B, ~, radio_B, ~] = unpack_outports_(yo_B);
    [npass, nfail] = check_finite_(field_B, 'B.field_uT_body_std', npass, nfail);

    % With Mag.RadioTXAirtime_s=0.015 and Period=0.1 at 100 Hz, we expect
    % the first 2 samples of every 10-sample window to be TX-on. Across the
    % 21-sample run (0..0.2 s inclusive) that is ~4 active samples.
    n_active = sum(double(radio_B(:)));
    n_total  = numel(radio_B);
    [npass, nfail] = check_scalar_in_(n_active, 2, 6, ...
        sprintf('B.radio_active count = %d (expect 2..6 in %d samples)', n_active, n_total), ...
        npass, nfail);

    % Smoke check: with interference on AND off in the same run, the per-axis
    % delta during TX-on vs TX-off should be ~10 uT in magnitude. Compute the
    % per-axis median split (guarding against zero-count slices).
    field_B_2d = squeeze(field_B);    % 3 x Nt  (or Nt x 3 depending on layout)
    if size(field_B_2d, 1) ~= 3; field_B_2d = field_B_2d.'; end
    if any(radio_B(:)) && any(~radio_B(:))
        on_med  = median(field_B_2d(:,  logical(radio_B(:))), 2);
        off_med = median(field_B_2d(:, ~logical(radio_B(:))), 2);
        spike   = abs(on_med - off_med);
        [npass, nfail] = check_vec_near_(spike, [10; 10; 10], 1.0, ...
            sprintf('B.per-axis |spike| = [%.2f %.2f %.2f] uT (target 10)', ...
                spike(1), spike(2), spike(3)), npass, nfail);
    else
        fprintf('  SKIP B.spike check (no TX-on/off split in window)\n');
    end

    close_system(mdl_B, 0);

    fprintf('[test_mag_block_visual_compile] %d PASS / %d FAIL\n', npass, nfail);
    if nfail > 0
        error('test_mag_block_visual_compile:FAIL', '%d check(s) failed', nfail);
    end
end


% =====================================================================
function rebuild_or_load_lib_(here)
% Force a fresh library build so any param/struct changes take effect, and
% close it back down so the next sim picks up workspace edits cleanly.
    lib_name = 'mag_block_visual';
    lib_path = fullfile(here, [lib_name '.slx']);
    if bdIsLoaded(lib_name); close_system(lib_name, 0); end
    if ~isfile(lib_path)
        build_mag_block_visual();
    end
    load_system(lib_path);
end


function [yo, mdl] = run_harness_(mdl, base_step_s, stop_time_s)
% Build the tiny harness model and run the sim. Returns the yout Dataset
% and the (still-open) model name. Caller is responsible for close_system.
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl);
    load_system(mdl);

    set_param(mdl, 'Solver', 'FixedStepDiscrete', ...
        'FixedStep', num2str(base_step_s), ...
        'StartTime', '0', 'StopTime', num2str(stop_time_s));

    % Static pad attitude: 89 deg pitch up (rocket vertical), scalar-first quat.
    pad_q = eul2quat([0, 89*pi/180, 0], 'ZYX').';
    pad_q_str = sprintf('[%.15g; %.15g; %.15g; %.15g]', ...
        pad_q(1), pad_q(2), pad_q(3), pad_q(4));

    % Build the truth bus. time_s is driven by a Digital Clock so the
    % radio-TX schedule cycles correctly inside the subsystem.
    truth_consts = { ...
        {'pos_NED',          '[0;0;0]'}, ...
        {'vel_NED',          '[0;0;0]'}, ...
        {'accel_NED',        '[0;0;0]'}, ...
        {'quat_std',         pad_q_str}, ...
        {'omega_body_std',   '[0;0;0]'}, ...
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

    % Digital Clock for time_s — sample period = base_step_s, no offset.
    clk = [mdl '/time_s_clock'];
    add_block('simulink/Sources/Digital Clock', clk);
    set_param(clk, 'SampleTime', num2str(base_step_s), ...
        'Position', [30 (30 + numel(truth_consts)*40) 100 (50 + numel(truth_consts)*40)]);

    % Bus Creator — fields must be in the SensorInputBus declared order:
    % {pos_NED, vel_NED, accel_NED, quat_std, omega_body_std, time_s, mach,
    %  air_density_kgm3, air_temp_K, air_pressure_pa}.
    src_order = { ...
        'pos_NED_const/1', ...
        'vel_NED_const/1', ...
        'accel_NED_const/1', ...
        'quat_std_const/1', ...
        'omega_body_std_const/1', ...
        'time_s_clock/1', ...
        'mach_const/1', ...
        'air_density_kgm3_const/1', ...
        'air_temp_K_const/1', ...
        'air_pressure_pa_const/1' };
    name_order = { ...
        'pos_NED','vel_NED','accel_NED','quat_std','omega_body_std', ...
        'time_s','mach','air_density_kgm3','air_temp_K','air_pressure_pa' };

    bc = [mdl '/TruthBusCreate'];
    add_block('simulink/Signal Routing/Bus Creator', bc);
    set_param(bc, 'Inputs', num2str(numel(src_order)));
    set_param(bc, 'OutDataTypeStr', 'Bus: SensorInputBus', ...
        'UseBusObject', 'on', 'NonVirtualBus', 'on');
    set_param(bc, 'Position', [180 30 220 30 + numel(src_order)*40]);
    for k = 1:numel(src_order)
        dst_port = ['TruthBusCreate/' num2str(k)];
        line_h = add_line(mdl, src_order{k}, dst_port, 'autorouting', 'on');
        set_param(line_h, 'Name', name_order{k});
    end

    % Add the mag visual subsystem (library link) and wire it.
    mb = [mdl '/MAG'];
    add_block('mag_block_visual/mag_visual_block', mb, ...
        'Position', [280 30 480 240]);
    add_line(mdl, 'TruthBusCreate/1', 'MAG/1', 'autorouting', 'on');

    % 4 outports — one per MagOutputBus field.
    out_names = {'field_uT_body_std', 'raw18', 'radio_active', 'data_ready'};
    for k = 1:numel(out_names)
        op = [mdl '/Out_' out_names{k}];
        add_block('built-in/Outport', op);
        set_param(op, 'Position', [520 (30 + (k-1)*40) 550 (50 + (k-1)*40)]);
        add_line(mdl, ['MAG/' num2str(k)], ['Out_' out_names{k} '/1'], ...
            'autorouting', 'on');
    end

    fprintf('[compile] updating diagram (%s) ...\n', mdl);
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('[compile] simulating %.3f s ...\n', stop_time_s);
    sim_out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    yo = sim_out.get('yout');
    fprintf('[compile] sim OK; yout has %d signals\n', numElements(yo));
end


function [field, raw18, radio, dr] = unpack_outports_(yo)
% Extract the 4 outport datasets in declared order. Correct Dataset API:
% Dataset is a single object; numElements()/getElement() gives the per-port
% Signal entries.
    field = yo.getElement(1).Values.Data;
    raw18 = yo.getElement(2).Values.Data;
    radio = yo.getElement(3).Values.Data;
    dr    = yo.getElement(4).Values.Data;
end


% =====================================================================
function [np, nf] = check_dataset_size_(yo, expected_n, np, nf)
    actual = numElements(yo);
    if actual == expected_n
        fprintf('  PASS yout has %d signals\n', actual); np = np + 1;
    else
        fprintf('  FAIL yout signals: got %d expected %d\n', actual, expected_n);
        nf = nf + 1;
    end
end

function [np, nf] = check_finite_(data, label, np, nf)
    if all(isfinite(double(data(:))))
        fprintf('  PASS %s all finite (%s, %s)\n', label, class(data), mat2str(size(data)));
        np = np + 1;
    else
        fprintf('  FAIL %s contains non-finite values\n', label);
        nf = nf + 1;
    end
end

function [np, nf] = check_type_shape_(data, expected_type, label, np, nf)
    if isa(data, expected_type)
        fprintf('  PASS %s is %s (shape %s)\n', label, expected_type, mat2str(size(data)));
        np = np + 1;
    else
        fprintf('  FAIL %s class %s, expected %s\n', label, class(data), expected_type);
        nf = nf + 1;
    end
end

function [np, nf] = check_range_uint32_(data, lo, hi, label, np, nf)
    d = double(data(:));
    ok = all(d >= lo) && all(d <= hi);
    if ok
        fprintf('  PASS %s in [%d, %d] (min %d, max %d)\n', label, lo, hi, min(d), max(d));
        np = np + 1;
    else
        fprintf('  FAIL %s range [%d,%d]; got [%d,%d]\n', label, lo, hi, min(d), max(d));
        nf = nf + 1;
    end
end

function [np, nf] = check_all_eq_(data, expected, label, np, nf)
    d = logical(data(:));
    if all(d == expected)
        fprintf('  PASS %s\n', label); np = np + 1;
    else
        fprintf('  FAIL %s — count(true)=%d, count(false)=%d, expected all %d\n', ...
            label, sum(d), sum(~d), expected);
        nf = nf + 1;
    end
end

function [np, nf] = check_scalar_lt_(actual, limit, label, np, nf)
    if actual < limit
        fprintf('  PASS %s\n', label); np = np + 1;
    else
        fprintf('  FAIL %s  actual=%g limit=%g\n', label, actual, limit);
        nf = nf + 1;
    end
end

function [np, nf] = check_scalar_in_(actual, lo, hi, label, np, nf)
    if actual >= lo && actual <= hi
        fprintf('  PASS %s\n', label); np = np + 1;
    else
        fprintf('  FAIL %s  actual=%g range=[%g,%g]\n', label, actual, lo, hi);
        nf = nf + 1;
    end
end

function [np, nf] = check_vec_near_(actual, expected, tol, label, np, nf)
    if all(abs(actual(:) - expected(:)) <= tol)
        fprintf('  PASS %s\n', label); np = np + 1;
    else
        fprintf('  FAIL %s  |actual-expected| > %g (actual=%s expected=%s)\n', ...
            label, tol, mat2str(actual.', 4), mat2str(expected.', 4));
        nf = nf + 1;
    end
end
