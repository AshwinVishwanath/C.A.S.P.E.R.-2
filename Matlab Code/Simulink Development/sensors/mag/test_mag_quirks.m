function test_mag_quirks()
%TEST_MAG_QUIRKS Isolation unit tests for casper_mag_quirks + casper_radio_tx_step.
%
%   Verifies:
%     C2-quirks  : hard/soft iron round-trip identity < 0.1 uT (T05 STATUS C2)
%     C5-quirks  : 18-bit raw counts in [0, 262143], on the 1-LSB grid
%     C6-quirks  : tx_active duty cycle 15 ms / 100 ms
%     C7-quirks  : ±spike applied when (tx_active && interf_active)
%     C9-quirks  : reproducibility under fixed seed (axis_sign deterministic)
%
% Tolerates one LSB of quantization residual on the round-trip (1/163.84 ≈
% 0.0061 uT) when running through casper_mag_quirks; the legacy spec
% tolerance is 0.1 uT, which is ~16x looser.

    here = fileparts(mfilename('fullpath'));         % .../sensors/mag
    simroot = fileparts(fileparts(here));            % .../Simulink Development
    addpath(here);
    addpath(fullfile(simroot, 'params'));

    % Pull parameters from params/.
    casper_sensor_params;  %#ok<NASGU> populates Mag, Sim, etc. in this workspace

    npass = 0; nfail = 0;
    fprintf('[T05-quirks] tests starting...\n');

    % --- C2: round-trip identity, no noise --------------------------------
    % Build a representative pad-frame field and run it through the quirks
    % wrapper. The output mag_uT_out must equal the input within < 0.1 uT.
    mag_clean = [22.0; 0.5; 41.5] * (40.18 / norm([22.0, 0.5, 41.5]));
    [mag_out, raw18] = casper_mag_quirks(mag_clean, ...
        Mag.HardIron_uT, Mag.SoftIron, Mag.AxisFlipSign, ...
        Mag.ScaleCountsPerGauss, Mag.OffsetCounts);
    rt_err = max(abs(mag_out - mag_clean));
    [npass, nfail] = check_scalar_lt(rt_err, 0.1, ...
        sprintf('C2 round-trip identity (max-abs-err = %.3e uT, limit 0.1)', rt_err), ...
        npass, nfail);

    % Slightly tighter: with quantization the bound should be <~ 1 LSB.
    lsb_uT = 100 / Mag.ScaleCountsPerGauss;
    [npass, nfail] = check_scalar_lt(rt_err, 2 * lsb_uT, ...
        sprintf('C2 round-trip <= 2 LSB (max-abs-err = %.3e uT, 2*LSB = %.3e)', ...
            rt_err, 2*lsb_uT), npass, nfail);

    % --- C2b: round-trip for a vector well off the calibration center ----
    mag_offset = [40; -30; 25];   % uT, intentionally far from earth pad field
    [mag_out2, ~] = casper_mag_quirks(mag_offset, ...
        Mag.HardIron_uT, Mag.SoftIron, Mag.AxisFlipSign, ...
        Mag.ScaleCountsPerGauss, Mag.OffsetCounts);
    rt_err2 = max(abs(mag_out2 - mag_offset));
    [npass, nfail] = check_scalar_lt(rt_err2, 0.1, ...
        sprintf('C2b off-center round-trip (max-abs-err = %.3e uT)', rt_err2), ...
        npass, nfail);

    % --- C5: 18-bit raw counts in [0, 262143] and on the 1-LSB grid ------
    in_range_ok  = all(raw18 >= uint32(0)) && all(raw18 <= uint32(262143));
    [npass, nfail] = check_scalar(in_range_ok, true, ...
        'C5 raw18 in [0, 262143]', npass, nfail);

    is_uint32 = isa(raw18, 'uint32') && isequal(size(raw18), [3 1]);
    [npass, nfail] = check_scalar(is_uint32, true, ...
        'C5 raw18 type/shape (3x1 uint32)', npass, nfail);

    % Decode-back sanity: decoded values must be exactly on the 1-LSB grid.
    decoded = (double(raw18) - Mag.OffsetCounts) / Mag.ScaleCountsPerGauss * 100;
    on_grid = max(abs(decoded - round(decoded / lsb_uT) * lsb_uT));
    [npass, nfail] = check_scalar_lt(on_grid, 1e-9, ...
        sprintf('C5 decoded mag on 1-LSB grid (residual %.2e uT)', on_grid), ...
        npass, nfail);

    % --- C6: tx_active duty cycle ---------------------------------------
    % Sample at 100 kHz for 1 s, count tx_active fraction.
    dt_fine = 1e-5;
    t = (0:dt_fine:1-dt_fine).';
    tx_count = 0;
    seed_radio = double(Sim.Seed + 7);
    clear casper_radio_tx_step;
    for k = 1:numel(t)
        reset_k = (k == 1);
        [~, tx_k, ~, ~] = casper_radio_tx_step(zeros(3,1), t(k), ...
            Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s, ...
            Mag.RadioSpikeAmp_uT, false, seed_radio, reset_k);
        if tx_k
            tx_count = tx_count + 1;
        end
    end
    duty = tx_count * dt_fine;
    expected_duty = (1.0 / Mag.RadioTXPeriod_s) * Mag.RadioTXAirtime_s;
    [npass, nfail] = check_scalar_lt(abs(duty - expected_duty), 5 * dt_fine, ...
        sprintf('C6 tx_active duty = %.4f s/s (target %.4f, 1s window)', duty, expected_duty), ...
        npass, nfail);

    % --- C7: ±spike applied during TX ------------------------------------
    % Run a 0.2 s window at 100 Hz and measure the per-axis spike.
    dt_mag = 1 / Mag.Rate_Hz;
    Nm = round(0.2 / dt_mag);
    clear casper_radio_tx_step;
    mag_input = mag_clean;   % constant clean field
    mag_out_arr = zeros(Nm, 3);
    tx_arr = false(Nm, 1);
    for k = 1:Nm
        reset_k = (k == 1);
        [m_out, tx_k, axis_sign_k, ~] = casper_radio_tx_step( ...
            mag_input, (k-1) * dt_mag, ...
            Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s, ...
            Mag.RadioSpikeAmp_uT, true, seed_radio, reset_k);
        mag_out_arr(k, :) = m_out.';
        tx_arr(k) = tx_k;
    end
    spike_per_axis = zeros(3, 1);
    for ax = 1:3
        on_med  = median(mag_out_arr( tx_arr, ax));
        off_med = median(mag_out_arr(~tx_arr, ax));
        spike_per_axis(ax) = on_med - off_med;
    end
    spike_amp_abs = abs(spike_per_axis);
    spike_ok = all(abs(spike_amp_abs - Mag.RadioSpikeAmp_uT) <= 0.01);
    [npass, nfail] = check_scalar(spike_ok, true, ...
        sprintf('C7 per-axis |spike|=[%.3f %.3f %.3f] vs target %.1f uT', ...
            spike_amp_abs(1), spike_amp_abs(2), spike_amp_abs(3), Mag.RadioSpikeAmp_uT), ...
        npass, nfail);

    % Axis signs should be ±1 only.
    sign_ok = all(abs(abs(axis_sign_k) - 1) < 1e-12);
    [npass, nfail] = check_scalar(sign_ok, true, ...
        sprintf('C7 axis_sign is ±1 ([%+d %+d %+d])', ...
            round(axis_sign_k(1)), round(axis_sign_k(2)), round(axis_sign_k(3))), ...
        npass, nfail);

    % --- C9: reproducibility under fixed seed ---------------------------
    clear casper_radio_tx_step;
    [~, ~, sign_a, ~] = casper_radio_tx_step(zeros(3,1), 0, ...
        Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s, ...
        Mag.RadioSpikeAmp_uT, true, seed_radio, true);
    clear casper_radio_tx_step;
    [~, ~, sign_b, ~] = casper_radio_tx_step(zeros(3,1), 0, ...
        Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s, ...
        Mag.RadioSpikeAmp_uT, true, seed_radio, true);
    repro_ok = isequal(sign_a, sign_b);
    [npass, nfail] = check_scalar(repro_ok, true, ...
        sprintf('C9 axis_sign deterministic under fixed seed ([%+d %+d %+d])', ...
            round(sign_a(1)), round(sign_a(2)), round(sign_a(3))), ...
        npass, nfail);

    % Different seed should generally give a different draw (probabilistic;
    % we just assert the function accepts a different seed without error).
    [~, ~, sign_c, ~] = casper_radio_tx_step(zeros(3,1), 0, ...
        Mag.RadioTXPeriod_s, Mag.RadioTXAirtime_s, ...
        Mag.RadioSpikeAmp_uT, true, seed_radio + 1, true);
    diff_seed_ok = isequal(size(sign_c), [3 1]) && all(abs(abs(sign_c) - 1) < 1e-12);
    [npass, nfail] = check_scalar(diff_seed_ok, true, ...
        'C9 different seed yields valid ±1 draw', npass, nfail);

    fprintf('[T05-quirks] %d PASS / %d FAIL\n', npass, nfail);
    if nfail > 0
        error('test_mag_quirks:FAIL', '%d test(s) failed', nfail);
    end
end


function [np, nf] = check_scalar(actual, expected, label, np, nf)
    if isequal(actual, expected)
        fprintf('  PASS %s\n', label);
        np = np + 1;
    else
        fprintf('  FAIL %s  actual=%s expected=%s\n', label, ...
            mat2str(actual), mat2str(expected));
        nf = nf + 1;
    end
end

function [np, nf] = check_scalar_lt(actual, limit, label, np, nf)
    if actual < limit
        fprintf('  PASS %s\n', label);
        np = np + 1;
    else
        fprintf('  FAIL %s  actual=%g limit=%g\n', label, actual, limit);
        nf = nf + 1;
    end
end
