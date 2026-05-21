function test_baro_quirks()
%TEST_BARO_QUIRKS Isolation unit tests for casper_baro_quirks.
%
%   Verifies:
%     1. Sea-level pressure (101325 Pa) decodes to ~0 m altitude.
%     2. 5000 ft = 1524 m altitude decodes from the corresponding pressure.
%     3. Round-trip: feed firmware-formula pressure -> get back same altitude.
%     4. Quantization grid: input not-on-grid rounds to nearest 1-Pa step.
%     5. Quantization disabled (pressure_res_pa = 0) is a no-op.
%     6. Clipping: input < 1 Pa is clipped to 1 Pa.
%     7. Determinism: two identical calls produce identical outputs.
%     8. Pure function: no persistent state (calling reset_data is a no-op
%        because there is none — verified implicitly by repeat-call equality).
%
% Source firmware reference:
%   Software/App/drivers/ms5611.c -- ms5611_get_altitude() formula.

    fprintf('[T04-quirks] tests starting...\n');
    npass = 0; nfail = 0;

    % Firmware constants (FIRMWARE_CONSTANTS.md §5.3 + casper_sensor_params.m)
    SEA_LEVEL_HPA = 1013.25;
    ALT_COEFF_M   = 44307.694;
    ALT_EXP       = 0.190284;
    PRES_RES_PA   = 1.0;

    % --- T1: Sea-level pressure -> ~0 m altitude --------------------------
    p_sea = 101325.0;       % Pa
    [p_out, alt_out] = casper_baro_quirks( ...
        p_sea, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    % 1013.25 hPa exact -> ratio = 1.000, alt = 44307.694*(1-1) = 0
    [npass, nfail] = check(p_out,  101325.0, 'T1a sea-level p quantized', ...
                           npass, nfail, 1e-9);
    [npass, nfail] = check(alt_out, 0.0,     'T1b sea-level alt ~ 0 m',  ...
                           npass, nfail, 1e-6);

    % --- T2: Known altitude pair -- 1524 m (5000 ft, ISA) ------------------
    % ISA pressure at 1524 m is ~84307 Pa. Round-trip: compute pressure
    % via the inverse of the same formula so the test is self-consistent.
    target_alt_m = 1524.0;
    p_expected = SEA_LEVEL_HPA * 100.0 * ...
        (1.0 - target_alt_m / ALT_COEFF_M)^(1.0 / ALT_EXP);
    [~, alt_out] = casper_baro_quirks( ...
        p_expected, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [npass, nfail] = check(alt_out, target_alt_m, ...
        'T2 1524 m round-trip', npass, nfail, 0.5);   % 0.5 m tol covers
                                                       % 1-Pa quantize step

    % --- T3: Higher altitude -- 10000 m -----------------------------------
    target_alt_m = 10000.0;
    p_expected = SEA_LEVEL_HPA * 100.0 * ...
        (1.0 - target_alt_m / ALT_COEFF_M)^(1.0 / ALT_EXP);
    [~, alt_out] = casper_baro_quirks( ...
        p_expected, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [npass, nfail] = check(alt_out, target_alt_m, ...
        'T3 10000 m round-trip', npass, nfail, 0.5);

    % --- T4: Quantization rounds to nearest 1-Pa step ---------------------
    [p_out, ~] = casper_baro_quirks( ...
        101325.4, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [npass, nfail] = check(p_out, 101325.0, 'T4a quantize 101325.4 -> 101325', ...
                           npass, nfail, 1e-12);
    [p_out, ~] = casper_baro_quirks( ...
        101325.6, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [npass, nfail] = check(p_out, 101326.0, 'T4b quantize 101325.6 -> 101326', ...
                           npass, nfail, 1e-12);

    % --- T5: Custom quantization step (10 Pa) -----------------------------
    [p_out, ~] = casper_baro_quirks( ...
        101327.0, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, 10.0);
    [npass, nfail] = check(p_out, 101330.0, 'T5a quantize step=10 Pa', ...
                           npass, nfail, 1e-12);

    % --- T6: Quantization disabled (pressure_res_pa = 0) ------------------
    [p_out, ~] = casper_baro_quirks( ...
        101325.4, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, 0.0);
    [npass, nfail] = check(p_out, 101325.4, 'T6 quantize disabled = no-op', ...
                           npass, nfail, 1e-12);

    % --- T7: Clipping below 1 Pa ------------------------------------------
    [p_out, alt_out] = casper_baro_quirks( ...
        -50.0, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [npass, nfail] = check(p_out, 1.0, 'T7a clip negative -> 1 Pa', ...
                           npass, nfail, 1e-12);
    % And alt_out should be finite (not NaN/Inf) -- the 1 Pa floor decodes
    % to a very high altitude but is well-defined.
    [npass, nfail] = check_scalar(isfinite(alt_out), true, ...
        'T7b alt finite after clip', npass, nfail);

    % --- T8: Determinism (repeated calls match exactly) --------------------
    p_in_test = 95000.0;
    [p1, a1] = casper_baro_quirks( ...
        p_in_test, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [p2, a2] = casper_baro_quirks( ...
        p_in_test, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [npass, nfail] = check(p1, p2, 'T8a determinism (pressure)', ...
                           npass, nfail, 0);
    [npass, nfail] = check(a1, a2, 'T8b determinism (altitude)', ...
                           npass, nfail, 0);

    % --- T9: Monotonicity (lower pressure -> higher altitude) -------------
    [~, alt_low_alt]  = casper_baro_quirks( ...
        100000.0, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [~, alt_high_alt] = casper_baro_quirks( ...
         50000.0, SEA_LEVEL_HPA, ALT_COEFF_M, ALT_EXP, PRES_RES_PA);
    [npass, nfail] = check_scalar(alt_high_alt > alt_low_alt, true, ...
        'T9 monotonic: lower P -> higher alt', npass, nfail);

    fprintf('[T04-quirks] %d PASS / %d FAIL\n', npass, nfail);
    if nfail > 0
        error('test_baro_quirks:FAIL', '%d test(s) failed', nfail);
    end
end


function [np, nf] = check(actual, expected, label, np, nf, tol)
    if isequal(size(actual), size(expected)) && ...
            max(abs(actual(:) - expected(:))) <= tol
        fprintf('  PASS %s\n', label);
        np = np + 1;
    else
        fprintf('  FAIL %s  max|delta|=%.3e  (tol=%.3e)\n', label, ...
            max(abs(actual(:) - expected(:))), tol);
        nf = nf + 1;
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
