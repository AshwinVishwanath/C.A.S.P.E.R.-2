function [press_pa_out, alt_m_out] = casper_baro_quirks( ...
        press_pa_in, sea_level_ref_hpa, alt_coeff_m, alt_exponent, ...
        pressure_res_pa)
%CASPER_BARO_QUIRKS Firmware-quirk wrapper for the MS5611 PostQuirks stage.
%
% Synopsis:
%   [press_pa, alt_m] = casper_baro_quirks( ...
%                           press_pa_in, sea_level_ref_hpa, ...
%                           alt_coeff_m, alt_exponent, pressure_res_pa)
%
% Purpose:
%   The visual-model T04 baro subsystem uses MATLAB's atmoscoesa truth
%   atmosphere (or the truth bus's COESA-derived air_pressure_pa) plus a
%   Mach-shock layer + noise stage. The MS5611 sensor itself does two
%   "firmware" things on top of the pressure value before the EKF sees it:
%       1. 1-Pa quantization (24-bit ADC resolution)
%       2. The firmware's altitude decode formula
%               alt_m = AltCoeff * (1 - (P_hPa / SeaLevelRef_hPa)^AltExp)
%          which the EKF consumes (the firmware calls ms5611_get_altitude
%          in casper_ekf.c update_baro).
%
%   Both are pure / stateless / deterministic. This wrapper covers them.
%   The Mach-shock layer and noise live in casper_baro_step.m, NOT here,
%   because they have RNG state or depend on truth signals.
%
% Inputs:
%   press_pa_in        (1x1 double, Pa)  pressure after noise + clipping
%   sea_level_ref_hpa  (1x1 double, hPa) sea-level reference (1013.25)
%   alt_coeff_m        (1x1 double, m)   altitude coefficient (44307.694)
%   alt_exponent       (1x1 double)      altitude exponent     (0.190284)
%   pressure_res_pa    (1x1 double, Pa)  quantization step     (1 Pa)
%
% Outputs:
%   press_pa_out  (1x1 double, Pa)  quantized pressure (>= 1 Pa)
%   alt_m_out     (1x1 double, m)   firmware-decoded altitude
%
% This function is PURE (no persistent state, no randomness) so it is safe
% to use inside a MATLAB Function block at any sample rate. Determinism is
% guaranteed by construction.
%
% Source firmware reference:
%   Software/App/drivers/ms5611.c -- ms5611_get_altitude(); the formula
%   matches FIRMWARE_CONSTANTS.md §5.3.

    % --- Quantize to 1-Pa grid (24-bit ADC resolution) --------------------
    if pressure_res_pa > 0
        p_pa = round(press_pa_in / pressure_res_pa) * pressure_res_pa;
    else
        p_pa = press_pa_in;
    end

    % --- Clip to >= 1 Pa (no negative/zero pressure) ----------------------
    if p_pa < 1.0
        p_pa = 1.0;
    end

    % --- Firmware MS5611 altitude decode formula --------------------------
    %   alt_m = AltCoeff * (1 - (P_hPa / SeaLevelRef_hPa)^AltExp)
    %   P_hPa = p_pa / 100
    p_hpa = p_pa / 100.0;
    ratio = p_hpa / sea_level_ref_hpa;
    if ratio < 0
        ratio = 0;   % guard the fractional power for negative bases
    end
    alt_m = alt_coeff_m * (1.0 - ratio^alt_exponent);

    press_pa_out = p_pa;
    alt_m_out    = alt_m;
end
