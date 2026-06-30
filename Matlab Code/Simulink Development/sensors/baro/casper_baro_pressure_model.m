function pressure_clean_pa = casper_baro_pressure_model(air_pressure_pa_truth)
%CASPER_BARO_PRESSURE_MODEL Ideal-atmosphere pressure pass-through.
%
% Synopsis:
%   pressure_clean_pa = casper_baro_pressure_model(air_pressure_pa_truth)
%
% Inputs:
%   air_pressure_pa_truth : scalar double, Pa. Comes from the truth bus
%                           (T01 computed via COESA at the current altitude).
%
% Outputs:
%   pressure_clean_pa : scalar double, Pa. Clean ideal-atmosphere static
%                       pressure. Same value as input -- T01 already did
%                       the COESA evaluation; this block does NOT re-do it.
%
% Notes:
%   Per T04 spec section 5.1, the pressure model is a pass-through of the
%   truth bus's air_pressure_pa field. Gravity / altitude lapse is already
%   baked in. No re-computation here.
%
% Source firmware reference:
%   None. The firmware sensor reads bytes from MS5611; the simulator
%   produces the equivalent ideal-atmosphere pressure that the truth
%   trajectory implies.

    % Guard against bad inputs (hard fail per project rules).
    if ~isscalar(air_pressure_pa_truth) || ~isfinite(air_pressure_pa_truth)
        error('casper_baro_pressure_model:bad_input', ...
              'air_pressure_pa_truth must be a finite scalar; got class=%s, isscalar=%d', ...
              class(air_pressure_pa_truth), isscalar(air_pressure_pa_truth));
    end
    if air_pressure_pa_truth <= 0
        error('casper_baro_pressure_model:nonpositive', ...
              'air_pressure_pa_truth must be > 0 Pa; got %g', air_pressure_pa_truth);
    end

    pressure_clean_pa = double(air_pressure_pa_truth);
end
