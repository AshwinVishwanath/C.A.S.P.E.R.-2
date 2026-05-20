function pressure_with_shock_pa = casper_baro_mach_shock( ...
        pressure_clean_pa, mach, vel_NED, air_density_kgm3)
%CASPER_BARO_MACH_SHOCK Phenomenological transonic static-pressure error.
%
% Synopsis:
%   p = casper_baro_mach_shock(pressure_clean_pa, mach, vel_NED, ...
%                              air_density_kgm3)
%
% Inputs:
%   pressure_clean_pa : scalar double, Pa, clean pressure from
%                       casper_baro_pressure_model().
%   mach              : scalar double, Mach number from truth bus.
%   vel_NED           : 3x1 double, m/s, velocity in NED frame (truth).
%                       Speed = norm(vel_NED).
%   air_density_kgm3  : scalar double, kg/m^3, air density from truth bus.
%
% Outputs:
%   pressure_with_shock_pa : scalar double, Pa. Equals input when |Mach| < 0.6.
%                            When Mach >= 0.6, applies a piecewise-linear
%                            error proportional to dynamic pressure q.
%
% Model (per T04 spec section 5.2, phenomenological, NOT CFD):
%   q = 0.5 * rho * V^2
%   For 0.6  <= M < 0.8: err = 0.05*q * (M-0.6)/0.2          (ramp 0% to 5% q)
%   For 0.8  <= M < 1.2: err = 0.05*q + 0.20*q * (M-0.8)/0.4 (5% to 25% q)
%   For 1.2  <= M < 2.0: err = 0.25*q - 0.15*q * (M-1.2)/0.8 (25% down to 10% q)
%   For       M >= 2.0: err = 0.10*q                         (residual)
%   For       M  < 0.6: err = 0
%   pressure_with_shock_pa = pressure_clean_pa - err
%
%   (Positive q -> lower measured static pressure; matches rocket transonic
%   baro behavior on static ports.)
%
% Source firmware reference:
%   None. Simulator-only failure-mode generator -- gives the EKF Mach gate
%   something to reject. Phase 0 acceptance only checks that the gate fires
%   in the right window, not that the error magnitude is calibrated.

    if ~isscalar(pressure_clean_pa) || ~isfinite(pressure_clean_pa)
        error('casper_baro_mach_shock:bad_pressure', ...
              'pressure_clean_pa must be a finite scalar.');
    end
    if ~isscalar(mach) || ~isfinite(mach)
        error('casper_baro_mach_shock:bad_mach', ...
              'mach must be a finite scalar.');
    end
    if numel(vel_NED) ~= 3 || any(~isfinite(vel_NED(:)))
        error('casper_baro_mach_shock:bad_vel', ...
              'vel_NED must be a finite 3-vector.');
    end
    if ~isscalar(air_density_kgm3) || ~isfinite(air_density_kgm3) || air_density_kgm3 < 0
        error('casper_baro_mach_shock:bad_rho', ...
              'air_density_kgm3 must be a finite non-negative scalar.');
    end

    M = double(mach);
    rho = double(air_density_kgm3);
    v = double(vel_NED(:));
    V = sqrt(v(1)*v(1) + v(2)*v(2) + v(3)*v(3));   % speed [m/s]

    % Dynamic pressure
    q = 0.5 * rho * V * V;                          % Pa

    if M < 0.6
        err_pa = 0.0;
    elseif M < 0.8
        % 0.6 <= M < 0.8: ramp 0 -> 5% q
        err_pa = 0.05 * q * (M - 0.6) / 0.2;
    elseif M < 1.2
        % 0.8 <= M < 1.2: 5% q -> 25% q
        err_pa = 0.05 * q + 0.20 * q * (M - 0.8) / 0.4;
    elseif M < 2.0
        % 1.2 <= M < 2.0: 25% q -> 10% q
        err_pa = 0.25 * q - 0.15 * q * (M - 1.2) / 0.8;
    else
        % M >= 2.0
        err_pa = 0.10 * q;
    end

    pressure_with_shock_pa = double(pressure_clean_pa) - err_pa;
end
