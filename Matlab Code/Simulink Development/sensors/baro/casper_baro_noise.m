function [pressure_meas_pa, state_out] = casper_baro_noise( ...
        pressure_in_pa, dt_s, seed, state_in, Baro_params)
%CASPER_BARO_NOISE Apply baro bias offset, bias random walk, white noise,
% and 1-Pa quantization. Reproducible via seed + persistent state.
%
% Synopsis:
%   [p_meas, state_out] = casper_baro_noise(p_in_pa, dt_s, seed, state_in, Baro)
%
% Inputs:
%   pressure_in_pa : scalar double, Pa, clean (or Mach-shocked) pressure.
%   dt_s           : scalar double, s, time step between samples (e.g. 0.01).
%   seed           : scalar uint32 / double, RNG seed (typ. Sim.Seed + 2).
%   state_in       : struct with fields:
%                        .initialized (logical)
%                        .stream      (RandStream handle)
%                        .bias_offset_pa     (constant per run)
%                        .bias_drift_pa      (random walk accumulator)
%                        .pressure_to_alt_pa_per_m (~8.4 Pa/m at sea level)
%                    If state_in.initialized == false, the struct is
%                    initialized from seed and Baro on entry.
%   Baro_params    : struct (Baro from casper_sensor_params) -- only
%                    Baro.PressureRes_Pa is read here.
%
% Outputs:
%   pressure_meas_pa : scalar double, Pa, measured pressure after:
%                       1. bias_offset added
%                       2. bias_drift updated and added
%                       3. white noise N(0, sigma_white^2) added
%                       4. clip to [1, +Inf) Pa
%                       5. rounded to nearest Baro.PressureRes_Pa
%   state_out        : updated state struct (carry into next call).
%
% Noise model (T04 spec section 5.3):
%   1. Bias offset (constant per run): N(0, 5 Pa)
%   2. Bias drift (random walk per step):
%         pressure-equivalent sigma = 8.4 Pa/m * BARO_BI_SIGMA * sqrt(dt)
%      With BARO_BI_SIGMA = 1e-3 m/sqrt(s) (casper_ekf.c).
%   3. White noise per sample:
%         sigma_white = 8.4 * sqrt(R_BARO)  ~ 8.4 * sqrt(0.5) ~ 5.94 Pa
%      (R_BARO is in altitude variance (m^2); scale by ~8.4 Pa/m.)
%   4. Quantization: round to nearest Baro.PressureRes_Pa (= 1 Pa).
%
% Source firmware reference:
%   casper_ekf.c (R_BARO, BARO_BI_SIGMA defines).
%   ms5611.c     (24-bit ADC resolution = 1 Pa).

    if ~isscalar(pressure_in_pa) || ~isfinite(pressure_in_pa)
        error('casper_baro_noise:bad_input', ...
              'pressure_in_pa must be a finite scalar.');
    end
    if ~isscalar(dt_s) || dt_s <= 0
        error('casper_baro_noise:bad_dt', 'dt_s must be a positive scalar.');
    end

    state_out = state_in;

    % --- One-time init ---
    if ~isfield(state_out, 'initialized') || isempty(state_out.initialized) || ...
            ~state_out.initialized
        state_out.stream = RandStream('mt19937ar', 'Seed', uint32(seed));

        % Sea-level pressure-to-altitude scaling. dP/dh ~ -rho*g ~ -1.225*9.80665
        % ~ -12.01 Pa/m. The classic spec value of ~8.4 Pa/m corresponds to a
        % linearization at ~5 km. Use 8.4 Pa/m to match the spec's stated
        % "pressure-to-altitude scaling near sea level being ~8.4 Pa/m".
        state_out.pressure_to_alt_pa_per_m = 8.4;

        % Static per-run bias offset N(0, 5 Pa)
        state_out.bias_offset_pa = 5.0 * randn(state_out.stream);

        % Bias drift accumulator (starts at 0)
        state_out.bias_drift_pa = 0.0;

        state_out.initialized = true;
    end

    % --- Constants & per-step sigmas ---
    BARO_BI_SIGMA = 1.0e-3;     % m / sqrt(s), casper_ekf.c
    R_BARO        = 0.5;        % m^2,         casper_ekf.c

    pa_per_m = state_out.pressure_to_alt_pa_per_m;

    % White-noise sigma per sample (Pa)
    sigma_white_pa = pa_per_m * sqrt(R_BARO);            % ~5.94 Pa

    % Bias-drift step sigma (Pa)
    sigma_drift_pa = pa_per_m * BARO_BI_SIGMA * sqrt(dt_s);

    % --- Apply noise (order per spec) ---
    p = double(pressure_in_pa);

    % 1. Bias offset (constant per run)
    p = p + state_out.bias_offset_pa;

    % 2. Bias drift random walk
    state_out.bias_drift_pa = state_out.bias_drift_pa + ...
        sigma_drift_pa * randn(state_out.stream);
    p = p + state_out.bias_drift_pa;

    % 3. White noise
    p = p + sigma_white_pa * randn(state_out.stream);

    % --- Clip to >= 1 Pa (no negative/zero pressure) ---
    if p < 1.0
        p = 1.0;
    end

    % --- Quantize ---
    q_pa = Baro_params.PressureRes_Pa;
    if q_pa > 0
        p = round(p / q_pa) * q_pa;
    end

    pressure_meas_pa = p;
end
