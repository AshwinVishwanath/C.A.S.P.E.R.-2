function [press_pa_noisy, temp_C, rebuilt] = casper_baro_step( ...
        air_pressure_pa_truth, air_temp_K_truth, ...
        mach, vel_NED, air_density_kgm3, ...
        seed_base, reset_flag)
%CASPER_BARO_STEP Stateful wrapper for the MS5611 atmosphere + noise path.
%
% Synopsis:
%   [p_pa, t_C, rebuilt] = casper_baro_step( ...
%       air_pressure_pa_truth, air_temp_K_truth, ...
%       mach, vel_NED, air_density_kgm3, ...
%       seed_base, reset_flag)
%
% Inputs:
%   air_pressure_pa_truth (1x1 double, Pa)  truth-bus pressure (T01 ran
%                                            atmoscoesa for this altitude
%                                            already; we pass through unless
%                                            ForceAtmosCOESA is set in the
%                                            base-workspace Baro_T04 struct)
%   air_temp_K_truth      (1x1 double, K)   truth-bus temperature
%   mach                  (1x1 double)      truth-bus Mach
%   vel_NED               (3x1 double, m/s) truth-bus velocity in NED
%   air_density_kgm3      (1x1 double)      truth-bus air density
%   seed_base             (1x1 double)      RNG seed (typically Sim.Seed+2)
%   reset_flag            (1x1 logical)     true => re-seed persistent RNG
%
% Outputs:
%   press_pa_noisy (1x1 double, Pa)   pressure AFTER Mach-shock + noise
%                                     (NOT yet quantized — quantization +
%                                      altitude decode happen in
%                                      casper_baro_quirks.m)
%   temp_C         (1x1 double, degC) firmware-facing temperature
%   rebuilt        (1x1 logical)      true if persistent state was rebuilt
%                                     this call (diagnostic)
%
% Behavior:
%   - First call (or reset_flag / seed change): re-seed the persistent
%     RandStream, sample the constant per-run bias offset, zero the bias
%     drift accumulator.
%   - Subsequent calls:
%       1. Pull clean pressure from truth (either pass-through or
%          re-evaluated via atmoscoesa per ForceAtmosCOESA flag).
%       2. Apply Mach-shock layer (calls casper_baro_mach_shock).
%       3. Apply noise (bias offset + bias drift random walk + white noise).
%       4. Clip to >= 1 Pa.
%   - Quantization to 1-Pa grid and altitude decode are NOT done here;
%     they live in casper_baro_quirks.m (the PostQuirks stage).
%
% Side inputs (read from base workspace at construction time, like T03):
%   Sim   : casper_sensor_params Sim struct
%   Baro  : casper_sensor_params Baro struct (Rate_Hz, AltitudeCoeff, etc.)
%   Baro_T04 (optional) : T04-local supplemental params, with field
%       .ForceAtmosCOESA (logical, default false) — if true, recompute
%       pressure via MATLAB's atmoscoesa from the truth altitude implied by
%       the truth pressure. Default keeps the pass-through behaviour so
%       the visual block matches the legacy block bit-for-bit.
%
% This function is intended for use inside a MATLAB Function block in the
% T04 baro visual subsystem (baro_block_visual.slx). coder.extrinsic is
% used because RandStream / atmoscoesa are not codegen-friendly inline.
%
% Source firmware reference:
%   Software/App/drivers/ms5611.c     -- 1 Pa resolution, ~100 Hz tick
%   Software/App/nav/casper_ekf.c     -- R_BARO, BARO_BI_SIGMA constants
%
% See also: casper_baro_quirks, casper_imu_lsm_step (sibling pattern).

    coder.extrinsic('baro_construct_local', 'baro_step_local');

    persistent baro_state;
    persistent last_seed;

    if isempty(baro_state)
        baro_state = baro_construct_local(seed_base);
        last_seed  = seed_base;
        rebuilt    = true;
    elseif reset_flag || (seed_base ~= last_seed)
        baro_state = baro_construct_local(seed_base);
        last_seed  = seed_base;
        rebuilt    = true;
    else
        rebuilt = false;
    end

    % Reshape inputs to canonical orientation
    v_in = reshape(double(vel_NED), 3, 1);

    % Initialize outputs (so static analysis sees them defined on every path)
    press_pa_noisy = double(0);
    temp_C         = double(0);

    [press_pa_noisy, temp_C, baro_state] = baro_step_local( ...
        baro_state, ...
        double(air_pressure_pa_truth), double(air_temp_K_truth), ...
        double(mach), v_in, double(air_density_kgm3));
end


function s = baro_construct_local(seed_base)
% BARO_CONSTRUCT_LOCAL Build the persistent baro RNG + bias state.
%   Pulls every parameter from base workspace structs so the configuration
%   stays consistent with casper_sensor_params (T02 output).

    Sim   = evalin('base', 'Sim');     %#ok<NASGU> reserved (signature parity with T03)
    Baro  = evalin('base', 'Baro');

    % Optional T04-local supplemental params (analogous to T03's IMU_T03).
    if evalin('base', 'exist(''Baro_T04'',''var'')==1')
        Baro_T04 = evalin('base', 'Baro_T04');
    else
        Baro_T04 = struct('ForceAtmosCOESA', false);
    end
    if ~isfield(Baro_T04, 'ForceAtmosCOESA')
        Baro_T04.ForceAtmosCOESA = false;
    end

    % --- Persistent state struct ---
    rs = RandStream('mt19937ar', 'Seed', uint32(mod(seed_base, 2^32)));

    s = struct();
    s.stream = rs;

    % Time step between baro samples (s). Used by bias-drift random walk.
    s.dt_s = 1.0 / Baro.Rate_Hz;

    % Sea-level pressure-to-altitude linearization (~8.4 Pa/m) per spec.
    s.pa_per_m = 8.4;

    % Noise constants (mirror casper_ekf.c; documented in T04 spec §5.3).
    %   BARO_BI_SIGMA = 1e-3 m/sqrt(s); R_BARO = 0.5 m^2.
    BARO_BI_SIGMA = 1.0e-3;
    R_BARO        = 0.5;
    s.sigma_white_pa = s.pa_per_m * sqrt(R_BARO);
    s.sigma_drift_pa = s.pa_per_m * BARO_BI_SIGMA * sqrt(s.dt_s);

    % Static per-run bias offset N(0, 5 Pa). Drawn ONCE at construction.
    s.bias_offset_pa = 5.0 * randn(s.stream);

    % Random-walk bias accumulator (starts at zero).
    s.bias_drift_pa = 0.0;

    % Snapshot of the COESA-forcing flag and altitude formula constants so
    % the step function does not have to evalin('base',...) on every call.
    s.force_atmoscoesa = logical(Baro_T04.ForceAtmosCOESA);
    s.sea_level_ref_hpa = Baro.SeaLevelRef_hPa;
    s.alt_coeff_m       = Baro.AltitudeCoeff;
    s.alt_exponent      = Baro.AltitudeExponent;
end


function [p_pa, t_C, s] = baro_step_local( ...
        s, air_pressure_pa_truth, air_temp_K_truth, ...
        mach, vel_NED, air_density_kgm3)
% BARO_STEP_LOCAL Single-step truth -> pressure (post-noise, pre-quantize).
%   Stage 1: pull clean pressure (pass-through OR re-evaluate via atmoscoesa)
%   Stage 2: Mach-shock layer  (calls casper_baro_mach_shock)
%   Stage 3: noise             (bias offset + bias-drift RW + white noise)
%   Stage 4: clip to >= 1 Pa

    % --- Stage 1: clean pressure ---
    if s.force_atmoscoesa
        % Re-derive altitude from truth pressure, then atmoscoesa for the
        % canonical COESA atmosphere lookup. This is an explicit ground-
        % truth alternative path; the legacy behaviour is the pass-through.
        % T01 already runs COESA, so in practice this round-trips to within
        % numerical noise of the input pressure.
        alt_m_guess = double(s.alt_coeff_m) * ...
            (1.0 - ((air_pressure_pa_truth / 100.0) / ...
                    s.sea_level_ref_hpa)^s.alt_exponent);
        [~, ~, P_coesa, ~] = atmoscoesa(alt_m_guess, 'None');
        p_clean_pa = double(P_coesa);
    else
        p_clean_pa = double(air_pressure_pa_truth);
    end

    % --- Stage 2: Mach-shock ---
    p_with_shock_pa = casper_baro_mach_shock( ...
        p_clean_pa, mach, vel_NED, air_density_kgm3);

    % --- Stage 3: noise ---
    p = p_with_shock_pa;

    % (3.a) per-run constant bias offset (already drawn at construction)
    p = p + s.bias_offset_pa;

    % (3.b) bias drift random walk
    s.bias_drift_pa = s.bias_drift_pa + ...
        s.sigma_drift_pa * randn(s.stream);
    p = p + s.bias_drift_pa;

    % (3.c) white noise
    p = p + s.sigma_white_pa * randn(s.stream);

    % --- Stage 4: clip to >= 1 Pa ---
    if p < 1.0
        p = 1.0;
    end

    p_pa = p;

    % --- Temperature (no noise; firmware barely uses it) ---
    t_C = double(air_temp_K_truth) - 273.15;
end
