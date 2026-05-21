function p_meas_pa = casper_baro_noise_step(p_in_pa)
%CASPER_BARO_NOISE_STEP Single-step baro noise wrapper for Simulink.
%
% Synopsis:
%   p_meas_pa = casper_baro_noise_step(p_in_pa)
%
% Inputs:
%   p_in_pa : scalar double, Pa, pressure after the Mach-shock stage.
%
% Outputs:
%   p_meas_pa : scalar double, Pa, measured pressure after baro noise model.
%
% Purpose:
%   The Simulink NoiseModel MATLAB Function block calls this function via
%   coder.extrinsic so that Simulink does not have to size persistent
%   RandStream / state objects at compile time. All RNG state lives in
%   persistent variables HERE (not in the Stateflow chart). The chart
%   declares an explicit scalar-double output and simply assigns this
%   function's return value.
%
% Reproducibility:
%   The persistent state initializes itself from base-workspace `Sim` and
%   `Baro` on the first call. To reset between sim runs in the same
%   MATLAB session, call clear functions or use Simulink's
%   InitializeFcn to call casper_baro_noise_step_reset().
%
% Source firmware reference:
%   casper_ekf.c -- R_BARO, BARO_BI_SIGMA defines (read indirectly through
%                   casper_sensor_params.m / base WS Baro struct).
%   ms5611.c     -- 24-bit ADC resolution = 1 Pa.

    persistent state Baro_local dt_local seed_local
    if isempty(state) || ~isstruct(state)
        Baro_local = evalin('base', 'Baro');
        Sim_local  = evalin('base', 'Sim');
        dt_local   = 1.0 / Baro_local.Rate_Hz;
        seed_local = uint32(Sim_local.Seed + 2);
        state = struct('initialized', false);
    end

    [p_meas_pa, state] = casper_baro_noise(p_in_pa, dt_local, ...
        seed_local, state, Baro_local);
end
