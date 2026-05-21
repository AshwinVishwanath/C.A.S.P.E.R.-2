function [mag_uT_out, tx_active, axis_sign_out, rebuilt] = casper_radio_tx_step( ...
        mag_uT_in, time_s, ...
        tx_period_s, tx_airtime_s, ...
        spike_amp_uT, interf_active, ...
        seed_base, reset_flag)
%CASPER_RADIO_TX_STEP Persistent wrapper for the radio-TX interference layer.
%
% Synopsis:
%   [mag_uT_out, tx_active, axis_sign, rebuilt] = casper_radio_tx_step( ...
%       mag_uT_in, time_s, ...
%       tx_period_s, tx_airtime_s, ...
%       spike_amp_uT, interf_active, ...
%       seed_base, reset_flag)
%
% Purpose:
%   The visual-model T05 mag subsystem layers a placeholder radio-TX
%   interference effect on top of the imuSensor+quirks output. This wrapper
%   bundles the two pieces (TX schedule + ±spike interference) into a single
%   step() interface so the Simulink diagram only needs one block for the
%   environmental layer.
%
%   Semantics mirror casper_radio_tx_schedule.m and casper_mag_radio_interference.m:
%     - tx_active = (mod(time_s, tx_period_s) < tx_airtime_s)
%     - When tx_active && interf_active:
%         mag_uT_out = mag_uT_in + axis_sign_per_axis * spike_amp_uT
%       Otherwise mag_uT_out = mag_uT_in.
%     - axis_sign is drawn ONCE per sim run from {-1, +1} uniform with
%       the supplied seed (Sim.Seed + 7), then held constant.
%
% Inputs:
%   mag_uT_in       (3x1 double, uT)   mag pre-interference
%   time_s          (1x1 double, s)    current sim time
%   tx_period_s     (1x1 double, s)    TX cadence period (0.100 s default)
%   tx_airtime_s    (1x1 double, s)    TX airtime per event (0.015 s for SF7)
%   spike_amp_uT    (1x1 double, uT)   ±placeholder amplitude (10.0 uT)
%   interf_active   (1x1 logical)      master enable (Mag.RadioInterfActive)
%   seed_base       (1x1 double)       RNG seed (typically Sim.Seed + 7)
%   reset_flag      (1x1 logical)      true => re-draw axis_sign
%
% Outputs:
%   mag_uT_out      (3x1 double, uT)   mag post-interference
%   tx_active       (1x1 logical)      true while a TX is in progress
%   axis_sign_out   (3x1 double, ±1)   per-axis sign vector held constant
%                                       across the run (diagnostic / STATUS)
%   rebuilt         (1x1 logical)      true if axis_sign was re-drawn this call
%
% This is the SAFETY-LAYER PLACEHOLDER documented in ARCHITECTURE.md §5 and
% PHASE0_SPEC.md §6. It is NOT calibrated — the ±10 uT amplitude is a
% conservative Phase 0 stand-in for bench-measured radio→mag coupling.
% Phase 1 will fit amplitude + per-axis coupling matrix from real data.
%
% Source firmware reference:
%   Software/App/radio/radio_manager.c (10 Hz TX cadence)
%   Software/App/radio/radio_config.h  RADIO_TX_PERIOD_MS = 100

    %#codegen
    persistent axis_sign
    persistent last_seed

    mag_uT_out    = zeros(3, 1);
    axis_sign_out = zeros(3, 1);

    % Draw axis sign once per (re)seed.
    if isempty(axis_sign) || isempty(last_seed) ...
            || reset_flag || last_seed ~= double(seed_base)
        rs = RandStream('mt19937ar', 'Seed', double(seed_base));
        axis_sign = zeros(3, 1);
        for k = 1:3
            if rand(rs) < 0.5
                axis_sign(k) = -1;
            else
                axis_sign(k) = 1;
            end
        end
        last_seed = double(seed_base);
        rebuilt = true;
    else
        rebuilt = false;
    end
    axis_sign_out = axis_sign;

    % TX schedule (mirror casper_radio_tx_schedule.m exactly).
    if tx_period_s <= 0 || tx_airtime_s <= 0
        tx_active = false;
    else
        phase_s    = mod(time_s, tx_period_s);
        phase_snap = round(phase_s * 1e6) * 1e-6;
        tx_active  = phase_snap < tx_airtime_s;
    end

    % Apply spike if active.
    if tx_active && interf_active
        mag_uT_out(1) = mag_uT_in(1) + axis_sign(1) * spike_amp_uT;
        mag_uT_out(2) = mag_uT_in(2) + axis_sign(2) * spike_amp_uT;
        mag_uT_out(3) = mag_uT_in(3) + axis_sign(3) * spike_amp_uT;
    else
        mag_uT_out(1) = mag_uT_in(1);
        mag_uT_out(2) = mag_uT_in(2);
        mag_uT_out(3) = mag_uT_in(3);
    end
end
