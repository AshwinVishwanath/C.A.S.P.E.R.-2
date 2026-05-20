function mag_out_uT = casper_mag_radio_interference(mag_in_uT, ...
                                                      tx_active, ...
                                                      interf_active, ...
                                                      spike_amp_uT, ...
                                                      seed)
%CASPER_MAG_RADIO_INTERFERENCE Add ±spike_amp µT rectangular pulse during TX.
%
% Synopsis:
%   out = casper_mag_radio_interference(in, tx_active, interf_active, spike_amp_uT, seed)
%
% Inputs:
%   mag_in_uT     : (3x1) double, mag pre-interference [µT].
%   tx_active     : scalar logical, true while a radio TX is in progress.
%   interf_active : scalar logical, master enable (Mag.RadioInterfActive).
%   spike_amp_uT  : scalar double, rectangular pulse amplitude [µT] (10 placeholder).
%   seed          : scalar double, RNG seed (Sim.Seed+7) for the per-axis sign draw.
%
% Outputs:
%   mag_out_uT : (3x1) double, mag post-interference [µT].
%
% Semantics (T05 §6.2):
%   - When tx_active && interf_active: add (sign_per_axis .* spike_amp_uT)
%     to mag_in. Sign per axis is drawn ONCE per run from {-1,+1} uniform
%     with the provided seed, then held constant for the rest of the sim.
%   - When !(tx_active && interf_active): pass mag_in unchanged.
%   - Per-axis sign vector is documented in STATUS.md.
%
% Persistent state:
%   - axis_sign  (3x1): drawn lazily on first call.
%   - last_seed  (scalar): when it changes, axis_sign is re-drawn.
%
% Order matters: this function is invoked AFTER casper_mag_noise — sensor-
% intrinsic noise first, environmental interference second.
%
% Source firmware reference:
%   None directly. Placeholder for radio→mag coupling observed in bench
%   tests; calibration deferred to Phase 1 (see PHASE0_SPEC.md §6).

    %#codegen
    persistent axis_sign
    persistent last_seed

    if isempty(axis_sign) || isempty(last_seed) || last_seed ~= double(seed)
        rs        = RandStream('mt19937ar', 'Seed', double(seed));
        axis_sign = zeros(3, 1);
        for k = 1:3
            if rand(rs) < 0.5
                axis_sign(k) = -1;
            else
                axis_sign(k) = 1;
            end
        end
        last_seed = double(seed);
    end

    mag_out_uT = zeros(3, 1);
    if tx_active && interf_active
        mag_out_uT(1) = mag_in_uT(1) + axis_sign(1) * spike_amp_uT;
        mag_out_uT(2) = mag_in_uT(2) + axis_sign(2) * spike_amp_uT;
        mag_out_uT(3) = mag_in_uT(3) + axis_sign(3) * spike_amp_uT;
    else
        mag_out_uT(1) = mag_in_uT(1);
        mag_out_uT(2) = mag_in_uT(2);
        mag_out_uT(3) = mag_in_uT(3);
    end
end
