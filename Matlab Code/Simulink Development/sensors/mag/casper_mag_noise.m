function [mag_noisy_uT, mag_raw_18bit] = casper_mag_noise(mag_in_uT, ...
                                                            dt_s, ...
                                                            tau_s, ...
                                                            sigma_white_uT, ...
                                                            scale_counts_per_gauss, ...
                                                            offset_counts, ...
                                                            seed)
%CASPER_MAG_NOISE Apply AR(1) colored noise + 18-bit quantization.
%
% Synopsis:
%   [out_uT, raw18] = casper_mag_noise(in_uT, dt_s, tau_s, sigma_white_uT, ...
%                                       scale_counts_per_gauss, offset_counts, seed)
%
% Inputs:
%   mag_in_uT              : (3x1) double, mag pre-noise reading [µT].
%   dt_s                   : scalar double, sample period [s] (e.g. 0.01).
%   tau_s                  : scalar double, AR(1) correlation time [s] (160 ms).
%   sigma_white_uT         : scalar double, white-noise sigma per axis [µT].
%   scale_counts_per_gauss : scalar double, 16384 for MMC5983MA 18-bit mode.
%   offset_counts          : scalar double, 131072 (= 2^17, zero-field midpoint).
%   seed                   : scalar double/uint32, RNG seed (Sim.Seed+3).
%
% Outputs:
%   mag_noisy_uT  : (3x1) double, post-noise post-quantization mag [µT].
%   mag_raw_18bit : (3x1) uint32, 18-bit unsigned counts as the sensor reports.
%
% Algorithm:
%   1. AR(1) update per axis (3 independent streams, persistent state):
%        alpha    = exp(-dt/tau)            (for dt=0.01, tau=0.160: 0.9394)
%        noise[k] = alpha*noise[k-1] + sqrt(1-alpha^2)*sigma*randn
%   2. Add AR(1) noise to mag_in.
%   3. Quantize to MMC5983MA 18-bit grid via the firmware's decode equation
%      inverted:
%         counts = round(mag_uT/100 * scale + offset)   (Gauss = uT/100)
%      then clip to [0, 2^18-1] (262143).
%   4. Convert back to µT for the float output:
%         mag_uT = (counts - offset) / scale * 100
%
% Persistent state: noise[k-1] per-axis + RandStream handle. The state
% resets to zero (and a freshly seeded RandStream is created) whenever the
% supplied `seed` value changes between calls. This gives the surrounding
% Simulink subsystem a way to force-reset by writing a sentinel value via
% an Initialize Function callback.
%
% Source firmware reference:
%   Software/App/drivers/mmc5983ma.c lines 184-186 (decode equation).
%   Memory note: AR(1) tau ≈ 160 ms is the firmware-measured correlation.

    %#codegen
    persistent ar1_state  % 3x1 double, previous noise sample per axis
    persistent rs         % RandStream
    persistent last_seed  % double scalar, the seed in use

    if isempty(ar1_state) || isempty(rs) || isempty(last_seed) ...
            || last_seed ~= double(seed)
        ar1_state = zeros(3, 1);
        rs        = RandStream('mt19937ar', 'Seed', double(seed));
        last_seed = double(seed);
    end

    % AR(1) coefficient and matched-variance white driver scale.
    alpha = exp(-dt_s / tau_s);
    drive = sqrt(max(1 - alpha*alpha, 0)) * sigma_white_uT;

    % Per-axis AR(1) update (independent streams, sequential draws).
    n_new = zeros(3, 1);
    n_new(1) = alpha * ar1_state(1) + drive * randn(rs);
    n_new(2) = alpha * ar1_state(2) + drive * randn(rs);
    n_new(3) = alpha * ar1_state(3) + drive * randn(rs);
    ar1_state = n_new;

    pre_quant_uT = mag_in_uT + ar1_state;

    % Quantize to MMC5983MA 18-bit grid via firmware decode equation.
    counts_float = pre_quant_uT / 100 * scale_counts_per_gauss + offset_counts;
    counts_int   = round(counts_float);
    counts_int(counts_int <      0) =      0;
    counts_int(counts_int > 262143) = 262143;

    mag_raw_18bit = uint32(counts_int);
    mag_noisy_uT  = (double(counts_int) - offset_counts) / scale_counts_per_gauss * 100;
end
