function [gyro_filt_radps, state_out] = casper_attitude_gyro_lpf( ...
        gyro_raw_radps, dt_s, cutoff_hz, state_in)
%CASPER_ATTITUDE_GYRO_LPF  First-order IIR LPF, per-axis.
%
%   Source: casper_attitude.c §1 (the alpha = dt/(dt + 1/(2*pi*fc)) line).
%
%   Inputs:
%     gyro_raw_radps  (3x1)   raw gyro sample, body frame, rad/s
%     dt_s            (1x1)   sample period, s (e.g. 1/833)
%     cutoff_hz       (1x1)   LPF cutoff, Hz (Attitude.GyroLpfCutoff_Hz = 50)
%     state_in        struct  fields:
%        .gyro_filt   (3x1)   previous filtered output, rad/s
%
%   Outputs:
%     gyro_filt_radps (3x1)   filtered output, rad/s
%     state_out       struct  with updated .gyro_filt
%
%   Mirrors firmware exactly (no temperature comp, no second-order shaping).

    if nargin < 4 || isempty(state_in)
        state_in = struct('gyro_filt', zeros(3,1));
    end
    assert(numel(gyro_raw_radps) == 3, 'gyro_raw_radps must be 3x1');
    gyro_raw_radps = gyro_raw_radps(:);

    rc = 1 / (2 * pi * cutoff_hz);
    alpha = dt_s / (dt_s + rc);

    prev = state_in.gyro_filt(:);
    gyro_filt_radps = alpha * gyro_raw_radps + (1 - alpha) * prev;

    state_out = state_in;
    state_out.gyro_filt = gyro_filt_radps;
end
