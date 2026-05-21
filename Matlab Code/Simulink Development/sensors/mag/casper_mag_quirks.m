function [mag_uT_out, raw18_out] = casper_mag_quirks( ...
        mag_uT_in, ...
        hard_iron_uT, soft_iron, axis_flip_sign, ...
        scale_counts_per_gauss, offset_counts)
%CASPER_MAG_QUIRKS Firmware-quirk wrapper for the MMC5983MA PostQuirks stage.
%
% Synopsis:
%   [mag_uT, raw18] = casper_mag_quirks( ...
%       mag_uT_in, hard_iron_uT, soft_iron, axis_flip_sign, ...
%       scale_counts_per_gauss, offset_counts)
%
% Purpose:
%   The visual-model T05 mag subsystem uses imuSensor('accel-gyro-mag')
%   (Sensor Fusion Tbx) for the random bits — the NED-to-body rotation
%   and white noise floor. The Sensor Fusion path produces a calibrated,
%   in-body-frame magnetometer reading. The flight firmware, however,
%   ingests raw uncalibrated sensor counts, then runs them through:
%     1) per-axis sign flip (axis_flip_sign = [-1; -1; -1])
%     2) hard iron subtraction
%     3) soft iron rotation/scaling
%   This wrapper covers the firmware-canonical steps on the sim path:
%     a) Apply the INVERSE of the firmware calibration to turn the
%        clean body-frame field into a raw uncalibrated reading
%        (so the firmware's mag_cal_apply() round-trips back to truth).
%     b) Re-apply the forward firmware calibration to produce the
%        downstream-facing calibrated body-frame field that the EKF /
%        attitude estimator will see.
%     c) Emit the 18-bit unsigned raw counts that the MMC5983MA driver
%        would actually read off the sensor (per mmc5983ma.c decode):
%             counts = round(mag_uT/100 * scale + offset)
%        clipped to [0, 2^18 - 1] = 262143.
%
% Order chosen to match T05 §5.3 and the existing legacy block:
%   inverse cal     -> raw_uncal_uT
%   18-bit encode   -> raw18 (uint32)
%   forward cal     -> mag_uT_out (calibrated, body-std frame)
%
% This wrapper is PURE (no persistent state, no randomness) so it is safe
% to use inside a MATLAB Function block at any sample rate. Round-trip
% identity is exact in the absence of quantization; with quantization the
% residual is bounded by one LSB (1/163.84 uT ≈ 0.0061 uT) — well under
% the 0.1 uT spec tolerance.
%
% Inputs:
%   mag_uT_in              (3x1 double, uT)  imuSensor mag output, body-std
%   hard_iron_uT           (3x1 double, uT)  Mag.HardIron_uT
%   soft_iron              (3x3 double)      Mag.SoftIron
%   axis_flip_sign         (3x1 double, ±1)  Mag.AxisFlipSign = [-1;-1;-1]
%   scale_counts_per_gauss (1x1 double)      Mag.ScaleCountsPerGauss = 16384
%   offset_counts          (1x1 double)      Mag.OffsetCounts = 131072
%
% Outputs:
%   mag_uT_out  (3x1 double, uT)  firmware-calibrated mag in body-std frame
%   raw18_out   (3x1 uint32)      18-bit unsigned raw counts as the sensor
%                                 register would report
%
% Source firmware reference:
%   Software/App/cal/mag_cal.c   (mag_hard_iron, mag_soft_iron, mag_cal_apply)
%   Software/App/flight/flight_loop.c line 383  (frame mapping = ×-1)
%   Software/App/drivers/mmc5983ma.c lines 184-186  (18-bit decode equation)

    %#codegen
    mag_uT_out = zeros(3, 1);
    raw18_out  = uint32(zeros(3, 1));

    % --- (a) Inverse calibration: clean body field -> raw uncal sensor field
    % Mirrors casper_mag_distort_field exactly.
    Si = soft_iron_inverse_3x3_(soft_iron);
    intermediate = zeros(3, 1);
    intermediate(1) = Si(1,1)*mag_uT_in(1) + Si(1,2)*mag_uT_in(2) + Si(1,3)*mag_uT_in(3);
    intermediate(2) = Si(2,1)*mag_uT_in(1) + Si(2,2)*mag_uT_in(2) + Si(2,3)*mag_uT_in(3);
    intermediate(3) = Si(3,1)*mag_uT_in(1) + Si(3,2)*mag_uT_in(2) + Si(3,3)*mag_uT_in(3);

    frame_mapped = intermediate + hard_iron_uT;

    raw_uncal_uT = zeros(3, 1);
    raw_uncal_uT(1) = frame_mapped(1) * axis_flip_sign(1);
    raw_uncal_uT(2) = frame_mapped(2) * axis_flip_sign(2);
    raw_uncal_uT(3) = frame_mapped(3) * axis_flip_sign(3);

    % --- (b) 18-bit encode of the raw uncal reading
    % counts = round(uT/100 * scale + offset), clipped to [0, 262143].
    counts_float = raw_uncal_uT / 100.0 * scale_counts_per_gauss + offset_counts;
    counts_int   = round(counts_float);
    for k = 1:3
        if counts_int(k) < 0
            counts_int(k) = 0;
        elseif counts_int(k) > 262143
            counts_int(k) = 262143;
        end
    end
    raw18_out = uint32(counts_int);

    % --- (c) Forward calibration on the post-quantization raw, mirroring
    %         the firmware path. We decode the raw18 back to uT, then run
    %         the mag_cal_apply forward chain. This way the calibrated
    %         output reflects the quantization grid that the firmware sees.
    raw_decoded_uT = (double(raw18_out) - offset_counts) / scale_counts_per_gauss * 100.0;

    % frame_mapped_fw = axis_flip_sign .* raw_decoded
    fm_fw = zeros(3, 1);
    fm_fw(1) = axis_flip_sign(1) * raw_decoded_uT(1);
    fm_fw(2) = axis_flip_sign(2) * raw_decoded_uT(2);
    fm_fw(3) = axis_flip_sign(3) * raw_decoded_uT(3);

    % subtract hard iron
    delta = fm_fw - hard_iron_uT;

    % cal = soft_iron * delta
    mag_uT_out(1) = soft_iron(1,1)*delta(1) + soft_iron(1,2)*delta(2) + soft_iron(1,3)*delta(3);
    mag_uT_out(2) = soft_iron(2,1)*delta(1) + soft_iron(2,2)*delta(2) + soft_iron(2,3)*delta(3);
    mag_uT_out(3) = soft_iron(3,1)*delta(1) + soft_iron(3,2)*delta(2) + soft_iron(3,3)*delta(3);
end


% ---------------------------------------------------------------------------

function Minv = soft_iron_inverse_3x3_(M)
% Cofactor-based 3x3 inverse. Determinant guard via assert.
    a = M(1,1); b = M(1,2); c = M(1,3);
    d = M(2,1); e = M(2,2); f = M(2,3);
    g = M(3,1); h = M(3,2); i = M(3,3);

    det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    assert(abs(det) > 1e-9, ...
        'casper_mag_quirks:SingularSoftIron', ...
        'soft_iron matrix is singular (det = %g).', det);

    inv_det = 1 / det;

    Minv = zeros(3, 3);
    Minv(1,1) =  (e*i - f*h) * inv_det;
    Minv(1,2) = -(b*i - c*h) * inv_det;
    Minv(1,3) =  (b*f - c*e) * inv_det;
    Minv(2,1) = -(d*i - f*g) * inv_det;
    Minv(2,2) =  (a*i - c*g) * inv_det;
    Minv(2,3) = -(a*f - c*d) * inv_det;
    Minv(3,1) =  (d*h - e*g) * inv_det;
    Minv(3,2) = -(a*h - b*g) * inv_det;
    Minv(3,3) =  (a*e - b*d) * inv_det;
end
