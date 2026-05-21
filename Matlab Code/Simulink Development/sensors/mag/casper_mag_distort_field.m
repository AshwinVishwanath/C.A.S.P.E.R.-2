function mag_raw_body_uT = casper_mag_distort_field(mag_clean_body_uT, ...
                                                      hard_iron_uT, ...
                                                      soft_iron, ...
                                                      axis_flip_sign)
%CASPER_MAG_DISTORT_FIELD Inject hard/soft iron + sign flip (firmware-cal inverse).
%
% Synopsis:
%   raw = casper_mag_distort_field(clean, hard_iron, soft_iron, axis_flip_sign)
%
% Inputs:
%   mag_clean_body_uT : (3x1) double, ideal mag field in body frame [µT].
%                        (after rotate-to-body, before any sensor distortions.)
%   hard_iron_uT       : (3x1) double, firmware hard-iron offset [µT]
%                        (= Mag.HardIron_uT in casper_sensor_params.m).
%   soft_iron          : (3x3) double, firmware soft-iron matrix
%                        (= Mag.SoftIron in casper_sensor_params.m).
%   axis_flip_sign     : (3x1) double, per-axis sign flip applied by firmware
%                        between sensor frame and common frame
%                        (= Mag.AxisFlipSign = [-1;-1;-1]).
%
% Outputs:
%   mag_raw_body_uT : (3x1) double, raw uncalibrated sensor reading in µT —
%                     i.e. what the MMC5983MA would emit if it measured a
%                     field of `mag_clean_body_uT` in the body frame.
%
% Why: the firmware applies, in order (mag_cal.c + flight_loop.c):
%      frame_mapped = axis_flip_sign .* raw         (line: float mag_raw[3] = {-mag.mag_ut[0], ...})
%      cal_ut       = soft_iron * (frame_mapped - hard_iron)   (mag_cal_apply)
%
%   To inject a raw reading whose firmware-side calibration recovers
%   `mag_clean_body_uT`, invert the chain:
%      frame_mapped = soft_iron \ mag_clean_body_uT  +  hard_iron
%      raw          = frame_mapped ./ axis_flip_sign
%
%   axis_flip_sign is element-wise ±1, so dividing by it is identical to
%   multiplying by it. We keep the division form for clarity.
%
% Round-trip identity: apply the firmware calibration path to this output
% and you must recover `mag_clean_body_uT` to better than 0.1 µT.
% test_mag_model.m verifies this.
%
% Source firmware reference:
%   Software/App/cal/mag_cal.c lines 178-198 (mag_hard_iron, mag_soft_iron,
%   mag_cal_apply).
%   Software/App/flight/flight_loop.c line 383 (frame mapping = ×-1).

    %#codegen
    mag_raw_body_uT = zeros(3, 1);

    % Step 1: de-apply soft iron.  intermediate = soft_iron \ mag_clean
    % Hand-coded 3x3 inverse via cofactor expansion to avoid `mldivide`
    % codegen quirks in MATLAB Function blocks.
    Si = soft_iron_inverse_3x3(soft_iron);
    intermediate = zeros(3, 1);
    intermediate(1) = Si(1,1)*mag_clean_body_uT(1) + Si(1,2)*mag_clean_body_uT(2) + Si(1,3)*mag_clean_body_uT(3);
    intermediate(2) = Si(2,1)*mag_clean_body_uT(1) + Si(2,2)*mag_clean_body_uT(2) + Si(2,3)*mag_clean_body_uT(3);
    intermediate(3) = Si(3,1)*mag_clean_body_uT(1) + Si(3,2)*mag_clean_body_uT(2) + Si(3,3)*mag_clean_body_uT(3);

    % Step 2: de-apply hard iron.  frame_mapped = intermediate + hard_iron
    frame_mapped = intermediate + hard_iron_uT;

    % Step 3: de-apply axis sign flip (raw = frame_mapped ./ axis_flip_sign).
    % For ±1 entries division equals multiplication, so use multiplication.
    mag_raw_body_uT(1) = frame_mapped(1) * axis_flip_sign(1);
    mag_raw_body_uT(2) = frame_mapped(2) * axis_flip_sign(2);
    mag_raw_body_uT(3) = frame_mapped(3) * axis_flip_sign(3);
end

% ---------------------------------------------------------------------------

function Minv = soft_iron_inverse_3x3(M)
% Cofactor-based 3x3 inverse. Determinant guard via assert.
    a = M(1,1); b = M(1,2); c = M(1,3);
    d = M(2,1); e = M(2,2); f = M(2,3);
    g = M(3,1); h = M(3,2); i = M(3,3);

    det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    assert(abs(det) > 1e-9, ...
        'casper_mag_distort_field:SingularSoftIron', ...
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
