function mag_NED_uT = casper_mag_field_world(pos_NED_m) %#ok<INUSD>
%CASPER_MAG_FIELD_WORLD Return Earth's magnetic field in NED at the given position.
%
% Synopsis:
%   mag_NED_uT = casper_mag_field_world(pos_NED_m)
%
% Inputs:
%   pos_NED_m : (3x1) double, NED position in metres (currently unused —
%               Phase 0 uses a position-independent constant field).
%
% Outputs:
%   mag_NED_uT : (3x1) double, Earth magnetic field in NED frame, microtesla
%                ([B_north; B_east; B_down]).
%
% Phase 0: use a constant Northern-hemisphere field with magnitude scaled
% to MAG_CAL_EXPECTED_MAG = 40.18 µT (mag_cal.h). This is the true Earth-
% field magnitude at the launch site that the firmware's calibration path
% is expected to recover from a distorted sensor reading. T05 §5.1 names
% [22.0, 0.5, 41.5] µT (|B|≈47 µT) as the direction, but the acceptance
% criterion (line 170) requires that the post-calibration body-frame
% magnitude be 40.18 ± 0.5 µT. Since the sim's inverse-cal injection round-
% trips identically (truth in -> cal out == truth in), the truth field's
% own magnitude must equal 40.18 µT for both constraints to hold. We
% therefore preserve the [22.0, 0.5, 41.5] direction and scale to 40.18 µT.
%
%   raw  : [22.0, 0.5, 41.5] => |B|≈46.97 µT
%   scale: 40.18 / 46.97 ≈ 0.85540
%   used : [22.0, 0.5, 41.5] * 0.85540 = [18.81876, 0.42770, 35.49913] µT
%
% STATUS.md documents this deviation from the literal direction values in
% T05 §5.1 to resolve the spec's internal contradiction with the §7
% acceptance criterion.
%
% Phase 1+: replace with wrldmagm() at launch-site lat/lon, fed from extended
% truth bus once RasAero is augmented with horizontal position.
%
% Source firmware reference:
%   None (sim-side; firmware does not generate the field).

    %#codegen
    % MATLAB Function block compatibility: declare output size explicitly.
    %   raw direction [22.0, 0.5, 41.5] (magnitude ≈ 46.9734 µT)
    %   target magnitude = 40.18 µT (MAG_CAL_EXPECTED_MAG, mag_cal.h)
    %   scale factor = 40.18 / sqrt(22.0^2 + 0.5^2 + 41.5^2)
    raw_dir       = [22.0; 0.5; 41.5];
    target_mag_uT = 40.18;
    scale         = target_mag_uT / sqrt(raw_dir(1)^2 + raw_dir(2)^2 + raw_dir(3)^2);
    mag_NED_uT    = raw_dir * scale;
end
