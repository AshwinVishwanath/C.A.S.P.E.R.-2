function vec_fw = casper_frame_switch_body(vec_std)
%CASPER_FRAME_SWITCH_BODY std-body -> firmware-body rotation.
%
% Synopsis:
%   vec_fw = casper_frame_switch_body(vec_std)
%
% Inputs:
%   vec_std : 3x1 double, body-frame vector in std-aircraft (X-fwd, Y-rt,
%             Z-down) frame. Typically IMU accel, IMU gyro, ADXL accel,
%             mag, or truth body rates.
%
% Outputs:
%   vec_fw  : 3x1 double, same vector in firmware Y-nose body frame.
%
% Operation:
%   vec_fw = R_body * vec_std,
%   R_body = [ 0  1  0 ;
%              1  0  0 ;
%              0  0 -1 ]      (det = +1, R*R' = I)
%
%   Implies:  fw_X = std_Y    (right -> starboard)
%             fw_Y = std_X    (forward -> nose; pad up axis)
%             fw_Z = -std_Z   (operator-toward is opposite std-down)
%
% Inline-literal for Simulink MATLAB Function block use.
%
% Source firmware reference:
%   On-pad accel: std [+g,0,0] -> firmware [0,+g,0]; this is the
%   load-bearing identity used in casper_attitude.c static_init.

    assert(isnumeric(vec_std) && numel(vec_std) == 3, ...
        'casper_frame_switch_body:bad_input', ...
        'vec_std must be a 3-element numeric vector');

    v = vec_std(:);
    vec_fw = [  v(2); ...
                v(1); ...
               -v(3) ];
end
