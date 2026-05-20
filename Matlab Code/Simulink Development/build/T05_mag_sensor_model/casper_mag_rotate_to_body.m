function mag_body_uT = casper_mag_rotate_to_body(mag_NED_uT, quat_std)
%CASPER_MAG_ROTATE_TO_BODY Rotate an NED vector into standard aircraft body frame.
%
% Synopsis:
%   mag_body_uT = casper_mag_rotate_to_body(mag_NED_uT, quat_std)
%
% Inputs:
%   mag_NED_uT : (3x1) double, vector in NED, microtesla
%   quat_std   : (4x1) double, scalar-first Hamilton quaternion encoding the
%                body-to-NED rotation [w; x; y; z]
%
% Outputs:
%   mag_body_uT : (3x1) double, vector in standard aircraft body frame
%                 (X=forward, Y=right, Z=down).
%
% Algorithm:
%   1. Build R = quat_to_rotmat(quat_std). R maps body -> NED.
%   2. Body-frame projection is the transpose: mag_body = R' * mag_NED.
%
% Source firmware reference:
%   Frame conventions per ARCHITECTURE.md §3.1.
%   Hamilton rotation matrix matches casper_quat.c quat_to_rotmat.

    %#codegen
    % Explicit shape declarations for MATLAB Function block compatibility.
    mag_body_uT = zeros(3, 1);

    w = quat_std(1);
    x = quat_std(2);
    y = quat_std(3);
    z = quat_std(4);

    % Hand-coded body-to-NED rotation matrix (Hamilton scalar-first).
    % R = [ 1-2(y^2+z^2)   2(xy-wz)      2(xz+wy)   ;
    %       2(xy+wz)       1-2(x^2+z^2)  2(yz-wx)   ;
    %       2(xz-wy)       2(yz+wx)      1-2(x^2+y^2) ]
    R11 = 1 - 2*(y*y + z*z);
    R12 = 2*(x*y - w*z);
    R13 = 2*(x*z + w*y);
    R21 = 2*(x*y + w*z);
    R22 = 1 - 2*(x*x + z*z);
    R23 = 2*(y*z - w*x);
    R31 = 2*(x*z - w*y);
    R32 = 2*(y*z + w*x);
    R33 = 1 - 2*(x*x + y*y);

    % mag_body = R' * mag_NED  (NED -> body is the transpose of body -> NED)
    mag_body_uT(1) = R11*mag_NED_uT(1) + R21*mag_NED_uT(2) + R31*mag_NED_uT(3);
    mag_body_uT(2) = R12*mag_NED_uT(1) + R22*mag_NED_uT(2) + R32*mag_NED_uT(3);
    mag_body_uT(3) = R13*mag_NED_uT(1) + R23*mag_NED_uT(2) + R33*mag_NED_uT(3);
end
