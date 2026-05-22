function vec_zup = casper_eskf16_body_fw_to_zup(vec_fw)
%CASPER_ESKF16_BODY_FW_TO_ZUP Rotate a body-fw vector into EKF16's body-Zup
% convention.
%
% Rotation: 90 deg about +X (R_x(pi/2)).
%   R = [ 1   0   0;
%         0   0  -1;
%         0   1   0 ]
%   so v_zup = [v(1); -v(3); v(2)].
%
% Frame definitions:
%   body-fw (legacy firmware Y-nose convention):
%       Pad accel = [0; +g; 0]   (Y-up = nose up against gravity)
%       Verified by run_pad_only_test ('pad accel ~ [0, +g, 0] body-FW').
%
%   body-zup (EKF16 / EKF_Symbolic_Dev.m / EKF16Verify.m):
%       Pad accel = [0; 0; +g]   (Z-up)
%       Verified by EKF16Verify line 318-336 initial state container.
%
% Sanity check:
%   v_zup = R * v_fw
%   v_fw  = [0; +g; 0]   =>   v_zup = [0; 0; +g]   OK
%
% NOTE: This used to return the identity (an earlier mis-trace of the
% visual model's accel chain concluded the visual already produced Z-up;
% diag_visual_accel.m proved that wrong -- the visual chain produces
% Y-up just like the legacy driver). Identity convention was responsible
% for ~5 percent 16-state apogee error after the Round 1 pad-window fix.
%
% Inputs:
%   vec_fw  : 3x1 (any units)
%
% Outputs:
%   vec_zup : 3x1 (same units)

    v = double(vec_fw(:));
    vec_zup = [ v(1); -v(3); v(2) ];
end
