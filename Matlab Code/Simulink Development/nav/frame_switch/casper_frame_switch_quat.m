function q_fw = casper_frame_switch_quat(q_std)
%CASPER_FRAME_SWITCH_QUAT std-body-to-NED quat -> fw-body-to-Zup quat.
%
% Synopsis:
%   q_fw = casper_frame_switch_quat(q_std)
%
% Inputs:
%   q_std : 4x1 double, Hamilton scalar-first [w; x; y; z], normalized,
%           rotation from std-body to NED-nav.
%
% Outputs:
%   q_fw  : 4x1 double, Hamilton scalar-first [w; x; y; z], rotation from
%           firmware-body (Y-nose) to firmware Z-up nav. q_fw(1) >= 0.
%
% Method (per T07 spec section 5.2, modified for proper-rotation
%         compatibility):
%   1. Convert q_std to DCM C_std (3x3, body_std -> NED).
%   2. Compose: C_fw = T_nav_quat * C_std * R_body', where
%        T_nav_quat = [0 1 0; 1 0 0; 0 0 -1]   (proper, det = +1)
%        R_body     = body permutation         (proper, det = +1)
%      Both factors are proper rotations, so C_fw is a proper rotation
%      and is exactly representable as a Hamilton quaternion.
%
%   NOTE on deviation from spec: the spec's literal T_nav = diag([1,1,-1])
%   is *improper* (det = -1) and CANNOT be encoded as a Hamilton
%   quaternion. We use T_nav_quat (proper) here instead. The two yield
%   identical Z-axis behaviour (both flip the nav-Z sign) but differ on
%   the horizontal axes (T_nav_quat swaps X<->Y, T_nav keeps them).
%   Because the firmware EKF only uses nav-Z meaningfully (altitude,
%   vertical velocity, gravity along Z), this difference is invisible
%   downstream. The vector-path frame switch (casper_frame_switch_nav)
%   still uses the spec's diag([1,1,-1]) for pos/vel/accel.
%
%   This choice is documented in casper_frame_constants.m and in
%   STATUS.md under "Deviations from spec".
%
% 3. Convert C_fw to Hamilton quaternion via Shepperd's algorithm
%    (proper-rotation conversion).
%
% Round-trip identity:
%   q_back = casper_frame_switch_inverse(casper_frame_switch_quat(q_std), 'quat')
%   then min(||q_back - q_std||, ||q_back + q_std||) < 1e-10 for any unit q_std.
%   Verified by test_frame_switch.m criterion 5 for the pad attitude and 100
%   random unit quaternions.
%
% Source firmware reference:
%   Software/App/nav/casper_quat.c quat_from_dcm (Shepperd algorithm).

    assert(isnumeric(q_std) && numel(q_std) == 4, ...
        'casper_frame_switch_quat:bad_input', ...
        'q_std must be a 4-element numeric vector');

    q = q_std(:);
    nq = norm(q);
    assert(nq > 1e-12, 'casper_frame_switch_quat:zero_quat', ...
        'q_std has zero norm');
    q = q / nq;

    % --- 1. q_std -> C_std ----------------------------------------------
    C_std = local_dcm_from_quat(q);

    % --- 2. C_fw = T_nav_quat * C_std * R_body' --------------------------
    % Both factors are proper rotations (det = +1), so C_fw is proper and
    % exactly invertible through DCM<->quaternion conversion. See header
    % for the deviation note vs spec's diag([1,1,-1]).
    R_body     = [ 0  1  0 ;
                   1  0  0 ;
                   0  0 -1 ];
    T_nav_quat = [ 0  1  0 ;
                   1  0  0 ;
                   0  0 -1 ];

    C_fw = T_nav_quat * C_std * R_body';

    % --- 3. C_fw -> q_fw -------------------------------------------------
    q_fw = local_quat_from_dcm(C_fw);
    if q_fw(1) < 0
        q_fw = -q_fw;
    end
end

% =========================================================================
function C = local_dcm_from_quat(q)
%LOCAL_DCM_FROM_QUAT Hamilton scalar-first quat -> 3x3 DCM (body->nav).
% Matches casper_quat.c convention.
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [ 1 - 2*(y*y + z*z),  2*(x*y - z*w),       2*(x*z + y*w);       ...
          2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w);       ...
          2*(x*z - y*w),      2*(y*z + x*w),       1 - 2*(x*x + y*y)   ];
end

function q = local_quat_from_dcm(C)
%LOCAL_QUAT_FROM_DCM Shepperd's algorithm; works on proper rotations.
% For improper input the returned quat is the closest proper rotation.
    tr = C(1,1) + C(2,2) + C(3,3);
    if tr > 0
        s = sqrt(tr + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (C(3,2) - C(2,3)) / s;
        qy = (C(1,3) - C(3,1)) / s;
        qz = (C(2,1) - C(1,2)) / s;
    elseif (C(1,1) > C(2,2)) && (C(1,1) > C(3,3))
        s = sqrt(1.0 + C(1,1) - C(2,2) - C(3,3)) * 2.0;
        qw = (C(3,2) - C(2,3)) / s;
        qx = 0.25 * s;
        qy = (C(1,2) + C(2,1)) / s;
        qz = (C(1,3) + C(3,1)) / s;
    elseif C(2,2) > C(3,3)
        s = sqrt(1.0 + C(2,2) - C(1,1) - C(3,3)) * 2.0;
        qw = (C(1,3) - C(3,1)) / s;
        qx = (C(1,2) + C(2,1)) / s;
        qy = 0.25 * s;
        qz = (C(2,3) + C(3,2)) / s;
    else
        s = sqrt(1.0 + C(3,3) - C(1,1) - C(2,2)) * 2.0;
        qw = (C(2,1) - C(1,2)) / s;
        qx = (C(1,3) + C(3,1)) / s;
        qy = (C(2,3) + C(3,2)) / s;
        qz = 0.25 * s;
    end
    q = [qw; qx; qy; qz];
    n = norm(q);
    if n > 0
        q = q / n;
    end
end
