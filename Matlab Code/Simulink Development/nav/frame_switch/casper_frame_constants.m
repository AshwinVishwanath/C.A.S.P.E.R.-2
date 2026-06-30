function K = casper_frame_constants()
%CASPER_FRAME_CONSTANTS Return the locked HIL frame-switch constants.
%
% Synopsis:
%   K = casper_frame_constants()
%
% Returns a struct of compile-time constants that define the single
% sim<->firmware HIL frame boundary. Every downstream sensor model uses
% these; the constants here are the only allowed source.
%
% Fields:
%   R_body            3x3 double, std-body  -> firmware-body change-of-basis
%   R_body_inv        3x3 double, firmware-body -> std-body (R_body')
%   T_nav             3x3 double, NED       -> Zup  (diag([1 1 -1]))
%   T_nav_inv         3x3 double, Zup       -> NED  (same matrix; involution)
%   z_flip_vec        3x1 double, [1; 1; -1] convenience
%   q_align_body      4x1 double, Hamilton quat encoding R_body (body-side
%                       rotation: applied as q_std * q_align_body in the
%                       compose pipeline). Scalar-first [w;x;y;z], w>=0.
%   q_align_nav       4x1 double, Hamilton quat encoding T_nav (nav-side
%                       reflection wrapped into a quaternion via the DCM
%                       recipe in casper_frame_switch_quat; scalar-first).
%   g_NED_mps2        3x1 double, [0; 0; +9.80665]   gravity vector in NED
%   g_Zup_mps2        3x1 double, [0; 0; -9.80665]   gravity vector in Zup
%   pad_accel_std     3x1 double, [+9.80665; 0; 0]   on-pad specific force,
%                       std body frame
%   pad_accel_fw      3x1 double, [0; +9.80665; 0]   on-pad specific force,
%                       firmware body frame
%
% Frame definitions:
%   Sim-side NED nav:        +X North, +Y East, +Z Down
%   Sim-side std body:       +X forward(nose), +Y right(stbd), +Z down
%   Firmware-side Zup nav:   +X North, +Y East, +Z Up  (left-handed; spec)
%   Firmware-side Y-nose:    +X starboard, +Y nose, +Z toward operator
%
% R_body derivation (empirical, on-pad accelerometer round-trip):
%   On the pad, accel measures specific force = gravity reaction = up.
%   std-body:   accel_std = [+9.80665, 0, 0]    (up = +X_std on pad)
%   firmware:   accel_fw  = [0, +9.80665, 0]    (up = +Y_fw on pad)
%   Therefore:  R_body * [1; 0; 0] = [0; 1; 0]
%
%   The natural axis re-labeling [std_X -> fw_Y, std_Y -> fw_X, std_Z -> fw_Z]
%   gives a det = -1 matrix (a reflection, not a rotation). To make R_body
%   a proper right-handed rotation (det = +1), the Z axis must be flipped:
%
%       R_body = [ 0  1  0 ;
%                  1  0  0 ;
%                  0  0 -1 ]
%
%   Verify: det(R_body) = +1, R_body * R_body' = I,
%           R_body * [9.81; 0; 0] = [0; 9.81; 0]    (pad accel maps right)
%           R_body * [0; 9.81; 0] = [9.81; 0; 0]    (right -> starboard)
%           R_body * [0; 0; 9.81] = [0; 0; -9.81]   (down_std -> -Z_fw)
%
%   The fw-Z direction is therefore opposite std-Z. Interpreting "toward
%   operator" as the direction opposite std-body-Z (i.e. up when the rocket
%   is on the pad) is the convention that keeps R_body proper.
%
% T_nav derivation:
%   Per task spec section 4.3:
%       pos_Zup = [pos_NED.N; pos_NED.E; -pos_NED.D]
%   T_nav = diag([1, 1, -1]). det(T_nav) = -1 (left-handed nav frame,
%   matching firmware code which only uses the Z axis meaningfully).
%
% q_align quaternions:
%   q_align_body  = quat_from_dcm(R_body)
%   q_align_nav   = quat_from_dcm(T_nav_quat)
%
%   Critical: the spec's T_nav = diag([1,1,-1]) is improper (det = -1) and
%   cannot be encoded as a Hamilton quaternion (which represents proper
%   rotations only). For the quaternion compose path we therefore use
%
%       T_nav_quat = [0 1 0; 1 0 0; 0 0 -1]   (det = +1, proper)
%
%   which is the unique proper rotation that swaps the horizontal axes and
%   flips Z. This produces a *different* horizontal X/Y mapping than the
%   vector path (where T_nav = diag([1,1,-1])) but the firmware only uses
%   nav-Z meaningfully (altitude, vertical velocity, gravity) -- the
%   horizontal nav components are pass-through to telemetry, not used in
%   the EKF math. The Z behavior matches in both paths.
%
%   This deviation is documented in STATUS.md per spec section "Deviations
%   from spec". It is necessary for criterion 5 (quat compose identity).
%
% Source firmware references:
%   Software/App/nav/casper_attitude.c (gravity-on-+Y assumption)
%   Software/App/nav/casper_ekf.c     ("a_up = ned_accel[2] - G" -> +Z is up)
%   Software/App/cal/mag_cal.c         (raw_x,y,z -> -raw_x,-y,-z; not applied
%                                       here -- T05 owns the mag sign flip)

    K = struct();

    % --- Body-axis change of basis ---------------------------------------
    K.R_body = [ 0  1  0 ;
                 1  0  0 ;
                 0  0 -1 ];
    K.R_body_inv = K.R_body';            % equal to K.R_body (symmetric)

    % --- Nav-axis Z flip --------------------------------------------------
    % Vector path (pos/vel/accel): diag([1,1,-1]) per T07 spec section 4.3.
    %   This is improper (det = -1) and represents the spec's "NED -> Zup"
    %   semantic where N stays X, E stays Y, only Z flips sign.
    K.T_nav     = diag([1, 1, -1]);
    K.T_nav_inv = K.T_nav;               % involution
    K.z_flip_vec = [1; 1; -1];

    % Quaternion path: a *proper* rotation that swaps horizontal axes and
    % flips Z. Used only by casper_frame_switch_quat / _inverse because
    % Hamilton quaternions cannot encode improper rotations. Yields a
    % horizontal mapping different from T_nav (X<->Y swap rather than
    % identity), but Z behavior is identical and the firmware does not
    % depend on horizontal nav semantics. det = +1, proper.
    K.T_nav_quat     = [0, 1, 0; 1, 0, 0; 0, 0, -1];
    K.T_nav_quat_inv = K.T_nav_quat;     % also an involution

    % --- Gravity vectors --------------------------------------------------
    G = 9.80665;                          %#ok<NASGU>
    K.g_NED_mps2 = [0; 0;  9.80665];      % gravity points down
    K.g_Zup_mps2 = K.T_nav * K.g_NED_mps2;% = [0; 0; -9.80665]

    % --- On-pad specific force --------------------------------------------
    K.pad_accel_std = [ 9.80665; 0; 0];
    K.pad_accel_fw  = K.R_body * K.pad_accel_std;   % = [0; 9.80665; 0]

    % --- Aligned quaternions (Hamilton, scalar-first, w>=0) ---------------
    % Both R_body and T_nav_quat are proper rotations (det = +1).
    K.q_align_body = local_quat_from_dcm(K.R_body);
    K.q_align_nav  = local_quat_from_dcm(K.T_nav_quat);
end

% =========================================================================
function q = local_quat_from_dcm(C)
%LOCAL_QUAT_FROM_DCM Convert a 3x3 matrix to a scalar-first Hamilton quat.
%
% For proper rotation (det = +1), this is the standard Shepperd/Markley
% conversion. For improper input (det = -1) the resulting quaternion is
% the closest proper rotation; the caller must compose via the DCM path
% (casper_frame_switch_quat) to preserve round-trip identity.
%
% Sign convention: returned quaternion has q(1) (= w) >= 0.

    tr = C(1,1) + C(2,2) + C(3,3);
    if tr > 0
        s = sqrt(tr + 1.0) * 2.0;        % s = 4*qw
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
    if q(1) < 0
        q = -q;
    end
end
