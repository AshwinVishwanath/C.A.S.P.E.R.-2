function out = casper_quat_ops()
%CASPER_QUAT_OPS  Hamilton quaternion ops (scalar-first, body-to-nav).
%
%   ops = casper_quat_ops() returns a struct of function handles that
%   mirror Software/App/nav/casper_quat.c exactly:
%       ops.mult(a,b)        - Hamilton product r = a (x) b
%       ops.conj(q)          - quaternion conjugate
%       ops.normalize(q)     - unit-norm, with q.w >= 0 sign enforcement
%       ops.to_dcm(q)        - 3x3 row-major body-to-nav rotation matrix
%       ops.dcm_to_quat(R)   - Shepperd's method, q with w >= 0
%       ops.rotate_vec(q,v)  - apply rotation q to body-frame vector v
%                               (returns R*v, where R = to_dcm(q))
%       ops.from_euler(r,p,y) ZYX -> Hamilton quaternion (matches
%                               casper_quat.c casper_quat_from_euler)
%       ops.to_euler(q)      - ZYX decomposition matching casper_quat.c
%                               casper_quat_to_euler (returns [bodyZ; bodyY; bodyX]
%                               in DEGREES, same as firmware)
%       ops.from_accel(a)    - gravity-only init (matches casper_quat_from_accel)
%       ops.angle_between(qa,qb) - Smallest angle between two attitudes (rad)
%
%   All operations work in double precision in MATLAB; the firmware is
%   single precision. Numerical results match to ~1e-7.
%
%   Source firmware: Software/App/nav/casper_quat.c (do NOT reorder ops).

    out = struct( ...
        'mult',           @local_mult, ...
        'conj',           @local_conj, ...
        'normalize',      @local_normalize, ...
        'to_dcm',         @local_to_dcm, ...
        'dcm_to_quat',    @local_dcm_to_quat, ...
        'rotate_vec',     @local_rotate_vec, ...
        'from_euler',     @local_from_euler, ...
        'to_euler',       @local_to_euler, ...
        'from_accel',     @local_from_accel, ...
        'angle_between',  @local_angle_between);
end

% ---------------------------------------------------------------------------
function r = local_mult(a, b)
    a = a(:); b = b(:);
    aw = a(1); ax = a(2); ay = a(3); az = a(4);
    bw = b(1); bx = b(2); by = b(3); bz = b(4);
    r = zeros(4,1);
    r(1) = aw*bw - ax*bx - ay*by - az*bz;
    r(2) = aw*bx + ax*bw + ay*bz - az*by;
    r(3) = aw*by - ax*bz + ay*bw + az*bx;
    r(4) = aw*bz + ax*by - ay*bx + az*bw;
end

% ---------------------------------------------------------------------------
function q = local_conj(a)
    a = a(:);
    q = [a(1); -a(2); -a(3); -a(4)];
end

% ---------------------------------------------------------------------------
function q = local_normalize(a)
    a = a(:);
    n = sqrt(a(1)*a(1) + a(2)*a(2) + a(3)*a(3) + a(4)*a(4));
    if n > 1.0e-12
        q = a / n;
    else
        q = [1; 0; 0; 0];
    end
    % Sign convention: w >= 0
    if q(1) < 0
        q = -q;
    end
end

% ---------------------------------------------------------------------------
function R = local_to_dcm(q)
    % Row-major body-to-nav matrix; identical layout to casper_quat_to_rotmat.
    q = q(:);
    w = q(1); x = q(2); y = q(3); z = q(4);
    xx = x*x; yy = y*y; zz = z*z;
    xy = x*y; xz = x*z; yz = y*z;
    wx = w*x; wy = w*y; wz = w*z;
    R = zeros(3,3);
    R(1,1) = 1 - 2*(yy + zz);
    R(1,2) = 2*(xy - wz);
    R(1,3) = 2*(xz + wy);
    R(2,1) = 2*(xy + wz);
    R(2,2) = 1 - 2*(xx + zz);
    R(2,3) = 2*(yz - wx);
    R(3,1) = 2*(xz - wy);
    R(3,2) = 2*(yz + wx);
    R(3,3) = 1 - 2*(xx + yy);
end

% ---------------------------------------------------------------------------
function q = local_dcm_to_quat(R)
    % Shepperd's method; returns Hamilton quaternion with w >= 0.
    tr = R(1,1) + R(2,2) + R(3,3);
    if tr > 0
        S = sqrt(tr + 1) * 2;
        qw = 0.25 * S;
        qx = (R(3,2) - R(2,3)) / S;
        qy = (R(1,3) - R(3,1)) / S;
        qz = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1 + R(1,1) - R(2,2) - R(3,3)) * 2;
        qw = (R(3,2) - R(2,3)) / S;
        qx = 0.25 * S;
        qy = (R(1,2) + R(2,1)) / S;
        qz = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1 + R(2,2) - R(1,1) - R(3,3)) * 2;
        qw = (R(1,3) - R(3,1)) / S;
        qx = (R(1,2) + R(2,1)) / S;
        qy = 0.25 * S;
        qz = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1 + R(3,3) - R(1,1) - R(2,2)) * 2;
        qw = (R(2,1) - R(1,2)) / S;
        qx = (R(1,3) + R(3,1)) / S;
        qy = (R(2,3) + R(3,2)) / S;
        qz = 0.25 * S;
    end
    q = local_normalize([qw; qx; qy; qz]);
end

% ---------------------------------------------------------------------------
function vn = local_rotate_vec(q, v)
    % Rotate body-frame vector v into nav: vn = R * v  (R = body-to-nav).
    R = local_to_dcm(q);
    v = v(:);
    vn = R * v;
end

% ---------------------------------------------------------------------------
function q = local_from_euler(roll_rad, pitch_rad, yaw_rad)
    % Matches casper_quat.c casper_quat_from_euler exactly (ZYX order).
    cr = cos(roll_rad  * 0.5); sr = sin(roll_rad  * 0.5);
    cp = cos(pitch_rad * 0.5); sp = sin(pitch_rad * 0.5);
    cy = cos(yaw_rad   * 0.5); sy = sin(yaw_rad   * 0.5);
    q = zeros(4,1);
    q(1) = cr*cp*cy + sr*sp*sy;   % w
    q(2) = sr*cp*cy - cr*sp*sy;   % x
    q(3) = cr*sp*cy + sr*cp*sy;   % y
    q(4) = cr*cp*sy - sr*sp*cy;   % z
    q = local_normalize(q);
end

% ---------------------------------------------------------------------------
function euler_deg = local_to_euler(q)
    % Returns [bodyZ; bodyY; bodyX] in DEGREES, exactly matching
    % casper_quat.c casper_quat_to_euler.
    q = q(:);
    w = q(1); x = q(2); y = q(3); z = q(4);

    % Body Z (yaw)
    siny = 2*(w*z + x*y);
    cosy = 1 - 2*(y*y + z*z);
    eZ = atan2(siny, cosy) * (180/pi);

    % Body Y (roll about nose), clamped
    sinp = 2*(w*y - z*x);
    sinp = max(-1, min(1, sinp));
    eY = asin(sinp) * (180/pi);

    % Body X (lateral pitch)
    sinr = 2*(w*x + y*z);
    cosr = 1 - 2*(x*x + y*y);
    eX = atan2(sinr, cosr) * (180/pi);

    euler_deg = [eZ; eY; eX];
end

% ---------------------------------------------------------------------------
function q = local_from_accel(accel_mps2)
    % Matches casper_quat.c casper_quat_from_accel (yaw = 0).
    accel_mps2 = accel_mps2(:);
    ax = accel_mps2(1); ay = accel_mps2(2); az = accel_mps2(3);
    pitch = atan2(-ax, sqrt(ay*ay + az*az));
    roll  = atan2(ay, az);
    cp = cos(pitch*0.5); sp = sin(pitch*0.5);
    cr = cos(roll*0.5);  sr = sin(roll*0.5);
    q = zeros(4,1);
    q(1) = cp*cr;
    q(2) = cp*sr;
    q(3) = sp*cr;
    q(4) = -sp*sr;
    q = local_normalize(q);
end

% ---------------------------------------------------------------------------
function ang_rad = local_angle_between(qa, qb)
    % Smallest rotation angle (in radians) from qa to qb.
    qa = local_normalize(qa);
    qb = local_normalize(qb);
    qe = local_mult(local_conj(qa), qb);
    qe = local_normalize(qe);
    w = max(-1, min(1, qe(1)));
    ang_rad = 2 * acos(abs(w));
end
