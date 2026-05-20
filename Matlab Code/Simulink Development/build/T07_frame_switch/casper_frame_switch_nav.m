function vec_Zup = casper_frame_switch_nav(vec_NED)
%CASPER_FRAME_SWITCH_NAV NED -> Zup nav-frame scalar Z flip.
%
% Synopsis:
%   vec_Zup = casper_frame_switch_nav(vec_NED)
%
% Inputs:
%   vec_NED : 3x1 double, any nav-frame vector (position, velocity,
%             acceleration, specific force, etc.) in NED.
%
% Outputs:
%   vec_Zup : 3x1 double, same vector expressed in firmware Z-up nav frame.
%
% Operation:
%   vec_Zup = T_nav * vec_NED, where T_nav = diag([1 1 -1]).
%   N -> X (unchanged), E -> Y (unchanged), D -> -Z (sign flip).
%
% Notes:
%   - Inline literal so this is Simulink-MATLAB-Function block ready
%     (no struct lookup, no persistent state, deterministic).
%   - The inverse is casper_frame_switch_inverse(..., 'nav').
%
% Source firmware reference:
%   casper_ekf.c "a_up = ned_accel[2] - G_ACCEL - bias"
%   confirms +Z in firmware nav = up.

    assert(isnumeric(vec_NED) && numel(vec_NED) == 3, ...
        'casper_frame_switch_nav:bad_input', ...
        'vec_NED must be a 3-element numeric vector');

    v = vec_NED(:);                       % force column
    vec_Zup = [ v(1); v(2); -v(3) ];
end
