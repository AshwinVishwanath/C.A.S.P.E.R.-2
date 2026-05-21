function out = casper_frame_switch_inverse(in, kind)
%CASPER_FRAME_SWITCH_INVERSE Firmware -> sim frame, validation only.
%
% Synopsis:
%   out = casper_frame_switch_inverse(in, kind)
%
% Inputs:
%   in   : numeric, the firmware-frame quantity to invert.
%   kind : char, one of:
%            'nav'   - 3x1 nav vector (Zup -> NED), out = diag(1,1,-1)*in
%            'body'  - 3x1 body vector (fw -> std-body), out = R_body'*in
%            'quat'  - 4x1 Hamilton quat (fw_body->Zup -> std_body->NED),
%                      composed via DCM-recipe inverse.
%
% Outputs:
%   out  : numeric same size as in, expressed in sim-side convention.
%
% Used by T10 validation to compare firmware-side estimator output against
% sim-side truth.
%
% Round-trip identity:
%   casper_frame_switch_inverse(casper_frame_switch_nav(v), 'nav')   == v
%   casper_frame_switch_inverse(casper_frame_switch_body(v), 'body') == v
%   casper_frame_switch_inverse(casper_frame_switch_quat(q), 'quat') == q
%     (the quat identity is up to sign, modulo improper-DCM parity --
%      see casper_frame_switch_quat.m header)

    if nargin < 2
        error('casper_frame_switch_inverse:bad_args', ...
              'kind argument is required: nav | body | quat');
    end

    switch lower(char(kind))
        case 'nav'
            assert(numel(in) == 3, 'nav input must be 3-element');
            v = in(:);
            out = [ v(1); v(2); -v(3) ];

        case 'body'
            assert(numel(in) == 3, 'body input must be 3-element');
            v = in(:);
            % R_body is symmetric and involutive: R_body' = R_body, R_body*R_body=I.
            % v_std = R_body' * v_fw
            out = [  v(2); ...
                     v(1); ...
                    -v(3) ];

        case 'quat'
            assert(numel(in) == 4, 'quat input must be 4-element');
            q_fw = in(:);
            nq = norm(q_fw);
            assert(nq > 1e-12, 'q_fw has zero norm');
            q_fw = q_fw / nq;

            % Convert q_fw -> C_fw (proper)
            C_fw = local_dcm_from_quat(q_fw);

            % Apply inverse DCM recipe: C_std = T_nav_quat' * C_fw * R_body
            %   = T_nav_quat * C_fw * R_body since T_nav_quat' = T_nav_quat
            %   (the matrix is symmetric).
            % NOTE: we use T_nav_quat (proper, det = +1) here to match the
            % quaternion-side forward path, NOT the vector-side improper
            % T_nav = diag([1,1,-1]). See casper_frame_switch_quat.m header.
            R_body     = [ 0  1  0 ;
                           1  0  0 ;
                           0  0 -1 ];
            T_nav_quat = [ 0  1  0 ;
                           1  0  0 ;
                           0  0 -1 ];

            C_std = T_nav_quat * C_fw * R_body;

            % Convert back to Hamilton scalar-first quat
            out = local_quat_from_dcm(C_std);
            if out(1) < 0
                out = -out;
            end

        otherwise
            error('casper_frame_switch_inverse:bad_kind', ...
                  'unknown kind "%s"; want nav|body|quat', kind);
    end
end

% =========================================================================
function C = local_dcm_from_quat(q)
    w = q(1); x = q(2); y = q(3); z = q(4);
    C = [ 1 - 2*(y*y + z*z),  2*(x*y - z*w),       2*(x*z + y*w);       ...
          2*(x*y + z*w),      1 - 2*(x*x + z*z),   2*(y*z - x*w);       ...
          2*(x*z - y*w),      2*(y*z + x*w),       1 - 2*(x*x + y*y)   ];
end

function q = local_quat_from_dcm(C)
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
    if n > 0; q = q / n; end
end
