function [F_func, Q_func, H_MAG_func] = casper_eskf16_load_symbolic(varargin)
%CASPER_ESKF16_LOAD_SYMBOLIC One-shot loader for the 16-state EKF symbolic
% matrices. Compiles MATLAB function handles for F, Q, and H_MAG and caches
% them in the base workspace so the per-tick helper does not pay the
% symbolic->numeric compile cost on every step.
%
% Synopsis:
%   casper_eskf16_load_symbolic()                    % default mat path
%   casper_eskf16_load_symbolic('Force', true)       % recompile even if cached
%   [F_func, Q_func, H_MAG_func] = casper_eskf16_load_symbolic(...)
%
% The cached handles live in the base workspace as:
%   ESKF16_F_func    : @(q0,q1,q2,q3, wx,wy,wz, fx,fy,fz, dt) -> 16x16 F
%   ESKF16_Q_func    : @(q0,q1,q2,q3, dt, sig_gx,sig_gy,sig_gz, sig_ax,sig_ay,sig_az,
%                       sig_gbx,sig_gby,sig_gbz, sig_abx,sig_aby,sig_abz, sig_bb) -> 16x16 Q
%   ESKF16_H_MAG_func: @(q0,q1,q2,q3, magN,magE,magD) -> 3x16 H_MAG
%   ESKF16_loaded    : logical sentinel
%
% Reference: Matlab Code/EKF Dev/EKF_Symbolic_Dev.m (matrices)
%            Matlab Code/EKF Dev/EKF16Verify.m §6      (matlabFunction recipe)

    p = inputParser();
    addParameter(p, 'Force',   false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'MatPath', '',    @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});

    % Fast path: already cached.
    cached = false;
    try
        cached = logical(evalin('base', 'exist(''ESKF16_loaded'',''var'')'));
        if cached
            cached = logical(evalin('base', 'ESKF16_loaded'));
        end
    catch
        cached = false;
    end

    if cached && ~p.Results.Force
        F_func     = evalin('base', 'ESKF16_F_func');
        Q_func     = evalin('base', 'ESKF16_Q_func');
        H_MAG_func = evalin('base', 'ESKF16_H_MAG_func');
        return;
    end

    % Resolve mat path.
    if isempty(p.Results.MatPath)
        mat_path = locate_symbolic_mat_();
    else
        mat_path = char(p.Results.MatPath);
    end
    if ~isfile(mat_path)
        error('casper_eskf16_load_symbolic:NoMat', ...
            'Symbolic .mat not found at %s. Run EKF_Symbolic_Dev.m first.', mat_path);
    end

    fprintf('[eskf16] Loading symbolic workspace: %s\n', mat_path);
    S = load(mat_path);

    F     = S.F;       %#ok<NASGU>
    Q     = S.Q;       %#ok<NASGU>
    H_MAG = S.H_MAG;   %#ok<NASGU>

    % --- Compile F handle ---
    syms q0 q1 q2 q3 wx wy wz fx fy fz dt 'real'
    F_func = matlabFunction(S.F, 'Vars', {q0,q1,q2,q3, wx,wy,wz, fx,fy,fz, dt});

    % --- Compile Q handle ---
    syms sig_gx sig_gy sig_gz sig_ax sig_ay sig_az 'real'
    syms sig_gbx sig_gby sig_gbz sig_abx sig_aby sig_abz sig_bb 'real'
    Q_func = matlabFunction(S.Q, 'Vars', ...
        {q0,q1,q2,q3, dt, ...
         sig_gx,sig_gy,sig_gz, sig_ax,sig_ay,sig_az, ...
         sig_gbx,sig_gby,sig_gbz, sig_abx,sig_aby,sig_abz, sig_bb});

    % --- Compile H_MAG handle ---
    syms magN magE magD 'real'
    H_MAG_func = matlabFunction(S.H_MAG, 'Vars', {q0,q1,q2,q3, magN,magE,magD});

    assignin('base', 'ESKF16_F_func',     F_func);
    assignin('base', 'ESKF16_Q_func',     Q_func);
    assignin('base', 'ESKF16_H_MAG_func', H_MAG_func);
    assignin('base', 'ESKF16_loaded',     true);

    fprintf('[eskf16] Cached F/Q/H_MAG handles in base workspace.\n');
end


% =========================================================================
function p = locate_symbolic_mat_()
% Resolve the canonical .mat path relative to the repo root.
%
% The mat is committed at:
%   <repo_root>/Flight Images and Raw Data/casper_ekf16_symbolic.mat
%
% We walk up from this file (which lives in nav/eskf16/) to find the
% Simulink Development root, then up to the repo root.

    here = fileparts(mfilename('fullpath'));
    % here = .../Matlab Code/Simulink Development/nav/eskf16
    sim_root  = fileparts(fileparts(here));
    matlab_root = fileparts(sim_root);                  % .../Matlab Code
    repo_root   = fileparts(matlab_root);               % repo root
    p = fullfile(repo_root, 'Flight Images and Raw Data', ...
                 'casper_ekf16_symbolic.mat');
end
