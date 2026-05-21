function casper_prelaunch_pad_truth_ts(pad_s)
%CASPER_PRELAUNCH_PAD_TRUTH_TS Prepend stationary pad samples to truth_ts.
%
% Synopsis:
%   casper_prelaunch_pad_truth_ts()         % default pad_s = 5.0
%   casper_prelaunch_pad_truth_ts(5.0)
%
% Modifies the base-workspace `truth_ts` struct-of-timeseries in place by
% prepending pad_s seconds of stationary samples (replicas of the first
% row of each field) at negative times. After this call, sim time t=0 in
% the visual model corresponds to launch detect at sim time t=pad_s.
%
% Why: the RasAero CSV / truth_trajectory.mat starts at LAUNCH (t=0), so
% truth_ts(0) already has the rocket moving. The casper_attitude static
% init requires ~5 s of stationary accel + 500 mag samples (5 s at 100 Hz)
% to converge. Without a pre-launch pad window the attitude estimator
% runs init() against boost data, producing a garbage initial quaternion
% and ~80-180 deg attitude error. The canonical MATLAB driver
% (casper_phase0_run.m) prepends 5 s of pad data via its 'PreLaunchPad_s'
% option; this helper does the same for the visual model.
%
% Idempotent: if truth_ts.<field>.Time(1) <= -pad_s + 1e-6 we assume the
% pad has already been prepended and return without doing anything. This
% lets us register the call as an InitFcn callback that fires before
% every sim run.
%
% Does NOT modify the model StopTime. The build sets StartTime = -pad_s
% so the solver natively sweeps the pad window, and the user-facing
% StopTime stays = "post-launch flight duration" they asked for.
%
% Inputs:
%   pad_s  (default 5.0)  seconds of stationary pad data to prepend.
%
% Side effects:
%   - reassigns `truth_ts` in base workspace (idempotent).
%
% No outputs.

    if nargin < 1 || isempty(pad_s)
        pad_s = 5.0;
    end
    pad_s = double(pad_s);
    if pad_s <= 0
        return;
    end

    % --- Pull truth_ts from base workspace -------------------------------
    if ~evalin('base', 'exist(''truth_ts'', ''var'')')
        warning('casper_prelaunch_pad_truth_ts:NoTruthTs', ...
            'No truth_ts in base workspace; nothing to pad.');
        return;
    end
    truth_ts = evalin('base', 'truth_ts');

    fns = fieldnames(truth_ts);
    if isempty(fns)
        return;
    end

    % --- Idempotency check using the first timeseries --------------------
    first_ts = truth_ts.(fns{1});
    if ~isa(first_ts, 'timeseries')
        warning('casper_prelaunch_pad_truth_ts:NotTs', ...
            'truth_ts.%s is not a timeseries; aborting.', fns{1});
        return;
    end
    if first_ts.Time(1) <= -pad_s + 1e-6
        % Already padded; skip silently.
        return;
    end

    % --- Sample period (assumed uniform inside each ts) ------------------
    if numel(first_ts.Time) >= 2
        dt = first_ts.Time(2) - first_ts.Time(1);
    else
        dt = 1e-3;   % fallback
    end
    N_pad = max(1, round(pad_s / dt));

    % Pad timestamps run from (t_orig(1) - N_pad*dt) up through
    % (t_orig(1) - dt). They MUST land strictly before t_orig(1) so the
    % concatenated time vector is monotonic.
    t0 = first_ts.Time(1);
    pad_t = (t0 - N_pad*dt) + (0:N_pad-1).' * dt;

    % --- Prepend each field ----------------------------------------------
    % All truth_ts fields are constructed via timeseries(Nx*, Nx1, ...),
    % so Data is row-major (rows == time). Replicate the first row N_pad
    % times and prepend it.
    truth_ts_out = truth_ts;
    for k = 1:numel(fns)
        f = fns{k};
        ts_old = truth_ts.(f);
        if ~isa(ts_old, 'timeseries')
            continue;
        end
        D = ts_old.Data;
        T = ts_old.Time(:);

        if ~(ismatrix(D) && size(D, 1) == numel(T))
            % Unknown shape; leave field alone.
            continue;
        end

        row0 = D(1, :);
        pad_data = repmat(row0, N_pad, 1);
        new_data = [pad_data; D];
        T_concat = [pad_t; T];

        % Special case: time_s carries timestamps both as Data AND Time
        % (built as `timeseries(t, t, ...)`). Replace Data with the new
        % monotonic time vector so consumers that read truth_ts.time_s
        % as a SIGNAL get the negative-through-positive timestamps too.
        if strcmp(f, 'time_s')
            new_data = T_concat;
        end

        ts_new = timeseries(new_data, T_concat, 'Name', ts_old.Name);
        truth_ts_out.(f) = ts_new;
    end

    assignin('base', 'truth_ts', truth_ts_out);

    fprintf('[prelaunch_pad] prepended %.2f s (%d samples @ dt=%.4g) to truth_ts; new t0 = %.3f s\n', ...
        pad_s, N_pad, dt, pad_t(1));
end
