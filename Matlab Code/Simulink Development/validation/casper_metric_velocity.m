function result = casper_metric_velocity(Truth, Estimate)
%CASPER_METRIC_VELOCITY Compute velocity error metrics (powered/coast/burnout).
%
% Synopsis:
%   result = casper_metric_velocity(Truth, Estimate)
%
% Inputs:
%   Truth    : struct with fields
%       time_s     (Nx1 double, seconds)
%       vel_v_mps  (Nx1 double, vertical velocity up, m/s)
%       accel_NED  (Nx3 double; column 3 = vertical accel NED, Z-down)
%       alt_agl_m  (Nx1 double, m)
%   Estimate : struct with fields
%       time_s    (Mx1 double, seconds)
%       state_x   (Mx4 double; column 2 = velocity_mps)
%
% Outputs:
%   result : MetricResult struct
%       value     : struct with burnout/apogee indices, RMS values
%       threshold : struct(rms_powered_mps=5, burnout_err_mps=2, rms_coast_mps=3)
%       pass      : bool
%
% Burnout detection: first time Truth.accel_NED(:,3) >= -G * 0.99 after t>=2 s.
% This corresponds to vertical NED accel (down-positive) crossing -0.99*g, i.e.
% net vertical accel becoming roughly free-fall (gravity-only).

    arguments
        Truth    struct
        Estimate struct
    end

    G = 9.80665;

    % --- Burnout detection ---
    accel_v_ned_mps2 = Truth.accel_NED(:, 3);   % Z-down NED accel (gravity-free net?)
    % Truth.accel_NED is "gravity-free net acceleration" in NED Z-down (from
    % T01 docstring). So at burnout, net accel transitions from large negative
    % (thrust upward => -accel_z) to ~0. Use the firmware-style detector:
    % first time vertical NED accel goes back to >= -0.99*G (i.e. powered
    % thrust collapses). For a "gravity-free" net field this is just >= 0.
    %
    % To be tolerant of either convention, treat burnout as the first sample
    % after t>=2 s where accel_NED(:,3) >= -G*0.99 (works for both: with
    % gravity-included Z-down accel, free-fall is -G; with gravity-free,
    % powered is large positive Z-down (thrust = -accel_v_up < 0 means
    % accel_NED_z = -accel_v_up which is negative under thrust)). Use the
    % spec literal:
    t_s = Truth.time_s;
    mask = (t_s >= 2.0) & (accel_v_ned_mps2 >= -G * 0.99);
    if any(mask)
        burnout_idx_truth = find(mask, 1, 'first');
    else
        % Fallback: find when vel_v transitions from increasing to decreasing
        % during ascent (post t>=2 s)
        warning('casper_metric_velocity:NoBurnout', ...
            'Standard burnout detector found no candidate; falling back to vel-derivative.');
        [~, burnout_idx_truth] = max(Truth.vel_v_mps);
    end

    burnout_time_s = t_s(burnout_idx_truth);

    % Apogee in truth
    [~, apogee_idx_truth] = max(Truth.alt_agl_m);
    apogee_time_s = t_s(apogee_idx_truth);

    % --- Sample truth onto estimate time grid (or vice versa) ---
    % Resample truth to estimate timeline (interp). Use interp1, linear,
    % extrap with nearest endpoint.
    truth_vel_on_est = interp1(t_s, Truth.vel_v_mps, Estimate.time_s, ...
        'linear', 'extrap');

    % Burnout and apogee indices on the estimate grid
    [~, burnout_idx_est] = min(abs(Estimate.time_s - burnout_time_s));
    [~, apogee_idx_est ] = min(abs(Estimate.time_s - apogee_time_s));

    if burnout_idx_est < 2
        burnout_idx_est = 2;
    end
    if apogee_idx_est <= burnout_idx_est
        apogee_idx_est = min(numel(Estimate.time_s), burnout_idx_est + 1);
    end

    % --- Burnout point velocity error ---
    v_truth_at_burnout = truth_vel_on_est(burnout_idx_est);
    v_est_at_burnout   = Estimate.state_x(burnout_idx_est, 2);
    burnout_err_mps    = abs(v_est_at_burnout - v_truth_at_burnout);

    % --- RMS powered (0 .. burnout) ---
    idx_powered = 1:burnout_idx_est;
    err_powered = Estimate.state_x(idx_powered, 2) - truth_vel_on_est(idx_powered);
    rms_powered_mps = sqrt(mean(err_powered.^2));

    % --- RMS coast (burnout .. apogee) ---
    idx_coast = burnout_idx_est:apogee_idx_est;
    err_coast = Estimate.state_x(idx_coast, 2) - truth_vel_on_est(idx_coast);
    rms_coast_mps = sqrt(mean(err_coast.^2));

    thr = struct( ...
        'rms_powered_mps',  5.0, ...
        'burnout_err_mps',  2.0, ...   % PHASE0_SPEC §3.1
        'rms_coast_mps',    3.0);

    pass_pwr  = rms_powered_mps <= thr.rms_powered_mps;
    pass_brn  = burnout_err_mps <= thr.burnout_err_mps;
    pass_coa  = rms_coast_mps   <= thr.rms_coast_mps;
    pass_all  = pass_pwr && pass_brn && pass_coa;

    val = struct( ...
        'burnout_time_s',       burnout_time_s, ...
        'apogee_time_s',        apogee_time_s, ...
        'burnout_idx_est',      burnout_idx_est, ...
        'apogee_idx_est',       apogee_idx_est, ...
        'v_truth_at_burnout_mps', v_truth_at_burnout, ...
        'v_est_at_burnout_mps',   v_est_at_burnout, ...
        'burnout_err_mps',      burnout_err_mps, ...
        'rms_powered_mps',      rms_powered_mps, ...
        'rms_coast_mps',        rms_coast_mps);

    details = sprintf( ...
        'burnout @ %.2f s; rms_pwr=%.3f m/s (<=%.1f), burnout_err=%.3f m/s (<=%.1f), rms_coast=%.3f m/s (<=%.1f)', ...
        burnout_time_s, rms_powered_mps, thr.rms_powered_mps, ...
        burnout_err_mps, thr.burnout_err_mps, ...
        rms_coast_mps,   thr.rms_coast_mps);

    result = struct( ...
        'name',      'velocity', ...
        'value',     val, ...
        'threshold', thr, ...
        'pass',      pass_all, ...
        'details',   details);
end
