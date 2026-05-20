function result = casper_metric_attitude(Truth, Estimate)
%CASPER_METRIC_ATTITUDE Compute tilt RMS error from quaternions.
%
% Synopsis:
%   result = casper_metric_attitude(Truth, Estimate)
%
% Inputs:
%   Truth    : struct
%       time_s   (Nx1 double, seconds)
%       quat_fw  (Nx4 double; scalar-first [w x y z], body-to-nav)
%       (optional) alt_agl_m, vel_v_mps for burnout/apogee detection
%       (optional) accel_NED for burnout detection
%   Estimate : struct
%       time_s   (Mx1 double, seconds)
%       quat_fw  (Mx4 double; scalar-first)
%       state_x  (Mx4 double; column 1=alt, column 2=vel)
%
% Outputs:
%   result : MetricResult struct
%       value.rms_powered_deg
%       value.rms_coast_deg
%       value.tilt_max_deg
%       threshold struct (1 deg powered, 2 deg coast per PHASE0_SPEC §3.1)
%
% Tilt error: q_err = quat_mult(quat_conj(Truth.quat_fw(k,:)), Estimate.quat_fw(k,:))
% Force q_err.w >= 0; tilt_angle_deg = 2 * acosd(q_err.w).

    arguments
        Truth    struct
        Estimate struct
    end

    G = 9.80665;

    % --- Time alignment: interpolate Truth.quat_fw onto Estimate.time_s ---
    quat_truth_on_est = zeros(numel(Estimate.time_s), 4);
    for ax = 1:4
        quat_truth_on_est(:, ax) = interp1(Truth.time_s, Truth.quat_fw(:, ax), ...
            Estimate.time_s, 'linear', 'extrap');
    end
    % Re-normalize after interp
    nrm = sqrt(sum(quat_truth_on_est.^2, 2));
    nrm(nrm < eps) = 1;
    quat_truth_on_est = quat_truth_on_est ./ nrm;

    quat_est = Estimate.quat_fw;
    nrm_e = sqrt(sum(quat_est.^2, 2));
    nrm_e(nrm_e < eps) = 1;
    quat_est = quat_est ./ nrm_e;

    M = size(quat_est, 1);
    tilt_angle_deg = zeros(M, 1);
    for k = 1:M
        qt = quat_truth_on_est(k, :);
        qe = quat_est(k, :);
        % q_err = conj(qt) * qe   (Hamilton)
        qc = [qt(1), -qt(2), -qt(3), -qt(4)];
        w =  qc(1)*qe(1) - qc(2)*qe(2) - qc(3)*qe(3) - qc(4)*qe(4);
        x =  qc(1)*qe(2) + qc(2)*qe(1) + qc(3)*qe(4) - qc(4)*qe(3);
        y =  qc(1)*qe(3) - qc(2)*qe(4) + qc(3)*qe(1) + qc(4)*qe(2);
        z =  qc(1)*qe(4) + qc(2)*qe(3) - qc(3)*qe(2) + qc(4)*qe(1);
        if w < 0
            w = -w;
            x = -x;
            y = -y;
            z = -z;
        end
        % Clamp to [-1,1] for numerical safety
        w_clipped = max(min(w, 1.0), -1.0);
        tilt_angle_deg(k) = 2 * acosd(w_clipped);
        %#ok<*NASGU> -- x,y,z intentionally unused
    end

    % --- Burnout detection (same as velocity metric) ---
    accel_v_ned = Truth.accel_NED(:, 3);
    t_s = Truth.time_s;
    mask = (t_s >= 2.0) & (accel_v_ned >= -G * 0.99);
    if any(mask)
        burnout_time_s = t_s(find(mask, 1, 'first'));
    else
        [~, ii] = max(Truth.vel_v_mps);
        burnout_time_s = t_s(ii);
    end
    [~, apogee_idx_truth] = max(Truth.alt_agl_m);
    apogee_time_s = t_s(apogee_idx_truth);

    [~, burnout_idx_est] = min(abs(Estimate.time_s - burnout_time_s));
    [~, apogee_idx_est ] = min(abs(Estimate.time_s - apogee_time_s));
    if burnout_idx_est < 2
        burnout_idx_est = 2;
    end
    if apogee_idx_est <= burnout_idx_est
        apogee_idx_est = min(numel(Estimate.time_s), burnout_idx_est + 1);
    end

    rms_powered_deg = sqrt(mean(tilt_angle_deg(1:burnout_idx_est).^2));
    rms_coast_deg   = sqrt(mean(tilt_angle_deg(burnout_idx_est:apogee_idx_est).^2));
    tilt_max_deg    = max(tilt_angle_deg);

    thr = struct('rms_powered_deg', 1.0, 'rms_coast_deg', 2.0);

    pass_pwr = rms_powered_deg <= thr.rms_powered_deg;
    pass_coa = rms_coast_deg   <= thr.rms_coast_deg;
    pass_all = pass_pwr && pass_coa;

    val = struct( ...
        'rms_powered_deg',  rms_powered_deg, ...
        'rms_coast_deg',    rms_coast_deg, ...
        'tilt_max_deg',     tilt_max_deg, ...
        'tilt_angle_deg',   tilt_angle_deg, ...
        'burnout_time_s',   burnout_time_s, ...
        'apogee_time_s',    apogee_time_s);

    details = sprintf( ...
        'rms_pwr=%.4f deg (<=%.1f), rms_coast=%.4f deg (<=%.1f), max=%.4f deg', ...
        rms_powered_deg, thr.rms_powered_deg, ...
        rms_coast_deg,   thr.rms_coast_deg, ...
        tilt_max_deg);

    result = struct( ...
        'name',      'attitude', ...
        'value',     val, ...
        'threshold', thr, ...
        'pass',      pass_all, ...
        'details',   details);
end
