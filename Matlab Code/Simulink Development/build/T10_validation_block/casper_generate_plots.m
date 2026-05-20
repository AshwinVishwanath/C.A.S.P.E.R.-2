function info = casper_generate_plots(Truth, Estimate, Sensors, RadioTX, OutDir, RunMeta)
%CASPER_GENERATE_PLOTS Render the PHASE 0 minimum-plot bundle.
%
% Synopsis:
%   info = casper_generate_plots(Truth, Estimate, Sensors, RadioTX, OutDir, RunMeta)
%
% Inputs:
%   Truth    : struct with time_s, alt_agl_m, vel_v_mps, accel_NED, mach,
%              quat_fw
%   Estimate : struct with time_s, state_x (Mx4), state_P_diag (Mx4),
%              quat_fw, mach_gate_active (Mx1), baro_innov struct (optional),
%              zupt_innov struct (optional)
%   Sensors  : struct (may be empty) with sensor traces for snapshot plots
%              fields are optional:
%                imu.time_s, imu.accel_mps2 (Nx3), imu.gyro_radps (Nx3)
%                adxl.time_s, adxl.accel_mps2 (Nx3)
%                baro.time_s, baro.alt_m
%                mag.time_s,  mag.uT (Nx3)
%                gps.time_s,  gps.pos_m (Nx3)
%   RadioTX  : struct with time_s, active (Nx1 logical), or empty
%   OutDir   : char path where PNGs will be saved
%   RunMeta  : struct with .test_id and .seed (used in titles)
%
% Outputs:
%   info : struct
%       files : cellstr of saved PNG absolute paths
%       n_generated : integer
%
% Style: 300 DPI, 8 x 5 inches, axes with units, dashed truth, solid estimate,
% color-blind palette, threshold bands at 20% opacity, titles with test ID + seed.

    arguments
        Truth    struct
        Estimate struct
        Sensors  = struct()
        RadioTX  = struct()
        OutDir   char = '.'
        RunMeta  = struct('test_id','PHASE0_trustgate','seed',20260519)
    end

    if ~isfolder(OutDir)
        mkdir(OutDir);
    end

    BLUE   = [0.00, 0.45, 0.74];   % estimate
    ORANGE = [0.85, 0.33, 0.10];   % truth
    GREEN  = [0.10, 0.65, 0.30];   % gate-active
    RED    = [0.75, 0.15, 0.10];   % failure
    GRAY   = [0.45, 0.45, 0.45];

    saved = {};

    test_tag = sprintf('%s seed=%d', RunMeta.test_id, RunMeta.seed);

    % --- 1. altitude_truth_vs_est.png ---
    f = new_fig();
    plot(Truth.time_s, Truth.alt_agl_m, '--', 'Color', ORANGE, 'LineWidth', 1.2, ...
        'DisplayName', 'truth alt');
    hold on;
    plot(Estimate.time_s, Estimate.state_x(:, 1), '-', 'Color', BLUE, ...
        'LineWidth', 1.2, 'DisplayName', 'est alt');
    [truth_apo, ti] = max(Truth.alt_agl_m);
    [est_apo, ei]   = max(Estimate.state_x(:, 1));
    plot(Truth.time_s(ti),   truth_apo, 'o', 'Color', ORANGE, 'MarkerFaceColor', ORANGE, ...
        'DisplayName', sprintf('truth apogee %.1f m', truth_apo));
    plot(Estimate.time_s(ei), est_apo,  's', 'Color', BLUE,   'MarkerFaceColor', BLUE, ...
        'DisplayName', sprintf('est apogee %.1f m', est_apo));
    xlabel('time (s)'); ylabel('altitude AGL (m)');
    title(sprintf('Altitude truth vs estimate -- %s', test_tag), 'Interpreter', 'none');
    legend('Location', 'best'); grid on;
    saved{end+1} = save_fig(f, OutDir, 'altitude_truth_vs_est.png');

    % --- 2. velocity_truth_vs_est.png ---
    f = new_fig();
    plot(Truth.time_s, Truth.vel_v_mps, '--', 'Color', ORANGE, 'LineWidth', 1.2, ...
        'DisplayName', 'truth vel');
    hold on;
    plot(Estimate.time_s, Estimate.state_x(:, 2), '-', 'Color', BLUE, ...
        'LineWidth', 1.2, 'DisplayName', 'est vel');
    burnout_idx = find(Truth.time_s >= 2 & Truth.accel_NED(:,3) >= -9.80665*0.99, 1, 'first');
    if ~isempty(burnout_idx)
        xline(Truth.time_s(burnout_idx), '-', 'Color', GRAY, 'LineWidth', 1.0, ...
            'DisplayName', sprintf('burnout %.2f s', Truth.time_s(burnout_idx)));
    end
    xlabel('time (s)'); ylabel('vertical velocity (m/s)');
    title(sprintf('Velocity truth vs estimate -- %s', test_tag), 'Interpreter', 'none');
    legend('Location', 'best'); grid on;
    saved{end+1} = save_fig(f, OutDir, 'velocity_truth_vs_est.png');

    % --- 3. attitude_error_euler.png ---
    f = new_fig();
    [eul_truth, eul_est] = compute_euler(Truth, Estimate);
    plot(Estimate.time_s, eul_est(:,1) - eul_truth(:,1), 'Color', BLUE,   'DisplayName', 'roll err');
    hold on;
    plot(Estimate.time_s, eul_est(:,2) - eul_truth(:,2), 'Color', ORANGE, 'DisplayName', 'pitch err');
    plot(Estimate.time_s, eul_est(:,3) - eul_truth(:,3), 'Color', GREEN,  'DisplayName', 'yaw err');
    xlabel('time (s)'); ylabel('Euler error (deg)');
    title(sprintf('Attitude error (Euler RPY) -- %s', test_tag), 'Interpreter', 'none');
    legend('Location', 'best'); grid on;
    saved{end+1} = save_fig(f, OutDir, 'attitude_error_euler.png');

    % --- 4. tilt_angle_error.png ---
    f = new_fig();
    tilt_deg = compute_tilt_deg(Truth, Estimate);
    plot(Estimate.time_s, tilt_deg, 'Color', BLUE, 'LineWidth', 1.0, ...
        'DisplayName', 'tilt err');
    hold on;
    yl = ylim;
    yl(1) = min(yl(1), 0);
    yl(2) = max(yl(2), 3);
    shade_band(0, 1, yl, BLUE);
    shade_band(1, 2, yl, GREEN);
    ylim(yl);
    yline(1.0, '--', 'Color', GREEN, 'LineWidth', 1.0, 'DisplayName', '1 deg (powered)');
    yline(2.0, '--', 'Color', RED,   'LineWidth', 1.0, 'DisplayName', '2 deg (coast)');
    xlabel('time (s)'); ylabel('total tilt error (deg)');
    title(sprintf('Tilt error vs time -- %s', test_tag), 'Interpreter', 'none');
    legend('Location', 'best'); grid on;
    saved{end+1} = save_fig(f, OutDir, 'tilt_angle_error.png');

    % --- 5. mach_gate_state.png ---
    f = new_fig();
    yyaxis left;
    plot(Truth.time_s, Truth.mach, 'Color', ORANGE, 'LineWidth', 1.2);
    ylabel('Mach number'); ylim_left = ylim;
    yline(0.40, '--', 'Color', RED,   'LineWidth', 0.8);
    yline(0.35, '--', 'Color', GREEN, 'LineWidth', 0.8);
    yyaxis right;
    stairs(Estimate.time_s, double(Estimate.mach_gate_active), 'Color', BLUE, ...
        'LineWidth', 1.2);
    ylabel('gate active (0/1)'); ylim([-0.1, 1.1]);
    xlabel('time (s)');
    title(sprintf('Mach gate state vs Mach number -- %s', test_tag), 'Interpreter', 'none');
    grid on;
    saved{end+1} = save_fig(f, OutDir, 'mach_gate_state.png');

    % --- 6. eskf_innovations.png ---
    f = new_fig();
    has_baro = isfield(Estimate, 'baro_innov') && ~isempty(Estimate.baro_innov);
    has_zupt = isfield(Estimate, 'zupt_innov') && ~isempty(Estimate.zupt_innov);
    if has_baro
        baro = Estimate.baro_innov;
        plot(baro.time_s, baro.innov_m, '.', 'Color', BLUE, 'DisplayName', 'baro innov');
        hold on;
        if isfield(baro, 'sigma_m')
            plot(baro.time_s,  5 * baro.sigma_m, '--', 'Color', GREEN, 'DisplayName', '+5 sigma');
            plot(baro.time_s, -5 * baro.sigma_m, '--', 'Color', GREEN, 'DisplayName', '-5 sigma');
        end
        if isfield(baro, 'rejected') && any(baro.rejected)
            plot(baro.time_s(baro.rejected), baro.innov_m(baro.rejected), ...
                'x', 'Color', RED, 'DisplayName', 'rejected');
        end
    end
    if has_zupt
        zupt = Estimate.zupt_innov;
        plot(zupt.time_s, zupt.innov_mps, '.', 'Color', ORANGE, 'DisplayName', 'zupt innov');
        hold on;
    end
    if ~has_baro && ~has_zupt
        text(0.5, 0.5, 'No innovation data supplied', 'HorizontalAlignment','center', ...
             'Units','normalized');
    end
    xlabel('time (s)'); ylabel('innovation (m or m/s)');
    title(sprintf('EKF innovations -- %s', test_tag), 'Interpreter', 'none');
    legend('Location', 'best'); grid on;
    saved{end+1} = save_fig(f, OutDir, 'eskf_innovations.png');

    % --- 7. bias_states.png ---
    f = new_fig();
    P0_AB = 0.025; P0_BB = 0.75;
    sig3_ab = 3 * sqrt(P0_AB);
    sig3_bb = 3 * sqrt(P0_BB);
    yyaxis left;
    plot(Estimate.time_s, Estimate.state_x(:,3), 'Color', BLUE, 'LineWidth', 1.0);
    ylabel('accel bias (m/s^2)');
    yline(+sig3_ab, '--', 'Color', GREEN);
    yline(-sig3_ab, '--', 'Color', GREEN);
    yyaxis right;
    plot(Estimate.time_s, Estimate.state_x(:,4), 'Color', ORANGE, 'LineWidth', 1.0);
    ylabel('baro bias (m)');
    yline(+sig3_bb, '--', 'Color', RED);
    yline(-sig3_bb, '--', 'Color', RED);
    xlabel('time (s)');
    title(sprintf('Bias states with +/-3 sigma -- %s', test_tag), 'Interpreter', 'none');
    grid on;
    saved{end+1} = save_fig(f, OutDir, 'bias_states.png');

    % --- 8. covariance_diag.png ---
    f = new_fig();
    if isfield(Estimate, 'state_P_diag') && ~isempty(Estimate.state_P_diag)
        semilogy(Estimate.time_s, Estimate.state_P_diag(:,1), 'Color', BLUE,   'DisplayName', 'P[alt]');
        hold on;
        semilogy(Estimate.time_s, Estimate.state_P_diag(:,2), 'Color', ORANGE, 'DisplayName', 'P[vel]');
        semilogy(Estimate.time_s, Estimate.state_P_diag(:,3), 'Color', GREEN,  'DisplayName', 'P[ab]');
        semilogy(Estimate.time_s, Estimate.state_P_diag(:,4), 'Color', RED,    'DisplayName', 'P[bb]');
    else
        text(0.5, 0.5, 'No P diag data', 'HorizontalAlignment','center', 'Units','normalized');
    end
    xlabel('time (s)'); ylabel('P diagonal (log scale)');
    title(sprintf('Covariance diagonals -- %s', test_tag), 'Interpreter', 'none');
    legend('Location', 'best'); grid on;
    saved{end+1} = save_fig(f, OutDir, 'covariance_diag.png');

    % --- 9. sensor_snapshot_pad.png (0..1s) ---
    saved{end+1} = sensor_snapshot(Sensors, [0, 1], OutDir, ...
        'sensor_snapshot_pad.png', sprintf('Sensor snapshot @ pad (0-1 s) -- %s', test_tag));

    % --- 10. sensor_snapshot_peak_mach.png (~t=10s) ---
    saved{end+1} = sensor_snapshot(Sensors, [9.5, 10.5], OutDir, ...
        'sensor_snapshot_peak_mach.png', ...
        sprintf('Sensor snapshot @ peak Mach (9.5-10.5 s) -- %s', test_tag));

    % --- 11. sensor_snapshot_drogue.png (~t=70s) ---
    saved{end+1} = sensor_snapshot(Sensors, [69.5, 70.5], OutDir, ...
        'sensor_snapshot_drogue.png', ...
        sprintf('Sensor snapshot @ drogue (69.5-70.5 s) -- %s', test_tag));

    % --- 12. mag_radio_interference_zoom.png (200 ms during a TX) ---
    f = new_fig();
    if isfield(Sensors, 'mag') && ~isempty(fieldnames(Sensors.mag)) && ...
       isfield(RadioTX, 'active') && any(RadioTX.active)
        % Pick first TX event after t>=2 s
        tx_t = RadioTX.time_s(:); tx_a = logical(RadioTX.active(:));
        first_tx = find(tx_a & tx_t >= 2, 1, 'first');
        if isempty(first_tx)
            first_tx = find(tx_a, 1, 'first');
        end
        t0 = tx_t(first_tx) - 0.05;
        t1 = t0 + 0.20;
        idx = Sensors.mag.time_s >= t0 & Sensors.mag.time_s <= t1;
        plot(Sensors.mag.time_s(idx), Sensors.mag.uT(idx, 1), 'Color', BLUE,   'DisplayName', 'mx');
        hold on;
        plot(Sensors.mag.time_s(idx), Sensors.mag.uT(idx, 2), 'Color', ORANGE, 'DisplayName', 'my');
        plot(Sensors.mag.time_s(idx), Sensors.mag.uT(idx, 3), 'Color', GREEN,  'DisplayName', 'mz');
        tx_zoom_idx = tx_t >= t0 & tx_t <= t1;
        plot(tx_t(tx_zoom_idx), 50 * double(tx_a(tx_zoom_idx)), 'k--', 'DisplayName', 'TX (0/50)');
        xlabel('time (s)'); ylabel('mag (uT) / TX flag');
        title(sprintf('Mag radio-TX interference (200 ms zoom) -- %s', test_tag), 'Interpreter','none');
        legend('Location', 'best'); grid on;
    else
        text(0.5, 0.5, 'No mag/RadioTX data', 'HorizontalAlignment','center', 'Units','normalized');
        title(sprintf('Mag radio-TX interference (no data) -- %s', test_tag), 'Interpreter','none');
    end
    saved{end+1} = save_fig(f, OutDir, 'mag_radio_interference_zoom.png');

    info = struct('files', {saved}, 'n_generated', numel(saved));
end

% ===================================================================
% ---- helpers ----
% ===================================================================

function f = new_fig()
    f = figure('Visible','off','Units','inches','Position',[0,0,8,5], ...
               'Color','w', 'PaperPositionMode','auto');
end

function path = save_fig(f, dir, fname)
    path = fullfile(dir, fname);
    set(f, 'PaperPosition', [0 0 8 5]);
    set(f, 'PaperSize', [8 5]);
    print(f, path, '-dpng', '-r300');
    close(f);
end

function shade_band(ymin, ymax, ylim_, color)
    % Shade a horizontal band across the current x-axis at 20% opacity.
    xl = xlim;
    p = patch([xl(1) xl(2) xl(2) xl(1)], [ymin ymin ymax ymax], color, ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility','off');
    %#ok<NASGU>
    ylim(ylim_);
end

function [eul_truth, eul_est] = compute_euler(Truth, Estimate)
% Resample truth quat onto estimate times, then convert to Euler ZYX deg.
    Mq = zeros(numel(Estimate.time_s), 4);
    for ax = 1:4
        Mq(:, ax) = interp1(Truth.time_s, Truth.quat_fw(:, ax), Estimate.time_s, ...
            'linear', 'extrap');
    end
    nrm = sqrt(sum(Mq.^2, 2)); nrm(nrm<eps)=1; Mq = Mq ./ nrm;
    eul_truth = rad2deg(quat2eul(Mq, 'ZYX'));
    qe = Estimate.quat_fw;
    nrme = sqrt(sum(qe.^2,2)); nrme(nrme<eps)=1; qe = qe ./ nrme;
    eul_est   = rad2deg(quat2eul(qe, 'ZYX'));
end

function tilt_deg = compute_tilt_deg(Truth, Estimate)
    Mq = zeros(numel(Estimate.time_s), 4);
    for ax = 1:4
        Mq(:, ax) = interp1(Truth.time_s, Truth.quat_fw(:, ax), Estimate.time_s, ...
            'linear', 'extrap');
    end
    nrm = sqrt(sum(Mq.^2, 2)); nrm(nrm<eps)=1; Mq = Mq ./ nrm;
    qe = Estimate.quat_fw;
    nrme = sqrt(sum(qe.^2,2)); nrme(nrme<eps)=1; qe = qe ./ nrme;
    M = numel(Estimate.time_s);
    tilt_deg = zeros(M,1);
    for k = 1:M
        qt = Mq(k,:); qq = qe(k,:);
        qc = [qt(1), -qt(2), -qt(3), -qt(4)];
        w =  qc(1)*qq(1) - qc(2)*qq(2) - qc(3)*qq(3) - qc(4)*qq(4);
        if w < 0
            w = -w;
        end
        w = max(min(w, 1.0), -1.0);
        tilt_deg(k) = 2 * acosd(w);
    end
end

function path = sensor_snapshot(Sensors, twin, OutDir, fname, ttl)
    f = new_fig();
    BLUE   = [0.00, 0.45, 0.74];
    ORANGE = [0.85, 0.33, 0.10];
    GREEN  = [0.10, 0.65, 0.30];
    plotted = false;

    if isfield(Sensors,'imu') && ~isempty(fieldnames(Sensors.imu))
        idx = Sensors.imu.time_s >= twin(1) & Sensors.imu.time_s <= twin(2);
        if any(idx)
            subplot(3,1,1);
            plot(Sensors.imu.time_s(idx), Sensors.imu.accel_mps2(idx,:));
            xlabel('time (s)'); ylabel('accel (m/s^2)');
            title('IMU accel'); grid on;
            subplot(3,1,2);
            plot(Sensors.imu.time_s(idx), Sensors.imu.gyro_radps(idx,:));
            xlabel('time (s)'); ylabel('gyro (rad/s)');
            title('IMU gyro'); grid on;
            plotted = true;
        end
    end
    if isfield(Sensors,'baro') && ~isempty(fieldnames(Sensors.baro))
        idx = Sensors.baro.time_s >= twin(1) & Sensors.baro.time_s <= twin(2);
        if any(idx)
            subplot(3,1,3);
            plot(Sensors.baro.time_s(idx), Sensors.baro.alt_m(idx), 'Color', ORANGE);
            xlabel('time (s)'); ylabel('baro alt (m)');
            title('Baro altitude'); grid on;
            plotted = true;
        end
    end
    if ~plotted
        text(0.5, 0.5, sprintf('No sensor data in [%g, %g] s', twin(1), twin(2)), ...
             'HorizontalAlignment','center', 'Units','normalized');
    end
    sgtitle(ttl, 'Interpreter','none');  %#ok<TITLE>
    path = save_fig(f, OutDir, fname);
end
