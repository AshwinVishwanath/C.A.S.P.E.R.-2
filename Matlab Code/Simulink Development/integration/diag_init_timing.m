function diag_init_timing()
%DIAG_INIT_TIMING Direct test of attitude_step_helper init timing without
% running a full sim. Simulates the call pattern the visual model uses
% (1 kHz accel/gyro, 100 Hz mag) on stationary truth and reports when
% init_complete first goes true.

    fprintf('\n==== diag_init_timing ====\n');

    % Make sure paths are set up
    if ~exist('casper_sim_config', 'file')
        cd('..'); casper('Profile','smoke'); cd('integration');
    end

    % Clear persistent state in attitude_step_helper
    clear attitude_step_helper

    % Fresh Attitude struct in base
    Attitude = evalin('base', 'Attitude');
    assert(isfield(Attitude, 'StaticInitTimeout_s'));
    fprintf('  StaticInitTimeout_s = %.2f s\n', Attitude.StaticInitTimeout_s);
    fprintf('  StaticInitSamples   = %d\n', Attitude.StaticInitSamples);

    % Provide a mag ref (visual model defaults to [0;0;50] if not set)
    if ~isfield(Attitude, 'M_ref_nav_uT')
        Attitude.M_ref_nav_uT = [0;0;50];
        assignin('base', 'Attitude', Attitude);
    end

    % Stationary inputs in body-fw frame: accel = [0,0,+g] (per visual model
    % chain: imuSensor +g UP -> negate -> frame_switch_body), gyro=0,
    % mag = field rotated to body-fw frame.
    accel = [0; 0; 9.80665];
    gyro  = [0; 0; 0];
    mag   = [20; 0; -45];   % arbitrary stationary mag

    dt = 1e-3;             % 1 kHz attitude tick
    mag_dt = 1e-2;         % 100 Hz
    mag_period_ticks = round(mag_dt / dt);

    T_sim = 12.0;          % run 12 s to see if it ever fires
    N = round(T_sim / dt);
    init_t = NaN;
    for k = 1:N
        t_now = (k-1) * dt;
        is_mag_tick = (mod(k-1, mag_period_ticks) == 0);
        [~, ~, ~, init_complete] = attitude_step_helper( ...
            accel, gyro, mag, is_mag_tick, true, dt);
        if init_complete && isnan(init_t)
            init_t = t_now;
            fprintf('  -> init_complete TRUE at t = %.4f s (sample %d)\n', t_now, k);
            break;
        end
    end
    if isnan(init_t)
        fprintf('  -> init_complete NEVER fired in %.1f s\n', T_sim);
    end

    fprintf('==== diag_init_timing DONE ====\n\n');
end
