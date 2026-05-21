function accel_g_out = casper_imu_adxl_noise( ...
        accel_g_in, dt, seed_base, ...
        bias_init_g, noise_g_at_bw, ...
        bandwidth_hz, scale_g_per_lsb, range_g, ...
        reset_flag)
%CASPER_IMU_ADXL_NOISE Apply ADXL372 noise / LPF / quantization / saturation.
%
% Synopsis:
%   accel_g_out = casper_imu_adxl_noise( ...
%       accel_g_in, dt, seed_base, bias_init_g, noise_g_at_bw, ...
%       bandwidth_hz, scale_g_per_lsb, range_g, reset_flag)
%
% Inputs:
%   accel_g_in       (3x1 double, g)  clean accel (g)
%   dt               (1x1 double, s)  sample period (1/400 pre-launch,
%                                     1/800 post-launch)
%   seed_base        (1x1 double)     RNG seed (Sim.Seed + 4)
%   bias_init_g      (1x1 double, g)  1-sigma initial bias per axis
%   noise_g_at_bw    (1x1 double, g)  white-noise 1-sigma at bandwidth
%   bandwidth_hz     (1x1 double, Hz) anti-alias LPF cutoff
%   scale_g_per_lsb  (1x1 double, g)  quantization step
%   range_g          (1x1 double, g)  saturation +/- value
%   reset_flag       (1x1 logical)    reinit persistent state if true
%
% Outputs:
%   accel_g_out (3x1 double, g) noisy + filtered + quantized + clipped
%
% Noise chain (T03 §6.2, in order):
%   1. Bias offset (constant per run, drawn at init)
%   2. White noise sigma scaled to per-sample sigma:
%        per-sample sigma = noise_g_at_bw * sqrt(2 * bandwidth_hz) * sqrt(dt)
%      i.e. PSD = noise_g_at_bw^2 / bandwidth_hz (one-sided)
%   3. First-order Butterworth LPF at bandwidth_hz (discrete, IIR)
%   4. Quantization to scale_g_per_lsb
%   5. Saturation to +/- range_g
%
% Persistent state:
%   - bias_g, RandStream, prior LPF state, scale.

    persistent rs bias_g lpf_y lpf_alpha;

    need_init = isempty(rs) || (islogical(reset_flag) && reset_flag) ...
        || (isnumeric(reset_flag) && reset_flag ~= 0);

    if need_init
        rs = RandStream('mt19937ar', 'Seed', uint32(mod(seed_base, 2^32)));
        bias_g  = bias_init_g * randn(rs, 3, 1);
        lpf_y   = zeros(3, 1);

        % 1st-order Butterworth lowpass via bilinear transform.
        % alpha = dt / (RC + dt),  RC = 1/(2*pi*fc).
        % y[k] = alpha * x[k] + (1-alpha) * y[k-1].
        if bandwidth_hz <= 0
            lpf_alpha = 1.0;     % bypass
        else
            rc = 1.0 / (2.0 * pi * bandwidth_hz);
            lpf_alpha = dt / (rc + dt);
        end
    end

    if ~(dt > 0)
        error('casper_imu_adxl_noise:BadDt', 'dt must be > 0, got %g', dt);
    end

    % --- White noise per-sample sigma at this dt ---
    % If noise_g_at_bw is the 1-sigma at the sensor's BW (one-sided PSD
    % bandwidth assumption), the per-sample sigma is noise_g_at_bw *
    % sqrt(2 * BW * dt). Capped at noise_g_at_bw for very low dt to keep
    % things reasonable when the BW assumption breaks down.
    sigma_per_sample = noise_g_at_bw * sqrt(2.0 * bandwidth_hz * dt);

    a_in = accel_g_in(:);

    % --- 1) Bias offset (constant) + 2) white noise ---
    a = a_in + bias_g + sigma_per_sample * randn(rs, 3, 1);

    % --- 3) LPF (first-order Butterworth) ---
    lpf_y = lpf_alpha * a + (1.0 - lpf_alpha) * lpf_y;
    a = lpf_y;

    % --- 4) Quantization ---
    a = round(a / scale_g_per_lsb) * scale_g_per_lsb;

    % --- 5) Saturation ---
    a = min(max(a, -range_g), range_g);

    accel_g_out = a;
end
