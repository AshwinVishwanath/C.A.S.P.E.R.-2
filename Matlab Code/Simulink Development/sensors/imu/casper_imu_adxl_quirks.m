function [accel_g_out, lpf_state_out, fifo_active_out] = casper_imu_adxl_quirks( ...
        accel_mps2_in, lpf_state_in, ...
        alt_m, dt, bandwidth_hz, ...
        scale_g_per_lsb, range_g, ...
        fifo_active_in, fifo_alt_thresh_m)
%CASPER_IMU_ADXL_QUIRKS Firmware-quirk wrapper for the ADXL372 PostQuirks stage.
%
% Synopsis:
%   [a_g, lpf_state, fifo_active] = casper_imu_adxl_quirks( ...
%       a_mps2, lpf_state_prev, alt_m, dt, bw_hz, ...
%       scale_g_per_lsb, range_g, fifo_active_prev, fifo_alt_thresh)
%
% Purpose:
%   Visual-model T03 ADXL372 subsystem uses imuSensor (Sensor Fusion Tbx,
%   high-g config) for bias + white-noise generation. imuSensor outputs
%   SI units and does not provide a configurable anti-alias LPF, LSB
%   quantization, hardware saturation, or the FIFO altitude-trigger latch.
%   This wrapper covers all of those.
%
% Chain:
%   1. Unit convert m/s^2 -> g
%   2. First-order Butterworth LPF at bandwidth_hz (recursive y[k] form)
%   3. LSB quantization
%   4. Hardware saturation to +-range_g
%   5. FIFO latch: one-way alt > fifo_alt_thresh_m -> fifo_active=true.
%
% Inputs:
%   accel_mps2_in     (3x1 double, m/s^2) imuSensor accel output
%   lpf_state_in      (3x1 double, g)     previous y[k-1] of the LPF
%   alt_m             (1x1 double, m)     truth altitude (for FIFO latch)
%   dt                (1x1 double, s)     sample period
%   bandwidth_hz      (1x1 double, Hz)    LPF cutoff; 0 -> bypass
%   scale_g_per_lsb   (1x1 double, g)     quantization step (0.1 g typical)
%   range_g           (1x1 double, g)     saturation +- value (200 g typ)
%   fifo_active_in    (1x1 logical)       previous FIFO latch state
%   fifo_alt_thresh_m (1x1 double, m)     altitude that latches FIFO on (5 m)
%
% Outputs:
%   accel_g_out      (3x1 double, g)
%   lpf_state_out    (3x1 double, g) carry y[k] for next call
%   fifo_active_out  (1x1 logical)
%
% State:
%   The LPF and FIFO state are passed in/out explicitly so this function is
%   pure and Simulink-friendly (no persistent variables). Inside a MATLAB
%   Function block, wrap with Memory or Unit Delay blocks to hold lpf_state
%   and fifo_active across ticks.

    G0 = 9.80665;

    if dt <= 0
        error('casper_imu_adxl_quirks:BadDt', 'dt must be > 0, got %g', dt);
    end

    % 1) Unit convert
    a_g = accel_mps2_in(:) / G0;

    % 2) LPF (1st-order Butterworth via bilinear; bypass if bw_hz<=0)
    if bandwidth_hz <= 0
        lpf_alpha = 1.0;
    else
        rc = 1.0 / (2.0 * pi * bandwidth_hz);
        lpf_alpha = dt / (rc + dt);
    end
    y_prev = lpf_state_in(:);
    a_g = lpf_alpha * a_g + (1.0 - lpf_alpha) * y_prev;
    lpf_state_out = a_g;

    % 3) Quantization
    a_g = round(a_g / scale_g_per_lsb) * scale_g_per_lsb;

    % 4) Saturation
    a_g = min(max(a_g, -range_g), range_g);

    accel_g_out = a_g;

    % 5) FIFO latch (one-way: once true, stays true)
    if fifo_active_in
        fifo_active_out = true;
    else
        fifo_active_out = (alt_m > fifo_alt_thresh_m);
    end
end
