function [mag_uT_body, rebuilt] = casper_mag_step( ...
        mag_NED_uT, omega_body_radps, orientation_quat_wxyz, ...
        seed_base, reset_flag)
%CASPER_MAG_STEP Stateful wrapper around imuSensor('accel-gyro-mag') for MMC5983MA.
%
% Synopsis:
%   [mag_uT_body, rebuilt] = casper_mag_step( ...
%       mag_NED_uT, omega_body_radps, orientation_quat_wxyz, ...
%       seed_base, reset_flag)
%
% Inputs:
%   mag_NED_uT             (1x3 row, uT)     world-frame magnetic field
%                                            in NED [B_north, B_east, B_down]
%   omega_body_radps       (1x3 row, rad/s)  body-frame angular rate (unused
%                                            by the mag path but required by
%                                            the imuSensor call signature)
%   orientation_quat_wxyz  (1x4 row)         scalar-first body-to-NED quat
%   seed_base              (1x1 double)      RNG seed (typically Sim.Seed+3)
%   reset_flag             (1x1 logical)     true => re-construct imuSensor
%
% Outputs:
%   mag_uT_body  (1x3 row, uT)   noisy magnetometer measurement in body
%                                frame (standard aircraft, X-fwd Y-right Z-down).
%                                imuSensor performs the body-frame rotation
%                                via the orientation quaternion and adds
%                                white noise (Magnetometer.NoiseDensity),
%                                bias instability, and constant bias.
%   rebuilt      (1x1 logical)   true if the persistent magnetometer was
%                                rebuilt this call (diagnostic).
%
% Behavior:
%   - First call (or reset): constructs a persistent imuSensor System object
%     in 'accel-gyro-mag' mode and seeds from seed_base. Parameters drawn
%     from base workspace structs (Sim, Mag).
%   - The MagneticField property is updated every call from the input
%     mag_NED_uT so position-dependent fields (Phase 1 WMM) can flow through
%     without re-seeding.
%   - Hard iron / soft iron / axis flip / radio interference / 18-bit
%     quantization are NOT done here — they are firmware quirks and live
%     in casper_mag_quirks.m (post-step) and casper_radio_tx_step.m.
%
% This function is intended for use inside a MATLAB Function block in the
% T05 mag visual subsystem (mag_block_visual.slx). coder.extrinsic is used
% because imuSensor is a System object (not codegen-friendly inline).
%
% Source firmware reference:
%   Software/App/drivers/mmc5983ma.c (sample rate, bandwidth)
%   Frame conventions per ARCHITECTURE.md §3.1.

    coder.extrinsic('mag_construct_local', 'mag_step_local');

    persistent mag;
    persistent last_seed;

    if isempty(mag)
        mag = mag_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    elseif reset_flag || (seed_base ~= last_seed)
        mag = mag_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    else
        rebuilt = false;
    end

    % Ensure row-vector shape for imuSensor (it expects Nx3 / Nx4).
    field_in = reshape(double(mag_NED_uT),           1, 3);
    w_in     = reshape(double(omega_body_radps),     1, 3);
    q_in     = reshape(double(orientation_quat_wxyz), 1, 4);

    mag_uT_body = mag_step_local(mag, field_in, w_in, q_in);
end


function mag = mag_construct_local(seed_base)
% MAG_CONSTRUCT_LOCAL Build imuSensor('accel-gyro-mag') with mag params.
%   Pulls every parameter from base workspace structs so the configuration
%   stays consistent with casper_sensor_params (T02 output).
%
%   Only the Magnetometer block is configured (accel/gyro left at defaults
%   and ignored). MagneticField is initialized to [0 0 0] and overwritten
%   on every step() call so callers can supply position-dependent fields.

    Sim  = evalin('base', 'Sim');         %#ok<NASGU> reserved for future
    Magp = evalin('base', 'Mag');

    % Map firmware noise sigma to imuSensor NoiseDensity. Mag.NoiseStd_uT
    % is per-axis 1-sigma at the sensor sample rate (Mag.Rate_Hz). One-sided
    % PSD floor: NoiseDensity = sigma_per_sample / sqrt(Rate_Hz) (in uT/sqrt(Hz)
    % for one-sided spectrum). NoiseType 'double-sided' would halve the
    % required value; default in imuSensor is 'double-sided', so we set the
    % NoiseType explicitly to 'single-sided' to match the legacy AR(1)
    % white-driver convention (the AR(1) tail is added later in quirks).
    white_density_uT_sqrtHz = Magp.NoiseStd_uT / sqrt(Magp.Rate_Hz);

    mag = imuSensor('accel-gyro-mag', ...
        'SampleRate',     Magp.Rate_Hz, ...
        'RandomStream',   'mt19937ar with seed', ...
        'Seed',           uint32(mod(seed_base, 2^32)));

    % MagneticField is updated per-call; placeholder here.
    mag.MagneticField = [0, 0, 0];

    % Configure the Magnetometer (raw stock white-noise path only).
    mag.Magnetometer.NoiseDensity    = white_density_uT_sqrtHz * ones(1, 3);
    mag.Magnetometer.BiasInstability = zeros(1, 3);
    mag.Magnetometer.ConstantBias    = zeros(1, 3);
    mag.Magnetometer.NoiseType       = 'single-sided';
end


function mag_uT_body = mag_step_local(mag, field_NED_uT, w_body_radps, q_wxyz)
% MAG_STEP_LOCAL Call imuSensor with orientation + per-call magnetic field.
%   Wraps the orientation argument as a quaternion object and updates the
%   MagneticField property so position-dependent fields flow through
%   without forcing a re-construct. imuSensor handles the NED->body rotation
%   internally via the orientation quaternion.

    mag.MagneticField = field_NED_uT;
    q_obj = quaternion(q_wxyz);
    a_nav = [0, 0, 0];   % accel path ignored (mag-only consumer)
    [~, ~, mag_uT_body] = mag(a_nav, w_body_radps, q_obj);
end
