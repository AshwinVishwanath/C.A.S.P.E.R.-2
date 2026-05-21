function [lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps, rebuilt] = ...
        casper_gps_step_visual(pos_NED, vel_NED, seed_base, reset_flag)
%CASPER_GPS_STEP_VISUAL Stateful wrapper around gpsSensor for MAX-M10M.
%
% Synopsis:
%   [lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps, rebuilt] = ...
%       casper_gps_step_visual(pos_NED, vel_NED, seed_base, reset_flag)
%
% Inputs:
%   pos_NED       (1x3 row, m)    NED position (Z down) relative to launch site
%   vel_NED       (1x3 row, m/s)  NED velocity (Z down)
%   seed_base     (1x1 double)    RNG seed (typically Sim.Seed + 5)
%   reset_flag    (1x1 logical)   true => rebuild the persistent gpsSensor
%
% Outputs:
%   lat_deg   (1x1 double, deg)   noisy latitude  (WGS84)
%   lon_deg   (1x1 double, deg)   noisy longitude
%   alt_m     (1x1 double, m)     noisy altitude above launch site MSL
%   vn_mps    (1x1 double, m/s)   noisy NED north velocity
%   ve_mps    (1x1 double, m/s)   noisy NED east velocity
%   vd_mps    (1x1 double, m/s)   noisy NED down velocity
%   rebuilt   (1x1 logical)       diagnostic: true if the sensor was rebuilt
%
% Behavior:
%   - First call (or reset): constructs a persistent gpsSensor System object
%     with parameters drawn from base workspace structs (GPS, Sim). Reference
%     location is taken from GPS.LaunchLat_deg / LaunchLon_deg / LaunchAlt_m
%     when present, otherwise from defaults (London placeholder).
%   - Subsequent calls: calls step() on the persistent gpsSensor with the
%     truth NED position / velocity at the launch site reference frame.
%
% This function is intended for use inside a MATLAB Function block in the
% T06 GPS visual subsystem (gps_block_visual.slx). coder.extrinsic is used
% because gpsSensor is a System object (not codegen-friendly inline).
%
% Source firmware reference:
%   Software/App/drivers/max_m10m.c (NAV-PVT producer; this function only
%   provides the lat/lon/alt/vel quantities prior to firmware quantization).

    coder.extrinsic('gps_construct_local', 'gps_step_local');

    persistent gps;
    persistent last_seed;

    if isempty(gps)
        gps = gps_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    elseif reset_flag || (seed_base ~= last_seed)
        gps = gps_construct_local(seed_base);
        last_seed = seed_base;
        rebuilt = true;
    else
        rebuilt = false;
    end

    % Default initialisations so codegen-time types are known.
    lat_deg = 0.0;
    lon_deg = 0.0;
    alt_m   = 0.0;
    vn_mps  = 0.0;
    ve_mps  = 0.0;
    vd_mps  = 0.0;

    p_in = reshape(double(pos_NED), 1, 3);
    v_in = reshape(double(vel_NED), 1, 3);

    [lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps] = gps_step_local(gps, p_in, v_in);
end


function gps = gps_construct_local(seed_base)
% GPS_CONSTRUCT_LOCAL Build gpsSensor with parameters from base workspace.
%   Pulls every parameter from base workspace structs so the configuration
%   stays consistent with casper_sensor_params (T02) and the T06-local
%   launch-site override produced by casper_gps_local_params.

    GPSp = evalin('base', 'GPS');

    % Launch-site origin: prefer values pre-loaded onto the GPS struct
    % (casper_gps_local_params injects them), else use the spec defaults.
    if isfield(GPSp, 'LaunchLat_deg')
        ref_lat_deg = GPSp.LaunchLat_deg;
    else
        ref_lat_deg = 51.5074;   % London placeholder
    end
    if isfield(GPSp, 'LaunchLon_deg')
        ref_lon_deg = GPSp.LaunchLon_deg;
    else
        ref_lon_deg = -0.1278;
    end
    if isfield(GPSp, 'LaunchAlt_m')
        ref_alt_m = GPSp.LaunchAlt_m;
    else
        ref_alt_m = 35.0;
    end

    % Per-axis sigmas from spec: CEP_horizontal / sqrt(2); vertical = 1.5 * h.
    sigma_h = GPSp.PositionCEP_Horizontal_m / sqrt(2);
    sigma_v = 1.5 * sigma_h;
    sigma_vel = GPSp.VelocityNoise_mps;

    % gpsSensor noise inputs are per-axis sigmas. Decorrelation time of 0
    % => white noise (T06 anti-goal: no colored noise in Phase 0).
    gps = gpsSensor( ...
        'SampleRate',         double(GPSp.Rate_Hz), ...
        'ReferenceFrame',     'NED', ...
        'ReferenceLocation',  [ref_lat_deg, ref_lon_deg, ref_alt_m], ...
        'HorizontalPositionAccuracy',  sigma_h, ...
        'VerticalPositionAccuracy',    sigma_v, ...
        'VelocityAccuracy',            sigma_vel, ...
        'DecayFactor',                 0.0, ...
        'RandomStream',                'mt19937ar with seed', ...
        'Seed',                        uint32(mod(seed_base, 2^32)));
end


function [lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps] = ...
        gps_step_local(gps, pos_NED, vel_NED)
% GPS_STEP_LOCAL Call gpsSensor on the truth NED state and split the LLA output.
%   gpsSensor returns [lla, vel, groundspeed, course] for each input row.
%   We use only lla and vel.

    [lla, vel] = gps(pos_NED, vel_NED);

    lat_deg = lla(1, 1);
    lon_deg = lla(1, 2);
    alt_m   = lla(1, 3);
    vn_mps  = vel(1, 1);
    ve_mps  = vel(1, 2);
    vd_mps  = vel(1, 3);
end
