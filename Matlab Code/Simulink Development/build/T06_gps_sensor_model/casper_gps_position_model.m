function [lat_deg7, lon_deg7, alt_msl_mm, vel_n_mm_s, vel_e_mm_s, vel_d_mm_s] = ...
    casper_gps_position_model(pos_NED, vel_NED, ...
        lat0_deg, lon0_deg, alt0_m, earth_radius_m, ...
        sigma_h_m, sigma_v_m, sigma_vel_mps, seed)
%CASPER_GPS_POSITION_MODEL Convert truth pos/vel to MAX-M10M NAV-PVT units + noise.
%
% Synopsis:
%   [lat_deg7, lon_deg7, alt_msl_mm, vel_n_mm_s, vel_e_mm_s, vel_d_mm_s] = ...
%       casper_gps_position_model(pos_NED, vel_NED, ...
%           lat0_deg, lon0_deg, alt0_m, earth_radius_m, ...
%           sigma_h_m, sigma_v_m, sigma_vel_mps, seed)
%
% Inputs:
%   pos_NED          (3x1 double, m, NED with Z down)
%   vel_NED          (3x1 double, m/s, NED with Z down)
%   lat0_deg, lon0_deg, alt0_m  launch-site origin (scalar)
%   earth_radius_m   spherical Earth radius (scalar, m)
%   sigma_h_m        per-axis horizontal position sigma (m)
%   sigma_v_m        vertical position sigma (m)
%   sigma_vel_mps    per-axis velocity sigma (m/s)
%   seed             scalar uint32-compatible seed; set once on first call
%                    and ignored thereafter.
%
% Outputs (firmware NAV-PVT native units):
%   lat_deg7    int32, latitude  scaled by 1e7 deg
%   lon_deg7    int32, longitude scaled by 1e7 deg
%   alt_msl_mm  int32, altitude above MSL in mm
%   vel_n_mm_s  int32, north velocity in mm/s
%   vel_e_mm_s  int32, east  velocity in mm/s
%   vel_d_mm_s  int32, down  velocity in mm/s
%
% Noise:
%   White Gaussian, per-axis independent, seeded via a persistent RandStream
%   so the output is deterministic given the seed (SIMULINK_PATTERNS.md §5,
%   ARCHITECTURE.md §6).
%
% Source firmware reference:
%   Software/App/drivers/max_m10m.c (NAV-PVT parser fields lat/lon/hMSL/velN/velE/velD,
%   all stored as int32 in the same units returned here).

%#codegen

    persistent rs
    if isempty(rs)
        rs = RandStream('mt19937ar', 'Seed', uint32(seed));
    end

    % --- Flat-earth lat/lon conversion (T06 §5.1) ---
    lat0_rad = lat0_deg * pi / 180;

    dlat_rad = pos_NED(1) / earth_radius_m;
    dlon_rad = pos_NED(2) / (earth_radius_m * cos(lat0_rad));

    lat_deg = lat0_deg + dlat_rad * 180 / pi;
    lon_deg = lon0_deg + dlon_rad * 180 / pi;

    % Altitude (NED Z is down -> alt above launch = -pos_NED(3); MSL adds origin).
    alt_msl_m = alt0_m - pos_NED(3);

    % Velocity in NED (D is down, matches firmware's velD).
    vn_mps = vel_NED(1);
    ve_mps = vel_NED(2);
    vd_mps = vel_NED(3);

    % --- White position noise (per-axis N, E, D meters) ---
    n_noise_m   = sigma_h_m * randn(rs);
    e_noise_m   = sigma_h_m * randn(rs);
    d_noise_m   = sigma_v_m * randn(rs);

    % --- White velocity noise (per-axis m/s) ---
    vn_noise_mps = sigma_vel_mps * randn(rs);
    ve_noise_mps = sigma_vel_mps * randn(rs);
    vd_noise_mps = sigma_vel_mps * randn(rs);

    % Apply noise: position-noise in meters maps to lat/lon via the same
    % flat-earth scale used above (N -> latitude, E -> longitude).
    lat_noise_deg = (n_noise_m / earth_radius_m)                * 180 / pi;
    lon_noise_deg = (e_noise_m / (earth_radius_m * cos(lat0_rad))) * 180 / pi;

    lat_deg_noisy  = lat_deg + lat_noise_deg;
    lon_deg_noisy  = lon_deg + lon_noise_deg;
    % Vertical noise on altitude: NED-D positive is downward, so a +d_noise
    % in NED-D is a -altitude perturbation. Subtract.
    alt_msl_m_noisy = alt_msl_m - d_noise_m;

    vn_mps_noisy = vn_mps + vn_noise_mps;
    ve_mps_noisy = ve_mps + ve_noise_mps;
    vd_mps_noisy = vd_mps + vd_noise_mps;

    % --- Quantize to firmware NAV-PVT integer scales ---
    lat_deg7    = int32(round(lat_deg_noisy * 1e7));
    lon_deg7    = int32(round(lon_deg_noisy * 1e7));
    alt_msl_mm  = int32(round(alt_msl_m_noisy * 1000));
    vel_n_mm_s  = int32(round(vn_mps_noisy * 1000));
    vel_e_mm_s  = int32(round(ve_mps_noisy * 1000));
    vel_d_mm_s  = int32(round(vd_mps_noisy * 1000));
end
