function GPS_local = casper_gps_local_params(GPS, Sim)
%CASPER_GPS_LOCAL_PARAMS T06-local extensions to the GPS struct.
%
% Synopsis:
%   GPS_local = casper_gps_local_params(GPS, Sim)
%
% Inputs:
%   GPS : base GPS struct from casper_sensor_params (T02)
%   Sim : base Sim struct from casper_sensor_params (T02)
%
% Outputs:
%   GPS_local : GPS struct augmented with launch-site origin and derived
%               per-axis noise sigmas. Augmentation is additive only;
%               no T02 fields are modified.
%
% Added fields:
%   LaunchLat_deg            (deg, default 51.5074)
%   LaunchLon_deg            (deg, default -0.1278)
%   LaunchAlt_m              (m,   default 35.0)
%   PositionSigmaHorizontal_m   per-axis horizontal sigma  = CEP / sqrt(2)
%   PositionSigmaVertical_m     vertical sigma             = 1.5 * horizontal
%   VelocitySigma_mps           per-axis velocity sigma    = GPS.VelocityNoise_mps
%   EarthRadius_m            (m,   6371000)
%   ReacquireTime_s          (s,   1.0)  COCOM-exit re-acquire transient
%   Seed                     derived  = Sim.Seed + 5  (ARCHITECTURE.md §6)
%
% Source firmware reference:
%   None.  Phase-0 sim convention; launch site is a placeholder.

    GPS_local = GPS;

    % Launch-site origin (London default; configurable downstream).
    GPS_local.LaunchLat_deg = 51.5074;   % deg
    GPS_local.LaunchLon_deg = -0.1278;   % deg
    GPS_local.LaunchAlt_m   = 35.0;      % m above MSL

    % Per-axis noise sigmas derived from the T02 CEP.
    % CEP (50% probability circle) -> per-axis sigma:  CEP / sqrt(2)
    % (the standard 2-D Rayleigh approximation: sigma_axis = CEP / 1.1774
    % is closer for a true 50% CEP, but T06 spec section 5.2 specifies
    % CEP / sqrt(2)).
    GPS_local.PositionSigmaHorizontal_m = GPS.PositionCEP_Horizontal_m / sqrt(2);
    GPS_local.PositionSigmaVertical_m   = 1.5 * GPS_local.PositionSigmaHorizontal_m;
    GPS_local.VelocitySigma_mps         = GPS.VelocityNoise_mps;

    % Earth-radius approximation used by the spec's flat-earth lat/lon
    % conversion.  Sphere of mean Earth radius (m).
    GPS_local.EarthRadius_m = 6371000.0;

    % Time for the post-COCOM re-acquire transient (fix_type 0 -> 2 -> 3).
    GPS_local.ReacquireTime_s = 1.0;

    % Per-sensor seed (ARCHITECTURE.md §6: +5 for GPS).
    GPS_local.Seed = Sim.Seed + 5;
end
