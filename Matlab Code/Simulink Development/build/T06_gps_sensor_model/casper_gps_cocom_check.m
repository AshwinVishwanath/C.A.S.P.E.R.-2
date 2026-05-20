function [cocom_active, fix_type, num_sv] = casper_gps_cocom_check( ...
    vel_NED, pos_NED, time_s, vel_thresh_mps, alt_thresh_m, reacquire_s)
%CASPER_GPS_COCOM_CHECK COCOM dropout gate + fix-type / sat-count scheduler.
%
% Synopsis:
%   [cocom_active, fix_type, num_sv] = casper_gps_cocom_check( ...
%       vel_NED, pos_NED, time_s, vel_thresh_mps, alt_thresh_m, reacquire_s)
%
% Inputs:
%   vel_NED         (3x1 double, m/s, NED)
%   pos_NED         (3x1 double, m,   NED with Z down)
%   time_s          scalar  current sim time (s)
%   vel_thresh_mps  scalar  COCOM velocity threshold (m/s, e.g. 500)
%   alt_thresh_m    scalar  COCOM altitude threshold (m,   e.g. 18000)
%   reacquire_s     scalar  duration of 2D-fix transient after COCOM exit (s)
%
% Outputs:
%   cocom_active   logical  true if COCOM gate currently active
%   fix_type       uint8    0 = no fix (COCOM), 2 = 2D (re-acquire), 3 = 3D
%   num_sv         uint8    0  / 4 / 12  matching the fix-type schedule
%
% Schedule (T06 §5.3, §5.5):
%   in COCOM                  -> fix_type=0, num_sv=0
%   t in [t_exit, t_exit+1s]  -> fix_type=2, num_sv=4
%   otherwise                 -> fix_type=3, num_sv=12
%
% Source firmware reference:
%   Software/App/drivers/max_m10m.h  gps_fix_type_t {0=NONE, 2=2D, 3=3D}

%#codegen

    persistent cocom_exit_time_s prev_active
    if isempty(cocom_exit_time_s)
        cocom_exit_time_s = -Inf;   % no exit yet
        prev_active       = false;
    end

    v_total = sqrt(vel_NED(1)^2 + vel_NED(2)^2 + vel_NED(3)^2);
    alt_m   = -pos_NED(3);     % NED Z is down -> altitude = -Z

    cocom_active = (v_total > vel_thresh_mps) && (alt_m > alt_thresh_m);

    % Latch the most recent COCOM-exit time (transition true -> false).
    if prev_active && ~cocom_active
        cocom_exit_time_s = time_s;
    end
    prev_active = cocom_active;

    if cocom_active
        fix_type = uint8(0);
        num_sv   = uint8(0);
    elseif (time_s - cocom_exit_time_s) < reacquire_s
        fix_type = uint8(2);
        num_sv   = uint8(4);
    else
        fix_type = uint8(3);
        num_sv   = uint8(12);
    end
end
