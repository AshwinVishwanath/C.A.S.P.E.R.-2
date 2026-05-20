function [lat_deg7, lon_deg7, alt_mm, ...
          vel_n_mms, vel_e_mms, vel_d_mms, ...
          fix, sv, data_ready] = casper_gps_quirks( ...
            lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps, ...
            v_total_mps, alt_total_m, time_s, ...
            cocom_vel_thresh_mps, cocom_alt_thresh_m, reacquire_s)
%CASPER_GPS_QUIRKS Firmware quirks wrapper for the MAX-M10M GPS post-stage.
%
% Synopsis:
%   [lat_deg7, lon_deg7, alt_mm, vel_n_mms, vel_e_mms, vel_d_mms, ...
%    fix, sv, data_ready] = casper_gps_quirks( ...
%       lat_deg, lon_deg, alt_m, vn_mps, ve_mps, vd_mps, ...
%       v_total_mps, alt_total_m, time_s, ...
%       cocom_vel_thresh_mps, cocom_alt_thresh_m, reacquire_s)
%
% Purpose:
%   The visual-model T06 GPS subsystem uses gpsSensor (Sensor Fusion Tbx)
%   for the random bits (flat-earth lat/lon conversion + per-axis Gaussian
%   noise). gpsSensor outputs in WGS84 degrees, metres and m/s and has no
%   notion of:
%     - COCOM (Coordinating Committee for Multilateral Export Controls)
%       altitude+velocity export gating
%     - Last-valid hold during gating
%     - 100 ms typical end-of-epoch latency
%     - NAV-PVT int32 (1e-7 deg, mm, mm/s) encoding
%
%   This wrapper covers all of the above as a single pure-ish step. The
%   only state held is for the last-valid latch and the one-sample latency
%   FIFO; both are scoped to this function via persistent variables.
%
% Inputs:
%   lat_deg, lon_deg          (scalar double, deg)  gpsSensor lat / lon output
%   alt_m                     (scalar double, m)    gpsSensor altitude
%   vn_mps, ve_mps, vd_mps    (scalar double, m/s)  gpsSensor NED velocity
%   v_total_mps               (scalar double, m/s)  ||vel_NED||, truth-side
%   alt_total_m               (scalar double, m)    -pos_NED(3), truth-side
%   time_s                    (scalar double, s)    sim time
%   cocom_vel_thresh_mps      (scalar double, m/s)  e.g. 500
%   cocom_alt_thresh_m        (scalar double, m)    e.g. 18000
%   reacquire_s               (scalar double, s)    e.g. 1.0
%
% Outputs (firmware NAV-PVT native units):
%   lat_deg7    int32   latitude  scaled by 1e7 deg
%   lon_deg7    int32   longitude scaled by 1e7 deg
%   alt_mm      int32   altitude in mm above MSL (alt0 + alt_total ~ alt_m)
%   vel_n_mms   int32   N velocity in mm/s
%   vel_e_mms   int32   E velocity in mm/s
%   vel_d_mms   int32   D velocity in mm/s
%   fix         uint8   0 / 2 / 3 (fix type)
%   sv          uint8   satellite count
%   data_ready  logical 10 Hz strobe (false on the very first tick while
%                       the one-sample latency FIFO primes)
%
% Behaviour (T06 §5.3 / §5.4 / §5.5):
%   1. Compute v_total / altitude COCOM gate from truth-side scalars
%      (the gate must NOT be evaluated on the noisy gpsSensor output, since
%      a tail-of-the-distribution noise sample could spuriously toggle it).
%   2. While COCOM active: emit fix=0, sv=0; hold lat/lon/alt/vel at the
%      last sample taken pre-COCOM.
%   3. For 1 reacquire_s second after COCOM exit: fix=2, sv=4.
%   4. Otherwise fix=3, sv=12.
%   5. Quantize the (possibly held) noisy NED output to firmware NAV-PVT
%      int32 units.
%   6. Push through a 1-sample FIFO so the output is 100 ms (= 1/10 s)
%      behind the input. On the very first call data_ready=false.
%
% This function is impure (uses persistent state) and must be called at
% exactly 10 Hz from inside the visual subsystem (which the build script
% guarantees via a Rate Transition to 1/GPS.Rate_Hz on the upstream side).
%
% Source firmware reference:
%   Software/App/drivers/max_m10m.c -- NAV-PVT struct field encoding.
%   Software/App/drivers/max_m10m.h -- gps_fix_type_t {0=NONE, 2=2D, 3=3D}.

%#codegen

    % --- Persistent state ----------------------------------------------------
    persistent cocom_exit_time_s prev_cocom_active
    persistent last_lat_deg last_lon_deg last_alt_m
    persistent last_vn_mps last_ve_mps last_vd_mps
    persistent latched_lastvalid
    persistent buf_initialized
    persistent b_lat_deg7 b_lon_deg7 b_alt_mm
    persistent b_vn b_ve b_vd b_fix b_sv

    if isempty(cocom_exit_time_s)
        cocom_exit_time_s = -Inf;
        prev_cocom_active = false;
    end
    if isempty(latched_lastvalid)
        latched_lastvalid = false;
        last_lat_deg = 0.0; last_lon_deg = 0.0; last_alt_m = 0.0;
        last_vn_mps  = 0.0; last_ve_mps  = 0.0; last_vd_mps  = 0.0;
    end
    if isempty(buf_initialized)
        buf_initialized = false;
        b_lat_deg7 = int32(0); b_lon_deg7 = int32(0); b_alt_mm = int32(0);
        b_vn = int32(0); b_ve = int32(0); b_vd = int32(0);
        b_fix = uint8(0); b_sv = uint8(0);
    end

    % --- 1) COCOM gate (truth-side scalars) ----------------------------------
    cocom_active = (v_total_mps > cocom_vel_thresh_mps) && ...
                   (alt_total_m > cocom_alt_thresh_m);

    if prev_cocom_active && ~cocom_active
        cocom_exit_time_s = time_s;
    end
    prev_cocom_active = cocom_active;

    % --- 2) Fix-type / sat-count schedule ------------------------------------
    if cocom_active
        fix_now = uint8(0);
        sv_now  = uint8(0);
    elseif (time_s - cocom_exit_time_s) < reacquire_s
        fix_now = uint8(2);
        sv_now  = uint8(4);
    else
        fix_now = uint8(3);
        sv_now  = uint8(12);
    end

    % --- 3) Last-valid hold during COCOM -------------------------------------
    if ~cocom_active
        last_lat_deg = lat_deg;
        last_lon_deg = lon_deg;
        last_alt_m   = alt_m;
        last_vn_mps  = vn_mps;
        last_ve_mps  = ve_mps;
        last_vd_mps  = vd_mps;
        latched_lastvalid = true;
        held_lat = lat_deg;
        held_lon = lon_deg;
        held_alt = alt_m;
        held_vn  = vn_mps;
        held_ve  = ve_mps;
        held_vd  = vd_mps;
    else
        held_lat = last_lat_deg;
        held_lon = last_lon_deg;
        held_alt = last_alt_m;
        held_vn  = last_vn_mps;
        held_ve  = last_ve_mps;
        held_vd  = last_vd_mps;
    end

    % --- 4) int32 NAV-PVT encoding -------------------------------------------
    lat_q  = int32(round(held_lat * 1e7));
    lon_q  = int32(round(held_lon * 1e7));
    alt_q  = int32(round(held_alt * 1000));
    vn_q   = int32(round(held_vn  * 1000));
    ve_q   = int32(round(held_ve  * 1000));
    vd_q   = int32(round(held_vd  * 1000));

    % --- 5) 1-sample latency FIFO (= 100 ms at 10 Hz) ------------------------
    if ~buf_initialized
        % Prime cycle: emit zero / no-fix; defer current sample.
        lat_deg7    = int32(0);
        lon_deg7    = int32(0);
        alt_mm      = int32(0);
        vel_n_mms   = int32(0);
        vel_e_mms   = int32(0);
        vel_d_mms   = int32(0);
        fix         = uint8(0);
        sv          = uint8(0);
        data_ready  = false;

        b_lat_deg7 = lat_q; b_lon_deg7 = lon_q; b_alt_mm = alt_q;
        b_vn = vn_q; b_ve = ve_q; b_vd = vd_q;
        b_fix = fix_now; b_sv = sv_now;
        buf_initialized = true;
        return;
    end

    % Emit previous sample.
    lat_deg7   = b_lat_deg7;
    lon_deg7   = b_lon_deg7;
    alt_mm     = b_alt_mm;
    vel_n_mms  = b_vn;
    vel_e_mms  = b_ve;
    vel_d_mms  = b_vd;
    fix        = b_fix;
    sv         = b_sv;
    data_ready = true;

    % Buffer current sample for next tick.
    b_lat_deg7 = lat_q; b_lon_deg7 = lon_q; b_alt_mm = alt_q;
    b_vn = vn_q; b_ve = ve_q; b_vd = vd_q;
    b_fix = fix_now; b_sv = sv_now;
end
