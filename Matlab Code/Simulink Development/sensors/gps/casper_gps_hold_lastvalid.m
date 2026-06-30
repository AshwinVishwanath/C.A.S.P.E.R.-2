function [lat_out, lon_out, alt_out, vn_out, ve_out, vd_out] = ...
    casper_gps_hold_lastvalid(cocom_active, ...
        lat_in, lon_in, alt_in, vn_in, ve_in, vd_in)
%CASPER_GPS_HOLD_LASTVALID Pass-through when COCOM is inactive; hold last valid otherwise.
%
% Synopsis:
%   [lat_out, lon_out, alt_out, vn_out, ve_out, vd_out] = ...
%       casper_gps_hold_lastvalid(cocom_active, ...
%           lat_in, lon_in, alt_in, vn_in, ve_in, vd_in)
%
% Behaviour (T06 §5.3 step 3):
%   - If cocom_active is false, pass inputs straight through and latch them
%     as the most-recent-valid sample.
%   - If cocom_active is true, ignore inputs and emit the latched values
%     from the last call when cocom was not active.
%   - Before any non-COCOM sample has been seen, holds zeros.
%
% Inputs / Outputs:
%   cocom_active : logical scalar
%   lat/lon/alt/vn/ve/vd : int32 scalars (NAV-PVT native units)
%
% Source firmware reference:
%   None (sim-side gating helper for the spec's "hold last-valid" rule).

%#codegen

    persistent last_lat last_lon last_alt last_vn last_ve last_vd
    if isempty(last_lat)
        last_lat = int32(0); last_lon = int32(0); last_alt = int32(0);
        last_vn  = int32(0); last_ve  = int32(0); last_vd  = int32(0);
    end

    if ~cocom_active
        last_lat = lat_in; last_lon = lon_in; last_alt = alt_in;
        last_vn  = vn_in;  last_ve  = ve_in;  last_vd  = vd_in;
    end

    lat_out = last_lat;
    lon_out = last_lon;
    alt_out = last_alt;
    vn_out  = last_vn;
    ve_out  = last_ve;
    vd_out  = last_vd;
end
