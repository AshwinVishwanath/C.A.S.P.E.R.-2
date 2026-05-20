function [lat_out, lon_out, alt_out, vn_out, ve_out, vd_out, ...
          fix_out, sv_out, data_ready] = casper_gps_latency( ...
    lat_in, lon_in, alt_in, vn_in, ve_in, vd_in, fix_in, sv_in)
%CASPER_GPS_LATENCY One-sample (= 100 ms at 10 Hz) latency FIFO for the GPS stream.
%
% Synopsis:
%   [lat_out, lon_out, alt_out, vn_out, ve_out, vd_out, ...
%    fix_out, sv_out, data_ready] = casper_gps_latency( ...
%       lat_in, lon_in, alt_in, vn_in, ve_in, vd_in, fix_in, sv_in)
%
% Behaviour:
%   - On the first call the buffer is empty.  Outputs are forced to zero /
%     fix_type = 0 / num_sv = 0 and data_ready = false until the buffer has
%     been primed at the next 10 Hz tick.
%   - On every subsequent call (which the build script schedules at 10 Hz)
%     the buffered sample from the *previous* tick is emitted and the
%     current inputs are stored for the next tick.  This yields exactly
%     one 10 Hz sample period (= 100 ms) of latency between input and
%     output.
%
% Inputs / Outputs:
%   All position / velocity fields are int32, fix_type/num_sv are uint8,
%   data_ready is logical.
%
% Source firmware reference:
%   Software/App/drivers/max_m10m.c -- the MAX-M10M datasheet quotes ~100 ms
%   typical end-of-epoch-to-message latency.

%#codegen

    persistent buf_initialized
    persistent b_lat b_lon b_alt b_vn b_ve b_vd b_fix b_sv

    if isempty(buf_initialized)
        buf_initialized = false;
        b_lat = int32(0); b_lon = int32(0); b_alt = int32(0);
        b_vn  = int32(0); b_ve  = int32(0); b_vd  = int32(0);
        b_fix = uint8(0); b_sv  = uint8(0);
    end

    if ~buf_initialized
        % Prime: emit zero-valued / no-fix output, defer current sample.
        lat_out = int32(0);
        lon_out = int32(0);
        alt_out = int32(0);
        vn_out  = int32(0);
        ve_out  = int32(0);
        vd_out  = int32(0);
        fix_out = uint8(0);
        sv_out  = uint8(0);
        data_ready = false;

        b_lat = lat_in; b_lon = lon_in; b_alt = alt_in;
        b_vn  = vn_in;  b_ve  = ve_in;  b_vd  = vd_in;
        b_fix = fix_in; b_sv  = sv_in;
        buf_initialized = true;
        return;
    end

    % Emit the previous sample.
    lat_out = b_lat;
    lon_out = b_lon;
    alt_out = b_alt;
    vn_out  = b_vn;
    ve_out  = b_ve;
    vd_out  = b_vd;
    fix_out = b_fix;
    sv_out  = b_sv;
    data_ready = true;

    % Buffer the current sample for next tick.
    b_lat = lat_in; b_lon = lon_in; b_alt = alt_in;
    b_vn  = vn_in;  b_ve  = ve_in;  b_vd  = vd_in;
    b_fix = fix_in; b_sv  = sv_in;
end
