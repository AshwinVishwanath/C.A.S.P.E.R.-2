function test_gps_quirks()
%TEST_GPS_QUIRKS Isolation unit tests for casper_gps_quirks.
%
%   Verifies:
%     T1. int32 NAV-PVT encoding (lat * 1e7, alt * 1000, vel * 1000) is
%         bit-exact for a hand-picked sample (after the FIFO latency primes).
%     T2. data_ready=false on the very first call and true thereafter
%         (one-sample 100 ms latency FIFO).
%     T3. The first valid emission lags the input by exactly one call
%         (100 ms at 10 Hz).
%     T4. COCOM trigger: when v > 500 AND alt > 18000, output fix=0, sv=0
%         and the held position/velocity matches the last pre-COCOM sample.
%     T5. COCOM release + re-acquire schedule: 1 s of fix=2/sv=4 then fix=3/
%         sv=12.
%     T6. Held value during COCOM: a different "noisy" input does not move
%         the output until COCOM releases.
%
%   The function uses persistent state, so each test clears it via the
%   "clear functions" call.

    fprintf('[T06-quirks] tests starting...\n');
    npass = 0; nfail = 0;

    cocom_v = 500.0;
    cocom_a = 18000.0;
    reacq_s = 1.0;
    dt      = 0.1;

    % ===== T1 + T2 + T3: encoding + latency ==================================
    clear casper_gps_quirks;
    lat_in = 51.5074123;
    lon_in = -0.1278456;
    alt_in = 35.789;
    vn_in  = 0.123;
    ve_in  = -0.456;
    vd_in  = 0.789;
    t = 0.0;
    [lat7, lon7, altmm, vn, ve, vd, fix, sv, dr] = casper_gps_quirks( ...
        lat_in, lon_in, alt_in, vn_in, ve_in, vd_in, ...
        0.0, 0.0, t, cocom_v, cocom_a, reacq_s);
    % First call must be the priming cycle: data_ready false, all zero.
    [npass, nfail] = check_scalar(dr, false, 'T2a first-tick data_ready=false', npass, nfail);
    [npass, nfail] = check_scalar(lat7, int32(0), 'T2b first-tick lat=0', npass, nfail);
    [npass, nfail] = check_scalar(fix,  uint8(0), 'T2c first-tick fix=0', npass, nfail);

    % Second call: previous sample emerges, 100 ms later.
    t = t + dt;
    [lat7, lon7, altmm, vn, ve, vd, fix, sv, dr] = casper_gps_quirks( ...
        lat_in, lon_in, alt_in, vn_in, ve_in, vd_in, ...
        0.0, 0.0, t, cocom_v, cocom_a, reacq_s);
    [npass, nfail] = check_scalar(dr, true, 'T3a second-tick data_ready=true', npass, nfail);
    % Encoding check (T1):
    [npass, nfail] = check_scalar(lat7,  int32(round(lat_in * 1e7)),  'T1a lat_deg7 encoding', npass, nfail);
    [npass, nfail] = check_scalar(lon7,  int32(round(lon_in * 1e7)),  'T1b lon_deg7 encoding', npass, nfail);
    [npass, nfail] = check_scalar(altmm, int32(round(alt_in * 1000)), 'T1c alt_mm encoding',   npass, nfail);
    [npass, nfail] = check_scalar(vn,    int32(round(vn_in  * 1000)), 'T1d vel_n_mms encoding', npass, nfail);
    [npass, nfail] = check_scalar(ve,    int32(round(ve_in  * 1000)), 'T1e vel_e_mms encoding', npass, nfail);
    [npass, nfail] = check_scalar(vd,    int32(round(vd_in  * 1000)), 'T1f vel_d_mms encoding', npass, nfail);
    [npass, nfail] = check_scalar(fix,   uint8(3),  'T1g fix=3 (3D) pre-COCOM', npass, nfail);
    [npass, nfail] = check_scalar(sv,    uint8(12), 'T1h sv=12 pre-COCOM', npass, nfail);

    % ===== T4: COCOM trigger =================================================
    clear casper_gps_quirks;
    % Warm-up two calls outside COCOM to prime the FIFO and record a last-valid.
    lat_pre = 51.0; lon_pre = 0.0; alt_pre = 20.0;
    vn_pre  = 1.0; ve_pre  = 2.0; vd_pre  = -10.0;
    [~,~,~,~,~,~,~,~,~] = casper_gps_quirks( ...
        lat_pre, lon_pre, alt_pre, vn_pre, ve_pre, vd_pre, ...
        100.0, 1000.0, 0.0, cocom_v, cocom_a, reacq_s);
    [~,~,~,~,~,~,fix2,sv2,dr2] = casper_gps_quirks( ...
        lat_pre, lon_pre, alt_pre, vn_pre, ve_pre, vd_pre, ...
        100.0, 1000.0, 0.1, cocom_v, cocom_a, reacq_s);
    [npass, nfail] = check_scalar(dr2,  true,     'T4a post-prime data_ready=true', npass, nfail);
    [npass, nfail] = check_scalar(fix2, uint8(3), 'T4b pre-COCOM fix=3',            npass, nfail);

    % Now go into COCOM. Push two ticks so the COCOM sample emerges past the
    % latency FIFO.
    lat_cocom = 60.0; lon_cocom = 5.0; alt_cocom = 25000.0;
    vn_cocom  = 100.0; ve_cocom  = -50.0; vd_cocom  = -600.0;
    [~,~,~,~,~,~,~,~,~] = casper_gps_quirks( ...
        lat_cocom, lon_cocom, alt_cocom, vn_cocom, ve_cocom, vd_cocom, ...
        600.0, 20000.0, 0.2, cocom_v, cocom_a, reacq_s);
    [lat7_c, ~, altmm_c, vn_c, ~, ~, fix_c, sv_c, dr_c] = casper_gps_quirks( ...
        lat_cocom, lon_cocom, alt_cocom, vn_cocom, ve_cocom, vd_cocom, ...
        600.0, 20000.0, 0.3, cocom_v, cocom_a, reacq_s);
    [npass, nfail] = check_scalar(dr_c,    true,                                'T4c during-COCOM data_ready=true', npass, nfail);
    [npass, nfail] = check_scalar(fix_c,   uint8(0),                            'T4d during-COCOM fix=0',            npass, nfail);
    [npass, nfail] = check_scalar(sv_c,    uint8(0),                            'T4e during-COCOM sv=0',             npass, nfail);
    % T6: held lat/alt/vel equals pre-COCOM (lat_pre / alt_pre / vn_pre), NOT
    % the noisy lat_cocom inputs.
    [npass, nfail] = check_scalar(lat7_c,  int32(round(lat_pre * 1e7)),         'T6a held lat (COCOM)',  npass, nfail);
    [npass, nfail] = check_scalar(altmm_c, int32(round(alt_pre * 1000)),        'T6b held alt (COCOM)',  npass, nfail);
    [npass, nfail] = check_scalar(vn_c,    int32(round(vn_pre  * 1000)),        'T6c held vel (COCOM)',  npass, nfail);

    % ===== T5: COCOM release + re-acquire ====================================
    % Now drop out of COCOM and step forward.
    lat_post = 51.5; lon_post = 0.0; alt_post = 1000.0;
    vn_post  = 5.0; ve_post  = 0.0; vd_post  = -20.0;
    t = 0.3;
    % Tick 1 after exit: input goes into FIFO, fix_now=2 (re-acquire), but the
    % output still reflects the COCOM-buffered sample.
    t = t + dt;
    [~,~,~,~,~,~,fix5a,sv5a,~] = casper_gps_quirks( ...
        lat_post, lon_post, alt_post, vn_post, ve_post, vd_post, ...
        100.0, 1000.0, t, cocom_v, cocom_a, reacq_s);
    [npass, nfail] = check_scalar(fix5a, uint8(0), 'T5a immediate post-exit emits prior fix=0', npass, nfail);
    [npass, nfail] = check_scalar(sv5a,  uint8(0), 'T5b immediate post-exit emits prior sv=0',  npass, nfail);

    % Tick 2 after exit: the re-acquire sample now emerges.
    t = t + dt;
    [~,~,~,~,~,~,fix5b,sv5b,~] = casper_gps_quirks( ...
        lat_post, lon_post, alt_post, vn_post, ve_post, vd_post, ...
        100.0, 1000.0, t, cocom_v, cocom_a, reacq_s);
    [npass, nfail] = check_scalar(fix5b, uint8(2), 'T5c re-acquire fix=2',          npass, nfail);
    [npass, nfail] = check_scalar(sv5b,  uint8(4), 'T5d re-acquire sv=4',           npass, nfail);

    % March forward >= reacq_s + small extra and verify return to fix=3 / sv=12.
    for k = 1:15  % 1.5 s of ticks
        t = t + dt;
        [~,~,~,~,~,~,fix_now,sv_now,~] = casper_gps_quirks( ...
            lat_post, lon_post, alt_post, vn_post, ve_post, vd_post, ...
            100.0, 1000.0, t, cocom_v, cocom_a, reacq_s);
    end
    [npass, nfail] = check_scalar(fix_now, uint8(3),  'T5e post-reacquire fix=3 (3D)', npass, nfail);
    [npass, nfail] = check_scalar(sv_now,  uint8(12), 'T5f post-reacquire sv=12',      npass, nfail);

    fprintf('[T06-quirks] %d PASS / %d FAIL\n', npass, nfail);
    if nfail > 0
        error('test_gps_quirks:FAIL', '%d test(s) failed', nfail);
    end
end

function [np, nf] = check_scalar(actual, expected, label, np, nf)
    if isequal(actual, expected)
        fprintf('  PASS %s\n', label);
        np = np + 1;
    else
        fprintf('  FAIL %s  actual=%s expected=%s\n', label, ...
            mat2str(actual), mat2str(expected));
        nf = nf + 1;
    end
end
