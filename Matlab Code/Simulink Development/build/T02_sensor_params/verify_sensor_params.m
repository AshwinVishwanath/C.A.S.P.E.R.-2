function verify_sensor_params()
% verify_sensor_params  Regression check for casper_sensor_params.m
%
% Re-reads the C.A.S.P.E.R.-2 firmware source files at runtime, extracts
% every constant via deterministic regex parsing, and asserts that the
% MATLAB struct value matches the firmware value.
%
% Tolerance: floats compared with abs(a-b) < 1e-9 * max(1, abs(b))
%            integers and arrays compared with same float test.
%
% Usage (from MATLAB or matlab -batch):
%   verify_sensor_params
%
% Throws an error on any mismatch so MATLAB -batch returns nonzero.
% This script is invoked at T02 acceptance and at T11 integration as a
% regression check against firmware drift.

clc;

%% --- Locate firmware sources -------------------------------------------------
this_file = mfilename('fullpath');
this_dir  = fileparts(this_file);
% Climb: build/T02_sensor_params -> build -> Simulink Development -> Matlab Code -> repo root
repo_root = fullfile(this_dir, '..', '..', '..', '..');
repo_root = char(java.io.File(repo_root).getCanonicalPath());

src = struct();
src.ekf_c   = fullfile(repo_root, 'Software', 'App', 'nav',      'casper_ekf.c');
src.ekf_h   = fullfile(repo_root, 'Software', 'App', 'nav',      'casper_ekf.h');
src.att_c   = fullfile(repo_root, 'Software', 'App', 'nav',      'casper_attitude.c');
src.temp_h  = fullfile(repo_root, 'Software', 'App', 'nav',      'temp_cal_coeffs.h');
src.mag_c   = fullfile(repo_root, 'Software', 'App', 'cal',      'mag_cal.c');
src.mag_h   = fullfile(repo_root, 'Software', 'App', 'cal',      'mag_cal.h');
src.lsm_c   = fullfile(repo_root, 'Software', 'App', 'drivers',  'lsm6dso32.c');
src.mmc_h   = fullfile(repo_root, 'Software', 'App', 'drivers',  'mmc5983ma.h');
src.gps_c   = fullfile(repo_root, 'Software', 'App', 'drivers',  'max_m10m.c');
src.radio_h = fullfile(repo_root, 'Software', 'App', 'radio',    'radio_config.h');
src.radio_c = fullfile(repo_root, 'Software', 'App', 'radio',    'radio_config.c');
src.fsm_h   = fullfile(repo_root, 'Software', 'App', 'fsm',      'fsm_types.h');
src.tlm_h   = fullfile(repo_root, 'Software', 'App', 'telemetry','tlm_types.h');
src.main_c  = fullfile(repo_root, 'Software', 'Core',  'Src',    'main.c');

fns = fieldnames(src);
for i = 1:numel(fns)
    if ~exist(src.(fns{i}), 'file')
        error('verify_sensor_params:missing_source', ...
            'Firmware source not found: %s', src.(fns{i}));
    end
end

%% --- Load parameter file into local workspace -------------------------------
% Evaluate the parameter script in this function's workspace.
% (run() preserves script semantics: variables created at function scope.)
param_script = fullfile(this_dir, 'casper_sensor_params.m');
run(param_script);

%% --- Tracker ----------------------------------------------------------------
results = struct('name', {}, 'pass', {}, 'fw_value', {}, 'mat_value', {}, 'note', {});

%% --- EKF defines (casper_ekf.c) ---------------------------------------------
fprintf('\n[EKF defines]\n');
results(end+1) = record_match('G_ACCEL',             grep_define(src.ekf_c, 'G_ACCEL'),             Estimator.G);
results(end+1) = record_match('EKF_DT',              grep_define(src.ekf_c, 'EKF_DT'),              Estimator.Dt);
results(end+1) = record_match('P0_ALT',              grep_define(src.ekf_c, 'P0_ALT'),              Estimator.P0_Alt);
results(end+1) = record_match('P0_VEL',              grep_define(src.ekf_c, 'P0_VEL'),              Estimator.P0_Vel);
results(end+1) = record_match('P0_ACCEL_BIAS',       grep_define(src.ekf_c, 'P0_ACCEL_BIAS'),       Estimator.P0_AccelBias);
results(end+1) = record_match('P0_BARO_BIAS',        grep_define(src.ekf_c, 'P0_BARO_BIAS'),        Estimator.P0_BaroBias);
results(end+1) = record_match('ACCEL_VRW',           grep_define(src.ekf_c, 'ACCEL_VRW'),           Estimator.AccelVRW);
results(end+1) = record_match('ACCEL_BI_SIGMA',      grep_define(src.ekf_c, 'ACCEL_BI_SIGMA'),      Estimator.AccelBiSigma);
results(end+1) = record_match('BARO_BI_SIGMA',       grep_define(src.ekf_c, 'BARO_BI_SIGMA'),       Estimator.BaroBiSigma);
results(end+1) = record_match('R_BARO',              grep_define(src.ekf_c, 'R_BARO'),              Estimator.R_Baro);
results(end+1) = record_match('R_ZUPT',              grep_define(src.ekf_c, 'R_ZUPT'),              Estimator.R_Zupt);
results(end+1) = record_match('BARO_GATE_K2',        grep_define(src.ekf_c, 'BARO_GATE_K2'),        Estimator.BaroGateK2);
results(end+1) = record_match('P_FLOOR_BARO_BIAS',   grep_define(src.ekf_c, 'P_FLOOR_BARO_BIAS'),   Estimator.PFloorBaroBias);
results(end+1) = record_match('MACH_GATE_ON',        grep_define(src.ekf_c, 'MACH_GATE_ON'),        Estimator.MachGateOn);
results(end+1) = record_match('MACH_GATE_OFF',       grep_define(src.ekf_c, 'MACH_GATE_OFF'),       Estimator.MachGateOff);
results(end+1) = record_match('R_BARO_UNGATE',       grep_define(src.ekf_c, 'R_BARO_UNGATE'),       Estimator.R_BaroUngate);
results(end+1) = record_match('N_UNGATE_STEPS',      grep_define(src.ekf_c, 'N_UNGATE_STEPS'),      Estimator.N_UngateSteps);
results(end+1) = record_match('P_UNGATE_ACCEL_BIAS', grep_define(src.ekf_c, 'P_UNGATE_ACCEL_BIAS'), Estimator.P_UngateAccelBias);
results(end+1) = record_match('P_UNGATE_BARO_BIAS',  grep_define(src.ekf_c, 'P_UNGATE_BARO_BIAS'),  Estimator.P_UngateBaroBias);
results(end+1) = record_match('EKF_ZUPT_THRESHOLD',  grep_define(src.ekf_h, 'EKF_ZUPT_THRESHOLD'),  Estimator.ZuptThreshold);

%% --- Attitude estimator ------------------------------------------------------
fprintf('\n[Attitude estimator (casper_attitude.c)]\n');
results(end+1) = record_match('STATIC_INIT_MAG_SAMPLES', ...
    grep_define(src.att_c, 'STATIC_INIT_MAG_SAMPLES'), Attitude.StaticInitSamples);
results(end+1) = record_match('STATIC_INIT_TIMEOUT_S', ...
    grep_define(src.att_c, 'STATIC_INIT_TIMEOUT_S'), Attitude.StaticInitTimeout_s);
results(end+1) = record_match('HEADING_SIGMA_FLOOR', ...
    grep_define(src.att_c, 'HEADING_SIGMA_FLOOR'), Attitude.HeadingSigmaFloor_rad);
results(end+1) = record_match('BIAS_GYRO_THRESH', ...
    grep_define(src.att_c, 'BIAS_GYRO_THRESH'), Attitude.BiasGyroThresh_radps);
results(end+1) = record_match('BIAS_EMA_INV_TAU', ...
    grep_define(src.att_c, 'BIAS_EMA_INV_TAU'), Attitude.BiasEmaInvTau_perS);

% Gyro ARW values are assigned inline in casper_att_init() — grep each index.
att_src = fileread(src.att_c);
arw = zeros(3, 1);
for k = 0:2
    pat = sprintf('gyro_arw\\[%d\\]\\s*=\\s*([-+]?[\\d.eE+\\-]+)f?\\s*;', k);
    tok = regexp(att_src, pat, 'tokens', 'once');
    if isempty(tok)
        error('Could not parse gyro_arw[%d] in casper_attitude.c', k);
    end
    arw(k + 1) = str2double(tok{1});
end
results(end+1) = record_match('gyro_arw[0]', arw(1), Attitude.GyroArw_radSqrtS(1));
results(end+1) = record_match('gyro_arw[1]', arw(2), Attitude.GyroArw_radSqrtS(2));
results(end+1) = record_match('gyro_arw[2]', arw(3), Attitude.GyroArw_radSqrtS(3));

% Mahony gains — firmware canonical: main.c att_cfg initializer
fprintf('\n[Attitude config (main.c att_cfg initializer)]\n');
note_dev = '(deviates from FIRMWARE_CONSTANTS.md typical defaults)';
results(end+1) = record_match('att_cfg.Kp_grav', ...
    grep_struct_field(src.main_c, 'casper_att_config_t', 'Kp_grav'), ...
    Attitude.Kp_Grav, note_dev);
results(end+1) = record_match('att_cfg.Kp_mag_pad', ...
    grep_struct_field(src.main_c, 'casper_att_config_t', 'Kp_mag_pad'), ...
    Attitude.Kp_MagPad, note_dev);
results(end+1) = record_match('att_cfg.Kp_mag_flight', ...
    grep_struct_field(src.main_c, 'casper_att_config_t', 'Kp_mag_flight'), ...
    Attitude.Kp_MagFlight, note_dev);
results(end+1) = record_match('att_cfg.Ki', ...
    grep_struct_field(src.main_c, 'casper_att_config_t', 'Ki'), ...
    Attitude.Ki, note_dev);
results(end+1) = record_match('att_cfg.gyro_lpf_cutoff_hz', ...
    grep_struct_field(src.main_c, 'casper_att_config_t', 'gyro_lpf_cutoff_hz'), ...
    Attitude.GyroLpfCutoff_Hz);
results(end+1) = record_match('att_cfg.mag_update_hz', ...
    grep_struct_field(src.main_c, 'casper_att_config_t', 'mag_update_hz'), ...
    Attitude.MagUpdateRate_Hz);
results(end+1) = record_match('att_cfg.launch_accel_g', ...
    grep_struct_field(src.main_c, 'casper_att_config_t', 'launch_accel_g'), ...
    Attitude.LaunchAccel_g);

%% --- Gyro temperature coeffs ------------------------------------------------
fprintf('\n[Gyro temperature coeffs (temp_cal_coeffs.h)]\n');
results(end+1) = record_match('GYRO_TC_T0',      grep_define(src.temp_h, 'GYRO_TC_T0'),      GyroTempCal.T0_C);
results(end+1) = record_match('GYRO_TC_SLOPE_X', grep_define(src.temp_h, 'GYRO_TC_SLOPE_X'), GyroTempCal.Slope_X);
results(end+1) = record_match('GYRO_TC_SLOPE_Y', grep_define(src.temp_h, 'GYRO_TC_SLOPE_Y'), GyroTempCal.Slope_Y);
results(end+1) = record_match('GYRO_TC_SLOPE_Z', grep_define(src.temp_h, 'GYRO_TC_SLOPE_Z'), GyroTempCal.Slope_Z);

%% --- Mag calibration --------------------------------------------------------
fprintf('\n[Mag calibration (mag_cal.c arrays + mag_cal.h)]\n');
hard_iron_fw = grep_array_3f(src.mag_c, 'mag_hard_iron');
soft_iron_fw = grep_array_33f(src.mag_c, 'mag_soft_iron');
expected_mag_fw = grep_define(src.mag_h, 'MAG_CAL_EXPECTED_MAG');

results(end+1) = record_match('mag_hard_iron[0]', hard_iron_fw(1), Mag.HardIron_uT(1));
results(end+1) = record_match('mag_hard_iron[1]', hard_iron_fw(2), Mag.HardIron_uT(2));
results(end+1) = record_match('mag_hard_iron[2]', hard_iron_fw(3), Mag.HardIron_uT(3));
for r = 1:3
    for c = 1:3
        nm = sprintf('mag_soft_iron[%d][%d]', r-1, c-1);
        results(end+1) = record_match(nm, soft_iron_fw(r, c), Mag.SoftIron(r, c)); %#ok<AGROW>
    end
end
results(end+1) = record_match('MAG_CAL_EXPECTED_MAG', expected_mag_fw, Mag.ExpectedMag_uT);

%% --- IMU driver -------------------------------------------------------------
fprintf('\n[IMU driver constants (lsm6dso32.c)]\n');
lsm_txt = fileread(src.lsm_c);

tok = regexp(lsm_txt, 'accel_g\[0\][^;]*?\*\s*([-+]?[\d.eE+\-]+)f', 'tokens', 'once');
if isempty(tok), error('Could not parse accel scale from lsm6dso32.c'); end
results(end+1) = record_match('accel_scale_gPerLSB', str2double(tok{1}), IMU.AccelScale_gPerLSB);

tok = regexp(lsm_txt, 'gyro_dps\[0\][^;]*?\*\s*([-+]?[\d.eE+\-]+)f', 'tokens', 'once');
if isempty(tok), error('Could not parse gyro scale from lsm6dso32.c'); end
results(end+1) = record_match('gyro_scale_dpsPerLSB', str2double(tok{1}), IMU.GyroScale_dpsPerLSB);

tok = regexp(lsm_txt, 'raw_temp\s*/\s*([\d.]+)f', 'tokens', 'once');
if isempty(tok), error('Could not parse temp scale from lsm6dso32.c'); end
results(end+1) = record_match('temp_scale_LSBperC', str2double(tok{1}), IMU.TempScale_LSBperC);

tok = regexp(lsm_txt, 'raw_temp\s*/\s*[\d.]+f\s*\+\s*([\d.]+)f', 'tokens', 'once');
if isempty(tok), error('Could not parse temp offset from lsm6dso32.c'); end
results(end+1) = record_match('temp_offset_C', str2double(tok{1}), IMU.TempOffset_C);

% CTRL1_XL = 0x74 implies ODR=833 and FS=+/-32g
tok = regexp(lsm_txt, 'CTRL1_XL,\s*0x([0-9A-Fa-f]+)', 'tokens', 'once');
if isempty(tok), error('Could not parse CTRL1_XL value'); end
ctrl1_xl = hex2dec(tok{1});
odr_bits = bitshift(ctrl1_xl, -4);
fs_bits  = bitshift(bitand(ctrl1_xl, hex2dec('0C')), -2);
% LSM6DSO32 ODR bits: 0=PD, 1=12.5, 2=26, 3=52, 4=104, 5=208, 6=416, 7=833,
%                     8=1666, 9=3332, 10=6664
odr_table = [0, 12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664];
imu_rate_fw = odr_table(odr_bits + 1);
results(end+1) = record_match('IMU.Rate_Hz (CTRL1_XL[7:4])', imu_rate_fw, IMU.Rate_Hz);
% LSM6DSO32 accel FS bits: 00=4g, 01=32g, 10=8g, 11=16g
fs_table = [4, 32, 8, 16];
accel_range_fw = fs_table(fs_bits + 1);
results(end+1) = record_match('IMU.AccelRange_g (CTRL1_XL[3:2])', accel_range_fw, IMU.AccelRange_g);

% CTRL2_G = 0x7C => ODR=833, FS=+/-2000dps
tok = regexp(lsm_txt, 'CTRL2_G,\s*0x([0-9A-Fa-f]+)', 'tokens', 'once');
if isempty(tok), error('Could not parse CTRL2_G value'); end
ctrl2_g = hex2dec(tok{1});
odr_bits  = bitshift(ctrl2_g, -4);
fs_g_bits = bitshift(bitand(ctrl2_g, hex2dec('0E')), -1);
gyro_rate_fw = odr_table(odr_bits + 1);
results(end+1) = record_match('IMU.Rate_Hz (CTRL2_G[7:4])', gyro_rate_fw, IMU.Rate_Hz);
% LSM6DSO32 gyro FS bits[3:1]: 000=250, 001=125, 010=500, 100=1000, 110=2000
gyro_fs_keys = [0, 1, 2, 4, 6];
gyro_fs_vals = [250, 125, 500, 1000, 2000];
idx = find(gyro_fs_keys == fs_g_bits, 1);
if isempty(idx), error('Unknown gyro FS bits %d', fs_g_bits); end
gyro_range_fw = gyro_fs_vals(idx);
results(end+1) = record_match('IMU.GyroRange_dps (CTRL2_G[3:1])', gyro_range_fw, IMU.GyroRange_dps);

%% --- MMC5983MA driver -------------------------------------------------------
fprintf('\n[Mag driver constants (mmc5983ma.h)]\n');
results(end+1) = record_match('MMC5983MA_18BIT_OFFSET', ...
    grep_define(src.mmc_h, 'MMC5983MA_18BIT_OFFSET'), Mag.OffsetCounts);
results(end+1) = record_match('MMC5983MA_18BIT_SCALE', ...
    grep_define(src.mmc_h, 'MMC5983MA_18BIT_SCALE'), Mag.ScaleCountsPerGauss);

%% --- Radio ------------------------------------------------------------------
fprintf('\n[Radio config]\n');
results(end+1) = record_match('RADIO_TX_PERIOD_MS (->s)', ...
    grep_define(src.radio_h, 'RADIO_TX_PERIOD_MS') / 1000, Radio.TX_Period_s);
results(end+1) = record_match('RADIO_TX_TIMEOUT_MS (->s)', ...
    grep_define(src.radio_h, 'RADIO_TX_TIMEOUT_MS') / 1000, Radio.TX_Timeout_s);

radio_txt = fileread(src.radio_c);
pa_block = extract_initializer_block(radio_txt, 'RADIO_PROFILE_A');
pb_block = extract_initializer_block(radio_txt, 'RADIO_PROFILE_B');

results(end+1) = record_match('PROFILE_A.sf',           field_in_block(pa_block, 'sf'),           Radio.ProfileA_SF);
results(end+1) = record_match('PROFILE_A.bw_hz',        field_in_block(pa_block, 'bw_hz'),        Radio.ProfileA_BW_Hz);
results(end+1) = record_match('PROFILE_A.cr',           field_in_block(pa_block, 'cr'),           Radio.ProfileA_CR);
results(end+1) = record_match('PROFILE_A.freq_hz',      field_in_block(pa_block, 'freq_hz'),      Radio.Frequency_Hz);
results(end+1) = record_match('PROFILE_A.tx_power_dbm', field_in_block(pa_block, 'tx_power_dbm'), Radio.TXPower_dBm);
results(end+1) = record_match('PROFILE_A.preamble',     field_in_block(pa_block, 'preamble'),     Radio.Preamble_Symbols);
results(end+1) = record_match('PROFILE_A.sync_word',    field_in_block(pa_block, 'sync_word'),    double(Radio.SyncWord));
results(end+1) = record_match('PROFILE_B.sf',           field_in_block(pb_block, 'sf'),           Radio.ProfileB_SF);
results(end+1) = record_match('PROFILE_B.bw_hz',        field_in_block(pb_block, 'bw_hz'),        Radio.ProfileB_BW_Hz);

% SF7 airtime sanity (2^SF/BW * ~30 symbols ~ 15 ms).
% Tolerance: ±1 ms; airtime depends on payload length + overhead symbols.
sym_time = (2^Radio.ProfileA_SF) / Radio.ProfileA_BW_Hz;
airtime_est = sym_time * 30;
ok = abs(airtime_est - Radio.ProfileA_Airtime_s) < 1.0e-3;
results(end+1) = struct('name', 'ProfileA airtime (2^SF/BW*30, ±1ms)', ...
    'pass', ok, 'fw_value', airtime_est, ...
    'mat_value', Radio.ProfileA_Airtime_s, 'note', 'sanity (1ms tol)');
if ok
    fprintf('  PASS  %-38s fw=%s\n', 'ProfileA airtime (2^SF/BW*30, ±1ms)', ...
        fmt_val(airtime_est));
else
    fprintf('  FAIL  %-38s fw=%s mat=%s\n', 'ProfileA airtime (2^SF/BW*30, ±1ms)', ...
        fmt_val(airtime_est), fmt_val(Radio.ProfileA_Airtime_s));
end

%% --- FSM thresholds ---------------------------------------------------------
fprintf('\n[FSM thresholds (fsm_types.h)]\n');
results(end+1) = record_match('FSM_LAUNCH_ACCEL_G',        grep_define(src.fsm_h, 'FSM_LAUNCH_ACCEL_G'),        FSM.LaunchAccel_g);
results(end+1) = record_match('FSM_LAUNCH_ACCEL_DWELL_MS', grep_define(src.fsm_h, 'FSM_LAUNCH_ACCEL_DWELL_MS'), FSM.LaunchAccelDwell_ms);
results(end+1) = record_match('FSM_LAUNCH_VEL_MPS',        grep_define(src.fsm_h, 'FSM_LAUNCH_VEL_MPS'),        FSM.LaunchVel_mps);
results(end+1) = record_match('FSM_BURNOUT_ACCEL_G',       grep_define(src.fsm_h, 'FSM_BURNOUT_ACCEL_G'),       FSM.BurnoutAccel_g);
results(end+1) = record_match('FSM_BURNOUT_DWELL_MS',      grep_define(src.fsm_h, 'FSM_BURNOUT_DWELL_MS'),      FSM.BurnoutDwell_ms);
results(end+1) = record_match('FSM_SUSTAIN_ACCEL_G',       grep_define(src.fsm_h, 'FSM_SUSTAIN_ACCEL_G'),       FSM.SustainAccel_g);
results(end+1) = record_match('FSM_SUSTAIN_DWELL_MS',      grep_define(src.fsm_h, 'FSM_SUSTAIN_DWELL_MS'),      FSM.SustainDwell_ms);
results(end+1) = record_match('FSM_APOGEE_VEL_MPS',        grep_define(src.fsm_h, 'FSM_APOGEE_VEL_MPS'),        FSM.ApogeeVel_mps);
results(end+1) = record_match('FSM_APOGEE_VEL_DWELL_MS',   grep_define(src.fsm_h, 'FSM_APOGEE_VEL_DWELL_MS'),   FSM.ApogeeVelDwell_ms);
results(end+1) = record_match('FSM_APOGEE_MIN_FLIGHT_S',   grep_define(src.fsm_h, 'FSM_APOGEE_MIN_FLIGHT_S'),   FSM.ApogeeMinFlight_s);
results(end+1) = record_match('FSM_LANDED_VEL_MPS',        grep_define(src.fsm_h, 'FSM_LANDED_VEL_MPS'),        FSM.LandedVel_mps);
results(end+1) = record_match('FSM_LANDED_ALT_DELTA_M',    grep_define(src.fsm_h, 'FSM_LANDED_ALT_DELTA_M'),    FSM.LandedAltDelta_m);
results(end+1) = record_match('FSM_LANDED_DWELL_MS',       grep_define(src.fsm_h, 'FSM_LANDED_DWELL_MS'),       FSM.LandedDwell_ms);

%% --- Telemetry scales -------------------------------------------------------
fprintf('\n[Telemetry scales (tlm_types.h)]\n');
results(end+1) = record_match('ALT_SCALE_M',      grep_define(src.tlm_h, 'ALT_SCALE_M'),      Telemetry.AltScale_m);
results(end+1) = record_match('VEL_SCALE_DMS',    grep_define(src.tlm_h, 'VEL_SCALE_DMS'),    Telemetry.VelScale_dms);
results(end+1) = record_match('TIME_SCALE_100MS', grep_define(src.tlm_h, 'TIME_SCALE_100MS'), Telemetry.TimeScale_100ms);
results(end+1) = record_match('BATT_OFFSET_V',    grep_define(src.tlm_h, 'BATT_OFFSET_V'),    Telemetry.BattOffset_V);
results(end+1) = record_match('BATT_STEP_V',      grep_define(src.tlm_h, 'BATT_STEP_V'),      Telemetry.BattStep_V);

%% --- GPS rate (CFG_RATE_MEAS) -----------------------------------------------
fprintf('\n[GPS config (max_m10m.c)]\n');
gps_txt = fileread(src.gps_c);
tok = regexp(gps_txt, 'CFG_RATE_MEAS\s*,\s*(\d+)\s*,', 'tokens', 'once');
if isempty(tok), error('Could not parse CFG_RATE_MEAS from max_m10m.c'); end
meas_rate_ms = str2double(tok{1});
gps_rate_fw = 1000 / meas_rate_ms;
results(end+1) = record_match('GPS.Rate_Hz (1000/CFG_RATE_MEAS)', gps_rate_fw, GPS.Rate_Hz);

%% --- Summary ----------------------------------------------------------------
n_total = numel(results);
n_pass  = sum([results.pass]);
n_fail  = n_total - n_pass;

fprintf('\n========================================\n');
fprintf('  T02 verify_sensor_params summary\n');
fprintf('  Total: %d   Pass: %d   Fail: %d\n', n_total, n_pass, n_fail);
fprintf('========================================\n');

if n_fail > 0
    fails = results(~[results.pass]);
    for i = 1:numel(fails)
        fprintf('  FAILED: %s  fw=%s  mat=%s  %s\n', fails(i).name, ...
            fmt_val(fails(i).fw_value), fmt_val(fails(i).mat_value), fails(i).note);
    end
    error('verify_sensor_params:mismatch', ...
        'verify_sensor_params: %d of %d constants mismatched firmware.', n_fail, n_total);
end

% Expose counts via assignin for STATUS.md generator (callers like a wrapper script)
try
    assignin('base', 'verify_n_total', n_total);
    assignin('base', 'verify_n_pass',  n_pass);
    assignin('base', 'verify_n_fail',  n_fail);
catch
    % base workspace may not exist when running -batch as function — ignore
end

fprintf('\n[T02] verify_sensor_params: PASS (%d/%d)\n', n_pass, n_total);

end % verify_sensor_params

% ============================================================================
% Local helper functions
% ============================================================================

function r = record_match(name, fw_value, mat_value, note)
    if nargin < 4, note = ''; end
    if isnumeric(fw_value) && isnumeric(mat_value) ...
            && all(size(fw_value) == size(mat_value))
        denom = max(1, max(abs(fw_value(:))));
        ok = all(abs(fw_value(:) - mat_value(:)) < 1e-9 * denom);
    else
        ok = isequal(fw_value, mat_value);
    end
    r = struct('name', name, 'pass', ok, ...
               'fw_value', fw_value, 'mat_value', mat_value, 'note', note);
    if ok
        fprintf('  PASS  %-38s fw=%s\n', name, fmt_val(fw_value));
    else
        fprintf('  FAIL  %-38s fw=%s  mat=%s  %s\n', name, ...
            fmt_val(fw_value), fmt_val(mat_value), note);
    end
end

function s = fmt_val(v)
    if isnumeric(v)
        if isscalar(v)
            s = sprintf('%.10g', v);
        else
            s = mat2str(v, 6);
        end
    elseif ischar(v) || isstring(v)
        s = char(v);
    else
        s = '<?>';
    end
end

function val = grep_define(file_path, name)
    txt = fileread(file_path);
    pat = ['#define\s+' regexptranslate('escape', name) ...
           '\s+\(?\s*([-+]?[\d.eE+\-]+)f?\s*\)?'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        error('verify_sensor_params:parse', ...
            'Could not parse #define %s in %s', name, file_path);
    end
    val = str2double(tok{1});
    if isnan(val)
        error('verify_sensor_params:parse', ...
            'Could not convert "%s" to number for %s', tok{1}, name);
    end
end

function arr = grep_array_3f(file_path, name)
    txt = fileread(file_path);
    pat = [regexptranslate('escape', name) ...
           '\s*\[\s*3\s*\]\s*=\s*\{\s*' ...
           '([-+]?[\d.eE+\-]+)f?\s*,\s*' ...
           '([-+]?[\d.eE+\-]+)f?\s*,\s*' ...
           '([-+]?[\d.eE+\-]+)f?\s*\}'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        error('Could not parse 3-element array "%s" in %s', name, file_path);
    end
    arr = [str2double(tok{1}); str2double(tok{2}); str2double(tok{3})];
end

function M = grep_array_33f(file_path, name)
    txt = fileread(file_path);
    num = '\s*([-+]?[\d.eE+\-]+)f?\s*';
    row = ['\{' num ',' num ',' num '\}'];
    pat = [regexptranslate('escape', name) ...
           '\s*\[\s*3\s*\]\s*\[\s*3\s*\]\s*=\s*\{\s*' ...
           row '\s*,\s*' row '\s*,\s*' row '\s*\}'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        error('Could not parse 3x3 matrix "%s" in %s', name, file_path);
    end
    v = cellfun(@str2double, tok);
    M = reshape(v, [3, 3]).';  % row-major capture
end

function val = grep_struct_field(file_path, struct_name, field_name)
    % Find an initializer block "struct_name <var> = { ... }" and pull
    % ".field_name = VALUE" from inside it.
    txt = fileread(file_path);
    start_pat = [regexptranslate('escape', struct_name) '\s*\w*\s*=\s*\{'];
    start_idx = regexp(txt, start_pat, 'once');
    if isempty(start_idx)
        error('Initializer "%s = {" not found in %s', struct_name, file_path);
    end
    % Find the opening brace location after the match
    open_idx = start_idx + find(txt(start_idx:end) == '{', 1) - 1;
    depth = 0;
    close_idx = 0;
    for k = open_idx:length(txt)
        if txt(k) == '{'
            depth = depth + 1;
        elseif txt(k) == '}'
            depth = depth - 1;
            if depth == 0
                close_idx = k;
                break;
            end
        end
    end
    if close_idx == 0
        error('No matching brace for "%s" in %s', struct_name, file_path);
    end
    block = txt(open_idx:close_idx);
    pat = ['\.' regexptranslate('escape', field_name) ...
           '\s*=\s*([-+]?[\d.eE+\-]+)f?'];
    tok = regexp(block, pat, 'tokens', 'once');
    if isempty(tok)
        error('Field .%s not found in %s initializer', field_name, file_path);
    end
    val = str2double(tok{1});
end

function block = extract_initializer_block(txt, var_name)
    start_pat = [regexptranslate('escape', var_name) '\s*=\s*\{'];
    start_idx = regexp(txt, start_pat, 'once');
    if isempty(start_idx)
        error('Initializer "%s = {" not found', var_name);
    end
    open_idx = start_idx + find(txt(start_idx:end) == '{', 1) - 1;
    depth = 0;
    close_idx = 0;
    for k = open_idx:length(txt)
        if txt(k) == '{'
            depth = depth + 1;
        elseif txt(k) == '}'
            depth = depth - 1;
            if depth == 0
                close_idx = k;
                break;
            end
        end
    end
    if close_idx == 0
        error('No matching brace for initializer of %s', var_name);
    end
    block = txt(open_idx:close_idx);
end

function v = field_in_block(block, fname)
    pat = ['\.' regexptranslate('escape', fname) ...
           '\s*=\s*([-+]?[0-9xXa-fA-F.eE+\-]+)'];
    tok = regexp(block, pat, 'tokens', 'once');
    if isempty(tok)
        error('field %s not found in block', fname);
    end
    s = tok{1};
    if length(s) > 2 && (strcmpi(s(1:2), '0x'))
        v = hex2dec(s(3:end));
    else
        v = str2double(s);
    end
end
