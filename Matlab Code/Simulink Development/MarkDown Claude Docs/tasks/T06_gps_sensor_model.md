# T06 — MAX-M10M GPS Sensor Model

## 1. Goal

Build a Simulink subsystem that simulates the u-blox MAX-M10M GPS module's output: 10 Hz NAV-PVT messages with realistic position, velocity, fix-type, and satellite count. Include COCOM dropout (simultaneous high-speed + high-altitude) and basic latency modeling.

GPS is **de-emphasized** in Phase 0 — the EKF runs without GPS updates by default. The Phase 0 sim still produces realistic GPS output for completeness and to enable Phase 1+ when GPS becomes useful (e.g., post-apogee descent for recovery).

## 2. Inputs

| Input | Source |
|---|---|
| Truth bus signal | T01 (pos_NED, vel_NED, mach) |
| Sensor parameters | T02 `GPS` struct |
| Random seed | `Sim.Seed + 5` |
| Sample rate | `GPS.Rate_Hz = 10` Hz |

## 3. Outputs

In `Software/Sim/build/T06_gps_sensor_model/`:

| File | Purpose |
|---|---|
| `casper_gps_position_model.m` | MATLAB Function: position + velocity from truth, with noise |
| `casper_gps_cocom_check.m` | MATLAB Function: COCOM gating logic |
| `casper_gps_latency.m` | MATLAB Function: 100 ms latency buffer |
| `build_gps_block.m` | Simulink build script |
| `test_gps_model.m` | Unit test |
| `gps_block.slx` | Library subsystem |
| `plots/gps_pad_30s.png` | 30 s of pad-mode GPS noise |
| `plots/gps_cocom_window.png` | Flight altitude/velocity vs. COCOM gate |
| `STATUS.md` | Result summary |

## 4. Signal flow

```
truth bus  ─►  Rate Transition (10 kHz → 10 Hz)  ─►  cocom_check
                                                          │
                                                          ▼
                                                     gps_position_model
                                                       (only emits when not dropped out)
                                                          │
                                                          ▼
                                                     gps_latency
                                                       (100 ms delay buffer)
                                                          │
                                                          ▼
                                                     gps_block out:
                                                       lat_deg7 (int32, 1e-7 deg)
                                                       lon_deg7 (int32, 1e-7 deg)
                                                       alt_msl_mm (int32, mm above MSL)
                                                       vel_n_mm_s (int32, mm/s)
                                                       vel_e_mm_s (int32, mm/s)
                                                       vel_d_mm_s (int32, mm/s)
                                                       fix_type (uint8)
                                                       num_sv (uint8)
                                                       data_ready (bool)
```

## 5. Model spec

### 5.1 Position from truth

Inputs: `pos_NED` (3×1, m, Z down).

For Phase 0, **assume the launch site is fixed at a known lat/lon**:
- Default: `lat0 = 51.5074` deg, `lon0 = -0.1278` deg, `alt0 = 35` m (London — placeholder; configurable via `GPS.LaunchLat_deg`, `GPS.LaunchLon_deg`, `GPS.LaunchAlt_m`).
- Truth `pos_NED` is launch-relative meters.
- Convert to lat/lon/alt:
  - `dlat_rad = pos_NED(1) / 6371000`  (North component → latitude change)
  - `dlon_rad = pos_NED(2) / (6371000 * cos(lat0_rad))`  (East component → longitude change)
  - `alt_msl_m = alt0 - pos_NED(3)`  (NED Z is down; subtract for altitude above MSL)
- Convert to GPS native units (1e-7 deg, mm):
  - `lat_deg7 = round((lat0_deg + dlat_rad * 180/pi) * 1e7)`
  - `lon_deg7 = round((lon0_deg + dlon_rad * 180/pi) * 1e7)`
  - `alt_msl_mm = round(alt_msl_m * 1000)`

Phase 0 trajectory is vertical (zero horizontal), so `dlat = dlon = 0`; only `alt_msl_mm` varies.

### 5.2 Noise

Apply per sample:

1. **Position horizontal noise**: per-axis (N, E) `N(0, GPS.PositionCEP_Horizontal_m / sqrt(2))`. Default CEP = 1.5 m → per-axis σ ≈ 1.06 m. Convert to lat/lon deg-7 units.
2. **Position vertical noise**: vertical noise ≈ `1.5 * horizontal` σ. So `σ_v ≈ 1.6 m` in altitude.
3. **Velocity noise**: per-axis (N, E, D) `N(0, GPS.VelocityNoise_mps)`. Default σ = 0.05 m/s → 50 mm/s in firmware units.

GPS noise is approximately white (not colored) — the u-blox internal Kalman filter has already done the colored-noise smoothing.

### 5.3 COCOM dropout

Inputs: truth `vel_NED` (3×1), `pos_NED` (3×1).

Steps:
1. Compute total velocity magnitude: `v_total = norm(vel_NED)`.
2. Compute altitude: `alt = -pos_NED(3)`.
3. If `v_total > GPS.COCOMVelThreshold_mps (500 m/s)` AND `alt > GPS.COCOMAltThreshold_m (18000 m)`:
   - Set `fix_type = 0` (no fix)
   - Set `num_sv = 0`
   - Hold position/velocity outputs at last-valid values (no new updates)
4. Else, output fix and update freely.

For the RasAero trajectory, COCOM is active roughly t ∈ [10, 50] s (the high-velocity-high-altitude window).

### 5.4 Latency

Buffer outputs through a 100 ms delay (1 sample at 10 Hz). Simple FIFO with depth = 1. After delay, output `data_ready = true` once per 10 Hz tick.

### 5.5 Fix type and satellite count

- Pre-launch (alt < 1 m, time < 60 s): `fix_type = 3` (3D), `num_sv = 12` (constant — sim is ideal).
- In-flight, not in COCOM: same.
- In COCOM: `fix_type = 0`, `num_sv = 0`.
- During the 1 s after COCOM exit (transient): `fix_type = 2` (2D), `num_sv = 4`, gradually returning to 3D over the next second as the receiver re-acquires.

This is an idealized model. Real receivers can take 5–30 s to re-acquire after COCOM exit; sim can revisit this in Phase 1 if needed for descent-phase EKF validation.

## 6. Acceptance criteria

- [ ] On pad, lat/lon/alt outputs match the configured launch site within `±2 m` (after noise) over 30 s.
- [ ] Velocity outputs on pad are within `±0.1 m/s` of zero (each axis).
- [ ] Fix type on pad = 3 (3D fix), num_sv = 12.
- [ ] COCOM gate fires for the trajectory portion where `v > 500 m/s ∧ alt > 18 km`. Verify the on/off times in the test plot match `Flight_Test.CSV` truth values.
- [ ] During COCOM, `fix_type = 0`, `num_sv = 0`, position/velocity outputs are held at last-valid.
- [ ] After COCOM exit, `fix_type` returns to 3 within ~2 s; `num_sv` returns to 12.
- [ ] 100 ms latency: a step input in truth position arrives at the GPS output 100 ms later.
- [ ] Output rate is exactly 10 Hz.
- [ ] Reproducible: two runs with same seed produce identical GPS stream.
- [ ] Plots:
  - `gps_pad_30s.png`: 30 s of lat/lon/alt with noise visible at 1.5 m horizontal, 1.6 m vertical.
  - `gps_cocom_window.png`: t=0..60s showing velocity, altitude, fix_type, with shaded COCOM window.
- [ ] STATUS.md PASS.

## 7. Anti-goals

- Do not introduce non-white GPS noise. Real receivers have it (multipath, etc.), but the noise model would need real data to characterize. Phase 0 assumes white.
- Do not implement detailed Doppler ambiguity, ionospheric delays, or any propagation effects. Phase 0 is sea-level white noise.
- Do not produce GPS output during COCOM (would mislead the EKF).
- Do not add latency variance — Phase 0 uses fixed 100 ms.
- Do not couple GPS noise to mag noise via shared seed — independent streams.
- Do not use `gpsSensor` from Sensor Fusion Toolbox unless the COCOM behavior can be exactly replicated (probably not — easier to write custom MATLAB Function).

## 8. Hand-off notes

T07 (frame switch) does **not** transform GPS output (lat/lon/alt are already global, frame-independent).

T08 (EKF port) ignores GPS by default in Phase 0 (EKF runs without GPS updates). The block still produces output; downstream just doesn't consume it.

T10 (validation) needs GPS stream logged for plotting (visualization, not metric).

## 9. Source firmware references

- `Software/App/drivers/max_m10m.c` — NAV-PVT parser, register layout, configuration commands
- `Software/App/drivers/max_m10m.h` — UBX protocol constants, fix types, GPS_FIX_3D enum

## 10. References

- `references/FIRMWARE_CONSTANTS.md` §5.5 (GPS config)
- `references/SIMULINK_PATTERNS.md` §5 (MATLAB Function blocks)
- `ARCHITECTURE.md` §5 (de-emphasized in Phase 0)
- u-blox M10 SPG 5.10 Interface Description (external reference, UBX-21035062)
