"""Synthesise raw-sensor packets from a Trajectory state.

Coordinate convention (mirrors flight_loop.c comments):
  body X = starboard, body Y = nose (up on pad), body Z = toward operator.

For the OpenRocket dataset we use, the trajectory is purely vertical
(no roll/pitch/yaw of consequence), so:
  - body Y is aligned with the inertial up axis,
  - lateral motion is negligible,
  - the body-to-NED rotation is identity.

If we ever drive a 6-DOF trajectory, this module is the right place
to compose the rotation from the trajectory's quaternion. For now it
keeps the math 1-D.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .trajectory import State

G = 9.80665              # m/s², matches firmware G_CONST
DEG_TO_RAD = math.pi / 180.0

SEA_LEVEL_PA = 101325.0   # Pa
ALT_RAIL_LEAVE_M = 1.0    # alt above ground at which we treat the
                          # rocket as airborne (not on rail)
THRUST_RAIL_LEAVE_N = 50.0


# ── Atmosphere ─────────────────────────────────────────────────────

def alt_to_pressure_pa(alt_m: float, *, sea_level_pa: float = SEA_LEVEL_PA) -> float:
    """ISA-ish atmosphere: altitude (m AGL) → pressure (Pa).

    Same formula the firmware uses to invert pressure into altitude
    in flight_loop.c, so the round-trip is consistent.
    """
    return sea_level_pa * (1.0 - alt_m / 44307.694) ** (1.0 / 0.190284)


# ── Noise model ────────────────────────────────────────────────────

@dataclass
class NoiseModel:
    """Per-sensor Gaussian noise scales. Defaults are conservative
    estimates that produce realistic-looking traces; tighter values
    from `Matlab Code/casper_sensor_characterization.m` Allan variance
    fits can be plugged in by the scenario."""
    accel_sigma_g:    float = 0.005    # 5 mg
    gyro_sigma_dps:   float = 0.05     # 50 mdps
    baro_sigma_pa:    float = 12.0     # MS5611 OSR_4096
    mag_sigma_ut:     float = 0.4
    gps_sigma_m:      float = 2.5
    gps_vel_sigma_m_s: float = 0.05
    seed:             int   = 42

    rng: random.Random = field(default=None, init=False)

    def __post_init__(self):
        self.rng = random.Random(self.seed)

    def gauss(self, sigma: float) -> float:
        return self.rng.gauss(0.0, sigma)


# ── Body-frame sensor synthesis ────────────────────────────────────

@dataclass
class SensorReadings:
    accel_ms2:  tuple[float, float, float]
    gyro_rads:  tuple[float, float, float]
    baro_pa:    float
    mag_ut:     tuple[float, float, float]
    baro_valid: bool
    mag_valid:  bool


def synthesise_imu_baro_mag(
    state: State,
    noise: NoiseModel,
    *,
    baro_valid: bool = True,
    mag_valid:  bool = True,
    pad_alt_msl_m: float = 0.0,
    earth_field_body_ut: tuple[float, float, float] = (5.0, -40.0, 15.0),
) -> SensorReadings:
    """Map a Trajectory state into raw IMU + baro + mag in body frame.

    On-pad kludge: while the rocket is sitting on the rail the OpenRocket
    integrator reports vertical accel ≈ -g (the unconstrained free-fall
    value). A real accelerometer reads +g along the up-axis at rest, so
    we override the body-Y axis to +g until thrust kicks in or the
    rocket has cleared the rail. Without this the Mahony filter never
    sees a stable gravity vector during the 30 s pad calibration.
    """
    # Body Y = up; vertical flight ⇒ specific force purely on Y.
    on_rail = (
        (math.isnan(state.thrust_n) or state.thrust_n < THRUST_RAIL_LEAVE_N)
        and (math.isnan(state.alt_m) or state.alt_m < ALT_RAIL_LEAVE_M)
    )
    if on_rail:
        ay_clean = G
    else:
        ay_clean = (state.vert_acc_mps2 if not math.isnan(state.vert_acc_mps2) else -G) + G

    accel = (
        0.0 + noise.gauss(noise.accel_sigma_g) * G,
        ay_clean + noise.gauss(noise.accel_sigma_g) * G,
        0.0 + noise.gauss(noise.accel_sigma_g) * G,
    )

    # Gyro: OpenRocket reports rates in deg/s. Map roll → body Y
    # (longitudinal), pitch → body X, yaw → body Z. For this dataset
    # all three are ≈ 0 outside motor burn.
    def _rate(d):  # NaN → 0
        return 0.0 if math.isnan(d) else d * DEG_TO_RAD
    gyro = (
        _rate(state.pitch_rate_dps) + noise.gauss(noise.gyro_sigma_dps) * DEG_TO_RAD,
        _rate(state.roll_rate_dps)  + noise.gauss(noise.gyro_sigma_dps) * DEG_TO_RAD,
        _rate(state.yaw_rate_dps)   + noise.gauss(noise.gyro_sigma_dps) * DEG_TO_RAD,
    )

    # Baro: convert altitude AGL relative to pad. The CSV reports
    # altitude relative to the launch pad already (alt_m == 0 at t=0).
    alt_for_baro = state.alt_m if not math.isnan(state.alt_m) else 0.0
    baro_pa = alt_to_pressure_pa(alt_for_baro + pad_alt_msl_m)
    baro_pa += noise.gauss(noise.baro_sigma_pa)

    # Mag: assume body frame stays aligned with launch attitude (no
    # rotation in this dataset). Add per-axis Gaussian noise.
    mag = (
        earth_field_body_ut[0] + noise.gauss(noise.mag_sigma_ut),
        earth_field_body_ut[1] + noise.gauss(noise.mag_sigma_ut),
        earth_field_body_ut[2] + noise.gauss(noise.mag_sigma_ut),
    )

    return SensorReadings(accel, gyro, baro_pa, mag, baro_valid, mag_valid)


# ── GPS synthesis ─────────────────────────────────────────────────

@dataclass
class GpsReadings:
    dlat_mm:       int
    dlon_mm:       int
    alt_msl_m:     float
    vel_d_mps:     float    # +down
    fix_type:      int      # 3 = 3D
    sat_count:     int
    valid:         bool


def synthesise_gps(
    state: State,
    noise: NoiseModel,
    *,
    pad_alt_msl_m: float = 0.0,
    fix_type: int = 3,
    sat_count: int = 12,
) -> GpsReadings:
    """For a vertical OpenRocket trace, lat/lon stay at 0 (host can
    add lateral drift later by overriding this). Altitude and vertical
    velocity come from the trajectory."""
    alt_agl = state.alt_m if not math.isnan(state.alt_m) else 0.0
    vel_up  = state.vel_up_mps if not math.isnan(state.vel_up_mps) else 0.0

    return GpsReadings(
        dlat_mm=0,
        dlon_mm=0,
        alt_msl_m=alt_agl + pad_alt_msl_m + noise.gauss(noise.gps_sigma_m),
        vel_d_mps=-vel_up + noise.gauss(noise.gps_vel_sigma_m_s),
        fix_type=fix_type,
        sat_count=sat_count,
        valid=True,
    )


# ── ADXL372 high-G synthesis ──────────────────────────────────────

ADXL_LSB_G = 0.1   # 100 mg per 12-bit count


def synthesise_adxl_raw(state: State, noise: NoiseModel) -> tuple[int, int, int]:
    """Return raw int16 (12-bit left-justified ×16) values per axis,
    matching what `adxl372_read()` produces on real hardware. Body
    frame as for the IMU. ±200 g range with 12-bit resolution."""
    on_rail = (
        (math.isnan(state.thrust_n) or state.thrust_n < THRUST_RAIL_LEAVE_N)
        and (math.isnan(state.alt_m) or state.alt_m < ALT_RAIL_LEAVE_M)
    )
    if on_rail:
        ay_g = 1.0
    else:
        ay_g = ((state.vert_acc_mps2 if not math.isnan(state.vert_acc_mps2) else -G) + G) / G

    def _to_raw(a_g: float) -> int:
        a_g += noise.gauss(0.5)  # ADXL noise floor ≈ 0.5 g
        # Quantise to 12-bit signed at 100 mg/LSB, clamp to ±200 g.
        counts = int(round(a_g / ADXL_LSB_G))
        if counts >  2047: counts =  2047
        if counts < -2048: counts = -2048
        # Left-justify into int16 (the firmware right-shifts by 4).
        return counts << 4

    return (_to_raw(0.0), _to_raw(ay_g), _to_raw(0.0))
