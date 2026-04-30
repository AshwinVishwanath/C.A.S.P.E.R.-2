"""HPR flight from a real OpenRocket export (`CSVs/HIL Sim Data.csv`).

The trajectory is ~10 minutes long with 40 km apogee. We resample it
into the firmware's expected sensor cadences:

    0xD3 IMU  @ 833 Hz        — accel + gyro every packet
                                 + baro every Nth packet (≈50 Hz)
                                 + mag every Mth packet (≈100 Hz)
    0xD4 aux  @ 10 Hz         — GPS + continuity + battery
    0xD5 ADXL @ 400 Hz        — high-G sample (after rocket leaves rail)

The first 0xD4 is sent before the IMU stream so continuity / battery /
GPS-valid are populated by the time the firmware exits its 30 s
calibration window.

For development sanity the scenario sets `skip_cal=True` on every 0xD3
so the firmware fast-tracks pad init — otherwise a 1×-speed run
spends 30 wall-seconds twiddling thumbs before BOOST can trigger.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Tuple

from .. import protocol
from ..framing import cobs_encode
from ..sensor_models import NoiseModel, synthesise_adxl_raw, synthesise_gps, synthesise_imu_baro_mag
from ..trajectory import Trajectory


SCENARIO_NAME = "openrocket_hpr"

DEFAULT_CSV = Path("CSVs/HIL Sim Data.csv")

IMU_HZ  = 833.0
AUX_HZ  =  10.0
ADXL_HZ = 400.0

# How often each sub-channel rides along inside the 0xD3 stream.
BARO_DIVIDER = max(1, int(round(IMU_HZ /  50.0)))   # ≈ every 17th packet
MAG_DIVIDER  = max(1, int(round(IMU_HZ / 100.0)))   # ≈ every 8th packet


@dataclass
class ScenarioArgs:
    csv_path:       Path = DEFAULT_CSV
    seed:           int  = 42
    skip_cal:       bool = True
    duration_s:     float | None = None      # truncate the CSV (e.g. for quick smoke tests)
    pad_alt_msl_m:  float = 0.0
    enable_adxl:    bool = True
    earth_field_body_ut: tuple[float, float, float] = (5.0, -40.0, 15.0)


def _imu_period_s() -> float: return 1.0 / IMU_HZ
def _aux_period_s() -> float: return 1.0 / AUX_HZ
def _adxl_period_s() -> float: return 1.0 / ADXL_HZ


def build_packets(args: ScenarioArgs) -> Iterator[Tuple[float, bytes, str]]:
    """Yield (virtual_time_s, framed_packet_bytes, kind) tuples in time order.

    The runner uses virtual_time_s to pace transmission; framing is
    already applied (COBS + delimiter), so the runner just writes
    bytes to the serial port.
    """
    traj = Trajectory(args.csv_path)
    noise = NoiseModel(seed=args.seed)

    t_end = traj.t_end if args.duration_s is None else min(traj.t_end, args.duration_s)
    t_start = traj.t_start

    # ── Pre-flight 0xD4: set continuity / batt / fix BEFORE the IMU
    # stream begins, so when the firmware first looks at g_hil_aux it
    # already sees a sensible world.
    pre_state = traj.state(t_start)
    pre_gps   = synthesise_gps(pre_state, noise, pad_alt_msl_m=args.pad_alt_msl_m)
    yield (
        t_start - 0.005,
        cobs_encode(protocol.pack_hil_aux(
            tick_ms=int(t_start * 1000),
            gps_dlat_mm=pre_gps.dlat_mm,
            gps_dlon_mm=pre_gps.dlon_mm,
            gps_alt_msl_m=pre_gps.alt_msl_m,
            gps_vel_d_mps=pre_gps.vel_d_mps,
            gps_fix=pre_gps.fix_type, gps_sat=pre_gps.sat_count,
            cont_bitmap=0x0F, batt_v=7.4, adxl_activity=False,
            gps_valid=True,
        )),
        "AUX",
    )

    # Build a unified time-line of (when, kind, builder_callable).
    # Heap-merge the three streams so packets emit in time order.
    heap: list[Tuple[float, int, str]] = []   # (when_s, sequence_idx, kind)
    aux_counter  = 0
    adxl_counter = 0
    imu_counter  = 0

    # First-event seed for each stream.
    next_imu  = t_start
    next_aux  = t_start + _aux_period_s()
    next_adxl = t_start + _adxl_period_s() if args.enable_adxl else math.inf

    seq = 0
    while True:
        # Pick the soonest of the three.
        candidates = [(next_imu, "IMU"), (next_aux, "AUX"), (next_adxl, "ADXL")]
        when, kind = min(candidates, key=lambda c: c[0])
        if when > t_end:
            break

        state = traj.state(when)

        if kind == "IMU":
            mag_send  = (imu_counter % MAG_DIVIDER)  == 0
            baro_send = (imu_counter % BARO_DIVIDER) == 0
            sensors = synthesise_imu_baro_mag(
                state, noise,
                baro_valid=baro_send,
                mag_valid=mag_send,
                pad_alt_msl_m=args.pad_alt_msl_m,
                earth_field_body_ut=args.earth_field_body_ut,
            )
            pkt = protocol.pack_hil_raw(
                tick_ms=int(when * 1000),
                accel_ms2=sensors.accel_ms2,
                gyro_rads=sensors.gyro_rads,
                baro_pa=sensors.baro_pa,
                mag_ut=sensors.mag_ut,
                baro_valid=sensors.baro_valid,
                mag_valid=sensors.mag_valid,
                skip_cal=args.skip_cal,
            )
            yield (when, cobs_encode(pkt), "IMU")
            imu_counter += 1
            next_imu += _imu_period_s()

        elif kind == "AUX":
            gps = synthesise_gps(state, noise, pad_alt_msl_m=args.pad_alt_msl_m)
            pkt = protocol.pack_hil_aux(
                tick_ms=int(when * 1000),
                gps_dlat_mm=gps.dlat_mm,
                gps_dlon_mm=gps.dlon_mm,
                gps_alt_msl_m=gps.alt_msl_m,
                gps_vel_d_mps=gps.vel_d_mps,
                gps_fix=gps.fix_type, gps_sat=gps.sat_count,
                cont_bitmap=0x0F, batt_v=7.4,
                adxl_activity=False,
                gps_valid=True,
            )
            yield (when, cobs_encode(pkt), "AUX")
            aux_counter += 1
            next_aux += _aux_period_s()

        elif kind == "ADXL":
            ax, ay, az = synthesise_adxl_raw(state, noise)
            pkt = protocol.pack_hil_adxl(
                tick_ms=int(when * 1000),
                raw_ax=ax, raw_ay=ay, raw_az=az,
            )
            yield (when, cobs_encode(pkt), "ADXL")
            adxl_counter += 1
            next_adxl += _adxl_period_s()

        seq += 1


def expected_apogee(args: ScenarioArgs) -> tuple[float, float]:
    traj = Trajectory(args.csv_path)
    return traj.apogee()
