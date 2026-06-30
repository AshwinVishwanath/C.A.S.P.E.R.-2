"""Wire-format pack/parse for HIL packets and firmware telemetry.

Byte layouts mirror Software/App/telemetry/tlm_types.h exactly.
Every packet is CRC-32 protected; CRC is appended raw — COBS framing
is layered on top via framing.py.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Sequence

from .framing import crc32

# ── Message IDs (mirror tlm_types.h) ────────────────────────────────
MSG_ID_FAST          = 0x01
MSG_ID_GPS           = 0x02
MSG_ID_EVENT         = 0x03
MSG_ID_HIL_INJECT    = 0xD1
MSG_ID_HIL_RAW       = 0xD3
MSG_ID_HIL_AUX       = 0xD4
MSG_ID_HIL_ADXL      = 0xD5

# ── FSM states ──────────────────────────────────────────────────────
FSM_PAD       = 0x0
FSM_BOOST     = 0x1
FSM_COAST     = 0x2
FSM_COAST_1   = 0x3
FSM_SUSTAIN   = 0x4
FSM_COAST_2   = 0x5
FSM_APOGEE    = 0x6
FSM_DROGUE    = 0x7
FSM_MAIN      = 0x8
FSM_RECOVERY  = 0x9
FSM_TUMBLE    = 0xA
FSM_LANDED    = 0xB

FSM_NAMES = {
    FSM_PAD:      "PAD",
    FSM_BOOST:    "BOOST",
    FSM_COAST:    "COAST",
    FSM_COAST_1:  "COAST_1",
    FSM_SUSTAIN:  "SUSTAIN",
    FSM_COAST_2:  "COAST_2",
    FSM_APOGEE:   "APOGEE",
    FSM_DROGUE:   "DROGUE",
    FSM_MAIN:     "MAIN",
    FSM_RECOVERY: "RECOVERY",
    FSM_TUMBLE:   "TUMBLE",
    FSM_LANDED:   "LANDED",
}

# ── Event types ─────────────────────────────────────────────────────
FC_EVT_STATE   = 0x01
FC_EVT_PYRO    = 0x02
FC_EVT_APOGEE  = 0x03
FC_EVT_ERROR   = 0x04
FC_EVT_ORIGIN  = 0x05
FC_EVT_BURNOUT = 0x06
FC_EVT_STAGING = 0x07
FC_EVT_ARM     = 0x08

EVENT_NAMES = {
    FC_EVT_STATE:   "STATE",
    FC_EVT_PYRO:    "PYRO",
    FC_EVT_APOGEE:  "APOGEE",
    FC_EVT_ERROR:   "ERROR",
    FC_EVT_ORIGIN:  "ORIGIN",
    FC_EVT_BURNOUT: "BURNOUT",
    FC_EVT_STAGING: "STAGING",
    FC_EVT_ARM:     "ARM",
}

# ── Encoding scales (mirror tlm_types.h) ───────────────────────────
ALT_SCALE_M    = 0.01    # 1 cm per LSB, u24 max ≈ 167,772 m
VEL_SCALE_DMS  = 0.1     # 0.1 m/s per LSB
TIME_SCALE_DS  = 0.1     # 0.1 s per LSB
BATT_OFFSET_V  = 6.0
BATT_STEP_V    = 0.012

# ── HIL packet sizes ────────────────────────────────────────────────
SIZE_HIL_RAW   = 50
SIZE_HIL_AUX   = 33
SIZE_HIL_ADXL  = 16

# ── HIL aux flag bits ───────────────────────────────────────────────
HIL_AUX_FLAG_GPS_VALID = 0x0001


# ════════════════════════════════════════════════════════════════════
#  Pack: host → firmware
# ════════════════════════════════════════════════════════════════════

def pack_hil_raw(
    tick_ms: int,
    accel_ms2: Sequence[float],     # body frame
    gyro_rads: Sequence[float],     # body frame
    baro_pa: float,
    mag_ut: Sequence[float],        # body frame, calibrated
    *,
    baro_valid: bool = True,
    mag_valid: bool = True,
    skip_cal: bool = False,
) -> bytes:
    """0xD3 raw IMU+baro+mag inject. 50 bytes including CRC."""
    flags = 0
    if baro_valid: flags |= 0x01
    if mag_valid:  flags |= 0x02
    if skip_cal:   flags |= 0x04

    payload = struct.pack(
        "<BI3f3f f3f B",
        MSG_ID_HIL_RAW, tick_ms,
        accel_ms2[0], accel_ms2[1], accel_ms2[2],
        gyro_rads[0], gyro_rads[1], gyro_rads[2],
        baro_pa,
        mag_ut[0], mag_ut[1], mag_ut[2],
        flags,
    )
    assert len(payload) == 46, f"{len(payload)=} != 46"
    payload += struct.pack("<I", crc32(payload))
    assert len(payload) == SIZE_HIL_RAW
    return payload


def pack_hil_aux(
    tick_ms: int,
    *,
    gps_dlat_mm: int = 0,
    gps_dlon_mm: int = 0,
    gps_alt_msl_m: float = 0.0,
    gps_vel_d_mps: float = 0.0,    # +down
    gps_fix: int = 0,              # 0=no, 2=2D, 3=3D
    gps_sat: int = 0,
    cont_bitmap: int = 0x0F,       # default: all four channels show continuity
    batt_v: float = 7.4,
    adxl_activity: bool = False,
    gps_valid: bool = True,
) -> bytes:
    """0xD4 HIL aux (low-rate signals). 33 bytes including CRC."""
    flags = 0
    if gps_valid: flags |= HIL_AUX_FLAG_GPS_VALID

    batt_x100 = max(0, min(0xFFFF, int(round(batt_v * 100.0))))

    payload = struct.pack(
        "<BIii ff BB B H B H",
        MSG_ID_HIL_AUX, tick_ms,
        int(gps_dlat_mm), int(gps_dlon_mm),
        gps_alt_msl_m, gps_vel_d_mps,
        int(gps_fix) & 0xFF, int(gps_sat) & 0xFF,
        int(cont_bitmap) & 0xFF,
        batt_x100,
        1 if adxl_activity else 0,
        flags & 0xFFFF,
    )
    assert len(payload) == 29, f"{len(payload)=} != 29"
    payload += struct.pack("<I", crc32(payload))
    assert len(payload) == SIZE_HIL_AUX
    return payload


def pack_hil_adxl(
    tick_ms: int,
    raw_ax: int,
    raw_ay: int,
    raw_az: int,
    *,
    flags: int = 0,
) -> bytes:
    """0xD5 ADXL high-G sample. 16 bytes including CRC.

    raw_* are int16, the same int16-left-justified-12-bit format the
    real driver produces. Convert g → raw via raw = round(g / 0.1) << 4.
    """
    payload = struct.pack(
        "<BI hhh B",
        MSG_ID_HIL_ADXL, tick_ms,
        int(raw_ax), int(raw_ay), int(raw_az),
        int(flags) & 0xFF,
    )
    assert len(payload) == 12, f"{len(payload)=} != 12"
    payload += struct.pack("<I", crc32(payload))
    assert len(payload) == SIZE_HIL_ADXL
    return payload


# ════════════════════════════════════════════════════════════════════
#  Parse: firmware → host
# ════════════════════════════════════════════════════════════════════

@dataclass
class FastTelemetry:
    seq: int
    fsm_state: int
    fsm_name: str
    pyro_armed: list[bool]
    pyro_continuity: list[bool]
    pyro_fired: bool
    sys_error: bool
    alt_m: float
    vel_mps: float
    flight_time_s: float
    batt_v: float
    quat_raw: bytes  # 5 bytes, smallest-three packed (decode is informational)


@dataclass
class GpsTelemetry:
    dlat_mm: int
    dlon_mm: int
    alt_msl_m: float
    fix_type: int
    sat_count: int


@dataclass
class EventTelemetry:
    event_type: int
    event_name: str
    data: int
    flight_time_s: float


def _check_crc(payload: bytes) -> bool:
    """payload includes the trailing 4-byte CRC. Returns True if valid."""
    if len(payload) < 5:
        return False
    body, recv = payload[:-4], payload[-4:]
    return struct.unpack("<I", recv)[0] == crc32(body)


def parse_fast(payload: bytes) -> Optional[FastTelemetry]:
    """Decode a 21-byte FC_MSG_FAST. Returns None on any error."""
    if len(payload) != 21 or payload[0] != MSG_ID_FAST or not _check_crc(payload):
        return None

    status0, status1 = payload[1], payload[2]
    armed       = [bool(status0 & (1 << (4 + i))) for i in range(4)]
    continuity  = [bool(status0 & (1 << i))       for i in range(4)]
    fsm_state   = (status1 >> 4) & 0x0F
    fired       = bool(status1 & (1 << 3))
    sys_error   = bool(status1 & (1 << 2))

    alt_raw = payload[3] | (payload[4] << 8) | (payload[5] << 16)
    alt_m   = alt_raw * ALT_SCALE_M

    (vel_dms,) = struct.unpack("<h", payload[6:8])
    vel_mps    = vel_dms * VEL_SCALE_DMS

    quat_raw = bytes(payload[8:13])

    (time_ds,) = struct.unpack("<H", payload[13:15])
    flight_time_s = time_ds * TIME_SCALE_DS

    batt_v = BATT_OFFSET_V + payload[15] * BATT_STEP_V
    seq    = payload[16]

    return FastTelemetry(
        seq=seq, fsm_state=fsm_state, fsm_name=FSM_NAMES.get(fsm_state, f"?{fsm_state}"),
        pyro_armed=armed, pyro_continuity=continuity,
        pyro_fired=fired, sys_error=sys_error,
        alt_m=alt_m, vel_mps=vel_mps,
        flight_time_s=flight_time_s, batt_v=batt_v, quat_raw=quat_raw,
    )


def parse_gps(payload: bytes) -> Optional[GpsTelemetry]:
    """Decode an 18-byte FC_MSG_GPS."""
    if len(payload) != 18 or payload[0] != MSG_ID_GPS or not _check_crc(payload):
        return None

    (dlat_mm,)   = struct.unpack("<i", payload[1:5])
    (dlon_mm,)   = struct.unpack("<i", payload[5:9])
    alt_raw      = payload[9] | (payload[10] << 8) | (payload[11] << 16)
    alt_msl_m    = alt_raw * ALT_SCALE_M
    fix_type     = payload[12]
    sat_count    = payload[13]

    return GpsTelemetry(
        dlat_mm=dlat_mm, dlon_mm=dlon_mm,
        alt_msl_m=alt_msl_m,
        fix_type=fix_type, sat_count=sat_count,
    )


def parse_event(payload: bytes) -> Optional[EventTelemetry]:
    """Decode an 11-byte FC_MSG_EVENT."""
    if len(payload) != 11 or payload[0] != MSG_ID_EVENT or not _check_crc(payload):
        return None

    event_type   = payload[1]
    (data,)      = struct.unpack("<H", payload[2:4])
    (time_ds,)   = struct.unpack("<H", payload[4:6])

    return EventTelemetry(
        event_type=event_type,
        event_name=EVENT_NAMES.get(event_type, f"?{event_type}"),
        data=data,
        flight_time_s=time_ds * TIME_SCALE_DS,
    )


def dispatch(payload: bytes):
    """Route a decoded COBS frame to the right parser. Returns the
    parsed dataclass, or None if the frame is not a known telemetry
    message (could be an ACK, NACK, dump byte, etc.)."""
    if not payload:
        return None
    msg_id = payload[0]
    if msg_id == MSG_ID_FAST:
        return parse_fast(payload)
    if msg_id == MSG_ID_GPS:
        return parse_gps(payload)
    if msg_id == MSG_ID_EVENT:
        return parse_event(payload)
    return None
