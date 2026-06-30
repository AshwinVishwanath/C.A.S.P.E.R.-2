"""Trajectory loader for OpenRocket CSV exports.

The exporter we expect produces one row per integration step with a
`# Time (s)` header line, followed by columns separated by commas.
Column order varies between export presets, so we discover indices
from the header line rather than assuming positions. NaN entries are
preserved as `math.nan`.

Provides linear-interpolated `state(t)` queries between samples so the
runner can pace at any rate (e.g. 833 Hz IMU) without being tied to the
CSV's native step.
"""
from __future__ import annotations

import csv
import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


# ── Column header → trajectory.state field name ─────────────────────
# Keys are the lowercase prefix we match against (so unit suffixes like
# "(m)" or "(m/s)" don't matter). Values are the field names exposed
# by `State` below. Missing columns just stay NaN.
_COLUMN_MAP = {
    "time":                          "t_s",
    "altitude":                      "alt_m",
    "vertical velocity":             "vel_up_mps",
    "vertical acceleration":         "vert_acc_mps2",   # +up, kinematic
    "lateral velocity":              "vel_lat_mps",
    "lateral acceleration":          "acc_lat_mps2",
    "total velocity":                "vel_total_mps",
    "total acceleration":            "acc_total_mps2",
    "gravitational acceleration":    "g_local_mps2",
    "angle of attack":               "aoa_deg",
    "roll rate":                     "roll_rate_dps",
    "pitch rate":                    "pitch_rate_dps",
    "yaw rate":                      "yaw_rate_dps",
    "mass":                          "mass_g",
    "motor mass":                    "motor_mass_g",
    "mach number":                   "mach",
    "thrust":                        "thrust_n",
    "drag force":                    "drag_n",
    "air density":                   "air_density_g_cm3",
    "dynamic pressure":              "dyn_pressure_mbar",
}


@dataclass
class State:
    t_s:               float = math.nan
    alt_m:             float = math.nan
    vel_up_mps:        float = math.nan
    vert_acc_mps2:     float = math.nan
    vel_lat_mps:       float = math.nan
    acc_lat_mps2:      float = math.nan
    vel_total_mps:     float = math.nan
    acc_total_mps2:    float = math.nan
    g_local_mps2:      float = math.nan
    aoa_deg:           float = math.nan
    roll_rate_dps:     float = math.nan
    pitch_rate_dps:    float = math.nan
    yaw_rate_dps:      float = math.nan
    mass_g:            float = math.nan
    motor_mass_g:      float = math.nan
    mach:              float = math.nan
    thrust_n:          float = math.nan
    drag_n:            float = math.nan
    air_density_g_cm3: float = math.nan
    dyn_pressure_mbar: float = math.nan


def _parse_float(s: str) -> float:
    s = s.strip()
    if not s or s.lower() == "nan":
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def _resolve_columns(header: List[str]) -> Dict[str, int]:
    """Map our state field names to CSV column indices."""
    field_to_idx: Dict[str, int] = {}
    for idx, name in enumerate(header):
        n = name.strip().lstrip("#").strip().lower()
        for prefix, field in _COLUMN_MAP.items():
            if n.startswith(prefix) and field not in field_to_idx:
                field_to_idx[field] = idx
                break
    if "t_s" not in field_to_idx:
        raise ValueError("CSV is missing a Time column")
    if "alt_m" not in field_to_idx:
        raise ValueError("CSV is missing an Altitude column")
    return field_to_idx


class Trajectory:
    """Loads an OpenRocket CSV and provides interpolated state queries."""

    def __init__(self, csv_path: Path) -> None:
        self.path = Path(csv_path)
        self._times: List[float] = []
        self._rows:  List[State] = []
        self._load()

    def _load(self) -> None:
        with self.path.open("r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header: Optional[List[str]] = None
            for raw in reader:
                if not raw:
                    continue
                first = raw[0].strip()
                # OpenRocket prefaces the header with '#'. Skip blanks
                # and other metadata comments before the header line.
                if first.startswith("#") and header is None:
                    # Strip the leading '#' from the first field.
                    header = [raw[0].lstrip("#").strip()] + raw[1:]
                    cols = _resolve_columns(header)
                    self._cols = cols
                    continue
                if first.startswith("#"):
                    continue  # in-flight comment line
                if header is None:
                    raise ValueError("CSV header row not found")
                row = State()
                for field, idx in self._cols.items():
                    if idx < len(raw):
                        setattr(row, field, _parse_float(raw[idx]))
                if not math.isnan(row.t_s):
                    self._times.append(row.t_s)
                    self._rows.append(row)

        if len(self._rows) < 2:
            raise ValueError(f"Trajectory {self.path} has < 2 samples")

        # Assert monotonic time (the runner relies on bisect).
        for a, b in zip(self._times, self._times[1:]):
            if b < a:
                raise ValueError(f"Trajectory time goes backwards: {a} -> {b}")

    # ── Convenience accessors ──────────────────────────────────────

    @property
    def t_start(self) -> float:
        return self._times[0]

    @property
    def t_end(self) -> float:
        return self._times[-1]

    @property
    def n_rows(self) -> int:
        return len(self._rows)

    def apogee(self) -> tuple[float, float]:
        """(t_apogee_s, peak_alt_m). Linear scan, called once per scenario."""
        i_max = max(range(len(self._rows)),
                    key=lambda i: (self._rows[i].alt_m
                                   if not math.isnan(self._rows[i].alt_m)
                                   else -math.inf))
        return self._times[i_max], self._rows[i_max].alt_m

    def state(self, t_s: float) -> State:
        """Return interpolated state at time t_s. Clamps to endpoints
        outside the trajectory window so callers don't have to special-
        case the start and end of the file."""
        if t_s <= self._times[0]:
            return self._rows[0]
        if t_s >= self._times[-1]:
            return self._rows[-1]

        # bisect_left returns the index of the first time >= t_s
        i = bisect_left(self._times, t_s)
        if i == 0:
            return self._rows[0]
        t1, t2 = self._times[i - 1], self._times[i]
        r1, r2 = self._rows[i - 1], self._rows[i]
        if t2 == t1:
            return r1
        f = (t_s - t1) / (t2 - t1)

        out = State(t_s=t_s)
        for field in State.__dataclass_fields__:
            if field == "t_s":
                continue
            a = getattr(r1, field)
            b = getattr(r2, field)
            if math.isnan(a) or math.isnan(b):
                # Don't interpolate through NaN — pick whichever is finite,
                # else NaN. Avoids producing nonsense across motor cutoff
                # or recovery phase where many columns are NaN.
                if math.isnan(a) and math.isnan(b):
                    setattr(out, field, math.nan)
                elif math.isnan(a):
                    setattr(out, field, b)
                else:
                    setattr(out, field, a)
                continue
            setattr(out, field, a + (b - a) * f)
        return out
