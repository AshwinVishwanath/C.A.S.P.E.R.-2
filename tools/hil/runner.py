"""HIL runner — streams scenario packets to the firmware and parses
the COBS telemetry that comes back.

Usage:
  python -m tools.hil.runner --port COM3 --scenario openrocket_hpr [--speed 1.0]

Without --port we run a dry mode that lists the packet schedule (useful
for sanity-checking byte layouts before plugging in hardware).
"""
from __future__ import annotations

import argparse
import importlib
import sys
import time
from collections import Counter
from pathlib import Path
from typing import Iterable, Iterator, Tuple

from . import protocol
from .framing import CobsRxAccumulator


# ── Pacing ─────────────────────────────────────────────────────────

def _busy_wait_until(deadline_perf: float) -> None:
    """Sleep most of the remaining time, then busy-spin the last 200 µs.

    Windows' time.sleep() has 1–15 ms jitter depending on the system
    timer resolution. For 833 Hz IMU pacing we want sub-ms accuracy,
    so we coarse-sleep then spin. The `time.perf_counter` clock is
    monotonic and high-resolution on every platform we care about.
    """
    coarse = deadline_perf - time.perf_counter() - 0.0002
    if coarse > 0:
        time.sleep(coarse)
    while time.perf_counter() < deadline_perf:
        pass


# ── RX side ────────────────────────────────────────────────────────

def _drain_serial(ser, accum: CobsRxAccumulator, on_frame) -> None:
    """Pull whatever's available from the serial port and feed the
    decoder. Non-blocking — we call this opportunistically between
    TX writes."""
    if ser is None:
        return
    try:
        in_waiting = ser.in_waiting
    except OSError:
        return
    if in_waiting:
        chunk = ser.read(in_waiting)
        accum.feed(chunk)
        for f in accum.frames():
            on_frame(f)


# ── Telemetry log ─────────────────────────────────────────────────

class TelemetryLog:
    """Aggregates parsed FAST/GPS/EVENT telemetry from the firmware
    and prints it as it arrives. Tracks state transitions so the
    operator can see PAD→BOOST→… without scrolling through every
    FAST packet."""

    def __init__(self) -> None:
        self.fast_count   = 0
        self.gps_count    = 0
        self.events: list[protocol.EventTelemetry] = []
        self.fsm_history: list[tuple[float, int, str]] = []   # (t_s, state, name)
        self.last_fsm     = -1
        self.last_alt_m   = 0.0
        self.last_vel_mps = 0.0
        self.last_print_t = 0.0
        self.start_perf   = time.perf_counter()

    def on_frame(self, payload: bytes) -> None:
        parsed = protocol.dispatch(payload)
        if parsed is None:
            return
        elapsed = time.perf_counter() - self.start_perf

        if isinstance(parsed, protocol.FastTelemetry):
            self.fast_count += 1
            self.last_alt_m   = parsed.alt_m
            self.last_vel_mps = parsed.vel_mps
            if parsed.fsm_state != self.last_fsm:
                self.fsm_history.append((parsed.flight_time_s, parsed.fsm_state, parsed.fsm_name))
                self.last_fsm = parsed.fsm_state
                print(f"  [{elapsed:7.2f}s] FSM → {parsed.fsm_name:8s} "
                      f"(t_flight={parsed.flight_time_s:6.1f}s alt={parsed.alt_m:7.1f} m "
                      f"vel={parsed.vel_mps:6.1f} m/s)")
            else:
                # Compact periodic update at ~1 Hz so the operator sees
                # progress without drowning in FAST packets.
                if elapsed - self.last_print_t >= 1.0:
                    self.last_print_t = elapsed
                    armed = "".join("A" if a else "." for a in parsed.pyro_armed)
                    cont  = "".join("C" if c else "." for c in parsed.pyro_continuity)
                    print(f"  [{elapsed:7.2f}s] {parsed.fsm_name:8s} "
                          f"alt={parsed.alt_m:7.1f}m vel={parsed.vel_mps:6.1f}m/s "
                          f"batt={parsed.batt_v:.2f}V arm={armed} cont={cont} "
                          f"seq={parsed.seq:3d}")

        elif isinstance(parsed, protocol.GpsTelemetry):
            self.gps_count += 1

        elif isinstance(parsed, protocol.EventTelemetry):
            self.events.append(parsed)
            extra = ""
            if parsed.event_type == protocol.FC_EVT_PYRO:
                ch  = (parsed.data >> 8) & 0xFF
                dur = parsed.data & 0xFF
                extra = f" ch{ch} dur≈{dur*100}ms"
            elif parsed.event_type == protocol.FC_EVT_APOGEE:
                extra = f" peak≈{parsed.data*10} m"
            elif parsed.event_type == protocol.FC_EVT_STATE:
                extra = f" → {protocol.FSM_NAMES.get(parsed.data, parsed.data)}"
            elif parsed.event_type == protocol.FC_EVT_ARM:
                ch_code = (parsed.data >> 8) & 0xFF
                manual  = (parsed.data & 0xFF) == 0
                extra = f" ch{ch_code} ({'manual' if manual else 'auto'})"
            elif parsed.event_type == protocol.FC_EVT_BURNOUT:
                extra = f" peak_g≈{parsed.data/1000.0:.2f}"
            print(f"  [{elapsed:7.2f}s] EVENT {parsed.event_name}{extra} "
                  f"(t_flight={parsed.flight_time_s:.1f}s)")

    def summary(self) -> str:
        lines = ["", "── Run summary ────────────────────────────"]
        lines.append(f"  FAST packets received: {self.fast_count}")
        lines.append(f"  GPS  packets received: {self.gps_count}")
        lines.append(f"  Events received:       {len(self.events)}")
        lines.append(f"  Last alt seen:         {self.last_alt_m:.1f} m")
        lines.append(f"  Last vel seen:         {self.last_vel_mps:.1f} m/s")

        ev_counts = Counter(e.event_name for e in self.events)
        if ev_counts:
            lines.append("  Events by type:")
            for name, n in sorted(ev_counts.items()):
                lines.append(f"    {name:8s} ×{n}")

        if self.fsm_history:
            lines.append("  FSM transitions:")
            for t, _, name in self.fsm_history:
                lines.append(f"    {name:8s} at t_flight={t:.1f}s")
        return "\n".join(lines)


# ── Main loop ──────────────────────────────────────────────────────

def stream(scenario_packets: Iterator[Tuple[float, bytes, str]],
           ser, *, speed: float, log: TelemetryLog,
           tx_kinds: Counter | None = None,
           silence_warn_s: float = 5.0) -> None:
    """Drain `scenario_packets` (sorted by virtual_time) and write
    each one at virtual_time / speed wall seconds from start.

    Reads incoming serial data between writes. If no recognised frame
    arrives within `silence_warn_s` wall seconds, prints a one-shot
    diagnostic so the operator doesn't sit through a 10-minute run
    that's silently dropping packets."""
    accum = CobsRxAccumulator(max_frame=64)
    start_perf = time.perf_counter()
    first_t: float | None = None
    n = 0
    silence_warned = False

    last_rx_count = 0
    def _rx_count():
        return log.fast_count + log.gps_count + len(log.events)

    for when_s, frame, kind in scenario_packets:
        if first_t is None:
            first_t = when_s
        target_perf = start_perf + (when_s - first_t) / speed
        if target_perf > time.perf_counter():
            _busy_wait_until(target_perf)
        if ser is not None:
            try:
                ser.write(frame)
            except OSError as e:
                print(f"  serial write failed: {e}", file=sys.stderr)
                break
        n += 1
        if tx_kinds is not None:
            tx_kinds[kind] += 1
        # Drain RX every ~32 packets so the parser doesn't lag.
        if (n & 31) == 0:
            _drain_serial(ser, accum, log.on_frame)
            cur_rx = _rx_count()
            if cur_rx != last_rx_count:
                last_rx_count = cur_rx
            elif (not silence_warned
                  and cur_rx == 0
                  and (time.perf_counter() - start_perf) >= silence_warn_s):
                silence_warned = True
                print(
                    "\n  ⚠  No telemetry from firmware after "
                    f"{silence_warn_s:.0f}s. Likely causes:\n"
                    "     • Wrong firmware loaded (need -DHIL_MODE,\n"
                    "       check `make hil` build at "
                    "build/FlightComputer/Casper2_Flight.hex)\n"
                    "     • Pre-fix firmware (CDC ring was 256 bytes\n"
                    "       and overflowed at this rate — rebuild from\n"
                    "       a tree containing the 4 KB ring fix)\n"
                    "     • COM port pointing at the wrong device\n"
                    "     • Try --speed 1.0 to rule out throughput issues\n",
                    file=sys.stderr,
                )

    # Final drain — give the firmware ~250 ms to flush its TX queue.
    if ser is not None:
        deadline = time.perf_counter() + 0.25
        while time.perf_counter() < deadline:
            _drain_serial(ser, accum, log.on_frame)
            time.sleep(0.005)


def cli(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", default=None, help="Serial port (e.g. COM3 or /dev/ttyACM0).")
    p.add_argument("--baud", type=int, default=921600, help="Ignored on USB CDC; harmless.")
    p.add_argument("--scenario", default="openrocket_hpr",
                   help="Scenario module name (under tools.hil.scenarios).")
    p.add_argument("--speed", type=float, default=1.0,
                   help="Wall-clock multiplier. >1 = fast (e.g. 5.0 = 5× faster).")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--duration", type=float, default=None,
                   help="Truncate the scenario to this many seconds of virtual time.")
    p.add_argument("--csv", type=Path, default=None,
                   help="Override the OpenRocket CSV path (default: CSVs/HIL Sim Data.csv).")
    p.add_argument("--no-skip-cal", action="store_true",
                   help="Force the firmware to wait the full 30 s pad cal window.")
    p.add_argument("--dry-run", action="store_true",
                   help="Don't open the port; just enumerate the packet schedule.")
    p.add_argument("--show-schedule", type=int, default=0, metavar="N",
                   help="In dry-run, print the first N packets in detail.")
    args = p.parse_args(argv)

    # Build the scenario.
    try:
        mod = importlib.import_module(f"tools.hil.scenarios.{args.scenario}")
    except ModuleNotFoundError as e:
        print(f"unknown scenario '{args.scenario}': {e}", file=sys.stderr)
        return 2

    sc_args = mod.ScenarioArgs(
        seed=args.seed,
        skip_cal=(not args.no_skip_cal),
        duration_s=args.duration,
    )
    if args.csv is not None:
        sc_args.csv_path = args.csv
    print(f"Scenario:  {mod.SCENARIO_NAME}")
    print(f"CSV:       {sc_args.csv_path}")
    if hasattr(mod, "expected_apogee"):
        try:
            t_apo, peak = mod.expected_apogee(sc_args)
            print(f"Trajectory apogee: {peak:.1f} m at t_flight={t_apo:.1f}s")
        except Exception as e:
            print(f"  (apogee lookup failed: {e})")

    packets = mod.build_packets(sc_args)

    if args.dry_run or args.port is None:
        kinds = Counter()
        first_t = last_t = None
        head = []
        for when, frame, kind in packets:
            kinds[kind] += 1
            if first_t is None: first_t = when
            last_t = when
            if args.show_schedule and len(head) < args.show_schedule:
                head.append((when, kind, len(frame)))
        print()
        print(f"Packet count: {sum(kinds.values())}")
        for k in ("IMU", "AUX", "ADXL"):
            print(f"  {k}:   {kinds.get(k, 0):>6d}")
        print(f"Span:        {first_t:.3f}s … {last_t:.3f}s ({last_t-first_t:.1f}s)")
        if head:
            print("First packets:")
            for w, k, n in head:
                print(f"  t={w:9.4f}s  {k:4s}  {n:3d} bytes (incl. delim)")
        if args.port is None and not args.dry_run:
            print("\nNo --port specified — exiting after dry enumeration.")
        return 0

    # Real run — open the serial port and stream.
    try:
        import serial as pyserial   # late import so dry-run works without pyserial installed
    except ImportError:
        print("pyserial is required for live runs (pip install pyserial)", file=sys.stderr)
        return 3

    ser = pyserial.Serial(args.port, args.baud, timeout=0)
    log = TelemetryLog()
    tx_kinds = Counter()
    print(f"Opened {args.port} @ {args.baud} baud, speed×{args.speed}. Streaming…")
    try:
        stream(packets, ser, speed=args.speed, log=log, tx_kinds=tx_kinds)
    except KeyboardInterrupt:
        print("\n(interrupted)")
    finally:
        ser.close()

    print()
    print(f"TX summary: {dict(tx_kinds)}")
    print(log.summary())
    return 0


if __name__ == "__main__":
    sys.exit(cli())
