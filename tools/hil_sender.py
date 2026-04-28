#!/usr/bin/env python3
"""
Raw-sensor HIL sender for C.A.S.P.E.R.-2

Generates a realistic flight profile, encodes as 0xD3 packets (COBS + CRC-32),
and sends over USB CDC. Reads serial plotter output for live display.

Build firmware with: make clean && make -j8 EXTRA_DEFS="-DHIL_MODE"

Usage:
  python hil_sender.py [--port COM3] [--speed 1.0] [--no-plot]
"""

import argparse
import binascii
import math
import struct
import sys
import time

import serial

# ── COBS encode (matches firmware decoder) ───────────────────────────

def cobs_encode(data: bytes) -> bytes:
    """COBS encode with 0x00 delimiter appended."""
    out = bytearray()
    idx = 0
    while idx < len(data):
        # Find next zero (or end)
        block_start = idx
        while idx < len(data) and data[idx] != 0x00 and (idx - block_start) < 254:
            idx += 1
        code = idx - block_start + 1
        out.append(code)
        out.extend(data[block_start:idx])
        if idx < len(data) and data[idx] == 0x00:
            idx += 1
    # If last byte wasn't zero, we still need to indicate end
    if len(data) == 0 or data[-1] != 0x00:
        pass  # code already accounts for it
    return bytes(out) + b'\x00'


# ── Packet builder ───────────────────────────────────────────────────

MSG_ID_HIL_RAW = 0xD3

def build_packet(tick_ms: int, accel: list, gyro: list, baro_pa: float,
                 mag: list, baro_valid: bool, mag_valid: bool,
                 skip_cal: bool) -> bytes:
    """Build a COBS-encoded 0xD3 HIL raw inject packet."""
    flags = 0
    if baro_valid:  flags |= 0x01
    if mag_valid:   flags |= 0x02
    if skip_cal:    flags |= 0x04

    payload = struct.pack('<BI',  MSG_ID_HIL_RAW, tick_ms)
    payload += struct.pack('<3f', *accel)
    payload += struct.pack('<3f', *gyro)
    payload += struct.pack('<f',  baro_pa)
    payload += struct.pack('<3f', *mag)
    payload += struct.pack('<B',  flags)

    assert len(payload) == 46, f"Payload is {len(payload)}, expected 46"
    crc = binascii.crc32(payload) & 0xFFFFFFFF
    payload += struct.pack('<I', crc)
    assert len(payload) == 50

    return cobs_encode(payload)


# ── Barometric model ─────────────────────────────────────────────────

SEA_LEVEL_PA = 101325.0

def alt_to_pressure(alt_m: float) -> float:
    """ISA atmosphere: altitude (m) -> pressure (Pa)."""
    return SEA_LEVEL_PA * (1.0 - alt_m / 44307.694) ** (1.0 / 0.190284)

def pressure_to_alt(pa: float) -> float:
    """ISA atmosphere: pressure (Pa) -> altitude (m)."""
    return 44307.694 * (1.0 - (pa / SEA_LEVEL_PA) ** 0.190284)


# ── Flight profile generator ────────────────────────────────────────

G = 9.80665

def generate_profile(dt_s=0.0012):
    """
    Generate a complete flight profile as a list of tuples:
      (tick_ms, accel[3], gyro[3], baro_pa, mag[3], baro_valid, mag_valid, skip_cal)

    Body frame: +X=starboard, +Y=nose (up on pad), +Z=toward operator
    On pad, gravity shows as accel_y = +9.81 m/s^2 (sensor measures reaction force)

    Phases:
      1. Pad      (0-1s):     Stationary, gravity only
      2. Boost    (1-4s):     5g thrust along body Y + gravity reaction
      3. Coast    (4-12s):    Free-fall + drag, decelerating
      4. Apogee   (12-13s):   Near-zero velocity
      5. Drogue   (13-28s):   ~10 m/s descent under drogue
      6. Main     (28-45s):   ~5 m/s descent under main (deploy at 250m)
      7. Landed   (45-50s):   Stationary on ground
    """
    samples = []
    t = 0.0
    alt = 0.0
    vel = 0.0

    # Ground-level pressure
    ground_pa = SEA_LEVEL_PA

    # Mag field (body frame, roughly nose-up on pad in northern hemisphere)
    mag_body = [5.0, -40.0, 15.0]  # uT, approximate

    def sample(t, accel, gyro, alt, skip_cal=False, baro_every=10):
        tick = int(t * 1000)
        baro_pa = alt_to_pressure(alt)
        # Baro at ~100 Hz (every 10th sample at 833 Hz)
        baro_valid = (tick % baro_every) == 0
        return (tick, accel, gyro, baro_pa, mag_body, baro_valid, True, skip_cal)

    # ── Phase 1: Pad (0-1s) — skip cal on first packet ──
    while t < 1.0:
        accel = [0.0, G, 0.0]   # gravity reaction along nose
        gyro = [0.0, 0.0, 0.0]
        samples.append(sample(t, accel, gyro, alt, skip_cal=True))
        t += dt_s

    # ── Phase 2: Boost (1-4s) — 5g thrust ──
    thrust_accel = 5.0 * G  # net vertical accel (after gravity)
    while t < 4.0:
        # Body accel = thrust + gravity reaction = (5+1)g along nose
        accel = [0.0, (5.0 + 1.0) * G, 0.0]
        gyro = [0.0, 0.0, 0.0]
        vel += thrust_accel * dt_s
        alt += vel * dt_s
        samples.append(sample(t, accel, gyro, alt))
        t += dt_s

    # ── Phase 3: Coast (4-12s) — free-fall with light drag ──
    while t < 12.0:
        # Drag deceleration increases with velocity, simplified
        drag_g = 0.1 * (vel / 100.0) ** 2  # rough
        net_accel = -G - drag_g * G  # gravity + drag (both opposing upward vel)
        vel += net_accel * dt_s
        if vel < 0:
            vel = max(vel, -50.0)  # cap descent rate
        alt += vel * dt_s
        # Body accel during coast: mostly free-fall, small drag
        accel = [0.0, G * drag_g, 0.0]  # near-zero in free fall
        gyro = [0.01, 0.005, 0.0]  # small tumble
        samples.append(sample(t, accel, gyro, alt))
        t += dt_s

    # ── Phase 4: Apogee (12-13s) ──
    while t < 13.0:
        vel += -G * dt_s
        alt += vel * dt_s
        accel = [0.0, 0.2, 0.0]  # near free-fall
        gyro = [0.02, 0.01, 0.0]
        samples.append(sample(t, accel, gyro, alt))
        t += dt_s

    # ── Phase 5: Drogue descent (13-28s) — ~10 m/s ──
    target_vel = -10.0
    while t < 28.0:
        # Converge to target descent rate
        if vel > target_vel:
            vel += -G * dt_s * 0.3  # reduced gravity due to drogue
        else:
            vel = target_vel
        alt += vel * dt_s
        if alt < 0:
            alt = 0
        # Under drogue: partial gravity reaction
        accel = [0.0, G * 0.7, 0.0]
        gyro = [0.05, 0.03, 0.02]  # swinging
        samples.append(sample(t, accel, gyro, alt))
        t += dt_s

    # ── Phase 6: Main chute (28-45s) — ~5 m/s ──
    target_vel = -5.0
    while t < 45.0:
        if vel < target_vel:
            vel += G * dt_s * 0.1  # brake to main rate
        else:
            vel = target_vel
        alt += vel * dt_s
        if alt < 0:
            alt = 0
            vel = 0
        accel = [0.0, G * 0.85, 0.0]
        gyro = [0.03, 0.02, 0.01]
        samples.append(sample(t, accel, gyro, alt))
        t += dt_s

    # ── Phase 7: Landed (45-50s) ──
    while t < 50.0:
        accel = [0.0, G, 0.0]
        gyro = [0.0, 0.0, 0.0]
        alt = 0.0
        vel = 0.0
        samples.append(sample(t, accel, gyro, alt))
        t += dt_s

    return samples


# ── Main ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="C.A.S.P.E.R.-2 Raw HIL Sender")
    parser.add_argument("--port", default="COM3", help="Serial port (default: COM3)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Time multiplier (1.0=real-time, 10.0=10x faster)")
    parser.add_argument("--no-plot", action="store_true",
                        help="Don't print plotter output")
    parser.add_argument("--dt", type=float, default=0.0012,
                        help="Sample interval in seconds (default: 0.0012 = 833Hz)")
    args = parser.parse_args()

    print(f"Generating flight profile (dt={args.dt*1000:.1f}ms)...")
    profile = generate_profile(dt_s=args.dt)
    duration_s = profile[-1][0] / 1000.0
    print(f"  {len(profile)} samples, {duration_s:.1f}s virtual time")

    print(f"Opening {args.port}...")
    ser = serial.Serial(args.port, timeout=0.01)
    time.sleep(0.5)  # let USB CDC settle

    # Flush any pending data
    ser.reset_input_buffer()

    print(f"Sending at {args.speed}x speed...")
    t_start = time.time()
    sent = 0
    last_status = 0

    try:
        for i, (tick_ms, accel, gyro, baro_pa, mag, baro_v, mag_v, skip) in enumerate(profile):
            pkt = build_packet(tick_ms, accel, gyro, baro_pa, mag, baro_v, mag_v, skip)
            ser.write(pkt)
            sent += 1

            # Read and print any plotter output
            while ser.in_waiting:
                try:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if line and not args.no_plot:
                        print(line)
                except:
                    pass

            # Pace sending based on speed multiplier
            if args.speed > 0:
                expected_elapsed = (tick_ms / 1000.0) / args.speed
                actual_elapsed = time.time() - t_start
                sleep_time = expected_elapsed - actual_elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # Status update every second
            if tick_ms // 1000 > last_status:
                last_status = tick_ms // 1000
                sys.stderr.write(f"\r  t={tick_ms/1000:.1f}s  sent={sent}  ")
                sys.stderr.flush()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")

    # Drain remaining output
    time.sleep(0.5)
    while ser.in_waiting:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line and not args.no_plot:
                print(line)
        except:
            pass

    elapsed = time.time() - t_start
    print(f"\n\nDone: {sent} packets in {elapsed:.1f}s "
          f"({sent/elapsed:.0f} pkt/s, {args.speed}x speed)")
    ser.close()


if __name__ == "__main__":
    main()
