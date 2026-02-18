#!/usr/bin/env python3
"""
decode_flight_log.py - C.A.S.P.E.R.-2 Flight Log Decoder (v2)

Decodes high-rate, low-rate, and summary flight logs from CASPER-2
flight firmware.  Supports reading from a serial port (sending readout
commands) or from a raw binary file.

Readout protocol:
  HR/LR header: [CASP:4][stream_id:1][entry_size:1][rsvd:2][count:4][CRC:4] = 16
  HR/LR data:   [N * 64-byte entries]
  HR/LR footer:  [data_CRC:4]

  Summary header: [SUMM:4][payload_size:4][CRC:4] = 12
  Summary data:   [payload bytes]
  Summary footer:  [data_CRC:4]

  Metadata:       [META:4][hr_count:4][lr_count:4][sum_bytes:4]
                  [hr_addr:4][lr_addr:4][CRC:4] = 28

Usage:
  python decode_flight_log.py --port COM3 --type hr --output hr.csv
  python decode_flight_log.py --port COM3 --type lr --output lr.csv
  python decode_flight_log.py --port COM3 --type summary
  python decode_flight_log.py --port COM3 --type meta
  python decode_flight_log.py --port COM3 --type all --output flight
  python decode_flight_log.py --file hr_dump.bin --type hr --output hr.csv
"""

import argparse
import csv
import math
import struct
import sys
import time
import zlib


# ---- Readout command bytes ----
READOUT_CMD_HR       = 0x01
READOUT_CMD_LR       = 0x02
READOUT_CMD_SUMMARY  = 0x03
READOUT_CMD_METADATA = 0x04
READOUT_CMD_ERASE    = 0x05

# ---- Entry sizes ----
HR_ENTRY_SIZE = 64
LR_ENTRY_SIZE = 64

# ---- Struct formats (little-endian, packed) ----

# highrate_entry_t (64 bytes):
#   timestamp_us     u32
#   fresh            u8
#   fsm_state        u8
#   accel_mg[3]      3 * i16
#   gyro_raw[3]      3 * i16
#   imu_temp_c100    i16
#   highg_10mg[3]    3 * i16
#   baro_pa          u32
#   baro_temp_c100   i16
#   ekf_alt_cm       i32
#   ekf_vel_cmps     i32
#   ekf_abias_mmps2  i16
#   ekf_bbias_cm     i16
#   quat_packed[3]   3 * i16
#   tilt_cdeg        i16
#   flags            u8
#   reserved         11 bytes
# Format: I BB hhh hhh h hhh I h ii hh hhh h B 11s = 64
HR_FORMAT = "<IBBhhhhhhhhhhIhiihhhhhhB11s"

# lowrate_entry_t (64 bytes):
#   timestamp_us     u32          (4)
#   fsm_state        u8           (1)
#   flags            u8           (1)
#   mag_raw[3]       3 * i16      (6)
#   mag_temp_c100    i16          (2)
#   batt_mv          u16          (2)
#   batt_ma          i16          (2)
#   cont_scaled[4]   4 * u8       (4)
#   gps_lat_deg7     i32          (4)
#   gps_lon_deg7     i32          (4)
#   gps_alt_dm       i16          (2)
#   gps_vel_d_cmps   i16          (2)
#   gps_sats         u8           (1)
#   gps_fix          u8           (1)
#   gps_pdop         u8           (1)
#   gps_fresh        u8           (1)
#   radio_tx_seq     u8           (1)
#   radio_rx_good    u8           (1)
#   radio_rx_bad     u8           (1)
#   radio_rssi       i8           (1)
#   radio_snr        i8           (1)
#   pyro_arm_cont    u8           (1)
#   reserved         20 bytes     (20)
# Total: 4+1+1+6+2+2+2+4+4+4+2+2+1+1+1+1+1+1+1+1+1+1+20 = 64
LR_FORMAT = "<IBBhhhhHhBBBBiihhBBBBBBBbbB20s"

# ---- FSM state names ----
FSM_STATE_NAMES = {
    0x0: "PAD",
    0x1: "BOOST",
    0x2: "COAST",
    0x3: "COAST_1",
    0x4: "SUSTAIN",
    0x5: "COAST_2",
    0x6: "APOGEE",
    0x7: "DROGUE",
    0x8: "MAIN",
    0x9: "RECOVERY",
    0xA: "TUMBLE",
    0xB: "LANDED",
}

# ---- CRC-32 ----
def crc32(data):
    """Compute standard CRC-32 matching firmware (IEEE 802.3)."""
    return zlib.crc32(data) & 0xFFFFFFFF


# ---- IO helpers ----
def read_exact(source, n):
    """Read exactly n bytes from a source (file or serial port)."""
    data = b""
    while len(data) < n:
        chunk = source.read(n - len(data))
        if chunk is None or len(chunk) == 0:
            raise IOError(
                f"Unexpected end of stream: expected {n} bytes, got {len(data)}"
            )
        data += chunk
    return data


# ---- Quaternion unpacking ----
def unpack_quaternion_smallest_three(packed):
    """
    Unpack quaternion from Agent A's simplified packing:
    w is always the dropped component (forced positive),
    packed[0..2] = Q14-scaled x, y, z.
    """
    x = packed[0] / 16384.0
    y = packed[1] / 16384.0
    z = packed[2] / 16384.0
    w_sq = 1.0 - x*x - y*y - z*z
    w = math.sqrt(max(0.0, w_sq))
    return (w, x, y, z)


# ---- HR entry decoding ----
def decode_hr_entry(raw):
    """Decode a 64-byte high-rate entry into a dict with derived fields."""
    if len(raw) != HR_ENTRY_SIZE:
        raise ValueError(f"HR entry must be {HR_ENTRY_SIZE} bytes, got {len(raw)}")

    fields = struct.unpack(HR_FORMAT, raw)
    idx = 0

    timestamp_us = fields[idx]; idx += 1
    fresh        = fields[idx]; idx += 1
    fsm_state    = fields[idx]; idx += 1
    accel_mg     = (fields[idx], fields[idx+1], fields[idx+2]); idx += 3
    gyro_raw     = (fields[idx], fields[idx+1], fields[idx+2]); idx += 3
    imu_temp_c100 = fields[idx]; idx += 1
    highg_10mg   = (fields[idx], fields[idx+1], fields[idx+2]); idx += 3
    baro_pa      = fields[idx]; idx += 1
    baro_temp_c100 = fields[idx]; idx += 1
    ekf_alt_cm   = fields[idx]; idx += 1
    ekf_vel_cmps = fields[idx]; idx += 1
    ekf_abias_mmps2 = fields[idx]; idx += 1
    ekf_bbias_cm = fields[idx]; idx += 1
    quat_packed  = (fields[idx], fields[idx+1], fields[idx+2]); idx += 3
    tilt_cdeg    = fields[idx]; idx += 1
    flags        = fields[idx]; idx += 1
    # reserved   = fields[idx]

    # Derived values
    w, x, y, z = unpack_quaternion_smallest_three(quat_packed)

    return {
        "timestamp_us":   timestamp_us,
        "timestamp_s":    timestamp_us / 1e6,
        "fresh":          fresh,
        "fresh_imu":      bool(fresh & 0x01),
        "fresh_highg":    bool(fresh & 0x02),
        "fresh_baro":     bool(fresh & 0x04),
        "fsm_state":      fsm_state,
        "fsm_name":       FSM_STATE_NAMES.get(fsm_state, f"UNKNOWN({fsm_state})"),
        "accel_x_mg":     accel_mg[0],
        "accel_y_mg":     accel_mg[1],
        "accel_z_mg":     accel_mg[2],
        "accel_x_g":      accel_mg[0] / 1000.0,
        "accel_y_g":      accel_mg[1] / 1000.0,
        "accel_z_g":      accel_mg[2] / 1000.0,
        "gyro_x_raw":     gyro_raw[0],
        "gyro_y_raw":     gyro_raw[1],
        "gyro_z_raw":     gyro_raw[2],
        "gyro_x_dps":     gyro_raw[0] * (2000.0 / 32768.0),
        "gyro_y_dps":     gyro_raw[1] * (2000.0 / 32768.0),
        "gyro_z_dps":     gyro_raw[2] * (2000.0 / 32768.0),
        "imu_temp_c":     imu_temp_c100 / 100.0,
        "highg_x_10mg":   highg_10mg[0],
        "highg_y_10mg":   highg_10mg[1],
        "highg_z_10mg":   highg_10mg[2],
        "highg_x_g":      highg_10mg[0] / 100.0,
        "highg_y_g":      highg_10mg[1] / 100.0,
        "highg_z_g":      highg_10mg[2] / 100.0,
        "baro_pa":        baro_pa,
        "baro_hpa":       baro_pa / 100.0,
        "baro_temp_c":    baro_temp_c100 / 100.0,
        "ekf_alt_cm":     ekf_alt_cm,
        "ekf_alt_m":      ekf_alt_cm / 100.0,
        "ekf_vel_cmps":   ekf_vel_cmps,
        "ekf_vel_mps":    ekf_vel_cmps / 100.0,
        "ekf_abias_mmps2": ekf_abias_mmps2,
        "ekf_abias_mps2": ekf_abias_mmps2 / 1000.0,
        "ekf_bbias_cm":   ekf_bbias_cm,
        "ekf_bbias_m":    ekf_bbias_cm / 100.0,
        "quat_w":         w,
        "quat_x":         x,
        "quat_y":         y,
        "quat_z":         z,
        "tilt_cdeg":      tilt_cdeg,
        "tilt_deg":       tilt_cdeg / 100.0,
        "flags":          flags,
        "flag_baro_gated": bool(flags & 0x01),
        "flag_launched":   bool(flags & 0x02),
        "flag_mag_valid":  bool(flags & 0x04),
    }


# ---- LR entry decoding ----
def decode_lr_entry(raw):
    """Decode a 64-byte low-rate entry into a dict with derived fields."""
    if len(raw) != LR_ENTRY_SIZE:
        raise ValueError(f"LR entry must be {LR_ENTRY_SIZE} bytes, got {len(raw)}")

    fields = struct.unpack(LR_FORMAT, raw)
    idx = 0

    timestamp_us   = fields[idx]; idx += 1
    fsm_state      = fields[idx]; idx += 1
    flags          = fields[idx]; idx += 1
    mag_raw        = (fields[idx], fields[idx+1], fields[idx+2]); idx += 3
    mag_temp_c100  = fields[idx]; idx += 1
    batt_mv        = fields[idx]; idx += 1
    batt_ma        = fields[idx]; idx += 1
    cont_scaled    = (fields[idx], fields[idx+1], fields[idx+2], fields[idx+3]); idx += 4
    gps_lat_deg7   = fields[idx]; idx += 1
    gps_lon_deg7   = fields[idx]; idx += 1
    gps_alt_dm     = fields[idx]; idx += 1
    gps_vel_d_cmps = fields[idx]; idx += 1
    gps_sats       = fields[idx]; idx += 1
    gps_fix        = fields[idx]; idx += 1
    gps_pdop       = fields[idx]; idx += 1
    gps_fresh      = fields[idx]; idx += 1
    radio_tx_seq   = fields[idx]; idx += 1
    radio_rx_good  = fields[idx]; idx += 1
    radio_rx_bad   = fields[idx]; idx += 1
    radio_rssi     = fields[idx]; idx += 1
    radio_snr      = fields[idx]; idx += 1
    pyro_arm_cont  = fields[idx]; idx += 1
    # reserved     = fields[idx]

    return {
        "timestamp_us":     timestamp_us,
        "timestamp_s":      timestamp_us / 1e6,
        "fsm_state":        fsm_state,
        "fsm_name":         FSM_STATE_NAMES.get(fsm_state, f"UNKNOWN({fsm_state})"),
        "flags":            flags,
        "flag_firing":      bool(flags & 0x01),
        "flag_test_mode":   bool(flags & 0x02),
        "flag_sim_active":  bool(flags & 0x04),
        "mag_x_raw":        mag_raw[0],
        "mag_y_raw":        mag_raw[1],
        "mag_z_raw":        mag_raw[2],
        "mag_temp_c":       mag_temp_c100 / 100.0 if mag_temp_c100 != 0x7FFF else None,
        "batt_mv":          batt_mv,
        "batt_v":           batt_mv / 1000.0,
        "batt_ma":          batt_ma,
        "cont_scaled":      list(cont_scaled),
        "gps_lat_deg7":     gps_lat_deg7,
        "gps_lon_deg7":     gps_lon_deg7,
        "gps_lat_deg":      gps_lat_deg7 / 1e7,
        "gps_lon_deg":      gps_lon_deg7 / 1e7,
        "gps_alt_dm":       gps_alt_dm,
        "gps_alt_m":        gps_alt_dm / 10.0,
        "gps_vel_d_cmps":   gps_vel_d_cmps,
        "gps_vel_d_mps":    gps_vel_d_cmps / 100.0,
        "gps_sats":         gps_sats,
        "gps_fix":          gps_fix,
        "gps_pdop":         gps_pdop,
        "gps_fresh":        bool(gps_fresh),
        "radio_tx_seq":     radio_tx_seq,
        "radio_rx_good":    radio_rx_good,
        "radio_rx_bad":     radio_rx_bad,
        "radio_rssi":       radio_rssi,
        "radio_snr":        radio_snr,
        "pyro_arm_bitmap":  (pyro_arm_cont >> 4) & 0x0F,
        "pyro_cont_bitmap": pyro_arm_cont & 0x0F,
    }


# ---- Summary decoding ----
def decode_summary_entries(data):
    """Parse variable-length summary entries: [timestamp_ms:4][len:1][msg:N]"""
    entries = []
    offset = 0
    while offset < len(data):
        if offset + 5 > len(data):
            break
        ts_ms = struct.unpack_from("<I", data, offset)[0]
        msg_len = data[offset + 4]
        if ts_ms == 0xFFFFFFFF:
            break  # erased region
        if offset + 5 + msg_len > len(data):
            break
        msg = data[offset + 5 : offset + 5 + msg_len].decode("utf-8", errors="replace")
        entries.append({"timestamp_ms": ts_ms, "timestamp_s": ts_ms / 1000.0, "msg": msg})
        offset += 5 + msg_len
    return entries


# ---- CSV writers ----
HR_CSV_COLUMNS = [
    "timestamp_us", "timestamp_s", "fsm_name",
    "accel_x_g", "accel_y_g", "accel_z_g",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "imu_temp_c",
    "highg_x_g", "highg_y_g", "highg_z_g",
    "baro_hpa", "baro_temp_c",
    "ekf_alt_m", "ekf_vel_mps", "ekf_abias_mps2", "ekf_bbias_m",
    "quat_w", "quat_x", "quat_y", "quat_z", "tilt_deg",
    "flag_baro_gated", "flag_launched",
    "fresh_imu", "fresh_highg", "fresh_baro",
]

LR_CSV_COLUMNS = [
    "timestamp_us", "timestamp_s", "fsm_name",
    "mag_x_raw", "mag_y_raw", "mag_z_raw", "mag_temp_c",
    "batt_v", "batt_ma",
    "cont_scaled",
    "gps_lat_deg", "gps_lon_deg", "gps_alt_m", "gps_vel_d_mps",
    "gps_sats", "gps_fix", "gps_fresh",
    "radio_tx_seq", "radio_rssi", "radio_snr",
    "pyro_arm_bitmap", "pyro_cont_bitmap",
    "flag_firing", "flag_test_mode", "flag_sim_active",
]


def write_hr_csv(entries, path):
    """Write high-rate entries to CSV."""
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=HR_CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for e in entries:
            writer.writerow(e)
    print(f"  Wrote {len(entries)} HR entries -> {path}")


def write_lr_csv(entries, path):
    """Write low-rate entries to CSV."""
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=LR_CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for e in entries:
            writer.writerow(e)
    print(f"  Wrote {len(entries)} LR entries -> {path}")


# ---- Stream readers ----

def read_hr_lr_stream(source, verbose=False):
    """
    Read an HR or LR stream.
    Returns (stream_id, entry_size, entries_raw_list).
    """
    # Header: [CASP:4][stream_id:1][entry_size:1][rsvd:2][count:4][CRC:4] = 16
    header = read_exact(source, 16)
    magic = header[0:4]
    if magic != b"CASP":
        raise ValueError(f"Bad magic: expected b'CASP', got {magic!r}")

    stream_id  = header[4]
    entry_size = header[5]
    count      = struct.unpack_from("<I", header, 8)[0]
    hdr_crc_rx = struct.unpack_from("<I", header, 12)[0]

    hdr_crc_calc = crc32(header[0:12])
    if hdr_crc_rx != hdr_crc_calc:
        raise ValueError(
            f"Header CRC mismatch: received 0x{hdr_crc_rx:08X}, "
            f"computed 0x{hdr_crc_calc:08X}"
        )

    stream_name = {0x01: "HR", 0x02: "LR"}.get(stream_id, f"0x{stream_id:02X}")
    if verbose:
        print(f"  Header OK: stream={stream_name}, entry_size={entry_size}, count={count}")

    # Data
    total_bytes = count * entry_size
    data = read_exact(source, total_bytes) if total_bytes > 0 else b""

    # Data CRC
    data_crc_rx = struct.unpack("<I", read_exact(source, 4))[0]
    data_crc_calc = crc32(data) if data else crc32(b"")
    if data_crc_rx != data_crc_calc:
        raise ValueError(
            f"Data CRC mismatch: received 0x{data_crc_rx:08X}, "
            f"computed 0x{data_crc_calc:08X}"
        )
    if verbose:
        print(f"  Data CRC OK: 0x{data_crc_calc:08X}")

    # Split into entries
    entries_raw = []
    for i in range(count):
        entries_raw.append(data[i * entry_size : (i + 1) * entry_size])

    return stream_id, entry_size, entries_raw


def read_summary_stream(source, verbose=False):
    """
    Read a summary stream.
    Returns list of summary entry dicts.
    """
    # Header: [SUMM:4][payload_size:4][CRC:4] = 12
    header = read_exact(source, 12)
    magic = header[0:4]
    if magic != b"SUMM":
        raise ValueError(f"Bad magic: expected b'SUMM', got {magic!r}")

    payload_size = struct.unpack_from("<I", header, 4)[0]
    hdr_crc_rx   = struct.unpack_from("<I", header, 8)[0]

    hdr_crc_calc = crc32(header[0:8])
    if hdr_crc_rx != hdr_crc_calc:
        raise ValueError(
            f"Header CRC mismatch: received 0x{hdr_crc_rx:08X}, "
            f"computed 0x{hdr_crc_calc:08X}"
        )

    if verbose:
        print(f"  Summary header OK: {payload_size} bytes")

    # Data
    data = read_exact(source, payload_size) if payload_size > 0 else b""

    # Data CRC
    data_crc_rx = struct.unpack("<I", read_exact(source, 4))[0]
    data_crc_calc = crc32(data) if data else crc32(b"")
    if data_crc_rx != data_crc_calc:
        raise ValueError(
            f"Data CRC mismatch: received 0x{data_crc_rx:08X}, "
            f"computed 0x{data_crc_calc:08X}"
        )
    if verbose:
        print(f"  Summary CRC OK: 0x{data_crc_calc:08X}")

    return decode_summary_entries(data)


def read_metadata(source, verbose=False):
    """
    Read a metadata response.
    Returns dict with hr_count, lr_count, summary_bytes, hr_addr, lr_addr.
    """
    # [META:4][hr_count:4][lr_count:4][sum_bytes:4][hr_addr:4][lr_addr:4][CRC:4] = 28
    resp = read_exact(source, 28)
    magic = resp[0:4]
    if magic != b"META":
        raise ValueError(f"Bad magic: expected b'META', got {magic!r}")

    hr_count   = struct.unpack_from("<I", resp, 4)[0]
    lr_count   = struct.unpack_from("<I", resp, 8)[0]
    sum_bytes  = struct.unpack_from("<I", resp, 12)[0]
    hr_addr    = struct.unpack_from("<I", resp, 16)[0]
    lr_addr    = struct.unpack_from("<I", resp, 20)[0]
    crc_rx     = struct.unpack_from("<I", resp, 24)[0]

    crc_calc = crc32(resp[0:24])
    if crc_rx != crc_calc:
        raise ValueError(
            f"Metadata CRC mismatch: received 0x{crc_rx:08X}, "
            f"computed 0x{crc_calc:08X}"
        )

    meta = {
        "hr_count":      hr_count,
        "lr_count":      lr_count,
        "summary_bytes": sum_bytes,
        "hr_addr":       hr_addr,
        "lr_addr":       lr_addr,
    }

    if verbose:
        print(f"  Metadata CRC OK")

    return meta


# ---- Serial port helpers ----

def open_serial(port, baud):
    """Open a serial port for readout."""
    try:
        import serial
    except ImportError:
        print(
            "ERROR: pyserial is required for serial port reading. "
            "Install with: pip install pyserial",
            file=sys.stderr,
        )
        sys.exit(1)

    ser = serial.Serial(port=port, baudrate=baud, timeout=10)
    time.sleep(0.5)
    return ser


def send_command(ser, cmd_byte):
    """Send a single readout command byte."""
    ser.write(bytes([cmd_byte]))
    ser.flush()
    time.sleep(0.1)


# ---- High-level operations ----

def do_hr(source, output_path, verbose, is_serial):
    """Read and decode high-rate log."""
    if is_serial:
        send_command(source, READOUT_CMD_HR)

    print("Reading high-rate log...")
    _, _, entries_raw = read_hr_lr_stream(source, verbose)
    entries = [decode_hr_entry(raw) for raw in entries_raw]
    print(f"  Decoded {len(entries)} high-rate entries")

    if output_path:
        write_hr_csv(entries, output_path)
    return entries


def do_lr(source, output_path, verbose, is_serial):
    """Read and decode low-rate log."""
    if is_serial:
        send_command(source, READOUT_CMD_LR)

    print("Reading low-rate log...")
    _, _, entries_raw = read_hr_lr_stream(source, verbose)
    entries = [decode_lr_entry(raw) for raw in entries_raw]
    print(f"  Decoded {len(entries)} low-rate entries")

    if output_path:
        write_lr_csv(entries, output_path)
    return entries


def do_summary(source, verbose, is_serial):
    """Read and decode summary log."""
    if is_serial:
        send_command(source, READOUT_CMD_SUMMARY)

    print("Reading summary log...")
    entries = read_summary_stream(source, verbose)
    print(f"  Summary entries: {len(entries)}")
    for i, e in enumerate(entries):
        print(f"    [{i}] t={e['timestamp_s']:.3f}s: {e['msg']}")
    return entries


def do_metadata(source, verbose, is_serial):
    """Read and display metadata."""
    if is_serial:
        send_command(source, READOUT_CMD_METADATA)

    print("Reading metadata...")
    meta = read_metadata(source, verbose)
    print(f"  HR entries:     {meta['hr_count']}")
    print(f"  LR entries:     {meta['lr_count']}")
    print(f"  Summary bytes:  {meta['summary_bytes']}")
    print(f"  HR flash addr:  0x{meta['hr_addr']:08X}")
    print(f"  LR flash addr:  0x{meta['lr_addr']:08X}")
    return meta


# ---- Main ----

def main():
    parser = argparse.ArgumentParser(
        description="C.A.S.P.E.R.-2 Flight Log Decoder v2"
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--port", help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    group.add_argument("--file", help="Binary file to decode")

    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate for serial (default: 115200)",
    )
    parser.add_argument(
        "--type", choices=["hr", "lr", "summary", "meta", "all"],
        default="hr",
        help="Log type to decode (default: hr)",
    )
    parser.add_argument(
        "--output", "-o",
        help="Output CSV path (or base name for --type=all)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Print verbose status messages",
    )

    args = parser.parse_args()

    is_serial = args.port is not None

    if is_serial:
        source = open_serial(args.port, args.baud)
    else:
        source = open(args.file, "rb")

    try:
        if args.type == "hr":
            output = args.output or "hr_data.csv"
            do_hr(source, output, args.verbose, is_serial)

        elif args.type == "lr":
            output = args.output or "lr_data.csv"
            do_lr(source, output, args.verbose, is_serial)

        elif args.type == "summary":
            do_summary(source, args.verbose, is_serial)

        elif args.type == "meta":
            do_metadata(source, args.verbose, is_serial)

        elif args.type == "all":
            base = args.output or "flight"
            do_metadata(source, args.verbose, is_serial)
            do_hr(source, f"{base}_hr.csv", args.verbose, is_serial)
            do_lr(source, f"{base}_lr.csv", args.verbose, is_serial)
            do_summary(source, args.verbose, is_serial)

    except (IOError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        source.close()


if __name__ == "__main__":
    main()
