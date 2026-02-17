#!/usr/bin/env python3
"""
decode_flight_log.py - C.A.S.P.E.R.-2 Flight Log Decoder

Reads the raw binary stream from a serial port or file, validates CRC,
decodes each 32-byte log entry, and outputs CSV.

Raw stream format:
  [4 bytes] Magic: "CASP" (0x43 0x41 0x53 0x50)
  [4 bytes] Entry count (u32 LE)
  [4 bytes] Entry size (u32 LE, should be 32)
  [4 bytes] CRC-32 of header (first 12 bytes)
  [N * 32 bytes] Log entries, sequential
  [4 bytes] CRC-32 of all entry data

Usage:
  python decode_flight_log.py --port COM3 --baud 115200 --output flight_data.csv
  python decode_flight_log.py --file raw_dump.bin --output flight_data.csv
"""

import argparse
import binascii
import csv
import struct
import sys
import time


# ---- Log entry format (32 bytes packed) ----
# [timestamp_us:4][fsm_state:1][flags:1][ax:2][ay:2][az:2]
# [gx:2][gy:2][gz:2][alt:2][vel:2][baro:2][tilt:2][batt:2][rsvd:4] = 32
ENTRY_FORMAT = "<IBBhhhhhhhhhhH4s"
ENTRY_SIZE = 32

# Header magic
MAGIC_CASP = b"CASP"
MAGIC_SUMM = b"SUMM"

# Header: [magic:4][count:4][entry_size:4][crc:4] = 16
HEADER_SIZE = 16

# FSM state names for human-readable output
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


def crc32(data: bytes) -> int:
    """Compute CRC-32 (IEEE 802.3) matching the firmware implementation.

    Uses the standard polynomial 0x04C11DB7 with reflected I/O and
    final XOR of 0xFFFFFFFF.  Python's binascii.crc32 produces the
    same result (unsigned).
    """
    return binascii.crc32(data) & 0xFFFFFFFF


def read_exact(source, n: int) -> bytes:
    """Read exactly n bytes from a source (file or serial port)."""
    data = b""
    while len(data) < n:
        remaining = n - len(data)
        chunk = source.read(remaining)
        if chunk is None or len(chunk) == 0:
            raise IOError(
                f"Unexpected end of stream: expected {n} bytes, got {len(data)}"
            )
        data += chunk
    return data


def decode_entry(raw: bytes) -> dict:
    """Decode a single 32-byte log entry into a dictionary."""
    if len(raw) != ENTRY_SIZE:
        raise ValueError(f"Entry must be {ENTRY_SIZE} bytes, got {len(raw)}")

    (
        timestamp_us,
        fsm_state,
        flags,
        accel_x_mg,
        accel_y_mg,
        accel_z_mg,
        gyro_x_mdps,
        gyro_y_mdps,
        gyro_z_mdps,
        alt_dm,
        vel_cmps,
        baro_alt_dm,
        tilt_cdeg,
        batt_mv,
        _reserved,
    ) = struct.unpack(ENTRY_FORMAT, raw)

    return {
        "timestamp_us": timestamp_us,
        "fsm_state": fsm_state,
        "flags": flags,
        "accel_x_mg": accel_x_mg,
        "accel_y_mg": accel_y_mg,
        "accel_z_mg": accel_z_mg,
        "gyro_x_mdps": gyro_x_mdps,
        "gyro_y_mdps": gyro_y_mdps,
        "gyro_z_mdps": gyro_z_mdps,
        "alt_dm": alt_dm,
        "vel_cmps": vel_cmps,
        "baro_alt_dm": baro_alt_dm,
        "tilt_cdeg": tilt_cdeg,
        "batt_mv": batt_mv,
    }


def decode_raw_stream(source, output_path: str, verbose: bool = False):
    """Decode a raw flight log stream and write CSV output."""

    # --- Read and validate header ---
    header_raw = read_exact(source, HEADER_SIZE)

    magic = header_raw[0:4]
    if magic != MAGIC_CASP:
        print(
            f"ERROR: Bad magic: expected {MAGIC_CASP!r}, got {magic!r}",
            file=sys.stderr,
        )
        sys.exit(1)

    entry_count = struct.unpack_from("<I", header_raw, 4)[0]
    entry_size = struct.unpack_from("<I", header_raw, 8)[0]
    header_crc_received = struct.unpack_from("<I", header_raw, 12)[0]

    # Validate header CRC (over first 12 bytes)
    header_crc_computed = crc32(header_raw[0:12])
    if header_crc_received != header_crc_computed:
        print(
            f"ERROR: Header CRC mismatch: received 0x{header_crc_received:08X}, "
            f"computed 0x{header_crc_computed:08X}",
            file=sys.stderr,
        )
        sys.exit(1)

    if entry_size != ENTRY_SIZE:
        print(
            f"WARNING: Entry size {entry_size} != expected {ENTRY_SIZE}",
            file=sys.stderr,
        )

    if verbose:
        print(f"Header OK: {entry_count} entries, {entry_size} bytes each")

    # --- Read all entry data ---
    total_data_bytes = entry_count * entry_size
    if total_data_bytes > 0:
        entry_data = read_exact(source, total_data_bytes)
    else:
        entry_data = b""

    # --- Read and validate data CRC ---
    data_crc_raw = read_exact(source, 4)
    data_crc_received = struct.unpack("<I", data_crc_raw)[0]

    if total_data_bytes > 0:
        data_crc_computed = crc32(entry_data)
    else:
        data_crc_computed = crc32(b"")

    if data_crc_received != data_crc_computed:
        print(
            f"ERROR: Data CRC mismatch: received 0x{data_crc_received:08X}, "
            f"computed 0x{data_crc_computed:08X}",
            file=sys.stderr,
        )
        sys.exit(1)

    if verbose:
        print(f"Data CRC OK: 0x{data_crc_computed:08X}")

    # --- Decode entries and write CSV ---
    csv_columns = [
        "timestamp_us",
        "fsm_state",
        "flags",
        "accel_x_mg",
        "accel_y_mg",
        "accel_z_mg",
        "gyro_x_mdps",
        "gyro_y_mdps",
        "gyro_z_mdps",
        "alt_dm",
        "vel_cmps",
        "baro_alt_dm",
        "tilt_cdeg",
        "batt_mv",
    ]

    with open(output_path, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=csv_columns)
        writer.writeheader()

        for i in range(entry_count):
            offset = i * entry_size
            raw_entry = entry_data[offset : offset + entry_size]
            entry = decode_entry(raw_entry)
            writer.writerow(entry)

    print(f"Decoded {entry_count} entries -> {output_path}")


def decode_summary_stream(source, verbose: bool = False):
    """Decode a summary stream and print events to stdout."""

    # Read header: [magic:4][total_size:4] = 8
    header_raw = read_exact(source, 8)

    magic = header_raw[0:4]
    if magic != MAGIC_SUMM:
        print(
            f"ERROR: Bad magic: expected {MAGIC_SUMM!r}, got {magic!r}",
            file=sys.stderr,
        )
        sys.exit(1)

    total_size = struct.unpack_from("<I", header_raw, 4)[0]

    if verbose:
        print(f"Summary header OK: {total_size} bytes of data")

    # Read all summary data
    if total_size > 0:
        summary_data = read_exact(source, total_size)
    else:
        summary_data = b""

    # Read and validate CRC
    crc_raw = read_exact(source, 4)
    crc_received = struct.unpack("<I", crc_raw)[0]

    if total_size > 0:
        crc_computed = crc32(summary_data)
    else:
        crc_computed = crc32(b"")

    if crc_received != crc_computed:
        print(
            f"ERROR: Summary CRC mismatch: received 0x{crc_received:08X}, "
            f"computed 0x{crc_computed:08X}",
            file=sys.stderr,
        )
        sys.exit(1)

    if verbose:
        print(f"Summary CRC OK: 0x{crc_computed:08X}")

    # Parse summary entries
    # Each entry: [timestamp_ms:4][type:1][len:1][text:128] = 134 bytes
    SUMMARY_ENTRY_SIZE = 134  # 4 + 1 + 1 + 128
    offset = 0
    entry_num = 0

    while offset + SUMMARY_ENTRY_SIZE <= total_size:
        ts_ms = struct.unpack_from("<I", summary_data, offset)[0]
        evt_type = summary_data[offset + 4]
        text_len = summary_data[offset + 5]
        text_raw = summary_data[offset + 6 : offset + 6 + text_len]

        try:
            text = text_raw.decode("utf-8", errors="replace")
        except Exception:
            text = repr(text_raw)

        print(f"  [{entry_num}] t={ts_ms}ms type={evt_type}: {text}")
        entry_num += 1
        offset += SUMMARY_ENTRY_SIZE

    print(f"Total summary events: {entry_num}")


def open_source(args):
    """Open the data source (serial port or file)."""
    if args.file:
        return open(args.file, "rb")
    elif args.port:
        try:
            import serial
        except ImportError:
            print(
                "ERROR: pyserial is required for serial port reading. "
                "Install with: pip install pyserial",
                file=sys.stderr,
            )
            sys.exit(1)

        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=10,
        )
        # Give the device time to start streaming
        time.sleep(0.5)
        return ser
    else:
        print("ERROR: Must specify either --port or --file", file=sys.stderr)
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="C.A.S.P.E.R.-2 Flight Log Decoder"
    )

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--port", help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    group.add_argument("--file", help="Binary file to decode")

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for serial (default: 115200)",
    )
    parser.add_argument(
        "--output",
        default="flight_data.csv",
        help="Output CSV file (default: flight_data.csv)",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Decode summary stream instead of raw log",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print verbose status messages",
    )

    args = parser.parse_args()

    source = open_source(args)

    try:
        if args.summary:
            decode_summary_stream(source, verbose=args.verbose)
        else:
            decode_raw_stream(source, args.output, verbose=args.verbose)
    finally:
        source.close()


if __name__ == "__main__":
    main()
