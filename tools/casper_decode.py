#!/usr/bin/env python3
"""
C.A.S.P.E.R.-2 Flight Logger Decoder

Reads a raw 64MB flash dump and decodes flight index, summaries,
HR/LR/ADXL records with Hamming SECDED + CRC integrity validation.

Usage:
  python casper_decode.py flash_dump.bin [--flight N] [--csv] [--verbose]
"""
import argparse
import csv
import struct
import sys
import zlib
from collections import Counter
from pathlib import Path

# ═══════════════════════════════════════════════════════════════════════
#  Flash layout constants (must match log_types.h)
# ═══════════════════════════════════════════════════════════════════════

FLASH_INDEX_BASE   = 0x00000000
FLASH_INDEX_SIZE   = 0x00001000
FLASH_SUMMARY_BASE = 0x00001000
FLASH_SUMMARY_SIZE = 0x00010000
FLASH_LR_BASE      = 0x00011000
FLASH_LR_SIZE      = 0x00080000
FLASH_ADXL_BASE    = 0x00091000
FLASH_ADXL_SIZE    = 0x01000000
FLASH_HR_BASE      = 0x01091000
LOG_FLASH_END      = 0x04000000

PAGE_SIZE   = 256
MAX_FLIGHTS = 128
INDEX_DIRTY = 0xFFFFFFFF
SUMMARY_MAGIC = 0x43535052  # "CSPR"

HR_REC_SIZE   = 64
LR_REC_SIZE   = 32
ADXL_REC_SIZE = 10
HR_PER_PAGE   = 4
LR_PER_PAGE   = 8
ADXL_PER_PAGE = 25

FSM_NAMES = {
    0: "PAD", 1: "BOOST", 2: "COAST", 3: "COAST_1", 4: "SUSTAIN",
    5: "COAST_2", 6: "APOGEE", 7: "DROGUE", 8: "MAIN", 9: "RECOVERY",
    10: "TUMBLE", 11: "LANDED",
}

# ═══════════════════════════════════════════════════════════════════════
#  CRC16-CCITT (poly 0x1021, init 0xFFFF) — matches hamming.c
# ═══════════════════════════════════════════════════════════════════════

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ═══════════════════════════════════════════════════════════════════════
#  Hamming SECDED — port of hamming.c
# ═══════════════════════════════════════════════════════════════════════

def _is_power_of_2(n):
    return n > 0 and (n & (n - 1)) == 0

def _data_bit_to_pos(data_idx):
    """Map 0-based data bit index to 1-based Hamming position."""
    pos = 1
    count = 0
    while True:
        if not _is_power_of_2(pos):
            if count == data_idx:
                return pos
            count += 1
        pos += 1

def hamming_encode(data: bytes) -> int:
    """Compute 10-bit Hamming SECDED over data bytes. Returns uint16."""
    total_bits = len(data) * 8
    syndrome = 0
    overall = 0
    for i in range(total_bits):
        bit = (data[i >> 3] >> (i & 7)) & 1
        if bit:
            pos = _data_bit_to_pos(i)
            syndrome ^= pos
            overall ^= 1
    syn_parity = 0
    for j in range(9):
        syn_parity ^= (syndrome >> j) & 1
    overall ^= syn_parity
    return (syndrome & 0x1FF) | ((overall & 1) << 9)

# ═══════════════════════════════════════════════════════════════════════
#  Struct parsers
# ═══════════════════════════════════════════════════════════════════════

def parse_index_entry(data):
    """Parse 32-byte flight_index_entry_t (packed)."""
    return {
        "flight_id":      struct.unpack_from("<H", data, 0)[0],
        "start_tick_ms":  struct.unpack_from("<I", data, 2)[0],
        "end_tick_ms":    struct.unpack_from("<I", data, 6)[0],
        "hr_start_addr":  struct.unpack_from("<I", data, 10)[0],
        "hr_end_addr":    struct.unpack_from("<I", data, 14)[0],
        "lr_start_addr":  struct.unpack_from("<I", data, 18)[0],
        "lr_end_addr":    struct.unpack_from("<I", data, 22)[0],
        "adxl_start_addr":struct.unpack_from("<I", data, 26)[0],
        "flags":          struct.unpack_from("<H", data, 30)[0],
    }

def parse_hr_record(data):
    """Parse 64-byte hr_record_t (packed)."""
    return {
        "timestamp_ms":   struct.unpack_from("<I", data, 0)[0],
        "baro_pressure":  struct.unpack_from("<H", data, 4)[0],
        "lsm6_accel":     struct.unpack_from("<3h", data, 6),
        "lsm6_gyro":      struct.unpack_from("<3h", data, 12),
        "adxl372":        struct.unpack_from("<3h", data, 18),
        "mmc":            struct.unpack_from("<3H", data, 24),
        "ekf_alt_m":      struct.unpack_from("<f", data, 30)[0],
        "ekf_vel_mps":    struct.unpack_from("<f", data, 34)[0],
        "quat_packed":    data[38:43],
        "fsm_state":      data[43],
        "flags":          data[44],
        "ekf_accel_bias": struct.unpack_from("<h", data, 45)[0],
        "ekf_baro_bias":  struct.unpack_from("<h", data, 47)[0],
        "imu_temp":       struct.unpack_from("<h", data, 49)[0],
        "baro_temp":      struct.unpack_from("<h", data, 51)[0],
        "seq_num":        struct.unpack_from("<H", data, 53)[0],
        "sustain_ms":     struct.unpack_from("<H", data, 55)[0],
        "hamming":        struct.unpack_from("<H", data, 60)[0],
        "crc16":          struct.unpack_from("<H", data, 62)[0],
    }

def parse_lr_record(data):
    """Parse 32-byte lr_record_t (packed)."""
    return {
        "timestamp_ms":     struct.unpack_from("<I", data, 0)[0],
        "pyro_state":       data[4],
        "pyro_cont_adc":    struct.unpack_from("<4H", data, 5),
        "radio_rssi":       struct.unpack_from("<b", data, 13)[0],
        "radio_snr":        struct.unpack_from("<b", data, 14)[0],
        "radio_tx_count":   struct.unpack_from("<H", data, 15)[0],
        "radio_rx_count":   struct.unpack_from("<H", data, 17)[0],
        "radio_fail_count": struct.unpack_from("<H", data, 19)[0],
        "gps_dlat_mm":      struct.unpack_from("<i", data, 21)[0],
        "gps_dlon_mm":      struct.unpack_from("<i", data, 25)[0],
        "gps_alt_msl_m":    struct.unpack_from("<h", data, 29)[0],
        "gps_fix_sats":     data[31],
    }

def parse_adxl_record(data):
    """Parse 10-byte adxl_record_t (packed)."""
    return {
        "timestamp_ms": struct.unpack_from("<I", data, 0)[0],
        "accel_x":      struct.unpack_from("<h", data, 4)[0],
        "accel_y":      struct.unpack_from("<h", data, 6)[0],
        "accel_z":      struct.unpack_from("<h", data, 8)[0],
    }

def parse_summary(data):
    """Parse 256-byte flight_summary_t (packed)."""
    s = {}
    s["magic"]              = struct.unpack_from("<I", data, 0)[0]
    s["format_version"]     = data[4]
    s["summary_type"]       = data[5]
    s["flags"]              = struct.unpack_from("<H", data, 6)[0]
    # Timing
    s["launch_tick"]        = struct.unpack_from("<I", data, 8)[0]
    s["burnout1_tick"]      = struct.unpack_from("<I", data, 12)[0]
    s["burnout2_tick"]      = struct.unpack_from("<I", data, 16)[0]
    s["apogee_tick"]        = struct.unpack_from("<I", data, 20)[0]
    s["main_deploy_tick"]   = struct.unpack_from("<I", data, 24)[0]
    s["landing_tick"]       = struct.unpack_from("<I", data, 28)[0]
    s["total_flight_tick"]  = struct.unpack_from("<I", data, 32)[0]
    # Derived
    s["time_to_apogee"]     = struct.unpack_from("<I", data, 36)[0]
    s["peak_vel_tick"]      = struct.unpack_from("<I", data, 40)[0]
    s["max_q_tick"]         = struct.unpack_from("<I", data, 44)[0]
    # Peak dynamics
    s["peak_accel_boost_g"] = struct.unpack_from("<f", data, 48)[0]
    s["peak_accel_sustain_g"] = struct.unpack_from("<f", data, 52)[0]
    s["peak_velocity_mps"]  = struct.unpack_from("<f", data, 56)[0]
    s["peak_mach"]          = struct.unpack_from("<f", data, 60)[0]
    s["max_q_pa"]           = struct.unpack_from("<f", data, 64)[0]
    s["max_q_alt_m"]        = struct.unpack_from("<f", data, 68)[0]
    # Apogee
    s["apogee_ekf_m"]       = struct.unpack_from("<f", data, 72)[0]
    s["apogee_baro_m"]      = struct.unpack_from("<f", data, 76)[0]
    s["apogee_gps_msl_m"]   = struct.unpack_from("<f", data, 80)[0]
    # Attitude
    s["pad_tilt_deg"]       = struct.unpack_from("<f", data, 84)[0]
    s["max_tilt_deg"]       = struct.unpack_from("<f", data, 88)[0]
    s["max_roll_rate_dps"]  = struct.unpack_from("<f", data, 92)[0]
    s["max_descent_rate_mps"] = struct.unpack_from("<f", data, 96)[0]
    # Temperature
    s["peak_temp_avg_c"]    = struct.unpack_from("<f", data, 100)[0]
    s["min_temp_avg_c"]     = struct.unpack_from("<f", data, 104)[0]
    # Pyro
    s["pyro_fire_tick"]     = struct.unpack_from("<4I", data, 108)
    s["pyro_cont_at_launch"] = struct.unpack_from("<4H", data, 124)
    # GPS
    s["launch_lat_1e7"]     = struct.unpack_from("<i", data, 132)[0]
    s["launch_lon_1e7"]     = struct.unpack_from("<i", data, 136)[0]
    s["landing_lat_1e7"]    = struct.unpack_from("<i", data, 140)[0]
    s["landing_lon_1e7"]    = struct.unpack_from("<i", data, 144)[0]
    s["gps_max_alt_msl_m"]  = struct.unpack_from("<f", data, 148)[0]
    # Diagnostics
    s["hr_records_written"] = struct.unpack_from("<I", data, 152)[0]
    s["lr_records_written"] = struct.unpack_from("<I", data, 156)[0]
    s["adxl_records_written"] = struct.unpack_from("<I", data, 160)[0]
    s["total_drop_count"]   = struct.unpack_from("<H", data, 164)[0]
    s["flash_err_count"]    = struct.unpack_from("<H", data, 166)[0]
    s["num_fsm_transitions"] = struct.unpack_from("<H", data, 168)[0]
    # Integrity
    s["crc32"]              = struct.unpack_from("<I", data, 252)[0]
    return s

# ═══════════════════════════════════════════════════════════════════════
#  Record extraction helpers
# ═══════════════════════════════════════════════════════════════════════

def is_erased(data):
    """Check if a chunk is all 0xFF (erased NOR flash)."""
    return all(b == 0xFF for b in data)

def extract_records(flash, start_addr, end_addr, rec_size, recs_per_page, parser):
    """Extract records from a flash region, skipping erased pages/slots."""
    records = []
    addr = start_addr
    while addr < end_addr:
        page = flash[addr:addr + PAGE_SIZE]
        if is_erased(page):
            addr += PAGE_SIZE
            continue
        for slot in range(recs_per_page):
            off = slot * rec_size
            rec_data = page[off:off + rec_size]
            # Check for erased slot (padding at end of partial page)
            if is_erased(rec_data):
                continue
            # Check for seq_num=0xFFFF padding sentinel (HR records)
            if rec_size == HR_REC_SIZE:
                seq = struct.unpack_from("<H", rec_data, 53)[0]
                if seq == 0xFFFF:
                    continue
            rec = parser(rec_data)
            rec["_raw"] = rec_data
            rec["_addr"] = addr + off
            records.append(rec)
        addr += PAGE_SIZE
    return records

# ═══════════════════════════════════════════════════════════════════════
#  Main decode logic
# ═══════════════════════════════════════════════════════════════════════

def decode(flash, flight_num=None, export_csv=False, verbose=False):
    print(f"Flash dump: {len(flash)} bytes ({len(flash) / (1024*1024):.1f} MB)")
    print()

    # ── Flight index ──
    print("=" * 60)
    print("FLIGHT INDEX")
    print("=" * 60)
    flights = []
    for i in range(MAX_FLIGHTS):
        offset = FLASH_INDEX_BASE + i * 32
        entry_data = flash[offset:offset + 32]
        if is_erased(entry_data):
            break
        entry = parse_index_entry(entry_data)
        if entry["flight_id"] == 0xFFFF:
            break
        flights.append(entry)

    if not flights:
        print("  No flights recorded.")
        return

    print(f"  {len(flights)} flight(s) found:\n")
    for f in flights:
        dirty = f["end_tick_ms"] == INDEX_DIRTY
        clean = (f["flags"] & 0x0001) != 0
        print(f"  Flight {f['flight_id']:3d}: "
              f"t={f['start_tick_ms']}-{'DIRTY' if dirty else f['end_tick_ms']}ms  "
              f"HR=0x{f['hr_start_addr']:07X}-0x{f['hr_end_addr']:07X}  "
              f"LR=0x{f['lr_start_addr']:07X}-0x{f['lr_end_addr']:07X}  "
              f"ADXL=0x{f['adxl_start_addr']:07X}  "
              f"{'CLEAN' if clean else 'DIRTY'}")
    print()

    # ── Summaries ──
    print("=" * 60)
    print("FLIGHT SUMMARIES")
    print("=" * 60)
    for f in flights:
        fid = f["flight_id"]
        for page_off, label in [(0, "partial"), (PAGE_SIZE, "final")]:
            base = FLASH_SUMMARY_BASE + (fid - 1) * 2 * PAGE_SIZE + page_off
            sdata = flash[base:base + PAGE_SIZE]
            if is_erased(sdata):
                print(f"  Flight {fid} {label}: not written")
                continue
            s = parse_summary(sdata)
            if s["magic"] != SUMMARY_MAGIC:
                print(f"  Flight {fid} {label}: bad magic 0x{s['magic']:08X}")
                continue
            computed_crc = zlib.crc32(sdata[:252]) & 0xFFFFFFFF
            crc_ok = computed_crc == s["crc32"]
            print(f"\n  Flight {fid} {label} (type={s['summary_type']}, "
                  f"CRC32={'OK' if crc_ok else 'FAIL'}):")
            print(f"    Launch:  {s['launch_tick']} ms")
            print(f"    Apogee:  {s['apogee_tick']} ms  "
                  f"({s['apogee_ekf_m']:.1f}m EKF, "
                  f"{s['apogee_baro_m']:.1f}m baro, "
                  f"{s['apogee_gps_msl_m']:.1f}m GPS)")
            print(f"    Landing: {s['landing_tick']} ms  "
                  f"(total {s['total_flight_tick']} ms)")
            print(f"    Peak accel: {s['peak_accel_boost_g']:.2f}g boost, "
                  f"{s['peak_accel_sustain_g']:.2f}g sustain")
            print(f"    Peak vel:   {s['peak_velocity_mps']:.1f} m/s  "
                  f"(Mach {s['peak_mach']:.3f})")
            print(f"    Max-Q:      {s['max_q_pa']:.0f} Pa at {s['max_q_alt_m']:.0f}m")
            print(f"    Attitude:   pad tilt {s['pad_tilt_deg']:.1f}deg, "
                  f"max tilt {s['max_tilt_deg']:.1f}deg, "
                  f"max roll {s['max_roll_rate_dps']:.0f}dps")
            print(f"    Records:    HR={s['hr_records_written']}, "
                  f"LR={s['lr_records_written']}, "
                  f"ADXL={s['adxl_records_written']}")
            print(f"    Drops={s['total_drop_count']}, "
                  f"Errors={s['flash_err_count']}, "
                  f"FSM transitions={s['num_fsm_transitions']}")
    print()

    # ── Select flight for detailed decode ──
    target_id = flight_num or flights[0]["flight_id"]
    target = next((f for f in flights if f["flight_id"] == target_id), None)
    if not target:
        print(f"Flight {target_id} not found in index.")
        return

    print("=" * 60)
    print(f"FLIGHT {target_id} DETAILED RECORDS")
    print("=" * 60)

    # ── HR records ──
    hr_end = target["hr_end_addr"]
    if hr_end == INDEX_DIRTY:
        print("  HR: end address DIRTY (unclean shutdown), scanning for last page...")
        # Scan from start until erased page
        addr = target["hr_start_addr"]
        while addr < FLASH_HR_BASE + (LOG_FLASH_END - FLASH_HR_BASE):
            if is_erased(flash[addr:addr + PAGE_SIZE]):
                break
            addr += PAGE_SIZE
        hr_end = addr
        print(f"  HR: scanned end = 0x{hr_end:07X}")

    hr_records = extract_records(flash, target["hr_start_addr"], hr_end,
                                 HR_REC_SIZE, HR_PER_PAGE, parse_hr_record)
    crc_ok = crc_fail = hamming_ok = hamming_fail = 0
    for r in hr_records:
        raw = r["_raw"]
        # CRC16 check over bytes [0:62]
        if crc16_ccitt(raw[:62]) == r["crc16"]:
            crc_ok += 1
        else:
            crc_fail += 1
        # Hamming SECDED check over bytes [0:60]
        if hamming_encode(raw[:60]) == r["hamming"]:
            hamming_ok += 1
        else:
            hamming_fail += 1

    print(f"\n  HR records: {len(hr_records)}")
    if hr_records:
        print(f"    Time range: {hr_records[0]['timestamp_ms']}-"
              f"{hr_records[-1]['timestamp_ms']} ms "
              f"({(hr_records[-1]['timestamp_ms'] - hr_records[0]['timestamp_ms'])/1000:.1f}s)")
        print(f"    CRC16:   {crc_ok} OK, {crc_fail} FAIL")
        print(f"    Hamming: {hamming_ok} OK, {hamming_fail} FAIL")

        # Sequence continuity
        gaps = 0
        for i in range(1, len(hr_records)):
            expected = (hr_records[i-1]["seq_num"] + 1) & 0xFFFF
            if hr_records[i]["seq_num"] != expected:
                gaps += 1
        print(f"    Seq gaps: {gaps}")

        # FSM state distribution
        states = Counter(r["fsm_state"] for r in hr_records)
        print("    FSM states:")
        for state, count in sorted(states.items()):
            name = FSM_NAMES.get(state, f"?{state}")
            print(f"      {name:10s}: {count:6d} records")

    # ── LR records ──
    lr_end = target["lr_end_addr"]
    if lr_end == INDEX_DIRTY:
        addr = target["lr_start_addr"]
        while addr < FLASH_LR_BASE + FLASH_LR_SIZE:
            if is_erased(flash[addr:addr + PAGE_SIZE]):
                break
            addr += PAGE_SIZE
        lr_end = addr

    lr_records = extract_records(flash, target["lr_start_addr"], lr_end,
                                 LR_REC_SIZE, LR_PER_PAGE, parse_lr_record)
    print(f"\n  LR records: {len(lr_records)}")
    if lr_records:
        print(f"    Time range: {lr_records[0]['timestamp_ms']}-"
              f"{lr_records[-1]['timestamp_ms']} ms")
        if verbose and lr_records:
            for r in lr_records[:5]:
                print(f"    t={r['timestamp_ms']:8d}  pyro=0x{r['pyro_state']:02X}  "
                      f"rssi={r['radio_rssi']:4d}  snr={r['radio_snr']:3d}  "
                      f"tx={r['radio_tx_count']}  rx={r['radio_rx_count']}")
            if len(lr_records) > 5:
                print(f"    ... ({len(lr_records) - 5} more)")

    # ── ADXL records ──
    # ADXL pool doesn't have end addr in index; scan from start
    adxl_start = target["adxl_start_addr"]
    adxl_scan_end = adxl_start
    while adxl_scan_end < FLASH_ADXL_BASE + FLASH_ADXL_SIZE:
        if is_erased(flash[adxl_scan_end:adxl_scan_end + PAGE_SIZE]):
            break
        adxl_scan_end += PAGE_SIZE

    adxl_records = extract_records(flash, adxl_start, adxl_scan_end,
                                   ADXL_REC_SIZE, ADXL_PER_PAGE, parse_adxl_record)
    print(f"\n  ADXL records: {len(adxl_records)}")
    if adxl_records:
        print(f"    Time range: {adxl_records[0]['timestamp_ms']}-"
              f"{adxl_records[-1]['timestamp_ms']} ms")
    print()

    # ── CSV export ──
    if export_csv:
        base_name = f"flight_{target_id}"

        with open(f"{base_name}_hr.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_ms", "baro_pressure_2Pa", "accel_x", "accel_y",
                         "accel_z", "gyro_x", "gyro_y", "gyro_z", "adxl_x", "adxl_y",
                         "adxl_z", "mag_x", "mag_y", "mag_z", "ekf_alt_m", "ekf_vel_mps",
                         "fsm_state", "flags", "imu_temp_c100", "baro_temp_c100",
                         "seq_num"])
            for r in hr_records:
                w.writerow([
                    r["timestamp_ms"], r["baro_pressure"],
                    *r["lsm6_accel"], *r["lsm6_gyro"], *r["adxl372"], *r["mmc"],
                    f"{r['ekf_alt_m']:.4f}", f"{r['ekf_vel_mps']:.4f}",
                    r["fsm_state"], r["flags"], r["imu_temp"], r["baro_temp"],
                    r["seq_num"],
                ])
        print(f"  Exported {len(hr_records)} HR records to {base_name}_hr.csv")

        with open(f"{base_name}_lr.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_ms", "pyro_state", "cont_adc_1", "cont_adc_2",
                         "cont_adc_3", "cont_adc_4", "rssi", "snr", "tx_count",
                         "rx_count", "fail_count", "gps_dlat_mm", "gps_dlon_mm",
                         "gps_alt_msl_m", "gps_fix_sats"])
            for r in lr_records:
                w.writerow([
                    r["timestamp_ms"], r["pyro_state"], *r["pyro_cont_adc"],
                    r["radio_rssi"], r["radio_snr"], r["radio_tx_count"],
                    r["radio_rx_count"], r["radio_fail_count"],
                    r["gps_dlat_mm"], r["gps_dlon_mm"], r["gps_alt_msl_m"],
                    r["gps_fix_sats"],
                ])
        print(f"  Exported {len(lr_records)} LR records to {base_name}_lr.csv")

        if adxl_records:
            with open(f"{base_name}_adxl.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp_ms", "accel_x_100mg", "accel_y_100mg",
                             "accel_z_100mg"])
                for r in adxl_records:
                    w.writerow([r["timestamp_ms"], r["accel_x"], r["accel_y"],
                                r["accel_z"]])
            print(f"  Exported {len(adxl_records)} ADXL records to {base_name}_adxl.csv")

    # ── Summary ──
    print("=" * 60)
    print("INTEGRITY SUMMARY")
    print("=" * 60)
    all_ok = crc_fail == 0 and hamming_fail == 0
    print(f"  HR CRC16:   {crc_ok}/{len(hr_records)} pass")
    print(f"  HR Hamming: {hamming_ok}/{len(hr_records)} pass")
    print(f"  Result:     {'ALL PASS' if all_ok else 'FAILURES DETECTED'}")


def main():
    parser = argparse.ArgumentParser(description="C.A.S.P.E.R.-2 Flight Logger Decoder")
    parser.add_argument("dump", help="Path to raw 64MB flash dump file")
    parser.add_argument("--flight", type=int, default=None,
                        help="Flight ID to decode (default: first)")
    parser.add_argument("--csv", action="store_true",
                        help="Export decoded records to CSV files")
    parser.add_argument("--verbose", action="store_true",
                        help="Show detailed record data")
    args = parser.parse_args()

    dump_path = Path(args.dump)
    if not dump_path.exists():
        print(f"Error: {dump_path} not found", file=sys.stderr)
        sys.exit(1)

    flash = dump_path.read_bytes()
    if len(flash) < LOG_FLASH_END:
        print(f"Warning: dump is {len(flash)} bytes, expected {LOG_FLASH_END} (64 MB). "
              f"Padding with 0xFF.", file=sys.stderr)
        flash += b"\xFF" * (LOG_FLASH_END - len(flash))

    decode(flash, flight_num=args.flight, export_csv=args.csv, verbose=args.verbose)


if __name__ == "__main__":
    main()
