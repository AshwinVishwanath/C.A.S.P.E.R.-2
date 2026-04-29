#!/usr/bin/env python3
"""
C.A.S.P.E.R.-2 Flight Logger Decoder

Reads a raw 64MB flash dump and decodes flight index, summaries,
HR/LR/ADXL records with Hamming SECDED + CRC integrity validation.

Usage:
  python casper_decode.py flash_dump.bin [--flight N] [--csv] [--verbose]
  python casper_decode.py --disk [--save dump.bin] [--flight N] [--csv]
"""
import argparse
import csv
import os
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

def decode(flash, flight_num=None, export_csv=False, verbose=False, plot=False):
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
        # log_index_end_flight() clears bit 0 of flags (NOR 1->0 trick) to mark
        # a clean shutdown, so 0 = clean, 1 (default 0xFFFF) = dirty.
        clean = (f["flags"] & 0x0001) == 0
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

    # ── Plotting ──
    if plot and hr_records:
        plot_flight(target_id, hr_records, lr_records, adxl_records)


# ═══════════════════════════════════════════════════════════════════════
#  Plotting
# ═══════════════════════════════════════════════════════════════════════

# Sensor scaling constants
LSM6_ACCEL_SCALE = 0.000976       # raw → g (±32g, 16-bit)
LSM6_GYRO_SCALE  = 0.070          # raw → dps (±2000dps, 16-bit)
ADXL_SCALE       = 0.100          # raw → g (±200g, 100 mg/LSB)
BARO_SCALE       = 2.0 / 100.0   # raw → hPa (2 Pa/LSB → hPa)
TEMP_SCALE       = 0.01           # raw → °C
BIAS_ACCEL_SCALE = 0.001          # raw → m/s²
BIAS_BARO_SCALE  = 0.01           # raw → m

FSM_COLORS = {
    0: "#9e9e9e",   # PAD — grey
    1: "#e53935",   # BOOST — red
    2: "#ff9800",   # COAST — orange
    3: "#ffb74d",   # COAST_1 — light orange
    4: "#d32f2f",   # SUSTAIN — dark red
    5: "#ffa726",   # COAST_2 — amber
    6: "#43a047",   # APOGEE — green
    7: "#1e88e5",   # DROGUE — blue
    8: "#8e24aa",   # MAIN — purple
    9: "#00897b",   # RECOVERY — teal
    10: "#f4511e",  # TUMBLE — deep orange
    11: "#546e7a",  # LANDED — blue-grey
}


def _fsm_spans(times, states):
    """Build (start, end, state) spans from parallel time/state arrays."""
    spans = []
    if not times:
        return spans
    cur_state = states[0]
    span_start = times[0]
    for t, s in zip(times[1:], states[1:]):
        if s != cur_state:
            spans.append((span_start, t, cur_state))
            cur_state = s
            span_start = t
    spans.append((span_start, times[-1], cur_state))
    return spans


def _shade_fsm(ax, spans):
    """Add translucent FSM state shading to an axes."""
    for t0, t1, state in spans:
        color = FSM_COLORS.get(state, "#cccccc")
        ax.axvspan(t0, t1, alpha=0.12, color=color, linewidth=0)


def _add_fsm_legend(fig, spans):
    """Add a small FSM colour legend below the figure."""
    seen = []
    for _, _, s in spans:
        if s not in seen:
            seen.append(s)
    import matplotlib.patches as mpatches
    patches = [mpatches.Patch(color=FSM_COLORS.get(s, "#ccc"), alpha=0.5,
               label=FSM_NAMES.get(s, f"?{s}")) for s in seen]
    fig.legend(handles=patches, loc="lower center", ncol=min(len(patches), 8),
               fontsize=7, frameon=False)


def plot_flight(flight_id, hr_records, lr_records, adxl_records):
    """Generate matplotlib plots for a decoded flight."""
    try:
        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend (works headless)
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
    except ImportError:
        print("\n  matplotlib not installed — skipping plots. "
              "Install with: pip install matplotlib", file=sys.stderr)
        return

    # ── Extract & convert HR arrays ──
    t0_ms = hr_records[0]["timestamp_ms"]
    t_hr   = [(r["timestamp_ms"] - t0_ms) / 1000.0 for r in hr_records]
    alt    = [r["ekf_alt_m"] for r in hr_records]
    vel    = [r["ekf_vel_mps"] for r in hr_records]
    ax_g   = [r["lsm6_accel"][0] * LSM6_ACCEL_SCALE for r in hr_records]
    ay_g   = [r["lsm6_accel"][1] * LSM6_ACCEL_SCALE for r in hr_records]
    az_g   = [r["lsm6_accel"][2] * LSM6_ACCEL_SCALE for r in hr_records]
    a_mag  = [(x*x + y*y + z*z)**0.5 for x, y, z in zip(ax_g, ay_g, az_g)]
    gx     = [r["lsm6_gyro"][0] * LSM6_GYRO_SCALE for r in hr_records]
    gy     = [r["lsm6_gyro"][1] * LSM6_GYRO_SCALE for r in hr_records]
    gz     = [r["lsm6_gyro"][2] * LSM6_GYRO_SCALE for r in hr_records]
    baro   = [r["baro_pressure"] * BARO_SCALE for r in hr_records]
    fsm_st = [r["fsm_state"] for r in hr_records]
    imu_t  = [r["imu_temp"] * TEMP_SCALE for r in hr_records]
    bar_t  = [r["baro_temp"] * TEMP_SCALE for r in hr_records]
    ab     = [r["ekf_accel_bias"] * BIAS_ACCEL_SCALE for r in hr_records]
    bb     = [r["ekf_baro_bias"] * BIAS_BARO_SCALE for r in hr_records]

    fsm_spans = _fsm_spans(t_hr, fsm_st)

    # ── Style ──
    plt.rcParams.update({
        "font.size": 8,
        "axes.titlesize": 9,
        "axes.labelsize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "lines.linewidth": 0.8,
        "figure.dpi": 150,
    })

    base_name = f"flight_{flight_id}"

    # ════════════════════════════════════════════════════════════════
    #  Figure 1 — Flight Overview (2x2)
    # ════════════════════════════════════════════════════════════════
    fig1, axes1 = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    fig1.suptitle(f"C.A.S.P.E.R.-2  —  Flight {flight_id} Overview", fontweight="bold")

    # Altitude
    ax = axes1[0, 0]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, alt, color="#1565c0")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("EKF Altitude")
    ax.grid(True, alpha=0.3)

    # Velocity
    ax = axes1[0, 1]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, vel, color="#c62828")
    ax.axhline(0, color="grey", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("EKF Velocity")
    ax.grid(True, alpha=0.3)

    # Accel magnitude
    ax = axes1[1, 0]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, a_mag, color="#2e7d32", linewidth=0.6)
    ax.axhline(1.0, color="grey", linewidth=0.5, linestyle="--", label="1g")
    ax.set_ylabel("Accel (g)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Accel Magnitude (LSM6DSO32)")
    ax.grid(True, alpha=0.3)

    # FSM state timeline
    ax = axes1[1, 1]
    for t0, t1, state in fsm_spans:
        color = FSM_COLORS.get(state, "#cccccc")
        name = FSM_NAMES.get(state, f"?{state}")
        ax.barh(0, t1 - t0, left=t0, height=0.6, color=color, edgecolor="white",
                linewidth=0.5)
        mid = (t0 + t1) / 2
        if (t1 - t0) > (t_hr[-1] - t_hr[0]) * 0.03:
            ax.text(mid, 0, name, ha="center", va="center", fontsize=6,
                    fontweight="bold", color="white")
    ax.set_yticks([])
    ax.set_xlabel("Time (s)")
    ax.set_title("FSM State Timeline")
    ax.set_xlim(t_hr[0], t_hr[-1])

    _add_fsm_legend(fig1, fsm_spans)
    fig1.tight_layout(rect=[0, 0.04, 1, 0.96])
    fig1.savefig(f"{base_name}_overview.png")
    print(f"  Saved {base_name}_overview.png")

    # ════════════════════════════════════════════════════════════════
    #  Figure 2 — IMU Detail (3 rows)
    # ════════════════════════════════════════════════════════════════
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig2.suptitle(f"C.A.S.P.E.R.-2  —  Flight {flight_id} IMU Detail", fontweight="bold")

    # 3-axis accel
    ax = axes2[0]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, ax_g, linewidth=0.5, label="X", color="#e53935")
    ax.plot(t_hr, ay_g, linewidth=0.5, label="Y", color="#43a047")
    ax.plot(t_hr, az_g, linewidth=0.5, label="Z", color="#1e88e5")
    ax.set_ylabel("Accel (g)")
    ax.set_title("LSM6DSO32 Accelerometer (±32g)")
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.3)

    # 3-axis gyro
    ax = axes2[1]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, gx, linewidth=0.5, label="X", color="#e53935")
    ax.plot(t_hr, gy, linewidth=0.5, label="Y", color="#43a047")
    ax.plot(t_hr, gz, linewidth=0.5, label="Z", color="#1e88e5")
    ax.set_ylabel("Gyro (dps)")
    ax.set_title("LSM6DSO32 Gyroscope (±2000 dps)")
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.3)

    # Baro pressure
    ax = axes2[2]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, baro, color="#6a1b9a", linewidth=0.6)
    ax.set_ylabel("Pressure (hPa)")
    ax.set_xlabel("Time (s)")
    ax.set_title("MS5611 Barometer")
    ax.grid(True, alpha=0.3)

    _add_fsm_legend(fig2, fsm_spans)
    fig2.tight_layout(rect=[0, 0.04, 1, 0.96])
    fig2.savefig(f"{base_name}_imu.png")
    print(f"  Saved {base_name}_imu.png")

    # ════════════════════════════════════════════════════════════════
    #  Figure 3 — EKF & Environment (2x2)
    # ════════════════════════════════════════════════════════════════
    fig3, axes3 = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    fig3.suptitle(f"C.A.S.P.E.R.-2  —  Flight {flight_id} EKF & Environment",
                  fontweight="bold")

    # Alt + baro overlay
    ax = axes3[0, 0]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, alt, label="EKF alt", color="#1565c0")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("EKF Altitude")
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.3)

    # Velocity
    ax = axes3[0, 1]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, vel, color="#c62828")
    ax.axhline(0, color="grey", linewidth=0.5, linestyle="--")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("EKF Velocity")
    ax.grid(True, alpha=0.3)

    # EKF biases
    ax = axes3[1, 0]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, ab, label="Accel bias (m/s²)", color="#ff6f00")
    ax2 = ax.twinx()
    ax2.plot(t_hr, bb, label="Baro bias (m)", color="#0277bd", linestyle="--")
    ax.set_ylabel("Accel bias (m/s²)", color="#ff6f00")
    ax2.set_ylabel("Baro bias (m)", color="#0277bd")
    ax.set_xlabel("Time (s)")
    ax.set_title("EKF Biases")
    ax.grid(True, alpha=0.3)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=7)

    # Temperatures
    ax = axes3[1, 1]
    _shade_fsm(ax, fsm_spans)
    ax.plot(t_hr, imu_t, label="IMU", color="#e53935", linewidth=0.6)
    ax.plot(t_hr, bar_t, label="Baro", color="#1565c0", linewidth=0.6)
    ax.set_ylabel("Temperature (°C)")
    ax.set_xlabel("Time (s)")
    ax.set_title("Sensor Temperatures")
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.3)

    _add_fsm_legend(fig3, fsm_spans)
    fig3.tight_layout(rect=[0, 0.04, 1, 0.96])
    fig3.savefig(f"{base_name}_ekf.png")
    print(f"  Saved {base_name}_ekf.png")

    # ════════════════════════════════════════════════════════════════
    #  Figure 4 — LR: Radio + Pyro (if records exist)
    # ════════════════════════════════════════════════════════════════
    if lr_records:
        t_lr = [(r["timestamp_ms"] - t0_ms) / 1000.0 for r in lr_records]
        rssi = [r["radio_rssi"] for r in lr_records]
        snr  = [r["radio_snr"] for r in lr_records]
        tx   = [r["radio_tx_count"] for r in lr_records]
        rx   = [r["radio_rx_count"] for r in lr_records]
        cont = [[r["pyro_cont_adc"][ch] for r in lr_records] for ch in range(4)]

        fig4, axes4 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        fig4.suptitle(f"C.A.S.P.E.R.-2  —  Flight {flight_id} Radio & Pyro",
                      fontweight="bold")

        # RSSI / SNR
        ax = axes4[0]
        _shade_fsm(ax, fsm_spans)
        ax.plot(t_lr, rssi, label="RSSI (dBm)", color="#1565c0", linewidth=0.6)
        ax_snr = ax.twinx()
        ax_snr.plot(t_lr, snr, label="SNR (dB)", color="#43a047",
                    linewidth=0.6, linestyle="--")
        ax.set_ylabel("RSSI (dBm)", color="#1565c0")
        ax_snr.set_ylabel("SNR (dB)", color="#43a047")
        ax.set_title("Radio Link Quality")
        ax.grid(True, alpha=0.3)
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax_snr.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=7)

        # TX/RX counts
        ax = axes4[1]
        _shade_fsm(ax, fsm_spans)
        ax.plot(t_lr, tx, label="TX", color="#e53935", linewidth=0.6)
        ax.plot(t_lr, rx, label="RX", color="#1e88e5", linewidth=0.6)
        ax.set_ylabel("Packet Count")
        ax.set_title("Radio Packet Counts")
        ax.legend(loc="upper left", fontsize=7)
        ax.grid(True, alpha=0.3)

        # Pyro continuity
        ax = axes4[2]
        _shade_fsm(ax, fsm_spans)
        colors_pyro = ["#e53935", "#ff9800", "#43a047", "#1e88e5"]
        for ch in range(4):
            ax.plot(t_lr, cont[ch], label=f"CH{ch+1}", color=colors_pyro[ch],
                    linewidth=0.6)
        ax.set_ylabel("ADC raw")
        ax.set_xlabel("Time (s)")
        ax.set_title("Pyro Continuity ADC")
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)

        _add_fsm_legend(fig4, fsm_spans)
        fig4.tight_layout(rect=[0, 0.04, 1, 0.96])
        fig4.savefig(f"{base_name}_radio_pyro.png")
        print(f"  Saved {base_name}_radio_pyro.png")

    # ════════════════════════════════════════════════════════════════
    #  Figure 5 — ADXL372 high-g (if records exist)
    # ════════════════════════════════════════════════════════════════
    if adxl_records:
        t_adxl = [(r["timestamp_ms"] - t0_ms) / 1000.0 for r in adxl_records]
        hx = [r["accel_x"] * ADXL_SCALE for r in adxl_records]
        hy = [r["accel_y"] * ADXL_SCALE for r in adxl_records]
        hz = [r["accel_z"] * ADXL_SCALE for r in adxl_records]

        fig5, ax5 = plt.subplots(figsize=(12, 4))
        fig5.suptitle(f"C.A.S.P.E.R.-2  —  Flight {flight_id} ADXL372 High-G",
                      fontweight="bold")
        _shade_fsm(ax5, fsm_spans)
        ax5.plot(t_adxl, hx, linewidth=0.4, label="X", color="#e53935")
        ax5.plot(t_adxl, hy, linewidth=0.4, label="Y", color="#43a047")
        ax5.plot(t_adxl, hz, linewidth=0.4, label="Z", color="#1e88e5")
        ax5.set_ylabel("Accel (g)")
        ax5.set_xlabel("Time (s)")
        ax5.set_title("ADXL372 (±200g)")
        ax5.legend(loc="upper right", fontsize=7)
        ax5.grid(True, alpha=0.3)
        fig5.tight_layout()
        fig5.savefig(f"{base_name}_adxl.png")
        print(f"  Saved {base_name}_adxl.png")

    plt.close("all")
    print(f"\n  All plots saved as {base_name}_*.png")


# ═══════════════════════════════════════════════════════════════════════
#  Direct disk read (Windows — requires admin)
# ═══════════════════════════════════════════════════════════════════════

FLASH_SIZE = LOG_FLASH_END  # 64 MB
DISK_BLOCK = 4096           # read in 4K chunks (matches flash sector)

def _find_casper_disk():
    """Find CASPER QSPI Flash physical drive on Windows via WMI."""
    if sys.platform != "win32":
        print("Error: --disk only supported on Windows. "
              "On Linux use: dd if=/dev/sdX of=dump.bin bs=4096 count=16384",
              file=sys.stderr)
        sys.exit(1)

    try:
        import subprocess
        # Use PowerShell to find USB disks — no extra packages needed
        ps = subprocess.run(
            ["powershell", "-Command",
             "Get-Disk | Where-Object { $_.BusType -eq 'USB' } | "
             "Select-Object Number,FriendlyName,Size | ConvertTo-Json"],
            capture_output=True, text=True, timeout=10
        )
        if ps.returncode != 0:
            print(f"Error querying disks: {ps.stderr.strip()}", file=sys.stderr)
            sys.exit(1)

        import json
        disks = json.loads(ps.stdout) if ps.stdout.strip() else []
        if isinstance(disks, dict):
            disks = [disks]  # single result comes as dict, not list

        # Look for 64MB disk (CASPER QSPI flash)
        candidates = [d for d in disks
                      if 60_000_000 < d.get("Size", 0) < 70_000_000]

        if not candidates:
            # Show all USB disks for debugging
            print("No 64MB USB disk found. USB disks detected:", file=sys.stderr)
            for d in disks:
                sz_mb = d.get("Size", 0) / 1_000_000
                print(f"  Drive {d['Number']}: {d.get('FriendlyName','?')} "
                      f"({sz_mb:.0f} MB)", file=sys.stderr)
            sys.exit(1)

        disk = candidates[0]
        drive_num = disk["Number"]
        name = disk.get("FriendlyName", "?")
        print(f"Found CASPER flash: Drive {drive_num} ({name}, "
              f"{disk['Size']/1_000_000:.0f} MB)")
        return drive_num

    except FileNotFoundError:
        print("Error: PowerShell not found.", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError:
        print("Error: Could not parse disk list.", file=sys.stderr)
        sys.exit(1)


def read_disk(drive_num):
    """Read 64MB from \\\\.\\PhysicalDriveN. Requires admin."""
    path = f"\\\\.\\PhysicalDrive{drive_num}"
    print(f"Reading {FLASH_SIZE // (1024*1024)} MB from {path}...")

    try:
        fd = os.open(path, os.O_RDONLY | os.O_BINARY)
    except PermissionError:
        print("Error: Access denied. Run as Administrator.", file=sys.stderr)
        sys.exit(1)
    except OSError as e:
        print(f"Error opening {path}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        chunks = []
        total = 0
        while total < FLASH_SIZE:
            to_read = min(DISK_BLOCK, FLASH_SIZE - total)
            data = os.read(fd, to_read)
            if not data:
                break
            chunks.append(data)
            total += len(data)
            # Progress every 4 MB
            if total % (4 * 1024 * 1024) == 0:
                print(f"  {total // (1024*1024)}/{FLASH_SIZE // (1024*1024)} MB", end="\r")
        print(f"  Read {total:,} bytes.                    ")
    finally:
        os.close(fd)

    flash = b"".join(chunks)
    if len(flash) < FLASH_SIZE:
        print(f"Warning: only read {len(flash)} bytes, padding to {FLASH_SIZE}.",
              file=sys.stderr)
        flash += b"\xFF" * (FLASH_SIZE - len(flash))
    return flash


# ═══════════════════════════════════════════════════════════════════════
#  CDC serial dump (no admin required, needs pyserial)
# ═══════════════════════════════════════════════════════════════════════

DUMP_ACK  = 0x06
DUMP_CHUNK = 4096
REGION_NAMES = {0: "INDEX", 1: "SUMMARY", 2: "LR", 3: "ADXL", 4: "HR"}

def read_serial(port):
    """Send 0xD2 dump command, receive flash data over CDC with ACK handshake."""
    try:
        import serial
    except ImportError:
        print("Error: pyserial required. Install with: pip install pyserial",
              file=sys.stderr)
        sys.exit(1)

    print(f"Connecting to {port}...")
    ser = serial.Serial(port, timeout=5)
    import time; time.sleep(0.5)
    ser.reset_input_buffer()

    # Send COBS-encoded dump command: [0x02, 0xD2, 0x00]
    ser.write(bytes([0x02, 0xD2, 0x00]))
    print("Dump command sent, scanning for header...")

    # Scan for "CDMP" magic — discard any stale ASCII telemetry
    magic = b""
    deadline = time.time() + 10  # 10s timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        magic += b
        # Keep only last 4 bytes
        if len(magic) > 4:
            magic = magic[-4:]
        if magic == b"CDMP":
            break
    else:
        print("Error: timed out waiting for CDMP header", file=sys.stderr)
        ser.close()
        sys.exit(1)

    # Next byte is num_regions
    nr = ser.read(1)
    if not nr:
        print("Error: no region count received", file=sys.stderr)
        ser.close()
        sys.exit(1)

    num_regions = nr[0]
    print(f"Header OK — {num_regions} regions")

    # Read region descriptors: id(1) + offset(4 LE) + size(4 LE)
    regions = []
    for _ in range(num_regions):
        desc = ser.read(9)
        if len(desc) < 9:
            print("Error: truncated region descriptor", file=sys.stderr)
            ser.close()
            sys.exit(1)
        rid = desc[0]
        offset = struct.unpack_from("<I", desc, 1)[0]
        size = struct.unpack_from("<I", desc, 5)[0]
        regions.append({"id": rid, "offset": offset, "size": size})
        name = REGION_NAMES.get(rid, f"?{rid}")
        print(f"  Region {name}: 0x{offset:07X}, {size:,} bytes "
              f"({size // 1024} KB)")

    # Send ACK to start streaming
    ser.write(bytes([DUMP_ACK]))

    # Receive each region
    flash = bytearray(b"\xFF" * LOG_FLASH_END)
    for reg in regions:
        name = REGION_NAMES.get(reg["id"], f"?{reg['id']}")
        addr = reg["offset"]
        remain = reg["size"]
        received = 0

        while remain > 0:
            n = min(DUMP_CHUNK, remain)
            data = ser.read(n)
            if len(data) < n:
                print(f"\nError: timeout reading {name} at 0x{addr:07X} "
                      f"(got {len(data)}/{n})", file=sys.stderr)
                ser.close()
                sys.exit(1)
            flash[addr:addr + len(data)] = data
            addr += len(data)
            remain -= len(data)
            received += len(data)
            # Progress
            pct = received * 100 // reg["size"]
            print(f"\r  {name}: {received:,}/{reg['size']:,} ({pct}%)", end="")
            # Send ACK
            ser.write(bytes([DUMP_ACK]))

        print()  # newline after progress

    # Read end marker
    end = ser.read(4)
    if end == b"DONE":
        print("Transfer complete.")
    else:
        print(f"Warning: expected DONE, got {end!r}")

    ser.close()
    return bytes(flash)


def main():
    parser = argparse.ArgumentParser(description="C.A.S.P.E.R.-2 Flight Logger Decoder")
    parser.add_argument("dump", nargs="?", default=None,
                        help="Path to raw 64MB flash dump file")
    parser.add_argument("--serial", type=str, default=None, metavar="PORT",
                        help="Read flash over CDC serial (e.g. --serial COM3)")
    parser.add_argument("--disk", action="store_true",
                        help="Read directly from CASPER USB MSC drive (admin required)")
    parser.add_argument("--drive", type=int, default=None,
                        help="Physical drive number (skip auto-detect)")
    parser.add_argument("--save", type=str, default=None,
                        help="Save raw dump to file after reading")
    parser.add_argument("--flight", type=int, default=None,
                        help="Flight ID to decode (default: first)")
    parser.add_argument("--csv", action="store_true",
                        help="Export decoded records to CSV files")
    parser.add_argument("--verbose", action="store_true",
                        help="Show detailed record data")
    parser.add_argument("--plot", action="store_true",
                        help="Generate flight plots (requires matplotlib)")
    args = parser.parse_args()

    if args.serial:
        flash = read_serial(args.serial)
        if args.save:
            Path(args.save).write_bytes(flash)
            print(f"Saved raw dump to {args.save}")
    elif args.disk or args.drive is not None:
        drive_num = args.drive if args.drive is not None else _find_casper_disk()
        flash = read_disk(drive_num)
        if args.save:
            Path(args.save).write_bytes(flash)
            print(f"Saved raw dump to {args.save}")
    elif args.dump:
        dump_path = Path(args.dump)
        if not dump_path.exists():
            print(f"Error: {dump_path} not found", file=sys.stderr)
            sys.exit(1)
        flash = dump_path.read_bytes()
        if len(flash) < LOG_FLASH_END:
            print(f"Warning: dump is {len(flash)} bytes, expected {LOG_FLASH_END} (64 MB). "
                  f"Padding with 0xFF.", file=sys.stderr)
            flash += b"\xFF" * (LOG_FLASH_END - len(flash))
    else:
        parser.print_help()
        print("\nError: provide a dump file, --serial PORT, or --disk",
              file=sys.stderr)
        sys.exit(1)

    decode(flash, flight_num=args.flight, export_csv=args.csv,
           verbose=args.verbose, plot=args.plot)


if __name__ == "__main__":
    main()
