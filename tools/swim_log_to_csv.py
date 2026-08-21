#!/usr/bin/env python3
"""Convert a SWIM v1 binary log into per-record-type CSV files."""

import argparse
import csv
import math
import struct
from pathlib import Path

FILE_HEADER = struct.Struct("<8sHHIIII16s")
RECORD_HEADER = struct.Struct("<BBHQQ")
IMU = struct.Struct("<11f")
GNSS = struct.Struct("<BBBBB3xfddfff")
WAVES = struct.Struct("<B3x7fIHH")
TIME_SYNC = struct.Struct("<Q")
EVENT = struct.Struct("<I")
SUMMARY = struct.Struct("<B3xdd8f32B32b32b32b32b")
EVENT_NAMES = {1: "MARK", 2: "SET_ZERO"}


def iso_utc(ms: int) -> str:
    """Convert Unix milliseconds to an ISO 8601 UTC timestamp."""
    if not ms:
        return ""
    from datetime import datetime, timezone

    return datetime.fromtimestamp(ms / 1000, timezone.utc).isoformat(timespec="milliseconds")


def main() -> None:
    """Convert one SWIM v1/v2 binary log into per-record CSV files."""
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, help="Output prefix; defaults to input stem")
    args = parser.parse_args()
    prefix = args.output or args.log.with_suffix("")

    outputs = {
        1: ("imu", ["monotonic_us", "utc_ms", "utc_iso", "ax_g", "ay_g", "az_g",
                    "gx_dps", "gy_dps", "gz_dps", "roll_rad", "pitch_rad",
                    "level_ax_ms2", "level_ay_ms2", "level_az_ms2"]),
        2: ("gnss", ["monotonic_us", "utc_ms", "utc_iso", "uart", "time_valid", "fix",
                     "fix_quality", "sat_used", "signals_visible", "best_cn0_dbhz", "hdop",
                     "latitude_deg", "longitude_deg", "altitude_m", "sog_mps", "cog_deg"]),
        3: ("waves", ["monotonic_us", "utc_ms", "utc_iso", "imu_ready", "gnss_alt_ready", "low_frequency_edge_peak",
                      "hs_m", "fp_hz", "tp_s", "tm02_s", "direction_from_deg",
                      "gnss_alt_hs_m", "gnss_alt_tz_s", "imu_samples", "gnss_alt_samples"]),
        4: ("time_sync", ["monotonic_us", "utc_ms", "utc_iso", "anchor_utc_ms"]),
        5: ("events", ["monotonic_us", "utc_ms", "utc_iso", "event_code", "event_name"]),
        6: ("summary", ["monotonic_us", "utc_ms", "utc_iso", "uart", "time_valid",
             "fix", "waves_ready", "edge_peak", "latitude_deg", "longitude_deg",
             "altitude_m", "sog_mps", "cog_deg", "hs_m", "fp_hz", "tp_s",
             "tm02_s", "direction_from_deg"] +
             [f"s_norm_{0.4*i/31:.3f}hz" for i in range(32)] +
             [f"{name}_{0.4*i/31:.3f}hz" for name in ("a1", "b1", "a2", "b2") for i in range(32)]),
    }
    handles = {}
    writers = {}

    try:
        for kind, (suffix, columns) in outputs.items():
            handle = Path(f"{prefix}_{suffix}.csv").open("w", newline="")
            handles[kind] = handle
            writers[kind] = csv.writer(handle)
            writers[kind].writerow(columns)

        with args.log.open("rb") as source:
            raw = source.read(FILE_HEADER.size)
            if len(raw) != FILE_HEADER.size:
                raise SystemExit("Truncated SWIM file header")
            magic, version, header_size, imu_mhz, wave_mhz, fft_size, buffer_size, firmware = FILE_HEADER.unpack(raw)
            if not magic.startswith(b"SWIMLOG") or version not in (1, 2):
                raise SystemExit("Not a supported SWIM v1/v2 log")
            if header_size > FILE_HEADER.size:
                source.seek(header_size - FILE_HEADER.size, 1)

            while True:
                raw = source.read(RECORD_HEADER.size)
                if not raw:
                    break
                if len(raw) != RECORD_HEADER.size:
                    print("Warning: trailing incomplete record header ignored")
                    break
                kind, record_version, payload_size, mono_us, utc_ms = RECORD_HEADER.unpack(raw)
                payload = source.read(payload_size)
                if len(payload) != payload_size:
                    print("Warning: trailing incomplete payload ignored")
                    break
                prefix_fields = [mono_us, utc_ms or "", iso_utc(utc_ms)]

                if kind == 1 and payload_size == IMU.size:
                    writers[kind].writerow(prefix_fields + list(IMU.unpack(payload)))
                elif kind == 2 and payload_size == GNSS.size:
                    flags, quality, used, visible, cn0, hdop, lat, lon, alt, sog, cog = GNSS.unpack(payload)
                    writers[kind].writerow(prefix_fields + [bool(flags & 1), bool(flags & 2),
                        bool(flags & 4), quality, used, visible, cn0, hdop, lat, lon, alt, sog, cog])
                elif kind == 3 and payload_size == WAVES.size:
                    values = list(WAVES.unpack(payload))
                    flags = values.pop(0)
                    writers[kind].writerow(prefix_fields + [bool(flags & 1), bool(flags & 2), bool(flags & 4)] + values)
                elif kind == 4 and payload_size == TIME_SYNC.size:
                    writers[kind].writerow(prefix_fields + [TIME_SYNC.unpack(payload)[0]])
                elif kind == 5 and payload_size == EVENT.size:
                    event_code = EVENT.unpack(payload)[0]
                    writers[kind].writerow(prefix_fields + [event_code,
                        EVENT_NAMES.get(event_code, "UNKNOWN")])
                elif kind == 6 and payload_size == SUMMARY.size:
                    values = list(SUMMARY.unpack(payload))
                    flags = values.pop(0)
                    writers[kind].writerow(prefix_fields + [bool(flags & 1), bool(flags & 2),
                        bool(flags & 4), bool(flags & 8), bool(flags & 16)] + values)

        print(f"Converted {args.log} using firmware {firmware.rstrip(bytes([0])).decode(errors='replace')}")
    finally:
        for handle in handles.values():
            handle.close()


if __name__ == "__main__":
    main()
