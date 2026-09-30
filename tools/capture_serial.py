"""Capture one Teensy USB-serial session without mixing runs or losing diagnostics."""

import argparse
import csv
import json
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


REQUIRED = {"time_ms", "mag_x_uT", "mag_y_uT", "mag_z_uT", "mag_valid"}
SESSION = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}\Z")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Teensy USB serial port, e.g. COM4")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--session", required=True, help="Unique name; never appends to an old run")
    parser.add_argument("--phase", choices=("bench", "installed", "validation"), required=True)
    parser.add_argument("--state", required=True, help="e.g. motors_off, wheel_on, laptop_2m")
    parser.add_argument("--note", default="", help="Mount, location, height and other conditions")
    parser.add_argument("--seconds", type=float, help="Stop after this many seconds; Ctrl-C also stops")
    parser.add_argument("--output-root", type=Path, default=Path(__file__).resolve().parents[1] / "logs" / "sessions")
    args = parser.parse_args()
    if not SESSION.fullmatch(args.session):
        parser.error("--session must use letters, numbers, underscores or hyphens")
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    return args


def lines_from_serial(port, baud):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is missing; run this with PlatformIO's Python") from exc
    with serial.Serial(port, baudrate=baud, timeout=0.5) as device:
        while True:
            data = device.readline()
            yield data.decode("utf-8", errors="replace").strip()


def capture(source, csv_file, raw_file, deadline=None):
    writer = csv.writer(csv_file)
    header = None
    count = 0
    for line in source:
        if deadline is not None and time.monotonic() >= deadline:
            break
        print(line, flush=True)
        raw_file.write(line + "\n")
        raw_file.flush()
        if not line or line.startswith("#"):
            continue
        row = next(csv.reader([line]))
        if row and row[0] == "time_ms":
            if not REQUIRED.issubset(row):
                raise ValueError("Unexpected logger header; required magnetic fields missing")
            if header is not None and row != header:
                raise ValueError("Logger schema changed during session")
            if header is None:
                header = row
                writer.writerow(header)
                csv_file.flush()
            continue
        if header is None or len(row) != len(header) or not row[0].isdigit():
            continue
        writer.writerow(row)
        csv_file.flush()
        count += 1
    return count


def main():
    args = arguments()
    root = args.output_root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    paths = {suffix: root / f"{args.session}.{suffix}" for suffix in ("csv", "log", "json")}
    if any(path.exists() for path in paths.values()):
        raise SystemExit("Session name already exists; choose a new --session")
    metadata = {
        "session": args.session,
        "phase": args.phase,
        "state": args.state,
        "note": args.note,
        "port": args.port or "stdin",
        "baud": args.baud,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "csv": paths["csv"].name,
        "raw_log": paths["log"].name,
    }
    paths["json"].write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    source = lines_from_serial(args.port, args.baud) if args.port else (line.rstrip("\r\n") for line in sys.stdin)
    deadline = time.monotonic() + args.seconds if args.seconds else None
    count = 0
    try:
        with paths["csv"].open("x", newline="", encoding="utf-8") as csv_file, paths["log"].open("x", encoding="utf-8") as raw_file:
            count = capture(source, csv_file, raw_file, deadline)
    except KeyboardInterrupt:
        pass
    finally:
        metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
        metadata["rows"] = count
        paths["json"].write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Saved {count} rows to {paths['csv']}", file=sys.stderr)


if __name__ == "__main__":
    main()
