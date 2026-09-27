import csv
import re
import sys
from pathlib import Path


CSV_HEADER = [
    "time_ms",
    "mag_x_uT",
    "mag_y_uT",
    "mag_z_uT",
    "mag_norm_uT",
    "ir_ambient_C",
    "ir_object_C",
    "rtd_raw",
    "rtd_ohm",
    "rtd_C",
    "mag_valid",
    "ir_valid",
    "rtd_valid",
    "rtd_fault",
]
ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
RTD_FAULT = re.compile(r"0x[0-9a-fA-F]{2}")
LOG_PATH = Path(__file__).resolve().parents[1] / "logs" / "sensor_data.csv"


def parse_sensor_row(line: str) -> list[str] | None:
    cleaned = ANSI_ESCAPE.sub("", line).strip()
    if ">" in cleaned:
        cleaned = cleaned.rsplit(">", 1)[-1].strip()

    try:
        row = next(csv.reader([cleaned]))
    except csv.Error:
        return None

    if (
        len(row) != len(CSV_HEADER)
        or not row[0].isdigit()
        or RTD_FAULT.fullmatch(row[-1]) is None
    ):
        return None
    return row


def main() -> None:
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with LOG_PATH.open("a", newline="", encoding="utf-8") as log_file:
        writer = csv.writer(log_file)
        if log_file.tell() == 0:
            writer.writerow(CSV_HEADER)
            log_file.flush()

        for line in sys.stdin:
            sys.stdout.write(line)
            sys.stdout.flush()

            row = parse_sensor_row(line)
            if row is not None:
                writer.writerow(row)
                log_file.flush()


if __name__ == "__main__":
    main()
