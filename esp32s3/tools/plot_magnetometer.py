import csv
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def main() -> None:
    log_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])
    rows = []
    pattern = re.compile(r">\s*(\d+,[^\r\n]+)$")

    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line in log_file:
            match = pattern.search(line)
            if not match:
                continue
            fields = next(csv.reader([match.group(1)]))
            if len(fields) < 14 or fields[10] != "1":
                continue
            try:
                rows.append(tuple(float(fields[i]) for i in range(5)))
            except ValueError:
                continue

    if not rows:
        raise SystemExit("No valid magnetometer samples found")

    start_ms = rows[0][0]
    seconds = [(row[0] - start_ms) / 1000.0 for row in rows]
    x_values = [row[1] for row in rows]
    y_values = [row[2] for row in rows]
    z_values = [row[3] for row in rows]
    norms = [row[4] for row in rows]

    fig, (axis_xyz, axis_norm) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True, constrained_layout=True
    )
    axis_xyz.plot(seconds, x_values, label="X", linewidth=1.2)
    axis_xyz.plot(seconds, y_values, label="Y", linewidth=1.2)
    axis_xyz.plot(seconds, z_values, label="Z", linewidth=1.2)
    axis_xyz.set_title("LIS3MDL Magnetometer")
    axis_xyz.set_ylabel("Magnetic field (uT)")
    axis_xyz.grid(True, alpha=0.3)
    axis_xyz.legend(ncol=3)

    axis_norm.plot(seconds, norms, label="Magnitude", color="tab:purple", linewidth=1.3)
    axis_norm.set_xlabel("Elapsed time (s)")
    axis_norm.set_ylabel("Magnitude (uT)")
    axis_norm.grid(True, alpha=0.3)
    axis_norm.legend()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    print(f"samples={len(rows)} output={output_path}")


if __name__ == "__main__":
    main()
