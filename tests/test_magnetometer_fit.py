import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from fit_magnetometer import (correction_matrix, fit_robust,
                              read_magnetometer_csv, statistics)


def observations(seed, count, offset, distortion, field=48.0, outliers=0):
    rng = np.random.default_rng(seed)
    directions = rng.normal(size=(count, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    points = directions @ distortion.T * field + offset
    points += rng.normal(0, 0.12, size=points.shape)
    if outliers:
        indices = rng.choice(count, size=outliers, replace=False)
        points[indices] += rng.normal(0, 45, size=(outliers, 3))
    return points


class MagnetometerFitTests(unittest.TestCase):
    def test_independent_rotation_recovers_field_with_outliers(self):
        offset = np.array([8.0, -5.0, 3.0])
        distortion = np.array([[1.13, 0.08, 0.02],
                               [0.02, 0.88, -0.05],
                               [0.04, 0.01, 1.04]])
        training = observations(2, 600, offset, distortion, outliers=45)
        validation = observations(3, 400, offset, distortion)
        center, shape, inliers, coverage = fit_robust(training)
        matrix = correction_matrix(shape, 48.0)
        report = statistics(validation, center, matrix, 48.0)
        self.assertGreater(inliers.mean(), 0.80)
        self.assertGreaterEqual(coverage["occupied_octants"], 6)
        self.assertLess(np.linalg.norm(center - offset), 1.0)
        self.assertLess(report["corrected_relative_error_p95"], 0.04)

    def test_yaw_only_is_rejected(self):
        angle = np.linspace(0, 4 * np.pi, 300)
        points = np.column_stack((30 * np.cos(angle), 30 * np.sin(angle),
                                  np.full_like(angle, 20)))
        with self.assertRaises(ValueError):
            fit_robust(points)

    def test_invalid_rows_are_not_used_for_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "sample.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(("time_ms", "mag_x_uT", "mag_y_uT", "mag_z_uT", "mag_valid"))
                writer.writerows(((0, 1, 2, 3, 1), (500, "nan", 2, 3, 1),
                                  (1000, 4, 5, 6, 0)))
            points, total = read_magnetometer_csv(path)
            self.assertEqual(total, 3)
            self.assertEqual(points.tolist(), [[1, 2, 3]])

    def test_cli_exports_only_validated_installed_calibration(self):
        offset = np.array([4.0, -2.0, 6.0])
        distortion = np.diag([1.10, 0.91, 1.04])
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            paths = []
            for name, seed, count in (("cal", 4, 420), ("val", 5, 350)):
                path = root / f"{name}.csv"
                with path.open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(("time_ms", "mag_x_uT", "mag_y_uT", "mag_z_uT", "mag_valid"))
                    for i, point in enumerate(observations(seed, count, offset, distortion)):
                        writer.writerow((i * 500, *point, 1))
                path.with_suffix(".json").write_text(
                    json.dumps({"phase": "installed", "state": "motors_off"}), encoding="utf-8")
                paths.append(path)
            header = root / "mag_calibration_params.h"
            command = [sys.executable, str(Path(__file__).resolve().parents[1] / "tools" / "fit_magnetometer.py"),
                       "--calibration", str(paths[0]), "--validation", str(paths[1]),
                       "--reference-ut", "48", "--output-dir", str(root / "fit"),
                       "--header", str(header)]
            subprocess.run(command, check=True, capture_output=True, text=True)
            self.assertIn("kEnabled = true", header.read_text(encoding="utf-8"))
            self.assertIn("kScienceReady = true", header.read_text(encoding="utf-8"))
            result = json.loads((root / "fit" / "calibration.json").read_text(encoding="utf-8"))
            self.assertEqual(result["status"], "absolute_reference")
            self.assertLess(result["validation"]["corrected_relative_error_p95"], 0.08)
            for path, phase in zip(paths, ("bench", "validation")):
                path.with_suffix(".json").write_text(
                    json.dumps({"phase": phase, "state": "motors_off"}), encoding="utf-8")
            bench_header = root / "bench_params.h"
            bench_command = [part for part in command if part not in ("--reference-ut", "48", "--header", str(header))]
            bench_command.extend(("--bench-header", str(bench_header)))
            subprocess.run(bench_command, check=True, capture_output=True, text=True)
            self.assertIn("kScienceReady = false", bench_header.read_text(encoding="utf-8"))
            self.assertIn("BENCH_ONLY-", bench_header.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
