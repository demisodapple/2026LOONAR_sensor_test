import csv
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from capture_serial import FIRMWARE_HEADER, LEGACY_FIRMWARE_HEADER, capture


class CaptureTests(unittest.TestCase):
    def test_captures_rows_after_firmware_header_was_missed(self):
        row = ["500"] + ["1"] * (len(FIRMWARE_HEADER) - 1)
        csv_out, raw_out = io.StringIO(), io.StringIO()
        self.assertEqual(capture(["# already running", ",".join(row)], csv_out, raw_out), 1)
        rows = list(csv.reader(io.StringIO(csv_out.getvalue())))
        self.assertEqual(rows, [FIRMWARE_HEADER, row])

    def test_legacy_rows_without_header(self):
        row = ["500"] + ["1"] * (len(LEGACY_FIRMWARE_HEADER) - 1)
        csv_out, raw_out = io.StringIO(), io.StringIO()
        self.assertEqual(capture([",".join(row)], csv_out, raw_out), 1)
        self.assertEqual(list(csv.reader(io.StringIO(csv_out.getvalue()))),
                         [LEGACY_FIRMWARE_HEADER, row])

    def test_lis_only_schema(self):
        self.assertEqual(len(FIRMWARE_HEADER), 20)
        self.assertFalse(any("qmc" in field or "lis3mdl_" in field
                             for field in FIRMWARE_HEADER))

    def test_preserves_invalid_rows_and_ignores_diagnostics(self):
        header = "time_ms,mag_x_uT,mag_y_uT,mag_z_uT,mag_valid"
        lines = ["# LIS3MDL BUS STATE", header, "500,1,2,3,1",
                 "# periodic reinit", header, "1000,nan,nan,nan,0"]
        csv_out, raw_out = io.StringIO(), io.StringIO()
        self.assertEqual(capture(lines, csv_out, raw_out), 2)
        rows = list(csv.reader(io.StringIO(csv_out.getvalue())))
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[-1][-1], "0")
        self.assertIn("# periodic reinit", raw_out.getvalue())


if __name__ == "__main__":
    unittest.main()
