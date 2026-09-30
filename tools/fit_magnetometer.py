"""Fit a 3D hard/soft-iron correction from a dedicated, uniform-field rotation.

Requires NumPy. Training and validation must be separate rotation sessions.
Never fit this to moving-rover science observations: real spatial anomalies can
otherwise be absorbed by the calibration.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def read_magnetometer_csv(path):
    points = []
    total = 0
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        expected = {"mag_x_uT", "mag_y_uT", "mag_z_uT", "mag_valid"}
        if not expected.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: missing {sorted(expected)}")
        for row in reader:
            total += 1
            if row["mag_valid"] != "1":
                continue
            try:
                point = [float(row[key]) for key in ("mag_x_uT", "mag_y_uT", "mag_z_uT")]
            except (TypeError, ValueError):
                continue
            if np.isfinite(point).all():
                points.append(point)
    return np.asarray(points, dtype=float).reshape((-1, 3)), total


def design(points, origin, scale):
    x, y, z = ((points - origin) / scale).T
    return np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z,
                            2*x, 2*y, 2*z))


def algebraic_fit(points, origin, scale):
    d = design(points, origin, scale)
    coefficients, _, rank, _ = np.linalg.lstsq(d, np.ones(len(d)), rcond=None)
    if rank != 9:
        raise ValueError("3D directions are insufficient to fit an ellipsoid")
    a, b, c, dxy, dxz, dyz, px, py, pz = coefficients
    q = np.array(((a, dxy, dxz), (dxy, b, dyz), (dxz, dyz, c)))
    eigenvalues = np.linalg.eigvalsh(q)
    if eigenvalues[0] <= 0 or eigenvalues[-1] / eigenvalues[0] > 100:
        raise ValueError("fit is not a well-conditioned ellipsoid")
    center_scaled = -np.linalg.solve(q, [px, py, pz])
    radius_sq = 1 + center_scaled @ q @ center_scaled
    if radius_sq <= 0 or not np.isfinite(radius_sq):
        raise ValueError("ellipsoid radius is invalid")
    center = origin + scale * center_scaled
    shape = q / (radius_sq * scale * scale)
    if np.linalg.norm(center - origin) > 2 * scale:
        raise ValueError("ellipsoid center is far outside the measurements")
    return center, shape


def relative_radius(points, center, shape):
    shifted = points - center
    squared = np.einsum("ni,ij,nj->n", shifted, shape, shifted)
    return np.sqrt(np.maximum(squared, 0))


def coverage(points, center):
    shifted = points - center
    spans = np.percentile(shifted, 95, axis=0) - np.percentile(shifted, 5, axis=0)
    octants = len({tuple(row >= 0) for row in shifted})
    covariance_eigen = np.linalg.eigvalsh(np.cov(shifted, rowvar=False))
    return {
        "occupied_octants": octants,
        "axis_span_ratio": float(np.min(spans) / np.max(spans)),
        "covariance_ratio": float(covariance_eigen[0] / covariance_eigen[-1]),
    }


def fit_robust(points, *, trials=400, tolerance=0.08):
    if len(points) < 120:
        raise ValueError("need at least 120 valid 3D rotation samples")
    origin = np.median(points, axis=0)
    scale = float(np.median(np.linalg.norm(points - origin, axis=1)))
    if scale < 1 or not np.isfinite(scale):
        raise ValueError("magnetic field spread is too small")
    rng = np.random.default_rng(20260930)
    best = None
    best_rank = (-1, -np.inf)
    for _ in range(trials):
        indices = rng.choice(len(points), size=12, replace=False)
        try:
            center, shape = algebraic_fit(points[indices], origin, scale)
        except (ValueError, np.linalg.LinAlgError):
            continue
        errors = np.abs(relative_radius(points, center, shape) - 1)
        inliers = errors < tolerance
        rank = (int(inliers.sum()), -float(np.median(errors[inliers])) if inliers.any() else -np.inf)
        if rank > best_rank:
            best, best_rank = (center, shape, inliers), rank
    if best is None or best_rank[0] < max(100, int(0.65 * len(points))):
        raise ValueError("no consensus ellipsoid; check interference and 3D coverage")
    center, shape, inliers = best
    for _ in range(4):
        center, shape = algebraic_fit(points[inliers], origin, scale)
        new_inliers = np.abs(relative_radius(points, center, shape) - 1) < tolerance
        if new_inliers.sum() < max(100, int(0.65 * len(points))):
            raise ValueError("refined fit lost its consensus")
        if np.array_equal(new_inliers, inliers):
            break
        inliers = new_inliers
    # One last fit to the final inlier set.
    center, shape = algebraic_fit(points[inliers], origin, scale)
    inliers = np.abs(relative_radius(points, center, shape) - 1) < tolerance
    geometry = coverage(points[inliers], center)
    if (geometry["occupied_octants"] < 6 or geometry["axis_span_ratio"] < 0.45
            or geometry["covariance_ratio"] < 0.025):
        raise ValueError(f"rotation did not cover enough 3D directions: {geometry}")
    return center, shape, inliers, geometry


def correction_matrix(shape, field_ut):
    eigenvalues, eigenvectors = np.linalg.eigh(shape)
    if eigenvalues[0] <= 0:
        raise ValueError("shape is not positive definite")
    return field_ut * (eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T)


def statistics(points, center, matrix, target_ut):
    raw = np.linalg.norm(points, axis=1)
    corrected = np.linalg.norm((points - center) @ matrix.T, axis=1)
    return {
        "samples": int(len(points)),
        "raw_norm_median_uT": float(np.median(raw)),
        "raw_norm_std_uT": float(np.std(raw)),
        "corrected_norm_median_uT": float(np.median(corrected)),
        "corrected_norm_std_uT": float(np.std(corrected)),
        "corrected_relative_error_p95": float(np.percentile(np.abs(corrected / target_ut - 1), 95)),
    }


def write_preview(path, points, center, matrix):
    corrected = (points - center) @ matrix.T
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("raw_x_uT", "raw_y_uT", "raw_z_uT", "raw_norm_uT",
                         "cal_x_uT", "cal_y_uT", "cal_z_uT", "cal_norm_uT"))
        for raw, cal in zip(points, corrected):
            writer.writerow([*raw, np.linalg.norm(raw), *cal, np.linalg.norm(cal)])


def float_literal(value):
    literal = f"{float(value):.9g}"
    if "." not in literal and "e" not in literal:
        literal += ".0"
    return literal + "f"


def write_header(path, calibration_id, center, matrix, science_ready):
    offsets = ", ".join(map(float_literal, center))
    rows = ",\n".join("    {" + ", ".join(map(float_literal, row)) + "}" for row in matrix)
    path.write_text(
        "#pragma once\n\n// Generated by tools/fit_magnetometer.py. Check kScienceReady.\n"
        "namespace magcal {\n"
        f"inline constexpr bool kEnabled = true;\ninline constexpr bool kScienceReady = {'true' if science_ready else 'false'};\n"
        f"inline constexpr char kId[] = \"{calibration_id}\";\n"
        f"inline constexpr float kOffsetUt[3] = {{{offsets}}};\n"
        f"inline constexpr float kMatrix[3][3] = {{\n{rows}\n}};\n"
        "}  // namespace magcal\n", encoding="utf-8")


def require_session(path, phase):
    sidecar = Path(path).with_suffix(".json")
    if not sidecar.exists():
        raise ValueError(f"{sidecar}: metadata required for firmware export")
    metadata = json.loads(sidecar.read_text(encoding="utf-8"))
    if metadata.get("phase") != phase or metadata.get("state") != "motors_off":
        raise ValueError(f"{sidecar}: expected {phase}, motors_off session")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True, help="Dedicated 3D rotation CSV")
    parser.add_argument("--validation", type=Path, help="Independent repeat of the 3D rotation")
    parser.add_argument("--reference-ut", type=float, help="Independent field magnitude in µT; required for firmware export")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--header", type=Path, help="Export an enabled Teensy header after all gates pass")
    parser.add_argument("--bench-header", type=Path, help="Export BENCH_ONLY Teensy header; never science-ready")
    parser.add_argument("--tolerance", type=float, default=0.08, help="RANSAC relative radial tolerance")
    args = parser.parse_args()
    if args.reference_ut is not None and (not np.isfinite(args.reference_ut) or args.reference_ut <= 0):
        parser.error("--reference-ut must be positive and finite")
    if not 0 < args.tolerance < 0.3:
        parser.error("--tolerance must be between 0 and 0.3")
    if args.header and args.bench_header:
        parser.error("choose --header or --bench-header, not both")
    if args.header and (args.reference_ut is None or args.validation is None):
        parser.error("science-ready header requires both --reference-ut and --validation")
    if args.bench_header and (args.reference_ut is not None or args.validation is None):
        parser.error("bench header requires independent --validation and no --reference-ut")
    if args.header:
        require_session(args.calibration, "installed")
        require_session(args.validation, "installed")
    if args.bench_header:
        require_session(args.calibration, "bench")
        require_session(args.validation, "validation")
    training, training_rows = read_magnetometer_csv(args.calibration)
    if training_rows == 0 or len(training) / training_rows < 0.80:
        raise ValueError("fewer than 80% of calibration rows have valid magnetometer XYZ")
    center, shape, inliers, geometry = fit_robust(training, tolerance=args.tolerance)
    # A sphere fit alone cannot identify the absolute field scale. Without an
    # external reference, this radius is only a bench-test visualization.
    reference = args.reference_ut if args.reference_ut is not None else float(
        np.median(np.linalg.norm(training[inliers] - center, axis=1)))
    matrix = correction_matrix(shape, reference)
    result = {
        "status": "absolute_reference" if args.reference_ut is not None else "relative_only_not_for_science",
        "calibration_csv": str(args.calibration.resolve()),
        "calibration_rows_total": training_rows,
        "calibration_inliers": int(inliers.sum()),
        "calibration_inlier_fraction": float(inliers.mean()),
        "coverage": geometry,
        "reference_uT": reference,
        "offset_uT": center.tolist(),
        "matrix": matrix.tolist(),
        "train": statistics(training[inliers], center, matrix, reference),
    }
    if args.validation:
        validation, validation_rows = read_magnetometer_csv(args.validation)
        if validation_rows == 0 or len(validation) / validation_rows < 0.80:
            raise ValueError("fewer than 80% of validation rows have valid magnetometer XYZ")
        if len(validation) < 120:
            raise ValueError("validation needs at least 120 valid samples")
        result["validation_csv"] = str(args.validation.resolve())
        result["validation_rows_total"] = validation_rows
        result["validation"] = statistics(validation, center, matrix, reference)
        result["validation_coverage"] = coverage(validation, center)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_preview(args.output_dir / "train_preview.csv", training, center, matrix)
    if args.validation:
        write_preview(args.output_dir / "validation_preview.csv", validation, center, matrix)
    digest = hashlib.sha256(np.asarray([*center, *matrix.ravel(), reference], dtype="<f8").tobytes()).hexdigest()[:12]
    result["id"] = f"mag3d-{digest}"
    (args.output_dir / "calibration.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    if args.header or args.bench_header:
        validation_ok = (result["validation"]["corrected_relative_error_p95"] <= args.tolerance
                         and result["validation_coverage"]["occupied_octants"] >= 6
                         and result["validation_coverage"]["axis_span_ratio"] >= 0.45
                         and result["validation_coverage"]["covariance_ratio"] >= 0.025)
        if not validation_ok:
            raise ValueError("independent validation did not pass; header not exported")
        science_ready = args.header is not None
        header_path = args.header if science_ready else args.bench_header
        calibration_id = result["id"] if science_ready else "BENCH_ONLY-" + result["id"]
        write_header(header_path, calibration_id, center, matrix, science_ready)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
