"""Build the project homepage from the checked-in experiment archives.

Run ``python scripts/build_site.py`` to copy ``site/`` and selected research
assets into ``dist/``. No simulation is run and no source result is rewritten.

``data.js`` assigns ``window.BENCHMARK_DATA`` with the following schema:

* methods: [{id, label, group}], using stable experiment IDs and current labels;
* scene: {bounds, start, goal, robot_radius, static: [{center, size}]};
* seeds and moving: per-seed scripted cart {name, radius, waypoints} schedules;
* runs: [{method, seed, status, finish_time, path_length, min_clearance,
  replan_count, trajectory: [[t, x, y, theta, clearance]], initial_path,
  route_updates: [{t, path}]}]; every recorded trajectory sample is retained;
* summary: [{method, n, success, collision, timeout, mean_time, mean_path,
  mean_clearance, mean_curvature, mean_angular_accel, mean_replans}]; time,
  path length and replans average successful runs only, other metrics all runs;
* webots: {version, source_commit, ci_url, seed, runs}; these are archived native
  validation results, not a claim that this website build reran Webots;
* sources: source paths, archive checksum, source commit and metric definitions.

Coordinates and times are rounded to six decimal places for the browser;
downloadable archives remain byte-for-byte copies of the originals. An absent
metric is null, never a fabricated zero. The build rejects missing, duplicate,
inconsistent or non-finite research data rather than publishing a partial set.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from navigation.scenarios import make_scene
from navigation.settings import METHOD_LABELS


RESULT_ASSETS = (
    "benchmark.png", "collision_case.png", "manifest.json", "map.png",
    "raw_runs.json.gz", "safety_smoothness.png", "smoothing_ablation.json.gz",
    "summary.csv", "voronoi_smoothing.png", "warehouse.mp4", "warehouse.png",
    "webots_raw_runs.json.gz", "webots_scene.png", "webots_validation.json",
)

CSV_NUMERIC_FIELDS = (
    "finish_time", "actual_path_length", "min_surface_clearance",
    "replan_count", "collision_count", "planned_peak_curvature",
    "moving_angular_accel_rms",
)


def _read_json(path: Path):
    if path.suffix == ".gz":
        with gzip.open(path, "rt", encoding="utf-8") as stream:
            return json.load(stream)
    return json.loads(path.read_text(encoding="utf-8"))


def _finite_json(value, context="data"):
    """Reject NaN/Infinity before rounding or serializing a scientific value."""
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"Non-finite value in {context}")
    if isinstance(value, dict):
        for key, item in value.items():
            _finite_json(item, f"{context}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _finite_json(item, f"{context}[{index}]")


def _rounded(value):
    if isinstance(value, float):
        return round(value, 6)
    if isinstance(value, dict):
        return {key: _rounded(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_rounded(item) for item in value]
    return value


def _mean(runs, key):
    values = [run[key] for run in runs if run.get(key) is not None]
    return statistics.fmean(values) if values else None


def _validate_sources(archive, rows, manifest, archive_path):
    _finite_json(archive, "archive")
    _finite_json(manifest, "manifest")
    expected_hash = manifest.get("raw_runs_sha256")
    actual_hash = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    if expected_hash and actual_hash != expected_hash:
        raise ValueError("raw_runs.json.gz does not match manifest raw_runs_sha256")

    methods = manifest["methods"]
    seeds = manifest["seeds"]
    if len(methods) != len(set(methods)) or len(seeds) != len(set(seeds)):
        raise ValueError("Duplicate method or seed in manifest")
    if set(methods) != set(METHOD_LABELS):
        raise ValueError("Manifest methods do not match the configured methods")
    expected = {(method, seed) for seed in seeds for method in methods}
    if set(archive) != set(manifest["runs"]):
        raise ValueError("Archive filenames do not match the manifest")

    indexed = {}
    scenes = {seed: make_scene(manifest["scene"], seed) for seed in seeds}
    for filename, run in archive.items():
        key = (run["algorithm"], run["seed"])
        if key not in expected or key in indexed:
            raise ValueError(f"Unexpected or duplicate archived run: {key}")
        if run["scene_sha256"] != scenes[run["seed"]].signature():
            raise ValueError(f"Scene geometry/schedule differs from archived run: {filename}")
        if run["backend"] != "kinematic_2d" or run["scene"] != manifest["scene"]:
            raise ValueError(f"Unexpected backend or scene: {filename}")
        if run["status"] not in ("success", "collision", "timeout"):
            raise ValueError(f"Unknown run status: {filename}")
        trajectory = run["trajectory"]
        if not trajectory:
            raise ValueError(f"Empty trajectory: {filename}")
        times = [point["t"] for point in trajectory]
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError(f"Non-increasing trajectory timestamps: {filename}")
        if not math.isclose(times[-1], run["finish_time"], abs_tol=1e-8):
            raise ValueError(f"Trajectory end differs from finish_time: {filename}")
        if not math.isclose(run["config"]["radius"], manifest["config"]["radius"]):
            raise ValueError(f"Robot radius differs from manifest: {filename}")
        indexed[key] = run
    if set(indexed) != expected:
        raise ValueError("Archive is missing a method/seed run")

    csv_keys = set()
    for row in rows:
        key = (row["algorithm"], int(row["seed"]))
        if key not in indexed or key in csv_keys:
            raise ValueError(f"Unexpected or duplicate CSV run: {key}")
        csv_keys.add(key)
        run = indexed[key]
        if row["status"] != run["status"]:
            raise ValueError(f"CSV/archive status mismatch: {key}")
        if row["scene_sha256"] != run["scene_sha256"]:
            raise ValueError(f"CSV/archive scene mismatch: {key}")
        for field in CSV_NUMERIC_FIELDS:
            raw = row[field]
            value = None if raw in ("", "None") else float(raw)
            expected_value = run[field]
            if value is None or expected_value is None:
                matches = value is None and expected_value is None
            else:
                matches = math.isfinite(value) and math.isclose(
                    value, expected_value, rel_tol=1e-10, abs_tol=1e-10)
            if not matches:
                raise ValueError(f"CSV/archive {field} mismatch: {key}")
    if csv_keys != expected:
        raise ValueError("CSV is missing a method/seed run")
    return indexed, scenes, actual_hash


def create_payload(root: Path = ROOT):
    """Return verified browser data without changing any input files."""
    root = Path(root)
    results = root / "results"
    archive_path = results / "raw_runs.json.gz"
    archive = _read_json(archive_path)
    manifest = _read_json(results / "manifest.json")
    webots = _read_json(results / "webots_validation.json")
    with (results / "summary.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    indexed, scenes, archive_hash = _validate_sources(archive, rows, manifest, archive_path)
    _finite_json(webots, "webots")
    methods, seeds = manifest["methods"], manifest["seeds"]
    scene = scenes[seeds[0]]
    if any(obstacle.kind != "box" for obstacle in scene.static):
        raise ValueError("The website scene exporter currently supports box obstacles")
    runs = []
    for seed in seeds:
        for method in methods:
            run = indexed[(method, seed)]
            runs.append({
                "method": method, "seed": seed, "status": run["status"],
                "finish_time": run["finish_time"],
                "path_length": run["actual_path_length"],
                "min_clearance": run["min_surface_clearance"],
                "replan_count": run["replan_count"],
                "trajectory": [[p[key] for key in ("t", "x", "y", "theta", "clearance")]
                               for p in run["trajectory"]],
                "initial_path": run["initial_path"],
                "route_updates": [{"t": update["t"], "path": update["path"]}
                                  for update in run["route_updates"]],
            })
    summaries = []
    for method in methods:
        selected = [indexed[(method, seed)] for seed in seeds]
        successes = [run for run in selected if run["status"] == "success"]
        summaries.append({
            "method": method, "n": len(selected), "success": len(successes),
            "collision": sum(run["status"] == "collision" for run in selected),
            "timeout": sum(run["status"] == "timeout" for run in selected),
            "mean_time": _mean(successes, "finish_time"),
            "mean_path": _mean(successes, "actual_path_length"),
            "mean_clearance": _mean(selected, "min_surface_clearance"),
            "mean_curvature": _mean(selected, "planned_peak_curvature"),
            "mean_angular_accel": _mean(selected, "moving_angular_accel_rms"),
            "mean_replans": _mean(successes, "replan_count"),
        })
    native_runs = [{
        "method": run["algorithm"], "seed": run["seed"], "status": run["status"],
        "finish_time": run["finish_time"], "path_length": run["actual_path_length"],
        "min_clearance": run["min_surface_clearance"],
        "replan_count": run["replan_count"],
        "replan_attempt_count": run["replan_attempt_count"],
    } for run in webots["runs"]]
    if {run["method"] for run in native_runs} != set(methods):
        raise ValueError("Native Webots validation methods differ from the benchmark")
    payload = {
        "schema_version": 1,
        "methods": [{"id": method, "label": METHOD_LABELS[method],
                     "group": "local" if method in ("apf", "dwa") else "global"}
                    for method in methods],
        "scene": {
            "bounds": [-5, 5, -5, 5], "start": scene.start, "goal": scene.goal,
            "robot_radius": manifest["config"]["radius"],
            "static": [{"center": obstacle.params["center"],
                        "size": obstacle.params["size"]} for obstacle in scene.static],
        },
        "seeds": seeds,
        "moving": {str(seed): [vars(cart) for cart in scenes[seed].moving] for seed in seeds},
        "runs": runs, "summary": summaries,
        "webots": {"version": webots["webots_version"],
                   "source_commit": webots["source_commit"], "ci_url": webots["ci_url"],
                   "seed": webots["seed"], "runs": native_runs},
        "sources": {
            "archive": "results/raw_runs.json.gz", "summary": "results/summary.csv",
            "manifest": "results/manifest.json", "raw_runs_sha256": archive_hash,
            "source_commit": manifest.get("source_commit"),
            "trajectory_columns": ["t_s", "x_m", "y_m", "theta_rad", "clearance_m"],
            "successful_runs_only": ["mean_time", "mean_path", "mean_replans"],
            "all_runs": ["mean_clearance", "mean_curvature", "mean_angular_accel"],
        },
    }
    _finite_json(payload)
    return _rounded(payload)


def build(root: Path = ROOT, output_dir: Path | None = None):
    """Copy only site files and selected assets; never delete the destination."""
    root = Path(root).resolve()
    output = Path(output_dir).resolve() if output_dir is not None else root / "dist"
    source = root / "site"
    if output == root or output in root.parents:
        raise ValueError("Output directory must not contain or replace the repository")
    for protected in (source, root / "results", root / "navigation", root / "docs"):
        if output == protected or protected in output.parents:
            raise ValueError("Output directory must not overwrite source files")
    if not (source / "index.html").is_file():
        raise FileNotFoundError("site/index.html is required before building the homepage")
    payload = create_payload(root)
    assets = [root / "results" / name for name in RESULT_ASSETS]
    for asset in assets:
        if not asset.is_file():
            raise FileNotFoundError(f"Missing downloadable result asset: {asset.name}")
    for path in source.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"Site source must not contain symlinks: {path}")
    output.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, output, dirs_exist_ok=True)
    (output / "results").mkdir(exist_ok=True)
    for asset in assets:
        shutil.copy2(asset, output / "results" / asset.name)
    serialized = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    (output / "data.js").write_text(
        "// Generated from archived experiments; run python scripts/build_site.py to refresh.\n"
        f"window.BENCHMARK_DATA = {serialized};\n", encoding="utf-8")
    (output / ".nojekyll").touch()
    return output, payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "dist",
                        help="Output folder (default: repository dist/)")
    args = parser.parse_args()
    output, payload = build(output_dir=args.output_dir)
    print(f"Built {output}: {len(payload['runs'])} recorded runs, "
          f"{len(payload['methods'])} methods, {len(payload['seeds'])} seeds.")


if __name__ == "__main__":
    main()
