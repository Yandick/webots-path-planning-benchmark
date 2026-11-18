"""Build the GitHub Pages replay from verified per-map 2D and Webots archives."""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from navigation.scenarios import MAP_FILE, SCENES, SCENE_INFO, make_scene
from navigation.settings import METHOD_LABELS


RESULT_ASSETS = (
    "benchmark.png", "manifest.json", "map.png", "raw_runs.json.gz",
    "smoothing_ablation.json.gz", "summary.csv", "course_maps.mp4", "trajectories.png",
)
NATIVE_ASSETS = ("webots_validation.json", "webots_raw_runs.json.gz")

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
    names = manifest["scenes"]
    if set(names) != set(SCENES) or len(names) != len(SCENES):
        raise ValueError("Manifest scenes do not match the course maps")
    map_hash = hashlib.sha256(MAP_FILE.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
    if map_hash != manifest["map_definitions_sha256"]:
        raise ValueError("Map definitions differ from the benchmark manifest")
    expected = {(name, method, seed) for name in names for seed in seeds for method in methods}
    if set(archive) != set(manifest["runs"]):
        raise ValueError("Archive filenames do not match the manifest")

    indexed = {}
    scenes = {(name, seed): make_scene(name, seed) for name in names for seed in seeds}
    for filename, run in archive.items():
        key = (run["scene"], run["algorithm"], run["seed"])
        if key not in expected or key in indexed:
            raise ValueError(f"Unexpected or duplicate archived run: {key}")
        if run["scene_sha256"] != scenes[run["scene"], run["seed"]].signature():
            raise ValueError(f"Scene geometry/schedule differs from archived run: {filename}")
        if run["backend"] != "kinematic_2d":
            raise ValueError(f"Unexpected backend or scene: {filename}")
        if run["status"] not in ("success", "collision", "timeout", "planning_failed"):
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
        key = (row["scene"], row["algorithm"], int(row["seed"]))
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


def export_webots(results, names, methods):
    native = results / NATIVE_ASSETS[0]
    raw = results / NATIVE_ASSETS[1]
    if not native.exists() and not raw.exists():
        return None
    data = _read_json(native)
    _finite_json(data, "webots")
    if hashlib.sha256(raw.read_bytes()).hexdigest() != data["raw_runs_sha256"]:
        raise ValueError("Native archive checksum differs from validation record")
    archive = _read_json(raw)
    seed = data["seed"]
    expected = {(name, method) for name in names for method in methods}
    runs = []
    for run in data["runs"]:
        name, method = run["scene"], run["algorithm"]
        if (name, method) not in expected or run["seed"] != seed:
            raise ValueError("Unexpected or duplicate native map/method/seed")
        expected.remove((name, method))
        if run["scene_sha256"] != make_scene(name, seed).signature():
            raise ValueError("Native scene differs from the replay map")
        recorded = archive.pop(run["raw_run"], None)
        fields = ("scene", "scene_sha256", "seed", "algorithm", "status", "finish_time",
                  "actual_path_length", "min_surface_clearance", "replan_count", "replan_attempt_count")
        if recorded is None or any(run[key] != recorded[key] for key in fields):
            raise ValueError("Native summary differs from its recorded run")
        runs.append({
            "scene": name, "method": method, "seed": seed, "status": run["status"],
            "finish_time": run["finish_time"], "path_length": run["actual_path_length"],
            "min_clearance": run["min_surface_clearance"], "replan_count": run["replan_count"],
            "replan_attempt_count": run["replan_attempt_count"],
        })
    if expected or archive:
        raise ValueError("Native archive and summary do not cover the same map/method runs")
    return {"version": data["webots_version"], "source_commit": data.get("source_commit"),
            "ci_url": data.get("ci_url"), "seed": seed, "runs": runs}


def create_payload(root: Path = ROOT):
    """Return browser data for every map, preserving samples and route changes."""
    results = Path(root) / "results"
    archive_path = results / "raw_runs.json.gz"
    archive = _read_json(archive_path)
    manifest = _read_json(results / "manifest.json")
    with (results / "summary.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    indexed, scenes, archive_hash = _validate_sources(archive, rows, manifest, archive_path)
    names, methods, seeds = manifest["scenes"], manifest["methods"], manifest["seeds"]
    maps, runs, summaries = [], [], []
    for name in names:
        scene = scenes[name, seeds[0]]
        info = SCENE_INFO[name]
        maps.append({
            "id": name, **{k: info[k] for k in ("label", "description", "source", "source_label")},
            "bounds": scene.bounds, "start": scene.start, "goal": scene.goal,
            "robot_radius": manifest["config"]["radius"],
            "static": [{"kind": obs.kind, **obs.params} for obs in scene.static],
            "moving": {str(seed): [vars(cart) for cart in scenes[name, seed].moving] for seed in seeds},
        })
        for method in methods:
            selected = [indexed[name, method, seed] for seed in seeds]
            successes = [run for run in selected if run["status"] == "success"]
            summaries.append({
                "scene": name, "method": method, "n": len(selected), "success": len(successes),
                **{state: sum(run["status"] == state for run in selected)
                   for state in ("collision", "timeout", "planning_failed")},
                "mean_time": _mean(successes, "finish_time"),
                "mean_path": _mean(successes, "actual_path_length"),
                "mean_clearance": _mean(selected, "min_surface_clearance"),
                "mean_curvature": _mean(selected, "planned_peak_curvature"),
                "mean_angular_accel": _mean(selected, "moving_angular_accel_rms"),
                "mean_replans": _mean(successes, "replan_count"),
            })
            for run in selected:
                runs.append({
                    "scene": name, "method": method, "seed": run["seed"], "status": run["status"],
                    "finish_time": run["finish_time"], "path_length": run["actual_path_length"],
                    "min_clearance": run["min_surface_clearance"], "replan_count": run["replan_count"],
                    "trajectory": [[p[k] for k in ("t", "x", "y", "theta", "clearance")] for p in run["trajectory"]],
                    "initial_path": run["initial_path"],
                    "route_updates": [{"t": item["t"], "path": item["path"]} for item in run["route_updates"]],
                })
    payload = {
        "schema_version": 2,
        "methods": [{"id": method, "label": METHOD_LABELS[method],
                     "group": "local" if method in ("apf", "dwa") else "global"} for method in methods],
        "scenes": maps, "seeds": seeds, "runs": runs, "summary": summaries,
        "webots": export_webots(results, names, methods),
        "sources": {
            "archive": "results/raw_runs.json.gz", "summary": "results/summary.csv",
            "manifest": "results/manifest.json", "raw_runs_sha256": archive_hash,
            "trajectory_columns": ["t_s", "x_m", "y_m", "theta_rad", "clearance_m"],
            "successful_runs_only": ["mean_time", "mean_path", "mean_replans"],
            "all_runs": ["mean_clearance", "mean_curvature", "mean_angular_accel"],
        },
    }
    _finite_json(payload)
    return _rounded(payload)


def _version_asset_references(source: Path, output: Path):
    """Version copied HTML assets by content, leaving source templates untouched."""
    reference = re.compile(
        r"(?P<attribute>(?<![\w:-])(?:href|src)\s*=\s*)"
        r"(?P<quote>[\"'])(?P<asset>(?:\./)?(?:style\.css|app\.js|data\.js))(?P=quote)",
        re.IGNORECASE,
    )
    fingerprints = {}
    for source_html in source.rglob("*.html"):
        built_html = output / source_html.relative_to(source)

        def version(match):
            asset = built_html.parent / match["asset"]
            if asset not in fingerprints:
                fingerprints[asset] = hashlib.sha256(asset.read_bytes()).hexdigest()[:12]
            return (f"{match['attribute']}{match['quote']}{match['asset']}"
                    f"?v={fingerprints[asset]}{match['quote']}")

        html = built_html.read_text(encoding="utf-8")
        built_html.write_text(reference.sub(version, html), encoding="utf-8")


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
    names = RESULT_ASSETS + (NATIVE_ASSETS if payload["webots"] else ())
    assets = [root / "results" / name for name in names]
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
    _version_asset_references(source, output)
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
