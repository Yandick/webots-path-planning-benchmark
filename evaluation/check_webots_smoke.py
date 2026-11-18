"""Validate native controller logs and optionally export the complete seed-0 comparison."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import SCENES, make_scene, static_distance
from navigation.settings import METHODS, YIELD_BASES


def validate_run(run):
    scene = make_scene(run["scene"], run["seed"])
    label = (scene.name, run["algorithm"], run["seed"])
    if run["backend"] != "webots" or run["scene_sha256"] != scene.signature():
        raise ValueError(f"Different map or backend in {label}")
    status = run["status"]
    if status not in ("success", "timeout", "collision", "planning_failed"):
        raise ValueError(f"Unknown status for {label}: {status}")
    if run["success"] != (status == "success") or run["collision_count"] != int(status == "collision"):
        raise ValueError(f"Inconsistent outcome for {label}")
    trajectory = run["trajectory"]
    if not trajectory or not run["control_trajectory"]:
        raise ValueError(f"Missing controller or supervisor observations for {label}")
    times = [row["t"] for row in trajectory]
    if (any(b <= a for a, b in zip(times, times[1:])) or times[-1] != run["finish_time"]
            or min(row["clearance"] for row in trajectory) != run["min_surface_clearance"]
            or (run["min_surface_clearance"] <= 0) != (status == "collision")):
        raise ValueError(f"Inconsistent supervisor observations for {label}")
    if any(len(row["obstacles"]) != len(scene.moving) for row in trajectory):
        raise ValueError(f"Different moving obstacles for {label}")
    positions = np.asarray([row["obstacles"] for row in trajectory])
    for index, cart in enumerate(scene.moving):
        if (not np.allclose(positions[:, index, :2], cart.position(times), rtol=0., atol=1e-8)
                or not np.all(positions[:, index, 2] == cart.radius)):
            raise ValueError(f"Different cart schedule or footprint for {label}")
    if run["algorithm"] in ("apf", "dwa") and (run["role"], run["guide_planner"]) != ("local_controller", "astar"):
        raise ValueError(f"Local controller was not guided by A* in {label}")
    # Earlier baseline-only archives predate explicit yielding and recorded no such events.
    if run["algorithm"] in YIELD_BASES and not {"yield_events", "yield_time_s", "yield_count"} <= run.keys():
        raise ValueError(f"Missing yielding measurements in {label}")
    events = run.get("yield_events", [])
    last_end = 0.
    for event in events:
        start, end = event["start"], event["end"]
        if not (start >= 0. and start + 1e-9 >= last_end and start < end <= run["finish_time"] + 1e-9):
            raise ValueError(f"Yield interval outside the experiment or out of order in {label}")
        last_end = end
    total = sum(event["end"] - event["start"] for event in events)
    if (run.get("yield_count", 0) != len(events) or not math.isclose(total, run.get("yield_time_s", 0.), abs_tol=1e-6)
            or (events and run["algorithm"] not in YIELD_BASES)):
        raise ValueError(f"Inconsistent yielding measurements in {label}")
    errors = []
    for row in run["control_trajectory"]:
        local = row.get("nearest_lidar_local")
        if local is None:
            continue
        c, s = math.cos(row["theta"]), math.sin(row["theta"])
        point = np.array([row["x"], row["y"]]) + np.array([[c, -s], [s, c]]) @ local[:2]
        distance = abs(float(static_distance(point, scene.static)))
        for obs in scene.moving:
            distance = min(distance, abs(np.linalg.norm(point - obs.position(row["t"])) - obs.radius))
        errors.append(float(distance))
    # The 8 cm bound includes sensor lag, native polygon surfaces and rasterization.
    maximum_error = max(errors, default=None)
    if status != "planning_failed" and (maximum_error is None or maximum_error >= .08):
        raise ValueError(f"Lidar surface error for {label}: {maximum_error} m")
    return maximum_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "results" / "webots")
    parser.add_argument("--export", type=Path, help="Write seed-0 summary and compressed raw runs here")
    args = parser.parse_args()
    manifest = json.loads((args.input / "manifest.json").read_text(encoding="utf-8"))
    scenes, methods, seeds = (manifest[key] for key in ("scenes", "methods", "seeds"))
    if (manifest["backend"] != "webots" or not manifest["webots_version"]
            or any(not values or len(values) != len(set(values)) for values in (scenes, methods, seeds))
            or not set(scenes) <= set(SCENES) or not set(methods) <= set(METHODS)):
        raise ValueError("Unexpected native run manifest")
    expected = {(scene, method, seed) for scene in scenes for method in methods for seed in seeds}
    runs, observed, summaries = {}, set(), []
    fields = ("scene", "algorithm", "seed", "scene_sha256", "status", "success", "collision_count",
              "finish_time", "actual_path_length", "min_surface_clearance", "role", "guide_planner",
              "replan_count", "replan_attempt_count", "stationary_time_s", "longest_pause_s")
    for name in manifest["runs"]:
        if Path(name).name != name:
            raise ValueError(f"Invalid native run file name: {name}")
        run = json.loads((args.input / name).read_text(encoding="utf-8"))
        key = (run["scene"], run["algorithm"], run["seed"])
        if key in observed or key not in expected:
            raise ValueError(f"Unexpected or duplicate native run: {key}")
        observed.add(key)
        max_error = validate_run(run)
        print(f"{key}: {run['status']} at {run['finish_time']:.3f}s")
        canonical = f"{key[0]}_{key[1]}_seed{key[2]}.json"
        runs[canonical] = run
        summaries.append({**{field: run[field] for field in fields}, "raw_run": canonical,
                          "nearest_lidar_geometry_max_error_m": max_error,
                          "yield_count": run.get("yield_count", 0), "yield_time_s": run.get("yield_time_s", 0.)})
    if expected != observed:
        raise ValueError(f"Missing native runs: {sorted(expected - observed)}")
    for scene, method, seed in observed:
        base = YIELD_BASES.get(method)
        if base is None or (scene, base, seed) not in observed:
            continue
        variant = runs[f"{scene}_{method}_seed{seed}.json"]
        reference = runs[f"{scene}_{base}_seed{seed}.json"]
        if variant["guide_planner"] != base or not reference.get("config") or any(
                variant[key] != reference[key] for key in ("initial_path", "guide_planner", "role", "config", "control_step_s")):
            raise ValueError(f"Different initial plan or control parameters for {scene}/{base}/{method}/seed{seed}")
    if set(scenes) == set(SCENES):
        for method in {"astar", "rrt_star"} & set(methods):
            if not any(row["replan_attempt_count"] for row in summaries if row["algorithm"] == method):
                raise ValueError(f"No online replanning attempt was observed across the maps for {method}")
    if args.export:
        if set(scenes) != set(SCENES) or set(methods) != set(METHODS) or seeds != [0]:
            raise ValueError("The homepage comparison requires seed 0 for every scene and method.")
        output = args.export
        output.mkdir(parents=True, exist_ok=True)
        raw = gzip.compress(json.dumps(runs, allow_nan=False, separators=(",", ":")).encode("utf-8"), mtime=0)
        summary = {"backend": "webots", "webots_version": manifest["webots_version"],
                   "source_commit": manifest.get("source_commit"), "ci_url": manifest.get("ci_url"),
                   "scenes": scenes, "seed": 0, "runs": summaries,
                   "raw_runs_sha256": hashlib.sha256(raw).hexdigest()}
        encoded = json.dumps(summary, allow_nan=False, indent=2) + "\n"
        raw_path = output / "webots_raw_runs.json.gz"
        summary_path = output / "webots_validation.json"
        raw_temp = raw_path.with_suffix(".gz.tmp")
        summary_temp = summary_path.with_suffix(".json.tmp")
        raw_temp.write_bytes(raw)
        summary_temp.write_text(encoded, encoding="utf-8")
        raw_temp.replace(raw_path)
        summary_temp.replace(summary_path)


if __name__ == "__main__":
    main()
