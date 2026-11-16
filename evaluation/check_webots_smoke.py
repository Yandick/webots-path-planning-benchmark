"""Validate native sensor geometry, complete execution and the VV path-length property."""
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import make_scene, static_distance
from navigation.settings import METHODS


def main():
    out = ROOT / "results" / "webots"
    manifest = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    runs = [json.loads((out / name).read_text(encoding="utf-8")) for name in manifest["runs"]]
    for run in runs:
        assert run["status"] in ("success", "timeout", "collision", "planning_failed"), run["status"]
        assert run["collision_count"] == int(run["status"] == "collision")
        if run["status"] != "collision":
            assert run["min_surface_clearance"] > 0
        scene = make_scene(run["scene"], run["seed"])
        errors = []
        for row in run["control_trajectory"]:
            local = row.get("nearest_lidar_local")
            if local is None:
                continue
            c, s = math.cos(row["theta"]), math.sin(row["theta"])
            point = np.array([row["x"], row["y"]])+np.array([[c,-s],[s,c]])@np.array(local[:2])
            dist = abs(float(static_distance(point, scene.static)))
            for obs in scene.moving:
                dist = min(dist, abs(float(np.linalg.norm(point-obs.position(row["t"]))) - obs.radius))
            errors.append(dist)
        # Includes one-step sensor lag, polygon tessellation and rasterization error.
        if run["status"] != "planning_failed":
            assert errors and max(errors)<.08, (run["scene"], run["algorithm"], max(errors, default=-1))
            print(f"{run['scene']} {run['algorithm']}: native nearest-point geometry error max={max(errors):.4f}m")
    indexed = {r["algorithm"]:r for r in runs}
    assert set(indexed) == set(METHODS), "Missing native method run"
    proposed = indexed["voronoi_visibility"]
    assert proposed["status"] == "success", "Improved method did not reach the goal"
    assert proposed["planned_path_length"] <= indexed["voronoi"]["planned_path_length"]+1e-8
    for method, run in indexed.items():
        print(f"Native warehouse {method}: {run['status']} at {run['finish_time']:.3f}s")


if __name__ == "__main__":
    main()
