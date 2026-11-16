"""Validate native sensor geometry, successful execution and the blocked-route behavior."""
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
        assert run["status"]=="success" and run["min_surface_clearance"]>0, run["status"]
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
        assert errors and max(errors)<.08, (run["scene"], run["algorithm"], max(errors))
        print(f"{run['scene']} {run['algorithm']}: native nearest-point geometry error max={max(errors):.4f}m")
    indexed = {r["algorithm"]:r for r in runs}
    assert set(indexed) == set(METHODS), "Missing native method run"
    fixed = indexed["fixed_dwa"]
    proposed = indexed["replan_dwa"]
    assert proposed["replans"]>=1, "Blocked run never adopted a new route"
    assert proposed["finish_time"]<fixed["finish_time"], "Blocked-route mechanism did not improve arrival time"
    print(f"Native warehouse: {fixed['finish_time']:.3f}s -> {proposed['finish_time']:.3f}s")


if __name__ == "__main__":
    main()
