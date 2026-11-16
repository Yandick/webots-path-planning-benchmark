"""Audit original geometry and approximate robot/actor interactions; infer no intent."""
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation.scenarios import GridMap, static_distance


def actors(t):
    t = np.asarray(t)
    phase = np.sin(2*np.pi*t/12)
    return [np.column_stack((-1+1.5*np.sin(2*np.pi*t/10), np.full_like(t, -.6))),
            np.column_stack((np.full_like(t, .6), .8+1.6*np.sin(2*np.pi*t/8))),
            np.column_stack((-.7+1.1*phase, 1.8+.6*phase))]


def main():
    grid = GridMap()
    radii = [.35, .35, .3]
    times = np.linspace(0, 120, 12001)
    overlap = [float(np.mean(static_distance(p, grid.obstacles) < r))
               for p, r in zip(actors(times), radii)]
    result = {"audit_period_s": [0, 120], "audit_step_s": .01,
              "dynamic_static_overlap_fraction": overlap,
              "max_actor_speed_m_s": [1.5*2*np.pi/10, 1.6*2*np.pi/8, math.hypot(1.1, .6)*2*np.pi/12],
              "interaction_estimates": [],
              "interpretation": "Static geometry has matching physical boundingObjects. Dynamic schedules intersect static geometry. Old logs lack synchronized actor states; reconstructed gaps are estimates, not confirmed collisions. No inference about author intent or reward optimization."}
    for key in ("a_star_dwa", "rrt_star_dwa", "informed_rrt_star_dwa"):
        d = json.loads((ROOT / "results" / f"trajectory_{key}.json").read_text(encoding="utf-8"))
        tr = d["trajectory"]
        p = np.array([[r["x"], r["y"]] for r in tr])
        t = np.array([r["t"] for r in tr])
        for offset in (0., .096, .128):
            gaps = [float(np.min(np.linalg.norm(p-q, axis=1)-r-.18)) for q, r in zip(actors(t+offset), radii)]
            result["interaction_estimates"].append({"algorithm": key, "assumed_time_offset_s": offset,
                                                     "min_surface_gaps_m": gaps})
    out = ROOT / "results" / "revised" / "legacy_audit.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
