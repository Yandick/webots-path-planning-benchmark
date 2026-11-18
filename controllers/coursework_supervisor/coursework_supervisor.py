"""Scripted kinematic actors and independent geometric collision auditing."""
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np
from controller import Supervisor
from navigation.scenarios import make_scene, clearance, validate_scene
from navigation.settings import resolve_method


def main():
    supervisor = Supervisor()
    step = int(supervisor.getBasicTimeStep())
    meta = json.loads(supervisor.getCustomData())
    scene = make_scene(meta["scene"], meta["seed"])
    validate_scene(scene)
    nodes = [supervisor.getFromDef(f"MOVING_{i}") for i in range(len(scene.moving))]
    robot = supervisor.getFromDef("ROBOT")
    method = resolve_method()
    tag = os.environ.get("RUN_ID", f"{scene.name}_{method}_seed{scene.seed}")
    out = Path(os.environ.get("RUN_OUTPUT", str(ROOT / "results" / "webots")))
    out.mkdir(parents=True, exist_ok=True)
    navigator_path = out / f"{tag}_navigator.json"
    rows, min_gap, length, prev = [], float("inf"), 0., None
    status = "timeout"
    snapshots = set()
    recording = os.environ.get("RECORD_VIDEO", "0") == "1"
    if recording:
        supervisor.movieStartRecording(str(out / f"{tag}.mp4"), 1280, 720, 0, 85, 1, False)
    while supervisor.step(step) != -1:
        t = supervisor.getTime()
        # These are kinematic scripted carts. Their schedule is fixed before any run.
        for node, obs in zip(nodes, scene.moving):
            x, y = obs.position(t)
            node.getField("translation").setSFVec3f([float(x), float(y), .35])
        p = np.array(robot.getPosition()[:2])
        gap = clearance(scene, p, t)
        min_gap = min(min_gap, gap)
        if prev is not None:
            length += float(np.linalg.norm(p-prev))
        prev = p
        rows.append({"t": t, "x": float(p[0]), "y": float(p[1]), "clearance": gap,
                     "obstacles": [[*o.position(t), o.radius] for o in scene.moving]})
        for snapshot in (12, 28):
            if t >= snapshot and snapshot not in snapshots:
                supervisor.exportImage(str(out / f"{tag}_t{snapshot}.png"), 100)
                snapshots.add(snapshot)
        if gap <= 0:
            status = "collision"; break
        if navigator_path.exists() and json.loads(navigator_path.read_text(encoding="utf-8")).get("planning_failed"):
            status = "planning_failed"; break
        if np.linalg.norm(p-scene.goal) <= .25:
            status = "success"; break
        if t >= scene.duration:
            break
    result = {"backend": "webots", "scene": scene.name, "seed": scene.seed,
              "scene_sha256": scene.signature(), "algorithm": method, "status": status,
              "success": status=="success", "collision_count": int(status=="collision"),
              "min_surface_clearance": min_gap, "actual_path_length": length,
              "finish_time": rows[-1]["t"], "trajectory": rows,
              "collision_model": "signed geometry distance minus 0.18m robot radius at every 32ms step"}
    temporary = out / f"{tag}_supervisor.tmp"
    temporary.write_text(json.dumps(result, indent=2), encoding="utf-8")
    temporary.replace(out / f"{tag}_supervisor.json")
    if recording:
        supervisor.movieStopRecording()
        while not supervisor.movieIsReady():
            if supervisor.step(step)==-1:
                break
    # Let navigator observe the outcome and flush its own log.
    for _ in range(16):
        supervisor.step(step)
    print(f"[Supervisor] {tag}: {status} clearance={min_gap:.3f}", flush=True)
    if os.environ.get("BENCHMARK_MODE", "0")=="1":
        supervisor.simulationQuit(0 if status=="success" else 1)
    else:
        supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)


if __name__ == "__main__":
    main()
