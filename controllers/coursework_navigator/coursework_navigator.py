"""Webots adapter for the same Navigator used by the explicit 2D backend."""
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np
from controller import Robot
from navigation.core import Navigator, angle
from navigation.scenarios import make_scene


def main():
    robot = Robot()
    step = int(robot.getBasicTimeStep())
    dt = step/1000
    meta = json.loads(robot.getCustomData())
    scene = make_scene(meta["scene"], meta["seed"])
    config = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
    method = os.environ.get("PLANNER_ALGO", config.get("algorithm", "replan_dwa"))
    nav = Navigator(scene, method)
    left, right = robot.getDevice("left wheel motor"), robot.getDevice("right wheel motor")
    for motor in (left, right):
        motor.setPosition(float("inf")); motor.setVelocity(0.)
    gps, imu, lidar = [robot.getDevice(n) for n in ("gps", "inertial_unit", "lidar")]
    for device in (gps, imu, lidar):
        device.enable(step)
    lidar.enablePointCloud()
    out = Path(os.environ.get("RUN_OUTPUT", str(ROOT / "results" / "webots")))
    out.mkdir(parents=True, exist_ok=True)
    tag = os.environ.get("RUN_ID", f"{scene.name}_{method}_seed{scene.seed}")
    truth_path = out / f"{tag}_supervisor.json"
    rows, previous = [], None
    velocity = (0., 0.)
    robot.step(step)
    while robot.step(step) != -1:
        t = robot.getTime()
        xyz, rpy = gps.getValues(), imu.getRollPitchYaw()
        pose = np.array([xyz[0], xyz[1], rpy[2]])
        if previous is not None:
            delta = pose[:2]-previous[:2]
            velocity = (max(0., float(delta @ np.array([math.cos(pose[2]), math.sin(pose[2])]))/dt),
                        angle(pose[2]-previous[2])/dt)
        previous = pose.copy()
        # Use native local point coordinates instead of guessing range-image ordering.
        local = np.array([[p.x, p.y] for p in lidar.getPointCloud()
                          if math.isfinite(p.x) and math.isfinite(p.y)])
        if len(local):
            cs, sn = math.cos(pose[2]), math.sin(pose[2])
            points = local @ np.array([[cs, sn], [-sn, cs]])+pose[:2]
        else:
            points = np.empty((0, 2))
        v, w = nav.step(pose, velocity, points, t, dt)
        left.setVelocity((v-.15*w)/.05); right.setVelocity((v+.15*w)/.05)
        rows.append({"t": t, "x": float(pose[0]), "y": float(pose[1]), "theta": float(pose[2]),
                     "v": v, "omega": w, "measured_v": velocity[0], "measured_omega": velocity[1]})
        if truth_path.exists() or t >= scene.duration or np.linalg.norm(pose[:2]-scene.goal) <= .25:
            break
    left.setVelocity(0.); right.setVelocity(0.)
    result = {"backend": "webots", "scene": scene.name, "seed": scene.seed,
              "scene_sha256": scene.signature(), "algorithm": method,
              "initial_path": nav.initial_path.tolist(), "path_history": nav.path_history,
              "replans": nav.replans, "replan_attempts": nav.replan_attempts,
              "replan_time_ms": nav.replan_ms, "trajectory": rows}
    (out / f"{tag}_navigator.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"[Navigator] Saved {tag}; replans={nav.replans}", flush=True)


if __name__ == "__main__":
    main()
