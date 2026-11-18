"""Webots adapter for the same Navigator used by the explicit 2D backend."""
from dataclasses import asdict
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
from navigation.metrics import path_metrics, control_metrics
from navigation.scenarios import make_scene
from navigation.settings import resolve_method


def main():
    robot = Robot()
    step = 3*int(robot.getBasicTimeStep())
    dt = step/1000
    meta = json.loads(robot.getCustomData())
    scene = make_scene(meta["scene"], meta["seed"])
    method = resolve_method()
    nav = Navigator(scene, method)
    left, right = robot.getDevice("left wheel motor"), robot.getDevice("right wheel motor")
    for motor in (left, right):
        motor.setPosition(float("inf")); motor.setVelocity(0.)
    gps, imu = [robot.getDevice(n) for n in ("gps", "inertial_unit")]
    lidars = [(robot.getDevice(name), yaw) for name, yaw in
              (("lidar",0.), ("lidar_left",math.pi/2), ("lidar_back",math.pi), ("lidar_right",-math.pi/2))]
    for device in (gps, imu, *(sensor for sensor, _ in lidars)):
        device.enable(step)
    for sensor, _ in lidars:
        sensor.enablePointCloud()
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
        sectors = []
        for sensor, yaw in lidars:
            cloud = np.array([[p.x, p.y, p.z] for p in sensor.getPointCloud()
                              if math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)])
            if len(cloud):
                c, s = math.cos(yaw), math.sin(yaw)
                cloud[:, :2] = cloud[:, :2] @ np.array([[c,s],[-s,c]])
                sectors.append(cloud)
        cloud = np.vstack(sectors) if sectors else np.empty((0,3))
        local = cloud[:, :2] if len(cloud) else np.empty((0, 2))
        nearest = cloud[int(np.argmin(np.linalg.norm(local, axis=1)))].tolist() if len(cloud) else None
        if len(local):
            cs, sn = math.cos(pose[2]), math.sin(pose[2])
            points = local @ np.array([[cs, sn], [-sn, cs]])+pose[:2]
        else:
            points = np.empty((0, 2))
        v, w = nav.step(pose, velocity, points, t, dt)
        left.setVelocity((v-.15*w)/.05); right.setVelocity((v+.15*w)/.05)
        rows.append({"t": t, "x": float(pose[0]), "y": float(pose[1]), "theta": float(pose[2]),
                     "v": v, "omega": w, "measured_v": velocity[0], "measured_omega": velocity[1],
                     "roll": rpy[0], "pitch": rpy[1], "nearest_lidar_local": nearest,
                     "observed_dynamic_cells": nav.observation.snapshot()})
        if nav.planning_failed or truth_path.exists() or t >= scene.duration or np.linalg.norm(pose[:2]-scene.goal) <= .25:
            break
    left.setVelocity(0.); right.setVelocity(0.)
    finish_time = rows[-1]["t"] if rows else 0.
    if truth_path.is_file():
        truth = json.loads(truth_path.read_text(encoding="utf-8"))
        finish_time = min(finish_time, truth["finish_time"])
    result = {**nav.diagnostics(finish_time), **control_metrics(rows, dt),
              **(path_metrics(nav.initial_path) if method not in ("apf", "dwa") else {"planned_peak_curvature": None}),
              "backend": "webots", "scene": scene.name, "seed": scene.seed,
              "scene_sha256": scene.signature(), "algorithm": method,
              "planning_failed": nav.planning_failed, "planning_time_ms": nav.planning_time_ms,
              "control_step_s": dt, "config": asdict(nav.cfg),
              "initial_path": nav.initial_path.tolist(),
              "planned_path_length": (float(np.linalg.norm(np.diff(nav.initial_path, axis=0), axis=1).sum())
                                      if method not in ("apf", "dwa") else None),
              "trajectory": rows}
    temporary = out / f"{tag}_navigator.tmp"
    temporary.write_text(json.dumps(result, indent=2), encoding="utf-8")
    temporary.replace(out / f"{tag}_navigator.json")
    print(f"[Navigator] Saved {tag}", flush=True)


if __name__ == "__main__":
    main()
