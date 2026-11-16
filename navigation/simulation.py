"""Deterministic 2D unicycle simulation; explicitly not Webots physics results."""
from dataclasses import asdict
import math
import time

import numpy as np

from .core import Navigator, angle
from .scenarios import clearance, validate_scene


def scan(scene, pose, t, beams=180, max_range=5.):
    """Analytic first-hit lidar with occlusion, full 360 degrees, no near-point deletion."""
    origin = np.asarray(pose[:2])
    theta = pose[2]+np.linspace(-np.pi, np.pi, beams, endpoint=False)
    directions = np.column_stack((np.cos(theta), np.sin(theta)))
    ranges = np.full(beams, max_range)
    objects = [(o.kind, o.params) for o in scene.static]
    objects += [("cylinder", {"center": o.position(t), "radius": o.radius}) for o in scene.moving]
    for kind, p in objects:
        delta = origin-np.asarray(p["center"])
        if kind == "cylinder":
            b = directions@delta
            discriminant = b*b-np.dot(delta, delta)+p["radius"]**2
            d = -b-np.sqrt(np.maximum(discriminant, 0))
            ranges = np.minimum(ranges, np.where((discriminant>=0)&(d>=.02), d, max_range))
        else:
            half = np.asarray(p["size"])/2
            safe = np.where(np.abs(directions)>1e-12, directions, 1e-12)
            aa, bb = (-half-delta)/safe, (half-delta)/safe
            near, far = np.minimum(aa, bb).max(axis=1), np.maximum(aa, bb).min(axis=1)
            ranges = np.minimum(ranges, np.where((far>=near)&(near>=.02), near, max_range))
    hit = ranges < max_range
    return origin+directions[hit]*ranges[hit, None]


def run(scene, method="replan_dwa", dt=.10):
    validate_scene(scene)
    nav = Navigator(scene, method)
    pose = np.array([*scene.start, math.atan2(scene.goal[1]-scene.start[1], scene.goal[0]-scene.start[0])])
    velocity = (0., 0.)
    rows, distance, min_gap, collision = [], 0., math.inf, False
    started = time.perf_counter()
    status = "timeout"
    for step in range(int(scene.duration/dt)):
        t = step*dt
        if nav.planning_failed:
            status = "planning_failed"
            rows.append({"t": 0., "x": float(pose[0]), "y": float(pose[1]), "theta": float(pose[2]),
                         "v": 0., "omega": 0., "clearance": clearance(scene, pose[:2], 0),
                         "tracked_obstacles": [], "obstacles": [[*o.position(0), o.radius] for o in scene.moving]})
            min_gap = rows[-1]["clearance"]
            break
        points = scan(scene, pose, t)
        v, w = nav.step(pose, velocity, points, t, dt)
        old = pose.copy()
        # Swept checks at 10ms intervals include simultaneous obstacle motion.
        substeps = max(1, int(math.ceil(dt/.01)))
        for j in range(substeps):
            small = dt/substeps
            pose[:2] += small*v*np.array([math.cos(pose[2]+w*small/2), math.sin(pose[2]+w*small/2)])
            pose[2] = angle(pose[2]+w*small)
            gap = clearance(scene, pose[:2], t+(j+1)*small)
            min_gap = min(min_gap, gap)
            if gap <= 0:
                collision = True
                break
        distance += float(np.linalg.norm(pose[:2]-old[:2]))
        velocity = v, w
        rows.append({"t": t+dt, "x": float(pose[0]), "y": float(pose[1]), "theta": float(pose[2]),
                     "v": v, "omega": w, "clearance": gap,
                     "tracked_obstacles": nav.tracker.snapshot(),
                     "obstacles": [[*o.position(t+dt), o.radius] for o in scene.moving]})
        if collision:
            status = "collision"; break
        if np.linalg.norm(pose[:2]-scene.goal) < .25:
            status = "success"; break
    return {"backend": "kinematic_2d", "scene": scene.name, "seed": scene.seed,
            "scene_sha256": scene.signature(), "algorithm": method, "config": asdict(nav.cfg),
            "success": status=="success", "status": status, "finish_time": rows[-1]["t"],
            "actual_path_length": distance, "min_surface_clearance": min_gap,
            "collision_count": int(collision), "replans": nav.replans,
            "planning_time_ms": nav.planning_time_ms,
            "replan_attempts": nav.replan_attempts, "replan_time_ms": nav.replan_ms,
            "control_mean_ms": float(np.mean(nav.steps_ms)) if nav.steps_ms else 0.,
            "control_p95_ms": float(np.percentile(nav.steps_ms, 95)) if nav.steps_ms else 0.,
            "wall_time_s": time.perf_counter()-started,
            "initial_path": nav.initial_path.tolist(), "path_history": nav.path_history,
            "trajectory": rows}
