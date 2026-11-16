"""Deterministic scenes: obstacles never observe the robot or planner identity."""
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "robot_navigator"))
from utils.grid_map import GridMap, StaticObstacle


@dataclass
class MovingDisc:
    name: str
    radius: float
    waypoints: list  # [time, x, y]; linear interpolation, endpoint dwell permitted

    def position(self, t):
        a = np.asarray(self.waypoints)
        return np.array([np.interp(t, a[:, 0], a[:, 1]),
                         np.interp(t, a[:, 0], a[:, 2])])


@dataclass
class Scene:
    name: str
    start: tuple = (-4.0, 0.0)
    goal: tuple = (4.0, 0.0)
    static: list = field(default_factory=list)
    moving: list = field(default_factory=list)
    duration: float = 65.0
    seed: int = 0

    def grid(self, extra_points=None):
        obs = list(self.static)
        if extra_points is not None:
            obs += [StaticObstacle("cylinder", {"center": tuple(p), "radius": 0.07})
                    for p in extra_points]
        return GridMap(resolution=0.10, robot_radius=0.18,
                       safety_margin=0.14, obstacles=obs)

    def signature(self):
        data = {"name": self.name, "seed": self.seed, "start": self.start,
                "goal": self.goal, "static": [(o.kind, o.params) for o in self.static],
                "moving": [vars(o) for o in self.moving], "duration": self.duration}
        return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def box(x, y, w, h):
    return StaticObstacle("box", {"center": (x, y), "size": (w, h)})


def make_scene(name="blocked_passage", seed=0):
    # The same seed is supplied to every method; phase/start time are fixed upfront.
    rng = np.random.default_rng(seed)
    shift = float(rng.uniform(-1.0, 1.0)) if seed else 0.0
    walls = [box(0, 5, 10.2, .2), box(0, -5, 10.2, .2),
             box(5, 0, .2, 10.2), box(-5, 0, .2, 10.2)]
    scene = Scene(name=name, static=walls, seed=seed)
    if name == "static_detour":
        scene.static += [box(0, -.6, 1.2, 3.0), box(2.1, 1.4, .6, 1.4)]
    elif name in ("blocked_passage", "brief_crossing", "moving_crossing", "open_control"):
        # Central wall has a short passage (y=0) and a longer upper bypass (y>2.2).
        scene.static += [box(0, -2.8, .6, 4.2), box(0, 1.45, .6, 1.5)]
        if name == "blocked_passage":
            # A cart enters and dwells at the passage, then exits along free space.
            scene.moving = [MovingDisc("cart", .48,
                [[0, 1.4, -2], [2+shift, 1.4, -2], [4+shift, 1.4, 0],
                 [8+shift, 0, 0], [38+shift, 0, 0], [42+shift, 1.4, 0],
                 [46+shift, 1.4, -2], [65, 1.4, -2]])]
        elif name == "brief_crossing":
            scene.moving = [MovingDisc("cart", .38,
                [[0, 1.2, -2], [8+shift, 1.2, -2], [16+shift, 1.2, 2],
                 [65, 1.2, 2]])]
        elif name == "moving_crossing":
            scene.moving = [MovingDisc("cart", .38,
                [[0, -1.2, -2.5], [4+shift, -1.2, -2.5], [10+shift, -1.2, 2.5],
                 [16+shift, -1.2, -2.5], [22+shift, -1.2, 2.5], [65, -1.2, 2.5]])]
    else:
        raise ValueError(f"Unknown scene: {name}")
    return scene


SCENES = ("open_control", "static_detour", "brief_crossing", "moving_crossing", "blocked_passage")


def static_distance(points, obstacles):
    """Exact signed point-to-boundary distance; boxes use rounded Euclidean corners."""
    p = np.asarray(points, dtype=float)
    out = np.full(p.shape[:-1], np.inf)
    for obs in obstacles:
        delta = p - obs.params["center"]
        if obs.kind == "box":
            q = np.abs(delta) - np.asarray(obs.params["size"]) / 2
            d = np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(np.max(q, axis=-1), 0)
        else:
            d = np.linalg.norm(delta, axis=-1) - obs.params["radius"]
        out = np.minimum(out, d)
    return out


def clearance(scene, position, t, robot_radius=.18):
    gap = float(static_distance(np.asarray(position), scene.static)) - robot_radius
    for obs in scene.moving:
        gap = min(gap, float(np.linalg.norm(position-obs.position(t))) - obs.radius - robot_radius)
    return gap


def validate_scene(scene):
    for t in np.arange(0, scene.duration+.01, .05):
        for obs in scene.moving:
            if float(static_distance(obs.position(t), scene.static)) <= obs.radius+.02:
                raise ValueError(f"{scene.name}: {obs.name} intersects static geometry at t={t:.2f}")
    for p in (scene.start, scene.goal):
        if clearance(scene, np.array(p), 0) <= 0:
            raise ValueError("Start/goal in collision")
