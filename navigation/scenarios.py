"""Deterministic scenes: obstacles never observe the robot or planner identity."""
from dataclasses import dataclass, field
import hashlib
import json
import math

import numpy as np

from .grid_map import GridMap, StaticObstacle


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
    start: tuple = (-4.0, -3.5)
    goal: tuple = (4.0, 3.5)
    static: list = field(default_factory=list)
    moving: list = field(default_factory=list)
    duration: float = 100.0
    seed: int = 0

    def grid(self):
        return GridMap(resolution=0.10, robot_radius=0.18,
                       safety_margin=0.14, obstacles=self.static)

    def signature(self):
        data = {"name": self.name, "seed": self.seed, "start": self.start,
                "goal": self.goal, "static": [(o.kind, o.params) for o in self.static],
                "moving": [vars(o) for o in self.moving], "duration": self.duration}
        return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def box(x, y, w, h):
    return StaticObstacle("box", {"center": (x, y), "size": (w, h)})


DEFAULT_SCENE = "warehouse"
SCENES = (DEFAULT_SCENE,)


def make_scene(name=DEFAULT_SCENE, seed=0):
    # The same seed is supplied to every method; phase/start time are fixed upfront.
    rng = np.random.default_rng(seed)
    shift = float(rng.uniform(-1.0, 1.0)) if seed else 0.0
    walls = [box(0, 5, 10.2, .2), box(0, -5, 10.2, .2),
             box(5, 0, .2, 10.2), box(-5, 0, .2, 10.2)]
    if name not in SCENES:
        raise ValueError(f"Unknown scene: {name}")
    scene = Scene(name=name, static=walls, seed=seed)
    # Seven shelves: a central passage, an upper bypass, and offset approach/exit
    # aisles. These force turns before AND after the temporary blockage.
    scene.static += [box(0, -2.8, .6, 4.2), box(0, 1.45, .6, 1.5),
                     box(-2.65, -1.9, 2.5, .65), box(-3.1, 1.3, 1.6, 1.8),
                     box(-1.8, 3.65, 2., .6), box(2.2, 1.7, 2., .7),
                     box(3.3, -1.8, 1.6, 2.)]
    scene.moving = [
        MovingDisc("loading_cart", .48,
            [[0, 1.4, -2], [2+shift, 1.4, -2], [8+shift, 1.4, 0],
             [12+shift, 0, 0], [58+shift, 0, 0], [62+shift, 1.4, 0],
             [68+shift, 1.4, -2], [100, 1.4, -2]]),
        MovingDisc("aisle_cart", .30,
            [[0, 3.9, 4.1], [20+shift, 3.9, 4.1], [28+shift, 3.9, 2.8],
             [44+shift, 1.4, 2.8], [60+shift, 3.9, 2.8],
             [70+shift, 3.9, 4.1], [100, 3.9, 4.1]]),
    ]
    return scene


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
    for obs in scene.moving:
        schedule = np.asarray(obs.waypoints, dtype=float)
        if (not np.isfinite(schedule).all() or np.any(np.diff(schedule[:, 0]) <= 0)
                or schedule[0, 0] != 0 or schedule[-1, 0] < scene.duration):
            raise ValueError(f"Invalid schedule for {obs.name}")
    for t in np.arange(0, scene.duration+.01, .05):
        for i, obs in enumerate(scene.moving):
            if float(static_distance(obs.position(t), scene.static)) <= obs.radius+.02:
                raise ValueError(f"{scene.name}: {obs.name} intersects static geometry at t={t:.2f}")
            for other in scene.moving[i+1:]:
                if np.linalg.norm(obs.position(t)-other.position(t)) <= obs.radius+other.radius+.02:
                    raise ValueError(f"{scene.name}: actors intersect at t={t:.2f}")
    for p in (scene.start, scene.goal):
        if clearance(scene, np.array(p), 0) <= 0:
            raise ValueError("Start/goal in collision")
