"""Deterministic scenes: obstacles never observe the robot or planner identity."""
from dataclasses import dataclass, field
from itertools import groupby
from pathlib import Path
import hashlib
import json

import numpy as np

from .grid_map import GridMap, StaticObstacle


@dataclass
class MovingDisc:
    name: str
    radius: float
    waypoints: list  # [time, x, y]; linear interpolation, endpoint dwell permitted

    def position(self, t):
        a = np.asarray(self.waypoints)
        return np.stack([np.interp(t, a[:, 0], a[:, 1]),
                         np.interp(t, a[:, 0], a[:, 2])], axis=-1)


@dataclass
class Scene:
    name: str
    start: tuple = (-4.0, -3.5)
    goal: tuple = (4.0, 3.5)
    static: list = field(default_factory=list)
    moving: list = field(default_factory=list)
    duration: float = 100.0
    seed: int = 0
    bounds: tuple = (-5., 5., -5., 5.)

    def grid(self):
        return GridMap(self.static, *self.bounds, resolution=0.10,
                       robot_radius=0.18, safety_margin=0.14)

    def signature(self):
        data = {"name": self.name, "seed": self.seed, "start": self.start,
                "goal": self.goal, "static": [(o.kind, o.params) for o in self.static],
                "moving": [vars(o) for o in self.moving], "duration": self.duration,
                "bounds": self.bounds}
        return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def box(x, y, w, h):
    return StaticObstacle("box", {"center": (x, y), "size": (w, h)})


MAP_FILE = Path(__file__).resolve().parents[1] / "assets" / "maps" / "course_maps.json"
SCENE_INFO = json.loads(MAP_FILE.read_text(encoding="utf-8"))
SCENES = tuple(SCENE_INFO)
DEFAULT_SCENE = SCENES[0]


def maze_walls(rows, cell):
    height, width = len(rows), len(rows[0])
    left, top = -width * cell / 2, height * cell / 2
    obstacles = []
    for r, row in enumerate(rows):
        for wall, cells in groupby(enumerate(row), key=lambda entry: entry[1]):
            if wall == "1":
                columns = [c for c, _ in cells]
                x = left + (columns[0] + len(columns) / 2) * cell
                obstacles.append(box(x, top - (r + .5) * cell, len(columns) * cell, cell))
    return obstacles, (left, -left, -top, top)


def make_scene(name=DEFAULT_SCENE, seed=0):
    if name not in SCENES:
        raise ValueError(f"Unknown scene: {name}")
    spec = SCENE_INFO[name]
    if "grid" in spec:
        static, bounds = maze_walls(spec["grid"], spec["cell_size"])
    else:
        bounds = tuple(spec["bounds"])
        static = [box(*rect) for rect in spec["boxes"]]
        static.extend(StaticObstacle("cylinder", {"center": (x, y), "radius": radius})
                      for x, y, radius in spec["cylinders"])
    # Every method sees the same course and clock phase for a given seed.
    shift = float(np.random.default_rng(seed).uniform(-.6, .6)) if seed else 0.
    waypoints = [point.copy() for point in spec["cart"]["waypoints"]]
    for point in waypoints[1:-1]:
        point[0] += shift
    cart = MovingDisc("crossing_cart", spec["cart"]["radius"], waypoints)
    return Scene(name, start=tuple(spec["start"]), goal=tuple(spec["goal"]),
                 static=static, moving=[cart], seed=seed, bounds=bounds)


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
    times = np.arange(0, scene.duration+.01, .05)
    for i, obs in enumerate(scene.moving):
        positions = obs.position(times)
        hit = static_distance(positions, scene.static) <= obs.radius+.02
        if hit.any():
            raise ValueError(f"{scene.name}: {obs.name} intersects static geometry at t={times[hit][0]:.2f}")
        for other in scene.moving[i+1:]:
            hit = np.linalg.norm(positions-other.position(times), axis=1) <= obs.radius+other.radius+.02
            if hit.any():
                raise ValueError(f"{scene.name}: actors intersect at t={times[hit][0]:.2f}")
    for p in (scene.start, scene.goal):
        if clearance(scene, np.array(p), 0) <= 0:
            raise ValueError("Start/goal in collision")
