"""Classical baselines and safe-bubble QP smoothing of a Voronoi reference.

The controller receives only pose, measured velocity and obstacle surface points.
It does not query moving-obstacle schedules, true positions, radii or future states.
"""
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np

from .settings import DEFAULT_METHOD, METHODS
from .observation_map import ObservationMap
from .local import apf_command, dwa_command
from .classical import rrt_star, voronoi_path
from .smoothing import voronoi_qp_path
from .metrics import resample_path


def angle(a):
    return (a+math.pi) % (2*math.pi)-math.pi


def astar(grid, start, goal):
    source, target = grid.world_to_grid(*start), grid.world_to_grid(*goal)
    if grid.is_occupied(*source) or grid.is_occupied(*target):
        return None
    queue = [(0., source)]
    cost, parent, closed = {source: 0.}, {}, set()
    while queue:
        _, p = heapq.heappop(queue)
        if p in closed:
            continue
        if p == target:
            path = [p]
            while p != source:
                p = parent[p]
                path.append(p)
            path.reverse()
            points = [tuple(start)]+[grid.grid_to_world(*q) for q in path[1:-1]]+[tuple(goal)]
            # Shared safe shortcut; uses the same inflation as search.
            out, i = [points[0]], 0
            while i < len(points)-1:
                j = len(points)-1
                while j > i+1 and not grid.is_line_collision_free(points[i], points[j], step=.04):
                    j -= 1
                out.append(points[j]); i = j
            return np.asarray(out)
        closed.add(p)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            q = p[0]+dx, p[1]+dy
            if grid.is_occupied(*q):
                continue
            if dx and dy and (grid.is_occupied(p[0]+dx, p[1]) or grid.is_occupied(p[0], p[1]+dy)):
                continue
            new = cost[p]+math.hypot(dx, dy)*grid.resolution
            if new < cost.get(q, math.inf):
                cost[q], parent[q] = new, p
                h = math.hypot(q[0]-target[0], q[1]-target[1])*grid.resolution
                heapq.heappush(queue, (new+h, q))
    return None


@dataclass(frozen=True)
class Config:
    max_speed: float = .45
    max_omega: float = 1.5
    acceleration: float = .8
    angular_acceleration: float = 3.
    radius: float = .18
    margin: float = .10
    horizon: float = 1.7
    replan_period: float = 1.0
    observation_retention: float = 1.0


class Navigator:
    def __init__(self, scene, method=DEFAULT_METHOD, config=None):
        if method not in METHODS:
            raise ValueError(method)
        self.scene, self.method = scene, method
        self.cfg = config or Config()
        self.observation = ObservationMap(scene, self.cfg.observation_retention)
        self.grid = self.observation.grid
        self.role = "local_controller" if method in ("apf", "dwa") else "global_planner"
        self.guide_planner = "astar" if self.role == "local_controller" else method
        self.plan_attempts = []
        self.route_updates = []
        self.steps_ms = []
        self.next_route_check = 0.
        begin = time.perf_counter()
        self.path = self.plan(scene.start)
        self.planning_time_ms = (time.perf_counter()-begin)*1000
        self.planning_failed = self.path is None
        if self.planning_failed:
            self.path = np.asarray([scene.start])
        self.initial_path = self.path.copy()
        self.route_updates.append({"t": 0., "reason": "initial", "path": self.path.tolist()})

    def plan(self, start):
        start = np.asarray(start)
        connector = None
        if not self.grid.is_collision_free_point(*start):
            # Inflation is a planning comfort region, not a physical obstacle.
            # Find a checked escape connection without clearing any occupied cell.
            ys, xs = np.nonzero(self.grid.inflated_grid == 0)
            candidates = np.column_stack((self.grid.min_x+(xs+.5)*self.grid.resolution,
                                          self.grid.min_y+(ys+.5)*self.grid.resolution))
            distance = np.linalg.norm(candidates-start, axis=1)
            for i in np.argsort(distance):
                if distance[i] > .8:
                    break
                if self.grid.is_line_collision_free(start, candidates[i], margin=self.cfg.radius+.03):
                    connector, start = start, candidates[i]
                    break
            if connector is None:
                return None
        if self.guide_planner == "rrt_star":
            path = rrt_star(self.grid, start, self.scene.goal,
                            seed=self.scene.seed+1009*len(self.plan_attempts))
        elif self.guide_planner == "voronoi":
            path = voronoi_path(self.grid, start, self.scene.goal)
        elif self.guide_planner == "voronoi_qp":
            path = voronoi_qp_path(self.grid, start, self.scene.goal)
        else:
            path = astar(self.grid, start, self.scene.goal)
        return np.vstack((connector, path)) if connector is not None and path is not None else path

    def remaining_path(self, position):
        dense = resample_path(self.path, .08)
        i = int(np.argmin(np.linalg.norm(dense-position, axis=1)))
        return dense[i:]

    def target(self, position):
        dense = self.remaining_path(position)
        j = min(8, len(dense)-1)
        margin = None if self.grid.is_collision_free_point(*position) else self.cfg.radius+.03
        while j > 1 and not self.grid.is_line_collision_free(position, dense[j], margin=margin):
            j -= 1
        return dense[j]

    def update_route(self, pose, points, t):
        self.observation.update(pose[:2], points, t)
        if t+1e-9 < self.next_route_check:
            return
        if not self.observation.blocks(self.remaining_path(pose[:2])):
            return
        self.next_route_check = t+self.cfg.replan_period
        begin = time.perf_counter()
        candidate = self.plan(pose[:2])
        elapsed = (time.perf_counter()-begin)*1000
        self.plan_attempts.append({"t": float(t), "success": candidate is not None,
                                   "time_ms": elapsed, "reason": "observed_route_blockage"})
        if candidate is not None:
            self.path = candidate
            self.route_updates.append({"t": float(t), "reason": "observed_route_blockage",
                                       "path": candidate.tolist()})

    def step(self, pose, velocity, points, t, dt):
        tick = time.perf_counter()
        if self.planning_failed:
            return 0., 0.
        self.update_route(pose, points, t)
        target = self.target(np.asarray(pose[:2]))
        if self.method == "apf":
            raw = apf_command(pose, velocity, target, points, dt, self.cfg)
        elif self.method == "dwa":
            raw = self.dwa(pose, velocity, target, points, dt)
        else:
            raw = self.follow_path(pose, target)
        command = self.brake(pose, velocity, points, self.limit_command(raw, velocity, dt), dt)
        self.steps_ms.append((time.perf_counter()-tick)*1000)
        return command

    def limit_command(self, command, velocity, dt):
        c = self.cfg
        v = float(np.clip(command[0], max(0., velocity[0]-c.acceleration*dt),
                          min(c.max_speed, velocity[0]+c.acceleration*dt)))
        w = float(np.clip(command[1], max(-c.max_omega, velocity[1]-c.angular_acceleration*dt),
                          min(c.max_omega, velocity[1]+c.angular_acceleration*dt)))
        wheel = max(abs(v-.15*w), abs(v+.15*w))
        scale = min(1., .5/max(wheel, 1e-9))
        return v*scale, w*scale

    def follow_path(self, pose, target):
        delta = target-np.asarray(pose[:2])
        error = angle(math.atan2(delta[1], delta[0])-pose[2])
        remaining = np.linalg.norm(np.asarray(self.scene.goal)-pose[:2])
        return self.cfg.max_speed*max(0., math.cos(error))*min(1., remaining/.8), 2*error

    def brake(self, pose, velocity, points, command, dt):
        c = self.cfg
        v, w = command
        if len(points) and v > 0:
            times = np.linspace(0, dt+v/c.acceleration, 12)
            headings = pose[2]+w*times/2
            positions = np.asarray(pose[:2])+v*times[:, None]*np.column_stack((np.cos(headings), np.sin(headings)))
            clearance = np.linalg.norm(positions[:, None, :]-points[None, :, :], axis=-1).min(axis=1)
            escaping = (clearance[0] < c.radius+c.margin
                        and clearance.min() >= c.radius+.03
                        and clearance.min() >= clearance[0]-.002
                        and clearance[-1] > clearance[0]+.001)
            if clearance.min() < c.radius+c.margin and not escaping:
                v, _ = self.limit_command((0., w), velocity, dt)
        return v, w

    def dwa(self, pose, velocity, target, points, dt):
        return dwa_command(np.asarray(pose), velocity, target, points, dt, self.cfg)

    def diagnostics(self):
        return {"role": self.role, "guide_planner": self.guide_planner,
                "replan_count": len(self.route_updates)-1,
                "replan_attempt_count": len(self.plan_attempts),
                "replan_attempts": self.plan_attempts,
                "replan_time_ms": sum(p["time_ms"] for p in self.plan_attempts),
                "route_updates": self.route_updates}
