"""A* + shared DWA, with an optional sensor-triggered global replanning policy.

The controller receives only pose, measured velocity and obstacle surface points.
It cannot read moving-obstacle schedules, true positions, radii or future states.
"""
from collections import deque
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np

from .scenarios import static_distance


def angle(a):
    return (a+math.pi) % (2*math.pi)-math.pi


def astar(grid, start, goal):
    source, target = grid.world_to_grid(*start), grid.world_to_grid(*goal)
    if grid.is_occupied(*target):
        return None
    if grid.is_occupied(*source):
        # A safe stopped robot may already be inside a newly inflated obstacle band.
        # Find a short, geometrically checked exit; never erase occupied cells.
        escape_margin = grid.robot_radius+.01
        if not grid.is_collision_free_point(*start, margin=escape_margin):
            return None
        queue, parents = deque([source]), {source: None}
        while queue:
            node = queue.popleft()
            position = start if node==source else grid.grid_to_world(*node)
            if not grid.is_occupied(*node):
                tail = astar(grid, position, goal)
                if tail is not None:
                    prefix, cur = [], node
                    while cur is not None:
                        prefix.append(tuple(start) if cur==source else grid.grid_to_world(*cur))
                        cur = parents[cur]
                    prefix.reverse()
                    return np.vstack((np.asarray(prefix[:-1]), tail)) if len(prefix)>1 else tail
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = node[0]+dx, node[1]+dy
                if nxt in parents or not grid.is_in_bounds(*nxt):
                    continue
                dest = grid.grid_to_world(*nxt)
                if np.linalg.norm(np.asarray(dest)-start)>.8:
                    continue
                if grid.is_line_collision_free(position, dest, step=.02, margin=escape_margin):
                    parents[nxt] = node; queue.append(nxt)
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
    horizon: float = 1.5
    prediction_dt: float = .15
    blocked_seconds: float = 2.5
    replan_cooldown: float = 4.0
    memory_seconds: float = 4.0


class Navigator:
    def __init__(self, scene, method="replan_dwa", config=None):
        if method not in ("fixed_dwa", "replan_dwa", "periodic_dwa"):
            raise ValueError(method)
        self.scene, self.method = scene, method
        self.cfg = config or Config()
        self.grid = scene.grid()
        self.path = astar(self.grid, scene.start, scene.goal)
        if self.path is None:
            raise ValueError("No static path")
        self.initial_path = self.path.copy()
        self.path_history = []
        self.positions = deque()
        self.memory = {}
        self.last_replan = -math.inf
        self.replans = 0
        self.replan_attempts = 0
        self.replan_ms = 0.
        self.steps_ms = []

    def target(self, position):
        # Densify by arc length, then pick a reachable forward reference point.
        dense = []
        for a, b in zip(self.path[:-1], self.path[1:]):
            n = max(1, int(np.linalg.norm(b-a)/.08))
            dense.extend(a+(b-a)*u for u in np.linspace(0, 1, n, endpoint=False))
        dense.append(self.path[-1])
        dense = np.asarray(dense)
        i = int(np.argmin(np.linalg.norm(dense-position, axis=1)))
        j = min(i+8, len(dense)-1)
        # Prevent lookahead from jumping through a static corner.
        while j > i+1 and not self.grid.is_line_collision_free(position, dense[j], step=.05, margin=self.cfg.radius+.04):
            j -= 1
        return dense[j]

    def update_map(self, points, t):
        if len(points):
            unknown = points[static_distance(points, self.scene.static) > .12]
            for p in unknown:
                key = tuple(np.round(p/.15).astype(int))
                self.memory[key] = (p, t)
        self.memory = {key: val for key, val in self.memory.items()
                       if t-val[1] <= self.cfg.memory_seconds}

    def step(self, pose, velocity, points, t, dt):
        tick = time.perf_counter()
        p = np.asarray(pose[:2])
        self.update_map(points, t)
        self.positions.append((t, p.copy()))
        while self.positions and t-self.positions[0][0] > self.cfg.blocked_seconds+.05:
            self.positions.popleft()
        stalled = (len(self.positions)>1 and t-self.positions[0][0] >= self.cfg.blocked_seconds-.05
                   and np.linalg.norm(p-self.positions[0][1]) < .18)
        request = ((self.method == "replan_dwa" and stalled) or
                   (self.method == "periodic_dwa" and t >= self.cfg.replan_cooldown))
        if request and t-self.last_replan >= self.cfg.replan_cooldown and self.memory:
            self.last_replan = t
            self.replan_attempts += 1
            begin = time.perf_counter()
            grid = self.scene.grid([v[0] for v in self.memory.values()])
            new_path = astar(grid, p, self.scene.goal)
            self.replan_ms += (time.perf_counter()-begin)*1000
            if new_path is not None:
                self.path = new_path
                self.replans += 1
                self.path_history.append({"t": t, "path": new_path.tolist()})
                self.positions.clear()
        cmd = self.dwa(pose, velocity, self.target(p), points, dt)
        self.steps_ms.append((time.perf_counter()-tick)*1000)
        return cmd

    def dwa(self, pose, velocity, target, points, dt):
        c = self.cfg
        x, y, theta = pose
        v0, w0 = velocity
        err = angle(math.atan2(target[1]-y, target[0]-x)-theta)
        lo, hi = max(0, v0-c.acceleration*dt), min(c.max_speed, v0+c.acceleration*dt)
        wl, wh = max(-c.max_omega, w0-c.angular_acceleration*dt), min(c.max_omega, w0+c.angular_acceleration*dt)
        vv, ww = np.meshgrid(np.linspace(lo, hi, 7), np.linspace(wl, wh, 19))
        v, w = vv.ravel(), ww.ravel()
        # The same wheel motor limits used by Webots are enforced before rollout.
        wheel = np.maximum(np.abs(v-.15*w), np.abs(v+.15*w))
        scale = np.minimum(1, .5/np.maximum(wheel, 1e-9))
        v, w = v*scale, w*scale
        times = np.arange(c.prediction_dt, c.horizon+.001, c.prediction_dt)
        headings = theta+w[:, None]*(times[None, :]-.5*c.prediction_dt)
        xs = x+np.cumsum(v[:, None]*np.cos(headings)*c.prediction_dt, axis=1)
        ys = y+np.cumsum(v[:, None]*np.sin(headings)*c.prediction_dt, axis=1)
        min_dist = np.full(len(v), 5.)
        if len(points):
            local = points[np.linalg.norm(points-[x, y], axis=1) < 2.5]
            if len(local):
                dist = (xs[:, :, None]-local[:, 0])**2+(ys[:, :, None]-local[:, 1])**2
                min_dist = np.sqrt(dist.min(axis=(1, 2)))
        valid = min_dist >= c.radius+c.margin
        # A conservative braking-distance filter is shared by all variants.
        valid &= min_dist-c.radius >= v*v/(2*c.acceleration)+.03
        last_heading = theta+w*times[-1]
        desired = np.arctan2(target[1]-ys[:, -1], target[0]-xs[:, -1])
        heading_err = np.abs((desired-last_heading+np.pi) % (2*np.pi)-np.pi)
        distance = np.hypot(target[0]-xs[:, -1], target[1]-ys[:, -1])
        score = -1.7*distance-.6*heading_err+.5*v+.15*np.minimum(min_dist, 1.)-.04*np.abs(w-w0)
        # Favor turning before translating when the new route lies behind the robot.
        if abs(err)>1.0:
            score -= 3*v
        score[~valid] = -np.inf
        if not np.any(valid):
            return max(0., v0-c.acceleration*dt), 0.
        k = int(np.argmax(score))
        return float(v[k]), float(w[k])
