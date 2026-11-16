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
from .tracking import ScanTracker
from .classical import rrt_star, voronoi_path, apf_command
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
    horizon: float = 1.5
    prediction_dt: float = .15


class Navigator:
    def __init__(self, scene, method=DEFAULT_METHOD, config=None):
        if method not in METHODS:
            raise ValueError(method)
        self.scene, self.method = scene, method
        self.cfg = config or Config()
        self.grid = scene.grid()
        begin = time.perf_counter()
        if method == "rrt_star":
            self.path = rrt_star(self.grid, scene.start, scene.goal, seed=scene.seed)
        elif method == "voronoi":
            self.path = voronoi_path(self.grid, scene.start, scene.goal)
        elif method == "voronoi_qp":
            self.path = voronoi_qp_path(self.grid, scene.start, scene.goal)
        elif method in ("apf", "dwa"):
            self.path = np.asarray([scene.start, scene.goal])
        else:
            self.path = astar(self.grid, scene.start, scene.goal)
        self.planning_time_ms = (time.perf_counter()-begin)*1000
        self.planning_failed = self.path is None
        if self.planning_failed:
            self.path = np.asarray([scene.start])
        self.initial_path = self.path.copy()
        self.steps_ms = []
        self.tracker = ScanTracker()

    def target(self, position):
        # Densify by arc length, then pick a reachable forward reference point.
        dense = resample_path(self.path, .08)
        i = int(np.argmin(np.linalg.norm(dense-position, axis=1)))
        j = min(i+8, len(dense)-1)
        # Prevent lookahead from jumping through a static corner.
        while j > i+1 and not self.grid.is_line_collision_free(position, dense[j], step=.05):
            j -= 1
        return dense[j]

    def step(self, pose, velocity, points, t, dt):
        tick = time.perf_counter()
        if self.planning_failed:
            return 0., 0.
        if self.method in ("astar", "rrt_star", "voronoi", "voronoi_qp"):
            cmd = self.follow_path(pose, velocity, points, dt)
            self.steps_ms.append((time.perf_counter()-tick)*1000)
            return cmd
        if self.method == "apf":
            cmd = self.limit_command(apf_command(pose, velocity, self.scene.goal, points, dt, self.cfg), velocity, dt)
            self.steps_ms.append((time.perf_counter()-tick)*1000)
            return cmd
        velocities = self.tracker.update(points, self.scene.static, t)
        cmd = self.dwa(pose, velocity, self.scene.goal, points, dt, velocities)
        self.steps_ms.append((time.perf_counter()-tick)*1000)
        return cmd

    def limit_command(self, command, velocity, dt):
        c = self.cfg
        v = float(np.clip(command[0], max(0., velocity[0]-c.acceleration*dt),
                          min(c.max_speed, velocity[0]+c.acceleration*dt)))
        w = float(np.clip(command[1], max(-c.max_omega, velocity[1]-c.angular_acceleration*dt),
                          min(c.max_omega, velocity[1]+c.angular_acceleration*dt)))
        wheel = max(abs(v-.15*w), abs(v+.15*w))
        scale = min(1., .5/max(wheel, 1e-9))
        return v*scale, w*scale

    def follow_path(self, pose, velocity, points, dt):
        """Common execution of a global path: heading control and emergency braking.

        This follower never samples alternative motions or plans another route.
        """
        c = self.cfg
        target = self.target(np.asarray(pose[:2]))
        delta = target-np.asarray(pose[:2])
        error = angle(math.atan2(delta[1], delta[0])-pose[2])
        remaining = np.linalg.norm(np.asarray(self.scene.goal)-pose[:2])
        desired = c.max_speed*max(0., math.cos(error))*min(1., remaining/.8)
        v, w = self.limit_command((desired, 2*error), velocity, dt)
        # Stop along the commanded arc if the braking envelope meets a scan hit.
        if len(points) and v > 0:
            travel_time = dt+v/c.acceleration
            times = np.linspace(0, travel_time, 12)
            headings = pose[2]+w*times/2
            positions = np.asarray(pose[:2])+v*times[:, None]*np.column_stack((np.cos(headings), np.sin(headings)))
            distance = np.linalg.norm(positions[:, None, :]-points[None, :, :], axis=-1).min()
            if distance < c.radius+c.margin:
                v, _ = self.limit_command((0., w), velocity, dt)
        return v, w

    def dwa(self, pose, velocity, target, points, dt, point_velocities=None):
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
            nearby = np.linalg.norm(points-[x, y], axis=1) < 2.5
            local = points[nearby]
            if len(local):
                dist = (xs[:, :, None]-local[:, 0])**2+(ys[:, :, None]-local[:, 1])**2
                if point_velocities is not None:
                    pv = point_velocities[nearby]
                    predicted = local[None, :, :]+times[:, None, None]*pv[None, :, :]
                    future_dist = ((xs[:, :, None]-predicted[None, :, :, 0])**2+
                                   (ys[:, :, None]-predicted[None, :, :, 1])**2)
                    dist = np.minimum(dist, future_dist)
                min_dist = np.sqrt(dist.min(axis=(1, 2)))
        valid = min_dist >= c.radius+c.margin
        # A conservative braking-distance filter is shared by all variants.
        valid &= min_dist-c.radius >= v*v/(2*c.acceleration)+.03
        # A circular body can rotate without translating even inside the comfort
        # margin. Keep the physical clearance requirement; translation still has
        # to satisfy the full margin. Otherwise a stopped robot cannot turn away.
        valid |= (v < 1e-9) & (min_dist >= c.radius+.03)
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
