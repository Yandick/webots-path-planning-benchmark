"""Global and local navigation baselines, with optional yielding at crossings.

The controller receives only pose, measured velocity and obstacle surface points.
It does not query moving-obstacle schedules, true positions, radii or future states.
"""
from dataclasses import dataclass, replace
import heapq
import math
import time

import numpy as np

from .settings import (DEFAULT_METHOD, EXTENSION_METHODS, GUARD_FACTORIES, GUARD_VARIANTS,
                       METHODS, TIMED_PLANNERS, YIELD_BASES)
from .observation_map import ObservationMap
from .local import apf_command, dwa_command
from .classical import rrt_star, voronoi_path
from .smoothing import voronoi_qp_path
from .metrics import resample_path
from .yielding import CrossingGuard
from .timed import effective_speed, nominal_schedule, timed_astar, timed_conflict


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
        if method not in METHODS+EXTENSION_METHODS:
            raise ValueError(method)
        self.scene, self.method = replace(scene, moving=[]), method
        self.cfg = config or Config()
        self.observation = ObservationMap(self.scene, self.cfg.observation_retention)
        self.grid = self.observation.grid
        self.role = "local_controller" if method in ("apf", "dwa") else "global_planner"
        variant = GUARD_VARIANTS.get(method, {})
        self.timed_base = TIMED_PLANNERS.get(method)
        factory = GUARD_FACTORIES.get(method)
        base = YIELD_BASES.get(method, variant.get("base", self.timed_base or method))
        if factory:
            base = factory.get("base", base)
        self.guide_planner = "astar" if self.role == "local_controller" else base
        if factory:
            import importlib
            module, name = factory["factory"].split(":")
            self.crossing_guard = getattr(importlib.import_module(module), name)(self.cfg)
        elif method in YIELD_BASES or variant:
            self.crossing_guard = CrossingGuard(self.cfg, **variant.get("params", {}))
        elif self.timed_base:
            self.crossing_guard = CrossingGuard(self.cfg)
        else:
            self.crossing_guard = None
        self.plan_attempts = []
        self.route_updates = []
        self.steps_ms = []
        self.next_route_check = 0.
        self.schedule_times = None
        # When timed replans repeatedly fail to produce motion (e.g. the cart
        # resumes an unobservable schedule), fall back to pure reactive
        # yielding for a while, exactly like the base early-yield method.
        self.timed_reactive_until = 0.
        self.timed_failures = 0
        begin = time.perf_counter()
        self.path = self.plan(scene.start)
        self.planning_time_ms = (time.perf_counter()-begin)*1000
        self.planning_failed = self.path is None
        if self.planning_failed:
            self.path = np.asarray([scene.start])
        self.initial_path = self.path.copy()
        self.route_updates.append({"t": 0., "reason": "initial", "path": self.path.tolist()})

    def plan(self, start, t=0., timed=True):
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
        elif self.timed_base and timed:
            reference = astar(self.grid, start, self.scene.goal)
            if reference is None:
                return None
            length = float(np.linalg.norm(np.diff(reference, axis=0), axis=1).sum())
            horizon = min(90., max(25., 1.4*length/effective_speed(self.grid, self.cfg.max_speed)))
            schedule = timed_astar(self.grid, start, self.scene.goal, t,
                                   tracks=self.crossing_guard.tracks if self.crossing_guard else (),
                                   speed=self.cfg.max_speed, horizon=horizon,
                                   clearance_extra=self.cfg.margin+.06)
            if schedule is None:
                # No feasible schedule in range; the caller falls back to the
                # reactive proceed-vs-brake comparison until a replan succeeds.
                return None
            path, self.schedule_times = schedule
            if connector is not None:
                self.schedule_times = np.concatenate(([t], self.schedule_times))
        else:
            path = astar(self.grid, start, self.scene.goal)
            if self.timed_base and path is not None:
                # Reactive fallback: keep the spatial route on a nominal
                # timetable so schedule following keeps working.
                self.schedule_times = nominal_schedule(path, t, self.cfg.max_speed)
                if connector is not None:
                    self.schedule_times = np.concatenate(([t], self.schedule_times))
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

    def update_route(self, pose, points, t, speed=0., dt=.1):
        position = np.asarray(pose[:2])
        hits = self.observation.update(position, points, t)
        stalled = False
        if self.crossing_guard is not None:
            self.crossing_guard.observe(t, position, hits)
            if self.timed_base and t >= self.timed_reactive_until:
                return self._update_route_timed(pose, t, speed, dt)
            holding = self.crossing_guard.should_yield(t, dt, position, speed, self.remaining_path(position))
            if self.plan_attempts or self.crossing_guard.events:
                stalled = self.crossing_guard.stalled(t, position)
            if holding:
                return
        if t+1e-9 < self.next_route_check:
            return
        remaining = self.remaining_path(pose[:2])
        connection = resample_path([pose[:2], remaining[0]], .08)
        blocked = self.observation.blocks(np.vstack((connection, remaining)))
        if not blocked and not stalled:
            return
        reason = "observed_route_blockage" if blocked else "stalled_after_blockage"
        self.next_route_check = t+self.cfg.replan_period
        begin = time.perf_counter()
        candidate = self.plan(pose[:2], t, timed=self.timed_base is None or t >= self.timed_reactive_until)
        elapsed = (time.perf_counter()-begin)*1000
        self.plan_attempts.append({"t": float(t), "success": candidate is not None,
                                   "time_ms": elapsed, "reason": reason, "pos": [float(pose[0]), float(pose[1])]})
        if candidate is not None:
            self.path = candidate
            self.route_updates.append({"t": float(t), "reason": reason,
                                       "path": candidate.tolist()})

    def _update_route_timed(self, pose, t, speed, dt):
        """Replan the (space, time) schedule when predictions disagree with it.

        The schedule governs while it is consistent with the track predictions.
        When it is not, the reactive proceed-vs-brake comparison arbitrates the
        approach (and holds if necessary) while timed replanning is throttled.
        """
        self.crossing_guard.release_hold()
        remaining = self.remaining_path(pose[:2])
        demanded = True
        conflict = True
        if self.schedule_times is not None and len(self.path) > 1:
            dense = resample_path(self.path, .08)
            progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
            waypoint_progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))]
            i = int(np.argmin(np.linalg.norm(dense-np.asarray(pose[:2]), axis=1)))
            conflict = timed_conflict(self.path, self.schedule_times, t,
                                      self.crossing_guard.tracks, robot_radius=self.cfg.radius,
                                      clearance_extra=self.cfg.margin+.06)
            tq = min(t+.5, float(self.schedule_times[-1]))
            demanded = bool(np.interp(tq, self.schedule_times, waypoint_progress)-progress[i] > .1)
        # Personal space: a track resuming motion toward the parked robot must
        # trigger arbitration immediately, not wait for the stalled cycle.
        for track in self.crossing_guard.tracks:
            clearance = self.cfg.radius+track.radius+self.cfg.margin
            future = track.predict(t+1.)
            if np.linalg.norm(future-np.asarray(pose[:2])) < clearance+.06:
                conflict = True
                break
        stalled = False
        if self.plan_attempts or self.crossing_guard.events:
            stalled = self.crossing_guard.stalled(t, pose[:2])
        stalled = stalled and demanded
        if not stalled and (not conflict or not demanded):
            return
        if t+1e-9 < self.next_route_check:
            self.crossing_guard.should_yield(t, dt, pose[:2], speed, remaining)
            return
        reason = "predicted_track_conflict" if conflict else "stalled_after_blockage"
        # Three timed replans without physical progress means the prediction
        # model cannot explain the scene (e.g. the cart resumed its scripted
        # route). Defer to reactive yielding for a while, like the base method.
        moved = True
        if self.plan_attempts:
            moved = np.linalg.norm(np.asarray(pose[:2])
                                   -np.asarray(self.plan_attempts[-1].get("pos", pose[:2]))) > .1
        self.timed_failures = 0 if moved else self.timed_failures+1
        if self.timed_failures >= 3:
            self.timed_failures = 0
            self.timed_reactive_until = t+8.
            self.crossing_guard.should_yield(t, dt, pose[:2], speed, remaining)
            return
        self.next_route_check = t+self.cfg.replan_period
        begin = time.perf_counter()
        candidate = self.plan(pose[:2], t)
        elapsed = (time.perf_counter()-begin)*1000
        self.plan_attempts.append({"t": float(t), "success": candidate is not None,
                                   "time_ms": elapsed, "reason": reason,
                                   "pos": [float(pose[0]), float(pose[1])]})
        if candidate is not None:
            self.path = candidate
            self.crossing_guard.release_hold()
            self.route_updates.append({"t": float(t), "reason": reason,
                                       "path": candidate.tolist()})
        else:
            # No feasible schedule right now: defer to the reactive
            # proceed-vs-brake comparison until a replan succeeds.
            self.crossing_guard.should_yield(t, dt, pose[:2], speed, remaining)

    def timed_target(self, pose, t):
        """Schedule-interpolated lookahead point plus an explicit wait flag.

        Waiting is commanded explicitly when the robot is ahead of schedule;
        the target is never a point behind the robot (a unicycle would have to
        turn around to chase it).
        """
        if self.schedule_times is None or len(self.path) < 2:
            return self.target(np.asarray(pose[:2])), False
        dense = resample_path(self.path, .08)
        progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
        waypoint_progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))]
        i = int(np.argmin(np.linalg.norm(dense-np.asarray(pose[:2]), axis=1)))
        tq = min(t+1., float(self.schedule_times[-1]))
        target = np.array([np.interp(tq, self.schedule_times, self.path[:, k]) for k in range(2)])
        scheduled = np.interp(tq, self.schedule_times, waypoint_progress)
        return target, bool(progress[i] > scheduled+.05)

    def step(self, pose, velocity, points, t, dt):
        tick = time.perf_counter()
        if self.planning_failed:
            return 0., 0.
        self.update_route(pose, points, t, velocity[0], dt)
        if self.crossing_guard is not None and self.crossing_guard.holding:
            raw = (0., 0.)
        elif self.timed_base and t < self.timed_reactive_until:
            # Reactive fallback window: plain spatial following, exactly like
            # the base early-yield method (which manages holds via should_yield).
            raw = self.follow_path(pose, self.target(np.asarray(pose[:2])))
        elif self.timed_base:
            target, waiting = self.timed_target(pose, t)
            raw = (0., 0.) if waiting else self.follow_path(pose, target)
        else:
            target = self.target(np.asarray(pose[:2]))
            if self.method == "apf":
                raw = apf_command(pose, velocity, target, points, dt, self.cfg)
            elif self.method == "dwa":
                raw = self.dwa(pose, velocity, target, points, dt)
            else:
                raw = self.follow_path(pose, target)
        if self.crossing_guard is not None and getattr(self.crossing_guard, "speed_scale", 1.) != 1.:
            raw = raw[0]*self.crossing_guard.speed_scale, raw[1]
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

    def diagnostics(self, finish_time=None):
        events = list(self.crossing_guard.events) if self.crossing_guard else []
        if events and finish_time is not None:
            events = [{**event, "end": min(event["end"], finish_time)} for event in events
                      if event["start"] < finish_time]
        return {"role": self.role, "guide_planner": self.guide_planner,
                "yield_count": len(events), "yield_time_s": sum(e["end"]-e["start"] for e in events),
                "yield_events": events,
                "replan_count": len(self.route_updates)-1,
                "replan_attempt_count": len(self.plan_attempts),
                "replan_attempts": self.plan_attempts,
                "replan_time_ms": sum(p["time_ms"] for p in self.plan_attempts),
                "route_updates": self.route_updates}
