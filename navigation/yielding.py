"""Predict short crossing encounters from recent, world-frame laser returns."""
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree


def _detections(points, robot):
    """Group nearby surface hits, keeping separate objects separate across lidar sectors."""
    if not len(points):
        return []
    neighbours = cKDTree(points).query_ball_point(points, .23)
    unseen = set(range(len(points)))
    objects = []
    while unseen:
        group = [unseen.pop()]
        for i in group:
            adjacent = unseen.intersection(neighbours[i])
            unseen.difference_update(adjacent)
            group.extend(sorted(adjacent))
        surface = points[group]
        mean = surface.mean(axis=0)
        centered = surface-mean
        # A circle fit avoids apparent centroid motion when the robot changes
        # viewing angle. Short or nearly straight arcs use a range-based fallback.
        if len(surface) >= 3:
            design = np.column_stack((2*centered, np.ones(len(surface))))
            fit, _, rank, _ = np.linalg.lstsq(design, (centered**2).sum(axis=1), rcond=None)
            center = mean+fit[:2]
            radii = np.linalg.norm(surface-center, axis=1)
            radius = float(np.mean(radii))
            if rank == 3 and .06 <= radius <= .50 and radii.std() < .025:
                objects.append((center, radius))
                continue
        radius = float(np.clip(np.linalg.norm(centered, axis=1).max(), .16, .50))
        sight = mean-robot
        objects.append((mean+sight*(radius/max(np.linalg.norm(sight), 1e-9)), radius))
    return objects


@dataclass
class _Track:
    samples: deque = field(default_factory=deque)
    radius: float = .16
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def predict(self, t):
        stamp, center = self.samples[-1]
        return center+(t-stamp)*self.velocity

    def update(self, t, center, radius):
        self.samples.append((t, center))
        self.radius = radius
        while t-self.samples[0][0] > .65:
            self.samples.popleft()
        if t-self.samples[0][0] < .25:
            self.velocity = np.zeros(2)
            return
        stamps = np.array([s for s, _ in self.samples])
        times = stamps-stamps.mean()
        self.velocity = times @ np.array([p for _, p in self.samples]) / np.dot(times, times)


class CrossingGuard:
    def __init__(self, config):
        self.config = config
        self.tracks = []
        self.hold_until = -np.inf
        self.events = []
        self.holding = False
        self.progress_position = None
        self.progress_time = 0.

    def stalled(self, t, position):
        """After a blockage, retry from the actual pose if the follower cannot move."""
        if (self.holding or self.progress_position is None
                or np.linalg.norm(position-self.progress_position) >= .10):
            self.progress_position = position.copy()
            self.progress_time = t
        if t-self.progress_time < 2.5:
            return False
        self.progress_time = t
        return True

    def observe(self, t, robot, hits):
        self.tracks = [track for track in self.tracks if t-track.samples[-1][0] <= .55]
        objects = _detections(hits, robot)
        pairs = sorted((np.linalg.norm(track.predict(t)-center), i, j)
                       for i, track in enumerate(self.tracks) for j, (center, _) in enumerate(objects))
        assigned, taken = set(), set()
        for distance, i, j in pairs:
            if distance <= .38 and i not in assigned and j not in taken:
                self.tracks[i].update(t, *objects[j])
                assigned.add(i)
                taken.add(j)
        for j, (center, radius) in enumerate(objects):
            if j not in taken:
                track = _Track()
                track.update(t, center, radius)
                self.tracks.append(track)

    def should_yield(self, t, dt, position, speed, route):
        """Compare simultaneous positions for the next part of the robot's route."""
        cfg = self.config
        horizon = np.arange(0., 3.6, .12)
        path = np.vstack((position, route))
        distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        # A stopped robot must test the trip it would make if it restarted.
        delay = max(0., cfg.max_speed-speed) / cfg.acceleration
        accel_t = np.minimum(horizon, delay)
        travel = speed*accel_t + .5*cfg.acceleration*accel_t**2 + cfg.max_speed*(horizon-accel_t)
        robot = np.column_stack([np.interp(travel, distance, path[:, k]) for k in range(2)])
        braking_t = np.minimum(horizon, speed/cfg.acceleration)
        stopping = speed*braking_t-.5*cfg.acceleration*braking_t**2
        stopped = np.column_stack([np.interp(stopping, distance, path[:, k]) for k in range(2)])
        conflict, unsafe_stop = False, False
        for track in self.tracks:
            if np.linalg.norm(track.velocity) < .05:
                continue
            other = track.predict(t) + horizon[:, None]*track.velocity
            clearance = cfg.radius + track.radius + cfg.margin
            passing_gap = np.linalg.norm(robot-other, axis=1).min()
            stopped_gap = np.linalg.norm(stopped-other, axis=1).min()
            conflict |= passing_gap <= clearance + stopping[-1]
            # If already crossing, stopping in the cart's path can be more
            # dangerous than clearing it. Keep following and checking the route.
            unsafe_stop |= stopped_gap < clearance and stopped_gap < passing_gap
        if unsafe_stop:
            self.hold_until = t
        elif conflict:
            self.hold_until = t+.55
        hold = t < self.hold_until
        if hold and not self.holding:
            self.events.append({"start": float(t), "end": float(t)})
        if hold:
            self.events[-1]["end"] = float(t+dt)
        self.holding = hold
        return hold
