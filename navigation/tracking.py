"""Short-term obstacle velocity estimates from successive world-frame lidar scans."""
import numpy as np
from scipy.spatial import cKDTree

from .scenarios import static_distance


class ScanTracker:
    def __init__(self):
        self.previous = []
        self.tracks = []

    def update(self, points, static, t):
        velocities = np.zeros_like(points)
        indices = np.flatnonzero(static_distance(points, static) > .12)
        self.tracks = []
        if len(indices):
            unknown = points[indices]
            tree = cKDTree(unknown)
            remaining = set(range(len(unknown)))
            clusters = []
            while remaining:
                stack, component = [remaining.pop()], []
                while stack:
                    i = stack.pop()
                    component.append(i)
                    nearby = remaining.intersection(tree.query_ball_point(unknown[i], .40))
                    remaining.difference_update(nearby)
                    stack.extend(nearby)
                if len(component) >= 2:
                    clusters.append(component)
            available = set(range(len(self.previous)))
            for component in clusters:
                center = unknown[component].mean(axis=0)
                velocity = np.zeros(2)
                candidates = [i for i in available if 0 < t-self.previous[i]["t"] <= .5]
                if candidates:
                    i = min(candidates, key=lambda j: np.linalg.norm(center-self.previous[j]["center"]))
                    old = self.previous[i]
                    elapsed = t-old["t"]
                    if np.linalg.norm(center-old["center"]) <= .6*elapsed+.15:
                        measured = (center-old["center"])/elapsed
                        measured *= min(1., .6/max(np.linalg.norm(measured), 1e-9))
                        alpha = elapsed/(.25+elapsed)
                        velocity = (1-alpha)*old["velocity"]+alpha*measured
                        available.remove(i)
                track = {"center": center, "velocity": velocity, "t": t}
                self.tracks.append(track)
                velocities[indices[component]] = velocity
        self.previous = self.tracks
        return velocities

    def snapshot(self):
        return [{"center": v["center"].tolist(), "velocity": v["velocity"].tolist()}
                for v in self.tracks]
