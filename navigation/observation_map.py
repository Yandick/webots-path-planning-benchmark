"""Shared static/observed obstacle layers with ray clearing and finite retention.

The interface accepts pose and measured surface hits, never actor trajectories.
Layering, marking and ray clearing follow the ROS costmap obstacle-layer design.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt

from .scenarios import static_distance


class ObservationMap:
    def __init__(self, scene, retention=1.):
        self.grid = scene.grid()
        self.static = scene.static
        self.retention = retention
        self.stamps = np.full_like(self.grid.dist_map, -np.inf)
        self.dynamic = np.zeros_like(self.grid.static_grid, dtype=bool)
        self.dynamic_distance = np.full_like(self.grid.dist_map, np.inf)

    def update(self, position, points, t):
        grid = self.grid
        if len(points):
            delta = points-np.asarray(position)
            ranges = np.linalg.norm(delta, axis=1)
            # Hit rays clear old dynamic cells before marking their endpoints.
            steps = np.arange(0., float(ranges.max()), grid.resolution*.8)
            free = np.asarray(position)+delta[:, None, :]/np.maximum(ranges[:, None, None], 1e-9)*steps[None, :, None]
            free = free[steps[None, :] < ranges[:, None]-.10]
            cells = np.floor((free-[grid.min_x, grid.min_y])/grid.resolution).astype(int)
            valid = (cells[:, 0]>=0)&(cells[:, 0]<grid.width)&(cells[:, 1]>=0)&(cells[:, 1]<grid.height)
            self.stamps[cells[valid, 1], cells[valid, 0]] = -np.inf
            hits = points[static_distance(points, self.static) > .05]
            cells = np.floor((hits-[grid.min_x, grid.min_y])/grid.resolution).astype(int)
            valid = (cells[:, 0]>=0)&(cells[:, 0]<grid.width)&(cells[:, 1]>=0)&(cells[:, 1]<grid.height)
            self.stamps[cells[valid, 1], cells[valid, 0]] = t
        dynamic = self.stamps >= t-self.retention
        if not np.array_equal(dynamic, self.dynamic):
            self.dynamic = dynamic
            occupied = (grid.static_grid != 0)|dynamic
            grid.dist_map = distance_transform_edt(~occupied)*grid.resolution
            grid.inflated_grid = (grid.dist_map <= grid.inflation_radius).astype(np.uint8)
            self.dynamic_distance = (distance_transform_edt(~dynamic)*grid.resolution
                                     if dynamic.any() else np.full_like(grid.dist_map, np.inf))

    def blocks(self, path):
        if not self.dynamic.any():
            return False
        grid = self.grid
        cells = np.floor((path-[grid.min_x, grid.min_y])/grid.resolution).astype(int)
        cells[:, 0] = np.clip(cells[:, 0], 0, grid.width-1)
        cells[:, 1] = np.clip(cells[:, 1], 0, grid.height-1)
        return bool(np.any(self.dynamic_distance[cells[:, 1], cells[:, 0]] <= grid.inflation_radius))

    def snapshot(self):
        ys, xs = np.nonzero(self.dynamic)
        grid = self.grid
        return np.column_stack((grid.min_x+(xs+.5)*grid.resolution,
                                grid.min_y+(ys+.5)*grid.resolution)).tolist()
