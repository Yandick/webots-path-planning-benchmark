"""Scene-independent occupancy grids and Euclidean obstacle inflation."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


class StaticObstacle:
    def __init__(self, kind: str, params: dict):
        if kind not in ("box", "cylinder"):
            raise ValueError(f"Unsupported obstacle geometry: {kind}")
        self.kind = kind
        self.params = params


class GridMap:
    def __init__(self, obstacles, min_x=-5., max_x=5., min_y=-5., max_y=5.,
                 resolution=.05, robot_radius=.20, safety_margin=.12):
        self.min_x, self.max_x = min_x, max_x
        self.min_y, self.max_y = min_y, max_y
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.inflation_radius = robot_radius+safety_margin
        self.width = int(math.ceil((max_x-min_x)/resolution))
        self.height = int(math.ceil((max_y-min_y)/resolution))
        self.obstacles = list(obstacles)
        self.static_grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self._rasterize_and_inflate()

    def _rasterize_and_inflate(self):
        xs = np.linspace(self.min_x+self.resolution/2,
                         self.max_x-self.resolution/2, self.width)
        ys = np.linspace(self.min_y+self.resolution/2,
                         self.max_y-self.resolution/2, self.height)
        xv, yv = np.meshgrid(xs, ys)
        for obs in self.obstacles:
            cx, cy = obs.params["center"]
            if obs.kind == "box":
                sx, sy = obs.params["size"]
                mask = ((xv >= cx-sx/2) & (xv <= cx+sx/2) &
                        (yv >= cy-sy/2) & (yv <= cy+sy/2))
            else:
                mask = (xv-cx)**2+(yv-cy)**2 <= obs.params["radius"]**2
            self.static_grid[mask] = 1
        self.dist_map = distance_transform_edt(self.static_grid == 0)*self.resolution
        self.inflated_grid = (self.dist_map <= self.inflation_radius).astype(np.uint8)

    def world_to_grid(self, x, y):
        return int((x-self.min_x)/self.resolution), int((y-self.min_y)/self.resolution)

    def grid_to_world(self, gx, gy):
        return (self.min_x+(gx+.5)*self.resolution,
                self.min_y+(gy+.5)*self.resolution)

    def is_in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx, gy, use_inflation=True):
        if not self.is_in_bounds(gx, gy):
            return True
        grid = self.inflated_grid if use_inflation else self.static_grid
        return grid[gy, gx] == 1

    def is_collision_free_point(self, x, y, margin=None):
        if x <= self.min_x or x >= self.max_x or y <= self.min_y or y >= self.max_y:
            return False
        gx, gy = self.world_to_grid(x, y)
        if not self.is_in_bounds(gx, gy):
            return False
        required = self.inflation_radius if margin is None else margin
        return self.dist_map[gy, gx] > required

    def is_line_collision_free(self, p1, p2, step=.05, margin=None):
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-5:
            return self.is_collision_free_point(*p1, margin)
        count = int(math.ceil(distance/step))
        return all(self.is_collision_free_point(p1[0]+i/count*dx,
                                                p1[1]+i/count*dy, margin)
                   for i in range(count+1))
