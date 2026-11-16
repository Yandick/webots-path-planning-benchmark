"""
栅格地图与障碍物先验建模模块 (Grid Map & Obstacle Representation)
负责静态环境建模、坐标转换、障碍物膨胀 (Inflation)、碰撞检测与视线检测 (Line of Sight)。
"""

import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from typing import Tuple, List, Optional

class StaticObstacle:
    """静态障碍物几何描述 (支持轴对齐矩形 Box 和 圆柱 Cylinder)"""
    def __init__(self, kind: str, params: dict):
        self.kind = kind  # 'box' or 'cylinder'
        self.params = params

    def is_inside(self, x: float, y: float, margin: float = 0.0) -> bool:
        if self.kind == 'box':
            cx, cy = self.params['center']
            sx, sy = self.params['size']
            half_x = sx / 2.0 + margin
            half_y = sy / 2.0 + margin
            return (cx - half_x <= x <= cx + half_x) and (cy - half_y <= y <= cy + half_y)
        elif self.kind == 'cylinder':
            cx, cy = self.params['center']
            r = self.params['radius'] + margin
            return math.hypot(x - cx, y - cy) <= r
        return False


class GridMap:
    """
    2D 栅格地图类
    提供栅格/全局笛卡尔坐标双向投影、障碍物膨胀与碰撞检测。
    """
    def __init__(
        self,
        min_x: float = -5.0,
        max_x: float = 5.0,
        min_y: float = -5.0,
        max_y: float = 5.0,
        resolution: float = 0.05,
        robot_radius: float = 0.20,
        safety_margin: float = 0.12,
        obstacles: Optional[List[StaticObstacle]] = None
    ):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.inflation_radius = robot_radius + safety_margin

        self.width = int(math.ceil((max_x - min_x) / resolution))
        self.height = int(math.ceil((max_y - min_y) / resolution))

        # 0: 空闲, 1: 障碍物
        self.static_grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.inflated_grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self.dist_map = np.zeros((self.height, self.width), dtype=np.float32)

        # 预定义场景中的静态障碍物列表
        self.obstacles: List[StaticObstacle] = [
            # 边界四周矮墙
            StaticObstacle('box', {'center': (0.0, 5.0), 'size': (10.2, 0.2)}),
            StaticObstacle('box', {'center': (0.0, -5.0), 'size': (10.2, 0.2)}),
            StaticObstacle('box', {'center': (5.0, 0.0), 'size': (0.2, 10.2)}),
            StaticObstacle('box', {'center': (-5.0, 0.0), 'size': (0.2, 10.2)}),
            # 内部静态障碍物
            StaticObstacle('box', {'center': (0.0, 0.0), 'size': (1.4, 1.4)}),
            StaticObstacle('box', {'center': (-2.2, 1.5), 'size': (1.0, 2.4)}),
            StaticObstacle('box', {'center': (2.2, -1.5), 'size': (2.4, 1.0)}),
            StaticObstacle('cylinder', {'center': (-1.8, -1.8), 'radius': 0.5}),
            StaticObstacle('cylinder', {'center': (1.8, 1.8), 'radius': 0.5}),
            StaticObstacle('box', {'center': (-0.2, 3.2), 'size': (2.2, 0.7)}),
            StaticObstacle('cylinder', {'center': (0.2, -3.2), 'radius': 0.5}),
        ]

        if obstacles is not None:
            self.obstacles = list(obstacles)
        self._rasterize_and_inflate()

    def _rasterize_and_inflate(self):
        """栅格化静态障碍物并利用欧氏距离变换进行碰撞安全半径膨胀"""
        # 1. 栅格化基本几何
        xs = np.linspace(self.min_x + self.resolution / 2, self.max_x - self.resolution / 2, self.width)
        ys = np.linspace(self.min_y + self.resolution / 2, self.max_y - self.resolution / 2, self.height)
        xv, yv = np.meshgrid(xs, ys)

        for obs in self.obstacles:
            if obs.kind == 'box':
                cx, cy = obs.params['center']
                sx, sy = obs.params['size']
                hx, hy = sx / 2.0, sy / 2.0
                mask = (xv >= cx - hx) & (xv <= cx + hx) & (yv >= cy - hy) & (yv <= cy + hy)
                self.static_grid[mask] = 1
            elif obs.kind == 'cylinder':
                cx, cy = obs.params['center']
                r = obs.params['radius']
                dist_sq = (xv - cx) ** 2 + (yv - cy) ** 2
                mask = dist_sq <= (r ** 2)
                self.static_grid[mask] = 1

        # 2. 计算欧氏距离场 (距离最近障碍物的米制距离)
        free_space = (self.static_grid == 0)
        # distance_transform_edt 计算像素距离，乘以 resolution 转换为物理米数
        self.dist_map = distance_transform_edt(free_space) * self.resolution

        # 3. 膨胀层：距离小于机器人膨胀半径的区域视为致命不可通行区
        self.inflated_grid = np.where(self.dist_map <= self.inflation_radius, 1, 0).astype(np.uint8)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """连续世界坐标 (x, y) 转换为栅格索引 (gx, gy)"""
        gx = int((x - self.min_x) / self.resolution)
        gy = int((y - self.min_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """栅格索引 (gx, gy) 转换为连续世界坐标中心点 (x, y)"""
        x = self.min_x + (gx + 0.5) * self.resolution
        y = self.min_y + (gy + 0.5) * self.resolution
        return x, y

    def is_in_bounds(self, gx: int, gy: int) -> bool:
        """检查栅格索引是否在地图有效边界内"""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx: int, gy: int, use_inflation: bool = True) -> bool:
        """检查栅格是否为障碍物 (可选择是否考虑膨胀层)"""
        if not self.is_in_bounds(gx, gy):
            return True
        grid = self.inflated_grid if use_inflation else self.static_grid
        return grid[gy, gx] == 1

    def is_collision_free_point(self, x: float, y: float, margin: float = None) -> bool:
        """在连续坐标下检测单点是否安全无碰撞"""
        if x <= self.min_x or x >= self.max_x or y <= self.min_y or y >= self.max_y:
            return False
        gx, gy = self.world_to_grid(x, y)
        if not self.is_in_bounds(gx, gy):
            return False
        req_margin = self.inflation_radius if margin is None else margin
        return self.dist_map[gy, gx] > req_margin

    def is_line_collision_free(
        self, 
        p1: Tuple[float, float], 
        p2: Tuple[float, float], 
        step: float = 0.05,
        margin: float = None
    ) -> bool:
        """视线连续碰撞检测 (Line-of-Sight Check)"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-5:
            return self.is_collision_free_point(p1[0], p1[1], margin)

        steps = int(math.ceil(dist / step))
        for i in range(steps + 1):
            ratio = i / float(steps)
            x = p1[0] + ratio * dx
            y = p1[1] + ratio * dy
            if not self.is_collision_free_point(x, y, margin):
                return False
        return True
