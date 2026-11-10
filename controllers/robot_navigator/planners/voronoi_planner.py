"""
广义沃罗诺伊图 (Generalized Voronoi Diagram, GVD) 路径规划算法
利用计算几何提取自由空间中轴骨架 (Medial Axis)，生成具有最大避障安全间距 (Maximal Clearance) 的拓扑路径。
参考文献:
1. Ó'Dúnlaing, C., & Yap, C. K. (1982). A "retraction" method for planning the shortest collision-free paths of a disc between obstacles.
   SIAM Journal on Computing, 11(4), 608-635.
2. Choset, H., & Burdick, J. (2000). Sensor-based exploration: The hierarchical generalized Voronoi graph.
   The International Journal of Robotics Research, 19(2), 96-125.
"""

import math
import time
import heapq
import numpy as np
from scipy.spatial import Voronoi
from typing import List, Tuple, Optional, Dict
from utils.grid_map import GridMap
from utils.geometry import euclidean_dist

class VoronoiPlanner:
    def __init__(self, grid_map: GridMap, min_clearance: float = 0.35):
        self.grid_map = grid_map
        self.min_clearance = min_clearance

    def _sample_obstacle_boundaries(self) -> np.ndarray:
        """从已知静态障碍物几何表面均匀采样边界点以构造沃罗诺伊图"""
        obs_pts = []
        # 四周边界墙采样
        for y in np.linspace(-4.9, 4.9, 60):
            obs_pts.extend([[-4.9, y], [4.9, y]])
        for x in np.linspace(-4.9, 4.9, 60):
            obs_pts.extend([[x, -4.9], [x, 4.9]])

        # 内部各静态障碍物轮廓采样
        for obs in self.grid_map.obstacles:
            if obs.kind == 'box':
                cx, cy = obs.params['center']
                sx, sy = obs.params['size']
                hx, hy = sx / 2.0, sy / 2.0
                nx = max(4, int(sx / 0.15))
                ny = max(4, int(sy / 0.15))
                for px in np.linspace(cx - hx, cx + hx, nx):
                    obs_pts.extend([[px, cy - hy], [px, cy + hy]])
                for py in np.linspace(cy - hy, cy + hy, ny):
                    obs_pts.extend([[cx - hx, py], [cx + hx, py]])
            elif obs.kind == 'cylinder':
                cx, cy = obs.params['center']
                r = obs.params['radius']
                num_pts = max(16, int(2.0 * math.pi * r / 0.12))
                for theta in np.linspace(0, 2.0 * math.pi, num_pts, endpoint=False):
                    obs_pts.append([cx + r * math.cos(theta), cy + r * math.sin(theta)])

        return np.array(obs_pts, dtype=np.float64)

    def plan(
        self, 
        start: Tuple[float, float], 
        goal: Tuple[float, float]
    ) -> Tuple[Optional[List[Tuple[float, float]]], Dict[str, float]]:
        """
        基于 Voronoi 中轴骨架图搜索全局安全路径
        """
        start_time = time.perf_counter()

        # 1. 采样障碍物表面点并求解 Voronoi 图
        obs_pts = self._sample_obstacle_boundaries()
        vor = Voronoi(obs_pts)

        # 2. 筛选合法骨架顶点 (必须在竞技场内部且与障碍物满足安全阈值)
        valid_verts = {}
        margin_bound = 0.25
        for i, v in enumerate(vor.vertices):
            vx, vy = float(v[0]), float(v[1])
            if (self.grid_map.min_x + margin_bound <= vx <= self.grid_map.max_x - margin_bound and
                self.grid_map.min_y + margin_bound <= vy <= self.grid_map.max_y - margin_bound):
                if self.grid_map.is_collision_free_point(vx, vy, margin=self.min_clearance):
                    valid_verts[i] = (vx, vy)

        if not valid_verts:
            return None, {'success': 0, 'error': 'No valid Voronoi vertices'}

        # 3. 构造 Voronoi 骨架连通邻接图 G(V, E)
        adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in valid_verts}
        filtered_edges = []

        for v1_idx, v2_idx in vor.ridge_vertices:
            if v1_idx in valid_verts and v2_idx in valid_verts:
                p1 = valid_verts[v1_idx]
                p2 = valid_verts[v2_idx]
                if self.grid_map.is_line_collision_free(p1, p2, step=0.04, margin=self.min_clearance):
                    d = euclidean_dist(p1, p2)
                    adj[v1_idx].append((v2_idx, d))
                    adj[v2_idx].append((v1_idx, d))
                    filtered_edges.append((p1, p2))

        # 4. 将起点与终点投影连接至 Voronoi 骨架网络
        start_node = -1
        goal_node = -2
        adj[start_node] = []
        adj[goal_node] = []

        # 连接起点至最近可见骨架点
        connect_candidates_start = []
        for v_idx, pos in valid_verts.items():
            d = euclidean_dist(start, pos)
            if d < 2.5:
                connect_candidates_start.append((d, v_idx, pos))
        connect_candidates_start.sort(key=lambda x: x[0])

        start_connected = False
        for d, v_idx, pos in connect_candidates_start[:8]:
            if self.grid_map.is_line_collision_free(start, pos, step=0.04, margin=self.min_clearance):
                adj[start_node].append((v_idx, d))
                adj[v_idx].append((start_node, d))
                start_connected = True

        # 连接终点至最近可见骨架点
        connect_candidates_goal = []
        for v_idx, pos in valid_verts.items():
            d = euclidean_dist(goal, pos)
            if d < 2.5:
                connect_candidates_goal.append((d, v_idx, pos))
        connect_candidates_goal.sort(key=lambda x: x[0])

        goal_connected = False
        for d, v_idx, pos in connect_candidates_goal[:8]:
            if self.grid_map.is_line_collision_free(goal, pos, step=0.04, margin=self.min_clearance):
                adj[goal_node].append((v_idx, d))
                adj[v_idx].append((goal_node, d))
                goal_connected = True

        if not start_connected or not goal_connected:
            return None, {'success': 0, 'error': 'Failed to connect start or goal to Voronoi skeleton'}

        # 5. Dijkstra 最短路搜索
        pq = [(0.0, start_node)]
        dist_dict = {start_node: 0.0}
        parent = {}

        while pq:
            cur_d, u = heapq.heappop(pq)
            if u == goal_node:
                break
            if cur_d > dist_dict.get(u, float('inf')):
                continue

            for v, w in adj[u]:
                if cur_d + w < dist_dict.get(v, float('inf')):
                    dist_dict[v] = cur_d + w
                    parent[v] = u
                    heapq.heappush(pq, (cur_d + w, v))

        if goal_node not in parent:
            planning_time = (time.perf_counter() - start_time) * 1000.0
            return None, {'success': 0, 'planning_time_ms': planning_time, 'error': 'Voronoi path disconnected'}

        # 回溯提取骨架路径
        raw_path = []
        curr = goal_node
        while curr != start_node:
            pt = goal if curr == goal_node else valid_verts[curr]
            raw_path.append(pt)
            curr = parent[curr]
        raw_path.append(start)
        raw_path.reverse()

        raw_length = sum(euclidean_dist(raw_path[i], raw_path[i+1]) for i in range(len(raw_path)-1))

        # 视线平滑修剪 (保留至少 0.50m 走廊中轴安全裕度)
        smoothed_path = self._smooth_path(raw_path)
        smoothed_length = sum(euclidean_dist(smoothed_path[i], smoothed_path[i+1]) for i in range(len(smoothed_path)-1))

        planning_time_ms = (time.perf_counter() - start_time) * 1000.0

        info = {
            'success': 1,
            'planning_time_ms': planning_time_ms,
            'voronoi_vertices_count': len(valid_verts),
            'voronoi_edges_count': len(filtered_edges),
            'raw_length_m': raw_length,
            'path_length_m': smoothed_length,
            'waypoints_count': len(smoothed_path),
            'voronoi_edges': filtered_edges  # 供画图可视化使用
        }

        return smoothed_path, info

    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """在保证充足安全间隙的前提下进行视线平滑修剪"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            furthest_idx = current_idx + 1
            for next_idx in range(len(path) - 1, current_idx, -1):
                if self.grid_map.is_line_collision_free(path[current_idx], path[next_idx], step=0.04, margin=0.50):
                    furthest_idx = next_idx
                    break
            smoothed.append(path[furthest_idx])
            current_idx = furthest_idx

        return smoothed
