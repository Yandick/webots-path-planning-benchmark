"""
A* 全局路径规划算法 (A* Global Path Planner)
基于 8-邻域网格搜索、欧几里得启发式函数以及视线平滑 (Line-of-Sight Shortcut Smoothing)。
参考文献:
Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths.
IEEE Transactions on Systems Science and Cybernetics, 4(2), 100-107.
"""

import heapq
import math
import time
from typing import List, Tuple, Optional, Dict
from utils.grid_map import GridMap
from utils.geometry import euclidean_dist

class AStarPlanner:
    def __init__(self, grid_map: GridMap, clearance_weight: float = 0.5):
        self.grid_map = grid_map
        self.clearance_weight = clearance_weight

        # 8-邻域移动方向及对应几何欧氏距离
        # (dx, dy, cost)
        sqrt2 = math.sqrt(2.0)
        self.motions = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, sqrt2),
            (1, -1, sqrt2),
            (-1, 1, sqrt2),
            (-1, -1, sqrt2),
        ]

    def _heuristic(self, gx: int, gy: int, goal_gx: int, goal_gy: int) -> float:
        """欧氏距离启发式函数 (Euclidean Distance Heuristic)"""
        return math.hypot(gx - goal_gx, gy - goal_gy) * self.grid_map.resolution

    def plan(
        self, 
        start: Tuple[float, float], 
        goal: Tuple[float, float]
    ) -> Tuple[Optional[List[Tuple[float, float]]], Dict[str, float]]:
        """
        执行 A* 规划
        
        参数:
            start: 起点物理坐标 (x, y)
            goal: 终点物理坐标 (x, y)
            
        返回:
            (path, info_dict):
                path: 规划出的平滑路径点列表 [(x0, y0), (x1, y1), ...] 或 None (若无解)
                info_dict: 包含 planning_time, raw_length, smoothed_length, nodes_expanded
        """
        start_time = time.perf_counter()

        start_gx, start_gy = self.grid_map.world_to_grid(start[0], start[1])
        goal_gx, goal_gy = self.grid_map.world_to_grid(goal[0], goal[1])

        # 起点和终点有效性检查
        if not self.grid_map.is_in_bounds(start_gx, start_gy) or not self.grid_map.is_in_bounds(goal_gx, goal_gy):
            return None, {'success': 0, 'error': 'Start or goal out of bounds'}

        if self.grid_map.is_occupied(start_gx, start_gy) or self.grid_map.is_occupied(goal_gx, goal_gy):
            return None, {'success': 0, 'error': 'Start or goal inside obstacle'}

        # 优先队列 open_set: 元素为 (f_score, gx, gy)
        open_set = []
        heapq.heappush(open_set, (0.0, start_gx, start_gy))

        # g_score 字典: (gx, gy) -> g
        g_score: Dict[Tuple[int, int], float] = {(start_gx, start_gy): 0.0}

        # 记录父节点用于回溯
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        # 闭合集合 closed_set
        closed_set = set()
        nodes_expanded = 0

        goal_reached = False

        while open_set:
            current_f, cur_gx, cur_gy = heapq.heappop(open_set)

            if (cur_gx, cur_gy) in closed_set:
                continue
            closed_set.add((cur_gx, cur_gy))
            nodes_expanded += 1

            # 到达目标点
            if cur_gx == goal_gx and cur_gy == goal_gy:
                goal_reached = True
                break

            current_g = g_score[(cur_gx, cur_gy)]

            for dx, dy, step_cost in self.motions:
                nbr_gx = cur_gx + dx
                nbr_gy = cur_gy + dy

                if not self.grid_map.is_in_bounds(nbr_gx, nbr_gy):
                    continue

                if (nbr_gx, nbr_gy) in closed_set:
                    continue

                if self.grid_map.is_occupied(nbr_gx, nbr_gy):
                    continue

                # 对角线穿墙保护 (Prevent diagonal crossing between adjacent obstacles)
                if dx != 0 and dy != 0:
                    if self.grid_map.is_occupied(cur_gx + dx, cur_gy) or self.grid_map.is_occupied(cur_gx, cur_gy + dy):
                        continue

                # 距离障碍物的安全额外代价值 (Clearance Cost: 距离障碍物越近，惩罚越大)
                dist_to_obs = self.grid_map.dist_map[nbr_gy, nbr_gx]
                clearance_penalty = 0.0
                if dist_to_obs < 1.0:
                    clearance_penalty = self.clearance_weight * (1.0 - dist_to_obs)

                tentative_g = current_g + (step_cost * self.grid_map.resolution) + clearance_penalty

                if (nbr_gx, nbr_gy) not in g_score or tentative_g < g_score[(nbr_gx, nbr_gy)]:
                    g_score[(nbr_gx, nbr_gy)] = tentative_g
                    came_from[(nbr_gx, nbr_gy)] = (cur_gx, cur_gy)
                    f_val = tentative_g + self._heuristic(nbr_gx, nbr_gy, goal_gx, goal_gy)
                    heapq.heappush(open_set, (f_val, nbr_gx, nbr_gy))

        if not goal_reached:
            planning_time = (time.perf_counter() - start_time) * 1000.0
            return None, {'success': 0, 'planning_time_ms': planning_time, 'nodes_expanded': nodes_expanded}

        # 路径回溯 (Reconstruct Path)
        raw_grid_path = []
        curr = (goal_gx, goal_gy)
        while curr in came_from:
            raw_grid_path.append(curr)
            curr = came_from[curr]
        raw_grid_path.append((start_gx, start_gy))
        raw_grid_path.reverse()

        # 转换回连续物理世界坐标
        raw_world_path = [start]
        for gx, gy in raw_grid_path[1:-1]:
            raw_world_path.append(self.grid_map.grid_to_world(gx, gy))
        raw_world_path.append(goal)

        # 计算原始路径长度
        raw_length = 0.0
        for i in range(len(raw_world_path) - 1):
            raw_length += euclidean_dist(raw_world_path[i], raw_world_path[i + 1])

        # 视线平滑修剪 (Line-of-Sight Shortcut Smoothing)
        smoothed_path = self._smooth_path(raw_world_path)

        # 计算平滑后路径长度
        smoothed_length = 0.0
        for i in range(len(smoothed_path) - 1):
            smoothed_length += euclidean_dist(smoothed_path[i], smoothed_path[i + 1])

        planning_time_ms = (time.perf_counter() - start_time) * 1000.0

        info = {
            'success': 1,
            'planning_time_ms': planning_time_ms,
            'nodes_expanded': nodes_expanded,
            'raw_length_m': raw_length,
            'path_length_m': smoothed_length,
            'waypoints_count': len(smoothed_path),
        }

        return smoothed_path, info

    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """利用连续无碰撞视线检测消除冗余网格阶梯点 (保留至少 0.55m 安全间隙，走廊居中)"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # 贪心向前探寻最远可直达的点，保持宽裕的居中安全间距
            furthest_idx = current_idx + 1
            for next_idx in range(len(path) - 1, current_idx, -1):
                if self.grid_map.is_line_collision_free(path[current_idx], path[next_idx], step=0.04, margin=0.55):
                    furthest_idx = next_idx
                    break
            smoothed.append(path[furthest_idx])
            current_idx = furthest_idx

        return smoothed
