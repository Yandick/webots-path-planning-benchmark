"""
RRT* (渐近最优快速扩展随机树) 全局路径规划算法
参考文献:
Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning.
The International Journal of Robotics Research, 30(7), 846-894.
"""

import math
import random
import time
from typing import List, Tuple, Optional, Dict
from utils.grid_map import GridMap
from utils.geometry import euclidean_dist

class RRTNode:
    def __init__(self, x: float, y: float, parent: Optional['RRTNode'] = None, cost: float = 0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost

class RRTStarPlanner:
    def __init__(
        self,
        grid_map: GridMap,
        max_iterations: int = 1500,
        step_size: float = 0.35,
        goal_sample_rate: float = 0.15,
        search_radius: float = 1.2,
        goal_tolerance: float = 0.35
    ):
        self.grid_map = grid_map
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.goal_tolerance = goal_tolerance

    def _sample(self, goal: Tuple[float, float]) -> Tuple[float, float]:
        """带目标偏置 (Goal Biasing) 的均匀随机空间采样"""
        if random.random() < self.goal_sample_rate:
            return goal
        rx = random.uniform(self.grid_map.min_x + 0.2, self.grid_map.max_x - 0.2)
        ry = random.uniform(self.grid_map.min_y + 0.2, self.grid_map.max_y - 0.2)
        return rx, ry

    def _get_nearest_node(self, nodes: List[RRTNode], q_rand: Tuple[float, float]) -> RRTNode:
        """寻找距离采样点最近的树节点"""
        best_node = nodes[0]
        best_dist = math.hypot(nodes[0].x - q_rand[0], nodes[0].y - q_rand[1])
        for node in nodes[1:]:
            d = math.hypot(node.x - q_rand[0], node.y - q_rand[1])
            if d < best_dist:
                best_dist = d
                best_node = node
        return best_node

    def _steer(self, q_from: RRTNode, q_to: Tuple[float, float]) -> Tuple[float, float]:
        """步长截断生长函数 (Steering)"""
        dist = math.hypot(q_to[0] - q_from.x, q_to[1] - q_from.y)
        if dist <= self.step_size:
            return q_to
        theta = math.atan2(q_to[1] - q_from.y, q_to[0] - q_from.x)
        nx = q_from.x + self.step_size * math.cos(theta)
        ny = q_from.y + self.step_size * math.sin(theta)
        return nx, ny

    def _find_near_nodes(self, nodes: List[RRTNode], q_new: Tuple[float, float], radius: float) -> List[RRTNode]:
        """寻找在动态搜索半径内的邻近节点集合"""
        near = []
        r_sq = radius ** 2
        for node in nodes:
            dist_sq = (node.x - q_new[0]) ** 2 + (node.y - q_new[1]) ** 2
            if dist_sq <= r_sq:
                near.append(node)
        return near

    def plan(
        self, 
        start: Tuple[float, float], 
        goal: Tuple[float, float]
    ) -> Tuple[Optional[List[Tuple[float, float]]], Dict[str, float]]:
        """
        执行 RRT* 规划
        
        参数:
            start: 起点 (x, y)
            goal: 终点 (x, y)
            
        返回:
            (path, info_dict)
        """
        start_time = time.perf_counter()

        # 起点与终点有效性检测
        if not self.grid_map.is_collision_free_point(start[0], start[1]):
            return None, {'success': 0, 'error': 'Start in collision'}
        if not self.grid_map.is_collision_free_point(goal[0], goal[1]):
            return None, {'success': 0, 'error': 'Goal in collision'}

        root = RRTNode(start[0], start[1], parent=None, cost=0.0)
        nodes: List[RRTNode] = [root]

        goal_nodes: List[RRTNode] = []
        best_goal_cost = float('inf')
        best_goal_node: Optional[RRTNode] = None

        gamma = 2.5 * math.sqrt(1.0 / math.pi)

        for i in range(self.max_iterations):
            q_rand = self._sample(goal)
            q_nearest = self._get_nearest_node(nodes, q_rand)
            q_new_pos = self._steer(q_nearest, q_rand)

            # 边碰撞检测
            if not self.grid_map.is_line_collision_free((q_nearest.x, q_nearest.y), q_new_pos, step=0.05):
                continue

            # 动态近邻搜索半径 (RRT* 理论收敛半径)
            n_nodes = len(nodes)
            cur_radius = min(self.search_radius, gamma * math.sqrt(math.log(n_nodes + 1) / (n_nodes + 1)) * 4.0)
            near_nodes = self._find_near_nodes(nodes, q_new_pos, max(cur_radius, self.step_size))

            # 1. Choose Parent (挑选代价最低的最优父节点)
            min_cost = q_nearest.cost + euclidean_dist((q_nearest.x, q_nearest.y), q_new_pos)
            best_parent = q_nearest

            for n_near in near_nodes:
                d = euclidean_dist((n_near.x, n_near.y), q_new_pos)
                if n_near.cost + d < min_cost:
                    if self.grid_map.is_line_collision_free((n_near.x, n_near.y), q_new_pos, step=0.05):
                        min_cost = n_near.cost + d
                        best_parent = n_near

            new_node = RRTNode(q_new_pos[0], q_new_pos[1], parent=best_parent, cost=min_cost)
            nodes.append(new_node)

            # 2. Rewiring (重连局部子树以优化路径代价)
            for n_near in near_nodes:
                if n_near == best_parent:
                    continue
                d = euclidean_dist((new_node.x, new_node.y), (n_near.x, n_near.y))
                if new_node.cost + d < n_near.cost:
                    if self.grid_map.is_line_collision_free((new_node.x, new_node.y), (n_near.x, n_near.y), step=0.05):
                        n_near.parent = new_node
                        n_near.cost = new_node.cost + d

            # 检查是否可到达目标点
            dist_to_goal = euclidean_dist((new_node.x, new_node.y), goal)
            if dist_to_goal <= self.goal_tolerance:
                if self.grid_map.is_line_collision_free((new_node.x, new_node.y), goal, step=0.05):
                    total_cost = new_node.cost + dist_to_goal
                    if total_cost < best_goal_cost:
                        best_goal_cost = total_cost
                        best_goal_node = new_node
                        # 找到较优可行解后，可稍作精炼优化后提前返回
                        if i >= 600:
                            break

        if best_goal_node is None:
            planning_time = (time.perf_counter() - start_time) * 1000.0
            return None, {'success': 0, 'planning_time_ms': planning_time, 'iterations': self.max_iterations}

        # 路径回溯
        raw_path = [goal]
        curr = best_goal_node
        while curr is not None:
            raw_path.append((curr.x, curr.y))
            curr = curr.parent
        raw_path.reverse()

        raw_length = 0.0
        for i in range(len(raw_path) - 1):
            raw_length += euclidean_dist(raw_path[i], raw_path[i + 1])

        # 视线平滑修剪
        smoothed_path = self._smooth_path(raw_path)

        smoothed_length = 0.0
        for i in range(len(smoothed_path) - 1):
            smoothed_length += euclidean_dist(smoothed_path[i], smoothed_path[i + 1])

        planning_time_ms = (time.perf_counter() - start_time) * 1000.0

        info = {
            'success': 1,
            'planning_time_ms': planning_time_ms,
            'nodes_count': len(nodes),
            'raw_length_m': raw_length,
            'path_length_m': smoothed_length,
            'waypoints_count': len(smoothed_path),
        }

        return smoothed_path, info

    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """贪心视线平滑修剪 (保留至少 0.55m 安全间隙，走廊居中)"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            furthest_idx = current_idx + 1
            for next_idx in range(len(path) - 1, current_idx, -1):
                if self.grid_map.is_line_collision_free(path[current_idx], path[next_idx], step=0.04, margin=0.55):
                    furthest_idx = next_idx
                    break
            smoothed.append(path[furthest_idx])
            current_idx = furthest_idx

        return smoothed
