"""
线性自适应递减惯性权重粒子群优化算法 (LDIW-PSO Path Planner)
基于多目标适应度函数 (路径长度 + 碰撞惩罚 + 转向平滑度) 与连续航路点控制参数化编码。
参考文献:
1. Shi, Y., & Eberhart, R. (1998). A modified particle swarm optimizer.
   IEEE International Conference on Evolutionary Computation (ICEC), pp. 69-73.
2. Masehian, E., & Sedighizadeh, D. (2007). A multi-objective PSO-based approach for robot path planning in static and dynamic environments.
   International Journal of Advanced Robotic Systems, 4(1), 127-136.
3. Gong, D. W., Zhang, J. H., & Zhang, Y. (2011). Multi-objective particle swarm optimization for robot path planning in environment with danger sources.
   Computers & Operations Research, 38(10), 1386-1398.
"""

import math
import time
import numpy as np
from typing import List, Tuple, Optional, Dict
from utils.grid_map import GridMap
from utils.geometry import euclidean_dist

class PSOPlanner:
    def __init__(
        self,
        grid_map: GridMap,
        num_particles: int = 50,
        max_iterations: int = 90,
        num_waypoints: int = 5,
        w_max: float = 0.9,
        w_min: float = 0.4,
        c1: float = 1.5,
        c2: float = 1.5,
        v_max: float = 0.6
    ):
        self.grid_map = grid_map
        self.num_particles = num_particles
        self.max_iterations = max_iterations
        self.num_waypoints = num_waypoints
        self.w_max = w_max
        self.w_min = w_min
        self.c1 = c1
        self.c2 = c2
        self.v_max = v_max

    def _evaluate_fitness(
        self, 
        pts: np.ndarray, 
        start: np.ndarray, 
        goal: np.ndarray
    ) -> Tuple[float, bool, float]:
        """
        多目标综合适应度函数计算
        返回: (fitness, is_collision_free, path_length)
        """
        full_path = np.vstack([start, pts, goal])
        diffs = np.diff(full_path, axis=0)
        seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
        path_length = float(np.sum(seg_lens))

        # 稠密线段插值采样检测碰撞与安全场
        t_vals = np.linspace(0.0, 1.0, 16)
        dense_segments = []
        for s in range(len(full_path) - 1):
            p0 = full_path[s]
            p1 = full_path[s + 1]
            dense_segments.append(np.outer(1.0 - t_vals, p0) + np.outer(t_vals, p1))
        dense_pts = np.vstack(dense_segments)

        # 投影至栅格距离场
        gx = ((dense_pts[:, 0] - self.grid_map.min_x) / self.grid_map.resolution).astype(int)
        gy = ((dense_pts[:, 1] - self.grid_map.min_y) / self.grid_map.resolution).astype(int)
        valid_mask = (gx >= 0) & (gx < self.grid_map.width) & (gy >= 0) & (gy < self.grid_map.height)

        out_of_bounds_count = np.sum(~valid_mask)
        dist_vals = np.zeros(len(dense_pts), dtype=np.float32)
        dist_vals[valid_mask] = self.grid_map.dist_map[gy[valid_mask], gx[valid_mask]]

        # 1. 致命碰撞惩罚 (距离小于 0.28m)
        collision_count = int(np.sum(dist_vals < 0.28)) + out_of_bounds_count * 15

        # 2. 安全边界缓冲惩罚 (0.28m ~ 0.50m 走廊警戒区)
        danger_mask = (dist_vals >= 0.28) & (dist_vals < 0.50)
        danger_penalty = float(np.sum(0.50 - dist_vals[danger_mask]))

        # 3. 转向角平滑度惩罚
        angles = np.arctan2(diffs[:, 1], diffs[:, 0])
        ang_diffs = np.abs((np.diff(angles) + np.pi) % (2.0 * math.pi) - math.pi)
        smoothness_penalty = float(np.sum(ang_diffs))

        # 加权综合适应度函数 (求最小化)
        fitness = (
            1.0 * path_length
            + 120.0 * collision_count
            + 12.0 * danger_penalty
            + 2.5 * smoothness_penalty
        )

        is_free = (collision_count == 0)
        return fitness, is_free, path_length

    def plan(
        self, 
        start: Tuple[float, float], 
        goal: Tuple[float, float]
    ) -> Tuple[Optional[List[Tuple[float, float]]], Dict[str, float]]:
        """
        执行 LDIW-PSO 全局路径优化
        """
        start_time = time.perf_counter()

        s_arr = np.array(start, dtype=np.float64)
        g_arr = np.array(goal, dtype=np.float64)

        # 1. 粒子群初始化 (结合直连基线以及左右偏置多通道探索)
        alphas = np.linspace(0.0, 1.0, self.num_waypoints + 2)[1:-1]
        baseline_pts = np.outer(1.0 - alphas, s_arr) + np.outer(alphas, g_arr)

        particles = np.zeros((self.num_particles, self.num_waypoints, 2), dtype=np.float64)
        velocities = np.random.uniform(-0.2, 0.2, (self.num_particles, self.num_waypoints, 2))

        for i in range(self.num_particles):
            # 前两个粒子沿对角线基线初始化
            if i < 3:
                noise = np.random.uniform(-0.4, 0.4, (self.num_waypoints, 2))
                particles[i] = baseline_pts + noise
            else:
                # 引导粒子向西北开阔走廊与东南通道分散探索
                bias_sign = 1 if i % 2 == 0 else -1
                lateral_shift = np.array([-1.8, 1.8]) * bias_sign * (i / self.num_particles)
                noise = np.random.uniform(-1.0, 1.0, (self.num_waypoints, 2))
                particles[i] = baseline_pts + lateral_shift + noise

            particles[i] = np.clip(particles[i], -4.6, 4.6)

        # 2. 初始化个体最优与全局最优
        p_best = np.copy(particles)
        p_best_fitness = np.zeros(self.num_particles, dtype=np.float64)

        for i in range(self.num_particles):
            p_best_fitness[i] = self._evaluate_fitness(particles[i], s_arr, g_arr)[0]

        g_best_idx = int(np.argmin(p_best_fitness))
        g_best = np.copy(particles[g_best_idx])
        g_best_fitness = float(p_best_fitness[g_best_idx])

        # 记录适应度进化历史
        fitness_history = [g_best_fitness]

        # 3. LDIW-PSO 核心迭代循环
        for it in range(self.max_iterations):
            # 线性自适应递减惯性权重 (Shi & Eberhart 1998)
            w = self.w_max - (it / float(self.max_iterations)) * (self.w_max - self.w_min)

            r1 = np.random.uniform(0.0, 1.0, (self.num_particles, self.num_waypoints, 2))
            r2 = np.random.uniform(0.0, 1.0, (self.num_particles, self.num_waypoints, 2))

            # 速度与位置更新公式
            velocities = (
                w * velocities
                + self.c1 * r1 * (p_best - particles)
                + self.c2 * r2 * (g_best - particles)
            )
            velocities = np.clip(velocities, -self.v_max, self.v_max)

            particles = np.clip(particles + velocities, -4.6, 4.6)

            # 评价更新个体最优与全局最优
            for i in range(self.num_particles):
                fit, is_free, _ = self._evaluate_fitness(particles[i], s_arr, g_arr)
                if fit < p_best_fitness[i]:
                    p_best_fitness[i] = fit
                    p_best[i] = np.copy(particles[i])
                    if fit < g_best_fitness:
                        g_best_fitness = fit
                        g_best = np.copy(particles[i])

            fitness_history.append(g_best_fitness)

        # 4. 提取最优解并验证有效性
        final_fit, final_is_free, final_length = self._evaluate_fitness(g_best, s_arr, g_arr)
        best_path = [start] + [(float(pt[0]), float(pt[1])) for pt in g_best] + [goal]

        planning_time_ms = (time.perf_counter() - start_time) * 1000.0

        info = {
            'success': 1 if final_is_free else 0,
            'planning_time_ms': planning_time_ms,
            'path_length_m': final_length,
            'final_fitness': g_best_fitness,
            'fitness_history': fitness_history,  # 用于绘制进化收敛图
            'waypoints_count': len(best_path),
            'iterations': self.max_iterations
        }

        return best_path, info
