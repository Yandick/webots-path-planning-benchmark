"""
人工势场法 (Artificial Potential Field, APF) 局部避障与导航算法
参考文献:
Khatib, O. (1986). Real-time obstacle avoidance for manipulators and mobile robots.
The International Journal of Robotics Research, 5(1), 90-98.
"""

import math
import numpy as np
from typing import Tuple, List, Optional
from utils.geometry import wrap_angle, euclidean_dist

class APFPlanner:
    def __init__(
        self,
        k_att: float = 1.2,          # 引力系数
        k_rep: float = 0.8,          # 斥力系数
        d_influence: float = 1.2,    # 障碍物斥力影响半径 (m)
        max_speed: float = 0.40,     # 最大线速度 (m/s)
        max_yaw_rate: float = 1.5,   # 最大角速度 (rad/s)
        k_yaw: float = 2.0           # 转向角速度增益系数
    ):
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_influence = d_influence
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.k_yaw = k_yaw

    def plan(
        self,
        current_pose: Tuple[float, float, float],
        target_point: Tuple[float, float],
        obstacle_points: np.ndarray
    ) -> Tuple[float, float, Tuple[float, float]]:
        """
        APF 合力与速度指令规划
        
        参数:
            current_pose: 机器人当前位姿 (x, y, theta)
            target_point: 目标点坐标 (gx, gy)
            obstacle_points: 激光雷达检测到的障碍物全局坐标点云 (N, 2)
            
        返回:
            (v, omega, (fx, fy)): 推荐线速度、角速度及当前合力向量
        """
        rx, ry, theta = current_pose
        gx, gy = target_point

        # 1. 计算引力向量 (Attractive Force)
        dx_g = gx - rx
        dy_g = gy - ry
        dist_to_goal = math.hypot(dx_g, dy_g)

        if dist_to_goal > 1e-4:
            f_att_x = self.k_att * (dx_g / dist_to_goal) * min(dist_to_goal, 2.0)
            f_att_y = self.k_att * (dy_g / dist_to_goal) * min(dist_to_goal, 2.0)
        else:
            return 0.0, 0.0, (0.0, 0.0)

        # 2. 计算斥力向量 (Repulsive Force)
        f_rep_x = 0.0
        f_rep_y = 0.0

        if len(obstacle_points) > 0:
            # 过滤影响半径内的障碍物点
            diffs = obstacle_points - np.array([rx, ry])
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            mask = (dists < self.d_influence) & (dists > 0.05)

            if np.any(mask):
                valid_diffs = diffs[mask]
                valid_dists = dists[mask]

                # 斥力公式: F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * (-diff/d)
                inv_d = 1.0 / valid_dists
                inv_d0 = 1.0 / self.d_influence
                rep_mag = self.k_rep * (inv_d - inv_d0) * (inv_d ** 2)

                # 方向远离障碍物: -diff / dist
                dirs_x = -valid_diffs[:, 0] / valid_dists
                dirs_y = -valid_diffs[:, 1] / valid_dists

                f_rep_x = np.sum(rep_mag * dirs_x)
                f_rep_y = np.sum(rep_mag * dirs_y)

        # 3. 计算合力向量 (Total Force)
        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y
        f_total_mag = math.hypot(f_total_x, f_total_y)

        # 4. 映射到非完整差动移动机器人控制量 (v, omega)
        if f_total_mag < 1e-3:
            return 0.0, 0.0, (0.0, 0.0)

        desired_heading = math.atan2(f_total_y, f_total_x)
        heading_error = wrap_angle(desired_heading - theta)

        # 转向比例控制
        omega = self.k_yaw * heading_error
        omega = max(-self.max_yaw_rate, min(self.max_yaw_rate, omega))

        # 前向速度控制: 角度偏差越大，线速度减小以优先转向
        cos_err = max(0.0, math.cos(heading_error))
        v = self.max_speed * cos_err

        # 减速区检测：当接近目标时平滑减速
        if dist_to_goal < 0.8:
            v *= (dist_to_goal / 0.8)

        v = max(0.0, min(self.max_speed, v))

        return v, omega, (f_total_x, f_total_y)
