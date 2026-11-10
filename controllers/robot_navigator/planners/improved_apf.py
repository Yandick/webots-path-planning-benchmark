"""
改进人工势场法 (Improved Artificial Potential Field, I-APF) 反应式避障与导航算法

参考文献与学术支撑:
1. Ge, S. S., & Cui, Y. J. (2000). New potential functions for mobile robot path planning.
   IEEE Transactions on Robotics and Automation, 16(5), 615-620.
   - 核心贡献: 引入相对目标距离调节因子 rho^n(q, q_goal)，从数学构造上根除目标不可达 (GNRON) 与目标附近斥力发散问题。
2. Sfeir, R., Saad, M., & Saliah, H. (2011). An improved artificial potential field approach to mobile robot path planning.
   IEEE 4th International Conference on Mechatronics and its Applications, 1-6.
   - 核心贡献: 引入虚拟切向力/旋转力 (Virtual Tangential/Rotational Force)，破坏对峙力的对称平衡，解决局部极小值死锁。
3. Montiel, O., Sepúlveda, R., & Orozco-Rosas, U. (2015). Optimal path planning for mobile robots in environments with 
   static and dynamic obstacles using bacterial potential field. Applied Soft Computing, 37, 234-245.
   - 核心贡献: 迟滞比较器 (Schmitt Trigger Hysteresis) 消除对称鞍点震荡。
"""

import math
import numpy as np
from typing import Tuple, List, Optional
from utils.geometry import wrap_angle, euclidean_dist

class ImprovedAPFPlanner:
    def __init__(
        self,
        k_att: float = 1.2,          # 引力增益系数
        k_rep: float = 1.0,          # 斥力增益系数
        k_tan: float = 2.5,          # 切向环绕逃逸力系数 (Sfeir et al. 2011)
        d_influence: float = 1.4,    # 障碍物斥力影响半径 (m)
        max_speed: float = 0.38,     # 最大巡航线速度 (m/s)
        max_yaw_rate: float = 1.5,   # 最大角速度 (rad/s)
        k_yaw: float = 2.5           # 转向角速度比例增益
    ):
        self.k_att = k_att
        self.k_rep = k_rep
        self.k_tan = k_tan
        self.d_influence = d_influence
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.k_yaw = k_yaw
        self.escape_dir: float = 0.0

    def reset(self):
        """重置内部历史状态"""
        self.escape_dir = 0.0

    def plan(
        self,
        current_pose: Tuple[float, float, float],
        target_point: Tuple[float, float],
        obstacle_points: np.ndarray
    ) -> Tuple[float, float, Tuple[float, float]]:
        """
        I-APF 改进势场力合成与速度指令规划
        
        参数:
            current_pose: 机器人当前位姿 (x, y, theta)
            target_point: 目标点坐标 (gx, gy)
            obstacle_points: 激光雷达检测到的障碍物全局坐标点云 (N, 2)
            
        返回:
            (v, omega, (fx, fy)): 推荐线速度、角速度及当前合力向量
        """
        rx, ry, theta = current_pose
        gx, gy = target_point

        dx_g = gx - rx
        dy_g = gy - ry
        dist_to_goal = math.hypot(dx_g, dy_g)

        if dist_to_goal < 0.28:
            return 0.0, 0.0, (0.0, 0.0)

        # 1. 计算目标引力 (Attractive Force)
        u_gx = dx_g / dist_to_goal
        u_gy = dy_g / dist_to_goal
        f_att_mag = self.k_att * min(dist_to_goal, 3.0)
        f_att_x = f_att_mag * u_gx
        f_att_y = f_att_mag * u_gy

        # 2. 改进斥力与虚拟切向力计算
        f_rep_x = 0.0
        f_rep_y = 0.0
        f_tan_x = 0.0
        f_tan_y = 0.0
        min_d = 999.0
        forward_blocked = False

        if len(obstacle_points) > 0:
            diffs = obstacle_points - np.array([rx, ry])
            dists = np.hypot(diffs[:, 0], diffs[:, 1])
            mask = (dists < self.d_influence) & (dists > 0.05)

            # 前向安全锥检测: 若正前向过近，禁止前冲，强制原地纯旋转
            angles = np.arctan2(diffs[:, 1], diffs[:, 0])
            angle_diffs = np.arctan2(np.sin(angles - theta), np.cos(angles - theta))
            fwd_mask = (np.abs(angle_diffs) < math.radians(45)) & (dists < 0.55)
            if np.any(fwd_mask):
                forward_blocked = True

            if np.any(mask):
                valid_diffs = diffs[mask]
                valid_dists = dists[mask]
                min_d = float(np.min(valid_dists))

                dirs_x = -valid_diffs[:, 0] / valid_dists
                dirs_y = -valid_diffs[:, 1] / valid_dists

                # Ge & Cui (2000) 距离调节因子: 随距离目标越来越近，斥力平滑衰减至零消除 GNRON
                goal_factor = min(dist_to_goal / 3.0, 1.0) ** 2

                inv_d = 1.0 / np.clip(valid_dists, 0.20, self.d_influence)
                inv_d0 = 1.0 / self.d_influence
                weights = (inv_d - inv_d0) ** 2
                total_w = float(np.sum(weights)) + 1e-6

                avg_dx = float(np.sum(weights * dirs_x)) / total_w
                avg_dy = float(np.sum(weights * dirs_y)) / total_w
                norm_d = math.hypot(avg_dx, avg_dy) + 1e-6
                n_rep_x = avg_dx / norm_d
                n_rep_y = avg_dy / norm_d

                raw_rep = self.k_rep * (1.0 / max(0.20, min_d) - inv_d0) * goal_factor * 2.5
                rep_mag = min(raw_rep, f_att_mag * 2.2)
                f_rep_x = rep_mag * n_rep_x
                f_rep_y = rep_mag * n_rep_y

                cos_oppose = -(n_rep_x * u_gx + n_rep_y * u_gy)

                # 3. 虚拟切向力与迟滞锁定 (Sfeir et al. 2011 + Montiel et al. 2015)
                if cos_oppose > 0.10:
                    t1_x, t1_y = -n_rep_y, n_rep_x
                    t2_x, t2_y = n_rep_y, -n_rep_x
                    dot1 = t1_x * u_gx + t1_y * u_gy
                    dot2 = t2_x * u_gx + t2_y * u_gy

                    # 迟滞方向保持锁定打破极限环振荡 (Hysteresis Direction Lock)
                    if self.escape_dir == 0.0:
                        diff_dot = dot1 - dot2
                        if abs(diff_dot) < 0.18:
                            self.escape_dir = -1.0  # 对称相峙时默认偏向开阔西北走廊破局
                        else:
                            self.escape_dir = 1.0 if dot1 >= dot2 else -1.0

                    chosen_dir = self.escape_dir
                    t_best_x = t1_x if chosen_dir == 1.0 else t2_x
                    t_best_y = t1_y if chosen_dir == 1.0 else t2_y

                    tan_factor = (0.7 + 1.3 * cos_oppose)
                    tan_mag = self.k_tan * max(rep_mag, f_att_mag * 0.7) * tan_factor
                    f_tan_x = tan_mag * t_best_x
                    f_tan_y = tan_mag * t_best_y
                else:
                    self.escape_dir = 0.0

        # 4. 合成总势场力
        f_total_x = f_att_x + f_rep_x + f_tan_x
        f_total_y = f_att_y + f_rep_y + f_tan_y
        f_total_mag = math.hypot(f_total_x, f_total_y)

        if f_total_mag < 1e-3:
            return 0.0, 0.0, (0.0, 0.0)

        # 5. 差动轮机器人非完整运动学映射
        desired_heading = math.atan2(f_total_y, f_total_x)
        heading_error = wrap_angle(desired_heading - theta)
        omega = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.k_yaw * heading_error))

        # 线速度控制: 优先原地转向与前向障碍物安全抑制
        cos_err = max(0.0, math.cos(heading_error))
        v = self.max_speed * (cos_err ** 2.0)

        # 前向受阻时严禁前冲撞击，允许原地纯旋转调整航向
        if forward_blocked:
            v = 0.0
        elif dist_to_goal > 0.35:
            v = max(0.10, v)

        if dist_to_goal < 0.8:
            v *= (dist_to_goal / 0.8)

        v = min(self.max_speed, v)

        return v, omega, (f_total_x, f_total_y)
