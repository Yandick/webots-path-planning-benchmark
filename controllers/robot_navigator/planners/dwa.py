"""
动态窗口法 (Dynamic Window Approach, DWA) 局部动态避障与轨迹跟踪算法
采用 NumPy 全向量化轨迹预测与评估，保证实时性与高动态避障响应。
参考文献:
Fox, D., Burgard, W., & Thrun, S. (1997). The dynamic window approach to collision avoidance.
IEEE Robotics & Automation Magazine, 4(1), 23-33.
"""

import math
import numpy as np
from typing import Tuple, List, Optional
from utils.geometry import wrap_angle, euclidean_dist

class DWAConfig:
    """DWA 参数配置类"""
    def __init__(self):
        # 机器人动力学限制
        self.max_speed = 0.45       # 最大线速度 (m/s)
        self.min_speed = 0.0        # 最小线速度 (不支持后退以保证安全性)
        self.max_yaw_rate = 1.8     # 最大角速度 (rad/s)
        self.max_accel = 1.0        # 最大线加速度 (m/s^2)
        self.max_dyaw_rate = 3.5    # 最大角加速度 (rad/s^2)

        # 采样配置
        self.n_v = 8                # 线速度采样数
        self.n_omega = 17           # 角速度采样数

        # 预测时间范围与仿真时间步长
        self.predict_time = 1.0     # 轨迹向前预测时间 (s)
        self.dt = 0.1               # 预测推演步长 (s)

        # 评价函数归一化权重 (Normalized Weights)
        self.weight_heading = 0.35   # 航向角朝向目标权重
        self.weight_speed = 0.30     # 前进速度权重 (鼓励积极前行避免冰冻陷阱)
        self.weight_clearance = 0.20 # 障碍物安全距离权重
        self.weight_path = 0.15      # 偏离目标点距离惩罚权重

        # 几何尺寸与安全包络
        self.robot_radius = 0.18     # 机器人碰撞外接圆半径 (m)
        self.safety_margin = 0.04    # 额外安全边界裕度 (m)


class DWAPlanner:
    def __init__(self, config: Optional[DWAConfig] = None):
        self.cfg = config if config is not None else DWAConfig()
        self.times = np.arange(self.cfg.dt, self.cfg.predict_time + self.cfg.dt / 2.0, self.cfg.dt)

    def plan(
        self,
        current_pose: Tuple[float, float, float],
        current_vel: Tuple[float, float],
        target_point: Tuple[float, float],
        obstacle_points: np.ndarray,
        control_dt: float = 0.032
    ) -> Tuple[float, float, List[Tuple[float, float]]]:
        """
        DWA 局部速度规划 (全向量化加速版)
        
        参数:
            current_pose: 机器人当前位姿 (x, y, theta)
            current_vel: 机器人当前速度 (v, omega)
            target_point: 局部跟踪目标点 (通常为全局路径上的前视引导点)
            obstacle_points: 当前时刻由激光雷达扫描到的障碍物全局坐标点云 (N, 2)
            control_dt: 控制周期 (s)
            
        返回:
            (best_v, best_omega, best_traj): 推荐线速度、角速度与对应预测轨迹
        """
        x0, y0, theta0 = current_pose
        v, omega = current_vel

        # 1. 过滤周围有效障碍物 (仅保留前方与侧方 2.5m 内的点云，排除车体正后方点云干扰前向推演)
        if len(obstacle_points) > 0:
            dx_obs = obstacle_points[:, 0] - x0
            dy_obs = obstacle_points[:, 1] - y0
            d_sq = dx_obs ** 2 + dy_obs ** 2
            # 投影到机器人车体局部纵向坐标轴 x_local (正前方为正)
            x_local = dx_obs * math.cos(theta0) + dy_obs * math.sin(theta0)
            valid_obs_mask = (d_sq <= (2.5 ** 2)) & (x_local >= -0.10)
            obs = obstacle_points[valid_obs_mask]
        else:
            obs = np.empty((0, 2))

        # 2. 计算动态窗口 (Dynamic Window)
        window_dt = max(control_dt, 0.25)
        v_min_reach = max(self.cfg.min_speed, v - self.cfg.max_accel * window_dt)
        v_max_reach = min(self.cfg.max_speed, v + self.cfg.max_accel * window_dt)

        omega_min_reach = max(-self.cfg.max_yaw_rate, omega - self.cfg.max_dyaw_rate * window_dt)
        omega_max_reach = min(self.cfg.max_yaw_rate, omega + self.cfg.max_dyaw_rate * window_dt)

        # 网格采样速度空间
        v_samples = np.linspace(v_min_reach, v_max_reach, self.cfg.n_v)
        omega_samples = np.linspace(omega_min_reach, omega_max_reach, self.cfg.n_omega)
        V, W = np.meshgrid(v_samples, omega_samples)
        V = V.flatten()
        W = W.flatten()
        num_candidates = len(V)

        # 3. 向量化前向推演全部候选轨迹
        thetas = theta0 + np.outer(W, self.times)  # (N_cand, N_steps)
        dx = V[:, None] * np.cos(thetas) * self.cfg.dt
        dy = V[:, None] * np.sin(thetas) * self.cfg.dt
        xs = x0 + np.cumsum(dx, axis=1)  # (N_cand, N_steps)
        ys = y0 + np.cumsum(dy, axis=1)

        collision_thresh = self.cfg.robot_radius + self.cfg.safety_margin

        # 4. 向量化计算与障碍物的最近距离
        if len(obs) > 0:
            diff_x = xs[:, :, None] - obs[:, 0]  # (N_cand, N_steps, N_obs)
            diff_y = ys[:, :, None] - obs[:, 1]
            dist_sq = diff_x ** 2 + diff_y ** 2
            min_dist = np.sqrt(np.min(dist_sq, axis=(1, 2)))
        else:
            min_dist = np.full(num_candidates, 3.0)

        valid_mask = min_dist >= collision_thresh

        # 若全部候选轨迹均发生碰撞，执行原地减速避障
        if not np.any(valid_mask):
            return 0.0, 0.6, [(x0, y0, theta0)]

        # 5. 多目标评价指标计算
        target_angle = np.arctan2(target_point[1] - ys[:, -1], target_point[0] - xs[:, -1])
        diff_angle = np.abs((target_angle - thetas[:, -1] + np.pi) % (2.0 * np.pi) - np.pi)
        heading_scores = np.pi - diff_angle

        # 障碍物距离饱和处理：超过安全阈值 0.5m 即可获得充足安全得分，避免因微小距离差异导致驻留停滞
        clearance_scores = np.minimum(min_dist, 0.50)
        speed_scores = V
        path_scores = -np.hypot(target_point[0] - xs[:, -1], target_point[1] - ys[:, -1])

        # 归一化函数
        def normalize_score(arr):
            v_arr = arr[valid_mask]
            span = v_arr.max() - v_arr.min()
            if span < 1e-6:
                return np.ones_like(arr)
            return (arr - v_arr.min()) / span

        norm_heading = normalize_score(heading_scores)
        norm_clearance = normalize_score(clearance_scores)
        norm_speed = normalize_score(speed_scores)
        norm_path = normalize_score(path_scores)

        total_scores = (
            self.cfg.weight_heading * norm_heading
            + self.cfg.weight_clearance * norm_clearance
            + self.cfg.weight_speed * norm_speed
            + self.cfg.weight_path * norm_path
        )
        total_scores[~valid_mask] = -1e9

        best_idx = int(np.argmax(total_scores))
        best_v = float(V[best_idx])
        best_omega = float(W[best_idx])

        # 提取最优轨迹点用于可视化
        best_traj = [(float(xs[best_idx, k]), float(ys[best_idx, k]), float(thetas[best_idx, k])) for k in range(len(self.times))]

        return best_v, best_omega, best_traj
