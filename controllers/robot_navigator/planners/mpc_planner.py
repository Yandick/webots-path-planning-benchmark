"""
模型预测控制 (Model Predictive Control, MPC) 局部轨迹跟踪与动态避障规划器
参考开源规范: PythonRobotics (Atsushi Sakai et al.) & 经典移动机器人非线性 MPC 架构。

数学原理与算法架构:
1. 机器人一阶非完整约束差动运动学模型 (Unicycle Kinematics with Midpoint Integration)
2. 全局路径高密插值 (Path Densification, ds = 0.08m) 与动态参考视界提取 (Receding Horizon, N = 8, dt = 0.10s, T = 0.8s)
3. 偏航角自适应协调转弯与时域优化解耦 (Coordinated Heading Alignment)
4. 多目标凸二次/非线性优化代价函数:
   J = J_pos (参考轨迹位置跟踪) + J_heading (朝向角对齐) + J_v (名义巡航速度激励)
       + J_omega (旋转角速度惩罚) + J_smooth (控制量一阶变化率平滑)
       + J_obstacle (动态/静态障碍物人工屏障函数 Barrier Penalty)
5. L-BFGS-B 极速有界优化求解与热启动 (Warm-Start) 加速
"""

import math
import numpy as np
from typing import Tuple, List, Optional
from scipy.optimize import minimize
from utils.geometry import wrap_angle, euclidean_dist


class MPCConfig:
    """MPC 参数配置类"""
    def __init__(self):
        # 优化时域配置
        self.N = 8                  # 预测时域步数 (Prediction Horizon Steps)
        self.dt = 0.10              # 预测离散时间步长 (s), 总预测时域 T = 0.8s
        
        # 机器人动力学物理约束
        self.max_speed = 0.45       # 最大线速度 (m/s)
        self.min_speed = 0.0        # 最小线速度 (不支持后退以规避盲区)
        self.max_yaw_rate = 1.8     # 最大角速度 (rad/s)
        self.max_accel = 1.2        # 最大线加速度 (m/s^2)
        self.max_dyaw_rate = 3.5    # 最大角加速度 (rad/s^2)
        
        # 几何尺寸与安全包络
        self.robot_radius = 0.18    # 机器人碰撞外半径 (m)
        self.safety_margin = 0.06   # 动态避障额外安全包络 (m) -> 绝对硬安全半径 = 0.24m
        self.warn_dist = 0.38       # 障碍物软预警影响半径 (m)
        
        # 目标函数加权系数 (Cost Function Weights)
        self.w_pos = 18.0           # 参考路径位置跟踪误差权重
        self.w_heading = 5.0        # 航向角误差权重
        self.w_v = 4.0              # 保持名义前进速度权重
        self.w_omega = 1.0          # 角速度绝对值惩罚 (鼓励顺畅行驶)
        self.w_dv = 1.5             # 线加速度平滑惩罚
        self.w_domega = 1.0         # 角加速度平滑惩罚
        self.w_obs = 35.0           # 障碍物排斥屏障权重
        
        # 标称巡航速度
        self.nominal_speed = 0.40   # 期望巡航线速度 (m/s)


class MPCPlanner:
    def __init__(self, config: Optional[MPCConfig] = None):
        self.cfg = config if config is not None else MPCConfig()
        # 热启动记忆序列: (N, 2) -> (v, omega)
        self.u_prev = np.zeros((self.cfg.N, 2), dtype=np.float64)
        self.u_prev[:, 0] = self.cfg.nominal_speed * 0.6
        
        # 密集路径缓存
        self._raw_path_hash = None
        self._dense_path = None
        
    def _densify_path(self, path: List[Tuple[float, float]], step: float = 0.08) -> np.ndarray:
        """
        将全局稀疏关键拐点路径高密度插值为连续路网 (步长 ~0.08m)
        """
        path_tuple = tuple(path)
        if self._raw_path_hash == hash(path_tuple) and self._dense_path is not None:
            return self._dense_path
            
        dense = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i], dtype=np.float64)
            p2 = np.array(path[i + 1], dtype=np.float64)
            seg_len = float(np.linalg.norm(p2 - p1))
            num_pts = max(1, int(np.ceil(seg_len / step)))
            for s in np.linspace(0.0, 1.0, num_pts, endpoint=False):
                dense.append(p1 + s * (p2 - p1))
        dense.append(np.array(path[-1], dtype=np.float64))
        
        self._dense_path = np.array(dense, dtype=np.float64)
        self._raw_path_hash = hash(path_tuple)
        return self._dense_path

    def _kinematics_rollout(
        self,
        x0: float,
        y0: float,
        theta0: float,
        u_seq: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        基于中点欧拉法 (Midpoint Integration) 积分推演 N 步预测轨迹
        返回: (x_traj, y_traj, theta_traj)，长度均为 N+1
        """
        N = self.cfg.N
        dt = self.cfg.dt
        
        x = np.zeros(N + 1, dtype=np.float64)
        y = np.zeros(N + 1, dtype=np.float64)
        theta = np.zeros(N + 1, dtype=np.float64)
        
        x[0] = x0
        y[0] = y0
        theta[0] = theta0
        
        for k in range(N):
            v_k = u_seq[k, 0]
            w_k = u_seq[k, 1]
            
            mid_theta = theta[k] + 0.5 * w_k * dt
            x[k + 1] = x[k] + v_k * math.cos(mid_theta) * dt
            y[k + 1] = y[k] + v_k * math.sin(mid_theta) * dt
            theta[k + 1] = theta[k] + w_k * dt
            
        return x, y, theta

    def _extract_reference_trajectory(
        self,
        current_pose: Tuple[float, float, float],
        dense_path: np.ndarray
    ) -> Tuple[np.ndarray, int]:
        """
        从高密全局路径中提取未来 N+1 个时域参考位姿 [x_ref, y_ref, theta_ref]
        """
        x0, y0, _ = current_pose
        N = self.cfg.N
        
        # 1. 寻找当前位置在高密路径上的投影最近点
        dists = np.hypot(dense_path[:, 0] - x0, dense_path[:, 1] - y0)
        closest_idx = int(np.argmin(dists))
        
        # 2. 沿路径向前抽取 N+1 个参考点
        # 步幅 stride 对应 nominal_speed * dt / step_size ≈ 0.40 * 0.1 / 0.08 = 0.5 -> 取 1 点/步
        stride = 1
        ref_points = np.zeros((N + 1, 3), dtype=np.float64)
        
        for k in range(N + 1):
            idx = min(closest_idx + k * stride, len(dense_path) - 1)
            next_idx = min(idx + 1, len(dense_path) - 1)
            if next_idx > idx:
                th = math.atan2(dense_path[next_idx, 1] - dense_path[idx, 1], dense_path[next_idx, 0] - dense_path[idx, 0])
            else:
                th = ref_points[max(0, k - 1), 2]
            ref_points[k] = [dense_path[idx, 0], dense_path[idx, 1], th]
            
        return ref_points, closest_idx

    def plan(
        self,
        current_pose: Tuple[float, float, float],
        current_vel: Tuple[float, float],
        global_path: List[Tuple[float, float]],
        obstacle_points: np.ndarray,
        control_dt: float = 0.032
    ) -> Tuple[float, float, List[Tuple[float, float]]]:
        """
        MPC 滚动时域优化单步求解
        
        参数:
            current_pose: (x, y, theta)
            current_vel: (v, omega)
            global_path: 全局参考路径点集
            obstacle_points: 当前时刻由激光雷达扫描到的障碍物全局坐标点云 (M, 2)
            control_dt: 控制周期 (s)
            
        返回:
            (best_v, best_omega, pred_trajectory)
        """
        x0, y0, theta0 = current_pose
        v_curr, w_curr = current_vel
        N = self.cfg.N
        
        # 1. 过滤周围有效障碍物 (仅考虑 2.5m 内局部避障范围)
        if len(obstacle_points) > 0:
            d_sq = (obstacle_points[:, 0] - x0) ** 2 + (obstacle_points[:, 1] - y0) ** 2
            nearby_obs = obstacle_points[d_sq < (2.5 ** 2)]
        else:
            nearby_obs = np.empty((0, 2))
            
        # 2. 高密路径提取未来参考视界
        dense_path = self._densify_path(global_path, step=0.08)
        ref_traj, closest_idx = self._extract_reference_trajectory(current_pose, dense_path)
        
        # 3. 目标航向角自适应协调
        # 如果当前航向与第一视界参考航向偏差超过 30 度 (0.52 rad)，执行差动转向优先策略快速纠偏
        target_heading = ref_traj[1, 2]
        heading_err = wrap_angle(target_heading - theta0)
        
        # 终端吸附检查
        goal_x, goal_y = global_path[-1]
        dist_to_final_goal = math.hypot(goal_x - x0, goal_y - y0)
        
        if abs(heading_err) > 0.50 and v_curr < 0.15:
            # 原地/低速协调对准，防止非完整约束轮式车辆侧移激增
            cmd_omega = float(np.clip(2.5 * heading_err, -self.cfg.max_yaw_rate, self.cfg.max_yaw_rate))
            cmd_v = float(0.08 * max(0.0, math.cos(heading_err)))
            pred_traj = [(x0, y0), (x0 + cmd_v * math.cos(theta0) * 0.1, y0 + cmd_v * math.sin(theta0) * 0.1)]
            return cmd_v, cmd_omega, pred_traj

        # 4. 构建多目标二次代价函数
        safe_radius = self.cfg.robot_radius + self.cfg.safety_margin
        warn_dist = self.cfg.warn_dist
        
        # 接近终点时平滑降低标称期望速度
        target_v = self.cfg.nominal_speed
        if dist_to_final_goal < 1.0:
            target_v = max(0.12, self.cfg.nominal_speed * (dist_to_final_goal / 1.0))
            
        def objective(u_flat: np.ndarray) -> float:
            u_seq = u_flat.reshape((N, 2))
            x_pred, y_pred, theta_pred = self._kinematics_rollout(x0, y0, theta0, u_seq)
            
            # (1) 参考轨迹位置跟踪误差
            pos_err = np.sum((x_pred[1:] - ref_traj[1:, 0]) ** 2 + (y_pred[1:] - ref_traj[1:, 1]) ** 2)
            cost = self.cfg.w_pos * pos_err
            
            # (2) 航向角朝向对齐误差
            theta_diff = (theta_pred[1:] - ref_traj[1:, 2] + math.pi) % (2 * math.pi) - math.pi
            cost += self.cfg.w_heading * np.sum(theta_diff ** 2)
            
            # (3) 标称巡航速度鼓励与角速度惩罚
            cost += self.cfg.w_v * np.sum((u_seq[:, 0] - target_v) ** 2)
            cost += self.cfg.w_omega * np.sum(u_seq[:, 1] ** 2)
            
            # (4) 控制量一阶平滑变化惩罚 (Rate of Change)
            dv_0 = u_seq[0, 0] - v_curr
            dw_0 = u_seq[0, 1] - w_curr
            cost += self.cfg.w_dv * (dv_0 ** 2) + self.cfg.w_domega * (dw_0 ** 2)
            if N > 1:
                cost += self.cfg.w_dv * np.sum(np.diff(u_seq[:, 0]) ** 2)
                cost += self.cfg.w_domega * np.sum(np.diff(u_seq[:, 1]) ** 2)
                
            # (5) 动态与静态障碍物人工屏障函数 (Barrier Penalty)
            if len(nearby_obs) > 0:
                dx_mat = x_pred[1:, np.newaxis] - nearby_obs[:, 0]
                dy_mat = y_pred[1:, np.newaxis] - nearby_obs[:, 1]
                dist_mat = np.sqrt(dx_mat ** 2 + dy_mat ** 2)
                
                # 硬安全边界强惩罚 (阻止进入侵入半径)
                hard_violations = np.maximum(0.0, safe_radius - dist_mat)
                cost += self.cfg.w_obs * 60.0 * np.sum(hard_violations ** 2)
                
                # 软缓冲区渐进排斥惩罚
                soft_violations = np.maximum(0.0, warn_dist - dist_mat)
                cost += self.cfg.w_obs * np.sum(soft_violations ** 2)
                
            return float(cost)

        # 5. 优化变量上下界配置
        bounds = []
        for i in range(N):
            bounds.append((self.cfg.min_speed, self.cfg.max_speed))
            bounds.append((-self.cfg.max_yaw_rate, self.cfg.max_yaw_rate))
            
        # 6. 热启动序列初始化 (将上一时刻解向后平移 1 步)
        u_init = np.zeros((N, 2), dtype=np.float64)
        u_init[:-1] = self.u_prev[1:]
        u_init[-1] = self.u_prev[-1]
        
        u_flat_init = u_init.flatten()
        for i in range(2 * N):
            u_flat_init[i] = np.clip(u_flat_init[i], bounds[i][0], bounds[i][1])
            
        # 7. 调用 L-BFGS-B 进行实时有界数值优化
        res = minimize(
            objective,
            u_flat_init,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 25, 'ftol': 1e-4, 'disp': False}
        )
        
        if res.success or res.fun < 1e5:
            u_opt = res.x.reshape((N, 2))
        else:
            u_opt = u_init
            
        # 更新记忆热启动解
        self.u_prev = u_opt
        
        best_v = float(u_opt[0, 0])
        best_omega = float(u_opt[0, 1])
        
        # 推演最优轨迹用于可视化与调试
        x_pred, y_pred, _ = self._kinematics_rollout(x0, y0, theta0, u_opt)
        pred_traj = list(zip(x_pred.tolist(), y_pred.tolist()))
        
        return best_v, best_omega, pred_traj
