"""
几何与运动学工具模块 (Geometry & Kinematics Utilities)
包含角度处理、差动轮运动学反解、激光雷达局部坐标到全局坐标变换等。
"""

import math
import numpy as np
from typing import Tuple, List

def wrap_angle(angle: float) -> float:
    """将角度归一化至 [-pi, pi] 区间"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def euclidean_dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """计算两点间的欧几里得距离"""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def diff_drive_ik(
    v: float, 
    omega: float, 
    wheel_radius: float = 0.05, 
    wheel_base: float = 0.30, 
    max_wheel_speed: float = 10.0
) -> Tuple[float, float]:
    """
    两轮差动移动机器人运动学逆解 (Inverse Kinematics)
    由期望线速度 v 和角速度 omega 计算左右轮角速度 (rad/s)
    
    参数:
        v: 期望线速度 (m/s)
        omega: 期望角速度 (rad/s)
        wheel_radius: 车轮半径 r (m)
        wheel_base: 两轮轮距 b (m)
        max_wheel_speed: 电机最大允许角速度 (rad/s)
        
    返回:
        (w_left, w_right): 左右轮角速度 (rad/s)
    """
    # 差动轮运动学逆解标准公式:
    # v_L = v - omega * b / 2
    # v_R = v + omega * b / 2
    # w_L = v_L / r, w_R = v_R / r
    w_left = (v - (omega * wheel_base / 2.0)) / wheel_radius
    w_right = (v + (omega * wheel_base / 2.0)) / wheel_radius
    
    # 考虑电机最大转速限幅，等比例缩放保持曲率不变
    max_val = max(abs(w_left), abs(w_right))
    if max_val > max_wheel_speed and max_val > 1e-5:
        scale = max_wheel_speed / max_val
        w_left *= scale
        w_right *= scale
        
    return w_left, w_right

def diff_drive_fk(
    w_left: float, 
    w_right: float, 
    wheel_radius: float = 0.05, 
    wheel_base: float = 0.30
) -> Tuple[float, float]:
    """两轮差动移动机器人正运动学求解 (Forward Kinematics)"""
    v_left = w_left * wheel_radius
    v_right = w_right * wheel_radius
    v = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / wheel_base
    return v, omega

def lidar_to_global_points(
    ranges: List[float],
    fov: float,
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    min_range: float = 0.08,
    max_range: float = 5.0
) -> np.ndarray:
    """
    将 2D 激光雷达扫描的极坐标距离数组转换为全局笛卡尔坐标点云
    
    参数:
        ranges: 激光测距列表
        fov: 视场角 (通常为 2*pi)
        robot_x, robot_y, robot_theta: 机器人当前全局位姿
        min_range, max_range: 有效测量距离门限
        
    返回:
        points: (N, 2) 形状的全局坐标点数组
    """
    n = len(ranges)
    if n == 0:
        return np.empty((0, 2))
        
    angle_step = fov / n
    points = []
    
    # Webots Lidar 扫描角通常从 -fov/2 顺时针/逆时针递增
    for i, r in enumerate(ranges):
        if min_range < r < max_range and not math.isinf(r) and not math.isnan(r):
            # 局部扫描角度
            local_angle = -fov / 2.0 + i * angle_step
            # 全局角度
            global_angle = robot_theta + local_angle
            gx = robot_x + r * math.cos(global_angle)
            gy = robot_y + r * math.sin(global_angle)
            points.append([gx, gy])
            
    if not points:
        return np.empty((0, 2))
    return np.array(points, dtype=np.float64)
