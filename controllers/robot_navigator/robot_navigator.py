"""
移动机器人核心导航控制器 (Robot Navigator Main Controller)
集成全局规划 (A* / RRT*)、局部动态避障 (DWA / APF) 以及传感器数据融合与轨迹记录。
"""

import os
import sys

# 确保当前控制器目录加入 Python 模块搜索路径
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)

import json
import math
import time
import numpy as np
from controller import Robot

# 导入工具与算法模块
from utils.geometry import wrap_angle, euclidean_dist, diff_drive_ik, lidar_to_global_points
from utils.grid_map import GridMap
from planners.a_star import AStarPlanner
from planners.rrt_star import RRTStarPlanner
from planners.informed_rrt_star import InformedRRTStarPlanner
from planners.dwa import DWAPlanner, DWAConfig
from planners.apf import APFPlanner
from planners.improved_apf import ImprovedAPFPlanner
from planners.voronoi_planner import VoronoiPlanner
from planners.pso_planner import PSOPlanner
from planners.mpc_planner import MPCPlanner, MPCConfig

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0  # 控制周期 (s)

    # 1. 硬件执行器与传感器初始化
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    iu = robot.getDevice("inertial_unit")
    iu.enable(timestep)

    compass = robot.getDevice("compass")
    compass.enable(timestep)

    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)

    # 预热传感器
    for _ in range(3):
        robot.step(timestep)

    # 2. 读取配置与算法选择
    # 支持环境变量 PLANNER_ALGO 或 config.json: 'a_star_dwa', 'rrt_star_dwa', 'apf'
    algo = os.environ.get("PLANNER_ALGO", "a_star_dwa").lower()
    config_path = os.path.join(os.path.dirname(__file__), "..", "..", "config.json")
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg_data = json.load(f)
                algo = os.environ.get("PLANNER_ALGO", cfg_data.get("algorithm", algo)).lower()
        except Exception as e:
            print(f"[Navigator] 警告: 读取 config.json 失败: {e}")

    print(f"==================================================")
    print(f"  [Robot Navigator] 初始化完成")
    print(f"  选定导航规划算法: {algo.upper()}")
    print(f"  控制周期: {timestep} ms (dt = {dt:.3f} s)")
    print(f"==================================================")

    # 3. 规划问题定义: 起点与目标点
    start_pos = (-4.0, -4.0)
    goal_pos = (4.0, 4.0)
    goal_tolerance = 0.30  # 到达判定半径 (m)

    # 初始化全局栅格地图 (设置充裕的膨胀安全边距，使全局路径远离障碍物棱角)
    grid_map = GridMap(
        min_x=-5.0, max_x=5.0,
        min_y=-5.0, max_y=5.0,
        resolution=0.05,
        robot_radius=0.20,
        safety_margin=0.20
    )

    # 4. 全局规划器初始化与离线计算
    global_path = None
    plan_info = {}

    if algo in ["a_star_dwa", "a_star"]:
        print("[Navigator] 正在执行 A* 全局路径规划...")
        planner = AStarPlanner(grid_map, clearance_weight=0.4)
        global_path, plan_info = planner.plan(start_pos, goal_pos)
    elif algo in ["rrt_star_dwa", "rrt_star"]:
        print("[Navigator] 正在执行经典 RRT* 全局路径规划...")
        planner = RRTStarPlanner(grid_map, max_iterations=1200, step_size=0.40, goal_sample_rate=0.15)
        global_path, plan_info = planner.plan(start_pos, goal_pos)
    elif algo in ["informed_rrt_star_mpc", "irrt_star_mpc", "informed_rrt_star_dwa", "informed_rrt_star", "irrt_star"]:
        print("[Navigator] 正在执行改进型 Informed Bi-RRT* + B-Spline SOTA 全局规划...")
        planner = InformedRRTStarPlanner(grid_map, max_iterations=900, step_size=0.40)
        global_path, plan_info = planner.plan(start_pos, goal_pos)
    elif algo in ["voronoi_dwa", "voronoi"]:
        print("[Navigator] 正在执行广义沃罗诺伊图 (Voronoi/GVD) 全局中轴骨架规划...")
        planner = VoronoiPlanner(grid_map, min_clearance=0.35)
        global_path, plan_info = planner.plan(start_pos, goal_pos)
    elif algo in ["pso_dwa", "pso"]:
        print("[Navigator] 正在执行自适应惯性权重粒子群优化算法 (LDIW-PSO) 规划...")
        planner = PSOPlanner(grid_map, num_particles=50, max_iterations=90, num_waypoints=5)
        global_path, plan_info = planner.plan(start_pos, goal_pos)
    elif algo in ["mpc", "a_star_mpc"]:
        print("[Navigator] 正在执行 A* 全局拓扑引导 + 模型预测控制 (MPC) 规划...")
        planner = AStarPlanner(grid_map, clearance_weight=0.4)
        global_path, plan_info = planner.plan(start_pos, goal_pos)
    elif algo == "apf":
        print("[Navigator] 采用纯反应式人工势场法 (APF)，无需先验全局路径。")
    elif algo in ["improved_apf", "iapf", "iapf_dwa", "apf_dwa"]:
        print("[Navigator] 采用改进人工势场法 (I-APF, Ge & Cui 2000 + 虚拟切向力逃逸)，无需先验全局路径。")
    else:
        print(f"[Navigator] 未知算法 {algo}，默认回退至 A*+DWA")
        planner = AStarPlanner(grid_map)
        global_path, plan_info = planner.plan(start_pos, goal_pos)
        algo = "a_star_dwa"

    if global_path is not None:
        print(f"[Navigator] 全局规划成功! 耗时: {plan_info.get('planning_time_ms', 0):.2f} ms, "
              f"路径长度: {plan_info.get('path_length_m', 0):.2f} m, 路标点数: {len(global_path)}")
    elif algo not in ["apf", "improved_apf", "iapf", "iapf_dwa", "apf_dwa"]:
        print("[Navigator] 全局规划未能找到路径，退出！")
        return

    # 5. 局部动态避障规划器初始化
    dwa_planner = DWAPlanner()
    apf_planner = APFPlanner(k_att=1.2, k_rep=0.8, d_influence=1.5, max_speed=0.42)
    improved_apf_planner = ImprovedAPFPlanner(k_att=1.2, k_rep=1.0, k_tan=2.5, d_influence=1.4, max_speed=0.38)
    mpc_planner = MPCPlanner()

    # 6. 控制主循环状态变量
    current_wp_idx = 0
    current_v = 0.0
    current_omega = 0.0
    sim_time = 0.0
    max_sim_time = 75.0  # 超时保护 (s) (设置略低于 supervisor 的 80s 以便自主优雅落盘)
    step_count = 0

    # 实验数据采集记录
    trajectory_log = {
        'algorithm': algo,
        'start': start_pos,
        'goal': goal_pos,
        'global_path': global_path,
        'global_plan_info': plan_info,
        'trajectory': [],  # list of dict: [t, x, y, theta, v, omega, min_dist]
        'success': False,
        'finish_time': 0.0,
        'actual_path_length': 0.0,
        'min_clearance': float('inf'),
        'collision_count': 0
    }

    prev_x, prev_y = None, None
    total_travel_distance = 0.0
    min_overall_clearance = float('inf')

    print("[Navigator] 进入自主导航主控循环...")

    while robot.step(timestep) != -1:
        sim_time += dt

        # 读取传感器位姿
        gps_vals = gps.getValues()
        rx, ry = gps_vals[0], gps_vals[1]

        # 从 InertialUnit 读取偏航角 Yaw (rad)
        rpy = iu.getRollPitchYaw()
        r_theta = rpy[2]  # ENU 坐标系下的平面航向角

        if prev_x is not None:
            step_dist = math.hypot(rx - prev_x, ry - prev_y)
            total_travel_distance += step_dist
        prev_x, prev_y = rx, ry

        # 读取 2D 激光雷达点云 (设置 min_range 过滤车体自身结构反射)
        range_image = lidar.getRangeImage()
        fov = lidar.getFov()
        obs_points = lidar_to_global_points(range_image, fov, rx, ry, r_theta, min_range=0.22, max_range=5.0)

        # 实时计算当前离障碍物的最近物理距离
        current_min_obs_dist = 5.0
        if len(obs_points) > 0:
            dists = np.hypot(obs_points[:, 0] - rx, obs_points[:, 1] - ry)
            current_min_obs_dist = float(dists.min())
            if current_min_obs_dist < min_overall_clearance:
                min_overall_clearance = current_min_obs_dist

        trajectory_log['finish_time'] = round(sim_time, 2)
        trajectory_log['actual_path_length'] = round(total_travel_distance, 2)
        trajectory_log['min_clearance'] = round(min_overall_clearance, 2)

        # 检查是否到达终点
        dist_to_goal = math.hypot(goal_pos[0] - rx, goal_pos[1] - ry)
        if dist_to_goal <= goal_tolerance:
            print(f"[Navigator] 成功到达目标终点! 耗时: {sim_time:.2f} s, 行驶里程: {total_travel_distance:.2f} m")
            trajectory_log['success'] = True
            trajectory_log['finish_time'] = sim_time
            trajectory_log['actual_path_length'] = total_travel_distance
            trajectory_log['min_clearance'] = min_overall_clearance
            break

        if sim_time >= max_sim_time:
            print(f"[Navigator] 导航运行超时 ({max_sim_time} s)，结束任务。")
            trajectory_log['success'] = False
            trajectory_log['finish_time'] = sim_time
            trajectory_log['actual_path_length'] = total_travel_distance
            trajectory_log['min_clearance'] = min_overall_clearance
            break

        step_count += 1
        # 局部规划速度生成 (以约 15 Hz 频率周期性更新局部速度指令以保证控制效率)
        if step_count % 2 == 0 or current_v < 0.05:
            if algo in ["a_star_dwa", "rrt_star_dwa", "voronoi_dwa", "pso_dwa", "voronoi", "pso", "informed_rrt_star_dwa", "informed_rrt_star", "irrt_star"]:
                # 路径前视跟踪与路标点动态更新 (Adaptive Waypoint Tracking)
                while current_wp_idx < len(global_path) - 1:
                    wp = global_path[current_wp_idx]
                    if math.hypot(wp[0] - rx, wp[1] - ry) < 0.45:
                        current_wp_idx += 1
                    else:
                        break

                # 选取前视目标点 (Lookahead Point)
                lookahead_dist = 0.85
                target_pt = global_path[-1]
                for i in range(current_wp_idx, len(global_path)):
                    wp = global_path[i]
                    if math.hypot(wp[0] - rx, wp[1] - ry) >= lookahead_dist:
                        target_pt = wp
                        break

                # 调用 DWA 进行局部轨迹推演与避障
                cmd_v, cmd_omega, best_traj = dwa_planner.plan(
                    current_pose=(rx, ry, r_theta),
                    current_vel=(current_v, current_omega),
                    target_point=target_pt,
                    obstacle_points=obs_points,
                    control_dt=dt * 2
                )

            elif algo == "apf":
                # 纯反应式人工势场导航
                cmd_v, cmd_omega, force = apf_planner.plan(
                    current_pose=(rx, ry, r_theta),
                    target_point=goal_pos,
                    obstacle_points=obs_points
                )

            elif algo in ["improved_apf", "iapf"]:
                # 纯反应式改进人工势场法 (I-APF) 导航
                cmd_v, cmd_omega, force = improved_apf_planner.plan(
                    current_pose=(rx, ry, r_theta),
                    target_point=goal_pos,
                    obstacle_points=obs_points
                )

            elif algo in ["mpc", "a_star_mpc", "informed_rrt_star_mpc", "irrt_star_mpc"]:
                # 前沿模型预测控制 (MPC) 时域滚动优化与动态避障
                cmd_v, cmd_omega, best_traj = mpc_planner.plan(
                    current_pose=(rx, ry, r_theta),
                    current_vel=(current_v, current_omega),
                    global_path=global_path,
                    obstacle_points=obs_points,
                    control_dt=dt * 2
                )

            elif algo in ["iapf_dwa", "apf_dwa"]:
                # 改进人工势场引导 DWA 动态避障 (I-APF + DWA)
                _, _, force = improved_apf_planner.plan(
                    current_pose=(rx, ry, r_theta),
                    target_point=goal_pos,
                    obstacle_points=obs_points
                )
                f_norm = math.hypot(force[0], force[1])
                if f_norm > 1e-3:
                    target_pt = (rx + 1.2 * (force[0] / f_norm), ry + 1.2 * (force[1] / f_norm))
                else:
                    target_pt = goal_pos
                cmd_v, cmd_omega, best_traj = dwa_planner.plan(
                    current_pose=(rx, ry, r_theta),
                    current_vel=(current_v, current_omega),
                    target_point=target_pt,
                    obstacle_points=obs_points,
                    control_dt=dt * 2
                )

            current_v = cmd_v
            current_omega = cmd_omega

        if step_count % 75 == 0:
            print(f"[Navigator t={sim_time:4.1f}s] pos=({rx:5.2f}, {ry:5.2f}), v={current_v:.2f} m/s, dist_to_goal={dist_to_goal:.2f} m, min_clearance={current_min_obs_dist:.2f} m", flush=True)

        # 差动轮逆运动学计算并下发轮速
        w_left, w_right = diff_drive_ik(cmd_v, cmd_omega, wheel_radius=0.05, wheel_base=0.30, max_wheel_speed=10.0)
        left_motor.setVelocity(w_left)
        right_motor.setVelocity(w_right)

        # 记录关键运行轨迹数据
        trajectory_log['trajectory'].append({
            't': round(sim_time, 3),
            'x': round(rx, 4),
            'y': round(ry, 4),
            'theta': round(r_theta, 4),
            'v': round(cmd_v, 4),
            'omega': round(cmd_omega, 4),
            'dist_to_goal': round(dist_to_goal, 4),
            'clearance': round(current_min_obs_dist, 4)
        })

    # 停止机器人电机
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # 保存实验日志文件
    results_dir = os.path.join(os.path.dirname(__file__), "..", "..", "results")
    os.makedirs(results_dir, exist_ok=True)
    out_file = os.path.join(results_dir, f"trajectory_{algo}.json")
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(trajectory_log, f, indent=2, ensure_ascii=False)
    print(f"[Navigator] 实验轨迹数据已保存至: {out_file}")

if __name__ == "__main__":
    main()
