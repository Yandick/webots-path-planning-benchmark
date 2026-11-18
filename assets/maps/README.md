# 三张课设地图

[course_maps.json](course_maps.json) 保存三张图的固定障碍、起终点和小车路线。二维仿真、Webots 世界和网页回放都从这里读地图。

| 地图 | 公开布局 | 本项目的用法 |
| --- | --- | --- |
| 四柱场地 | [ROBOTIS TurtleBot3 DQN Stage 2 世界](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/a35a56c8b04877dc89772b598084d8ce648a9023/turtlebot3_gazebo/worlds/turtlebot3_dqn_stage2.world) | 读取围墙和四根圆柱的碰撞尺寸与位置，保留原始米制比例。围墙外缘为 5 × 5 m，圆柱半径均为 0.15 m。 |
| U 形走廊 | [Farama PointMaze 的 U_MAZE](https://github.com/Farama-Foundation/Gymnasium-Robotics/blob/91f8a92916a0ca0c6032549c9964d8f4b0f68450/gymnasium_robotics/envs/maze/maps.py#L48-L54) | 完整保留 5 × 5 格布局，每格取 1.4 m。 |
| 岔路迷宫 | [Farama PointMaze 的 MEDIUM_MAZE](https://github.com/Farama-Foundation/Gymnasium-Robotics/blob/91f8a92916a0ca0c6032549c9964d8f4b0f68450/gymnasium_robotics/envs/maze/maps.py#L56-L65) | 完整保留 8 × 8 格布局，每格取 1.4 m。 |

两张迷宫图的第一行在北侧，列从西向东排列，数字 1 表示墙。相邻墙格组成矩形墙体。每张图的起终点、移动小车和行驶时刻由本课设设置。小车按一串时间、横坐标、纵坐标做分段匀速运动。

ROBOTIS 的地图使用 Apache 2.0 许可。Farama 的地图文件注明源于 Justin Fu 的 D4RL 并使用 Apache 2.0，其项目同时附有 Farama Foundation 的 MIT 许可。这里保留两份许可证。[Apache 2.0](LICENSE-Apache-2.0.txt) · [Farama MIT](LICENSE-Farama.txt)
