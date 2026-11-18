# 移动机器人路径规划课设

A* 和 RRT* 在路被堵住时都会重新规划。可是在窄路里，机器人即使及时刹住，也可能正好停在小车横穿的位置。本课设给两种算法加上提前让行，并在三张公开地图上对照改动前后的实际轨迹。[实验主页与同步回放](https://yandick.github.io/webots-path-planning-benchmark/)

| 地图 | 固定障碍 | 小车怎样运动 |
| --- | --- | --- |
| 四柱场地 · TurtleBot3 Stage 2 | 方形围墙和四根圆柱 | 从中间两根柱子之间横穿，途中短暂停留 |
| U 形走廊 · PointMaze U-Maze | 两条由隔墙分开的走廊，在右侧相通 | 横过下方走廊，稍后折返 |
| 岔路迷宫 · PointMaze Medium | 带支路和转角的迷宫 | 沿中部通道往返，经过交叉口 |

四柱场地保留 ROBOTIS 原地图的米制尺寸。两张 PointMaze 保留原始墙格，每格换算为 1.4 m。起终点和三辆小车各自的路线由本课设设置。[地图来源、布局与许可证](assets/maps/README.md)

![三张地图的起点、终点、固定障碍和小车路线](results/map.png)

## 实验结果

每张图跑五个种子。每一对原版和改进版共用起终点、初始路线、小车时间表和速度限制。机器人只从激光观测估计来车速度。如果两车可能相遇，会比较停车和继续通过这两种情况下的距离，在来车经过之前选择更安全的动作。

| 二维仿真到达次数 | A* 原版 | A* 提前让行 | RRT* 原版 | RRT* 提前让行 |
| --- | ---: | ---: | ---: | ---: |
| 四柱场地 | 1/5 | 5/5 | 4/5 | 5/5 |
| U 形走廊 | 1/5 | 5/5 | 3/5 | 5/5 |
| 岔路迷宫 | 5/5 | 5/5 | 3/5 | 5/5 |

四柱场地种子 0 中，原版 A* 两次重新规划，仍在 5.19 s 撞上小车。改进版在 1.80 s 开始减速让行，5.60 s 恢复行驶，14.50 s 到达。让行也有时间代价。四柱场地种子 4 中，两版 A* 都到达，改进版多用了 5.00 s。[实验报告](docs/coursework_path_planning_report.md)

![两对方法在三张地图上的到达次数和耗时](results/benchmark.png)

另有 Voronoi、G²VD Planner、APF 和 DWA 四个参考方法。G²VD Planner 采用 Wen 等人论文中的方法名，项目复现其第 IV-D 节的二次规划平滑，输入为本项目的几何 Voronoi 路线。八种方法共跑 120 次二维仿真，Webots R2025a 在同三张地图上用种子 0 另跑 24 次。[原论文](https://arxiv.org/abs/2201.12981) · [二维逐次结果](results/summary.csv) · [Webots 记录](results/webots_validation.json)

## 本地运行

```bash
python -m pip install -r requirements.txt
python scripts/generate_worlds.py
python evaluation/benchmark.py --workers 4
python evaluation/plot_results.py --video
python scripts/build_site.py
python -m http.server 8000 --directory dist
```

单独试一张图时，可以把输出写入另一个目录。下面这条命令只跑种子 0 的 U 形走廊。

```bash
python evaluation/benchmark.py --scenes pointmaze_u --methods astar astar_yield rrt_star rrt_star_yield --seeds 0 --output build/quick-run
```

在 Webots R2025a 中打开 `worlds/turtlebot3_stage2.wbt`、`worlds/pointmaze_u.wbt` 或 `worlds/pointmaze_medium.wbt`。默认运行 A* 提前让行版。用 `config.json` 或环境变量 `PLANNER_ALGO` 选择方法，G²VD Planner 沿用配置名 `voronoi_qp`。批量运行要求 Webots 能调用已安装项目依赖的 Python，Linux 无显示器时使用 Xvfb。

```bash
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py --export results
python -m unittest discover -s tests -v
```

[Webots 验证摘要](results/webots_validation.json) · [三图同步录像](results/course_maps.mp4) · [网站构建与发布](docs/website.md)
