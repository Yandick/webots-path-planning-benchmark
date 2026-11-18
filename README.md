# 移动机器人路径规划课设

六种方法在三张公开地图上驾驶同一台差速机器人。机器人知道墙和圆柱的位置，通过激光发现移动的小车。主页可以切换地图、方法和种子，也可以暂停在相遇前，看机器人接下来往哪边走。[实验主页](https://yandick.github.io/webots-path-planning-benchmark/)

| 地图 | 固定障碍 | 小车怎样运动 |
| --- | --- | --- |
| 四柱场地 · TurtleBot3 Stage 2 | 方形围墙和四根圆柱 | 从中间两根柱子之间横穿，途中短暂停留 |
| U 形走廊 · PointMaze U-Maze | 两条由隔墙分开的走廊，在右侧相通 | 横过下方走廊，稍后折返 |
| 岔路迷宫 · PointMaze Medium | 带支路和转角的迷宫 | 沿中部通道往返，经过交叉口 |

四柱场地保留 ROBOTIS 原地图的米制尺寸。两张 PointMaze 保留原始墙格，每格换算为 1.4 m。起终点和三辆小车各自的路线由本课设设置。[地图来源、布局与许可证](assets/maps/README.md)

![三张地图的起点、终点、固定障碍和小车路线](results/map.png)

## 实验结果

每张图有 6 种方法，每种方法跑 5 个种子，共 90 次二维仿真。Webots R2025a 在同三张图上分别验证 6 种方法，使用种子 0，共 18 次。两种仿真分别统计。

| 地图 | 二维到达 | 二维碰撞 | 二维超时 | Webots 到达 |
| --- | ---: | ---: | ---: | ---: |
| 四柱场地 | 23/30 | 4 | 3 | 5/6 |
| U 形走廊 | 22/30 | 8 | 0 | 6/6 |
| 岔路迷宫 | 22/30 | 0 | 8 | 4/6 |

A*、RRT*、Voronoi 和 Dev1ce's method 共用一个路线跟踪器。APF 和 DWA 使用 A* 引导，所以也能与运行 A* 时的跟踪器对照。Dev1ce's method 对 Voronoi 参考路线作安全气泡约束的 QP 平滑，配置名为 `voronoi_qp`。[算法与逐图结果](docs/coursework_path_planning_report.md) · [逐次运行记录](results/summary.csv)

![六种方法在三张地图上的实验结果](results/benchmark.png)

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
python evaluation/benchmark.py --scenes pointmaze_u --methods astar voronoi_qp --seeds 0 --output build/quick-run
```

在 Webots R2025a 中打开 `worlds/turtlebot3_stage2.wbt`、`worlds/pointmaze_u.wbt` 或 `worlds/pointmaze_medium.wbt`。用 `config.json` 或环境变量 `PLANNER_ALGO` 选择方法。批量运行要求 Webots 能调用已安装项目依赖的 Python，Linux 无显示器时使用 Xvfb。

```bash
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py --export results
python -m unittest discover -s tests -v
```

[Webots 验证摘要](results/webots_validation.json) · [三图同步录像](results/course_maps.mp4) · [网站构建与发布](docs/website.md)
