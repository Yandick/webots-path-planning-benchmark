# Voronoi 安全路径平滑：方法与答辩说明

## 1. 来源与改进点

采用 Jian Wen、Xuebo Zhang、Qingchen Bi、Hui Liu、Jing Yuan、Yongchun Fang 的 **G²VD Planner: Efficient Motion Planning With Grid-Based Generalized Voronoi Diagrams**，IEEE Transactions on Automation Science and Engineering，2024 年在线发表。[DOI](https://doi.org/10.1109/TASE.2024.3398996) · [全文](https://arxiv.org/html/2201.12981v4)

本项目复现第 IV-D 节、式 (5)–(8) 的安全气泡 QP 平滑方法，输入为几何 Voronoi 骨架路线，并采用第 IV-E 节的三次样条插值。课设方法名为“Voronoi＋安全 QP”，实现位于 `navigation/smoothing.py`。

## 2. Motivation

Voronoi 骨架位于自由空间中部，保留了较大的障碍间距。现有 baseline 用贪心直线简化，从当前点跳到最远的可见后续点。这能缩短路线，但折线可能贴近货架角落；跟踪器提前转向时还会进一步减小实际净空。

本课设以安全间距、转弯平顺性为主目标。改进保留安全中间路线，再在有限范围内平滑它，同时处理“转弯要顺”和“不能为了拉直而靠近障碍”。

## 3. 具体步骤及原因

1. **共同配置空间。** 机器人半径 0.18 m，加上 0.14 m 规划余量，得到相同的 0.32 m 障碍膨胀半径。
2. **安全参考路线。** 障碍边界构图，过滤不通行节点和边，接入起终点，用 Dijkstra 搜索未经贪心简化的 Voronoi 路线，保留中间路线的障碍间距。
3. **均匀取点。** 按 0.10 m 弧长采样，避免原始节点疏密影响优化。
4. **安全气泡。** 每个参考点有一个可移动的正方形区域；越靠近障碍，可移动范围越小，起终点固定。
5. **二次规划。** 同时减少相邻线段方向突变，并限制偏离安全参考线。
6. **三次样条。** 按 0.04 m 间隔离散插值曲线，逐条检查全部线段。若插值不通行，使用检查通过的 QP 折线；两者均不通行则记录规划失败。
7. **共同执行。** 四种全局方法使用同一跟踪器，按 0.08 m 弧长重新采样，采用 8 个间隔的前视点，保证前视距离不受输入节点疏密影响。

目标为

\[
\min_{p_i}\quad w_s\sum_i\|p_{i+1}-2p_i+p_{i-1}\|^2+w_r\sum_i\|p_i-r_i\|^2.
\]

\(r_i\) 是参考点，\(p_i\) 是优化点。第一项把突变折角分散成渐进转弯，第二项保留安全路线的障碍间距。所有点还必须在各自的气泡内。

根据论文式 (8)，气泡半宽为

\[
b_i=\max\left(0,\frac{d_i}{\sqrt2}-r_{\mathrm{inflate}}\right).
\]

本实现从栅格距离变换读取距离后，减去半个栅格对角线，保守转换为连续参考点处的距离下界。\(r_{\mathrm{inflate}}=0.32\,m\)，与其他全局算法相同。

使用 SciPy L-BFGS-B 求解带盒约束的凸二次问题。本项目固定 \(w_s=1000,w_r=1\)，强化骨架折角的平滑；论文原实验为 10 和 1。参数在 seed 0 开发测试后固定，再运行全部五个 seed。

![安全气泡与路线](../results/voronoi_smoothing.png)

## 4. 一张地图、五个 baseline

仓库为 10 m × 10 m，起点 (-4,-3.5)、终点 (4,3.5)，有七组货架、中央短通道和上方通道。装卸小车在 seed 0 的 t=12..58 s 占用中央通道，巡行小车沿上方出货区运动。两车最高速度不超过 0.4 m/s，时序预先给定，与机器人和算法身份无关。

物理地图由墙、静态货架和动态小车组成。激光点是物体表面的测量。机器人已知静态地图，动态物体由当前扫描感知；公共执行层根据扫描减速、制动。独立仿真器使用真实几何检查碰撞。每 0.05 s 检查小车是否与静态物体或另一辆车相交。

| baseline | 实现 |
| --- | --- |
| A* | 0.10 m 栅格、八邻域、禁止穿角、直线简化，公共跟踪器执行 |
| RRT* | 1800 次采样、0.4 m 步长、15% 目标采样、父节点选择和重连，公共跟踪器执行 |
| Voronoi | 障碍边界约 0.12 m 采样、骨架搜索、贪心直线简化，公共跟踪器执行 |
| APF | 引力系数 1.2、斥力系数 0.8、作用距离 1.2 m，直接输出速度 |
| DWA | 7×19 组速度候选、1.5 s 时域、0.15 s 预测步长，直接朝终点导航 |

四种全局方法不调用 DWA，APF、DWA 不调用全局规划。共同速度上限 0.45 m/s、角速度上限 1.5 rad/s，线加速度 0.8 m/s²、角加速度 3 rad/s²，并限制左右轮速。

## 5. 实验与指标

同一地图、五个时序 seed、六种方法，共 30 次。seed 0 无偏移，其余 seed 为 [-1,1] s 内预先采样的动作时间偏移；RRT* 使用对应固定采样种子。

二维后端使用 180 条射线的 360° 首交点扫描，最远 5 m，包含遮挡。控制周期 0.10 s，碰撞检查子步 0.01 s。100 s 内进入终点 0.25 m 范围且此前无碰撞计为到达；首次碰撞终止运行。

- **最低实际净空：** 整次运行中机器人外轮廓到静态或动态障碍表面的最小距离，主表再对五次运行取平均。
- **规划峰值曲率：** 规划路径均按 0.10 m 弧长重采样，计算三点外接圆曲率，单位 m⁻¹。
- **运动中指令角加速度 RMS：** 角速度指令差分除以控制周期，只对 |v|>0.02 m/s 的样本计算 RMS，避免停车稀释指标。
- **耗时与行程：** 成功运行单独计算平均值；时间图将失败记为 100 s。

[逐次结果](../results/summary.csv) · [实验清单](../results/manifest.json) · [完整路线与轨迹](../results/raw_runs.json.gz)

## 6. 正式二维结果

| 方法 | 到达 | 成功平均耗时 | 成功平均行程 | 碰撞 | 超时 |
| --- | --- | --- | --- | --- | --- |
| A* | 4/5 | 74.38 s | 11.582 m | 1 | 0 |
| RRT* | 4/5 | 74.78 s | 11.657 m | 1 | 0 |
| Voronoi | 5/5 | 29.10 s | 12.372 m | 0 | 0 |
| APF | 0/5 | — | — | 0 | 5 |
| DWA | 0/5 | — | — | 0 | 5 |
| 安全 QP | 5/5 | 30.80 s | 13.171 m | 0 | 0 |

| 指标（五次平均） | Voronoi | 安全 QP | 变化 |
| --- | --- | --- | --- |
| 最低实际净空 | 0.13158 m | 0.21048 m | 增加 59.96% |
| 规划峰值曲率 | 10.57364 m⁻¹ | 1.35288 m⁻¹ | 降低 87.21% |
| 运动中指令角加速度 RMS | 0.62968 rad/s² | 0.55004 rad/s² | 降低 12.65% |
| 耗时 | 29.10 s | 30.80 s | 增加 1.70 s |
| 实际行程 | 12.37171 m | 13.17102 m | 增加 0.79931 m |
| 初次规划开销（本机） | 89.9 ms | 138.4 ms | 增加 48.5 ms |

规划折线长度为 12.86579 m 和 13.57914 m。安全 QP 保留更大的转弯空间，因此路线增长；公共控制器下，实际净空与运动中的转向变化均得到改善。

![主要指标](../results/safety_smoothness.png)
![六方法结果](../results/benchmark.png)
![六方法轨迹](../results/warehouse.png)

## 7. 机制消融

seed 0 额外运行一次“保留原始安全参考、关闭 QP 与样条”，控制器完全相同。[完整消融轨迹](../results/smoothing_ablation.json.gz)

| seed 0 配置 | 最低实际净空 | 规划峰值曲率 | 运动中指令角加速度 RMS | 耗时 |
| --- | --- | --- | --- | --- |
| Voronoi 贪心简化 | 0.13158 m | 10.57364 m⁻¹ | 0.63075 rad/s² | 29.0 s |
| 原始安全参考，关闭平滑 | 0.19906 m | 7.91943 m⁻¹ | 0.63815 rad/s² | 31.7 s |
| 安全参考＋QP＋样条 | 0.21064 m | 1.35288 m⁻¹ | 0.55004 rad/s² | 30.8 s |

保留中间路线主要恢复安全间距；QP 与样条进一步把折角变为连续转弯，使运动中的角加速度下降，实际行程也从 13.501 m 降至 13.171 m。

关闭平滑的配置可用以下 Python 代码复现：

```python
from unittest.mock import patch
from navigation.classical import voronoi_route
from navigation.scenarios import make_scene
from navigation.simulation import run

with patch("navigation.core.voronoi_qp_path", voronoi_route):
    result = run(make_scene(seed=0), "voronoi_qp")
print({k: result[k] for k in ("finish_time", "min_surface_clearance",
                             "planned_peak_curvature", "moving_angular_accel_rms")})
```

## 8. Webots 与复现

原生仿真采用相同地图、控制代码与 seed 0。四个 90° 雷达扇区、每扇区 90 条射线构成 360° 扫描；导航周期 96 ms，Supervisor 每 32 ms 检查几何净空。每次核对方法、地图摘要和两个控制器的新日志。

[本次 GitHub Actions](https://github.com/Yandick/webots-path-planning-benchmark/actions/runs/43815885972)已通过全部六方法原生检查。

| 方法（seed 0） | 状态 | 到达/终止时间 | 实际行程 | 最低实际净空 |
| --- | --- | --- | --- | --- |
| A* | success | 74.784 s | 11.586 m | 0.11344 m |
| RRT* | success | 74.528 s | 11.572 m | 0.11108 m |
| Voronoi | success | 29.280 s | 12.370 m | 0.13130 m |
| APF | timeout | 100.000 s | 4.967 m | 0.34415 m |
| DWA | timeout | 100.000 s | 1.512 m | 0.11126 m |
| 安全 QP | success | 31.168 s | 13.176 m | 0.20967 m |

安全 QP 的最低实际净空增加 59.68%，运动中指令角加速度 RMS 从 0.63725 降至 0.54613 rad/s²，降低 14.30%；到达时间增加 1.888 s。规划峰值曲率为 10.57364 与 1.35288 m⁻¹。

本次原生运行的初次规划开销为 Voronoi 29.0 ms、安全 QP 944.6 ms，该步骤在启动时执行一次。原生雷达最近点的最大几何误差为 0.01159 m。

[原生汇总及来源](../results/webots_validation.json)记录源代码提交、CI、artifact 摘要、传感器配置和截图来源；[完整原生日志](../results/webots_raw_runs.json.gz)保存六种方法的控制轨迹、监督器轨迹和初始路线。

![Webots 原生截图，t=28s](../results/webots_scene.png)

```bash
python -m pip install -r requirements.txt
python scripts/generate_worlds.py
python -m unittest discover -s tests -v
python evaluation/benchmark.py --seeds 0 1 2 3 4
python evaluation/plot_results.py --video
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py
```

## 9. 答辩顺序

先展示贪心直线简化在拐角处压缩净空的现象，再展示安全参考、气泡约束和 QP 的两个目标项。用放大图说明路线如何逐渐转弯，然后展示正式结果与关闭平滑的消融。最后说明多用约 1.7 s 和 0.8 m 行程换来的净空与平顺性收益。
