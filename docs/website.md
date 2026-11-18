# 主页构建与发布

[项目主页](https://yandick.github.io/webots-path-planning-benchmark/) 由 GitHub Pages 发布。网页源码在 `site/`，静态构建输出在 `dist/`。

## 本地预览

```bash
python -m pip install -r requirements.txt
python scripts/build_site.py
python -m http.server 8000 --directory dist
```

打开 <http://localhost:8000/>。首页的三个地图按钮会切换固定障碍、小车轨迹、机器人回放、结果表和 Webots 世界文件名。方法和种子选择器读取当前地图下的运行。网页用表格区分到达、碰撞、超时和规划失败。

## 更新地图或实验

公开布局和小车时间表位于 [地图目录](../assets/maps/README.md)。选择新的起终点或修改小车路线后，在项目根目录重新生成世界和结果。

```bash
python scripts/generate_worlds.py
python evaluation/benchmark.py --workers 4
python evaluation/plot_results.py --video
python evaluation/webots_benchmark.py
python evaluation/check_webots_smoke.py --export results
python -m unittest discover -s tests -v
node --check site/app.js
python scripts/build_site.py
```

网页构建读取 `results/manifest.json`、汇总表和真实轨迹。每条运行的地图、种子、几何与小车时序都要与场景签名相符。Webots 单独归档并校验。网页构建会给应用脚本、样式和数据文件加上与内容对应的缓存版本。[构建脚本](../scripts/build_site.py)

## GitHub Pages

`main` 分支上的推送会触发 Project website 工作流。构建作业运行测试并生成 `dist/`，发布作业将其部署到 Pages。首次部署到新仓库时，在 **Settings / Pages / Build and deployment** 选择 **GitHub Actions**。Pull Request 会运行网站检查。[项目发布记录](https://github.com/Yandick/webots-path-planning-benchmark/actions/workflows/pages.yml)

Navigation checks 另行运行三图的 Webots R2025a 检查，在工作流产物中保存完整控制器日志和验证摘要。[原生验证工作流](../.github/workflows/navigation.yml)
