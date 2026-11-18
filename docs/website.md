# 项目主页与 GitHub Pages 发布

项目主页用于浏览导航方法、已有实验记录、轨迹和结果文件。网页源文件位于 `site/`，构建入口为 `scripts/build_site.py`，可发布文件输出到 `dist/`。

启用 GitHub Pages 并成功部署后，项目主页地址为：

**<https://yandick.github.io/webots-path-planning-benchmark/>**

该地址是项目网页地址；`https://github.com/Yandick/webots-path-planning-benchmark` 继续用于浏览源码和提交记录。上面的主页地址需要完成首次启用和部署后才可访问。

## 本地构建与预览

在仓库根目录运行：

```bash
python -m pip install -r requirements.txt
python scripts/build_site.py
python -m http.server 8000 --directory dist
```

在浏览器打开 <http://localhost:8000/>。静态网页本身不需要 Python 服务端；这里的 HTTP 服务仅用于本地预览。构建阶段读取已有实验数据并生成网页数据文件，不会重新运行二维基准或 Webots 仿真。页面中的轨迹演示是已记录实验的回放。

构建输出中的 `data.js` 由脚本生成，其他页面资源和选定结果文件随构建一起复制到 `dist/`。修改网站内容时编辑 `site/`；修改数据处理方式时编辑 `scripts/build_site.py`，然后重新构建。不要把 `dist/` 当作手工维护的源文件。

发布前可运行：

```bash
python -m unittest discover -s tests -v
node --check site/app.js
python scripts/build_site.py
```

JavaScript 语法检查需要 Node.js。本地仅构建和预览静态页面时不需要 Node.js；GitHub 托管的 Ubuntu runner 提供此工具。

## 首次启用 GitHub Pages

仓库需要先启用 GitHub Pages。具备相应仓库管理权限的用户可按以下步骤配置：

1. 打开 [仓库的 Pages 设置](https://github.com/Yandick/webots-path-planning-benchmark/settings/pages)。
2. 在 **Build and deployment → Source** 中选择 **GitHub Actions**。
3. 确认包含主页代码和 `.github/workflows/pages.yml` 的改动已合入 `main`。
4. 打开 [Project website 工作流](https://github.com/Yandick/webots-path-planning-benchmark/actions/workflows/pages.yml)。合入 `main` 会自动触发；如之前因尚未启用 Pages 而失败，可在启用后重新运行，或选择 **Run workflow → main**。
5. 等待 **Validate and build website** 与 **Publish GitHub Pages** 成功，再打开工作流给出的部署链接。

仅添加工作流文件不能代替首次启用 Pages。工作流采用默认的 `GITHUB_TOKEN`；`actions/configure-pages` 的自动启用选项需要额外权限和另一种凭据，因此这里不使用该选项，也不要求把个人访问令牌写入仓库。

部署成功后，可以在仓库主页右侧 **About → Website** 中填写实际部署地址，方便访问者从仓库进入网页。对外分享项目时直接使用网页地址即可。

## 自动验证与发布

`.github/workflows/pages.yml` 对 Pull Request 执行 Python 测试、浏览器 JavaScript 语法检查和静态构建。Pull Request 不部署到正式主页。

对 `main` 的推送以及在 `main` 上手动运行工作流，会在检查通过后上传 `dist/` 并部署。部署使用 `github-pages` environment，权限限定为 Pages 写入和部署所需的 OIDC token。源码读取权限仅提供给构建作业。

原有 `Navigation checks` 继续检查算法和控制器代码。网站源文件、网站构建脚本和 Pages 工作流的单独修改不会触发原生 Webots 仿真；网页工作流仍执行本地 Python 测试。涉及实际导航代码的改动继续触发原有验证。

## 方法名称与历史数据兼容

网页和文档使用 **Dev1ce's method** 作为方法展示名称。稳定的配置值、历史实验数据中的方法键和代码入口仍使用 `voronoi_qp`，例如：

```bash
PLANNER_ALGO=voronoi_qp bash run_simulation.sh
```

因此，已有配置和归档结果可以继续使用。展示名称从 `navigation/settings.py` 的 `METHOD_LABELS` 获取；`voronoi_qp` 不应被当作页面上给访客展示的名称。关于 Voronoi 参考路线、气泡约束 QP 和 Wen 等人的论文来源，仍在方法说明中保留，以便读者理解实现和复现实验。

## 官方参考

- [配置 GitHub Pages 的发布来源](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site)
- [使用自定义 GitHub Pages 工作流](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages)
- [configure-pages 的配置项与首次启用权限](https://github.com/actions/configure-pages/blob/main/action.yml)
