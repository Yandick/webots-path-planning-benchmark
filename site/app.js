/* Archived trajectories only: this page does not simulate or alter benchmark results. */
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const data = window.BENCHMARK_DATA;
  const menuButton = document.querySelector(".menu-toggle");
  const nav = $("main-nav");
  document.documentElement.classList.add("has-menu");
  menuButton.hidden = false;
  function showMenu(open) {
    menuButton.setAttribute("aria-expanded", String(open));
    menuButton.setAttribute("aria-label", open ? "收起导航" : "展开导航");
    nav.classList.toggle("open", open);
  }
  menuButton.addEventListener("click", function () {
    showMenu(this.getAttribute("aria-expanded") !== "true");
  });
  document.querySelectorAll("#main-nav a").forEach((link) => {
    link.addEventListener("click", () => showMenu(false));
  });
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && nav.classList.contains("open")) {
      showMenu(false);
      menuButton.focus();
    }
  });

  $("copy-commands").hidden = false;
  $("copy-commands").addEventListener("click", async () => {
    const code = $("quickstart-code");
    try {
      if (!navigator.clipboard || !window.isSecureContext) throw new Error("Clipboard unavailable");
      await navigator.clipboard.writeText(code.textContent);
      $("copy-feedback").textContent = "已复制命令。";
    } catch (_) {
      const range = document.createRange();
      range.selectNodeContents(code);
      const selection = window.getSelection();
      selection.removeAllRanges();
      selection.addRange(range);
      $("copy-feedback").textContent = "已选中命令，可以用系统的复制功能拷贝。";
    }
  });

  if (!data || data.schema_version !== 1 || !Array.isArray(data.runs) || !data.runs.length) {
    $("data-error").hidden = false;
    return;
  }
  document.documentElement.classList.add("has-data");

  const methodById = new Map(data.methods.map((method) => [method.id, method]));
  const summaryById = new Map(data.summary.map((summary) => [summary.method, summary]));
  const runsByKey = new Map(data.runs.map((run) => [run.method + ":" + run.seed, run]));
  const state = {
    method: "voronoi_qp", seed: data.seeds[0], time: 0, playing: false,
    speed: 2, view: "iso", group: "global", metric: "clearance", previousFrame: null
  };
  const canvas = $("trajectory-canvas");
  const ctx = canvas.getContext("2d");
  if (ctx) document.documentElement.classList.add("has-replay");
  const motionPreference = window.matchMedia("(prefers-reduced-motion: reduce)");
  let playbackRequested = !motionPreference.matches;
  let viewerVisible = !window.IntersectionObserver;
  let frameRequest = null;
  let canvasWidth = 1;
  let canvasHeight = 1;

  function label(id, localGroup) {
    if (localGroup && id === "astar") return "公共方向跟踪器";
    return methodById.get(id).label.replace(" controller", "");
  }
  function fixed(value, digits) {
    return value === null || value === undefined || !Number.isFinite(value) ? "—" : value.toFixed(digits);
  }
  function run() {
    return runsByKey.get(state.method + ":" + state.seed);
  }
  function element(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    return node;
  }

  /* Binary search plus interpolation preserves the recorded clock. */
  function sampleAt(trajectory, time) {
    const first = trajectory[0];
    if (time <= first[0]) {
      const ratio = first[0] > 0 ? Math.max(0, time / first[0]) : 1;
      return [time, data.scene.start[0] + ratio * (first[1] - data.scene.start[0]),
        data.scene.start[1] + ratio * (first[2] - data.scene.start[1]), first[3], first[4]];
    }
    const last = trajectory[trajectory.length - 1];
    if (time >= last[0]) return last.slice();
    let low = 0;
    let high = trajectory.length - 1;
    while (high - low > 1) {
      const middle = Math.floor((low + high) / 2);
      if (trajectory[middle][0] <= time) low = middle;
      else high = middle;
    }
    const a = trajectory[low];
    const b = trajectory[high];
    const ratio = (time - a[0]) / (b[0] - a[0]);
    const angleDifference = Math.atan2(Math.sin(b[3] - a[3]), Math.cos(b[3] - a[3]));
    return [time, a[1] + (b[1] - a[1]) * ratio, a[2] + (b[2] - a[2]) * ratio,
      a[3] + angleDifference * ratio, a[4] + (b[4] - a[4]) * ratio];
  }

  function cartPosition(waypoints, time) {
    if (time <= waypoints[0][0]) return waypoints[0].slice(1);
    for (let i = 1; i < waypoints.length; i += 1) {
      if (time <= waypoints[i][0]) {
        const a = waypoints[i - 1];
        const b = waypoints[i];
        const ratio = (time - a[0]) / (b[0] - a[0]);
        return [a[1] + ratio * (b[1] - a[1]), a[2] + ratio * (b[2] - a[2])];
      }
    }
    return waypoints[waypoints.length - 1].slice(1);
  }

  const { bounds, start, goal, robot_radius: robotRadius } = data.scene;
  const [minX, maxX, minY, maxY] = bounds;
  const spanX = maxX - minX;
  const spanY = maxY - minY;
  const shelves = [];
  const walls = [];
  data.scene.static.forEach((obstacle) => {
    const [x, y] = obstacle.center;
    const boundary = x <= minX || x >= maxX || y <= minY || y >= maxY;
    (boundary ? walls : shelves).push(obstacle);
  });

  function corners(x, y, width, depth, z) {
    const left = x - width / 2;
    const right = x + width / 2;
    const front = y - depth / 2;
    const back = y + depth / 2;
    return [[left, front, z], [right, front, z], [right, back, z], [left, back, z]];
  }

  function draw() {
    if (!ctx || canvasWidth < 2 || canvasHeight < 2) return;
    const active = run();
    const iso = state.view === "iso";
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const rise = 0.55;
    const scale = iso
      ? Math.min((canvasWidth - 36) / (0.8 * spanX + 0.6 * spanY + 0.6),
        (canvasHeight - 22) / ((0.6 * spanX + 0.8 * spanY) * rise + 0.95))
      : Math.min((canvasWidth - 48) / (spanX + 0.3), (canvasHeight - 30) / (spanY + 0.3));
    const centerV = canvasHeight / 2 + (iso ? scale * 0.15 : 0);
    function depth(x, y) { return 0.6 * x - 0.8 * y; }
    function project(x, y, z = 0) {
      const dx = x - centerX;
      const dy = y - centerY;
      return iso
        ? [canvasWidth / 2 + scale * (0.8 * dx + 0.6 * dy), centerV + scale * (depth(dx, dy) * rise - z * 0.835)]
        : [canvasWidth / 2 + scale * dx, centerV - scale * dy];
    }
    function trace(points) {
      ctx.beginPath();
      points.forEach((point, index) => {
        const [x, y] = project(...point);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
    }
    function face(points, color) {
      trace(points);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();
    }
    function line(points, color, width, dash = []) {
      trace(points);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.setLineDash(dash);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    // The cap and two visible faces use the same footprint in both views.
    function block(x, y, w, h, base, top, colors) {
      const lid = corners(x, y, w, h, top);
      if (iso) {
        const foot = corners(x, y, w, h, base);
        face([foot[0], foot[1], lid[1], lid[0]], colors[1]);
        face([foot[1], foot[2], lid[2], lid[1]], colors[2]);
      }
      face(lid, colors[0]);
    }
    function rack(obstacle) {
      const [x, y] = obstacle.center;
      const [w, h] = obstacle.size;
      block(x, y, w, h, 0, 0.41, ["#939e94", "#687970", "#53655d"]);
      block(x, y, w, h, 0.41, 0.49, ["#d3b595", "#c49672", "#a0775c"]);
      const baysX = Math.max(1, Math.round(w / 0.84));
      const baysY = Math.max(1, Math.round(h / 0.84));
      const cellW = w / baysX;
      const cellH = h / baysY;
      for (let row = baysY - 1; row >= 0; row -= 1) {
        for (let col = 0; col < baysX; col += 1) {
          const bx = x - w / 2 + (col + 0.5) * cellW;
          const by = y - h / 2 + (row + 0.5) * cellH;
          block(bx, by, cellW - 0.055, cellH - 0.055, 0.49, 0.8, ["#ddd1b2", "#bcaa88", "#a5967a"]);
          face(corners(bx, by, cellW * 0.16, cellH - 0.06, 0.8), "#efe2bd");
        }
      }
    }
    function disc(x, y, radius, height, cap, side) {
      const [u, v] = project(x, y, height);
      const bottom = project(x, y)[1];
      const rx = radius * scale;
      const ry = rx * (iso ? rise : 1);
      ctx.beginPath();
      ctx.ellipse(u, bottom, rx, ry, 0, 0, Math.PI);
      ctx.ellipse(u, v, rx, ry, 0, Math.PI, 2 * Math.PI);
      ctx.closePath();
      ctx.fillStyle = side;
      ctx.fill();
      ctx.beginPath();
      ctx.ellipse(u, v, rx, ry, 0, 0, 2 * Math.PI);
      ctx.fillStyle = cap;
      ctx.fill();
      return [u, v];
    }
    function ring(x, y, radius, color, width) {
      const [u, v] = project(x, y);
      ctx.beginPath();
      ctx.ellipse(u, v, radius * scale, radius * scale * (iso ? rise : 1), 0, 0, Math.PI * 2);
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.stroke();
    }
    function pin(point, name, direction) {
      const [x, y] = project(...point);
      const tagX = Math.max(17, Math.min(canvasWidth - 17, x + direction * 23));
      const tagY = Math.max(11, Math.min(canvasHeight - 11, y + (iso ? 18 : -direction * 22)));
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(tagX, tagY);
      ctx.strokeStyle = "#dce5d2";
      ctx.lineWidth = 0.9;
      ctx.stroke();
      ctx.fillStyle = "#27312e";
      ctx.fillRect(tagX - 15, tagY - 9, 30, 18);
      ctx.fillStyle = "#f0f2e7";
      ctx.font = '11px "Microsoft YaHei", sans-serif';
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(name, tagX, tagY);
    }

    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    block(centerX, centerY, spanX + 0.24, spanY + 0.24, -0.24, 0, ["#515f59", "#3c4943", "#35423e"]);
    for (let x = Math.ceil(minX); x <= maxX; x += 1) {
      line([[x, minY], [x, maxY]], "#728077", 0.7);
    }
    for (let y = Math.ceil(minY); y <= maxY; y += 1) {
      line([[minX, y], [maxX, y]], "#728077", 0.7);
    }
    walls.forEach((wall) => block(...wall.center, ...wall.size, 0, 0.17, ["#a2ada1", "#78877c", "#66786e"]));
    shelves.forEach(({center: [x, y], size: [w, h]}) => {
      face(corners(x + 0.13, y - 0.17, w, h, 0), "#2c383687");
    });

    let reference = active.initial_path;
    for (const update of active.route_updates) {
      if (update.t <= state.time) reference = update.path;
    }
    line(reference, "#e0e6cf", 1.5, [4, 4]);
    const sample = sampleAt(active.trajectory, state.time);
    const [, robotX, robotY, heading] = sample;
    const executed = [start];
    for (const [t, x, y] of active.trajectory) {
      if (t > state.time) break;
      executed.push([x, y]);
    }
    executed.push([robotX, robotY]);
    line(executed, "#343a30", 5.5);
    line(executed, "#ff8954", 3.3);
    ring(...start, 0.16, "#fff1d7", 1.4);
    ring(...goal, 0.22, "#ecf2de", 1.5);
    ring(...goal, 0.08, "#ecf2de", 1.5);

    // Paint objects from back to front so shelving can occlude a passing cart.
    const objects = shelves.map((shelf) => ({
      depth: depth(...shelf.center), paint: () => rack(shelf)
    }));
    data.moving[String(state.seed)].forEach((cart) => {
      const [x, y] = cartPosition(cart.waypoints, state.time);
      objects.push({ depth: depth(x, y), paint() {
        disc(x, y, cart.radius, 0.7, "#91d2d1", "#50888b");
        line([[x - cart.radius * 0.45, y, 0.7], [x + cart.radius * 0.45, y, 0.7]], "#458284", 1.6);
      }});
    });
    objects.push({ depth: depth(robotX, robotY), paint() {
      const collision = state.time >= active.finish_time && active.status === "collision";
      const robotColor = collision ? "#ff6956" : "#ffab70";
      ring(robotX, robotY, robotRadius + 0.13, robotColor, 1.3);
      disc(robotX, robotY, robotRadius, 0.14, robotColor, "#b75935");
      const arrow = [[0.78, 0], [-0.38, 0.5], [-0.38, -0.5]].map(([dx, dy]) =>
        [robotX + robotRadius * (dx * Math.cos(heading) - dy * Math.sin(heading)),
          robotY + robotRadius * (dx * Math.sin(heading) + dy * Math.cos(heading)), 0.14]);
      face(arrow, "#3c392b");
    }});
    objects.sort((a, b) => a.depth - b.depth).forEach((object) => object.paint());
    pin(start, "起点", -1);
    pin(goal, "终点", 1);

    // Keep the current position visible when the robot passes behind a shelf.
    if (iso) {
      const [x, y] = project(robotX, robotY, 0.14);
      const tagY = Math.max(11, y - Math.max(21, scale * 0.94));
      ctx.beginPath();
      ctx.moveTo(x, y - 2);
      ctx.lineTo(x, tagY);
      ctx.strokeStyle = "#ffb27e";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.fillStyle = "#ffb27e";
      ctx.fillRect(x - 20, tagY - 8, 40, 16);
      ctx.fillStyle = "#242924";
      ctx.textAlign = "center";
      ctx.fillText("机器人", x, tagY);
    }

    if (!iso) {
      ctx.fillStyle = "#b8c5bb";
      ctx.font = "10px Consolas, monospace";
      ctx.textAlign = "center";
      for (let tick = Math.ceil(minX / 2) * 2; tick < maxX; tick += 2) {
        const [x, y] = project(tick, minY);
        ctx.fillText(String(tick), x, y + 10);
      }
      ctx.textAlign = "right";
      for (let tick = Math.ceil(minY / 2) * 2; tick < maxY; tick += 2) {
        const [x, y] = project(minX, tick);
        ctx.fillText(String(tick), x - 7, y);
      }
    }
  }

  function resizeCanvas() {
    const bounds = canvas.getBoundingClientRect();
    canvasWidth = Math.max(100, bounds.width);
    canvasHeight = Math.max(100, bounds.height);
    const ratio = Math.min(window.devicePixelRatio || 1, 3);
    canvas.width = Math.round(canvasWidth * ratio);
    canvas.height = Math.round(canvasHeight * ratio);
    if (ctx) ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    draw();
  }

  function updateClock() {
    const active = run();
    $("timeline").value = String(state.time);
    $("timeline").style.setProperty("--played", (100 * state.time / active.finish_time) + "%");
    $("clock").textContent = fixed(state.time, 2) + " / " + fixed(active.finish_time, 2) + " s";
    $("timeline").setAttribute("aria-valuetext", fixed(state.time, 2) + " 秒，共 " + fixed(active.finish_time, 2) + " 秒");
    const finished = state.time >= active.finish_time;
    const descriptions = { success: "已到达", collision: "发生碰撞", timeout: "已超时" };
    const position = state.time === 0 ? "起点" : "已暂停";
    $("run-status").textContent = finished ? descriptions[active.status] : (state.playing ? "正在回放" : position);
    $("run-status").classList.toggle("collision", finished && active.status === "collision");
  }

  function syncPlayback() {
    const playing = Boolean(ctx && playbackRequested && viewerVisible && !document.hidden && state.time < run().finish_time);
    if (playing === state.playing) return;
    state.playing = playing;
    state.previousFrame = null;
    $("play-toggle").setAttribute("aria-pressed", String(playing));
    $("play-toggle").setAttribute("aria-label", playing ? "暂停轨迹" : "播放轨迹");
    updateClock();
    if (frameRequest !== null) cancelAnimationFrame(frameRequest);
    frameRequest = playing ? requestAnimationFrame(animate) : null;
  }

  function requestPlayback(play) {
    playbackRequested = play;
    syncPlayback();
  }

  function animate(timestamp) {
    if (!state.playing) return;
    if (state.previousFrame !== null) {
      state.time = Math.min(run().finish_time, state.time + Math.min((timestamp - state.previousFrame) / 1000, 0.1) * state.speed);
    }
    state.previousFrame = timestamp;
    updateClock();
    draw();
    if (state.time >= run().finish_time) requestPlayback(false);
    else frameRequest = requestAnimationFrame(animate);
  }

  function chooseRun() {
    const active = run();
    state.time = Math.min(state.time, active.finish_time);
    state.previousFrame = null;
    if (state.time >= active.finish_time) playbackRequested = false;
    $("timeline").max = String(active.finish_time);
    $("seed-select").value = String(state.seed);
    $("method-select").value = state.method;
    $("run-time-label").textContent = active.status === "success" ? "到达耗时" : "终止时间";
    $("run-time").textContent = fixed(active.finish_time, 2) + " s";
    $("run-clearance").textContent = fixed(active.min_clearance, 4) + " m";
    $("run-replans").textContent = active.replan_count + " 次";
    const outcome = { success: "无碰撞到达", collision: "发生碰撞", timeout: "超时" };
    canvas.setAttribute("aria-label", label(state.method) + " 在种子 " + state.seed + " 下的仓库轨迹，" + outcome[active.status] + "。");
    syncPlayback();
    updateClock();
    draw();
  }

  $("play-toggle").addEventListener("click", () => {
    if (!state.playing && state.time >= run().finish_time) state.time = 0;
    requestPlayback(!state.playing);
  });
  $("reset-playback").addEventListener("click", () => {
    state.time = 0;
    requestPlayback(false);
    updateClock();
    draw();
  });
  $("timeline").addEventListener("input", (event) => {
    const requestedTime = Number(event.target.value);
    state.time = Math.min(requestedTime, run().finish_time);
    requestPlayback(false);
    updateClock();
    draw();
  });
  $("speed-select").addEventListener("change", (event) => { state.speed = Number(event.target.value); });
  data.methods.forEach(({ id }) => $("method-select").add(new Option(label(id), id)));
  data.seeds.forEach((seed) => $("seed-select").add(new Option(String(seed), String(seed))));
  $("seed-select").addEventListener("change", (event) => { state.seed = Number(event.target.value); chooseRun(); });
  $("method-select").addEventListener("change", (event) => { state.method = event.target.value; chooseRun(); });
  const viewButtons = document.querySelectorAll("[data-view]");
  viewButtons.forEach((button) => {
    button.addEventListener("click", () => {
      state.view = button.dataset.view;
      viewButtons.forEach((option) => option.setAttribute("aria-pressed", String(option === button)));
      draw();
    });
  });
  document.addEventListener("visibilitychange", syncPlayback);
  motionPreference.addEventListener("change", (event) => {
    if (event.matches) requestPlayback(false);
  });
  if (window.IntersectionObserver && ctx) {
    const observer = new IntersectionObserver(([entry]) => {
      viewerVisible = entry.isIntersecting;
      syncPlayback();
    });
    observer.observe(document.querySelector(".instrument"));
  }

  const collisionCase = runsByKey.get("astar:4");
  const hasCollisionCase = collisionCase && collisionCase.status === "collision";
  const arrivals = data.runs.filter((record) => record.status === "success").length;
  $("replay-collision").addEventListener("click", () => {
    if (!hasCollisionCase) return;
    state.method = collisionCase.method;
    state.seed = collisionCase.seed;
    state.time = Math.max(0, collisionCase.finish_time - 5);
    chooseRun();
    $("play-toggle").focus({ preventScroll: true });
    document.querySelector(".instrument").scrollIntoView({ block: "center" });
    requestPlayback(true);
  });

  const groups = {
    global: ["astar", "rrt_star", "voronoi", "voronoi_qp"],
    local: ["astar", "apf", "dwa"],
    webots: ["astar", "rrt_star", "voronoi", "apf", "dwa", "voronoi_qp"]
  };
  function renderRecords() {
    const method = $("record-method").value;
    const statuses = { success: "到达", collision: "碰撞", timeout: "超时" };
    const rows = data.seeds.map((seed) => {
      const record = runsByKey.get(method + ":" + seed);
      const row = element("tr");
      row.append(
        element("td", "", seed),
        element("td", record.status === "success" ? "" : "failed-run", statuses[record.status]),
        element("td", "", fixed(record.finish_time, 2) + " s"),
        element("td", "", fixed(record.path_length, 3) + " m"),
        element("td", "", fixed(record.min_clearance, 4) + " m"),
        element("td", "", record.replan_count)
      );
      return row;
    });
    $("records-body").replaceChildren(...rows);
  }

  function setRecordMethods() {
    const native = state.group === "webots";
    $("run-records").hidden = native;
    if (native) return;
    const select = $("record-method");
    const previous = select.value;
    const methods = groups[state.group];
    select.replaceChildren(...methods.map((id) => new Option(label(id, state.group === "local"), id)));
    if (methods.includes(previous)) select.value = previous;
    renderRecords();
  }
  $("record-method").addEventListener("change", renderRecords);

  function resultRows() {
    return groups[state.group].map((id) => {
      if (state.group === "webots") {
        const native = data.webots.runs.find((item) => item.method === id);
        return { method: id, success: native.status === "success" ? 1 : 0, n: 1,
          mean_time: native.finish_time, mean_path: native.path_length, mean_clearance: native.min_clearance };
      }
      return summaryById.get(id);
    });
  }
  function renderResults() {
    const native = state.group === "webots";
    const local = state.group === "local";
    const rows = resultRows();
    const descriptions = {
      global: "四种规划器共用一个方向跟踪器，各跑五次。同一轮里，小车经过路口的时刻相同。",
      local: "三种控制器都沿 A* 给出的路线行驶。公共方向跟踪器一行取自 A* 的五次运行。",
      webots: "在 Webots R2025a 中另跑了一组验证。六种方法各跑一次，都使用种子 0。"
    };
    $("group-description").textContent = descriptions[state.group];
    const headers = { "time-header": "耗时", "length-header": native ? "实际行程" : "行程" };
    for (const [id, name] of Object.entries(headers)) {
      $(id).replaceChildren(name);
      if (!native) $(id).prepend(element("span", "header-prefix", "平均"));
    }
    $("distance-average").hidden = native;
    $("replay-collision").hidden = native || !hasCollisionCase;
    if (native) {
      $("table-note").replaceChildren("每行是一次 Webots 运行。代码版本 ", element("code", "", data.webots.source_commit.slice(0, 7)), "。");
    } else {
      $("table-note").textContent = "耗时和行程只对成功到达的运行取平均。距离一栏包含全部五次，发生碰撞的那次也算在内。";
    }
    const body = $("results-body");
    body.replaceChildren();
    rows.forEach((row) => {
      const tr = element("tr", row.method === "voronoi_qp" ? "featured" : "");
      tr.append(element("td", "", label(row.method, local)));
      const successCell = element("td");
      successCell.append(element("span", "success-count" + (row.success < row.n ? " partial" : ""), row.success + "/" + row.n));
      tr.append(successCell);
      tr.append(element("td", "", fixed(row.mean_time, native ? 3 : 2) + " s"));
      tr.append(element("td", "", fixed(row.mean_path, 3) + " m"));
      tr.append(element("td", "", fixed(row.mean_clearance, 4) + " m"));
      body.append(tr);
    });
    if (native) {
      $("evidence-text").textContent = "原生仿真每 32 毫秒检查一次机器人与障碍物的实际间距。六次运行都无碰撞到达了终点。";
    } else {
      const incident = hasCollisionCase
        ? "A* 在种子 4 的第 " + fixed(collisionCase.finish_time, 2) + " 秒撞上了从侧面驶入的装卸车。" : "";
      $("evidence-text").textContent = incident + data.runs.length + " 次二维运行中有 " + arrivals + " 次无碰撞到达。";
    }
    const settings = {
      clearance: { key: "mean_clearance", title: "与障碍物的最小距离", unit: "m", digits: 4,
        subtitle: native ? "每种方法跑一次，记下车身边缘离障碍物最近时的距离。" : "每次先记下车身边缘离障碍物最近时的距离，再取五次的平均值。" },
      time: { key: "mean_time", title: native ? "到达耗时" : "平均到达耗时", unit: "s", digits: native ? 3 : 2,
        subtitle: "从出发到进入目标范围所用的仿真时间。" },
      length: { key: "mean_path", title: native ? "实际行程" : "平均实际行程", unit: "m", digits: 3,
        subtitle: "沿机器人实际走过的轨迹计算距离。" }
    };
    const metric = settings[state.metric];
    $("chart-title").textContent = metric.title;
    $("chart-subtitle").textContent = metric.subtitle;
    const bars = $("metric-bars");
    bars.replaceChildren();
    const max = Math.max(...rows.map((row) => row[metric.key] || 0), 0.0001);
    rows.forEach((row) => {
      const wrapper = element("div", "metric-row" + (row.method === "voronoi_qp" ? " featured" : ""));
      const heading = element("div", "bar-label", label(row.method, local));
      const value = element("span", "bar-value", fixed(row[metric.key], metric.digits) + " " + metric.unit);
      const track = element("div", "bar-track");
      track.setAttribute("aria-hidden", "true");
      const fill = element("div", "bar-fill");
      fill.style.width = Math.max(0, (row[metric.key] || 0) / max * 100) + "%";
      track.append(fill); wrapper.append(heading, track, value); bars.append(wrapper);
    });
  }

  const tabs = Array.from(document.querySelectorAll("[data-group]"));
  function chooseGroup(button) {
    state.group = button.dataset.group;
    tabs.forEach((tab) => {
      const selected = tab === button;
      tab.setAttribute("aria-selected", String(selected));
      tab.tabIndex = selected ? 0 : -1;
    });
    $("results-panel").setAttribute("aria-labelledby", button.id);
    renderResults();
    setRecordMethods();
  }
  tabs.forEach((button, index) => {
    button.addEventListener("click", () => chooseGroup(button));
    button.addEventListener("keydown", (event) => {
      let next = index;
      if (event.key === "ArrowRight") next = (index + 1) % tabs.length;
      else if (event.key === "ArrowLeft") next = (index + tabs.length - 1) % tabs.length;
      else if (event.key === "Home") next = 0;
      else if (event.key === "End") next = tabs.length - 1;
      else return;
      event.preventDefault(); tabs[next].focus(); chooseGroup(tabs[next]);
    });
  });
  $("metric-select").addEventListener("change", (event) => {
    state.metric = event.target.value; renderResults();
  });

  const baseline = summaryById.get("voronoi");
  const featured = summaryById.get("voronoi_qp");
  function percentage(id, number) {
    $(id).textContent = (number >= 0 ? "+" : "−") + fixed(Math.abs(number), 1) + "%";
  }
  percentage("clearance-gain", (featured.mean_clearance / baseline.mean_clearance - 1) * 100);
  percentage("curvature-gain", (featured.mean_curvature / baseline.mean_curvature - 1) * 100);
  percentage("accel-gain", (featured.mean_angular_accel / baseline.mean_angular_accel - 1) * 100);
  $("extra-time").textContent = fixed(featured.mean_time - baseline.mean_time, 2) + " s";
  $("extra-distance").textContent = fixed(featured.mean_path - baseline.mean_path, 3) + " m";

  resizeCanvas();
  chooseRun();
  chooseGroup(tabs[0]);
  if (window.ResizeObserver) new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
  else window.addEventListener("resize", resizeCanvas);
  if (!ctx) {
    $("data-error").hidden = false;
    $("data-error").textContent = "当前浏览器用静态图显示种子 0 的六条路线。";
  }
}());
