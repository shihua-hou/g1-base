/* G1 iPad 上位机前端 — 纯原生 JS 单页应用，无构建步骤。 */
(() => {
  "use strict";

  const app = document.getElementById("app");

  const state = {
    baseUrl: localStorage.getItem("g1_base_url") || "",
    status: null,
    statusError: null,
    mapsCache: null,
    routesCache: null,
    selectedMapId: null,
    pollTimer: null,
  };

  // ── API 客户端 ──
  async function api(path, { method = "GET", body } = {}) {
    const res = await fetch(state.baseUrl + path, {
      method,
      headers: body !== undefined ? { "Content-Type": "application/json" } : {},
      body: body !== undefined ? JSON.stringify(body) : undefined,
    });
    const ct = res.headers.get("content-type") || "";
    if (!res.ok) {
      let msg = res.statusText;
      if (ct.includes("application/json")) {
        try { msg = (await res.json()).error || msg; } catch (_e) { /* ignore */ }
      }
      throw new Error(msg);
    }
    if (ct.includes("application/json")) return res.json();
    return res;
  }

  function toast(message, kind = "") {
    const el = document.createElement("div");
    el.className = `toast ${kind}`;
    el.textContent = message;
    document.body.appendChild(el);
    // 先加淡出类播放退出动画，再在动画结束后移除 DOM
    setTimeout(() => {
      el.classList.add("is-leaving");
      el.addEventListener("animationend", () => el.remove(), { once: true });
      // 兆底：如果动画事件没触发，400ms 后强制移除
      setTimeout(() => { if (el.isConnected) el.remove(); }, 400);
    }, 2900);
  }

  // 耗时操作的按钮忙碌态：禁用 + 换文案 + 转圈。
  //
  // 之前所有按钮都是"点下去毫无变化，十几秒后突然弹个 toast"——「开启导航栈」
  // 要跑完定位和 Nav2 拉起，实测 15 秒，这期间界面上完全看不出在干活，
  // 现场只能反复点。文案要写清楚正在做什么，光转圈还是不知道在等什么。
  async function withBusy(el, busyLabel, fn) {
    if (!el) return fn();
    const oldHtml = el.innerHTML;
    const oldDisabled = el.disabled;
    el.disabled = true;
    el.classList.add("is-busy");
    el.innerHTML = `<span class="btn-spin"></span>${escapeHtml(busyLabel)}`;
    try {
      return await fn();
    } finally {
      // 按钮可能已经被重绘掉了（比如成功后跳页），那就什么都不用还原
      if (el.isConnected) {
        el.classList.remove("is-busy");
        el.innerHTML = oldHtml;
        el.disabled = oldDisabled;
      }
    }
  }

  // 常用组合：忙碌态 + 统一的成功/失败 toast
  function bindBusy(id, busyLabel, fn, okMsg) {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener("click", async () => {
      if (el.disabled) return;
      try {
        await withBusy(el, busyLabel, () => guarded(fn, okMsg));
      } catch (_e) { /* toast 里已经报过 */ }
    });
  }

  async function guarded(fn, okMsg) {
    try {
      const result = await fn();
      if (okMsg) toast(okMsg, "success");
      return result;
    } catch (err) {
      toast(String(err.message || err), "error");
      throw err;
    }
  }

  // ── 路由 ──
  function currentRoute() {
    const hash = location.hash.replace(/^#\/?/, "");
    return hash ? hash.split("/") : [];
  }
  function nav(path) { location.hash = "#/" + path; }
  window.addEventListener("hashchange", render);

  // ── 浏览器播报 ──
  //
  // 机器人扬声器要靠 SDK 的 audio 进程活着，现场时好时坏；浏览器这条路
  // 只要页面开着就一定响，所以默认两条一起走。
  //
  // 浏览器有个硬限制：没有用户手势之前 speechSynthesis 会被静音策略挡掉。
  // 所以第一次用户点任意位置时"解锁"一次（念一段空白），之后才能自动播。
  const speech = {
    lastId: 0,          // 已经念到哪条
    unlocked: false,
    supported: typeof window !== "undefined" && "speechSynthesis" in window,
  };

  function unlockSpeech() {
    if (speech.unlocked || !speech.supported) return;
    try {
      // 念一个空白，纯粹为了拿到播放权限
      const u = new SpeechSynthesisUtterance(" ");
      u.volume = 0;
      window.speechSynthesis.speak(u);
      speech.unlocked = true;
    } catch (_e) { /* 不支持就算了，机器人那条路还在 */ }
  }
  document.addEventListener("pointerdown", unlockSpeech, { once: true });
  document.addEventListener("keydown", unlockSpeech, { once: true });

  function speakInBrowser(text) {
    if (!speech.supported || !text) return;
    try {
      const u = new SpeechSynthesisUtterance(String(text));
      u.lang = "zh-CN";
      u.rate = 1.0;
      window.speechSynthesis.speak(u);
    } catch (err) {
      console.warn("浏览器播报失败", err);
    }
  }

  // 每次状态刷新后把新条目念掉。服务端已经做完了去重和冷却，
  // 这里只负责发声——两边逻辑不重复，也就不会跑偏。
  function drainSpeechQueue() {
    const q = (state.status && state.status.speech) || null;
    if (!q || !Array.isArray(q.items)) return;
    // 首次拿到队列时不补播历史：页面刚打开不该把之前攒的一次性念出来
    if (speech.lastId === 0) { speech.lastId = q.next_id - 1; return; }
    q.items
      .filter((it) => it.id > speech.lastId)
      .forEach((it) => { speech.lastId = it.id; speakInBrowser(it.text); });
    if (q.next_id - 1 > speech.lastId) speech.lastId = q.next_id - 1;
  }

  // ── 轮询状态 ──
  function startPolling() {
    stopPolling();
    const tick = async () => {
      try {
        state.status = await api("/api/status");
        state.statusError = null;
      } catch (err) {
        state.statusError = String(err.message || err);
      }
      updateLiveRegions();
      drainSpeechQueue();
    };
    tick();
    state.pollTimer = setInterval(tick, 1500);
  }
  function stopPolling() {
    if (state.pollTimer) clearInterval(state.pollTimer);
    state.pollTimer = null;
  }

  // 只刷新页面里标了 data-live 的片段，避免打字/滑动时页面跳动
  function updateLiveRegions() {
    document.querySelectorAll("[data-live]").forEach((el) => {
      const renderer = LIVE_RENDERERS[el.getAttribute("data-live")];
      if (renderer) el.innerHTML = renderer();
    });
    // 导航栈那两个按钮不在 data-live 片段里（重绘会打断 details 折叠和焦点），
    // 只改 disabled，所以单独在这里跟着状态一起刷。不在当前页面时自己返回。
    syncNavStackButtons();
  }

  const LIVE_RENDERERS = {
    "conn-dot": () => connDotHtml(),
    "status-grid": () => statusGridHtml(),
    "nav-panel": () => navHeadlineHtml(),
    "status-col": () => statusColHtml(),
    "state-nav": () => stateNavHtml(),
    "state-detail": () => stateDetailHtml(),
    "nav-map-name": () => navMapNameHtml(),
  };

  function navMapNameHtml() {
    const cm = (state.status && state.status.current_map) || null;
    const cls = cm ? "" : " is-warn";
    return `<span class="map-name-tag${cls}">${escapeHtml(currentMapLabel())}</span>`;
  }

  function connDotHtml() {
    const ok = !!state.status && !state.statusError;
    return `<span class="dot ${ok ? "ok" : "bad"}"></span><span>${state.baseUrl.replace(/^https?:\/\//, "")}</span>`;
  }

  function fmtBool(v) { return v ? "是" : "否"; }

  function statusGridHtml() {
    if (state.statusError) {
      return `<div class="center-text">连接失败: ${escapeHtml(state.statusError)}</div>`;
    }
    if (!state.status) return `<div class="center-text">加载中…</div>`;
    const s = state.status;
    const control = s.control || {};
    const navm = s.navigation_manager || {};
    const pose = s.pose;
    const items = [
      ["活动状态", control.activity || "idle"],
      ["活动详情", control.activity_detail || "—"],
      ["导航栈状态", navm.state || "—"],
      ["导航栈就绪", fmtBool(navm.ready)],
      ["运行模式", navm.mode || "—"],
      ["蹲下状态", fmtBool(control.is_squatting)],
      ["急停锁定", fmtBool(control.stop_latched)],
      ["当前位姿", pose ? `x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${pose.yaw_deg.toFixed(0)}°` : "无 TF"],
      ["当前地图", (s.current_map && s.current_map.base_name) || "未知"],
    ];
    return items.map(([label, value]) => `
      <div class="status-item"><div class="label">${label}</div><div class="value">${escapeHtml(String(value))}</div></div>
    `).join("");
  }

  function escapeHtml(str) {
    return String(str).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  }

  // ── 页面外壳 ──
  // 全站只有这一条状态栏：首页与所有子页面共用，右侧信息位置完全一致。
  function statusbarHtml({ back = null, title = "Robot Console", sub = "", tabs = null, active = "" } = {}) {
    const tabsHtml = tabs
      ? `<div class="sb-tabs">${tabs.items
          .map(([k, label]) => `<button class="${k === active ? "active" : ""}" data-nav="${tabs.base}/${k}">${label}</button>`)
          .join("")}</div>`
      : "";
    return `
      <div class="statusbar">
        ${back ? `<button class="sb-back" data-nav="${back}">‹ 返回</button>` : ""}
        <div class="sb-brand">
          <span class="sb-mark"></span>
          <span class="sb-title">${escapeHtml(title)}</span>
          ${sub ? `<span class="sb-sub">${escapeHtml(sub)}</span>` : ""}
        </div>
        ${tabsHtml}
        <div class="sb-spacer"></div>
        <div class="sb-right">
          <div class="sb-conn" data-live="conn-dot">${connDotHtml()}</div>
          <div class="sb-clock" id="sb-clock">—</div>
        </div>
      </div>
    `;
  }

  // 页面级清理钩子：摇杆循环、地图视图、局部定时器都登记在这里，
  // 切页时统一收掉，免得后台还在发速度指令或者刷地图。
  let pageCleanups = [];
  function onPageLeave(fn) { pageCleanups.push(fn); }
  function runPageCleanups() {
    const list = pageCleanups;
    pageCleanups = [];
    list.forEach((fn) => { try { fn(); } catch (_e) { /* 清理失败不影响切页 */ } });
  }

  // cols 决定这一屏怎么分栏；页面本身永远不滚动，只有面板内部可能滚。
  function shell(opts, cols, bodyHtml) {
    runPageCleanups();
    app.innerHTML = statusbarHtml(opts) + `<div class="page ${cols}">${bodyHtml}</div>`;
    bindGlobalHandlers();
    startClock();
  }

  // ── 模态框（切换路线等） ──
  function showModal(title, bodyHtml, { onClose = null } = {}) {
    const wrap = document.createElement("div");
    wrap.className = "modal-mask";
    wrap.innerHTML = `
      <div class="modal">
        <div class="modal-head"><div class="eyebrow">${escapeHtml(title)}</div><button class="btn sm ghost" data-modal-close>关闭</button></div>
        <div class="modal-body">${bodyHtml}</div>
      </div>
    `;
    const close = () => { wrap.remove(); if (onClose) onClose(); };
    wrap.addEventListener("click", (ev) => {
      if (ev.target === wrap || ev.target.hasAttribute("data-modal-close")) close();
    });
    document.body.appendChild(wrap);
    return { el: wrap, close };
  }

  // 用事件委托而不是逐个绑定：data-live 区域每 1.5 秒会整块重绘，
  // 逐个绑的监听器会跟着元素一起没掉，委托到 #app 上就不受影响。
  let navDelegated = false;
  function bindGlobalHandlers() {
    if (navDelegated) return;
    navDelegated = true;
    app.addEventListener("click", (ev) => {
      const el = ev.target.closest && ev.target.closest("[data-nav]");
      if (el) nav(el.getAttribute("data-nav"));
    });
  }

  // ── 禁止整页缩放 ──
  // meta viewport 的 user-scalable=no 在部分内核上会被忽略，这里补一层：
  // 双指手势与双击一律吞掉，只有显式标了 data-allow-gesture 的元素
  //（3D 舞台）自己处理多指操作。
  function blockPageZoom() {
    const allowed = (target) => target && target.closest && target.closest("[data-allow-gesture]");
    document.addEventListener("touchmove", (ev) => {
      if (ev.touches.length > 1 && !allowed(ev.target)) ev.preventDefault();
    }, { passive: false });
    ["gesturestart", "gesturechange", "gestureend"].forEach((type) => {
      document.addEventListener(type, (ev) => { if (!allowed(ev.target)) ev.preventDefault(); }, { passive: false });
    });
    document.addEventListener("wheel", (ev) => {
      if (ev.ctrlKey && !allowed(ev.target)) ev.preventDefault();   // 触控板/鼠标的缩放手势
    }, { passive: false });
    // 双击缩放交给 CSS 的 touch-action: manipulation 处理。
    // 不用「300ms 内吞掉第二次 touchend」那套：连点两个不同按钮时
    // 会把第二次的 click 一起吞掉，控制页上这是实打实的误操作。
  }

  // ── 连接页 ──
  function renderConnect() {
    stopPolling();
    runPageCleanups();   // 这一页不走 shell()，清理要自己来
    app.innerHTML = `
      <div class="connect-screen">
        <div class="connect-card">
          <div class="connect-mark">${ICONS.nav}</div>
          <h1>Robot Console</h1>
          <p>输入机器人上运行 g1_web_bridge 的 IP 地址与端口</p>
          <div class="field"><label>IP 地址</label><input id="ip-input" placeholder="192.168.10.99" inputmode="decimal" /></div>
          <div class="field"><label>端口</label><input id="port-input" placeholder="8081" inputmode="numeric" /></div>
          <button class="btn primary block" id="connect-btn">连接</button>
        </div>
      </div>
    `;
    const saved = state.baseUrl.match(/^https?:\/\/([^:]+):(\d+)/);
    if (saved) {
      document.getElementById("ip-input").value = saved[1];
      document.getElementById("port-input").value = saved[2];
    }
    document.getElementById("connect-btn").addEventListener("click", async () => {
      const ip = document.getElementById("ip-input").value.trim();
      const port = document.getElementById("port-input").value.trim() || "8081";
      if (!ip) { toast("请输入 IP 地址", "error"); return; }
      const url = `http://${ip}:${port}`;
      try {
        const res = await fetch(url + "/api/status", { cache: "no-store" });
        if (!res.ok) throw new Error("HTTP " + res.status);
        state.baseUrl = url;
        localStorage.setItem("g1_base_url", url);
        nav("home");
      } catch (err) {
        toast("无法连接: " + (err.message || err), "error");
      }
    });
  }

  // ── 首页 ──
  // 线条图标：统一 24 网格、1.6 描边、currentColor，避免 emoji 的平台差异
  const SVG = (inner) =>
    `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6"
       stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${inner}</svg>`;

  const ICONS = {
    map: SVG('<path d="M9 4 3 6.5v13L9 17l6 2.5 6-2.5v-13L15 6.5 9 4Z"/><path d="M9 4v13M15 6.5v13"/>'),
    nav: SVG('<circle cx="12" cy="12" r="9"/><path d="m15.5 8.5-2.1 5.4-5.4 2.1 2.1-5.4 5.4-2.1Z"/>'),
    control: SVG('<circle cx="12" cy="12" r="3.2"/><path d="M12 2.8v3M12 18.2v3M2.8 12h3M18.2 12h3M5.5 5.5l2.1 2.1M16.4 16.4l2.1 2.1M18.5 5.5l-2.1 2.1M7.6 16.4l-2.1 2.1"/>'),
    teach: SVG('<path d="M8 11V5.5a1.5 1.5 0 0 1 3 0V11m0-1.5V4.5a1.5 1.5 0 0 1 3 0V11m0-1.2a1.5 1.5 0 0 1 3 0V15a6 6 0 0 1-6 6h-.7a5 5 0 0 1-4.2-2.3L5 15.5a1.6 1.6 0 0 1 2.7-1.7L8 14.4"/>'),
    state: SVG('<path d="M3 20h18"/><path d="M6 20v-6M11 20V8M16 20v-9M21 20V5"/>'),
    settings: SVG('<circle cx="12" cy="12" r="3"/><path d="M19.4 14.5a1.6 1.6 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.6 1.6 0 0 0-1.8-.3 1.6 1.6 0 0 0-1 1.5v.2a2 2 0 1 1-4 0v-.1a1.6 1.6 0 0 0-1.1-1.5 1.6 1.6 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.6 1.6 0 0 0 .3-1.8 1.6 1.6 0 0 0-1.5-1H3a2 2 0 1 1 0-4h.1a1.6 1.6 0 0 0 1.5-1.1 1.6 1.6 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.6 1.6 0 0 0 1.8.3H9a1.6 1.6 0 0 0 1-1.5V3a2 2 0 1 1 4 0v.1a1.6 1.6 0 0 0 1 1.5 1.6 1.6 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.6 1.6 0 0 0-.3 1.8V9a1.6 1.6 0 0 0 1.5 1h.2a2 2 0 1 1 0 4h-.1a1.6 1.6 0 0 0-1.5 1Z"/>'),
  };

  const HOME_TILES = [
    ["map", ICONS.map, "地图", "建图 · 编辑"],
    ["nav", ICONS.nav, "导航", "定点 · 巡航"],
    ["control", ICONS.control, "控制", "移动 · 动作"],
    ["teach", ICONS.teach, "示教", "录制 · 回放"],
    ["state", ICONS.state, "状态", "遥测 · 诊断"],
    ["settings", ICONS.settings, "设置", "日志 · 参数"],
  ];

  function renderHome() {
    startPolling();
    disposeRobot3d();

    shell({ title: "Robot Console" }, "cols-home", `
      <aside class="pane status-col">
        <div class="pane-head"><div class="eyebrow">系统运行状态</div></div>
        <div class="pane-body" data-live="status-col">${statusColHtml()}</div>
      </aside>
      <section class="pane robot-pane">
        <div class="stage-tag">Unitree G1</div>
        <div class="stage-hint">拖动旋转 · 双指缩放 · 双击复位</div>
        <div class="robot-stage" id="robot-stage" data-allow-gesture></div>
        <div class="motion-bar" id="motion-bar">
          <span class="motion-now" id="robot-caption">加载中…</span>
        </div>
      </section>
      <nav class="nav-grid">
        ${HOME_TILES.map(([route, icon, name, sub]) => `
          <button class="nav-tile" type="button" data-nav="${route}" data-accent="${route}">
            <span class="nav-tile-icon">${icon}</span>
            <span class="nav-tile-name">${name}</span>
            <span class="nav-tile-sub">${sub}</span>
          </button>
        `).join("")}
      </nav>
    `);
    mountRobot3d();
  }

  // ── 顶栏时钟 ──
  let clockTimer = null;
  function startClock() {
    stopClock();
    const pad = (n, w = 2) => String(n).padStart(w, "0");
    const tick = () => {
      const el = document.getElementById("sb-clock");
      if (!el) { stopClock(); return; }
      const d = new Date();
      el.textContent = `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} `
        + `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
    };
    tick();
    clockTimer = setInterval(tick, 1000);
  }
  function stopClock() {
    if (clockTimer) clearInterval(clockTimer);
    clockTimer = null;
  }

  // ── 3D 模型 ──
  let robotViewer = null;
  function mountRobot3d() {
    const stage = document.getElementById("robot-stage");
    if (!stage || !window.G1Robot3D) return;
    robotViewer = window.G1Robot3D.create(stage);
    if (!robotViewer) return;

    const caption = () => document.getElementById("robot-caption");
    const bar = () => document.getElementById("motion-bar");

    // 动作切换条：点哪个演哪个，名字由 viewer 在相位变化时推过来，
    // 不再靠定时轮询去猜，画面和文字永远一致。
    robotViewer.onReady = (playlist) => {
      const el = bar();
      if (!el) return;
      el.innerHTML =
        playlist.map((m, i) => `<button class="motion-chip" data-motion="${i}">${escapeHtml(m.label)}</button>`).join("")
        + `<span class="motion-now" id="robot-caption">—</span>`;
      el.querySelectorAll("[data-motion]").forEach((btn) => {
        btn.addEventListener("click", () => robotViewer && robotViewer.playMotion(parseInt(btn.getAttribute("data-motion"), 10)));
      });
    };
    robotViewer.onMotionChange = ({ index, label }) => {
      const el = caption();
      if (el) el.textContent = label || "—";
      const barEl = bar();
      if (barEl) {
        barEl.querySelectorAll("[data-motion]").forEach((btn) => {
          btn.classList.toggle("active", parseInt(btn.getAttribute("data-motion"), 10) === index);
        });
      }
    };
  }
  function disposeRobot3d() {
    if (robotViewer) {
      robotViewer.dispose();
      robotViewer = null;
    }
  }

  function fmtDuration(sec) {
    if (sec == null) return "—";
    const s = Math.floor(sec);
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), ss = s % 60;
    if (h > 0) return `${h}h ${m}m`;
    if (m > 0) return `${m}m ${ss}s`;
    return `${ss}s`;
  }

  // 占用率环：比细长条更好扫视，也更耐远距离观看
  const RING_C = 163.4;   // 2π × r(26)
  function gaugeHtml(cap, value) {
    const v = value == null ? null : Math.max(0, Math.min(100, value));
    const tone = v == null ? "" : v >= 90 ? "is-crit" : v >= 70 ? "is-warn" : "";
    const offset = v == null ? RING_C : RING_C * (1 - v / 100);
    return `
      <div class="gauge ${tone}">
        <svg viewBox="0 0 64 64" aria-hidden="true">
          <circle class="ring-bg" cx="32" cy="32" r="26" />
          <circle class="ring-fg" cx="32" cy="32" r="26"
                  stroke-dasharray="${RING_C}" stroke-dashoffset="${offset.toFixed(1)}"
                  transform="rotate(-90 32 32)" />
          <text class="ring-text" x="32" y="32" text-anchor="middle" dominant-baseline="central">${v == null ? "—" : Math.round(v)}</text>
        </svg>
        <div class="gauge-cap">${cap}</div>
      </div>`;
  }

  function kvHtml(key, value, tone = "", dot = null) {
    return `<div class="kv">
      ${dot ? `<span class="dot ${dot}"></span>` : ""}
      <span class="k">${key}</span>
      <span class="v ${tone}">${escapeHtml(String(value))}</span>
    </div>`;
  }

  // 左栏：一个任务锚点 + 两个环 + 一组带指示灯的清单 + 一行机器脚注。
  // 分成四个层级，比原来八行等权重的列表好扫得多。
  function statusColHtml() {
    if (state.statusError) {
      return `<div class="center-text">连接失败<br><small>${escapeHtml(state.statusError)}</small></div>`;
    }
    if (!state.status) return `<div class="center-text">加载中…</div>`;
    const s = state.status;
    const c = s.control || {};
    const navm = s.navigation_manager || {};
    const sys = s.system || {};
    const pose = s.pose;
    const patrol = s.patrol || {};

    let task = c.activity && c.activity !== "idle" ? c.activity : null;
    if (!task && patrol.running) task = `巡航 ${patrol.index}/${patrol.total}`;
    const idle = !task;
    if (!task) task = "空闲";
    const taskTone = c.stop_latched ? "tone-crit" : idle ? "tone-ok" : "tone-run";
    const taskSub = c.stop_latched ? "急停锁定" : (c.activity_detail || (idle ? "等待指令" : ""));

    const navDot = navm.ready ? "ok" : navm.state === "ERROR" ? "bad" : "warn";
    const navTone = navm.ready ? "is-ok" : navm.state === "ERROR" ? "is-crit" : "is-warn";
    const battery = sys.battery_percent;
    const batteryTone = battery == null ? "is-dim"
      : battery <= 20 ? "is-crit"
      : battery <= 40 ? "is-warn" : "is-ok";
    const batteryPct = battery != null ? Math.max(0, Math.min(100, battery)) : 0;
    const batteryText = battery != null ? `${battery}%`
      : (sys.battery_topic ? "读数过期" : "未接入");

    return `
      <div class="task-block ${taskTone}">
        <div class="task-label">当前任务</div>
        <div class="task-value">${escapeHtml(task)}</div>
        ${taskSub ? `<div class="task-sub">${escapeHtml(taskSub)}</div>` : ""}
      </div>

      <div class="gauge-row">
        ${gaugeHtml("CPU", sys.cpu_percent)}
        ${gaugeHtml("内存", sys.mem_percent)}
      </div>

      <div class="battery-block ${batteryTone}">
        <div class="battery-head">
          <span class="battery-label">电量</span>
          <span class="battery-value num">${escapeHtml(batteryText)}</span>
        </div>
        <div class="battery-track">
          <div class="battery-fill" style="width:${batteryPct}%"></div>
        </div>
      </div>

      <div class="kv-list">
        ${kvHtml("导航栈", navm.state || "未连接", navTone, navDot)}
        ${kvHtml("定位", pose ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}` : "无 TF",
                 pose ? "" : "is-dim", pose ? "ok" : "")}
        ${kvHtml("朝向", pose ? `${pose.yaw_deg.toFixed(0)}°` : "—", pose ? "" : "is-dim")}
        ${kvHtml("地图", (s.current_map && s.current_map.base_name) || "—",
                 s.current_map ? "" : "is-dim")}
        ${kvHtml("已运行", fmtDuration(sys.uptime_sec))}
      </div>

      <div class="status-foot">
        <span>${escapeHtml(sys.ip || "—")}</span>
        <span>${escapeHtml(sys.hostname || "")}</span>
      </div>
    `;
  }

  // ── 地图 ──
  // 列表与详情并到同一屏：左边选，右边立刻预览，不再跳二级页、不再上下滚。
  function renderMap(sub, rest) {
    startPolling();
    if (sub === "edit") return renderMapEdit(rest || []);
    const active = sub === "mapping" ? "mapping" : "list";
    const opts = {
      back: "home", title: "地图", sub: "建图 · 编辑",
      tabs: { base: "map", items: [["list", "地图列表"], ["mapping", "建图"]] }, active,
    };
    if (active === "mapping") return renderMapping(opts);

    shell(opts, "cols-side-main", `
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">地图列表</div></div>
        <div class="pane-body scroll" id="map-list"><div class="center-text">加载中…</div></div>
      </section>
      <section class="pane" id="map-detail">
        <div class="pane-head"><div class="eyebrow" id="map-detail-title">地图预览</div><span class="hint" id="map-detail-meta"></span></div>
        <div class="pane-body" style="display:flex"><div class="map-wrap" id="map-canvas-wrap"><div class="center-text">请选择左侧地图</div></div></div>
        <div class="pane-foot" id="map-detail-actions"></div>
      </section>
    `);
    loadMapList();
  }

  // ── 建图 ──
  // 中间是 super-lio 点云实时攒出来的栅格图（看着地图长出来），
  // 左边建图状态，右边三个动作，下面两个摇杆边走边建。
  let liveMapGeo = null;   // 最近一次实时地图的几何信息，供状态栏显示范围
  let liveMapView = null;  // 2D 栅格视图句柄，切标签时要通知它重新量尺寸

  function mappingStatusHtml(live) {
    const navm = (state.status && state.status.navigation_manager) || {};
    const pose = state.status && state.status.pose;
    const l = live || {};
    // 注意：cell_count × 分辨率² 是"被占据的栅格面积"（也就是墙面），
    // 不等于走过的场地大小，所以这里显示地图外接范围而不是那个数字。
    const extent = liveMapGeo
      ? `${(liveMapGeo.width * liveMapGeo.resolution).toFixed(1)} × ${(liveMapGeo.height * liveMapGeo.resolution).toFixed(1)} m`
      : "—";
    return `
      ${l.cloud_frame && l.tf_ok === false ? `
        <div class="task-block tone-crit" style="margin-bottom:10px">
          <div class="task-label">点云无法入图</div>
          <div class="task-value" style="font-size:var(--fs-md)">坐标系 ${escapeHtml(l.cloud_frame)}</div>
          <div class="task-sub" style="white-space:normal">${escapeHtml(l.tf_error || "")}<br>
            已丢弃 ${l.dropped_no_tf || 0} 帧。点云在传感器坐标系，没有定位就无法拼成地图。</div>
        </div>` : ""}
      <div class="kv-list">
        ${kvHtml("运行模式", navm.mode || "—", navm.mode === "mapping" ? "is-ok" : "is-dim",
                 navm.mode === "mapping" ? "ok" : "")}
        ${kvHtml("点云坐标系", l.cloud_frame || "—", l.tf_ok ? "is-ok" : "is-crit")}
        ${kvHtml("栈状态", navm.state || "—", navm.ready ? "is-ok" : "is-warn")}
        ${kvHtml("点云", l.streaming ? "接收中" : "无数据", l.streaming ? "is-ok" : "is-crit",
                 l.streaming ? "ok" : "bad")}
        ${kvHtml("点云帧", l.cloud_count != null ? String(l.cloud_count) : "—")}
        ${kvHtml("已建栅格", l.cell_count != null ? String(l.cell_count) : "—")}
        ${kvHtml("地图范围", extent)}
        ${kvHtml("当前位姿", pose ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}` : "无位姿",
                 pose ? "" : "is-dim")}
        ${kvHtml("位姿来源", pose ? (pose.source === "lio" ? "super-lio 里程计" : "TF") : "—",
                 pose ? "" : "is-dim")}
        ${kvHtml("LIO 高度 z", pose && pose.z != null ? `${pose.z.toFixed(2)} m` : "—",
                 pose && pose.z != null ? "" : "is-dim")}
        ${kvHtml("雷达姿态",
                 pose && pose.pitch_deg != null
                   ? `俯仰 ${pose.pitch_deg.toFixed(1)}° · 横滚 ${pose.roll_deg.toFixed(1)}°`
                   : "—",
                 pose && pose.pitch_deg != null ? "" : "is-dim")}
        ${kvHtml("分辨率", l.resolution ? l.resolution + " m/px" : "—")}
      </div>
      <p class="note">开始建图后用下方摇杆把场地走一遍；停止建图会保存点云，再点「保存地图」生成 2D 栅格快照，之后在「地图列表」里激活。</p>
      <p class="note">「LIO 高度 z」在平地上应等于雷达装机高度；「雷达姿态」在机器人直立站定时
      就是雷达的安装角。两者都用于核对 <code>lio.extrinsic.odom_robo</code>。</p>
    `;
  }

  function defaultMapName() {
    const d = new Date();
    const p = (n) => String(n).padStart(2, "0");
    return `map_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`;
  }

  // 四个按钮的可用性只由「在不在建图」决定，每次状态刷新都重算一遍。
  // 保存和清空都要读 map.pcd，而 super-lio 只在停止建图时才把点云落盘，
  // 所以建图过程中这两个也锁上 —— 不然存下来的是上一次的图。
  function syncMappingButtons() {
    const navm = (state.status && state.status.navigation_manager) || {};
    const known = !!navm.mode;
    const mapping = navm.mode === "mapping";
    const set = (id, disabled) => {
      const el = document.getElementById(id);
      if (el) el.disabled = disabled;
    };
    set("map-start", !known || mapping);
    set("map-stop", !known || !mapping);
    set("map-save", !known || mapping);
    set("live-reset", !known || mapping);
    const hint = document.getElementById("map-ctl-hint");
    if (hint) {
      hint.textContent = !known
        ? "读取状态…"
        : mapping
          ? "建图中：走完场地后点「停止建图」"
          : "保存地图 = 点云转 2D pgm 快照";
    }
  }

  function renderMapping(opts) {
    shell(opts, "cols-map-work", `
      <aside class="pane work-a">
        <div class="pane-head"><div class="eyebrow">建图状态</div></div>
        <div class="pane-body scroll" id="mapping-status">${mappingStatusHtml(null)}</div>
      </aside>
      ${stickPaneHtml("move")}
      <section class="pane robot-pane work-c" style="background:var(--surface)">
        <div class="pane-head">
          <div class="eyebrow">实时地图</div>
          <div class="sb-tabs" id="live-view-tabs">
            <button class="active" data-live-view="cloud">3D 点云</button>
            <button data-live-view="grid">2D 栅格</button>
          </div>
          <span class="hint" id="live-map-hint">等待点云…</span>
        </div>
        <div class="pane-body flush" style="display:flex;position:relative">
          <div class="cloud-stage" id="live-cloud"></div>
          <div class="map-wrap" id="live-map" hidden><div class="center-text">等待点云数据…</div></div>
          <div class="cloud-legend" id="cloud-legend" hidden>
            <span class="cloud-legend-bar"></span>
            <span class="cloud-legend-txt"><b id="legend-hi">—</b> 高<br><b id="legend-lo">—</b> 低</span>
          </div>
        </div>
      </section>
      <aside class="pane work-d">
        <div class="pane-head"><div class="eyebrow">建图控制</div></div>
        <div class="pane-body" style="display:flex;flex-direction:column;gap:10px">
          <button class="btn success block" id="map-start" disabled>开始建图</button>
          <button class="btn warn block" id="map-stop" disabled>停止建图</button>
          <button class="btn primary block" id="map-save" disabled>保存地图</button>
          <button class="btn ghost block" id="live-reset" disabled>清空点云</button>
        </div>
        <div class="pane-foot"><span class="note" style="margin:0" id="map-ctl-hint">读取状态…</span></div>
      </aside>
      ${stickPaneHtml("turn")}
    `);

    // 开始/停止：成功后先按预期把按钮翻过来，1.5 秒后的状态轮询会再校正一次。
    // 不这么做的话点完按钮要愣一下才变灰，现场会以为没点上又点一次。
    const bindMode = (id, path, msg, optimistic) => document.getElementById(id)
      .addEventListener("click", async () => {
        const el = document.getElementById(id);
        el.disabled = true;                            // 防连点
        try {
          await guarded(() => api(path, { method: "POST" }), msg);
          const navm = (state.status && state.status.navigation_manager) || {};
          navm.mode = optimistic;
          if (state.status) state.status.navigation_manager = navm;
        } catch (_e) { /* toast 里已经报过了 */ }
        syncMappingButtons();
      });
    bindMode("map-start", "/api/map/start_mapping", "已开始建图", "mapping");
    bindMode("map-stop", "/api/map/stop_mapping", "已停止建图，点云保存中", "navigation");

    // 保存地图：起个名字 → 后端把 map.pcd 投成 2D 栅格存进地图目录
    document.getElementById("map-save").addEventListener("click", async () => {
      const name = prompt("地图名称（字母 / 数字 / 下划线）：", defaultMapName());
      if (name === null) return;                       // 用户取消
      const res = await guarded(
        () => api("/api/map/save", { method: "POST", body: { name: name.trim() } }),
        "地图已保存",
      ).catch(() => null);
      if (res) nav("map/list");                        // 直接跳去列表，省得自己找
    });

    // 清空点云：连同本次建图落盘的 pcd 一起删，否则下次保存会存到旧点云
    document.getElementById("live-reset").addEventListener("click", async () => {
      if (!confirm("清空点云？\n\n会删除本次建图产生的 map.pcd 和分片文件。\n已保存进地图列表的地图不受影响。")) return;
      await guarded(
        () => api("/api/map/live/reset", { method: "POST", body: { purge_pcd: true } }),
        "点云已清空",
      ).catch(() => null);
      if (mapCloud) mapCloud.clear();
    });

    syncMappingButtons();

    mountTeleopSticks();
    startLiveCloud();
    startLiveMapLoop();

    // 3D / 2D 切换：两边都在后台跑，切过去立刻有内容
    app.querySelectorAll("[data-live-view]").forEach((btn) => btn.addEventListener("click", () => {
      const mode = btn.getAttribute("data-live-view");
      app.querySelectorAll("[data-live-view]").forEach((b) => b.classList.toggle("active", b === btn));
      const cloud = document.getElementById("live-cloud");
      const grid = document.getElementById("live-map");
      const legend = document.getElementById("cloud-legend");
      cloud.hidden = mode !== "cloud";
      legend.hidden = mode !== "cloud";
      grid.hidden = mode === "cloud";
      // 两边都要在"刚变可见"时重新量尺寸 —— 隐藏期间 clientWidth/Height 是 0，
      // 画布建出来就是 0×0，不补这一下就是一块白板。
      if (mode === "cloud") { if (mapCloud) mapCloud.resize(); }
      else if (liveMapView) liveMapView.resize();
    }));
  }

  // ── 建图 3D 点云 ──
  let mapCloud = null;
  function startLiveCloud() {
    const host = document.getElementById("live-cloud");
    if (!host || !window.G1MapCloud) return;
    document.getElementById("cloud-legend").hidden = false;
    mapCloud = window.G1MapCloud.create(host, {
      baseUrl: state.baseUrl,
      onStats: ({ shown, total, zmin, zmax, ground }) => {
        // 一律显示离地高度。ground 拿不到时退回原始 z，并在图例上标明。
        const g = (ground === null || ground === undefined || !isFinite(ground)) ? null : ground;
        const base = g === null ? 0 : g;
        const lo = zmin - base;
        const hi = zmax - base;
        const hint = document.getElementById("live-map-hint");
        // 提示条归当前视图用；切到 2D 时别再抢着写点云的数字
        if (hint && !host.hidden) {
          hint.textContent = total
            ? `${shown.toLocaleString()} / ${total.toLocaleString()} 体素 · ${g === null ? "高度" : "离地"} ${lo.toFixed(1)}~${hi.toFixed(1)} m`
            : "等待点云…";
        }
        const loEl = document.getElementById("legend-lo");
        const hiEl = document.getElementById("legend-hi");
        if (loEl && total) { loEl.textContent = `${lo.toFixed(1)}m`; hiEl.textContent = `${hi.toFixed(1)}m`; }
      },
    });
    if (!mapCloud) return;
    // 机器人位姿跟着状态轮询走，顺便画出走过的轨迹
    const poseTimer = setInterval(() => {
      if (!mapCloud) return;
      mapCloud.setPose((state.status && state.status.pose) || null);
    }, 500);
    onPageLeave(() => {
      clearInterval(poseTimer);
      if (mapCloud) { mapCloud.dispose(); mapCloud = null; }
    });
  }

  // 实时地图：每 2 秒拉一张 PNG，几何信息在响应头里，一次请求就够
  function startLiveMapLoop() {
    const wrap = document.getElementById("live-map");
    const hint = document.getElementById("live-map-hint");
    let view = null;
    let busy = false;
    let stopped = false;

    async function tick() {
      if (busy || stopped) return;
      busy = true;
      try {
        const info = await api("/api/map/live");
        const statusEl = document.getElementById("mapping-status");
        if (statusEl) statusEl.innerHTML = mappingStatusHtml(info);
        syncMappingButtons();
        // 提示条归当前显示的那个视图用：3D 时由点云的 onStats 写，别互相覆盖
        const gridVisible = !document.getElementById("live-map").hidden;
        if (hint && (gridVisible || !info.streaming)) {
          hint.textContent = info.streaming
            ? `${info.cell_count} 格 · ${info.topic}`
            : `无点云（${info.topic}）`;
        }
        const res = await fetch(`${state.baseUrl}/api/map/live/image?t=${Date.now()}`, { cache: "no-store" });
        if (!res.ok) {
          if (!view) wrap.innerHTML = `<div class="center-text">暂无点云数据<br><small>确认已开始建图，且 super-lio 正在发布点云</small></div>`;
          return;
        }
        const geo = JSON.parse(res.headers.get("X-Map-Geometry") || "null");
        if (geo) liveMapGeo = geo;
        const blob = await res.blob();
        if (stopped) return;
        if (!view) {
          view = await mountMapView(wrap, { imageBlob: blob, geo });
          liveMapView = view;
          if (view) onPageLeave(() => {
            liveMapView = null;
            view.destroy && view.destroy();
          });
          // 挂载时这个面板多半还藏在 3D 后面，尺寸是 0。若此刻正好可见，
          // 补一次 resize 把 0×0 的画布撑开。
          if (view && !wrap.hidden) view.resize();
        } else {
          const url = URL.createObjectURL(blob);
          view.setImage(url, geo);
          setTimeout(() => URL.revokeObjectURL(url), 4000);
        }
      } catch (_e) {
        /* 网络抖动就等下一轮 */
      } finally {
        busy = false;
      }
    }

    tick();
    const timer = setInterval(tick, 2000);
    onPageLeave(() => { stopped = true; clearInterval(timer); });
  }

  // 摇杆各自占住左右两列的下半屏：紧挨着上面的姿态 / 转向面板，
  // 中间那列让给画面通栏，底部不再留空，摇杆也能撑到侧栏满宽。
  function stickPaneHtml(kind) {
    const isMove = kind === "move";
    return `
      <section class="pane stick-pane ${isMove ? "work-b" : "work-e"}">
        <div class="pane-head">
          <div class="eyebrow">${isMove ? "行进摇杆" : "转向摇杆"}</div>
          <span class="hint">${isMove ? "前后左右 · 松手即停" : "左右转 · 断连自动停"}</span>
        </div>
        <div class="pane-body stick-body"><div class="stick" id="stick-${kind}"></div></div>
      </section>`;
  }

  // 两个摇杆 + 10Hz 速度循环，建图页与移动控制页共用
  function mountTeleopSticks() {
    const moveEl = document.getElementById("stick-move");
    const turnEl = document.getElementById("stick-turn");
    if (!moveEl || !turnEl) return;
    const cmd = { vx: 0, vy: 0, wz: 0 };
    // 屏幕坐标下为正 → 机器人前进为 -y；屏幕右为正 → 机器人左移(+vy)为 -x
    createJoystick(moveEl, { onChange: (v) => { cmd.vx = -v.y; cmd.vy = -v.x; } });
    createJoystick(turnEl, { onChange: (v) => { cmd.wz = -v.x; } });
    const loop = startTeleopLoop(() => cmd);
    onPageLeave(() => loop.stop());
  }

  async function loadMapList() {
    const listEl = document.getElementById("map-list");
    let maps;
    try { maps = await api("/api/maps"); }
    catch (err) { listEl.innerHTML = `<div class="center-text">加载失败: ${escapeHtml(err.message)}</div>`; return; }
    state.mapsCache = maps;
    if (!maps.length) { listEl.innerHTML = `<div class="center-text">暂无地图</div>`; return; }
    if (!maps.some((m) => m.id === state.selectedMapId)) {
      state.selectedMapId = (maps.find((m) => m.is_active) || maps[0]).id;
    }
    listEl.innerHTML = `<div class="list">${maps.map((m) => `
      <div class="list-item ${m.id === state.selectedMapId ? "selected" : ""}" data-select="${escapeHtml(m.id)}">
        ${m.pgm_exists ? `<img class="thumb" src="${mapImageUrl(m.id, m.mtime)}" loading="lazy" />` : ""}
        <div class="info">
          <div class="name">${escapeHtml(m.label)} ${m.is_active ? '<span class="badge">当前</span>' : ""}</div>
          <div class="meta">${m.source === "hall" ? "场馆预设" : m.source === "active" ? "使用中" : "历史快照"} · ${new Date(m.mtime * 1000).toLocaleDateString()}</div>
        </div>
      </div>`).join("")}</div>`;
    listEl.querySelectorAll("[data-select]").forEach((el) => el.addEventListener("click", () => {
      state.selectedMapId = el.getAttribute("data-select");
      listEl.querySelectorAll("[data-select]").forEach((x) => x.classList.toggle("selected", x === el));
      showMapDetail(state.selectedMapId);
    }));
    showMapDetail(state.selectedMapId);
  }

  async function showMapDetail(mapId) {
    const wrap = document.getElementById("map-canvas-wrap");
    const titleEl = document.getElementById("map-detail-title");
    const metaEl = document.getElementById("map-detail-meta");
    const actEl = document.getElementById("map-detail-actions");
    if (!wrap) return;
    const entry = (state.mapsCache || []).find((m) => m.id === mapId);
    wrap.innerHTML = `<div class="center-text">加载中…</div>`;
    actEl.innerHTML = "";
    let info;
    try { info = await api(`/api/maps/${encodeURIComponent(mapId)}`); }
    catch (err) { wrap.innerHTML = `<div class="center-text">加载失败: ${escapeHtml(err.message)}</div>`; return; }
    titleEl.textContent = info.label || "地图预览";
    metaEl.textContent = info.geometry
      ? `${info.geometry.resolution} m/px · ${info.geometry.width}×${info.geometry.height}`
      : "无栅格图";

    actEl.innerHTML = `
      ${entry && !entry.is_active ? `<button class="btn success" id="map-activate">设为当前</button>` : ""}
      ${entry && entry.source !== "hall" && info.geometry
          ? `<button class="btn" data-nav="map/edit/${encodeURIComponent(entry.id)}">编辑地图</button>` : ""}
      ${entry && entry.source !== "active" ? `<button class="btn danger" id="map-delete">删除</button>` : ""}
      <span class="note" style="margin:0 0 0 auto">巡航点坐标请在「导航 › 多点巡航」中编辑</span>
    `;
    const actBtn = document.getElementById("map-activate");
    if (actBtn) actBtn.addEventListener("click", async () => {
      await guarded(() => api(`/api/maps/${encodeURIComponent(mapId)}/activate`, { method: "POST" }), "已切换当前地图，请在设置中重启导航栈");
      loadMapList();
    });
    const delBtn = document.getElementById("map-delete");
    if (delBtn) delBtn.addEventListener("click", async () => {
      if (!confirm("确认删除该地图？此操作不可撤销。")) return;
      await guarded(() => api(`/api/maps/${encodeURIComponent(mapId)}`, { method: "DELETE" }), "已删除");
      state.selectedMapId = null;
      loadMapList();
    });

    if (!info.geometry) { wrap.innerHTML = `<div class="center-text">该地图没有 2D 栅格文件</div>`; return; }
    await mountMapView(wrap, {
      imageUrl: `${state.baseUrl}/api/maps/${encodeURIComponent(mapId)}/image`,
      geo: info.geometry,
    });
  }

  // ── 可复用地图画布：显示 PNG + 可选路点叠加 + 点击拾取世界坐标 ──
  function pixelFromWorld(geo, x, y) {
    const col = (x - geo.origin[0]) / geo.resolution;
    const rowFromBottom = (y - geo.origin[1]) / geo.resolution;
    const rowFromTop = geo.height - rowFromBottom;
    return { px: col / geo.render_scale, py: rowFromTop / geo.render_scale };
  }
  function worldFromPixel(geo, px, py) {
    const col = px * geo.render_scale;
    const rowFromTop = py * geo.render_scale;
    const rowFromBottom = geo.height - rowFromTop;
    return { x: geo.origin[0] + col * geo.resolution, y: geo.origin[1] + rowFromBottom * geo.resolution };
  }

  // 可平移 / 缩放 / 旋转的地图视图。
  // 视图状态是 {scale, tx, ty, rot}，绘制走 canvas 变换，点选再做一次逆变换
  // 回到图像像素 → 世界坐标，所以转过、缩放过之后点选依然准。
  // 地图上的颜色一律从 CSS 变量取，改主题时不会出现"界面换了、地图没换"的脏对比
  const FONT_UI = '-apple-system, "PingFang SC", "Helvetica Neue", sans-serif';
  const cssVar = (name, fallback) => {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
  };
  // 当前是不是夜间。不缓存：系统主题随时可能变，而这个查询很便宜。
  function isDarkTheme() {
    return !!(window.matchMedia
      && window.matchMedia("(prefers-color-scheme: dark)").matches);
  }

  function readMapColors() {
    return {
      robot: cssVar("--ok", "#17836a"),
      goal: cssVar("--crit", "#b8453f"),
      reloc: cssVar("--warn", "#96701f"),
      waypoint: cssVar("--signal", "#2b87cc"),
      waypointLine: "rgba(43, 135, 204, .45)",
      path: cssVar("--signal", "#2b87cc"),
      wedge: "rgba(23, 131, 106, .20)",
      placeHalo: "rgba(184, 69, 63, .16)",
      ink: cssVar("--ink", "#16202b"),
      // 实时激光：用边缘色青绿，跟静态地图的黑白灰拉开，一眼分得清
      // "地图里记着的障碍物" 和 "此刻真的挡在前面的东西"
      scan: cssVar("--edge", "#1fa89f"),
      // 禁行区：半透明红块 + 实线边。和地图本身的纯黑障碍拉开，
      // 不然改完看不出哪块是自己划的、哪块是雷达扫到的。
      zoneFill: "rgba(184, 69, 63, .28)",
      zoneLine: cssVar("--crit", "#b8453f"),
      brush: cssVar("--signal", "#2b87cc"),
    };
  }

  let MAP_COLORS = readMapColors();

  // 系统主题切换时把地图配色重算一遍。
  // cssVar 是加载时读一次的，不重算的话白天设好的深色点会留在夜间界面上，
  // 而夜间的亮色点在白天又会糊成一片——这类"只在切主题时才出现"的问题
  // 现场几乎不可能想到去查。
  if (window.matchMedia) {
    const themeQuery = window.matchMedia("(prefers-color-scheme: dark)");
    const onThemeChange = () => {
      MAP_COLORS = readMapColors();
      if (navUi.view) navUi.view.redraw();
      if (mapEdit.view) mapEdit.view.redraw();
      // 3D 点云视图不用管：它是固定的深色舞台（mapcloud.js 里写死的
      // 0x0f1720），两个主题下都成立，本来就不跟随。
    };
    if (themeQuery.addEventListener) themeQuery.addEventListener("change", onThemeChange);
    else if (themeQuery.addListener) themeQuery.addListener(onThemeChange);
  }
  const HOLD_MS = 320;   // 长按多久算"放置"，短于这个就是普通拖动地图

  function mountMapView(wrapEl, {
    imageUrl, imageBlob, geo, waypoints = [], marks = [],
    onPick = null,          // 兼容旧用法：单击取点
    onPlace = null,         // 长按放置：{x, y, yaw_deg|null}
    onWaypointMove = null,  // 直接在图上拖动巡航点
    onEditStroke = null,    // 编辑：画完一笔，给世界坐标点列
    onEditRect = null,      // 编辑：拉完一个框，给 {x,y,w,h}
  } = {}) {
    return new Promise((resolve) => {
      // 地图要双指缩放旋转，得让全局的防缩放拦截放行这一块
      wrapEl.setAttribute("data-allow-gesture", "");
      wrapEl.innerHTML = `
        <canvas class="map-canvas"></canvas>
        <div class="map-tools">
          <button class="map-tool" data-map-tool="fit" title="适应窗口">⤢</button>
          <button class="map-tool" data-map-tool="rot-l" title="逆时针旋转">↺</button>
          <button class="map-tool" data-map-tool="rot-r" title="顺时针旋转">↻</button>
        </div>
        <div class="map-hint">拖动平移 · 双指缩放旋转 · <b>长按地图放点，拖出朝向</b></div>
      `;
      const canvas = wrapEl.querySelector("canvas");
      const ctx = canvas.getContext("2d");
      const img = new Image();
      const view = { scale: 1, tx: 0, ty: 0, rot: 0 };
      const pointers = new Map();
      let pinch = null;
      let moved = 0;
      let objectUrl = null;
      let path = [];             // 世界坐标点列，Nav2 规划出来的路径
      let scan = [];             // 世界坐标点列，/scan 投出来的实时障碍点
      let zones = [];            // 禁行区矩形（世界坐标），编辑页用
      let editMode = null;       // null=不编辑 | "paint" | "zone" | "crop"
      let editStroke = [];       // 正在画的这一笔（世界坐标）
      let editRect = null;       // 正在拖的矩形 {x0,y0,x1,y1} 世界坐标
      let editBrushRadius = 0.1; // 画笔半径（米），编辑页调
      let placing = null;        // 长按放置中：{ px, py, world, yaw_deg, cur }
      let holdTimer = null;
      let dragWp = -1;           // 正在拖动的巡航点下标
      let fitted = false;        // 是否已在"真的可见"的状态下自适应过，见 fit()

      function resize() {
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = wrapEl.clientWidth, h = wrapEl.clientHeight;
        if (!w || !h) return;
        canvas.width = Math.round(w * dpr);
        canvas.height = Math.round(h * dpr);
        canvas.style.width = w + "px";
        canvas.style.height = h + "px";
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        draw();
      }

      function fit() {
        const w = wrapEl.clientWidth || 1, h = wrapEl.clientHeight || 1;
        const iw = img.naturalWidth || geo.render_width || 1;
        const ih = img.naturalHeight || geo.render_height || 1;
        view.scale = Math.min(w / iw, h / ih) * 0.94;
        view.tx = 0; view.ty = 0; view.rot = 0;
        // 元素还隐藏着的话上面拿到的是 0（这里兜成了 1），算出来的 scale 没意义。
        // 标记一下，等真正显示出来再补一次自适应。
        fitted = wrapEl.clientWidth > 0 && wrapEl.clientHeight > 0;
        draw();
      }

      // 图像像素 → 画布 CSS 坐标
      function toCanvas(px, py) {
        const iw = img.naturalWidth || geo.render_width;
        const ih = img.naturalHeight || geo.render_height;
        const cx = (wrapEl.clientWidth || 0) / 2 + view.tx;
        const cy = (wrapEl.clientHeight || 0) / 2 + view.ty;
        const dx = (px - iw / 2) * view.scale;
        const dy = (py - ih / 2) * view.scale;
        return {
          x: cx + dx * Math.cos(view.rot) - dy * Math.sin(view.rot),
          y: cy + dx * Math.sin(view.rot) + dy * Math.cos(view.rot),
        };
      }
      // 画布 CSS 坐标 → 图像像素（上面的逆运算）
      function toImage(x, y) {
        const iw = img.naturalWidth || geo.render_width;
        const ih = img.naturalHeight || geo.render_height;
        const cx = (wrapEl.clientWidth || 0) / 2 + view.tx;
        const cy = (wrapEl.clientHeight || 0) / 2 + view.ty;
        const dx = x - cx, dy = y - cy;
        const rx = dx * Math.cos(-view.rot) - dy * Math.sin(-view.rot);
        const ry = dx * Math.sin(-view.rot) + dy * Math.cos(-view.rot);
        return { px: rx / view.scale + iw / 2, py: ry / view.scale + ih / 2 };
      }

      // 世界系 yaw 逆时针为正、图像 y 向下，再叠加视图自身的旋转
      const screenAngle = (yawDeg) => -(yawDeg || 0) * Math.PI / 180 + view.rot;
      const worldPoint = (x, y) => {
        const { px, py } = pixelFromWorld(geo, x, y);
        return toCanvas(px, py);
      };

      function drawArrow(p, yawDeg, color, radius) {
        ctx.beginPath();
        ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 2;
        ctx.stroke();
        const a = screenAngle(yawDeg);
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + Math.cos(a) * radius * 2.4, p.y + Math.sin(a) * radius * 2.4);
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.stroke();
      }

      // 机器人：圆点 + 朝向扇形，一眼看出"头朝哪"
      function drawRobot(p, yawDeg, color) {
        const a = screenAngle(yawDeg);
        const span = 0.42;
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.arc(p.x, p.y, 30, a - span, a + span);
        ctx.closePath();
        ctx.fillStyle = MAP_COLORS.wedge;
        ctx.fill();
        ctx.beginPath();
        ctx.arc(p.x, p.y, 8, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 2.5;
        ctx.stroke();
      }

      // 巡航点：带序号的圆牌，比纯色点好认，也好点中
      function drawBadge(p, idx, yawDeg, color) {
        if (yawDeg != null) {
          const a = screenAngle(yawDeg);
          ctx.beginPath();
          ctx.moveTo(p.x, p.y);
          ctx.lineTo(p.x + Math.cos(a) * 22, p.y + Math.sin(a) * 22);
          ctx.strokeStyle = color;
          ctx.lineWidth = 3;
          ctx.stroke();
        }
        ctx.beginPath();
        ctx.arc(p.x, p.y, 12, 0, Math.PI * 2);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 2.5;
        ctx.stroke();
        ctx.fillStyle = "#fff";
        ctx.font = "700 12px " + FONT_UI;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(idx + 1), p.x, p.y + 0.5);
        ctx.textAlign = "start";
        ctx.textBaseline = "alphabetic";
      }

      // 规划路径：白色描边打底，保证在深浅栅格上都看得清
      function drawPath() {
        if (path.length < 2) return;
        ctx.beginPath();
        path.forEach(([x, y], i) => {
          const p = worldPoint(x, y);
          if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
        });
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.strokeStyle = "rgba(255,255,255,.85)";
        ctx.lineWidth = 7;
        ctx.stroke();
        ctx.strokeStyle = MAP_COLORS.path;
        ctx.lineWidth = 3.5;
        ctx.stroke();
      }

      // 实时激光点。点很多（一圈 180 个），用 fillRect 而不是 arc：
      // arc 每个点都要走一次路径栈，机器人转身时会明显掉帧。
      function drawScan() {
        if (!scan.length) return;
        const size = Math.max(1.5, Math.min(3, 2 * view.scale));
        ctx.fillStyle = MAP_COLORS.scan;
        ctx.globalAlpha = 0.85;
        for (let i = 0; i < scan.length; i++) {
          const p = worldPoint(scan[i][0], scan[i][1]);
          ctx.fillRect(p.x - size / 2, p.y - size / 2, size, size);
        }
        ctx.globalAlpha = 1;
      }

      // 禁行区与编辑中的图形。禁行区用半透明红块 + 实线边，
      // 和地图本身的纯黑障碍拉开——不然改完看不出哪块是自己划的。
      function drawZones() {
        const boxes = zones.map((z) => [z.x, z.y, z.w, z.h]);
        if (editRect) {
          boxes.push([
            Math.min(editRect.x0, editRect.x1), Math.min(editRect.y0, editRect.y1),
            Math.abs(editRect.x1 - editRect.x0), Math.abs(editRect.y1 - editRect.y0),
          ]);
        }
        if (!boxes.length) return;
        ctx.save();
        boxes.forEach(([x, y, w, h]) => {
          const a = worldPoint(x, y);           // 左下
          const b = worldPoint(x + w, y + h);   // 右上
          const rx = Math.min(a.x, b.x), ry = Math.min(a.y, b.y);
          const rw = Math.abs(b.x - a.x), rh = Math.abs(b.y - a.y);
          ctx.fillStyle = MAP_COLORS.zoneFill;
          ctx.fillRect(rx, ry, rw, rh);
          ctx.strokeStyle = MAP_COLORS.zoneLine;
          ctx.lineWidth = 2;
          ctx.strokeRect(rx, ry, rw, rh);
        });
        ctx.restore();
      }

      // 正在画的这一笔实时描出来，松手前就知道会涂到哪
      function drawStroke() {
        if (editStroke.length < 1) return;
        ctx.save();
        ctx.strokeStyle = MAP_COLORS.brush;
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        ctx.lineWidth = Math.max(2, (editBrushRadius * 2 / geo.resolution)
                                    / geo.render_scale * view.scale);
        ctx.globalAlpha = 0.55;
        ctx.beginPath();
        editStroke.forEach((pt, i) => {
          const p = worldPoint(pt[0], pt[1]);
          if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
        });
        if (editStroke.length === 1) {
          const p = worldPoint(editStroke[0][0], editStroke[0][1]);
          ctx.arc(p.x, p.y, ctx.lineWidth / 2, 0, Math.PI * 2);
        }
        ctx.stroke();
        ctx.restore();
      }

      function draw() {
        const w = wrapEl.clientWidth, h = wrapEl.clientHeight;
        if (!w || !h) return;
        ctx.clearRect(0, 0, w, h);
        ctx.save();
        ctx.translate(w / 2 + view.tx, h / 2 + view.ty);
        ctx.rotate(view.rot);
        ctx.scale(view.scale, view.scale);
        const iw = img.naturalWidth || geo.render_width;
        const ih = img.naturalHeight || geo.render_height;
        ctx.imageSmoothingEnabled = view.scale < 1;
        // 夜间把底图反相：pgm 是白底(可通行)黑线(障碍)，直接摆在深色界面上
        // 是一大块刺眼的白。反相之后可通行变近黑、障碍变亮线、未知变深灰，
        // 三者关系不变，观感和 RViz/Foxglove 的夜间地图一致。
        // 只包住 drawImage —— 位姿、激光、禁行区画在同一个 canvas 上，
        // 用 CSS 滤镜的话它们的颜色会一起被改掉。
        if (isDarkTheme()) ctx.filter = "invert(1)";
        ctx.drawImage(img, -iw / 2, -ih / 2, iw, ih);
        ctx.filter = "none";
        ctx.restore();

        drawPath();
        drawScan();
        drawZones();
        drawStroke();

        // 巡航点之间连虚线，看得出巡航顺序
        if (waypoints.length > 1) {
          ctx.beginPath();
          waypoints.forEach((wp, i) => {
            const p = worldPoint(wp.x, wp.y);
            if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
          });
          ctx.setLineDash([6, 6]);
          ctx.strokeStyle = MAP_COLORS.waypointLine;
          ctx.lineWidth = 2;
          ctx.stroke();
          ctx.setLineDash([]);
        }
        waypoints.forEach((wp, idx) => {
          drawBadge(worldPoint(wp.x, wp.y), idx, wp.yaw_deg, MAP_COLORS.waypoint);
        });

        marks.forEach((mk) => {
          if (mk == null || mk.x == null) return;
          const p = worldPoint(mk.x, mk.y);
          if (mk.kind === "robot") drawRobot(p, mk.yaw_deg, mk.color || MAP_COLORS.robot);
          else drawArrow(p, mk.yaw_deg, mk.color || MAP_COLORS.goal, mk.radius || 8);
        });

        // 放置预览：锚点 + 拖出来的朝向，松手前就能看清会去哪、朝哪
        if (placing) {
          const p = toCanvas(placing.px, placing.py);
          ctx.beginPath();
          ctx.arc(p.x, p.y, 26, 0, Math.PI * 2);
          ctx.fillStyle = MAP_COLORS.placeHalo;
          ctx.fill();
          if (placing.yaw_deg != null) {
            const a = screenAngle(placing.yaw_deg);
            ctx.beginPath();
            ctx.moveTo(p.x, p.y);
            ctx.lineTo(p.x + Math.cos(a) * 46, p.y + Math.sin(a) * 46);
            ctx.strokeStyle = MAP_COLORS.goal;
            ctx.lineWidth = 4;
            ctx.stroke();
          }
          drawArrow(p, placing.yaw_deg, MAP_COLORS.goal, 9);
          ctx.fillStyle = MAP_COLORS.ink;
          ctx.font = "600 12px " + FONT_UI;
          ctx.fillText(
            placing.yaw_deg == null ? "拖出朝向（松手＝保持当前朝向）" : `${placing.yaw_deg.toFixed(0)}°`,
            p.x + 16, p.y - 16,
          );
        }
      }

      // ── 手势 ──
      const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
      const angle = (a, b) => Math.atan2(b.y - a.y, b.x - a.x);
      const local = (ev) => {
        const r = canvas.getBoundingClientRect();
        return { x: ev.clientX - r.left, y: ev.clientY - r.top };
      };

      const cancelHold = () => { if (holdTimer) { clearTimeout(holdTimer); holdTimer = null; } };
      const inImage = (px, py) => {
        const iw = img.naturalWidth || geo.render_width;
        const ih = img.naturalHeight || geo.render_height;
        return px >= 0 && py >= 0 && px <= iw && py <= ih;
      };
      // 命中检测：点在哪个巡航点圆牌上（半径放宽到 20，手指没那么准）
      const hitWaypoint = (pt) => {
        for (let i = waypoints.length - 1; i >= 0; i--) {
          const p = worldPoint(waypoints[i].x, waypoints[i].y);
          if (Math.hypot(p.x - pt.x, p.y - pt.y) <= 20) return i;
        }
        return -1;
      };

      canvas.addEventListener("pointerdown", (ev) => {
        canvas.setPointerCapture(ev.pointerId);
        const pt = local(ev);
        pointers.set(ev.pointerId, pt);
        moved = 0;
        if (pointers.size === 2) {
          cancelHold();
          placing = null;
          dragWp = -1;
          const [a, b] = [...pointers.values()];
          pinch = { dist: dist(a, b), angle: angle(a, b), scale: view.scale, rot: view.rot };
          draw();
          return;
        }
        // 编辑模式：单指按下就开始画/拉框。双指依然是缩放平移（上面已 return），
        // 所以编辑时也能自由缩放，不用来回切模式。
        if (editMode) {
          const { px, py } = toImage(pt.x, pt.y);
          if (!inImage(px, py)) return;
          const world = worldFromPixel(geo, px, py);
          if (editMode === "paint") {
            editStroke = [[world.x, world.y]];
          } else {
            editRect = { x0: world.x, y0: world.y, x1: world.x, y1: world.y };
          }
          draw();
          return;
        }
        // 先看是不是按在某个巡航点上——那就是要拖它，而不是拖地图
        if (onWaypointMove) {
          const hit = hitWaypoint(pt);
          if (hit >= 0) { dragWp = hit; return; }
        }
        // 单指按住不动 320ms＝放置目标点，之后拖动定朝向。
        // 这样地图平移永远自由，也不需要任何"取点模式"开关。
        if (onPlace) {
          holdTimer = setTimeout(() => {
            holdTimer = null;
            const { px, py } = toImage(pt.x, pt.y);
            if (!inImage(px, py)) return;
            placing = { px, py, world: worldFromPixel(geo, px, py), yaw_deg: null, start: pt };
            draw();
          }, HOLD_MS);
        }
      });

      canvas.addEventListener("pointermove", (ev) => {
        const prev = pointers.get(ev.pointerId);
        if (!prev) return;
        const cur = local(ev);
        pointers.set(ev.pointerId, cur);

        if (pointers.size >= 2 && pinch) {
          const [a, b] = [...pointers.values()];
          const d = dist(a, b);
          if (pinch.dist > 0) view.scale = Math.max(0.05, Math.min(24, pinch.scale * (d / pinch.dist)));
          view.rot = pinch.rot + (angle(a, b) - pinch.angle);
          moved = 99;
          draw();
          return;
        }

        if (editMode && (editStroke.length || editRect)) {
          const { px, py } = toImage(cur.x, cur.y);
          if (inImage(px, py)) {
            const world = worldFromPixel(geo, px, py);
            if (editMode === "paint") {
              const last = editStroke[editStroke.length - 1];
              // 采样点太密没意义，按 1/3 笔刷半径抽稀，省得一笔几千个点发上去
              if (!last || Math.hypot(world.x - last[0], world.y - last[1])
                           > editBrushRadius / 3) {
                editStroke.push([world.x, world.y]);
              }
            } else if (editRect) {
              editRect.x1 = world.x; editRect.y1 = world.y;
            }
            draw();
          }
          moved = 99;
          return;
        }

        if (dragWp >= 0) {
          const { px, py } = toImage(cur.x, cur.y);
          if (inImage(px, py)) {
            const world = worldFromPixel(geo, px, py);
            waypoints[dragWp].x = world.x;
            waypoints[dragWp].y = world.y;
            if (onWaypointMove) onWaypointMove(dragWp, world);
            draw();
          }
          return;
        }

        if (placing) {
          // 从锚点拉出方向；拉得太短说明用户没打算定朝向
          const dx = cur.x - toCanvas(placing.px, placing.py).x;
          const dy = cur.y - toCanvas(placing.px, placing.py).y;
          const len = Math.hypot(dx, dy);
          placing.yaw_deg = len < 14 ? null : -((Math.atan2(dy, dx) - view.rot) * 180 / Math.PI);
          draw();
          return;
        }

        moved += Math.hypot(cur.x - prev.x, cur.y - prev.y);
        if (moved > 8) cancelHold();      // 动了就说明是平移，不是长按
        view.tx += cur.x - prev.x;
        view.ty += cur.y - prev.y;
        draw();
      });

      const endPointer = (ev) => {
        const p = pointers.get(ev.pointerId);
        pointers.delete(ev.pointerId);
        if (pointers.size < 2) pinch = null;
        cancelHold();
        // 编辑模式：松手就把这一笔 / 这个框交给上层去提交
        if (editMode && (editStroke.length || editRect)) {
          const stroke = editStroke;
          const rect = editRect;
          editStroke = []; editRect = null;
          draw();
          if (editMode === "paint" && stroke.length && onEditStroke) {
            onEditStroke(stroke);
          } else if (rect && onEditRect) {
            const x = Math.min(rect.x0, rect.x1), y = Math.min(rect.y0, rect.y1);
            const w = Math.abs(rect.x1 - rect.x0), h = Math.abs(rect.y1 - rect.y0);
            // 太小的框多半是误触，忽略掉——不然会画出一堆看不见的禁行区
            if (w > 0.05 && h > 0.05) onEditRect({ x, y, w, h });
          }
          return;
        }
        if (dragWp >= 0) { dragWp = -1; return; }
        if (placing) {
          const world = placing.world;
          const yaw = placing.yaw_deg;
          placing = null;
          draw();
          if (onPlace) onPlace({ x: world.x, y: world.y, yaw_deg: yaw });
          return;
        }
        // 兼容旧的"单击取点"用法（本页没传 onPick 就什么都不做）
        if (p && moved < 7 && onPick && pointers.size === 0) {
          const { px, py } = toImage(p.x, p.y);
          if (inImage(px, py)) onPick(worldFromPixel(geo, px, py));
        }
      };
      canvas.addEventListener("pointerup", endPointer);
      canvas.addEventListener("pointercancel", (ev) => {
        pointers.delete(ev.pointerId);
        pinch = null; dragWp = -1; placing = null; cancelHold(); draw();
      });
      canvas.addEventListener("wheel", (ev) => {
        ev.preventDefault();
        view.scale = Math.max(0.05, Math.min(24, view.scale * (ev.deltaY > 0 ? 0.9 : 1.1)));
        draw();
      }, { passive: false });

      wrapEl.querySelectorAll("[data-map-tool]").forEach((btn) => {
        btn.addEventListener("click", (ev) => {
          ev.stopPropagation();
          const act = btn.getAttribute("data-map-tool");
          if (act === "fit") fit();
          else { view.rot += (act === "rot-l" ? -1 : 1) * Math.PI / 12; draw(); }
        });
      });

      const onWinResize = () => resize();
      window.addEventListener("resize", onWinResize);

      const handle = {
        redraw: draw,
        fit,
        // 从隐藏切到显示时必须叫一次：canvas 是按 clientWidth/Height 建的，
        // 元素隐藏时那两个值是 0，resize() 会直接 return，画布就一直是 0×0
        // ——建图页默认停在 3D 标签，2D 栅格就是这么变成白板的。
        resize: () => { resize(); if (!fitted) fit(); },
        setWaypoints(list) { waypoints = list || []; draw(); },
        setMarks(list) { marks = list || []; draw(); },
        setPath(points) { path = points || []; draw(); },
        setScan(points) { scan = points || []; draw(); },
        setZones(list) { zones = list || []; draw(); },
        setEditMode(mode, opts) {
          editMode = mode || null;
          if (opts && opts.radius != null) editBrushRadius = opts.radius;
          editStroke = []; editRect = null;
          draw();
        },
        setImage(url, newGeo) {
          if (newGeo) geo = newGeo;
          img.src = url;
        },
        destroy() {
          window.removeEventListener("resize", onWinResize);
          if (objectUrl) URL.revokeObjectURL(objectUrl);
        },
      };

      img.onload = () => { if (view.scale === 1 && !view.tx && !view.ty) fit(); else draw(); resolve(handle); };
      img.onerror = () => {
        wrapEl.innerHTML = `<div class="center-text">地图图片加载失败</div>`;
        resolve(null);
      };
      if (imageBlob) { objectUrl = URL.createObjectURL(imageBlob); img.src = objectUrl; }
      else img.src = imageUrl;
      requestAnimationFrame(resize);
    });
  }

  // ── 虚拟摇杆 ──
  // 返回归一化的 {x, y}（右/下为正，含死区）。页面自己决定映射到哪个轴。
  function createJoystick(el, { onChange } = {}) {
    el.innerHTML = `<div class="stick-base"><div class="stick-thumb"></div></div>`;
    const base = el.querySelector(".stick-base");
    const thumb = el.querySelector(".stick-thumb");
    const DEAD = 0.14;
    let active = null;
    let value = { x: 0, y: 0 };

    const emit = (x, y) => {
      const mag = Math.hypot(x, y);
      let out = { x: 0, y: 0 };
      if (mag > DEAD) {
        // 死区外重新归一化，摇杆刚出死区时不会突然给一个大速度
        const k = Math.min(1, (mag - DEAD) / (1 - DEAD)) / mag;
        out = { x: x * k, y: y * k };
      }
      if (out.x !== value.x || out.y !== value.y) {
        value = out;
        if (onChange) onChange(out);
      }
    };

    const move = (ev) => {
      const r = base.getBoundingClientRect();
      const radius = r.width / 2;
      let dx = (ev.clientX - (r.left + radius)) / radius;
      let dy = (ev.clientY - (r.top + radius)) / radius;
      const mag = Math.hypot(dx, dy);
      if (mag > 1) { dx /= mag; dy /= mag; }
      thumb.style.transform = `translate(${dx * radius * 0.62}px, ${dy * radius * 0.62}px)`;
      emit(dx, dy);
    };

    const release = () => {
      active = null;
      thumb.style.transform = "translate(0px, 0px)";
      base.classList.remove("active");
      emit(0, 0);
    };

    base.addEventListener("pointerdown", (ev) => {
      active = ev.pointerId;
      base.setPointerCapture(ev.pointerId);
      base.classList.add("active");
      move(ev);
    });
    base.addEventListener("pointermove", (ev) => { if (active === ev.pointerId) move(ev); });
    base.addEventListener("pointerup", (ev) => { if (active === ev.pointerId) release(); });
    base.addEventListener("pointercancel", release);
    base.addEventListener("pointerleave", (ev) => { if (active === ev.pointerId) release(); });

    return { get value() { return value; }, reset: release };
  }

  // 两个摇杆合成一条速度指令，10Hz 发出去；全零时只发一次，不刷接口。
  // 机器人侧 set_manual_velocity 带 0.4s deadline，所以这个循环一停机器人就停。
  function startTeleopLoop(getCommand) {
    let lastZero = true;
    let limits = { max_vx: 0.45, max_vy: 0.25, max_wz: 0.7 };
    api("/api/control/velocity/limits").then((l) => { limits = l; }).catch(() => {});
    const timer = setInterval(async () => {
      const c = getCommand();
      const vx = c.vx * limits.max_vx, vy = c.vy * limits.max_vy, wz = c.wz * limits.max_wz;
      const zero = Math.abs(vx) < 1e-3 && Math.abs(vy) < 1e-3 && Math.abs(wz) < 1e-3;
      if (zero && lastZero) return;
      lastZero = zero;
      try {
        await api("/api/control/velocity", { method: "POST", body: { vx, vy, wz } });
      } catch (err) {
        if (!zero) toast(String(err.message || err), "error");
      }
    }, 100);
    return {
      stop() {
        clearInterval(timer);
        api("/api/control/velocity", { method: "POST", body: { vx: 0, vy: 0, wz: 0 } }).catch(() => {});
      },
    };
  }

  // 当前地图的 id 恒为 "active"（网关 list_maps 里写死的），所以换了地图之后
  // 图片 URL 一模一样，浏览器直接给缓存 —— 现场表现为"切了地图导航页还是旧图，
  // 硬刷新才更新"。用 mtime 当版本号把缓存打掉。
  function mapImageUrl(mapId, mtime) {
    const v = mtime ? `?v=${Math.round(mtime * 1000)}` : "";
    return `${state.baseUrl}/api/maps/${encodeURIComponent(mapId)}/image${v}`;
  }

  async function pickActiveMapGeometry() {
    const maps = await api("/api/maps");
    const active = maps.find((m) => m.is_active) || maps[0];
    if (!active) return null;
    const info = await api(`/api/maps/${encodeURIComponent(active.id)}`);
    return {
      mapId: active.id,
      geometry: info.geometry,
      mtime: active.mtime || null,
      label: active.base_name || active.label || "",
    };
  }

  // ── 导航 ──
  // 单点与巡航共用同一套骨架：左边一张可平移/缩放/旋转的地图，右边一列操作。
  // 放点统一走"长按地图"，每个子页只有一种落点语义，不再有取点模式。
  const navUi = {
    target: null, reloc: null, waypoints: [], routeName: null,
    view: null, geo: null, progressBase: null, presets: null,
  };

  function renderNav(sub) {
    startPolling();
    const key = sub && sub[0];
    const active = key === "patrol" ? "patrol" : key === "reloc" ? "reloc" : "point";
    const opts = {
      back: "home", title: "导航", sub: "定点 · 巡航",
      tabs: { base: "nav", items: [["point", "单点导航"], ["patrol", "多点巡航"], ["reloc", "重定位"]] },
      active,
    };
    if (active === "patrol") return renderNavPatrol(opts);
    if (active === "reloc") return renderNavReloc(opts);
    return renderNavPoint(opts);
  }

  // 栈正在起落的中间态：两个按钮都锁上，等它落定
  const NAV_STACK_BUSY = new Set(["STARTING_LOCALIZATION", "STARTING_NAVIGATION", "RECOVERING"]);
  // 栈是停的：只能开
  const NAV_STACK_DOWN = new Set(["STOPPED", "ERROR"]);

  // 两个按钮的可用性完全由 navigation_manager 的状态机决定，每次状态轮询重算。
  // 之前两个按钮永远可点，栈已经起来了还能再点「开启」、正在启动时也能点，
  // 现场只能靠看日志才知道点没点上 —— 这正是「点了没反应」的由来。
  function syncNavStackButtons() {
    const on = document.getElementById("nav-stack-on");
    const off = document.getElementById("nav-stack-off");
    if (!on && !off) return;                           // 当前页面没有这组按钮

    const navm = (state.status && state.status.navigation_manager) || {};
    const st = navm.state || "";
    const busy = NAV_STACK_BUSY.has(st);
    const down = NAV_STACK_DOWN.has(st);
    const mapping = st === "MAPPING" || navm.mode === "mapping";

    let canOn, canOff, hint;
    if (!st) {
      canOn = false; canOff = false;
      hint = "读取导航栈状态…";
    } else if (mapping) {
      canOn = false; canOff = false;
      hint = "建图中：导航栈开关不可用，请先在建图页停止建图。";
    } else if (busy) {
      canOn = false; canOff = false;
      hint = "导航栈启动中，请稍候…";
    } else if (down) {
      canOn = true; canOff = false;                    // 已经是停的，没什么好关
      hint = st === "ERROR"
        ? "上次启动失败，可重新点「开启导航栈」重试；反复失败请看容器日志。"
        : "导航栈已停止。点「开启导航栈」拉起定位与 Nav2。";
    } else if (navm.ready) {
      canOn = false; canOff = true;                    // 已就绪，别再重复点开启
      hint = "导航栈已就绪。关闭会停掉定位与 Nav2，仅在排障或收工时使用。";
    } else {
      canOn = true; canOff = true;                     // DEGRADED_*：可重试也可停
      hint = "导航栈未完全就绪，可再点「开启导航栈」重试，或关闭后重来。";
    }

    if (on) on.disabled = !canOn;
    if (off) off.disabled = !canOff;
    const el = document.getElementById("nav-stack-hint");
    if (el) el.textContent = hint;
  }

  // 导航栈开关：这两个按钮管的是底层栈起没起，跟单次目标的下发/取消是两件事
  function bindNavStackButtons() {
    const on = document.getElementById("nav-stack-on");
    const off = document.getElementById("nav-stack-off");

    // 点完先按预期把状态翻过去，别等 1.5 秒后的轮询 —— 中间那一下没反馈，
    // 现场会以为没点上又点一次。轮询到真状态后会自动校正。
    const optimistic = (nextState) => {
      const navm = (state.status && state.status.navigation_manager) || {};
      navm.state = nextState;
      navm.ready = false;
      if (state.status) state.status.navigation_manager = navm;
      syncNavStackButtons();
    };

    if (on) on.addEventListener("click", async () => {
      optimistic("STARTING_LOCALIZATION");
      try {
        // 这一步要等雷达起转、IMU 静止闸门、Super-LIO 载图、Nav2 全部 activate，
        // 十几秒起步。不写清楚在等什么，现场只会以为卡死了反复点。
        await withBusy(on, "启动定位与 Nav2…", () =>
          guarded(() => api("/api/nav/ensure_ready", { method: "POST" }), "导航栈已就绪"));
      } catch (_e) { /* toast 里已经报过了 */ }
      syncNavStackButtons();
    });

    if (off) off.addEventListener("click", async () => {
      if (!confirm("关闭导航会停掉定位与 Nav2，确认？")) return;
      optimistic("STOPPED");
      try {
        await withBusy(off, "停止中…", () =>
          guarded(() => api("/api/nav/stop_all", { method: "POST" }), "已关闭导航"));
      } catch (_e) { /* 同上 */ }
      syncNavStackButtons();
    });

    syncNavStackButtons();
  }

  // 地图上的标记：机器人当前位姿（带朝向扇形）+ 目标点 + 重定位点。
  // 颜色统一走 MAP_COLORS，跟界面主题同源。
  function refreshMapMarks() {
    if (!navUi.view) return;
    const marks = [];
    const pose = state.status && state.status.pose;
    if (pose) marks.push({ kind: "robot", x: pose.x, y: pose.y, yaw_deg: pose.yaw_deg, color: MAP_COLORS.robot });
    if (navUi.target) marks.push({ x: navUi.target.x, y: navUi.target.y, yaw_deg: navUi.target.yaw_deg, color: MAP_COLORS.goal, radius: 9 });
    if (navUi.reloc) marks.push({ x: navUi.reloc.x, y: navUi.reloc.y, yaw_deg: navUi.reloc.yaw_deg, color: MAP_COLORS.reloc, radius: 9 });
    navUi.view.setMarks(marks);
  }

  async function mountNavMap(elId, { onPlace = null, onWaypointMove = null } = {}) {
    const el = document.getElementById(elId);
    if (!el) return;
    let active = null;
    try { active = await pickActiveMapGeometry(); } catch (_e) { /* 无地图 */ }
    if (!active || !active.geometry) {
      el.innerHTML = `<div class="center-text">没有可用地图<br><small>请先在「地图」页激活一张地图</small></div>`;
      return;
    }
    navUi.geo = active.geometry;
    navUi.view = await mountMapView(el, {
      imageUrl: mapImageUrl(active.mapId, active.mtime),
      geo: active.geometry,
      waypoints: navUi.waypoints,
      onPlace,
      onWaypointMove,
    });
    refreshMapMarks();
    // 机器人在图上得跟着动，跟状态轮询同频
    const markTimer = setInterval(refreshMapMarks, 1500);

    // Nav2 规划路径：拿到就画，路径过期（>5s 没更新）网关会返回空列表
    let planBusy = false;
    const pullPlan = async () => {
      if (planBusy || !navUi.view) return;
      planBusy = true;
      try {
        const res = await api("/api/nav/path");
        if (navUi.view) navUi.view.setPath(res.points || []);
      } catch (_e) { /* 网关没这个接口或网络抖动，忽略 */ }
      finally { planBusy = false; }
    };
    pullPlan();
    const planTimer = setInterval(pullPlan, 1000);

    // 实时激光：静态 pgm 只是建图那一刻的世界，现场多出来的人和箱子只能靠它看。
    // 500ms 一次 —— /scan 本身 10Hz，再快也只是徒增流量，而人走动的尺度上
    // 半秒的滞后看不出来。
    let scanBusy = false;
    const pullScan = async () => {
      if (scanBusy || !navUi.view) return;
      scanBusy = true;
      try {
        const res = await api("/api/nav/scan");
        if (navUi.view) navUi.view.setScan(res.points || []);
      } catch (_e) { /* 老版本网关没这个接口，静默降级成不画激光 */ }
      finally { scanBusy = false; }
    };
    pullScan();
    const scanTimer = setInterval(pullScan, 500);

    // 地图热切换：在别的页面「设为当前」之后回到导航页，或者两个浏览器
    // 各开一个页面，都不该还盯着旧图。3 秒一次，只在 mtime 变了才换。
    let mapMtime = active.mtime;
    let mapBusy = false;
    const pullMap = async () => {
      if (mapBusy || !navUi.view) return;
      mapBusy = true;
      try {
        const cur = await pickActiveMapGeometry();
        if (cur && navUi.view && cur.mtime && cur.mtime !== mapMtime) {
          mapMtime = cur.mtime;
          navUi.geo = cur.geometry;
          navUi.view.setImage(mapImageUrl(cur.mapId, cur.mtime), cur.geometry);
          toast(`地图已切换为 ${cur.label}`, "success");
        }
      } catch (_e) { /* 网络抖动，下一轮再说 */ }
      finally { mapBusy = false; }
    };
    const mapTimer = setInterval(pullMap, 3000);

    onPageLeave(() => {
      clearInterval(markTimer);
      clearInterval(planTimer);
      clearInterval(scanTimer);
      clearInterval(mapTimer);
      if (navUi.view && navUi.view.destroy) navUi.view.destroy();
      navUi.view = null;
    });
  }

  function coordFieldsHtml(prefix, label) {
    return `
      <div class="field-row">
        <div class="field"><label>${label} X</label><input id="${prefix}-x" class="inline-input" type="number" step="0.01" /></div>
        <div class="field"><label>Y</label><input id="${prefix}-y" class="inline-input" type="number" step="0.01" /></div>
        <div class="field"><label>朝向 °</label><input id="${prefix}-yaw" class="inline-input" type="number" step="1" value="0" /></div>
      </div>`;
  }

  function readCoord(prefix) {
    const x = parseFloat(document.getElementById(`${prefix}-x`).value);
    const y = parseFloat(document.getElementById(`${prefix}-y`).value);
    const yaw_deg = parseFloat(document.getElementById(`${prefix}-yaw`).value || "0");
    if (Number.isNaN(x) || Number.isNaN(y)) { toast("请先在地图上取点或填入坐标", "error"); return null; }
    return { x, y, yaw_deg };
  }

  function writeCoord(prefix, world) {
    document.getElementById(`${prefix}-x`).value = world.x.toFixed(2);
    document.getElementById(`${prefix}-y`).value = world.y.toFixed(2);
  }

  // ── 落点确认卡片：放完点不直接执行，先给一次确认/撤销的机会 ──
  function showPlaceCard(title, world, actions) {
    const card = document.getElementById("place-card");
    if (!card) return;
    const yawTxt = world.yaw_deg == null ? "保持当前朝向" : `${world.yaw_deg.toFixed(0)}°`;
    card.innerHTML = `
      <div class="place-info">
        <div class="place-title">${escapeHtml(title)}</div>
        <div class="place-coord">${world.x.toFixed(2)}, ${world.y.toFixed(2)} · ${yawTxt}</div>
      </div>
      <div class="btn-row tight">
        ${actions.map((a, i) => `<button class="btn ${a.cls || ""}" data-place-act="${i}">${a.label}</button>`).join("")}
      </div>`;
    card.hidden = false;
    card.querySelectorAll("[data-place-act]").forEach((btn) => btn.addEventListener("click", () => {
      const act = actions[parseInt(btn.getAttribute("data-place-act"), 10)];
      hidePlaceCard();
      if (act && act.run) act.run();
    }));
  }
  function hidePlaceCard() {
    const card = document.getElementById("place-card");
    if (card) { card.hidden = true; card.innerHTML = ""; }
  }

  // 地图面板（三个导航子页共用）
  // 当前地图名。设为当前之后 current_map.json 会被改写，所以这里跟着状态刷新。
  function currentMapLabel() {
    const cm = (state.status && state.status.current_map) || null;
    if (!cm) return "未选择地图";
    return cm.label || cm.base_name || "未知地图";
  }

  function navMapPaneHtml(hint) {
    return `
      <section class="pane">
        <div class="pane-head">
          <div class="eyebrow">地图</div>
          <span class="map-name" data-live="nav-map-name">${navMapNameHtml()}</span>
          <span class="hint">${escapeHtml(hint)}</span>
        </div>
        <div class="pane-body flush map-host">
          <div class="map-wrap" id="nav-map"><div class="center-text">加载地图…</div></div>
          <div class="place-card" id="place-card" hidden></div>
        </div>
      </section>`;
  }

  // ── 导航状态：一句话主状态 + 进度条 + 次要指标 ──
  function navProgressRatio(nv) {
    if (!nv.active || nv.distance_to_goal == null) { navUi.progressBase = null; return null; }
    const d = nv.distance_to_goal;
    if (navUi.progressBase == null || d > navUi.progressBase) navUi.progressBase = d;
    if (!navUi.progressBase) return null;
    return Math.max(0, Math.min(1, 1 - d / navUi.progressBase));
  }

  // 后端状态词 -> 人话。词表见 g1_control_server.py:37-39
  const NAV_STATUS_TEXT = {
    success: "成功",
    error: "失败",
    canceled: "已取消",
    timeout: "超时",
    rejected: "被拒绝",
  };
  function navStatusText(status) {
    if (!status) return "";
    return NAV_STATUS_TEXT[String(status).toLowerCase()] || String(status);
  }

  // 停下来之后，机器人到目标点的直线距离。位姿或目标缺一不可。
  function poseToTargetDistance(pose, target) {
    if (!pose || !target || target.x == null || pose.x == null) return null;
    return Math.hypot(pose.x - target.x, pose.y - target.y);
  }

  function navHeadlineHtml() {
    const s = state.status || {};
    const nv = s.navigate || {};
    const patrol = s.patrol || {};
    const navm = s.navigation_manager || {};
    const pose = s.pose;
    let title, sub, tone = "", ratio = null, finalOffset = null;

    if (patrol.running) {
      title = `巡航中 ${patrol.index}/${patrol.total}`;
      sub = patrol.message || (nv.target ? `前往 (${nv.target.x.toFixed(2)}, ${nv.target.y.toFixed(2)})` : "");
      tone = "run";
      ratio = patrol.total ? patrol.index / patrol.total : null;
    } else if (nv.active) {
      const t = nv.target;
      title = t ? `前往 (${t.x.toFixed(2)}, ${t.y.toFixed(2)})` : "导航中";
      sub = nv.phase || "执行中";
      tone = "run";
      ratio = navProgressRatio(nv);
    } else {
      navUi.progressBase = null;
      // result_success 由服务端判定（g1_control_server.py:346），直接用。
      // 这里原先拿 result_status !== "SUCCEEDED" 判失败 —— 那是 Nav2 GoalStatus
      // 的常量，而本系统的词表是小写的 success/error/canceled
      // （g1_control_server.py:37-39），于是每次成功导航都显示「未完成 · success」。
      // 老版本网关没有 result_success 字段，退回按词表比对。
      const done = !!nv.result_status;
      const ok = nv.result_success != null
        ? !!nv.result_success
        : String(nv.result_status).toLowerCase() === "success";
      title = done ? (ok ? "已到达" : `未完成 · ${navStatusText(nv.result_status)}`) : "空闲";
      // 「已到达」不等于站在目标点上：g1_control_server 是按档位容差判到位的
      // （日志里的 "物理距离达标 (Dist: 0.26m)"，精准档容差 0.62m），
      // 到点后主动取消 Nav2。之前界面既说已到达又说剩余 0.00 m，
      // 把这个真实存在的偏差抹掉了，现场对不上就会怀疑定位有问题。
      finalOffset = done && ok ? poseToTargetDistance(pose, nv.target) : null;
      sub = nv.result_message || (navm.ready ? "长按地图放置目标点" : "导航栈未就绪，先在下方开启");
      if (finalOffset != null) sub = `${sub}（实际停在距目标 ${finalOffset.toFixed(2)} m 处）`;
      tone = done ? (ok ? "ok" : "crit") : "";
    }

    const chips = [
      ["导航栈", navm.state || "—", navm.ready ? "is-ok" : "is-warn"],
      ["位姿", pose ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}` : "无 TF", pose ? "" : "is-dim"],
      ["朝向", pose ? `${pose.yaw_deg.toFixed(0)}°` : "—", pose ? "" : "is-dim"],
      finalOffset != null
        ? ["实际偏差", `${finalOffset.toFixed(2)} m`, ""]
        : ["剩余", nv.distance_to_goal != null ? `${nv.distance_to_goal.toFixed(2)} m` : "—", ""],
      ["用时", nv.elapsed_sec != null ? `${nv.elapsed_sec} s` : "—", ""],
    ];

    return `
      <div class="headline ${tone ? "tone-" + tone : ""}">
        <div class="headline-main">${escapeHtml(title)}</div>
        ${sub ? `<div class="headline-sub">${escapeHtml(sub)}</div>` : ""}
        ${ratio != null ? `<div class="progress"><i style="width:${(ratio * 100).toFixed(1)}%"></i></div>` : ""}
      </div>
      <div class="chip-row">
        ${chips.map(([k, v, t]) => `
          <div class="chip"><span class="chip-k">${k}</span><span class="chip-v ${t}">${escapeHtml(String(v))}</span></div>
        `).join("")}
      </div>`;
  }

  // 导航栈开关这类破坏性操作统一收进折叠区，默认不占视线
  function navStackAdvancedHtml() {
    return `
      <details class="adv">
        <summary>高级 · 导航栈</summary>
        <div class="adv-body">
          <div class="btn-row">
            <button class="btn success" id="nav-stack-on">开启导航栈</button>
            <button class="btn danger" id="nav-stack-off">关闭导航栈</button>
          </div>
          <p class="note" id="nav-stack-hint">关闭会停掉定位与 Nav2，机器人将无法导航；仅在排障或收工时使用。</p>
        </div>
      </details>`;
  }

  // ── 单点导航 ──
  async function renderNavPoint(opts) {
    shell(opts, "cols-main-side", `
      ${navMapPaneHtml("长按地图放置目标点，拖出朝向")}
      <div class="stack">
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">导航状态</div><span class="hint">每 1.5 秒刷新</span></div>
          <div class="pane-body" data-live="nav-panel">${navHeadlineHtml()}</div>
        </section>
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">目标</div><span class="hint" id="pt-src">尚未设定</span></div>
          <div class="pane-body scroll">
            ${coordFieldsHtml("pt", "目标")}
            <p class="note">常规操作直接长按地图放点；这里用于手输精确坐标或微调朝向。</p>
            ${navStackAdvancedHtml()}
          </div>
          <div class="pane-foot">
            <button class="btn primary" id="pt-go">前往该点</button>
            <button class="btn danger" id="nav-cancel">取消导航</button>
          </div>
        </section>
      </div>
    `);
    bindNavStackButtons();

    const sendGoal = (c) => {
      navUi.target = c;
      refreshMapMarks();
      writeCoord("pt", c);
      document.getElementById("pt-yaw").value = (c.yaw_deg == null ? 0 : c.yaw_deg).toFixed(0);
      return withBusy(document.getElementById("pt-go"), "下发中…", () =>
        guarded(() => api("/api/nav/navigate", {
          method: "POST",
          body: {
            waypoint_name: "manual", x: c.x, y: c.y,
            yaw_deg: c.yaw_deg == null ? 0 : c.yaw_deg,
            align_final_yaw: c.yaw_deg != null,
          },
        }), "已发送导航目标")).catch(() => {});
    };

    document.getElementById("pt-go").addEventListener("click", () => {
      const c = readCoord("pt");
      if (c) sendGoal(c);
    });
    bindBusy("nav-cancel", "停止中…",
      () => api("/api/nav/cancel", { method: "POST" }), "已取消导航");

    await mountNavMap("nav-map", {
      onPlace: (world) => {
        navUi.target = { ...world, yaw_deg: world.yaw_deg == null ? 0 : world.yaw_deg };
        refreshMapMarks();
        writeCoord("pt", world);
        if (world.yaw_deg != null) document.getElementById("pt-yaw").value = world.yaw_deg.toFixed(0);
        document.getElementById("pt-src").textContent = "已从地图取点";
        showPlaceCard("目标点", world, [
          { label: "前往这里", cls: "primary", run: () => sendGoal({ ...world }) },
          { label: "取消", cls: "ghost", run: () => { navUi.target = null; refreshMapMarks(); } },
        ]);
      },
    });
  }

  // ── 重定位（独立子页：它改的是"机器人以为自己在哪"，风险和导航完全不同）──
  async function renderNavReloc(opts) {
    shell(opts, "cols-main-side", `
      ${navMapPaneHtml("长按地图放置机器人实际位置，拖出实际朝向")}
      <div class="stack">
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">当前定位</div><span class="hint">每 1.5 秒刷新</span></div>
          <div class="pane-body" data-live="nav-panel">${navHeadlineHtml()}</div>
        </section>
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">重定位</div></div>
          <div class="pane-body scroll">
            <p class="note" style="margin-top:0">
              当机器人在地图上的位置和它实际所在的位置对不上时，用这里纠正。<br>
              <b>填错会让机器人朝完全错误的方向走</b>，务必确认朝向也对。
            </p>
            ${coordFieldsHtml("rl", "实际")}
          </div>
          <div class="pane-foot"><button class="btn danger solid" id="rl-go">确认重定位</button></div>
        </section>
      </div>
    `);

    const doReloc = (c) => guarded(
      () => api("/api/nav/relocalize", { method: "POST", body: { x: c.x, y: c.y, yaw_deg: c.yaw_deg || 0 } }),
      "重定位完成",
    ).then(() => { navUi.reloc = null; refreshMapMarks(); }).catch(() => {});

    document.getElementById("rl-go").addEventListener("click", () => {
      const c = readCoord("rl");
      if (!c) return;
      if (!confirm(`确认把机器人位置改为 (${c.x.toFixed(2)}, ${c.y.toFixed(2)}) 朝向 ${c.yaw_deg.toFixed(0)}°？`)) return;
      doReloc(c);
    });

    await mountNavMap("nav-map", {
      onPlace: (world) => {
        const c = { ...world, yaw_deg: world.yaw_deg == null ? 0 : world.yaw_deg };
        navUi.reloc = c;
        refreshMapMarks();
        writeCoord("rl", c);
        document.getElementById("rl-yaw").value = c.yaw_deg.toFixed(0);
        showPlaceCard("机器人实际位置", world, [
          { label: "确认重定位", cls: "danger solid", run: () => doReloc(c) },
          { label: "取消", cls: "ghost", run: () => { navUi.reloc = null; refreshMapMarks(); } },
        ]);
      },
    });
  }

  // ── 多点巡航 ──
  async function renderNavPatrol(opts) {
    shell(opts, "cols-main-side", `
      ${navMapPaneHtml("长按地图添加巡航点 · 圆牌可直接拖动")}
      <div class="stack">
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">巡航状态</div><span class="hint" id="route-name-tag">未选择路线</span></div>
          <div class="pane-body" data-live="nav-panel">${navHeadlineHtml()}</div>
        </section>
        <section class="pane">
          <div class="pane-head">
            <div class="eyebrow">巡航点 (<span id="wp-count">0</span>)</div>
            <button class="btn sm ghost" id="wp-clear">清空</button>
          </div>
          <div class="pane-body scroll" id="wp-list"><div class="center-text">尚未载入路线</div></div>
        </section>
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">操作</div></div>
          <div class="pane-body">
            <div class="btn-row">
              <button class="btn" id="route-switch">切换路线</button>
              <button class="btn" id="route-save">保存路线</button>
            </div>
            ${navStackAdvancedHtml()}
          </div>
          <div class="pane-foot">
            <button class="btn primary" id="patrol-start">开始巡航</button>
            <button class="btn danger" id="patrol-stop">停止巡航</button>
          </div>
        </section>
      </div>
    `);
    bindNavStackButtons();

    const syncMap = () => { if (navUi.view) navUi.view.setWaypoints(navUi.waypoints); };

    // 动作下拉用真实名字，别让人去记「动作ID 25」
    async function ensurePresets() {
      if (navUi.presets) return navUi.presets;
      try { navUi.presets = await api("/api/actions/presets"); }
      catch (_e) { navUi.presets = []; }
      return navUi.presets;
    }

    function wpEditorHtml(wp, idx) {
      const presets = navUi.presets || [];
      const cur = wp.action_id == null ? 25 : wp.action_id;
      const known = presets.some((p) => p.action_id === cur);
      return `
        <div class="wp-editor" data-wp-editor="${idx}">
          <div class="field-row">
            <div class="field"><label>朝向 °</label>
              <input class="inline-input" type="number" step="1" data-e-yaw value="${(wp.yaw_deg || 0).toFixed(0)}" /></div>
            <div class="field" style="grid-column: span 2"><label>到达后动作</label>
              <select class="inline-input" data-e-action>
                <option value="0" ${cur === 0 ? "selected" : ""}>不做动作</option>
                ${presets.map((p) => `<option value="${p.action_id}" ${p.action_id === cur ? "selected" : ""}>${escapeHtml(p.name)} (${p.action_id})</option>`).join("")}
                ${known || cur === 0 ? "" : `<option value="${cur}" selected>动作 ${cur}</option>`}
              </select></div>
          </div>
          <div class="field"><label>到达后语音</label>
            <input class="inline-input" type="text" data-e-say value="${escapeHtml(wp.say_text || "")}" placeholder="留空则不说话" /></div>
          <div class="btn-row tight">
            <button class="btn primary sm" data-e-save="${idx}">保存</button>
            <button class="btn ghost sm" data-e-cancel>取消</button>
          </div>
        </div>`;
    }

    function renderWpList(editIdx) {
      const list = document.getElementById("wp-list");
      document.getElementById("wp-count").textContent = navUi.waypoints.length;
      if (!navUi.waypoints.length) {
        list.innerHTML = `<div class="center-text">长按地图添加巡航点</div>`;
        return;
      }
      list.innerHTML = `<div class="list">${navUi.waypoints.map((wp, i) => {
        const preset = (navUi.presets || []).find((p) => p.action_id === (wp.action_id == null ? 25 : wp.action_id));
        const actionTxt = (wp.action_id === 0) ? "无动作" : (preset ? preset.name : `动作 ${wp.action_id == null ? 25 : wp.action_id}`);
        return `
        <div class="list-item wp-item">
          <span class="wp-no">${i + 1}</span>
          <div class="info">
            <div class="name">${wp.x.toFixed(2)}, ${wp.y.toFixed(2)} @ ${(wp.yaw_deg || 0).toFixed(0)}°</div>
            <div class="meta">${escapeHtml(actionTxt)}${wp.say_text ? " · " + escapeHtml(wp.say_text) : ""}</div>
          </div>
          <div class="btn-row tight">
            <button class="btn sm ghost" data-wp-up="${i}" ${i === 0 ? "disabled" : ""}>↑</button>
            <button class="btn sm ghost" data-wp-down="${i}" ${i === navUi.waypoints.length - 1 ? "disabled" : ""}>↓</button>
            <button class="btn sm" data-wp-edit="${i}">编辑</button>
            <button class="btn danger sm" data-wp-del="${i}">删</button>
          </div>
        </div>
        ${editIdx === i ? wpEditorHtml(wp, i) : ""}`;
      }).join("")}</div>`;

      const move = (from, to) => {
        if (to < 0 || to >= navUi.waypoints.length) return;
        const [item] = navUi.waypoints.splice(from, 1);
        navUi.waypoints.splice(to, 0, item);
        renderWpList(); syncMap();
      };
      list.querySelectorAll("[data-wp-up]").forEach((el) => el.addEventListener("click", () =>
        move(parseInt(el.getAttribute("data-wp-up"), 10), parseInt(el.getAttribute("data-wp-up"), 10) - 1)));
      list.querySelectorAll("[data-wp-down]").forEach((el) => el.addEventListener("click", () =>
        move(parseInt(el.getAttribute("data-wp-down"), 10), parseInt(el.getAttribute("data-wp-down"), 10) + 1)));
      list.querySelectorAll("[data-wp-del]").forEach((el) => el.addEventListener("click", () => {
        navUi.waypoints.splice(parseInt(el.getAttribute("data-wp-del"), 10), 1);
        renderWpList(); syncMap();
      }));
      list.querySelectorAll("[data-wp-edit]").forEach((el) => el.addEventListener("click", async () => {
        await ensurePresets();
        renderWpList(parseInt(el.getAttribute("data-wp-edit"), 10));
      }));

      const editor = list.querySelector("[data-wp-editor]");
      if (editor) {
        editor.querySelector("[data-e-cancel]").addEventListener("click", () => renderWpList());
        editor.querySelector("[data-e-save]").addEventListener("click", () => {
          const wp = navUi.waypoints[editIdx];
          wp.yaw_deg = parseFloat(editor.querySelector("[data-e-yaw]").value) || 0;
          wp.action_id = parseInt(editor.querySelector("[data-e-action]").value, 10) || 0;
          wp.say_text = editor.querySelector("[data-e-say]").value.trim();
          renderWpList(); syncMap();
        });
      }
    }

    async function loadRoute(name) {
      try {
        const route = await api(`/api/routes/${encodeURIComponent(name)}`);
        navUi.routeName = name;
        navUi.waypoints = (route.waypoints || []).map((w) => ({ ...w }));
      } catch (_e) {
        navUi.routeName = name;
        navUi.waypoints = [];
      }
      document.getElementById("route-name-tag").textContent = `路线 ${navUi.routeName}`;
      renderWpList(); syncMap();
    }

    document.getElementById("route-switch").addEventListener("click", async () => {
      let routes = [];
      try { routes = await api("/api/routes"); } catch (err) { toast(String(err.message || err), "error"); return; }
      const modal = showModal("切换路线", `
        <div class="list">
          ${routes.map((r) => `
            <div class="list-item">
              <div class="info"><div class="name">${escapeHtml(r.route_name)}</div><div class="meta">${r.waypoint_count} 个巡航点</div></div>
              <div class="btn-row tight">
                <button class="btn primary sm" data-route-pick="${escapeHtml(r.name)}">载入</button>
                <button class="btn danger sm" data-route-del="${escapeHtml(r.name)}">删除</button>
              </div>
            </div>`).join("") || '<div class="center-text">暂无已保存路线</div>'}
        </div>
        <div class="btn-row" style="margin-top:12px"><button class="btn" id="route-new">新建路线</button></div>
      `);
      modal.el.querySelectorAll("[data-route-pick]").forEach((el) => el.addEventListener("click", async () => {
        modal.close();
        await loadRoute(el.getAttribute("data-route-pick"));
        toast(`已载入路线 ${navUi.routeName}`, "success");
      }));
      modal.el.querySelectorAll("[data-route-del]").forEach((el) => el.addEventListener("click", async () => {
        if (!confirm("确认删除该路线？")) return;
        await guarded(() => api(`/api/routes/${encodeURIComponent(el.getAttribute("data-route-del"))}`, { method: "DELETE" }), "已删除");
        modal.close();
      }));
      const btnNew = modal.el.querySelector("#route-new");
      if (btnNew) btnNew.addEventListener("click", () => {
        const name = prompt("新路线名称（英文/数字/下划线）：");
        if (!name) return;
        modal.close();
        navUi.routeName = name;
        navUi.waypoints = [];
        document.getElementById("route-name-tag").textContent = `路线 ${name}（未保存）`;
        renderWpList(); syncMap();
      });
    });

    document.getElementById("route-save").addEventListener("click", () => {
      if (!navUi.routeName) { toast("请先切换或新建一条路线", "error"); return; }
      if (!navUi.waypoints.length) { toast("巡航点为空", "error"); return; }
      guarded(() => api(`/api/routes/${encodeURIComponent(navUi.routeName)}`, {
        method: "PUT",
        body: { route_name: navUi.routeName, waypoints: navUi.waypoints },
      }), "路线已保存");
    });

    document.getElementById("wp-clear").addEventListener("click", () => {
      if (!navUi.waypoints.length) return;
      if (!confirm("清空当前巡航点？（不影响已保存的路线文件）")) return;
      navUi.waypoints = [];
      renderWpList(); syncMap();
    });

    document.getElementById("patrol-start").addEventListener("click", () => {
      if (!navUi.routeName) { toast("请先切换到一条已保存的路线", "error"); return; }
      guarded(() => api("/api/nav/patrol/start", { method: "POST", body: { route_name: navUi.routeName } }), "已开始巡航");
    });
    document.getElementById("patrol-stop").addEventListener("click", () =>
      guarded(() => api("/api/nav/patrol/stop", { method: "POST" }), "已停止巡航"));

    await ensurePresets();
    await mountNavMap("nav-map", {
      onPlace: (world) => {
        navUi.waypoints.push({
          x: world.x, y: world.y,
          yaw_deg: world.yaw_deg == null ? 0 : world.yaw_deg,
          action_id: 25, say_text: "",
        });
        const idx = navUi.waypoints.length;
        renderWpList(); syncMap();
        showPlaceCard(`已加为巡航点 #${idx}`, world, [
          { label: "编辑动作", run: async () => { await ensurePresets(); renderWpList(idx - 1); } },
          { label: "撤销", cls: "danger", run: () => { navUi.waypoints.pop(); renderWpList(); syncMap(); } },
        ]);
      },
      onWaypointMove: () => { renderWpList(); },
    });

    // 进页面时自动载入当前巡航状态里的路线，其次是上次选的，最后才自动挑一条。
    // 自动挑的时候优先 default —— 之前直接取 routes[0]（文件名排序第一个），
    // 一旦 default 不存在就会莫名其妙载入另一条路线，现场看到一堆自己没加过的点。
    const running = (state.status && state.status.patrol) || {};
    if (running.route_name) await loadRoute(running.route_name);
    else if (navUi.routeName) await loadRoute(navUi.routeName);
    else {
      try {
        const routes = await api("/api/routes");
        const pick = routes.find((r) => r.name === "default") || routes[0];
        if (pick) await loadRoute(pick.name);
        else renderWpList();
      } catch (_e) { renderWpList(); }
    }
  }

  // ── 地图编辑 ──
  //
  // 为什么要有这个页面：pcd_to_2d_map 是从点云一次投出来的，玻璃门和镂空
  // 栏杆雷达打不到，地图上是通的，机器人会径直撞过去；反过来，走动的人和
  // 临时堆的箱子会被扫成永久障碍把通道堵死。这些只能人工修。
  const mapEdit = {
    view: null, state: null, mapId: null,
    tool: "paint", brush: "occupied", radius: 0.15,
  };

  const EDIT_TOOLS = [
    ["paint", "画笔", "涂改栅格"],
    ["zone", "禁行区", "框出机器人不许进的区域"],
  ];
  const EDIT_BRUSHES = [
    ["occupied", "障碍", "补上雷达打不到的玻璃门、栏杆"],
    ["free", "可通行", "擦掉被扫成障碍的人和箱子"],
    ["unknown", "未知", "抹成未知，交给实时避障判断"],
  ];

  async function renderMapEdit(sub) {
    startPolling();
    const mapId = sub && sub[0] ? decodeURIComponent(sub[0]) : "active";
    mapEdit.mapId = mapId;

    shell({ back: "map/list", title: "地图编辑", sub: "涂改 · 禁行区 · 裁剪" },
      "cols-main-side", `
      <section class="pane">
        <div class="pane-head">
          <div class="eyebrow">地图</div>
          <span class="map-name" id="edit-name"></span>
          <span class="hint" id="edit-saved">改动即时保存</span>
        </div>
        <div class="pane-body flush map-host">
          <div class="map-wrap" id="edit-map"><div class="center-text">加载地图…</div></div>
        </div>
      </section>
      <div class="stack">
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">工具</div></div>
          <div class="pane-body">
            <div class="sb-tabs" id="edit-tools">
              ${EDIT_TOOLS.map(([k, name], i) => `
                <button class="${i === 0 ? "active" : ""}" data-tool="${k}">${name}</button>`).join("")}
            </div>
            <div id="edit-brush-box">
              <p class="note" style="margin:10px 0 4px">涂成什么</p>
              <div class="sb-tabs" id="edit-brushes">
                ${EDIT_BRUSHES.map(([k, name], i) => `
                  <button class="${i === 0 ? "active" : ""}" data-brush="${k}">${name}</button>`).join("")}
              </div>
              <div class="field" style="margin-top:10px">
                <label>画笔半径 <span id="edit-radius-view">0.15</span> m</label>
                <input id="edit-radius" type="range" min="0.05" max="1" step="0.05" value="0.15" />
              </div>
            </div>
            <p class="note" id="edit-tip"></p>
          </div>
        </section>
        <section class="pane fixed">
          <div class="pane-head">
            <div class="eyebrow">禁行区 (<span id="edit-zone-count">0</span>)</div>
            <button class="btn sm ghost" id="edit-zone-clear">清空</button>
          </div>
          <div class="pane-body scroll" id="edit-zone-list"></div>
        </section>
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">生效</div></div>
          <div class="pane-body">
            <p class="note" style="margin-top:0">
              画的每一笔、每个禁行区都<b>已经存进地图文件</b>，不需要另外保存。
              但 Nav2 是启动时把地图读进内存的，<b>要重启导航栈才会用上新地图</b>。
            </p>
            <div class="btn-row">
              <button class="btn primary" id="edit-apply">重启导航栈使改动生效</button>
            </div>
          </div>
        </section>
        <section class="pane fixed">
          <div class="pane-head"><div class="eyebrow">整图操作</div></div>
          <div class="pane-body">
            <div class="btn-row">
              <button class="btn" id="edit-autocrop">自动裁边</button>
              <button class="btn" id="edit-rot90">旋转 90°</button>
            </div>
            <p class="note">自动裁边去掉四周成片的未知区域，只留有内容的部分。
              旋转会让重定位用的 3D 点云对不上，只适合还没投入使用的地图。</p>
            <div class="btn-row" style="margin-top:8px">
              <button class="btn danger" id="edit-revert">还原为最初的地图</button>
            </div>
          </div>
        </section>
      </div>
    `);

    const nameEl = document.getElementById("edit-name");
    const tipEl = document.getElementById("edit-tip");
    const savedEl = document.getElementById("edit-saved");

    // 改动是即时落盘的，但界面上不说的话，用户会一直找"保存"按钮。
    // 每次成功写入都在这里标一下时间和改了多少。
    function markSaved(what) {
      const t = new Date().toLocaleTimeString("zh-CN", { hour12: false });
      savedEl.textContent = `${what} · 已保存 ${t}`;
      savedEl.classList.add("is-ok");
      setTimeout(() => savedEl.classList.remove("is-ok"), 1500);
    }

    async function refreshState() {
      mapEdit.state = await api(`/api/maps/${encodeURIComponent(mapId)}/edit`);
      nameEl.innerHTML = `<span class="map-name-tag">${escapeHtml(
        mapEdit.state.label || mapEdit.state.id)}</span>`;
      renderZones();
      document.getElementById("edit-revert").disabled = !mapEdit.state.can_revert;
    }

    function renderZones() {
      const zones = (mapEdit.state && mapEdit.state.zones) || [];
      document.getElementById("edit-zone-count").textContent = zones.length;
      document.getElementById("edit-zone-list").innerHTML = zones.length
        ? zones.map((z, i) => `
            <div class="wp-row">
              <div class="wp-main">
                <div class="title">${z.w.toFixed(2)} × ${z.h.toFixed(2)} m</div>
                <div class="meta">左下 ${z.x.toFixed(2)}, ${z.y.toFixed(2)}</div>
              </div>
              <button class="btn sm danger" data-zone-del="${i}">删</button>
            </div>`).join("")
        : `<p class="note">还没有禁行区。选「禁行区」工具后在图上拉框。</p>`;
      if (mapEdit.view) mapEdit.view.setZones(zones);
    }

    async function saveZones(zones) {
      // 不弹 toast：画一个框弹一次太吵，用标题栏的"已保存"提示代替
      const res = await api(`/api/maps/${encodeURIComponent(mapId)}/edit/zones`,
                            { method: "POST", body: { zones } });
      mapEdit.state.zones = res.zones || zones;
      renderZones();
      reloadImage();
      markSaved(`禁行区 ${(res.zones || zones).length} 个`);
    }

    // 改完地图要换掉底图：服务端已经重写了 pgm，但 URL 没变，
    // 浏览器会拿缓存。用时间戳强制刷新。
    function reloadImage() {
      if (!mapEdit.view) return;
      mapEdit.view.setImage(
        `${state.baseUrl}/api/maps/${encodeURIComponent(mapId)}/image?t=${Date.now()}`);
    }

    function syncTool() {
      const painting = mapEdit.tool === "paint";
      document.getElementById("edit-brush-box").hidden = !painting;
      tipEl.textContent = painting
        ? "在图上按住拖动即可涂改；双指缩放平移不受影响。"
        : "在图上拉一个框＝一个禁行区。禁行区单独存，随时可以删掉恢复。";
      if (mapEdit.view) {
        mapEdit.view.setEditMode(painting ? "paint" : "zone", { radius: mapEdit.radius });
      }
    }

    document.getElementById("edit-tools").addEventListener("click", (ev) => {
      const btn = ev.target.closest("[data-tool]");
      if (!btn) return;
      mapEdit.tool = btn.getAttribute("data-tool");
      [...ev.currentTarget.children].forEach((c) => c.classList.toggle("active", c === btn));
      syncTool();
    });
    document.getElementById("edit-brushes").addEventListener("click", (ev) => {
      const btn = ev.target.closest("[data-brush]");
      if (!btn) return;
      mapEdit.brush = btn.getAttribute("data-brush");
      [...ev.currentTarget.children].forEach((c) => c.classList.toggle("active", c === btn));
    });
    const radiusEl = document.getElementById("edit-radius");
    radiusEl.addEventListener("input", () => {
      mapEdit.radius = Number(radiusEl.value);
      document.getElementById("edit-radius-view").textContent = mapEdit.radius.toFixed(2);
      if (mapEdit.view) mapEdit.view.setEditMode(mapEdit.tool === "paint" ? "paint" : "zone",
                                                 { radius: mapEdit.radius });
    });

    document.getElementById("edit-zone-list").addEventListener("click", async (ev) => {
      const btn = ev.target.closest("[data-zone-del]");
      if (!btn) return;
      const idx = Number(btn.getAttribute("data-zone-del"));
      const zones = (mapEdit.state.zones || []).filter((_z, i) => i !== idx);
      await saveZones(zones).catch(() => {});
    });
    document.getElementById("edit-zone-clear").addEventListener("click", async () => {
      if (!(mapEdit.state.zones || []).length) return;
      if (!confirm("清空所有禁行区？")) return;
      await saveZones([]).catch(() => {});
    });

    bindBusy("edit-apply", "重拉定位与 Nav2…",
             () => api("/api/nav/restart_all", { method: "POST" }),
             "导航栈已重启，新地图已生效");

    bindBusy("edit-autocrop", "裁剪中…", async () => {
      const res = await api(`/api/maps/${encodeURIComponent(mapId)}/edit/transform`,
                            { method: "POST", body: { action: "autocrop" } });
      await remount();
      return res;
    }, "已自动裁边");

    document.getElementById("edit-rot90").addEventListener("click", async () => {
      if (!confirm("旋转会让重定位用的 3D 点云对不上，之后需要重新建图。\n\n确认旋转？")) return;
      await withBusy(document.getElementById("edit-rot90"), "旋转中…", async () => {
        const res = await guarded(
          () => api(`/api/maps/${encodeURIComponent(mapId)}/edit/transform`,
                    { method: "POST", body: { action: "rotate90" } }), "已旋转");
        if (res && res.message) toast(res.message, "warn");
        await remount();
      }).catch(() => {});
    });

    document.getElementById("edit-revert").addEventListener("click", async () => {
      if (!confirm("还原为最初的地图？\n\n所有涂改和禁行区都会丢失。")) return;
      await withBusy(document.getElementById("edit-revert"), "还原中…", async () => {
        await guarded(() => api(`/api/maps/${encodeURIComponent(mapId)}/edit/revert`,
                                { method: "POST" }), "已还原");
        await remount();
      }).catch(() => {});
    });

    async function remount() {
      await refreshState();
      reloadImage();
    }

    // 挂地图
    let info = null;
    try { info = await api(`/api/maps/${encodeURIComponent(mapId)}`); } catch (_e) { /* 下面兜底 */ }
    const host = document.getElementById("edit-map");
    if (!info || !info.geometry) {
      host.innerHTML = `<div class="center-text">这张地图没有栅格数据，无法编辑</div>`;
      return;
    }
    mapEdit.view = await mountMapView(host, {
      imageUrl: mapImageUrl(mapId, info.mtime),
      geo: info.geometry,
      onEditStroke: async (stroke) => {
        try {
          const res = await api(`/api/maps/${encodeURIComponent(mapId)}/edit/paint`, {
            method: "POST",
            body: { strokes: [stroke], brush: mapEdit.brush, radius_m: mapEdit.radius },
          });
          reloadImage();
          markSaved(`涂改 ${res.changed_pixels || 0} 格`);
        } catch (err) {
          toast(String(err.message || err), "error");
        }
      },
      onEditRect: async (rect) => {
        const zones = [...((mapEdit.state && mapEdit.state.zones) || []), rect];
        await saveZones(zones).catch(() => {});
      },
    });
    await refreshState();
    syncTool();
    onPageLeave(() => {
      if (mapEdit.view && mapEdit.view.destroy) mapEdit.view.destroy();
      mapEdit.view = null;
    });
  }

  // ── 控制 ──
  function renderControl(sub) {
    startPolling();
    const active = sub && sub[0] ? sub[0] : "move";
    const opts = {
      back: "home", title: "控制", sub: "移动 · 动作",
      tabs: { base: "control", items: [["move", "移动控制"], ["actions", "动作控制"]] }, active,
    };
    if (active === "actions") return renderControlActions(opts);
    return renderControlMove(opts);
  }

  // 移动控制：中间相机画面，左姿态右转向，下面两个摇杆
  function renderControlMove(opts) {
    shell(opts, "cols-map-work", `
      <aside class="pane work-a">
        <div class="pane-head"><div class="eyebrow">姿态 / FSM</div></div>
        <div class="pane-body" style="display:flex;flex-direction:column;gap:9px">
          <button class="btn block" data-squat="stand_up">站起</button>
          <button class="btn block" data-squat="squat">蹲下</button>
          <button class="btn block" data-squat="damp">阻尼模式</button>
          <button class="btn block" data-squat="start">切换站立</button>
          <div class="field" style="margin-top:4px"><label>FSM ID</label>
            <div class="btn-row"><input id="fsm-input" class="inline-input" type="number" style="flex:1" /><button class="btn primary" id="fsm-set">设置</button></div>
          </div>
        </div>
        <div class="pane-foot"><button class="btn danger solid block" id="ctrl-stop">急停</button></div>
      </aside>
      ${stickPaneHtml("move")}
      <section class="pane robot-pane work-c camera-pane">
        <div class="pane-head" style="background:var(--surface)">
          <div class="eyebrow">相机画面</div><span class="hint" id="cam-hint">检测中…</span>
        </div>
        <div class="pane-body flush camera-stage"><div class="camera-holder" id="camera-holder">
          <div class="center-text stage-text">等待相机画面…</div>
        </div></div>
      </section>
      <aside class="pane work-d">
        <div class="pane-head"><div class="eyebrow">定量转向</div></div>
        <div class="pane-body" style="display:flex;flex-direction:column;gap:9px">
          <!-- 后端 _rotate_robot: angle_deg >= 0 → wz 为正 → 逆时针 → 左转。
               之前这里把正负标反了，点"左转"实际会右转。 -->
          <button class="btn block" data-rotate="90">↺ 左转 90°</button>
          <button class="btn block" data-rotate="30">↺ 左转 30°</button>
          <button class="btn block" data-rotate="-30">↻ 右转 30°</button>
          <button class="btn block" data-rotate="-90">↻ 右转 90°</button>
        </div>
        <div class="pane-foot" style="display:block">
          <div class="field" style="margin:0"><label>定量移动 (m): <span id="dist-val">0.5</span></label>
            <div class="slider-row"><input id="dist-slider" type="range" min="0.1" max="3" step="0.1" value="0.5" /></div>
          </div>
          <div class="btn-row tight" style="margin-top:8px">
            <button class="btn sm" data-move="forward">前</button>
            <button class="btn sm" data-move="backward">后</button>
            <button class="btn sm" data-move="left">左</button>
            <button class="btn sm" data-move="right">右</button>
          </div>
        </div>
      </aside>
      ${stickPaneHtml("turn")}
    `);

    document.getElementById("dist-slider").addEventListener("input", (e) => {
      document.getElementById("dist-val").textContent = e.target.value;
    });
    app.querySelectorAll("[data-move]").forEach((el) => el.addEventListener("click", () => {
      const distance_m = parseFloat(document.getElementById("dist-slider").value);
      guarded(() => api("/api/control/move", { method: "POST", body: { direction: el.getAttribute("data-move"), distance_m, speed_scale: 1.0 } }));
    }));
    document.getElementById("ctrl-stop").addEventListener("click", () =>
      guarded(() => api("/api/control/stop", { method: "POST" }), "已停止"));
    app.querySelectorAll("[data-rotate]").forEach((el) => el.addEventListener("click", () =>
      guarded(() => api("/api/control/rotate", { method: "POST", body: { angle_deg: parseFloat(el.getAttribute("data-rotate")), speed_scale: 1.0 } }))));
    app.querySelectorAll("[data-squat]").forEach((el) => el.addEventListener("click", () =>
      guarded(() => api("/api/control/squat", { method: "POST", body: { action: el.getAttribute("data-squat") } }))));
    document.getElementById("fsm-set").addEventListener("click", () => {
      const fsm_id = parseInt(document.getElementById("fsm-input").value, 10);
      if (Number.isNaN(fsm_id)) { toast("请输入合法 FSM ID", "error"); return; }
      guarded(() => api("/api/control/fsm", { method: "POST", body: { fsm_id } }), "已设置");
    });

    mountTeleopSticks();
    startCameraLoop();
  }

  // 相机：网关把 CompressedImage 的 JPEG 原样转发，这里换 img.src 拉帧。
  // 拿不到画面就如实显示话题名和原因，不放假画面。
  function startCameraLoop() {
    const holder = document.getElementById("camera-holder");
    const hint = document.getElementById("cam-hint");
    if (!holder) return;
    let img = null;
    let stopped = false;
    let failing = 0;

    function showPlaceholder(info) {
      const topic = (info && info.topic) || "未配置";
      const reason = !info ? "网关未响应"
        : !info.subscribed ? "网关未订阅该话题"
        : info.frame_count ? `话题停更 ${info.age_sec ?? "?"}s`
        : "话题上没有数据";
      holder.innerHTML = `
        <div class="camera-empty">
          <div class="camera-empty-title">无相机画面</div>
          <div class="camera-empty-body">${escapeHtml(reason)}<br><code>${escapeHtml(topic)}</code></div>
          <div class="camera-empty-note">当前导航栈只用 MID360 激光，没有节点在发图像。<br>接上 RealSense 驱动后本页自动出画面，话题可用 --camera-topic 改。</div>
        </div>`;
      img = null;
    }

    async function poll() {
      if (stopped) return;
      let info = null;
      try { info = await api("/api/camera/info"); } catch (_e) { /* 下面按 null 处理 */ }
      if (hint) {
        hint.textContent = info
          ? (info.available ? `${info.topic} · ${Math.round((info.frame_bytes || 0) / 1024)} KB/帧` : info.topic)
          : "网关无响应";
      }
      if (!info || !info.available) { showPlaceholder(info); return; }
      if (!img) {
        holder.innerHTML = `<img class="camera-img" alt="camera" />`;
        img = holder.querySelector("img");
        img.onerror = () => { failing += 1; if (failing > 5) showPlaceholder(info); };
        img.onload = () => { failing = 0; };
      }
      img.src = `${state.baseUrl}/api/camera/frame?t=${Date.now()}`;
    }

    poll();
    // 有画面时 10fps 拉帧；info 每 2 秒查一次就够
    const frameTimer = setInterval(() => {
      if (img && !stopped) img.src = `${state.baseUrl}/api/camera/frame?t=${Date.now()}`;
    }, 100);
    const infoTimer = setInterval(poll, 2000);
    onPageLeave(() => { stopped = true; clearInterval(frameTimer); clearInterval(infoTimer); });
  }

  // 三个动作库并排，各自在自己面板里滚，页面骨架不动
  async function renderControlActions(opts) {
    shell(opts, "cols-3", `
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">预设动作</div><span class="hint" id="cnt-preset"></span></div>
        <div class="pane-body scroll"><div class="chip-grid" id="grid-preset"><div class="center-text">加载中…</div></div></div>
      </section>
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">命名动作 / 轨迹</div><span class="hint" id="cnt-named"></span></div>
        <div class="pane-body scroll"><div class="chip-grid" id="grid-named"></div></div>
      </section>
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">编排脚本</div><span class="hint" id="cnt-script"></span></div>
        <div class="pane-body scroll"><div class="chip-grid" id="grid-script"></div></div>
        <div class="pane-foot">
          <input id="custom-action-name" class="inline-input" placeholder="SDK 示教动作名称" style="flex:1" />
          <button class="btn primary" id="custom-action-go">执行</button>
        </div>
      </section>
    `);
    const [presets, named, scripts] = await Promise.all([
      api("/api/actions/presets").catch(() => []),
      api("/api/actions/named").catch(() => []),
      api("/api/actions/scripts").catch(() => []),
    ]);
    const fill = (id, cntId, html, n) => {
      document.getElementById(id).innerHTML = html || `<div class="center-text">无</div>`;
      document.getElementById(cntId).textContent = `${n} 项`;
    };
    fill("grid-preset", "cnt-preset",
      presets.map((p) => `<button class="btn" data-preset="${p.action_id}">${escapeHtml(p.name)}</button>`).join(""), presets.length);
    fill("grid-named", "cnt-named",
      named.map((n) => `<button class="btn" data-named="${escapeHtml(n)}">${escapeHtml(n)}</button>`).join(""), named.length);
    fill("grid-script", "cnt-script",
      scripts.map((s) => `<button class="btn" data-script="${escapeHtml(s)}">${escapeHtml(s.replace(/\.json$/, ""))}</button>`).join(""), scripts.length);

    app.querySelectorAll("[data-preset]").forEach((el) => el.addEventListener("click", () =>
      guarded(() => api("/api/control/arm_action", { method: "POST", body: { action_id: parseInt(el.getAttribute("data-preset"), 10) } }), "动作已执行")));
    app.querySelectorAll("[data-named]").forEach((el) => el.addEventListener("click", () =>
      guarded(() => api("/api/control/named_action", { method: "POST", body: { action_name: el.getAttribute("data-named") } }), "动作已执行")));
    app.querySelectorAll("[data-script]").forEach((el) => el.addEventListener("click", () =>
      guarded(() => api("/api/control/movement_script", { method: "POST", body: { script_path: el.getAttribute("data-script") } }), "脚本已执行")));
    document.getElementById("custom-action-go").addEventListener("click", () => {
      const name = document.getElementById("custom-action-name").value.trim();
      if (!name) { toast("请输入动作名称", "error"); return; }
      guarded(() => api("/api/control/custom_action", { method: "POST", body: { action_name: name } }), "动作已执行");
    });
  }

  // ── 设置 ──
  // 与状态页同构：左边一列栏目，右边只显示选中栏目的内容。
  const SETTINGS_SECTIONS = [
    ["logs", "日志查看", "g1_base.log"],
    ["speed", "速度控制", "行走模式 · 线速度"],
    ["network", "网络连接", "网关地址 · 切换设备"],
    ["navstack", "导航栈管理", "就绪 · 重启 · 停止"],
    ["voice", "语音播报", "音量 · 事件提示"],
    ["about", "关于", "版本 · 显示信息"],
  ];

  function settingsShell(active, title, hint, bodyHtml) {
    shell({ back: "home", title: "设置", sub: "日志 · 参数" }, "cols-nav-main", `
      <aside class="pane">
        <div class="pane-head"><div class="eyebrow">设置栏目</div></div>
        <div class="pane-body flush scroll">
          <div class="side-list">
            ${SETTINGS_SECTIONS.map(([key, name, sub]) => `
              <button class="side-item ${key === active ? "active" : ""}" type="button" data-nav="settings/${key}">
                <span class="side-item-text">
                  <span class="side-item-name">${name}</span>
                  <span class="side-item-sub">${sub}</span>
                </span>
              </button>`).join("")}
          </div>
        </div>
      </aside>
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">${title}</div>${hint ? `<span class="hint">${hint}</span>` : ""}</div>
        ${bodyHtml}
      </section>
    `);
  }

  function renderSettings(sub) {
    startPolling();
    const active = sub && sub[0] ? sub[0] : "logs";
    if (active === "speed") return renderSettingsSpeed(active);
    if (active === "network") return renderSettingsNetwork(active);
    if (active === "navstack") return renderSettingsNavStack(active);
    if (active === "voice") return renderSettingsVoice(active);
    if (active === "about") return renderSettingsAbout(active);
    return renderSettingsLogs(active);
  }

  function renderSettingsLogs(active) {
    settingsShell(active, "运行日志 (g1_base.log)", "", `
      <div class="pane-body" style="display:flex;flex-direction:column;gap:10px">
        <div class="btn-row">
          <button class="btn sm" id="log-refresh">刷新</button>
          <select id="log-lines" class="inline-input" style="width:auto;padding:6px 10px">
            <option value="100">最近100行</option><option value="200" selected>最近200行</option><option value="500">最近500行</option>
          </select>
        </div>
        <div class="log-box" id="log-box">加载中…</div>
      </div>
    `);
    async function load() {
      const lines = document.getElementById("log-lines").value;
      try {
        const res = await api(`/api/logs?lines=${lines}`);
        const box = document.getElementById("log-box");
        const rawLines = res.lines || [];
        if (!rawLines.length) { box.textContent = "（无日志）"; return; }
        // 按日志等级着色：ERROR=红、WARN=黄、DEBUG=灰、其余默认
        box.innerHTML = rawLines.map(line => {
          const escaped = escapeHtml(line);
          let cls = "log-line-info";
          if (/\bERROR\b|\bCRITICAL\b/i.test(line)) cls = "log-line-error";
          else if (/\bWARN(ING)?\b/i.test(line)) cls = "log-line-warn";
          else if (/\bDEBUG\b/i.test(line)) cls = "log-line-debug";
          return `<span class="${cls}">${escaped}</span>`;
        }).join("\n");
        box.scrollTop = box.scrollHeight;
      } catch (err) {
        document.getElementById("log-box").textContent = "加载失败: " + err.message;
      }
    }
    document.getElementById("log-refresh").addEventListener("click", load);
    document.getElementById("log-lines").addEventListener("change", load);
    load();
  }

  async function renderSettingsSpeed(active) {
    settingsShell(active, "速度控制", "改完需重启导航栈", `
      <div class="pane-body scroll">
        <div id="speed-body"><div class="center-text">加载中…</div></div>
        <p class="note">
          <b>固定腰部 (locked_waist)</b>：行走时锁住腰部自由度，姿态更稳，适合展厅内低速讲解巡航。<br>
          <b>解锁腰部 (unlocked_waist)</b>：允许腰部参与平衡，转向更灵活、速度上限更高，适合空旷场地。<br>
          线速度只影响「控制 › 方向移动」这类手动指令，导航过程中的速度由 Nav2 参数决定。
        </p>
      </div>
    `);
    const container = document.getElementById("speed-body");
    let settings;
    try { settings = await api("/api/settings/walking_mode"); }
    catch (err) { container.innerHTML = `<div class="center-text">加载失败: ${escapeHtml(err.message)}</div>`; return; }
    const mode = settings.walking_mode || "unlocked_waist";
    const linearSpeed = (settings[mode] && settings[mode].linear_speed) || 0.35;
    container.innerHTML = `
      <div class="field">
        <label>模式</label>
        <select id="mode-select">
          <option value="locked_waist" ${mode === "locked_waist" ? "selected" : ""}>固定腰部 (locked_waist)</option>
          <option value="unlocked_waist" ${mode === "unlocked_waist" ? "selected" : ""}>解锁腰部 (unlocked_waist)</option>
        </select>
      </div>
      <div class="field">
        <label>手动移动线速度 (m/s): <span id="speed-val">${linearSpeed}</span></label>
        <div class="slider-row"><input id="speed-slider" type="range" min="0.1" max="1.0" step="0.05" value="${linearSpeed}" /></div>
      </div>
      <button class="btn primary block" id="speed-save">保存</button>
    `;
    document.getElementById("speed-slider").addEventListener("input", (e) => { document.getElementById("speed-val").textContent = e.target.value; });
    document.getElementById("speed-save").addEventListener("click", () => guarded(() => api("/api/settings/walking_mode", {
      method: "PUT",
      body: { mode: document.getElementById("mode-select").value, linear_speed: parseFloat(document.getElementById("speed-slider").value) },
    }), "已保存"));
  }

  // 事件在界面上的排序和说明。key 要和 voice_prompts.yaml 对得上。
  const VOICE_EVENTS = [
    ["events", "nav_start",    "开始导航时"],
    ["events", "nav_arrived",  "到达目标点时"],
    ["events", "nav_failed",   "导航失败时"],
    ["events", "patrol_start", "开始巡航时"],
    ["events", "patrol_done",  "巡航结束时"],
    ["alerts", "stack_error",  "导航栈异常时"],
    ["alerts", "estop",        "急停触发时"],
    ["alerts", "battery_low",  "电量过低时（电量尚未接入，暂不会触发）"],
  ];

  async function renderSettingsVoice(active) {
    let prompts = null, vol = null;
    try { prompts = await api("/api/audio/prompts"); } catch (_e) { /* 用默认渲染 */ }
    try { vol = await api("/api/audio/volume"); } catch (_e) { /* 音频没起来 */ }

    const volValue = vol && vol.volume != null ? vol.volume : null;
    const rows = prompts ? VOICE_EVENTS.map(([sec, key, label]) => {
      const item = (prompts[sec] || {})[key] || {};
      return `
        <div class="voice-row">
          <label class="voice-toggle">
            <input type="checkbox" data-v-sec="${sec}" data-v-key="${key}" ${item.enabled ? "checked" : ""} />
            <span>${escapeHtml(label)}</span>
          </label>
          <input class="inline-input" type="text" data-v-text="${key}"
                 value="${escapeHtml(item.text || "")}" placeholder="留空则不播" maxlength="60" />
        </div>`;
    }).join("") : `<p class="note">读取播报设置失败，检查网关是否在运行。</p>`;

    settingsShell(active, "语音播报", "改完即时生效", `
      <div class="pane-body scroll">
        <div class="field-row">
          <div class="field" style="flex:1">
            <label>音量 ${volValue == null ? "（读取失败，音频服务可能未就绪）" : ""}</label>
            <input id="vol-range" type="range" min="0" max="100" step="5"
                   value="${volValue == null ? 60 : volValue}" ${volValue == null ? "disabled" : ""} />
          </div>
          <div class="field" style="width:5.5em">
            <label>当前</label>
            <div class="kv-v" id="vol-view">${volValue == null ? "—" : volValue}</div>
          </div>
        </div>

        <div class="field" style="margin-top:12px">
          <label>播报到哪儿</label>
          <div class="sb-tabs" id="voice-output">
            ${[["both", "两者"], ["robot", "机器人扬声器"], ["browser", "本机浏览器"]]
              .map(([k, name]) => `<button class="${
                (prompts && prompts.output || "both") === k ? "active" : ""
              }" data-output="${k}">${name}</button>`).join("")}
          </div>
          <p class="note" style="margin-top:6px">
            机器人扬声器要靠 SDK 的音频服务活着，现场时好时坏；浏览器只要页面开着就一定响。
            ${speech.supported ? "" : "<b>当前浏览器不支持语音合成，「本机浏览器」这项不会有声音。</b>"}
          </p>
        </div>

        <label class="voice-toggle" style="margin:10px 0 4px">
          <input type="checkbox" id="voice-master" ${prompts && prompts.enabled ? "checked" : ""} />
          <span><b>启用事件播报</b></span>
        </label>
        <p class="note" style="margin-top:0">
          关掉之后机器人只在巡航讲解时说话；下面逐条的开关不受影响，重新打开即恢复。
        </p>

        <div class="voice-list">${rows}</div>
      </div>
      <div class="pane-foot">
        <button class="btn primary" id="voice-save">保存播报设置</button>
        <button class="btn" id="voice-test">试听（机器人）</button>
        <button class="btn" id="voice-test-browser">试听（本机）</button>
      </div>
    `);

    const volRange = document.getElementById("vol-range");
    const volView = document.getElementById("vol-view");
    if (volRange) {
      // 拖动时只更新显示，松手才下发——每动一格都发一次请求会把服务打满
      volRange.addEventListener("input", () => { volView.textContent = volRange.value; });
      volRange.addEventListener("change", async () => {
        try {
          const res = await api("/api/audio/volume", { method: "POST", body: { volume: Number(volRange.value) } });
          if (res && res.volume != null) volView.textContent = res.volume;
        } catch (err) { toast(String(err.message || err), "error"); }
      });
    }

    let outputMode = (prompts && prompts.output) || "both";
    const outputBox = document.getElementById("voice-output");
    if (outputBox) {
      outputBox.addEventListener("click", (ev) => {
        const btn = ev.target.closest("[data-output]");
        if (!btn) return;
        outputMode = btn.getAttribute("data-output");
        [...outputBox.children].forEach((c) => c.classList.toggle("active", c === btn));
      });
    }

    document.getElementById("voice-test-browser").addEventListener("click", () => {
      unlockSpeech();
      if (!speech.supported) { toast("当前浏览器不支持语音合成", "error"); return; }
      speakInBrowser("语音播报测试，这是本机浏览器发出的声音");
    });

    bindBusy("voice-test", "播报中…",
             () => api("/api/audio/say", { method: "POST", body: { text: "语音播报测试，当前音量正常" } }),
             "已发送播报");

    bindBusy("voice-save", "保存中…", async () => {
      const patch = {
        enabled: document.getElementById("voice-master").checked,
        output: outputMode,
        events: {}, alerts: {},
      };
      VOICE_EVENTS.forEach(([sec, key]) => {
        const box = document.querySelector(`[data-v-sec="${sec}"][data-v-key="${key}"]`);
        const txt = document.querySelector(`[data-v-text="${key}"]`);
        if (!box || !txt) return;
        patch[sec][key] = { enabled: box.checked, text: txt.value.trim() };
      });
      return api("/api/audio/prompts", { method: "POST", body: patch });
    }, "播报设置已保存");
  }

  function renderSettingsNetwork(active) {
    const sys = (state.status && state.status.system) || {};
    settingsShell(active, "网络连接", "", `
      <div class="pane-body scroll">
        <div class="kv-list roomy">
          ${kvHtml("网关地址", state.baseUrl)}
          ${kvHtml("机器人 IP", sys.ip || "—")}
          ${kvHtml("主机名", sys.hostname || "—")}
          ${kvHtml("连接状态", state.statusError ? "断开" : "正常", state.statusError ? "is-crit" : "is-ok", state.statusError ? "bad" : "ok")}
        </div>
        <p class="note">断开后会回到连接页，可以换一台机器人的 IP 重新接入；本机记住的地址存在浏览器里，不影响机器人端。</p>
      </div>
      <div class="pane-foot"><button class="btn danger" id="disconnect">断开并切换设备</button></div>
    `);
    document.getElementById("disconnect").addEventListener("click", () => { state.baseUrl = ""; localStorage.removeItem("g1_base_url"); nav("connect"); });
  }

  function renderSettingsNavStack(active) {
    const navm = (state.status && state.status.navigation_manager) || {};
    settingsShell(active, "导航栈管理", "", `
      <div class="pane-body scroll">
        <div class="kv-list roomy">
          ${kvHtml("状态", navm.state || "—", navm.ready ? "is-ok" : "is-warn", navm.ready ? "ok" : "warn")}
          ${kvHtml("模式", navm.mode || "—")}
          ${kvHtml("整体就绪", navm.ready ? "是" : "否", navm.ready ? "is-ok" : "is-warn")}
          ${kvHtml("重启次数", navm.restart_count != null ? navm.restart_count : "—")}
        </div>
        <p class="note">「就绪检查」只做一次健康探测；「重启导航栈」会重拉定位与 Nav2，过程中机器人不接受导航指令；「停止全部」用于收工或排障。</p>
      </div>
      <div class="pane-foot">
        <button class="btn" id="btn-ensure">就绪检查</button>
        <button class="btn warn" id="btn-restart">重启导航栈</button>
        <button class="btn danger" id="btn-stopall">停止全部</button>
      </div>
    `);
    bindBusy("btn-ensure", "探测中…",
             () => api("/api/nav/ensure_ready", { method: "POST" }), "已请求");
    // 这条最慢：要停掉再重拉定位与 Nav2，现场实测十几秒
    bindBusy("btn-restart", "重拉定位与 Nav2…",
             () => api("/api/nav/restart_all", { method: "POST" }), "已重启");
    bindBusy("btn-stopall", "停止中…",
             () => api("/api/nav/stop_all", { method: "POST" }), "已停止");
  }

  function renderSettingsAbout(active) {
    const sys = (state.status && state.status.system) || {};
    settingsShell(active, "关于", "", `
      <div class="pane-body scroll">
        <div class="kv-list roomy">
          ${kvHtml("前端版本", "2.2.0")}
          ${kvHtml("网关运行", fmtDuration(sys.uptime_sec))}
          ${kvHtml("显示视口", window.innerWidth + "×" + window.innerHeight)}
          ${kvHtml("像素比", window.devicePixelRatio)}
        </div>
        <p class="note">界面按当前视口自适应，为横屏平板设计。若排版偏挤或偏空，把「显示视口」和「像素比」报给开发即可精调。</p>
      </div>
    `);
  }

  // ── 示教 ──
  async function renderTeach() {
    startPolling();
    shell({ back: "home", title: "示教", sub: "录制 · 回放" }, "cols-3", `
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">已录制动作</div><span class="hint" id="cnt-teach-named"></span></div>
        <div class="pane-body scroll"><div class="chip-grid" id="teach-named"><div class="center-text">加载中…</div></div></div>
      </section>
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">编排脚本</div><span class="hint" id="cnt-teach-script"></span></div>
        <div class="pane-body scroll"><div class="chip-grid" id="teach-script"></div></div>
      </section>
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">SDK 示教动作</div></div>
        <div class="pane-body">
          <p class="note" style="margin-top:0">
            左侧是 config/movement/motions/ 下已录制的关节轨迹，中间是 movement/scripts/ 下的编排脚本，点一下即在机器人上回放。<br><br>
            网关不提供在线录制：新动作要在机器人端用 g1_teach_v2 录好后放进对应目录，刷新本页即可出现。
          </p>
        </div>
        <div class="pane-foot">
          <input id="teach-custom" class="inline-input" placeholder="APP 端录制的动作名称" style="flex:1" />
          <button class="btn primary" id="teach-custom-go">回放</button>
        </div>
      </section>
    `);
    const [named, scripts] = await Promise.all([
      api("/api/actions/named").catch(() => []),
      api("/api/actions/scripts").catch(() => []),
    ]);
    document.getElementById("teach-named").innerHTML =
      named.map((n) => `<button class="btn" data-named="${escapeHtml(n)}">${escapeHtml(n)}</button>`).join("") || `<div class="center-text">无</div>`;
    document.getElementById("teach-script").innerHTML =
      scripts.map((s) => `<button class="btn" data-script="${escapeHtml(s)}">${escapeHtml(s.replace(/\.json$/, ""))}</button>`).join("") || `<div class="center-text">无</div>`;
    document.getElementById("cnt-teach-named").textContent = `${named.length} 项`;
    document.getElementById("cnt-teach-script").textContent = `${scripts.length} 项`;

    app.querySelectorAll("[data-named]").forEach((el) => el.addEventListener("click", () =>
      guarded(() => api("/api/control/named_action", { method: "POST", body: { action_name: el.getAttribute("data-named") } }), "动作已执行")));
    app.querySelectorAll("[data-script]").forEach((el) => el.addEventListener("click", () =>
      guarded(() => api("/api/control/movement_script", { method: "POST", body: { script_path: el.getAttribute("data-script") } }), "脚本已执行")));
    document.getElementById("teach-custom-go").addEventListener("click", () => {
      const name = document.getElementById("teach-custom").value.trim();
      if (!name) { toast("请输入动作名称", "error"); return; }
      guarded(() => api("/api/control/custom_action", { method: "POST", body: { action_name: name } }), "动作已执行");
    });
  }

  // ── 状态 ──
  // 左边一列栏目（自带实时摘要），右边只显示选中栏目的明细。
  const STATE_SECTIONS = [
    ["run", "运行"],
    ["sdk", "SDK 链路"],
    ["nav", "导航栈"],
    ["obstacle", "避障"],
    ["host", "主机"],
  ];

  function telemetryGroups() {
    if (!state.status) return null;
    const s = state.status;
    const c = s.control || {};
    const navm = s.navigation_manager || {};
    const sys = s.system || {};
    return {
      run: {
        summary: c.activity || "空闲",
        tone: c.stop_latched ? "is-crit" : "",
        rows: [
          ["当前活动", c.activity || "—"],
          ["活动详情", c.activity_detail || "—"],
          ["活动时长", c.activity_duration != null ? c.activity_duration + " s" : "—"],
          ["运动来源", c.motion_source || "—"],
          ["蹲下状态", c.is_squatting ? "是" : "否"],
          ["急停锁定", c.stop_latched ? "是" : "否", c.stop_latched ? "is-crit" : ""],
        ],
      },
      sdk: {
        summary: c.sdk_loco_latency_p95_ms != null ? `P95 ${c.sdk_loco_latency_p95_ms} ms` : "无数据",
        rows: [
          ["最近延迟", c.sdk_last_loco_latency_ms != null ? c.sdk_last_loco_latency_ms + " ms" : "—"],
          ["P95 延迟", c.sdk_loco_latency_p95_ms != null ? c.sdk_loco_latency_p95_ms + " ms" : "—"],
          ["慢包计数", c.sdk_slow_count != null ? c.sdk_slow_count : "—"],
        ],
      },
      nav: {
        summary: navm.state || "未连接",
        tone: navm.ready ? "is-ok" : navm.state === "ERROR" ? "is-crit" : "is-warn",
        rows: [
          ["状态", navm.state || "—", navm.ready ? "is-ok" : ""],
          ["模式", navm.mode || "—"],
          ["整体就绪", navm.ready ? "是" : "否", navm.ready ? "is-ok" : "is-warn"],
          ["定位就绪", navm.localization_ready ? "是" : "否"],
          ["TF 就绪", navm.tf_ready ? "是" : "否"],
          ["BT 导航器", navm.bt_navigator_state || "—"],
          ["重启次数", navm.restart_count != null ? navm.restart_count : "—"],
          ["状态原因", navm.state_reason || "—"],
        ],
      },
      obstacle: {
        summary: c.close_obstacle_active ? "触发" : "正常",
        tone: c.close_obstacle_active ? "is-crit" : "is-ok",
        rows: [
          ["近距障碍", c.close_obstacle_active ? "触发" : "正常", c.close_obstacle_active ? "is-crit" : "is-ok"],
          ["前方最近", c.close_obstacle_front_min != null ? c.close_obstacle_front_min + " m" : "—"],
          ["触发阈值", c.close_obstacle_trigger_distance != null ? c.close_obstacle_trigger_distance + " m" : "—"],
          ["导航失败原因", c.navigation_failure_reason || "—"],
        ],
      },
      host: {
        summary: sys.cpu_percent != null ? `CPU ${sys.cpu_percent}% · 内存 ${sys.mem_percent}%` : "无数据",
        rows: [
          ["主机名", sys.hostname || "—"],
          ["IP", sys.ip || "—"],
          ["CPU", sys.cpu_percent != null ? sys.cpu_percent + " %" : "—"],
          ["内存", sys.mem_percent != null ? sys.mem_percent + " %" : "—"],
          ["网关运行", fmtDuration(sys.uptime_sec)],
        ],
      },
    };
  }

  function stateNavHtml() {
    const groups = telemetryGroups();
    const active = currentRoute()[1] || "run";
    return STATE_SECTIONS.map(([key, name]) => {
      const g = groups && groups[key];
      return `
        <button class="side-item ${key === active ? "active" : ""}" type="button" data-nav="state/${key}">
          <span class="side-item-text">
            <span class="side-item-name">${name}</span>
            <span class="side-item-sub ${(g && g.tone) || ""}">${escapeHtml(g ? String(g.summary) : "—")}</span>
          </span>
        </button>`;
    }).join("");
  }

  function stateDetailHtml() {
    const groups = telemetryGroups();
    if (!groups) return `<div class="center-text">加载中…</div>`;
    const key = currentRoute()[1] || "run";
    const g = groups[key] || groups.run;
    return `<div class="kv-list roomy">${g.rows.map(([k, v, tone]) => kvHtml(k, v, tone || "")).join("")}</div>`;
  }

  function renderState(sub) {
    startPolling();
    const key = (sub && sub[0]) || "run";
    const section = STATE_SECTIONS.find(([k]) => k === key) || STATE_SECTIONS[0];
    shell({ back: "home", title: "状态", sub: "遥测 · 诊断" }, "cols-nav-main", `
      <aside class="pane">
        <div class="pane-head"><div class="eyebrow">诊断栏目</div></div>
        <div class="pane-body flush scroll"><div class="side-list" data-live="state-nav">${stateNavHtml()}</div></div>
      </aside>
      <section class="pane">
        <div class="pane-head"><div class="eyebrow">${section[1]}</div><span class="hint">每 1.5 秒刷新</span></div>
        <div class="pane-body scroll" data-live="state-detail">${stateDetailHtml()}</div>
      </section>
    `);
  }


  // ── 主渲染分发 ──
  function render() {
    const parts = currentRoute();
    if (!state.baseUrl) { disposeRobot3d(); renderConnect(); return; }
    const section = parts[0] || "home";
    const rest = parts.slice(1);
    if (section !== "home") disposeRobot3d();
    switch (section) {
      case "connect": renderConnect(); break;
      case "home": renderHome(); break;
      case "map": renderMap(rest[0], rest.slice(1)); break;
      case "nav": renderNav(rest); break;
      case "control": renderControl(rest); break;
      case "teach": renderTeach(); break;
      case "state": renderState(rest); break;
      case "settings": renderSettings(rest); break;
      default: renderHome();
    }
  }

  blockPageZoom();
  render();
})();
