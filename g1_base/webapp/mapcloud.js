/* 建图三维点云视图：把 super-lio 的 /lio/cloud_world 实时画成 3D 点云。
   依赖全局 THREE（assets/three.min.js）。

   数据通路：网关按体素去重后按到达顺序存着，前端拿 since 游标增量拉
   float32 裸字节，贴到一块预分配的 BufferGeometry 上，只推进 drawRange。
   这样走一圈几万个体素也不会每次重传整片地图。 */
(() => {
  "use strict";

  const MAX_POINTS = 500000;        // 预分配容量，约 6MB 显存
  const CHUNK = 60000;              // 单次最多拉这么多点
  const POLL_MS = 700;

  // 高度上色：低处冷、高处暖，一眼能分出地面 / 桌面 / 天花板
  function heightColor(t, out, i) {
    // t ∈ [0,1] → 深蓝 → 青 → 绿 → 黄 → 红
    const stops = [
      [0.10, 0.25, 0.55],
      [0.12, 0.62, 0.68],
      [0.30, 0.72, 0.42],
      [0.90, 0.76, 0.30],
      [0.86, 0.36, 0.32],
    ];
    const x = Math.max(0, Math.min(0.9999, t)) * (stops.length - 1);
    const k = Math.floor(x);
    const f = x - k;
    const a = stops[k], b = stops[k + 1] || stops[k];
    out[i] = a[0] + (b[0] - a[0]) * f;
    out[i + 1] = a[1] + (b[1] - a[1]) * f;
    out[i + 2] = a[2] + (b[2] - a[2]) * f;
  }

  class CloudViewer {
    constructor(container, opts = {}) {
      this.container = container;
      this.baseUrl = opts.baseUrl || "";
      this.onStats = opts.onStats || null;
      this.cursor = 0;
      this.count = 0;
      this.zMin = 0;
      this.zMax = 1;
      this.disposed = false;
      this._raf = null;
      this._timer = null;
      this._busy = false;
      this._pointers = new Map();
      this._pinch = 0;
      // 相机：绕原点的球坐标，和 3D 机器人视图同一套手感
      this._cam = { theta: -Math.PI * 0.25, phi: 1.05, radius: 18 };
      this._camTarget = { ...this._cam };
      this._pivot = new THREE.Vector3(0, 0, 0);
      this._follow = true;          // 跟随机器人（有位姿时把视角中心挪过去）
    }

    init() {
      const w = this.container.clientWidth || 640;
      const h = this.container.clientHeight || 480;

      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0x0f1720);

      this.camera = new THREE.PerspectiveCamera(55, w / h, 0.1, 400);
      this.renderer = new THREE.WebGLRenderer({ antialias: true });
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this.renderer.setSize(w, h);
      this.container.appendChild(this.renderer.domElement);

      // 地面网格：1 米一格，给点云一个尺度参照
      const grid = new THREE.GridHelper(60, 60, 0x2a4d5c, 0x1b2f3a);
      grid.rotation.x = Math.PI / 2;   // GridHelper 默认在 XZ 面，转到 XY（ROS 是 Z 朝上）
      this.scene.add(grid);

      // 世界原点的三轴，红=X 绿=Y 蓝=Z
      const axes = new THREE.AxesHelper(1.2);
      this.scene.add(axes);

      // 点云本体：一次性分配满容量，之后只改 drawRange
      const geom = new THREE.BufferGeometry();
      this.positions = new Float32Array(MAX_POINTS * 3);
      this.colors = new Float32Array(MAX_POINTS * 3);
      geom.setAttribute("position", new THREE.BufferAttribute(this.positions, 3));
      geom.setAttribute("color", new THREE.BufferAttribute(this.colors, 3));
      geom.setDrawRange(0, 0);
      this.geometry = geom;
      this.points = new THREE.Points(
        geom,
        new THREE.PointsMaterial({ size: 0.05, vertexColors: true, sizeAttenuation: true }),
      );
      this.scene.add(this.points);

      // 机器人位置：一个锥体指向朝向
      const robot = new THREE.Group();
      const body = new THREE.Mesh(
        new THREE.ConeGeometry(0.18, 0.55, 16),
        new THREE.MeshBasicMaterial({ color: 0x2fd6c3 }),
      );
      body.rotation.z = -Math.PI / 2;   // 锥尖朝 +X
      robot.add(body);
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.35, 0.42, 32),
        new THREE.MeshBasicMaterial({ color: 0x2fd6c3, transparent: true, opacity: 0.55, side: THREE.DoubleSide }),
      );
      robot.add(ring);
      robot.visible = false;
      this.robot = robot;
      this.scene.add(robot);

      // 走过的轨迹
      const trailGeom = new THREE.BufferGeometry();
      this.trail = new Float32Array(6000 * 3);
      this.trailCount = 0;
      trailGeom.setAttribute("position", new THREE.BufferAttribute(this.trail, 3));
      trailGeom.setDrawRange(0, 0);
      this.trailGeom = trailGeom;
      this.scene.add(new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: 0x2fd6c3 })));

      this._bindControls();
      this._onResize = () => this.resize();
      window.addEventListener("resize", this._onResize);
      this._animate();
      this.start();
    }

    resize() {
      if (this.disposed || !this.renderer) return;
      const w = this.container.clientWidth, h = this.container.clientHeight;
      if (!w || !h) return;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(w, h);
    }

    // ── 手势：拖拽转视角，双指缩放，双击回到俯视 ──
    _bindControls() {
      const el = this.renderer.domElement;
      el.style.touchAction = "none";
      const pts = this._pointers;
      const dist = () => {
        const [a, b] = [...pts.values()];
        return Math.hypot(a.x - b.x, a.y - b.y);
      };
      this._down = (ev) => {
        el.setPointerCapture(ev.pointerId);
        pts.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
        if (pts.size === 2) this._pinch = dist();
      };
      this._move = (ev) => {
        const prev = pts.get(ev.pointerId);
        if (!prev) return;
        const cur = { x: ev.clientX, y: ev.clientY };
        pts.set(ev.pointerId, cur);
        if (pts.size >= 2) {
          const d = dist();
          if (this._pinch > 0 && d > 0) {
            this._camTarget.radius = Math.max(2, Math.min(120, this._camTarget.radius * (this._pinch / d)));
          }
          this._pinch = d;
          return;
        }
        const w = this.container.clientWidth || 1;
        this._camTarget.theta -= ((cur.x - prev.x) / w) * Math.PI * 2;
        this._camTarget.phi = Math.min(1.54, Math.max(0.06,
          this._camTarget.phi - ((cur.y - prev.y) / w) * Math.PI * 1.4));
      };
      this._up = (ev) => {
        pts.delete(ev.pointerId);
        if (pts.size < 2) this._pinch = 0;
      };
      this._wheel = (ev) => {
        ev.preventDefault();
        this._camTarget.radius = Math.max(2, Math.min(120, this._camTarget.radius * (ev.deltaY > 0 ? 1.1 : 0.9)));
      };
      this._dbl = () => this.resetView();
      el.addEventListener("pointerdown", this._down);
      el.addEventListener("pointermove", this._move);
      el.addEventListener("pointerup", this._up);
      el.addEventListener("pointercancel", this._up);
      el.addEventListener("pointerleave", this._up);
      el.addEventListener("wheel", this._wheel, { passive: false });
      el.addEventListener("dblclick", this._dbl);
    }

    resetView() {
      this._camTarget.theta = -Math.PI * 0.25;
      this._camTarget.phi = 1.05;
      this._camTarget.radius = 18;
    }

    setFollow(on) { this._follow = !!on; }

    _animate() {
      const tick = () => {
        if (this.disposed) return;
        this._raf = requestAnimationFrame(tick);
        const c = this._cam, t = this._camTarget;
        c.theta += (t.theta - c.theta) * 0.2;
        c.phi += (t.phi - c.phi) * 0.2;
        c.radius += (t.radius - c.radius) * 0.2;
        const sp = Math.sin(c.phi);
        // ROS 是 Z 朝上，直接用世界坐标摆相机
        this.camera.position.set(
          this._pivot.x + c.radius * sp * Math.cos(c.theta),
          this._pivot.y + c.radius * sp * Math.sin(c.theta),
          this._pivot.z + c.radius * Math.cos(c.phi),
        );
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt(this._pivot);
        this.renderer.render(this.scene, this.camera);
      };
      this._raf = requestAnimationFrame(tick);
    }

    // ── 增量拉取点云 ──
    start() {
      this.stop();
      const pull = async () => {
        if (this._busy || this.disposed) return;
        this._busy = true;
        try {
          const res = await fetch(`${this.baseUrl}/api/map/live/cloud?since=${this.cursor}&max=${CHUNK}`,
                                  { cache: "no-store" });
          if (res.ok) {
            const total = parseInt(res.headers.get("X-Cloud-Total") || "0", 10);
            const end = parseInt(res.headers.get("X-Cloud-End") || "0", 10);
            const zmin = parseFloat(res.headers.get("X-Cloud-Zmin") || "0");
            const zmax = parseFloat(res.headers.get("X-Cloud-Zmax") || "1");
            const voxel = parseFloat(res.headers.get("X-Cloud-Voxel") || "0.08");
            const buf = await res.arrayBuffer();
            if (buf.byteLength) this._append(new Float32Array(buf), zmin, zmax);
            this.cursor = end;
            this.points.material.size = Math.max(0.02, voxel * 0.9);
            if (this.onStats) {
              this.onStats({ shown: this.count, total, zmin, zmax, voxel });
            }
            // 还没追上就立刻再拉一次，首次进页面能快速把已有的图铺出来
            if (end < total && !this.disposed) { this._busy = false; return pull(); }
          }
        } catch (_e) { /* 网络抖动等下一轮 */ }
        finally { this._busy = false; }
      };
      pull();
      this._timer = setInterval(pull, POLL_MS);
    }

    stop() {
      if (this._timer) clearInterval(this._timer);
      this._timer = null;
    }

    _append(xyz, zmin, zmax) {
      const n = Math.floor(xyz.length / 3);
      if (!n) return;
      // z 范围变了就把已有点的颜色重算一遍，免得前后两批配色对不上
      const spanChanged = Math.abs(zmin - this.zMin) > 1e-3 || Math.abs(zmax - this.zMax) > 1e-3;
      this.zMin = zmin;
      this.zMax = Math.max(zmax, zmin + 0.01);
      const span = this.zMax - this.zMin;

      let base = this.count;
      const room = MAX_POINTS - base;
      const take = Math.min(n, room);
      for (let i = 0; i < take; i++) {
        const p = (base + i) * 3;
        const x = xyz[i * 3], y = xyz[i * 3 + 1], z = xyz[i * 3 + 2];
        this.positions[p] = x;
        this.positions[p + 1] = y;
        this.positions[p + 2] = z;
        heightColor((z - this.zMin) / span, this.colors, p);
      }
      this.count = base + take;

      if (spanChanged && base > 0) {
        for (let i = 0; i < base; i++) {
          const p = i * 3;
          heightColor((this.positions[p + 2] - this.zMin) / span, this.colors, p);
        }
      }
      this.geometry.attributes.position.needsUpdate = true;
      this.geometry.attributes.color.needsUpdate = true;
      this.geometry.setDrawRange(0, this.count);
      this.geometry.computeBoundingSphere();
    }

    // 机器人位姿（世界系），由页面按状态轮询喂进来
    setPose(pose) {
      if (!this.robot) return;
      if (!pose) { this.robot.visible = false; return; }
      this.robot.visible = true;
      this.robot.position.set(pose.x, pose.y, 0.3);
      this.robot.rotation.set(0, 0, (pose.yaw_deg || 0) * Math.PI / 180);
      if (this._follow) this._pivot.set(pose.x, pose.y, 0.6);

      const i = this.trailCount * 3;
      const last = this.trailCount ? [this.trail[i - 3], this.trail[i - 2]] : null;
      if (!last || Math.hypot(pose.x - last[0], pose.y - last[1]) > 0.12) {
        if (this.trailCount < 6000) {
          this.trail[i] = pose.x;
          this.trail[i + 1] = pose.y;
          this.trail[i + 2] = 0.06;
          this.trailCount += 1;
          this.trailGeom.attributes.position.needsUpdate = true;
          this.trailGeom.setDrawRange(0, this.trailCount);
        }
      }
    }

    clear() {
      this.cursor = 0;
      this.count = 0;
      this.trailCount = 0;
      this.geometry.setDrawRange(0, 0);
      this.trailGeom.setDrawRange(0, 0);
    }

    dispose() {
      this.disposed = true;
      this.stop();
      if (this._raf) cancelAnimationFrame(this._raf);
      if (this._onResize) window.removeEventListener("resize", this._onResize);
      if (this.renderer) {
        const el = this.renderer.domElement;
        el.removeEventListener("pointerdown", this._down);
        el.removeEventListener("pointermove", this._move);
        el.removeEventListener("pointerup", this._up);
        el.removeEventListener("pointercancel", this._up);
        el.removeEventListener("pointerleave", this._up);
        el.removeEventListener("wheel", this._wheel);
        el.removeEventListener("dblclick", this._dbl);
        this.renderer.dispose();
        if (el.parentNode) el.parentNode.removeChild(el);
      }
    }
  }

  window.G1MapCloud = {
    create(container, opts) {
      if (typeof THREE === "undefined") {
        container.innerHTML = '<div class="center-text">3D 引擎未加载</div>';
        return null;
      }
      const v = new CloudViewer(container, opts);
      try { v.init(); } catch (err) {
        console.error("点云视图初始化失败", err);
        container.innerHTML = `<div class="center-text">点云视图初始化失败<br><small>${err.message}</small></div>`;
        return null;
      }
      return v;
    },
  };
})();
