/* G1 机器人 3D 可视化：加载 URDF 结构 + 打包网格，播放真实示教动作轨迹。
   依赖全局 THREE（assets/three.min.js）。 */
(() => {
  "use strict";

  // 版本号由网关注入（见 g1_web_bridge._stamp_asset_versions）。资源换了 URL
  // 就变，平板上不会再拿到旧的模型或动作包；没变则完全走缓存，不重下 2.6MB。
  const ASSET_V = window.__G1_ASSET_V ? `?v=${window.__G1_ASSET_V}` : "";
  const ASSET = {
    model: `/assets/g1_model.json${ASSET_V}`,
    meshes: `/assets/g1_meshes.bin${ASSET_V}`,
    motions: `/assets/g1_motions.json${ASSET_V}`,
  };

  // 胸前品牌字。几何参数由 logo_link.STL 顶点实测得到：
  // 前表面是一段圆弧（半径≈0.40m，中心 x≈0.0746），并向后仰约 13.2°。
  const BRAND = {
    text: "Whale Cloud",
    radius: 0.40,
    centerX: 0.0746,
    centerZ: 0.2712,
    width: 0.132,      // 原 unitree 字样通宽约 150mm，这里取相近观感
    height: 0.030,
    tiltDeg: 13.2,
    lift: 0.0016,      // 沿法线抬起，避免与胸壳表面 z-fighting
  };

  // 头顶 RealSense D435（真实外形 90×25×25mm）。
  // head_link.STL 顶面在 x=+0.015 处 z≈0.5297，支架底面据此贴合。
  const DEPTH_CAM = {
    x: 0.015,
    z: 0.5523,
    pitchDeg: 8,       // 略微下俯
  };

  // 解析 pack_meshes.py 产出的二进制：量化顶点 + 索引三角形
  function parseMeshBundle(buffer) {
    const dv = new DataView(buffer);
    const magic = new TextDecoder().decode(new Uint8Array(buffer, 0, 8));
    if (magic !== "G1MESH01") throw new Error("网格文件格式不正确: " + magic);

    let off = 8;
    const count = dv.getUint32(off, true); off += 4;

    const headers = [];
    for (let i = 0; i < count; i++) {
      const nameLen = dv.getUint16(off, true); off += 2;
      const name = new TextDecoder().decode(new Uint8Array(buffer, off, nameLen)); off += nameLen;
      const vertCount = dv.getUint32(off, true); off += 4;
      const triCount = dv.getUint32(off, true); off += 4;
      const bbox = [];
      for (let k = 0; k < 6; k++) { bbox.push(dv.getFloat32(off, true)); off += 4; }
      headers.push({ name, vertCount, triCount, bbox });
    }

    const out = {};
    for (const h of headers) {
      const quant = new Uint16Array(buffer, off, h.vertCount * 3);
      off += h.vertCount * 3 * 2;
      const indices = new Uint32Array(buffer.slice(off, off + h.triCount * 3 * 4));
      off += h.triCount * 3 * 4;

      // 反量化回真实坐标
      const [minx, miny, minz, maxx, maxy, maxz] = h.bbox;
      const sx = (maxx - minx) / 65535, sy = (maxy - miny) / 65535, sz = (maxz - minz) / 65535;
      const pos = new Float32Array(h.vertCount * 3);
      for (let i = 0; i < h.vertCount; i++) {
        pos[i * 3] = minx + quant[i * 3] * sx;
        pos[i * 3 + 1] = miny + quant[i * 3 + 1] * sy;
        pos[i * 3 + 2] = minz + quant[i * 3 + 2] * sz;
      }

      const geom = new THREE.BufferGeometry();
      geom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      geom.setIndex(new THREE.BufferAttribute(indices, 1));
      geom.computeVertexNormals();
      out[h.name] = geom;
    }
    return out;
  }

  // 相机初始机位（球坐标，围绕 target）
  const CAM = {
    theta: Math.PI * 0.12,   // 方位角
    phi: 1.4573,             // 天顶角（从 +Y 轴量起）
    radius: 2.9,
    target: [0, 0.72, 0],
    phiMin: 0.25, phiMax: 1.72,
    radiusMin: 1.4, radiusMax: 6.5,
  };

  class RobotViewer {
    constructor(container) {
      this.container = container;
      this.jointObjects = {};   // jointName -> { articulation, axis }
      this.jointTargets = {};   // jointName -> 目标角度
      this.jointCurrent = {};   // jointName -> 当前角度（用于平滑）
      this.motions = null;
      this.playlist = [];
      this.playIndex = 0;
      this.motionStart = 0;
      this.phase = "idle";      // play | reset | idle
      this.autoAdvance = true;  // 手动选过动作后就不再自动轮播
      this.onMotionChange = null;
      this.disposed = false;
      this._raf = null;
      // 相机由手势驱动：目标值 + 当前值，中间做阻尼，手感跟手但不抖
      this._cam = { theta: CAM.theta, phi: CAM.phi, radius: CAM.radius };
      this._camTarget = { ...this._cam };
      this._pointers = new Map();
      this._pinchDist = 0;
    }

    async init() {
      const [modelRes, meshRes, motionRes] = await Promise.all([
        fetch(ASSET.model).then((r) => r.json()),
        fetch(ASSET.meshes).then((r) => r.arrayBuffer()),
        fetch(ASSET.motions).then((r) => r.json()),
      ]);
      if (this.disposed) return;

      this.model = modelRes;
      this.motions = motionRes;
      this.playlist = Object.keys(motionRes);
      this._trimMotions();
      const geometries = parseMeshBundle(meshRes);

      this._setupScene();
      this._buildRobot(geometries);
      this._animate();
      if (this.onReady) this.onReady(this.getPlaylist());
      // 先把动作名推一次，别等第一帧渲染完才显示
      this.phase = "play";
      this._emitMotionChange();
    }

    // 示教录制的轨迹头尾常带大段静止（举着手臂等开始、录完没立刻停），
    // 直接从 t=0 播会看上去像"没有动作"。这里按关节角变化量裁掉首尾静止段，
    // 只播真正有动作的窗口，前后各留 0.3s 余量。
    _trimMotions() {
      const EPS = 0.04;   // rad，约 2.3°，低于这个幅度肉眼看不出
      const deviation = (a, b) => {
        let mx = 0;
        for (let k = 0; k < a.length; k++) mx = Math.max(mx, Math.abs(a[k] - b[k]));
        return mx;
      };
      for (const key of this.playlist) {
        const m = this.motions[key];
        const frames = m.frames, times = m.times, n = frames.length;
        if (!n) { m.startTime = 0; m.playDuration = m.duration; continue; }
        let s = 0;
        while (s < n - 1 && deviation(frames[s], frames[0]) < EPS) s++;
        let e = n - 1;
        while (e > s && deviation(frames[e], frames[n - 1]) < EPS) e--;
        m.startTime = Math.max(times[0], times[s] - 0.3);
        const endTime = Math.min(times[n - 1], times[e] + 0.3);
        m.playDuration = Math.max(0.5, endTime - m.startTime);
      }
    }

    // 程序化影棚环境：顶部天光渐变到底部地面反射，供金属材质取样
    _makeStudioEnv() {
      const c = document.createElement("canvas");
      c.width = 16;
      c.height = 128;
      const g = c.getContext("2d");
      const grad = g.createLinearGradient(0, 0, 0, c.height);
      grad.addColorStop(0.0, "#ffffff");   // 顶光
      grad.addColorStop(0.42, "#eef1f4");
      grad.addColorStop(0.55, "#c0c6cc");  // 地平线
      grad.addColorStop(1.0, "#7e848a");   // 地面
      g.fillStyle = grad;
      g.fillRect(0, 0, c.width, c.height);

      const tex = new THREE.CanvasTexture(c);
      tex.mapping = THREE.EquirectangularReflectionMapping;
      const pmrem = new THREE.PMREMGenerator(this.renderer);
      const env = pmrem.fromEquirectangular(tex).texture;
      pmrem.dispose();
      tex.dispose();
      return env;
    }

    _setupScene() {
      const w = this.container.clientWidth || 480;
      const h = this.container.clientHeight || 520;

      this.scene = new THREE.Scene();

      this.camera = new THREE.PerspectiveCamera(38, w / h, 0.05, 60);
      this.camera.position.set(2.0, 0.95, 2.5);
      this.camera.lookAt(0, 0.75, 0);

      this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this.renderer.setSize(w, h);
      this.container.appendChild(this.renderer.domElement);

      // 金属必须靠环境反射才有质感，否则只会被直射光打成死白一片
      this.scene.environment = this._makeStudioEnv();

      // 有了环境光照后，直射光调低，只负责高光与轮廓
      this.scene.add(new THREE.HemisphereLight(0xffffff, 0xccd8e4, 0.30));
      const key = new THREE.DirectionalLight(0xffffff, 0.85);
      key.position.set(3, 5, 4);
      this.scene.add(key);
      const rim = new THREE.DirectionalLight(0xc2d6de, 0.32);
      rim.position.set(-3, 2, -3);
      this.scene.add(rim);
      const fill = new THREE.DirectionalLight(0xdceaf5, 0.25);
      fill.position.set(0, -2, 2);
      this.scene.add(fill);

      // 脚下的青色光环
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.42, 0.62, 64),
        new THREE.MeshBasicMaterial({ color: 0x17b6d4, transparent: true, opacity: 0.3, side: THREE.DoubleSide })
      );
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = 0.002;
      this.scene.add(ring);

      const grid = new THREE.GridHelper(6, 24, 0x8fc4d8, 0xcfe0ec);
      grid.material.transparent = true;
      grid.material.opacity = 0.65;
      this.scene.add(grid);

      this._onResize = () => {
        if (this.disposed) return;
        const cw = this.container.clientWidth, ch = this.container.clientHeight;
        if (!cw || !ch) return;
        this.camera.aspect = cw / ch;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(cw, ch);
      };
      window.addEventListener("resize", this._onResize);

      this._bindControls();
    }

    // ── 手势：单指拖拽转视角，双指捏合缩放，双击复位 ──
    _bindControls() {
      const el = this.renderer.domElement;
      el.style.touchAction = "none";   // 交给我们自己处理，浏览器不要抢
      const pointers = this._pointers;

      const pinchDistance = () => {
        const [a, b] = [...pointers.values()];
        return Math.hypot(a.x - b.x, a.y - b.y);
      };

      this._onPointerDown = (ev) => {
        el.setPointerCapture(ev.pointerId);
        pointers.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
        if (pointers.size === 2) this._pinchDist = pinchDistance();
      };

      this._onPointerMove = (ev) => {
        const prev = pointers.get(ev.pointerId);
        if (!prev) return;
        const cur = { x: ev.clientX, y: ev.clientY };
        pointers.set(ev.pointerId, cur);

        if (pointers.size >= 2) {
          const d = pinchDistance();
          if (this._pinchDist > 0 && d > 0) {
            this._camTarget.radius = this._clampRadius(this._camTarget.radius * (this._pinchDist / d));
          }
          this._pinchDist = d;
          return;
        }

        // 以容器宽度归一化，保证不同尺寸屏幕上手感一致：整屏拖动 ≈ 转一圈
        const w = this.container.clientWidth || 1;
        this._camTarget.theta -= ((cur.x - prev.x) / w) * Math.PI * 2;
        this._camTarget.phi = Math.min(
          CAM.phiMax,
          Math.max(CAM.phiMin, this._camTarget.phi - ((cur.y - prev.y) / w) * Math.PI * 1.4)
        );
      };

      this._onPointerUp = (ev) => {
        pointers.delete(ev.pointerId);
        if (pointers.size < 2) this._pinchDist = 0;
        if (el.hasPointerCapture && el.hasPointerCapture(ev.pointerId)) el.releasePointerCapture(ev.pointerId);
      };

      this._onWheel = (ev) => {
        ev.preventDefault();
        this._camTarget.radius = this._clampRadius(this._camTarget.radius * (ev.deltaY > 0 ? 1.1 : 0.9));
      };

      this._onDblClick = () => { this.resetCamera(); };

      el.addEventListener("pointerdown", this._onPointerDown);
      el.addEventListener("pointermove", this._onPointerMove);
      el.addEventListener("pointerup", this._onPointerUp);
      el.addEventListener("pointercancel", this._onPointerUp);
      el.addEventListener("pointerleave", this._onPointerUp);
      el.addEventListener("wheel", this._onWheel, { passive: false });
      el.addEventListener("dblclick", this._onDblClick);
    }

    _clampRadius(r) { return Math.min(CAM.radiusMax, Math.max(CAM.radiusMin, r)); }

    resetCamera() {
      this._camTarget.theta = CAM.theta;
      this._camTarget.phi = CAM.phi;
      this._camTarget.radius = CAM.radius;
    }

    _updateCamera() {
      const c = this._cam, t = this._camTarget;
      c.theta += (t.theta - c.theta) * 0.22;
      c.phi += (t.phi - c.phi) * 0.22;
      c.radius += (t.radius - c.radius) * 0.22;
      const sinPhi = Math.sin(c.phi);
      this.camera.position.set(
        CAM.target[0] + c.radius * sinPhi * Math.sin(c.theta),
        CAM.target[1] + c.radius * Math.cos(c.phi),
        CAM.target[2] + c.radius * sinPhi * Math.cos(c.theta)
      );
      this.camera.lookAt(CAM.target[0], CAM.target[1], CAM.target[2]);
    }

    _buildRobot(geometries) {
      // URDF 是 Z-up，three.js 是 Y-up
      this.root = new THREE.Group();
      this.root.rotation.x = -Math.PI / 2;
      this.scene.add(this.root);

      // 银白色阳极氧化铝：中性偏冷的中高明度 + 较高金属度，靠环境贴图出反射
      const bodyMat = new THREE.MeshStandardMaterial({
        color: 0xd2d7dc, metalness: 0.74, roughness: 0.33,
      });
      const darkMat = new THREE.MeshStandardMaterial({
        color: 0x424b57, metalness: 0.68, roughness: 0.38,
      });

      const linkGroups = {};
      const makeLink = (name) => {
        if (linkGroups[name]) return linkGroups[name];
        const g = new THREE.Group();
        const link = this.model.links[name];
        // logo_link.STL 是 unitree 字样的立体字母（14 个独立字形），
        // 整体跳过不渲染，改由 _addChestBrand 贴上自有品牌字。
        if (link && name !== "logo_link") {
          for (const v of link.visuals) {
            const geom = geometries[v.mesh];
            if (!geom) continue;
            const isDark = v.color && v.color[0] < 0.4;
            const mesh = new THREE.Mesh(geom, isDark ? darkMat : bodyMat);
            mesh.position.set(v.xyz[0], v.xyz[1], v.xyz[2]);
            mesh.quaternion.setFromEuler(new THREE.Euler(v.rpy[0], v.rpy[1], v.rpy[2], "ZYX"));
            g.add(mesh);
          }
        }
        linkGroups[name] = g;
        return g;
      };

      const rootLink = makeLink(this.model.rootLink);
      this.root.add(rootLink);

      // 按父子关系挂接。URDF 的 joint 顺序保证父在前，一次遍历即可。
      for (const j of this.model.joints) {
        const parentGroup = makeLink(j.parent);
        const childGroup = makeLink(j.child);

        const jointObj = new THREE.Group();
        jointObj.position.set(j.xyz[0], j.xyz[1], j.xyz[2]);
        // URDF rpy 是固定轴 R = Rz(yaw)·Ry(pitch)·Rx(roll)，对应 three.js 的 'ZYX'
        jointObj.quaternion.setFromEuler(new THREE.Euler(j.rpy[0], j.rpy[1], j.rpy[2], "ZYX"));

        const articulation = new THREE.Group();
        jointObj.add(articulation);
        articulation.add(childGroup);
        parentGroup.add(jointObj);

        if (j.type === "revolute" || j.type === "continuous") {
          this.jointObjects[j.name] = {
            articulation,
            axis: new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
            lower: j.lower,
            upper: j.upper,
          };
          this.jointTargets[j.name] = 0;
          this.jointCurrent[j.name] = 0;
        }
      }

      this._addChestBrand(linkGroups);
      this._addDepthCamera(linkGroups);

      // 让机器人立在网格上：把整体下移到脚底贴地
      const box = new THREE.Box3().setFromObject(this.root);
      this.root.position.y = -box.min.y;
    }

    // ── 胸前品牌字 ──

    _makeTextTexture(text) {
      const c = document.createElement("canvas");
      c.width = 1024;
      c.height = 256;
      const g = c.getContext("2d");
      g.clearRect(0, 0, c.width, c.height);

      g.font = '700 128px -apple-system, "SF Pro Display", "Helvetica Neue", Arial, sans-serif';
      g.textAlign = "center";
      g.textBaseline = "middle";
      // 轻微外发光，让白字在深色 band 上更立体
      g.shadowColor = "rgba(0, 0, 0, 0.55)";
      g.shadowBlur = 12;
      g.fillStyle = "#ffffff";
      g.fillText(text, c.width / 2, c.height / 2 + 4);

      const tex = new THREE.CanvasTexture(c);
      tex.anisotropy = this.renderer ? this.renderer.capabilities.getMaxAnisotropy() : 1;
      // three.js 新旧版本的色彩空间属性名不同，两种都兼容
      if ("colorSpace" in tex && THREE.SRGBColorSpace) tex.colorSpace = THREE.SRGBColorSpace;
      else if ("encoding" in tex && THREE.sRGBEncoding) tex.encoding = THREE.sRGBEncoding;
      return tex;
    }

    // 沿实测圆弧生成贴合胸口的曲面条带（带 UV，供文字贴图使用）
    _buildBrandGeometry(cfg) {
      const N = 48;
      const R = cfg.radius;
      const cx = cfg.centerX - R;                 // 圆心落在 URDF 的 x 轴上
      const halfTheta = cfg.width / (2 * R);
      const tanTilt = Math.tan((cfg.tiltDeg * Math.PI) / 180);

      const pos = [], uv = [], idx = [];
      for (let i = 0; i <= N; i++) {
        const t = i / N;
        const a = -halfTheta + t * 2 * halfTheta;
        const rad = R + cfg.lift;
        for (let r = 0; r < 2; r++) {
          const dz = (r - 0.5) * cfg.height;      // r=0 底边，r=1 顶边
          pos.push(
            cx + rad * Math.cos(a) - tanTilt * dz,  // 越靠上越往后
            rad * Math.sin(a),
            cfg.centerZ + dz
          );
          uv.push(t, r);
        }
      }
      for (let i = 0; i < N; i++) {
        const a0 = i * 2, a1 = i * 2 + 1, b0 = i * 2 + 2, b1 = i * 2 + 3;
        idx.push(a0, b0, b1, a0, b1, a1);
      }

      const geom = new THREE.BufferGeometry();
      geom.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
      geom.setAttribute("uv", new THREE.Float32BufferAttribute(uv, 2));
      geom.setIndex(idx);
      geom.computeVertexNormals();
      return geom;
    }

    _addChestBrand(linkGroups) {
      const host = linkGroups["logo_link"] || linkGroups["torso_link"];
      if (!host) return;
      const mesh = new THREE.Mesh(
        this._buildBrandGeometry(BRAND),
        new THREE.MeshBasicMaterial({
          map: this._makeTextTexture(BRAND.text),
          transparent: true,
          side: THREE.DoubleSide,
          depthWrite: false,
        })
      );
      mesh.renderOrder = 2;
      host.add(mesh);
      this.brandMesh = mesh;
    }

    // ── 头顶 RealSense D435 ──

    _buildDepthCamera() {
      // 真机 D435 是银色阳极氧化铝外壳 + 深色前脸玻璃
      const shell = new THREE.MeshStandardMaterial({ color: 0xbcc2c8, metalness: 0.85, roughness: 0.26 });
      const trim = new THREE.MeshStandardMaterial({ color: 0x3a424b, metalness: 0.55, roughness: 0.45 });
      const glass = new THREE.MeshStandardMaterial({ color: 0x0d141b, metalness: 0.95, roughness: 0.08 });
      const emitter = new THREE.MeshStandardMaterial({
        color: 0x241539, metalness: 0.85, roughness: 0.16, emissive: 0x180b30,
      });

      const cam = new THREE.Group();

      // 主体：90(宽 Y) × 25(高 Z) × 25(厚 X) mm
      cam.add(new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.090, 0.025), shell));

      // 正面深色面板
      const face = new THREE.Mesh(new THREE.BoxGeometry(0.0035, 0.086, 0.0215), trim);
      face.position.x = 0.0125;
      cam.add(face);

      // 四个光学窗口：左红外 / 红外投射器 / RGB / 右红外
      const lens = (y, r, mat) => {
        const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.004, 20), mat);
        m.rotation.z = Math.PI / 2;   // 圆柱轴转到 +X，正对前方
        m.position.set(0.0143, y, 0);
        cam.add(m);
      };
      lens(-0.028, 0.0052, glass);
      lens(-0.009, 0.0040, emitter);
      lens(0.011, 0.0045, glass);
      lens(0.028, 0.0052, glass);

      // 底部支架，坐在头顶弧面上
      const foot = new THREE.Mesh(new THREE.BoxGeometry(0.020, 0.030, 0.010), shell);
      foot.position.z = -0.0175;
      cam.add(foot);

      return cam;
    }

    _addDepthCamera(linkGroups) {
      const head = linkGroups["head_link"];
      if (!head) return;
      const cam = this._buildDepthCamera();
      cam.position.set(DEPTH_CAM.x, 0, DEPTH_CAM.z);
      cam.rotation.y = (DEPTH_CAM.pitchDeg * Math.PI) / 180;
      head.add(cam);
      this.depthCam = cam;
    }

    _currentMotion() {
      if (!this.playlist.length) return null;
      return this.motions[this.playlist[this.playIndex % this.playlist.length]];
    }

    // 名字与画面必须同步：相位一变就立刻回调，而不是让外面定时去猜
    _setPhase(phase) {
      if (this.phase === phase) return;
      this.phase = phase;
      this._emitMotionChange();
    }

    _emitMotionChange() {
      if (this.onMotionChange) {
        this.onMotionChange({
          index: this.playIndex,
          key: this.playlist[this.playIndex] || "",
          label: this.currentMotionLabel(),
          phase: this.phase,
        });
      }
    }

    _updateMotion(now) {
      const motion = this._currentMotion();
      if (!motion) return;

      if (!this.motionStart) this.motionStart = now;
      const elapsed = (now - this.motionStart) / 1000;

      if (elapsed > motion.playDuration + 1.2) {
        if (this.autoAdvance) {
          // 播完停 1.2s 再切下一个动作
          this.playIndex = (this.playIndex + 1) % this.playlist.length;
          this.motionStart = now;
          this._frameHint = 0;
          this._setPhase("play");
          this._emitMotionChange();
        } else {
          // 手动选定的动作：原地循环重播，名字始终对得上
          this.motionStart = now;
          this._frameHint = 0;
          this._setPhase("play");
        }
        return;
      }

      if (elapsed > motion.playDuration) {
        // 收尾阶段：回到零位
        this._setPhase("reset");
        for (const name of motion.jointNames) this.jointTargets[name] = 0;
        return;
      }
      this._setPhase("play");

      // 在 times 里查找当前帧并做线性插值（时间轴按裁剪后的起点平移）
      const clock = motion.startTime + elapsed;
      const times = motion.times;
      let i = this._frameHint || 0;
      if (i >= times.length || times[i] > clock) i = 0;
      while (i < times.length - 1 && times[i + 1] <= clock) i++;
      this._frameHint = i;

      const f0 = motion.frames[i];
      const f1 = motion.frames[Math.min(i + 1, motion.frames.length - 1)];
      const t0 = times[i], t1 = times[Math.min(i + 1, times.length - 1)];
      const alpha = t1 > t0 ? (clock - t0) / (t1 - t0) : 0;

      motion.jointNames.forEach((name, k) => {
        this.jointTargets[name] = f0[k] + (f1[k] - f0[k]) * alpha;
      });
    }

    _animate() {
      const tick = (now) => {
        if (this.disposed) return;
        this._raf = requestAnimationFrame(tick);

        this._updateMotion(now);

        // 平滑逼近目标角，避免动作切换时突变
        for (const name in this.jointObjects) {
          const target = this.jointTargets[name] || 0;
          const cur = this.jointCurrent[name] || 0;
          const next = cur + (target - cur) * 0.18;
          this.jointCurrent[name] = next;
          const jo = this.jointObjects[name];
          jo.articulation.quaternion.setFromAxisAngle(jo.axis, next);
        }

        // 视角完全由手势决定，不再自动环绕
        this._updateCamera();

        this.renderer.render(this.scene, this.camera);
      };
      this._raf = requestAnimationFrame(tick);
    }

    currentMotionLabel() {
      const m = this._currentMotion();
      if (!m) return "";
      return this.phase === "reset" ? `${m.label} · 复位` : m.label;
    }

    // 供外部渲染动作切换条
    getPlaylist() {
      return this.playlist.map((key) => ({
        key,
        label: this.motions[key].label,
        duration: this.motions[key].playDuration != null ? this.motions[key].playDuration : this.motions[key].duration,
      }));
    }

    playMotion(index) {
      if (!this.playlist.length) return;
      this.playIndex = ((index % this.playlist.length) + this.playlist.length) % this.playlist.length;
      this.autoAdvance = false;
      this.motionStart = 0;
      this._frameHint = 0;
      this.phase = "play";
      this._emitMotionChange();
    }

    dispose() {
      this.disposed = true;
      if (this._raf) cancelAnimationFrame(this._raf);
      if (this._onResize) window.removeEventListener("resize", this._onResize);
      if (this.renderer && this._onPointerDown) {
        const el = this.renderer.domElement;
        el.removeEventListener("pointerdown", this._onPointerDown);
        el.removeEventListener("pointermove", this._onPointerMove);
        el.removeEventListener("pointerup", this._onPointerUp);
        el.removeEventListener("pointercancel", this._onPointerUp);
        el.removeEventListener("pointerleave", this._onPointerUp);
        el.removeEventListener("wheel", this._onWheel);
        el.removeEventListener("dblclick", this._onDblClick);
      }
      if (this.renderer) {
        this.renderer.dispose();
        if (this.renderer.domElement.parentNode) {
          this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
        }
      }
    }
  }

  window.G1Robot3D = {
    create(container) {
      if (typeof THREE === "undefined") {
        container.innerHTML = '<div class="center-text">3D 引擎未加载</div>';
        return null;
      }
      const viewer = new RobotViewer(container);
      viewer.init().catch((err) => {
        console.error("3D 初始化失败", err);
        container.innerHTML = `<div class="center-text">3D 模型加载失败<br><small>${err.message}</small></div>`;
      });
      return viewer;
    },
  };
})();
