/**
 * SceneManager —— three.js 场景管理
 *
 * 关键约定（照 robviz 的做法）：
 *   - ROS 是 Z-up 右手系，three.js 默认 Y-up。场景根节点做一次 Z-up 适配旋转，
 *     之后所有 ROS 数据原样进场景。
 *   - 缓冲预分配 + 复用：更新只写 attribute.array + needsUpdate，绝不逐帧 new。
 *   - 按需渲染：脏标记驱动，空闲时 render 0/s，主机占用趋近零。
 */
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { ParsedCloud } from "../core/pointcloud.ts";

/**
 * ROS(ENU, Z-up) → three 世界坐标 的映射。
 * #root 绕 X 轴 -90° 后：ROS(x,y,z) → three(x, z, -y)
 * 相机、控制器、raycaster 都在 three 世界里，涉及 ROS 坐标时必须过这个函数。
 */
export function rosToThree(x: number, y: number, z: number): THREE.Vector3 {
  return new THREE.Vector3(x, z, -y);
}
/** 反向：three → ROS */
export function threeToRos(v: THREE.Vector3): { x: number; y: number; z: number } {
  return { x: v.x, y: -v.z, z: v.y };
}

/**
 * 按高度上色：对齐 rviz 的 AxisColor + Use rainbow（Z 轴彩虹，蓝→红）。
 * rviz 那个 display 的 Color Transformer 是 AxisColor、Axis=Z、use_rainbow=true，
 * 所以取值按 Z 归一化后落到 HSV 色环上：低=蓝青、高=红。
 */
function heightColor(t: number, out: THREE.Color): THREE.Color {
  const x = Math.min(1, Math.max(0, t));
  const hue = (1 - x) * 240; // 240°(蓝) → 0°(红)
  // 降饱和 + 降亮度：纯 HSL(·,1,0.5) 在深色背景上太刺眼
  return out.setHSL(hue / 360, 0.72, 0.36);
}

/** 低于此高度的点视为地面，不画 */
export const GROUND_CUT_M = 0.2;

export class SceneManager {
  readonly scene: THREE.Scene;
  readonly camera: THREE.PerspectiveCamera;
  #renderer: THREE.WebGLRenderer;
  #controls: OrbitControls;
  #container: HTMLElement;
  #root: THREE.Group; // Z-up 适配 + fixed frame 变换

  #points: THREE.Points | undefined;
  #positions: THREE.BufferAttribute | undefined;
  #colors: THREE.BufferAttribute | undefined;
  #capacity = 0;

  /** 多话题叠加：每个话题一个独立图层 */
  #layers = new Map<
    string,
    {
      points: THREE.Points;
      positions: THREE.BufferAttribute;
      colors: THREE.BufferAttribute;
      capacity: number;
    }
  >();

  /** 线图层（轨迹 / 规划路径） */
  #lines = new Map<
    string,
    { line: THREE.Line; color: string; width: number }
  >();

  /** 安全区棱柱（线框） */
  #geofenceLines: THREE.LineSegments | undefined;
  #geofenceFill: THREE.Mesh | undefined;

  /** 四旋翼模型（按 ROS 坐标系建模：XY 水平面、Z 向上、机头朝 +X） */
  #drone: THREE.Group | undefined;
  /** 旋翼（桨盘 + 桨叶的组合，整体绕垂直轴旋转） */
  #rotors: THREE.Object3D[] = [];
  #lights: THREE.PointLight[] = [];

  #dirty = true;
  #raf = 0;
  #stop = false;
  #resizeObs: ResizeObserver;

  /** ROS 系下的视线焦点（Z-up），供外部读取 */
  readonly gridHelper: THREE.GridHelper;

  constructor(container: HTMLElement) {
    this.#container = container;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14171c);

    this.camera = new THREE.PerspectiveCamera(60, 1, 0.05, 2000);
    // 重要：不要动 camera.up。ROS 的 Z-up 由 #root 统一旋转处理，
    // 两者同时用会得到一个翻转的视图（这里是踩过的坑）。
    // 人体视觉：从 -X 侧（ROS 的西侧）往 +X 看，X 轴朝屏幕纵深，Z 向上。
    // 右手系下 Y（北）自然落在屏幕左侧，这是正确的。
    this.camera.position.set(-9, 4.5, 1.2);

    this.#renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.#renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.#renderer.domElement);

    // ROS Z-up → three Y-up 的适配：绕 X 轴 -90°
    this.#root = new THREE.Group();
    this.#root.rotation.x = -Math.PI / 2;
    this.scene.add(this.#root);

    // 地面网格：three 的 GridHelper 默认就躺在 XZ 平面（Y=0），而 #root 旋转后
    // ROS 的地面点也落在 XZ 平面——两者天然一致。
    // 所以网格必须加在 scene 上；加进 #root 会被再转 90°，立成一堵墙（踩过的坑）。
    this.gridHelper = new THREE.GridHelper(20, 20, 0x39414d, 0x272d36);
    (this.gridHelper.material as THREE.Material).transparent = true;
    (this.gridHelper.material as THREE.Material).opacity = 0.55;
    this.scene.add(this.gridHelper);

    // ROS 原点的坐标轴：加进 #root 让它跟随 ROS 旋转变换，
    // 于是红轴 = ROS X(东)、绿轴 = ROS Y(北)、蓝轴 = ROS Z(上)
    const axes = new THREE.AxesHelper(1.2);
    this.#root.add(axes);

    // 四旋翼模型（同样挂在 #root 下，跟随 ROS 坐标变换）
    this.#buildDrone();

    // 光源：实体模型（机身/机臂）需要光照，点云与线条不受影响
    this.scene.add(new THREE.HemisphereLight(0xcfd8e3, 0x2a3038, 1.1));
    const dir = new THREE.DirectionalLight(0xffffff, 1.0);
    dir.position.set(-1, 1.4, 0.9);
    this.scene.add(dir);

    this.#controls = new OrbitControls(this.camera, this.#renderer.domElement);
    this.#controls.enableDamping = true;
    this.#controls.dampingFactor = 0.12;
    this.#controls.addEventListener("change", () => this.invalidate());

    this.#resizeObs = new ResizeObserver(() => this.resize());
    this.#resizeObs.observe(container);
    container.addEventListener("wheel", (e) => e.preventDefault(), { passive: false });

    this.resize();
    this.#loop();
  }

  invalidate(): void {
    this.#dirty = true;
  }

  resize(): void {
    const w = this.#container.clientWidth || 1;
    const h = this.#container.clientHeight || 1;
    this.#renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.invalidate();
  }

  /** 让视图对准点云大致中心 */
  frameCloud(bounds: { min: THREE.Vector3; max: THREE.Vector3 } | undefined): void {
    if (!bounds) return;
    // bounds 是 ROS 坐标；相机与控制器在 three 世界，必须转换
    const centerRos = new THREE.Vector3()
      .addVectors(bounds.min, bounds.max)
      .multiplyScalar(0.5);
    const size = new THREE.Vector3().subVectors(bounds.max, bounds.min).length();
    const c = rosToThree(centerRos.x, centerRos.y, centerRos.z);

    this.#controls.target.copy(c);
    // 拉得比点云范围宽裕些，否则点云会顶满屏幕看不出全貌
    const d = Math.max(6, size * 1.15);
    // 与 resetView 保持同一观察方向：从 -X 侧上方看过去
    this.camera.position.set(c.x - d * 0.85, c.y + d * 0.45, c.z + d * 0.12);
    this.camera.lookAt(c);
    this.#controls.update();
    this.invalidate();
  }

  /** 恢复到默认的等轴视角（俯视东北上方） */
  resetView(): void {
    this.camera.up.set(0, 1, 0); // 恢复 three 默认的 Y-up
    this.#controls.target.set(0, 0, 0);
    this.camera.position.set(-9, 4.5, 1.2);
    this.camera.lookAt(0, 0, 0);
    this.#controls.update();
    this.invalidate();
  }

  /**
   * 正俯视，方向与 2D 地图保持一致：ROS 的 X 朝屏幕上方、Y 朝屏幕左。
   *
   * 为什么必须改 camera.up：相机在正上方时，如果 up 还是 three 默认的 (0,1,0)，
   * 它就与视线方向平行，lookAt 退化成不确定的朝向，画面会“旋”到一个奇怪角度。
   * 把 up 设为 three 的 +X（对应 ROS 的 X/东），屏幕上方就是 +X；
   * 此时相机右向 = three +Z = ROS -Y，所以 Y（北）落在屏幕左侧。
   */
  topView(): void {
    const t = this.#controls.target.clone();
    this.camera.up.set(1, 0, 0);
    this.camera.position.set(t.x, t.y + 18, t.z);
    this.camera.lookAt(t);
    this.#controls.update();
    this.invalidate();
  }

  /**
   * 更新一个话题的点云图层。
   * 多话题可同时存在（各自的 Points 对象），容量不够时翻倍重建。
   */
  setCloudLayer(
    key: string,
    cloud: ParsedCloud,
  ): { min: THREE.Vector3; max: THREE.Vector3 } | undefined {
    let layer = this.#layers.get(key);

    if (cloud.count === 0) {
      layer?.points.geometry.setDrawRange(0, 0);
      this.invalidate();
      return undefined;
    }

    if (!layer || cloud.count > layer.capacity) {
      let cap = Math.max(1 << 12, layer?.capacity ?? 0);
      while (cap < cloud.count) cap *= 2;

      if (layer) {
        this.#root.remove(layer.points);
        layer.points.geometry.dispose();
        (layer.points.material as THREE.Material).dispose();
      }

      const geo = new THREE.BufferGeometry();
      const pos = new THREE.BufferAttribute(new Float32Array(cap * 3), 3);
      pos.setUsage(THREE.DynamicDrawUsage);
      const col = new THREE.BufferAttribute(new Float32Array(cap * 3), 3);
      col.setUsage(THREE.DynamicDrawUsage);
      geo.setAttribute("position", pos);
      geo.setAttribute("color", col);
      geo.setDrawRange(0, 0);
      geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1e6);

      const points = new THREE.Points(
        geo,
        new THREE.PointsMaterial({ size: 0.09, vertexColors: true, sizeAttenuation: true }),
      );
      points.frustumCulled = false;
      this.#root.add(points);

      layer = { points, positions: pos, colors: col, capacity: cap };
      this.#layers.set(key, layer);
    }

    const posArr = layer.positions.array as Float32Array;
    const colArr = layer.colors.array as Float32Array;

    // 高度范围（跳过地面点，否则配色区间被拉扁）
    let minZ = Infinity;
    let maxZ = -Infinity;
    for (let i = 0; i < cloud.count; i++) {
      const z = cloud.positions[i * 3 + 2];
      if (z < GROUND_CUT_M) continue;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
    }
    if (!Number.isFinite(minZ)) {
      layer.points.geometry.setDrawRange(0, 0);
      this.invalidate();
      return undefined;
    }
    const span = maxZ - minZ || 1;

    const min = new THREE.Vector3(Infinity, Infinity, Infinity);
    const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);
    const c = new THREE.Color();
    let valid = 0;

    for (let i = 0; i < cloud.count; i++) {
      const x = cloud.positions[i * 3];
      const y = cloud.positions[i * 3 + 1];
      const z = cloud.positions[i * 3 + 2];
      if (z < GROUND_CUT_M) continue; // 地面点不画

      posArr[valid * 3] = x;
      posArr[valid * 3 + 1] = y;
      posArr[valid * 3 + 2] = z;

      const t = cloud.scalar ? cloud.scalar[i] / 255 : (z - minZ) / span;
      heightColor(t, c);
      colArr[valid * 3] = c.r;
      colArr[valid * 3 + 1] = c.g;
      colArr[valid * 3 + 2] = c.b;

      if (x < min.x) min.x = x;
      if (y < min.y) min.y = y;
      if (z < min.z) min.z = z;
      if (x > max.x) max.x = x;
      if (y > max.y) max.y = y;
      if (z > max.z) max.z = z;
      valid++;
    }

    layer.positions.needsUpdate = true;
    layer.colors.needsUpdate = true;
    layer.points.geometry.setDrawRange(0, valid);
    this.invalidate();
    return { min, max };
  }

  /**
   * 构建一个简化的四旋翼模型。
   * 在 ROS 坐标系里建模：XY 为水平面、Z 向上、机头朝 +X（与 yaw=0 一致）。
   * 模型加到 #root 下，跟随同一套 ROS→three 变换。
   */
  #buildDrone(): void {
    const g = new THREE.Group();

    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x3fbf5f,
      metalness: 0.25,
      roughness: 0.55,
    });
    const darkMat = new THREE.MeshStandardMaterial({
      color: 0x2a3138,
      metalness: 0.4,
      roughness: 0.7,
    });

    // 机身：x 向前、y 向右、z 向上
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.13, 0.055), bodyMat);
    g.add(body);

    // 机头指示（朝 +X）
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.035, 0.07, 12), bodyMat);
    nose.rotation.z = -Math.PI / 2; // 圆锥默认朝 +Y，转到 +X
    nose.position.x = 0.135;
    g.add(nose);

    const armLen = 0.20;
    const quads: [number, number][] = [
      [1, 1],
      [1, -1],
      [-1, 1],
      [-1, -1],
    ];

    for (const [sx, sy] of quads) {
      const armGroup = new THREE.Group();

      // 机臂（沿 +X 伸出去，后面整体旋转到对角线方向）
      const arm = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, armLen, 8), darkMat);
      arm.rotation.z = Math.PI / 2; // 圆柱默认沿 Y，转成沿 X
      arm.position.x = armLen / 2;
      armGroup.add(arm);

      // 电机
      const motorH = 0.032;
      const motorZ = 0.012;
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.019, 0.019, motorH, 12), darkMat);
      motor.rotation.x = Math.PI / 2; // 转成沿 Z
      motor.position.set(armLen, motorZ, 0);
      armGroup.add(motor);

      // 桨盘 + 三叶桨：
      // 以前只有一个空心圆，且浮在电机上方 4mm，看着像“飘着的盘子”。
      // 现在桨盘中心坐实在电机顶面，再加三片 120° 均布的桨叶，一眼就是装在电机上的旋翼。
      const rotorZ = motorZ + motorH / 2 + 0.002; // 刚好压在电机顶
      const rotorGroup = new THREE.Group();
      rotorGroup.position.set(armLen, rotorZ, 0);

      const bladeMat = new THREE.MeshStandardMaterial({
        color: 0xbfe9ff,
        metalness: 0.2,
        roughness: 0.5,
        side: THREE.DoubleSide,
      });
      const bladeLen = 0.125;
      // 三叶桨：每片相隔 120°
      const BLADES = 3;
      for (let k = 0; k < BLADES; k++) {
        const blade = new THREE.Mesh(new THREE.BoxGeometry(bladeLen, 0.014, 0.0025), bladeMat);
        blade.position.x = bladeLen / 2 - 0.008;
        const holder = new THREE.Group();
        holder.rotation.z = (k * Math.PI * 2) / BLADES;
        holder.add(blade);
        rotorGroup.add(holder);
      }

      // 桨盘（半透明光圈，标出扫掠范围）
      const rotor = new THREE.Mesh(
        new THREE.CircleGeometry(0.062, 24),
        new THREE.MeshBasicMaterial({
          color: 0x8ad6ff,
          transparent: true,
          opacity: 0.18,
          side: THREE.DoubleSide,
          depthWrite: false,
        }),
      );
      rotorGroup.add(rotor);

      armGroup.add(rotorGroup);
      this.#rotors.push(rotorGroup); // 整组一起转（桨盘+桨叶）

      // 指示灯：前两个绿、后两个红（好看飞行方向）
      const isFront = sx > 0;
      const led = new THREE.PointLight(isFront ? 0x48ff88 : 0xff5544, 0.6, 0.5);
      led.position.set(armLen * 0.8, -0.03, 0);
      armGroup.add(led);
      this.#lights.push(led);

      armGroup.rotation.z = Math.atan2(sy, sx);
      g.add(armGroup);
    }

    this.#drone = g;
    this.#root.add(g);
  }

  /** 更新无人机位姿（ROS 系下的位置 + 四元数姿态） */
  setDronePose(
    x: number,
    y: number,
    z: number,
    qx: number,
    qy: number,
    qz: number,
    qw: number,
  ): void {
    if (!this.#drone) return;
    this.#drone.position.set(x, y, z);
    this.#drone.quaternion.set(qx, qy, qz, qw);
    this.invalidate();
  }

  /** 隐藏/显示无人机模型 */
  setDroneVisible(v: boolean): void {
    if (this.#drone) this.#drone.visible = v;
    this.invalidate();
  }

  /**
   * 安全区：2D 多边形 + 高度上下限，在 3D 里画成棱柱。
   * 底/顶多边形 + 每条竖边，用 LineSegments 一次画完。
   */
  setGeofence(
    poly: { x: number; y: number }[],
    zMin: number,
    zMax: number,
    enabled: boolean,
  ): void {
    const n = poly.length;
    const real = enabled && n >= 3 && zMax > zMin;

    if (!real) {
      if (this.#geofenceLines) {
        this.#root.remove(this.#geofenceLines);
        this.#geofenceLines.geometry.dispose();
        (this.#geofenceLines.material as THREE.Material).dispose();
        this.#geofenceLines = undefined;
      }
      if (this.#geofenceFill) {
        this.#root.remove(this.#geofenceFill);
        this.#geofenceFill.geometry.dispose();
        (this.#geofenceFill.material as THREE.Material).dispose();
        this.#geofenceFill = undefined;
      }
      this.invalidate();
      return;
    }

    const segs: number[] = [];
    for (let i = 0; i < n; i++) {
      const a = poly[i];
      const b = poly[(i + 1) % n];
      // 底边
      segs.push(a.x, a.y, zMin, b.x, b.y, zMin);
      // 顶边
      segs.push(a.x, a.y, zMax, b.x, b.y, zMax);
      // 竖边
      segs.push(a.x, a.y, zMin, a.x, a.y, zMax);
    }

    if (!this.#geofenceLines) {
      const geo = new THREE.BufferGeometry();
      const mat = new THREE.LineBasicMaterial({
        color: 0x5fe08a,
        transparent: true,
        opacity: 0.85,
      });
      this.#geofenceLines = new THREE.LineSegments(geo, mat);
      this.#geofenceLines.frustumCulled = false;
      this.#root.add(this.#geofenceLines);
    }
    const geo = this.#geofenceLines.geometry;
    geo.setAttribute("position", new THREE.Float32BufferAttribute(segs, 3));
    geo.computeBoundingSphere();

    // 安全区棱柱：底面 + 顶面 + 四个侧面都铺一层浅绿。
    // 以前只有底面，从斜上方或侧面看就是一个悬空的圈，空间范围感很弱。
    // 材质是 DoubleSide，所以三角形缠绕方向不用纠结。
    const tri: number[] = [];
    const pushTri = (
      ax: number, ay: number, az: number,
      bx: number, by: number, bz: number,
      cx: number, cy: number, cz: number,
    ) => {
      tri.push(ax, ay, az, bx, by, bz, cx, cy, cz);
    };

    // 顶面 + 底面（扇形三角化；多边形已按极角排过序，对凸多边形足够了）
    for (let i = 1; i + 1 < n; i++) {
      pushTri(
        poly[0].x, poly[0].y, zMin,
        poly[i].x, poly[i].y, zMin,
        poly[i + 1].x, poly[i + 1].y, zMin,
      );
      pushTri(
        poly[0].x, poly[0].y, zMax,
        poly[i + 1].x, poly[i + 1].y, zMax,
        poly[i].x, poly[i].y, zMax,
      );
    }

    // 侧面：每对相邻顶点之间立一堵墙（两个三角形拼成一块梯形/矩形）
    for (let i = 0; i < n; i++) {
      const a = poly[i];
      const b = poly[(i + 1) % n];
      pushTri(a.x, a.y, zMin, b.x, b.y, zMin, b.x, b.y, zMax);
      pushTri(a.x, a.y, zMin, b.x, b.y, zMax, a.x, a.y, zMax);
    }

    if (tri.length > 0) {
      if (!this.#geofenceFill) {
        const fgeo = new THREE.BufferGeometry();
        const fmat = new THREE.MeshBasicMaterial({
          color: 0x5fe08a,
          transparent: true,
          opacity: 0.22,   // 之前 0.12 几乎看不到，与 2D 的浅绿覆盖保持一致
          side: THREE.DoubleSide,
          depthWrite: false,
        });
        this.#geofenceFill = new THREE.Mesh(fgeo, fmat);
        this.#geofenceFill.frustumCulled = false;
        this.#root.add(this.#geofenceFill);
      }
      this.#geofenceFill.geometry.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(tri, 3),
      );
      this.#geofenceFill.geometry.computeBoundingSphere();
    }

    this.invalidate();
  }

  /**
   * 线图层（轨迹 / 规划路径）。
   * points 为 ROS 坐标下的点序列；color 为 CSS 颜色。
   */
  setLineLayer(key: string, points: number[], color: string, width = 1): void {
    const vals = points.length / 3;
    let line = this.#lines.get(key);
    if (!line) {
      const geo = new THREE.BufferGeometry();
      const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.95 });
      line = { line: new THREE.Line(geo, mat), color, width };
      line.line.frustumCulled = false;
      this.#lines.set(key, line);
      this.#root.add(line.line);
    }
    // 容量不够就重建缓冲
    const attr = line.line.geometry.getAttribute("position") as THREE.BufferAttribute | undefined;
    if (!attr || attr.count < vals) {
      line.line.geometry.dispose();
      const geo = new THREE.BufferGeometry();
      geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1e6);
      line.line.geometry = geo;
    }
    const geo = line.line.geometry;
    let pos = geo.getAttribute("position") as THREE.BufferAttribute | undefined;
    if (!pos || pos.count < vals) {
      let cap = Math.max(1024, vals * 2);
      pos = new THREE.BufferAttribute(new Float32Array(cap * 3), 3);
      pos.setUsage(THREE.DynamicDrawUsage);
      geo.setAttribute("position", pos);
    }
    const arr = pos.array as Float32Array;
    for (let i = 0; i < vals; i++) {
      arr[i * 3] = points[i * 3];
      arr[i * 3 + 1] = points[i * 3 + 1];
      arr[i * 3 + 2] = points[i * 3 + 2];
    }
    pos.needsUpdate = true;
    geo.setDrawRange(0, vals);
    geo.computeBoundingSphere();
    this.invalidate();
  }

  clearLineLayer(key: string): void {
    const line = this.#lines.get(key);
    if (!line) return;
    this.#root.remove(line.line);
    line.line.geometry.dispose();
    (line.line.material as THREE.Material).dispose();
    this.#lines.delete(key);
    this.invalidate();
  }

  /** 移除某个话题的图层 */
  removeCloudLayer(key: string): void {
    const layer = this.#layers.get(key);
    if (!layer) return;
    this.#root.remove(layer.points);
    layer.points.geometry.dispose();
    (layer.points.material as THREE.Material).dispose();
    this.#layers.delete(key);
    this.invalidate();
  }

  /** 清空所有图层（重连时用） */
  clearAllLayers(): void {
    for (const key of [...this.#layers.keys()]) this.removeCloudLayer(key);
  }

  /** 已加载的图层 key */
  layerKeys(): string[] {
    return [...this.#layers.keys()];
  }

  #loop = (): void => {
    if (this.#stop) return;
    this.#raf = requestAnimationFrame(this.#loop);
    const controlsChanged = this.#controls.update();

    // 桨盘转动（纯视觉，不参与任何控制逻辑）
    if (this.#drone?.visible) {
      const t = performance.now() * 0.02;
      for (let i = 0; i < this.#rotors.length; i++) {
        // 对角两个同向，相邻反向
        this.#rotors[i].rotation.z = t * (i % 2 === 0 ? 1 : -1);
      }
      this.#dirty = true;
    }

    if (!this.#dirty && !controlsChanged) return; // 空闲不渲染
    this.#dirty = false;
    this.#renderer.render(this.scene, this.camera);
  };

  dispose(): void {
    this.#stop = true;
    cancelAnimationFrame(this.#raf);
    this.#resizeObs.disconnect();
    this.#controls.dispose();
    this.#renderer.dispose();
    this.#renderer.domElement.remove();
  }
}
