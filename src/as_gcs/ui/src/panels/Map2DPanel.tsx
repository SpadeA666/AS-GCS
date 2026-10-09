/**
 * 2D 地图面板（ENU 俯视）
 *
 * 屏幕约定：与 ENU 一致——X 向东（屏幕右），Y 向北（屏幕上）。
 * 打点分两类，这是核心交互：
 *   - 规划器点：交给 EGO/SUPER
 *   - PX4 点  ：直接位置控制
 * 两类点在视觉上必须能一眼区分，防止误发。
 */
import { useCallback, useEffect, useRef, useState } from "react";
import type { FoxgloveConnection } from "../core/FoxgloveConnection.ts";
import type { Geofence } from "../core/types.ts";
import { parsePointCloud2, type PointCloud2Msg, type ParsedCloud } from "../core/pointcloud.ts";
import { sortPolygon, geofenceCheck } from "../core/geofence.ts";
import { NumberField } from "./NumberField";

export interface Waypoint {
  id: number;
  kind: "planner" | "px4";
  x: number;
  y: number;
  z: number;
  heightMode: "hold" | "absolute";
  /**
   * 机头朝向，单位【度】，ENU 约定：0 = +X（东），90 = +Y（北），逆时针为正。
   * 下发时转成弧度；SUPER/EGO 按 goal 的 orientation 规划机头。
   */
  yaw: number;
}

interface Props {
  conn: FoxgloveConnection | undefined;
  /** 无人机位姿话题，用于画当前位置 */
  poseTopic: string;
  /** 点云话题：把 XY 投影下来当 2D 底图 */
  /** 原生点云话题（与 3D 面板同一套语义） */
  rawTopic?: string;
  /** 膨胀点云话题（SUPER 的 inf_occ） */
  inflatedTopic?: string;
  showRawCloud?: boolean;
  showInflatedCloud?: boolean;
  /** 视图状态由父组件持有，切 2D/3D 不丢 */
  view: { x: number; y: number; scale: number };
  onViewChange: (v: { x: number; y: number; scale: number }) => void;
  /** 当前选中的规划器，决定打点发到哪个 goal 话题 */
  planner: "ego" | "super";
  /** 新点默认高度（heightMode=absolute 时用）；也是「保持高度」的兜底初值 */
  defaultHeight: number;
  /**
   * 「保持高度」打点时用的高度：记录【最近一次确定的高度设定】
   * （起飞 / 上升下降 / 上一次指定高度打点），由父层统一维护。
   *
   * 以前这里取 drone.z（打点瞬间的实际高度）——飞机在升降过程中或悬停
   * 波动时打点会取到中间值，和设定过的值对不上，也就是「保持点没真正保持」。
   */
  holdHeight?: number;
  /** 「指定高度」打点确认后把该高度上报，作为后续「保持」的新基准 */
  onHeightCommit?: (h: number) => void;
  /** 飞行轨迹（扁平 [x,y,z,...]） */
  trajectory?: number[];
  /** 规划器路径（扁平） */
  planPath?: number[];
  /** SUPER 期望轨迹（rviz 的 ExpTraj），扁平 xyz */
  expTraj?: number[];
  expTrajColor?: string;
  /** 诊断用：ExpTraj 实际收到的消息数与最大段数 */
  expTrajStat?: { msgs: number; segs: number };
  /** 安全区（2D 编辑 + 绘制） */
  geofence?: Geofence;
  onGeofenceChange?: (g: Geofence) => void;
  /** 点「应用到网关」时触发，由父层调服务 */
  onApplyGeofence?: (g: Geofence) => void;
  /**
   * 安全区编辑模式。由左侧面板的「编辑」按钮控制，本面板只负责画布上的
   * 加点/拖点/删点交互，不再自持状态——否则左侧按钮点了画布不响应。
   */
  fenceMode?: boolean;
  onWaypointsChange?: (wps: Waypoint[]) => void;
  /** 打点结果反馈（成功 / 被安全区拒绝 / 调用失败）。不传则只在控制台告警。 */
  onLog?: (msg: string) => void;
  /** 清掉图上累积的轨迹与路径（仅清显示） */
  onClearTrails?: () => void;
}

/**
 * 高度 → 颜色：对齐 rviz 的 AxisColor + rainbow（Z 彩虹，蓝→红）。
 * 32 档预计算，避免每点都拼一次 hsl 字符串。
 */
const RAINBOW = (() => {
  const out: string[] = [];
  for (let i = 0; i < 32; i++) {
    const hue = (1 - i / 31) * 240; // 240°蓝 → 0°红
    // 降饱和 + 降亮度，与 3D 侧保持一致，深色底上不刺眼
    out.push(`hsl(${hue.toFixed(0)}, 72%, 36%)`);
  }
  return out;
})();

/** 低于此高度的点视为地面，不画 */
const GROUND_CUT_M = 0.2;

function rainbowOf(t: number): string {
  const i = Math.min(31, Math.max(0, Math.round(t * 31)));
  return RAINBOW[i];
}

const POSE_SCHEMAS = new Set(["nav_msgs/Odometry", "geometry_msgs/PoseStamped"]);

export const POSE_TOPIC_CANDIDATES = (topics: { topic: string; schemaName: string }[]): string[] =>
  topics.filter((t) => POSE_SCHEMAS.has(t.schemaName)).map((t) => t.topic);

export function Map2DPanel({
  conn,
  poseTopic,
  rawTopic,
  inflatedTopic,
  showRawCloud = true,
  showInflatedCloud = false,
  view,
  onViewChange,
  planner,
  defaultHeight,
  holdHeight = defaultHeight,
  onHeightCommit,
  trajectory,
  planPath,
  expTraj,
  expTrajColor = "#ff50c8",
  expTrajStat,
  geofence,
  onGeofenceChange,
  onApplyGeofence,
  fenceMode = false,
  onWaypointsChange,
  onLog,
  onClearTrails,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hostRef = useRef<HTMLDivElement>(null);

  const scale = view.scale;
  const setScale = (v: number | ((s: number) => number)) =>
    onViewChange({ ...view, scale: typeof v === "function" ? v(view.scale) : v });
  const setView = (v: { x: number; y: number }) => onViewChange({ ...view, ...v });

  const [drone, setDrone] = useState<
    { x: number; y: number; z: number; yaw: number } | undefined
  >();
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [pendingKind, setPendingKind] = useState<"planner" | "px4">("planner");
  const [heightMode, setHeightMode] = useState<"hold" | "absolute">("hold");
  const [heightInput, setHeightInput] = useState(defaultHeight);
  /** 目标机头朝向（度）。打点时写进待确认点，并用箭头画在图上。 */
  const [yawDeg, setYawDeg] = useState(0);
  // 安全区编辑模式由左侧面板控制（见 Props.fenceMode），本面板不再自持。
  const draggingVertexRef = useRef<number>(-1);

  // 点云投影：存到 ref 里避免每个点都进 React 状态；用计数触发重绘。
  // 原生和膨胀各存一份 —— 两者可以同时勾选，以前只传一个话题所以只能显示一个。
  const rawRef = useRef<ParsedCloud | undefined>(undefined);
  const inflatedRef = useRef<ParsedCloud | undefined>(undefined);
  const [cloudTick, setCloudTick] = useState(0);
  const [cloudPoints, setCloudPoints] = useState(0);
  const [showCloud, setShowCloud] = useState(true);

  // 订阅点云做 XY 投影（原生）。最多 10Hz 触发重绘，免得 30Hz 的点云拖满主线程
  useEffect(() => {
    if (!conn || !rawTopic) return;
    let lastTick = 0;
    return conn.subscribe(rawTopic, (msg) => {
      try {
        rawRef.current = parsePointCloud2(msg as PointCloud2Msg);
      } catch {
        return;
      }
      const now = performance.now();
      if (now - lastTick > 100) {
        lastTick = now;
        setCloudTick((t) => t + 1);
        setCloudPoints((rawRef.current?.count ?? 0) + (inflatedRef.current?.count ?? 0));
      }
    });
  }, [conn, rawTopic]);

  // 订阅点云（膨胀）—— 与原生独立，两者可同时在 2D 上叠加显示
  useEffect(() => {
    if (!conn || !inflatedTopic) return;
    let lastTick = 0;
    return conn.subscribe(inflatedTopic, (msg) => {
      try {
        inflatedRef.current = parsePointCloud2(msg as PointCloud2Msg);
      } catch {
        return;
      }
      const now = performance.now();
      if (now - lastTick > 100) {
        lastTick = now;
        setCloudTick((t) => t + 1);
        setCloudPoints((rawRef.current?.count ?? 0) + (inflatedRef.current?.count ?? 0));
      }
    });
  }, [conn, inflatedTopic]);

  const dragRef = useRef<{ x: number; y: number; vx: number; vy: number } | undefined>(undefined);
  /** 按下时的屏幕坐标，用来区分「点击打点」和「拖动平移」 */
  const downPosRef = useRef<{ x: number; y: number } | undefined>(undefined);
  const nextIdRef = useRef(1);

  /** 待确认的点：不直接落盘，确认后才加入并发送 */
  const [pending, setPending] = useState<
    | {
        kind: "planner" | "px4";
        x: number;
        y: number;
        z: number;
        heightMode: "hold" | "absolute";
        yaw: number;
      }
    | undefined
  >(undefined);

  // 订阅位姿
  useEffect(() => {
    if (!conn || !poseTopic) return;
    return conn.subscribe(poseTopic, (msg) => {
      const m = msg as Record<string, unknown>;
      // 兼容 PoseStamped 与 Odometry 两种结构
      const pose = (m.pose as Record<string, unknown> | undefined) ?? m;
      const p = (pose.pose as Record<string, unknown> | undefined)?.position ?? pose.position;
      if (!p) return;
      const pos = p as { x: number; y: number; z: number };
      const qq = ((pose.pose as Record<string, unknown> | undefined)?.orientation ??
        pose.orientation) as { x: number; y: number; z: number; w: number } | undefined;
      let yaw = 0;
      if (qq) {
        // ROS 的 yaw：绕 Z 轴，注意这里用的是 ENU 的 Z-up 约定
        const siny = 2 * (qq.w * qq.z + qq.x * qq.y);
        const cosy = 1 - 2 * (qq.y * qq.y + qq.z * qq.z);
        yaw = Math.atan2(siny, cosy);
      }
      setDrone({ x: pos.x, y: pos.y, z: pos.z, yaw });
    });
  }, [conn, poseTopic]);

  // 绘制
  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    const host = hostRef.current;
    if (!canvas || !host) return;
    const dpr = Math.min(window.devicePixelRatio, 2);
    const w = host.clientWidth;
    const h = host.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
      canvas.style.width = `${w}px`;
      canvas.style.height = `${h}px`;
    }
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // 屏幕映射：X 朝上、Y 朝左（贴合手动操作习惯）
    //   sx = cx - (y - vy) * scale
    //   sy = cy - (x - vx) * scale
    const toScreen = (x: number, y: number): [number, number] => [
      w / 2 - (y - view.y) * scale,
      h / 2 - (x - view.x) * scale,
    ];

    ctx.fillStyle = "#14171c";
    ctx.fillRect(0, 0, w, h);

    // ── 点云 XY 投影当作 2D 底图 ──
    // 原生与膨胀是两路独立数据，各自订阅、各自画，可以同时叠加。
    // 以前只传一个话题（三元选择），所以两个都勾也只有原生出现。
    if (showCloud) {
      // 原生点云：按高度着彩虹色
      const raw = rawRef.current;
      if (showRawCloud && raw && raw.count > 0) {
        const pts = raw.positions;
        let zMin = Infinity;
        let zMax = -Infinity;
        for (let i = 0; i < raw.count; i++) {
          const z = pts[i * 3 + 2];
          if (z < GROUND_CUT_M) continue; // 地面点不参与配色范围
          if (z < zMin) zMin = z;
          if (z > zMax) zMax = z;
        }
        if (Number.isFinite(zMin)) {
          const span = zMax - zMin || 1;
          const stride = raw.count > 60000 ? 4 : raw.count > 25000 ? 2 : 1;
          const size = scale > 80 ? 3 : 2;
          for (let i = 0; i < raw.count; i += stride) {
            const wz = pts[i * 3 + 2];
            if (wz < GROUND_CUT_M) continue;
            const [sx, sy] = toScreen(pts[i * 3], pts[i * 3 + 1]);
            if (sx < -4 || sx > w + 4 || sy < -4 || sy > h + 4) continue;
            ctx.fillStyle = rainbowOf((wz - zMin) / span);
            ctx.fillRect(sx, sy, size, size);
          }
        }
      }

      // 膨胀点云：单色半透（青），一眼区分于原生的彩虹色。
      // 它是“障碍物膨胀后的禁飞区”，用整块色比彩虹更贴切。
      const inf = inflatedRef.current;
      if (showInflatedCloud && inf && inf.count > 0) {
        const pts = inf.positions;
        const stride = inf.count > 60000 ? 4 : inf.count > 25000 ? 2 : 1;
        const size = scale > 80 ? 3 : 2;
        ctx.fillStyle = "rgba(90, 220, 235, 0.55)";
        for (let i = 0; i < inf.count; i += stride) {
          const wz = pts[i * 3 + 2];
          if (wz < GROUND_CUT_M) continue; // 地面不画（与原生一致）
          const [sx, sy] = toScreen(pts[i * 3], pts[i * 3 + 1]);
          if (sx < -4 || sx > w + 4 || sy < -4 || sy > h + 4) continue;
          ctx.fillRect(sx, sy, size, size);
        }
      }
    }

    // 网格：0.5m 细格 + 5m 粗格。缩放太小时只画粗格，避免糊成一片。
    const FINE = 0.5;
    const COARSE = 5;
    const step = scale < 26 ? COARSE : FINE;
    const isMajor = (v: number) => Math.abs(v / COARSE - Math.round(v / COARSE)) < 1e-6;

    const x0 = Math.floor(view.x - w / 2 / scale) - 1;
    const x1 = Math.ceil(view.x + w / 2 / scale) + 1;
    const y0 = Math.floor(view.y - h / 2 / scale) - 1;
    const y1 = Math.ceil(view.y + h / 2 / scale) + 1;

    // 用整数索引循环，避免浮点累加误差
    const ix0 = Math.floor(x0 / step);
    const ix1 = Math.ceil(x1 / step);
    const iy0 = Math.floor(y0 / step);
    const iy1 = Math.ceil(y1 / step);

    // 新映射下：x = 常数的线是【水平线】，y = 常数的线是【垂直线】
    for (let i = ix0; i <= ix1; i++) {
      const gx = i * step;
      const major = isMajor(gx);
      const [, sy] = toScreen(gx, 0);
      ctx.strokeStyle = gx === 0 ? "#4a5563" : major ? "#2f3742" : "#20252c";
      ctx.lineWidth = gx === 0 ? 1.5 : major ? 1 : 0.6;
      ctx.beginPath();
      ctx.moveTo(0, Math.round(sy) + 0.5);
      ctx.lineTo(w, Math.round(sy) + 0.5);
      ctx.stroke();
      if (major) {
        ctx.fillStyle = "#5c6673";
        ctx.font = "10px ui-monospace, monospace";
        // X 的刻度标在左侧
        ctx.fillText(`${gx}`, 4, Math.round(sy) - 3);
      }
    }
    for (let i = iy0; i <= iy1; i++) {
      const gy = i * step;
      const major = isMajor(gy);
      const [sx] = toScreen(0, gy);
      ctx.strokeStyle = gy === 0 ? "#4a5563" : major ? "#2f3742" : "#20252c";
      ctx.lineWidth = gy === 0 ? 1.5 : major ? 1 : 0.6;
      ctx.beginPath();
      ctx.moveTo(Math.round(sx) + 0.5, 0);
      ctx.lineTo(Math.round(sx) + 0.5, h);
      ctx.stroke();
      if (major) {
        ctx.fillStyle = "#5c6673";
        ctx.font = "10px ui-monospace, monospace";
        // Y 的刻度标在底部
        ctx.fillText(`${gy}`, Math.round(sx) + 3, h - 4);
      }
    }

    // 原点标记
    const [ox, oy] = toScreen(0, 0);
    ctx.strokeStyle = "#6b7684";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(ox, oy, 7, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillStyle = "#6b7684";
    ctx.font = "10px ui-monospace, monospace";
    ctx.fillText("O", ox + 9, oy - 6);

    // 航点：尺寸放大，带白边
    for (const wp of waypoints) {
      const [sx, sy] = toScreen(wp.x, wp.y);
      const isPlanner = wp.kind === "planner";
      // 已确认的点也标出越界；wp.z 在发送那一刻就已固化（保持模式 = 当时的 holdHeight）
      const wpBad = !geofenceCheck(geofence, wp.x, wp.y, wp.z).ok;
      const wpCol = wpBad ? "#ff5f5f" : isPlanner ? "#4ea1ff" : "#e0a030";
      ctx.strokeStyle = "#0f1216";
      ctx.lineWidth = 2.5;
      ctx.fillStyle = wpCol;
      if (isPlanner) {
        // 规划器点：圆形
        ctx.beginPath();
        ctx.arc(sx, sy, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      } else {
        // PX4 点：方形（形状不同，一眼可辨）
        ctx.beginPath();
        ctx.rect(sx - 7, sy - 7, 14, 14);
        ctx.fill();
        ctx.stroke();
      }

      // 机头朝向箭头：从点位朝目标 yaw 画一根带箭头的线。
      // 屏幕约定是 X ↑ / Y ←（见工具栏），所以屏幕角 = -π/2 - yaw_rad，
      // 与无人机机身的 ctx.rotate(-yaw - π/2) 保持一致。
      {
        const a = -Math.PI / 2 - (wp.yaw * Math.PI) / 180;
        const L = 22;
        const ex = sx + Math.cos(a) * L;
        const ey = sy + Math.sin(a) * L;
        const head = 6;
        ctx.strokeStyle = wpCol;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(ex, ey);
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - Math.cos(a - 0.4) * head, ey - Math.sin(a - 0.4) * head);
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - Math.cos(a + 0.4) * head, ey - Math.sin(a + 0.4) * head);
        ctx.stroke();
      }

      ctx.fillStyle = "#c8d0da";
      ctx.font = "11px ui-monospace, monospace";
      // 保持点也把实际高度写出来，一眼能看出「保持」到了多少
      const hTxt =
        wp.heightMode === "hold" ? `保持 ${wp.z.toFixed(1)}m` : `${wp.z.toFixed(1)}m`;
      ctx.fillText(`${wp.id}  ${hTxt}`, sx + 12, sy - 9);
    }

    // 安全区：棱柱投影（多边形 + 半透填充 + 顶点）
    // 顶点按极角排序后再画，避免点击顺序不对导致自交/折叠
    const polyRaw = geofence?.polygon ?? [];
    const poly = sortPolygon(polyRaw);
    if (poly.length > 0) {
      const on = geofence?.enabled;
      // 已启用 = 亮绿（与“禁飞/生效”语义一致，深色底上也最跳眼）；未启用 = 灰虚线
      const col = on ? "#5fe08a" : "#8a94a2";
      ctx.strokeStyle = col;
      ctx.lineWidth = on ? 2.5 : 1.6;
      ctx.setLineDash(on ? [] : [6, 4]);
      ctx.beginPath();
      for (let i = 0; i < poly.length; i++) {
        const [sx, sy] = toScreen(poly[i].x, poly[i].y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      }
      if (poly.length >= 3) ctx.closePath();
      if (poly.length >= 3 && on) {
        // 浅绿覆盖：之前是橙色 10%，太淡基本看不出来。提到 22% 既醒目又不盖住点云
        ctx.fillStyle = "rgba(110,230,140,0.22)";
        ctx.fill();
      }
      ctx.stroke();
      ctx.setLineDash([]);

      // 顶点（编辑模式下加大，好抓）
      for (let i = 0; i < poly.length; i++) {
        const [sx, sy] = toScreen(poly[i].x, poly[i].y);
        ctx.fillStyle = col;
        ctx.beginPath();
        ctx.arc(sx, sy, fenceMode ? 6 : 4, 0, Math.PI * 2);
        ctx.fill();
      }
      // 高度标注
      if (on) {
        const [lx, ly] = toScreen(poly[0].x, poly[0].y);
        ctx.fillStyle = col;
        ctx.font = "11px ui-monospace, monospace";
        ctx.fillText(
          `Z ${geofence!.zMin.toFixed(1)}~${geofence!.zMax.toFixed(1)}m`,
          lx + 10,
          ly - 10,
        );
      }
    }

    // 规划器路径（橙）与飞行轨迹（绿）
    const strokePolyline = (pts: number[] | undefined, color: string, width: number, dash: number[]) => {
      if (!pts || pts.length < 6) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.setLineDash(dash);
      ctx.lineJoin = "round";
      ctx.beginPath();
      for (let i = 0; i < pts.length; i += 3) {
        const [sx, sy] = toScreen(pts[i], pts[i + 1]);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    };

    strokePolyline(planPath, "#e0a030", 2, []);
    strokePolyline(expTraj, expTrajColor, 2, []);
    strokePolyline(trajectory, "#48c46a", 2, []);

    // 无人机：yaw=0 时 ROS 的机头朝 +X，而 +X 在屏幕上是【上】，
    // 所以基准旋转要从 -90° 起算：θ = -yaw - π/2
    if (drone) {
      const [sx, sy] = toScreen(drone.x, drone.y);
      ctx.save();
      ctx.translate(sx, sy);
      ctx.rotate(-drone.yaw - Math.PI / 2);
      // 机身：加白边提高对比，尺寸按缩放自适应（放大一点更好点）
      const L = 18;
      const W = 11;
      ctx.strokeStyle = "#0f1216";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(L, 0);
      ctx.lineTo(-W * 0.62, W);
      ctx.lineTo(-W * 0.28, 0);
      ctx.lineTo(-W * 0.62, -W);
      ctx.closePath();
      ctx.stroke();
      ctx.fillStyle = "#3fbf5f";
      ctx.fill();
      // 机头方向上的短引线，一眼能看出朝向
      ctx.strokeStyle = "#3fbf5f";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(L, 0);
      ctx.lineTo(L + 9, 0);
      ctx.stroke();
      ctx.restore();

      // 坐标数值跟随无人机显示
      ctx.fillStyle = "#3fbf5f";
      ctx.font = "11px ui-monospace, monospace";
      // 三维坐标：X/Y 是平面位置，Z 是高度。俯视图看不出高度，必须标出来。
      ctx.fillText(
        `X ${drone.x.toFixed(2)}  Y ${drone.y.toFixed(2)}  Z ${(drone.z ?? 0).toFixed(2)}`,
        sx + 16,
        sy + 22,
      );
    }

    // 待确认的点：虚线圈 + 闪烁点，视觉上明确「还没发出去」
    if (pending) {
      const [sx, sy] = toScreen(pending.x, pending.y);
      // 实时预判安全区：越界就整个变红（包括朝向箭头），
      // 不用等点下去被网关拒绝才知道。pending.z 在构造时已定，没有占位 0 的坑了。
      const bad = !geofenceCheck(geofence, pending.x, pending.y, pending.z).ok;
      const c = bad ? "#ff5f5f" : pending.kind === "planner" ? "#4ea1ff" : "#e0a030";
      ctx.strokeStyle = c;
      ctx.setLineDash([4, 3]);
      ctx.lineWidth = 2;
      ctx.beginPath();
      if (pending.kind === "planner") ctx.arc(sx, sy, 11, 0, Math.PI * 2);
      else ctx.rect(sx - 10, sy - 10, 20, 20);
      ctx.stroke();
      ctx.setLineDash([]);
      if (performance.now() % 900 < 450) {
        ctx.fillStyle = c;
        ctx.beginPath();
        ctx.arc(sx, sy, 3, 0, Math.PI * 2);
        ctx.fill();
      }
      // 机头朝向箭头（预览）——打点后、发送前就能看到会朝哪边
      {
        const a = -Math.PI / 2 - (pending.yaw * Math.PI) / 180;
        const L = 24;
        const ex = sx + Math.cos(a) * L;
        const ey = sy + Math.sin(a) * L;
        const head = 6;
        ctx.strokeStyle = c;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(ex, ey);
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - Math.cos(a - 0.4) * head, ey - Math.sin(a - 0.4) * head);
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - Math.cos(a + 0.4) * head, ey - Math.sin(a + 0.4) * head);
        ctx.stroke();
        ctx.fillStyle = c;
        ctx.font = "11px ui-monospace, monospace";
        ctx.fillText(`${pending.yaw}°`, ex + 4, ey - 4);
      }
      ctx.strokeStyle = "rgba(125,135,148,0.45)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(sx - 16, sy);
      ctx.lineTo(sx + 16, sy);
      ctx.moveTo(sx, sy - 16);
      ctx.lineTo(sx, sy + 16);
      ctx.stroke();
    }

    ctx.fillStyle = "#7d8794";
    ctx.font = "11px ui-monospace, monospace";
    ctx.fillText(`1 小格 = ${FINE} m，粗线 = ${COARSE} m`, 10, 16);
    ctx.fillStyle = "#5c6673";
    ctx.fillText("X ↑    Y ←", 10, 32);
  }, [scale, view, drone, waypoints, pending, cloudTick, showCloud, showRawCloud, showInflatedCloud, trajectory, planPath, expTraj, expTrajColor, geofence, fenceMode, yawDeg]);

  useEffect(() => {
    draw();
  }, [draw]);

  const [hostSize, setHostSize] = useState({ w: 0, h: 0 });

  useEffect(() => {
    const host = hostRef.current;
    if (!host) return;
    const ro = new ResizeObserver(() => {
      setHostSize({ w: host.clientWidth, h: host.clientHeight });
      draw();
    });
    ro.observe(host);
    setHostSize({ w: host.clientWidth, h: host.clientHeight });
    return () => ro.disconnect();
  }, [draw]);

  const screenToWorld = (clientX: number, clientY: number) => {
    const canvas = canvasRef.current;
    if (!canvas) return { x: 0, y: 0 };
    const r = canvas.getBoundingClientRect();
    const sx = clientX - r.left - r.width / 2;
    const sy = clientY - r.top - r.height / 2;
    // 反解 toScreen： sx = -(y-vy)*s，sy = -(x-vx)*s
    return {
      x: view.x - sy / scale,
      y: view.y - sx / scale,
    };
  };

  /** 真正发送。只有用户在确认卡片上点「确认」才会走到这里。 */
  const sendWaypoint = (wp: Omit<Waypoint, "id">) => {
    const full: Waypoint = { ...wp, id: nextIdRef.current++ };
    const next = [...waypoints, full];
    setWaypoints(next);
    onWaypointsChange?.(next);

    if (!conn) {
      // 以前这里是静默 return —— 用户点了「确认发送」却什么都不发生，也无从判断原因。
      onLog?.("打点失败：未连接（先点顶栏的「连接」）");
      return;
    }

    if (full.kind === "planner") {
      // 交给网关 → ASNAV::navigationSuper / navigationEgo → 规划器。
      // 以前是直接 publish /move_base_simple/goal：轨迹算出来了但没人执行，
      // 而且绕过了网关，安全区也管不到。
      conn.callService("/gcs/go_to_planner", {
        position: { x: full.x, y: full.y, z: full.z },
        yaw: (full.yaw * Math.PI) / 180,   // 度 → 弧度
        tol: 0.3,
      })
        .then((r) => {
          const res = r as { success?: boolean; message?: string };
          onLog?.(
            `规划器点 ${planner.toUpperCase()} ${res.success ? "已受理" : "被拒绝"} ` +
              `(${full.x.toFixed(2)}, ${full.y.toFixed(2)}, ${full.z.toFixed(2)}) ` +
              `yaw ${full.yaw.toFixed(0)}° ${res.message ?? ""}`,
          );
        })
        .catch((e) => onLog?.(`规划器点调用失败: ${String(e)}（网关起了吗？）`));
      return;
    }

    // PX4 点：直接给飞控 local 系下发位置控制（经网关 /gcs/go_to_px4）。
    // 网关侧会先做安全区校验；未解锁时会被忽略并在 message 里说明。
    conn.callService("/gcs/go_to_px4", {
      position: { x: full.x, y: full.y, z: full.z },
      yaw: (full.yaw * Math.PI) / 180,   // 度 → 弧度
      tol: 0.2,
    })
      .then((r) => {
        const res = r as { success?: boolean; message?: string };
        onLog?.(
          `PX4 点 ${res.success ? "已受理" : "被拒绝"} ` +
            `(${full.x.toFixed(2)}, ${full.y.toFixed(2)}, ${full.z.toFixed(2)}) ` +
            `yaw ${full.yaw.toFixed(0)}° ${res.message ?? ""}`,
        );
      })
      .catch((e) => onLog?.(`PX4 点调用失败: ${String(e)}（网关起了吗？）`));
  };

  const confirmPending = () => {
    if (!pending) return;
    // pending.z 在构造时已定好：绝对模式 = 输入值，保持模式 = holdHeight。
    // 「保持」不再读打点瞬间的实际高度——升降途中/悬停波动时取到的中间值
    // 和设定值对不上，那正是「保持点没真正保持」的原因。
    sendWaypoint(pending);
    // 「指定高度」打点后，该高度成为后续「保持」的新基准
    if (pending.heightMode === "absolute") onHeightCommit?.(pending.z);
    setPending(undefined);
  };

  /** 找屏幕坐标附近的顶点（用于拖拽/删除） */
  const vertexAt = (clientX: number, clientY: number): number => {
    const poly = geofence?.polygon ?? [];
    const canvas = canvasRef.current;
    if (!canvas) return -1;
    const r = canvas.getBoundingClientRect();
    for (let i = 0; i < poly.length; i++) {
      const [sx, sy] = [
        w0(r) / 2 - (poly[i].y - view.y) * scale,
        h0(r) / 2 - (poly[i].x - view.x) * scale,
      ];
      const dx = clientX - r.left - sx;
      const dy = clientY - r.top - sy;
      if (dx * dx + dy * dy < 144) return i; // 12px 内
    }
    return -1;
  };
  const w0 = (r: DOMRect) => r.width;
  const h0 = (r: DOMRect) => r.height;

  const onDownFence = (e: React.MouseEvent): boolean => {
    if (!fenceMode || !geofence || !onGeofenceChange) return false;
    const idx = vertexAt(e.clientX, e.clientY);

    // 右键：删顶点
    if (e.button === 2) {
      if (idx >= 0) {
        const poly = geofence.polygon.filter((_, i) => i !== idx);
        onGeofenceChange({ ...geofence, polygon: poly });
      }
      return true;
    }
    if (e.button !== 0) return false;

    // 左键：命中顶点则开始拖，否则加点
    if (idx >= 0) {
      draggingVertexRef.current = idx;
    } else {
      const w = screenToWorld(e.clientX, e.clientY);
      onGeofenceChange({ ...geofence, polygon: [...geofence.polygon, { x: w.x, y: w.y }] });
    }
    return true;
  };

  const onMoveFence = (e: React.MouseEvent): boolean => {
    if (!fenceMode || !geofence || !onGeofenceChange) return false;
    const idx = draggingVertexRef.current;
    if (idx < 0) return false;
    const w = screenToWorld(e.clientX, e.clientY);
    const poly = geofence.polygon.map((p, i) => (i === idx ? { x: w.x, y: w.y } : p));
    onGeofenceChange({ ...geofence, polygon: poly });
    return true;
  };

  const onWheel = (e: React.WheelEvent) => {
    e.preventDefault();
    setScale((s) => Math.min(300, Math.max(8, s * (e.deltaY < 0 ? 1.12 : 0.89))));
  };

  const onMouseDown = (e: React.MouseEvent) => {
    if (onDownFence(e)) return;
    downPosRef.current = { x: e.clientX, y: e.clientY };
    if (e.button === 1 || e.button === 2) {
      dragRef.current = { x: e.clientX, y: e.clientY, vx: view.x, vy: view.y };
    }
  };

  const onMouseMove = (e: React.MouseEvent) => {
    if (onMoveFence(e)) return;
    const d = dragRef.current;
    if (!d) return;
    // 新映射下，横向拖动改变的是 view.y，纵向拖动改变的是 view.x。
    // 取 + ：地图跟着鼠标走（掍地图的手感），而不是视点反向跑。
    const dx = (e.clientX - d.x) / scale;
    const dy = (e.clientY - d.y) / scale;
    setView({ x: d.vx + dy, y: d.vy + dx });
  };

  const onMouseUp = (e: React.MouseEvent) => {
    const down = downPosRef.current;
    dragRef.current = undefined;
    downPosRef.current = undefined;

    // 安全区编辑模式下不产生航点
    if (draggingVertexRef.current >= 0) {
      draggingVertexRef.current = -1;
      return;
    }
    if (fenceMode) return;

    // 只有左键、且几乎没移动，才算一次「点击打点」
    // （否则拖动平移后松手会被误当成打点——这是误触的主要来源）
    if (!down || e.button !== 0) return;
    if (Math.hypot(e.clientX - down.x, e.clientY - down.y) > 5) return;

    const w = screenToWorld(e.clientX, e.clientY);
    setPending({
      kind: pendingKind,
      x: w.x,
      y: w.y,
      // 绝对模式用输入值；保持模式直接用 holdHeight（最近一次确定的高度设定），
      // 构造时就把 z 定下来
      z: heightMode === "absolute" ? heightInput : holdHeight,
      heightMode,
      yaw: yawDeg,
    });
  };

  return (
    <div className="map2d">
      <div className="view3d-toolbar">
        <span className="lbl">打点</span>
        <div className="seg">
          <button
            className={pendingKind === "planner" ? "on" : ""}
            onClick={() => setPendingKind("planner")}
            title="交给 EGO/SUPER 规划器"
          >
            ● 规划器点
          </button>
          <button
            className={pendingKind === "px4" ? "on" : ""}
            onClick={() => setPendingKind("px4")}
            title="直接走 PX4 位置控制"
          >
            ■ PX4 点
          </button>
        </div>

        <div className="tb-group">
          <span className="lbl">高度</span>
          <div className="seg">
            <button
              className={heightMode === "hold" ? "on" : ""}
              onClick={() => setHeightMode("hold")}
              title="沿用当前飞行高度"
            >
              保持
            </button>
            <button
              className={heightMode === "absolute" ? "on" : ""}
              onClick={() => setHeightMode("absolute")}
              title="使用右侧填写的数值"
            >
              指定
            </button>
          </div>
          <NumberField
            value={heightInput}
            onValueChange={setHeightInput}
            step="0.1"
            min={0.2}
            max={20}
            disabled={heightMode !== "absolute"}
            title={heightMode === "absolute" ? "输入目标高度（米）" : "先点「指定」才能输入"}
          />
          <span className="unit">m</span>
        </div>

        <div className="tb-group">
          <span className="lbl">机头</span>
          <div className="seg">
            {[-90, 0, 90, 180].map((v) => (
              <button
                key={v}
                className={yawDeg === v ? "on" : ""}
                onClick={() => setYawDeg(v)}
                title={`机头朝向 ${v}°（ENU：0° = +X 东，90° = +Y 北，逆时针为正）`}
              >
                {v}°
              </button>
            ))}
          </div>
          <NumberField
            value={yawDeg}
            onValueChange={setYawDeg}
            step="15"
            title="也可以直接填写任意角度（度）"
          />
          <span className="unit">°</span>
        </div>

        <span className="spacer" />
        <span className="lbl">点云</span>
        <button
          className={showCloud ? "on" : ""}
          onClick={() => setShowCloud((v) => !v)}
          title="把点云投影成 2D 俯视底图"
        >
          {showCloud ? `已投影 ${cloudPoints.toLocaleString()} 点` : "已隐藏"}
        </button>
        <button
          onClick={() => {
            setWaypoints([]);
            onWaypointsChange?.([]);
            setPending(undefined);
          }}
        >
          清空航点 ({waypoints.length})
        </button>
        <button
          onClick={() => onClearTrails?.()}
          title="清掉飞行轨迹（绿）、规划路径（橙）、期望轨迹 — 只清显示，不影响当前任务"
        >
          清除航线
        </button>
        {expTrajStat && (
          <span
            className="muted mono"
            title="ExpTraj 只在规划器 replan 时才发；这里显示收到的消息数与最大折线段数。一直为 0 说明订阅没成功、或规划器没在规划。"
          >
            ExpTraj {expTrajStat.msgs} 条 / {expTrajStat.segs.toFixed(0)} 段
          </span>
        )}
      </div>

      <div className="view3d-host" ref={hostRef}>
        <canvas
          ref={canvasRef}
          className="map2d-canvas"
          onWheel={onWheel}
          onMouseDown={onMouseDown}
          onMouseMove={onMouseMove}
          onMouseUp={onMouseUp}
          onMouseLeave={() => {
            dragRef.current = undefined;
            downPosRef.current = undefined;
          }}
          onContextMenu={(e) => e.preventDefault()}
        />

        {pending &&
          (() => {
            // 卡片跟随待确认点：优先放右下方，越界就翻到另一侧
            const sx = hostSize.w / 2 - (pending.y - view.y) * scale;
            const sy = hostSize.h / 2 - (pending.x - view.x) * scale;
            const CARD_W = 202;
            const CARD_H = 134;
            const GAP = 22;

            let left = sx + GAP;
            let top = sy - 32;
            if (left + CARD_W > hostSize.w - 8) left = sx - CARD_W - GAP;
            if (top + CARD_H > hostSize.h - 8) top = sy - CARD_H - GAP;
            if (left < 8) left = 8;
            if (top < 8) top = 8;

            return (
              <div className={`wp-confirm ${pending.kind}`} style={{ left, top }}>
                <div className="wp-confirm-head">
                  <b>{pending.kind === "planner" ? "● 规划器点" : "■ PX4 点"}</b>
                  <span className="muted">待确认</span>
                </div>
                <div className="wp-confirm-body">
                  <div>
                    <span className="k">X</span> {pending.x.toFixed(2)} m
                  </div>
                  <div>
                    <span className="k">Y</span> {pending.y.toFixed(2)} m
                  </div>
                  <div>
                    <span className="k">高度</span>{" "}
                    {pending.heightMode === "hold"
                      ? `保持 ${pending.z.toFixed(2)} m（最近一次高度设定）`
                      : `${pending.z.toFixed(2)} m`}
                  </div>
                  <div>
                    <span className="k">机头</span> {pending.yaw}°
                  </div>
                </div>
                {/* 安全区实时预判：点确认之前就能看出会不会被网关拒 */}
                {(() => {
                  const v = geofenceCheck(geofence, pending.x, pending.y, pending.z);
                  if (v.ok) return null;
                  return (
                    <div className="wp-confirm-warn">⚠ 越界：{v.reason}（网关侧会拒绝）</div>
                  );
                })()}
                <div className="wp-confirm-actions">
                  <button className="primary" onClick={confirmPending}>
                    确认发送
                  </button>
                  <button onClick={() => setPending(undefined)}>取消</button>
                </div>
              </div>
            );
          })()}
      </div>
    </div>
  );
}
