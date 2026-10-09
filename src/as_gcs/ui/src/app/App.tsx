/**
 * AS 地面站主界面
 *
 * 布局：左侧控制面板 | 中间主视图（3D 点云 / 2D 地图）| 右侧话题与服务
 */
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  FoxgloveConnection,
  type ConnectionState,
  type TopicInfo,
} from "../core/FoxgloveConnection";
import { TfBuffer } from "../core/TfBuffer";
import { ThreeDPanel } from "../panels/ThreeDPanel";
import { Map2DPanel, POSE_TOPIC_CANDIDATES, type Waypoint } from "../panels/Map2DPanel";
import { ControlPanel, type CameraMode, type Planner } from "../panels/ControlPanel";
import { ImagePanel } from "../panels/ImagePanel";
import type { Geofence, DronePose } from "../core/types";

interface TopicStat {
  hz: number;
  decodeMs: number;
}

/** 日志级别：error = 红色醒目，用于越界/拒绝这类必须一眼看到的事 */
export type LogLevel = "info" | "ok" | "warn" | "error";

export interface LogEntry {
  time: string;
  msg: string;
  level: LogLevel;
}

/**
 * 按日志内容推断级别。
 * 重点：安全区越界拒绝必须是红色——否则用户只看到“点下去了但什么都没发生”，
 * 根本不知道是安全区拦下的。
 */
function inferLogLevel(msg: string): LogLevel {
  if (/安全区外|超出安全区|被安全区拒绝|越界/.test(msg)) return "error";
  if (/失败|异常|被拒绝|未连接|不存在|超时/.test(msg)) return "warn";
  if (/成功|已受理|已到达|已启用|已切换/.test(msg)) return "ok";
  return "info";
}

const STATE_LABEL: Record<ConnectionState, string> = {
  connecting: "连接中",
  open: "已连接",
  closed: "已断开",
};

const CLOUD_SCHEMAS = new Set(["sensor_msgs/PointCloud2", "livox_ros_driver2/CustomMsg"]);

type ViewMode = "3d" | "2d";

export default function App() {
  const [url, setUrl] = useState("ws://localhost:8765");
  const [state, setState] = useState<ConnectionState | "idle">("idle");
  const [conn, setConn] = useState<FoxgloveConnection | undefined>(undefined);
  const [topics, setTopics] = useState<TopicInfo[]>([]);
  const [services, setServices] = useState<string[]>([]);
  const [subscribed, setSubscribed] = useState<Set<string>>(new Set());
  const [stats, setStats] = useState<Record<string, TopicStat>>({});
  const [errors, setErrors] = useState<LogEntry[]>([]);

  const [viewMode, setViewMode] = useState<ViewMode>("3d");
  const [planner, setPlanner] = useState<Planner>("super");
  /** 规划器切换状态，来自 /gcs/planner_status："ready super" / "switching ego" / "failed ego" */
  const [plannerStatus, setPlannerStatus] = useState<string>("ready super");
  const [cameraMode, setCameraMode] = useState<CameraMode>("off");
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  /** 2D 视图状态（也提升，切回时保持位置和缩放） */
  const [map2d, setMap2d] = useState({ x: 0, y: 0, scale: 55 });
  /** 飞行轨迹（扁平 [x,y,z,...]，ROS 系）。提升到父层，切视图不丢 */
  const [trajectory, setTrajectory] = useState<number[]>([]);
  /** 规划器输出的路径（同样扁平存） */
  const [planPath, setPlanPath] = useState<number[]>([]);
  /** SUPER 期望轨迹（rviz 里的 ExpTraj）：扁平 xyz + marker 自带颜色 */
  const [expTraj, setExpTraj] = useState<number[]>([]);
  const [expTrajColor, setExpTrajColor] = useState("#ff50c8");
  const trajRef = useRef<number[]>([]);

  /** 3D 点云开关（也提升，切视图不丢） */
  const [showRawCloud, setShowRawCloud] = useState(true);
  const [showInflatedCloud, setShowInflatedCloud] = useState(false);
  /** 无人机位姿（供 3D 模型用） */
  const [dronePose, setDronePose] = useState<DronePose | undefined>(undefined);
  /** 当前显示的图像话题（提升，切视图不丢） */
  const [selectedImage, setSelectedImage] = useState("");
  /** 安全区（提升到父层，2D 编辑 / 3D 显示共用同一份） */
  const [geofence, setGeofence] = useState<Geofence>({
    enabled: false,
    polygon: [],
    zMin: 0.0,
    zMax: 2.5,
  });
  /**
   * 安全区编辑模式。控件在左侧面板、交互在 2D 画布，状态必须放父层，
   * 否则左侧点了「绘制边界」画布不知道。
   */
  const [fenceMode, setFenceMode] = useState(false);

  const connRef = useRef<FoxgloveConnection | undefined>(undefined);
  /** 指向 connect()，供切换规划器后触发重连用（connect 本身在 useCallback 里） */
  const connectRef = useRef<(() => void) | undefined>(undefined);
  /** 上一次已生效的规划器，用来判断 ready 通知是不是真的换了规划器 */
  const plannerRef = useRef<Planner>("super");
  /** 连接代际号：旧连接的回调（尤其是 close）不允许再改 UI 状态 */
  const genRef = useRef(0);
  const tfRef = useRef(new TfBuffer());
  const unsubRef = useRef(new Map<string, () => void>());
  const statRef = useRef(
    new Map<string, { count: number; decodeMs: number; t0: number; hz: number }>(),
  );

  const pushLog = useCallback((msg: string, level?: LogLevel) => {
    // 未显式指定级别时按内容推断：
    // “超出/在安全区外”这类越界拒绝必须一眼看到（红色），否则用户不知道为什么没飞。
    const lv: LogLevel = level ?? inferLogLevel(msg);
    setErrors((prev) =>
      [{ time: new Date().toLocaleTimeString(), msg, level: lv }, ...prev].slice(0, 60),
    );
  }, []);

  /**
   * 下发安全区到网关。2D 画布的「启用并下发」和左侧面板的按钮都走这里，
   * 所以提到父层做一份——两边各写一遍迟早会不一致。
   */
  const applyGeofence = useCallback(
    (g: Geofence) => {
      // 关键：先落到本地 state。
      // 以前本函数只调服务、不改 state，而调用方传进来的 g.enabled 不会被保存，
      // 于是本地 enabled 永远是 false —— 点了「启用并下发」界面毫无变化
      // （画不出浅绿覆盖、按钮也不切换），看起来就像“点了没反应”。
      setGeofence(g);
      const c = connRef.current;
      if (!c) {
        pushLog("安全区: 已更新本地，但未连接（没下发到网关）");
        return;
      }
      c.callService("/gcs/set_geofence", {
        enable: g.enabled,
        polygon: g.polygon.map((p) => ({ x: p.x, y: p.y, z: 0 })),
        z_min: g.zMin,
        z_max: g.zMax,
      })
        .then((r) => {
          const res = r as { success?: boolean; message?: string };
          pushLog(
            `安全区${g.enabled ? "启用" : "关闭"}: ${res.success ? "成功" : "失败"} ${res.message ?? ""}`,
          );
        })
        .catch((e) => pushLog(`安全区设置失败: ${String(e)}`));
    },
    [pushLog],
  );

  /**
   * 清掉图上累积的三条线：飞行轨迹（绿）、规划路径（橙）、期望轨迹（ExpTraj）。
   * 只清本地显示，不动规划器状态 —— 否则会变成“顺手停掉一次任务”。
   * 注意 trajRef 是累加用的 ref，光清 state 不管它的话，下一帧又会把旧点写回来。
   */
  const clearTrails = useCallback(() => {
    trajRef.current = [];
    setTrajectory([]);
    setPlanPath([]);
    setExpTraj([]);
    pushLog("已清除轨迹与路径（只是清显示，不影响任务）");
  }, [pushLog]);

  const connect = useCallback(() => {
    // 代际号自增：下面所有回调都只在“自己还是当前连接”时才生效。
    // 否则点第二次连接时，被关掉的旧连接会回调 state=closed，把新连接的状态冲掉。
    const gen = ++genRef.current;
    const isCurrent = () => genRef.current === gen;

    connRef.current?.close();
    unsubRef.current.forEach((f) => f());
    unsubRef.current.clear();
    statRef.current.clear();
    tfRef.current.clear();
    trajRef.current = [];
    setTopics([]);
    setServices([]);
    setSubscribed(new Set());
    setStats({});
    setTrajectory([]);
    setPlanPath([]);
    setDronePose(undefined);

    const c = new FoxgloveConnection(url, {
      reconnectMs: 500,
      staleTimeoutMs: 5000,
      onStateChange: (s) => {
        if (!isCurrent()) return;
        setState(s);
      },
      onChannels: (t) => {
        if (!isCurrent()) return;
        setTopics(t);
      },
      onServices: (s) => {
        if (!isCurrent()) return;
        setServices(s.map((x) => x.name));
      },
      onError: (e) => {
        if (!isCurrent()) return;
        pushLog(e.message);
      },
      onMessageMeta: (meta) => {
        if (!isCurrent()) return;
        const cur =
          statRef.current.get(meta.topic) ?? { count: 0, decodeMs: 0, t0: performance.now(), hz: 0 };
        cur.count += 1;
        cur.decodeMs = meta.decodeMs;
        const elapsed = (performance.now() - cur.t0) / 1000;
        if (elapsed >= 1) {
          cur.hz = cur.count / elapsed;
          cur.count = 0;
          cur.t0 = performance.now();
        }
        statRef.current.set(meta.topic, cur);
      },
    });
    connRef.current = c;
    setConn(c);
  }, [url, pushLog]);

  const disconnect = useCallback(() => {
    // 同样自增代际：让即将到来的 close 回调不再影响状态，由这里直接置 idle
    genRef.current++;
    connRef.current?.close();
    connRef.current = undefined;
    unsubRef.current.forEach((f) => f());
    unsubRef.current.clear();
    setConn(undefined);
    setState("idle");
  }, []);

  useEffect(() => {
    const t = setInterval(() => {
      const next: Record<string, TopicStat> = {};
      for (const [topic, s] of statRef.current) {
        next[topic] = { hz: s.hz, decodeMs: s.decodeMs };
      }
      setStats(next);
    }, 1000);
    return () => clearInterval(t);
  }, []);

  const toggle = useCallback((topic: string) => {
    const c = connRef.current;
    if (!c) return;
    const existing = unsubRef.current.get(topic);
    if (existing) {
      existing();
      unsubRef.current.delete(topic);
      setSubscribed((prev) => {
        const n = new Set(prev);
        n.delete(topic);
        return n;
      });
      return;
    }
    const isTf = topic === "/tf" || topic === "/tf_static";
    const unsub = c.subscribe(topic, (msg) => {
      if (!isTf) return;
      const m = msg as { transforms?: unknown[] };
      if (!Array.isArray(m.transforms)) return;
      tfRef.current.setTransforms(
        m.transforms as never[],
        topic === "/tf_static",
        BigInt(Math.round(performance.now() * 1e6)),
      );
    });
    unsubRef.current.set(topic, unsub);
    setSubscribed((prev) => new Set(prev).add(topic));
  }, []);

  useEffect(() => () => connRef.current?.close(), []);

  const cloudTopics = useMemo(
    () => topics.filter((t) => CLOUD_SCHEMAS.has(t.schemaName)).map((t) => t.topic),
    [topics],
  );

  const poseTopics = useMemo(() => POSE_TOPIC_CANDIDATES(topics), [topics]);
  /**
   * 无人机位姿话题：**按优先级选，不能取列表第一个**。
   *
   * 候选只按 schema 筛（nav_msgs/Odometry 或 geometry_msgs/PoseStamped），
   * 里面同时有 MAVROS 的位姿和 LIO 的里程计。之前取 [0]，
   * SUPER 下恰好是 MAVROS 的；一换到 EGO 话题集合变了、顺序也变了，
   * 就可能指到别的（甚至没数据的）话题上，表现就是“收不到无人机位姿”。
   *
   * 约定：MAVROS 的 local_position 最稳（PX4 EKF 输出），优先用它。
   */
  const POSE_PREFERRED = [
    "/iris_0/mavros/local_position/pose",
    "/iris_0/mavros/local_position/odom",
    "/mavros/local_position/pose",
    "/mavros/local_position/odom",
  ];
  const poseTopic =
    POSE_PREFERRED.find((p) => poseTopics.includes(p)) ?? poseTopics[0] ?? "";

  /** 图像话题（CompressedImage 优先） */
  const imageTopics = useMemo(
    () =>
      topics
        .filter(
          (t) =>
            t.schemaName === "sensor_msgs/Image" ||
            t.schemaName === "sensor_msgs/CompressedImage",
        )
        .map((t) => t.topic),
    [topics],
  );

  /** 检测框话题（yolov8_ros_msgs/BoundingBoxes）。真机上有 camera_1/camera_2，仿真里可能没有 */
  const detTopic = useMemo(() => {
    const boxes = topics
      .filter((t) => t.schemaName === "yolov8_ros_msgs/BoundingBoxes")
      .map((t) => t.topic);
    if (boxes.length === 0) return "";
    // 尽量和当前图像话题对上：图像里是 camera_2 / d435i 就选 d435i 的检测框
    const wantD435i = selectedImage.includes("camera_2") || selectedImage.includes("d435i");
    const hit = boxes.find((b) => (wantD435i ? b.includes("d435i") : !b.includes("d435i")));
    return hit ?? boxes[0];
  }, [topics, selectedImage]);

  /** 规划路径话题：优先 fsm_node（SUPER），否则取第一个 nav_msgs/Path */
  const pathTopic = useMemo(() => {
    const paths = topics.filter((t) => t.schemaName === "nav_msgs/Path").map((t) => t.topic);
    return paths.find((t) => t.includes("fsm")) ?? paths[0] ?? "";
  }, [topics]);

  /** 原生点云：优先 super_cloud，其次 cloud_registered */
  const rawCloudTopic = useMemo(() => {
    return (
      cloudTopics.find((t) => t.includes("super_cloud")) ??
      cloudTopics.find((t) => t.includes("cloud_registered")) ??
      cloudTopics[0] ??
      ""
    );
  }, [cloudTopics]);

  /**
   * 膨胀点云：
   *   SUPER -> /fsm_node/rog_map/inf_occ
   *   EGO   -> /grid_map/occupancy_inflate
   * 两个规划器命名不一样，所以要依次匹配。
   */
  const inflatedCloudTopic = useMemo(
    () =>
      cloudTopics.find((t) => t.includes("inf_occ")) ??
      cloudTopics.find((t) => t.includes("occupancy_inflate")) ??
      "",
    [cloudTopics],
  );

  // ── 累积飞行轨迹（按位移过滤 + 限长，避免高频 setState）──
  useEffect(() => {
    if (!conn || !poseTopic) return;
    let lastPush = 0;
    return conn.subscribe(poseTopic, (msg) => {
      const m = msg as Record<string, unknown>;
      const pose = (m.pose as Record<string, unknown> | undefined) ?? m;
      const inner = pose.pose as Record<string, unknown> | undefined;
      const p = (inner?.position ?? pose.position) as
        | { x: number; y: number; z: number }
        | undefined;
      if (!p) return;

      // 同步无人机位姿给 3D 模型
      const q = (inner?.orientation ?? pose.orientation) as
        | { x: number; y: number; z: number; w: number }
        | undefined;
      setDronePose({
        x: p.x,
        y: p.y,
        z: p.z,
        qx: q?.x ?? 0,
        qy: q?.y ?? 0,
        qz: q?.z ?? 0,
        qw: q?.w ?? 1,
      });

      const t = trajRef.current;
      const n = t.length;
      if (n >= 3) {
        const dx = p.x - t[n - 3];
        const dy = p.y - t[n - 2];
        const dz = p.z - t[n - 1];
        if (dx * dx + dy * dy + dz * dz < 4e-4) return; // 位移 < 2cm 不记
      }
      t.push(p.x, p.y, p.z);
      const MAX = 6000 * 3;
      if (t.length > MAX) t.splice(0, t.length - MAX);

      const now = performance.now();
      if (now - lastPush > 200) {
        lastPush = now;
        setTrajectory([...t]);
      }
    });
  }, [conn, poseTopic]);

  // ── 订阅规划器路径 ──
  // 两个坑都在这里：
  //   ① /fsm_node/fsm/path 是 100Hz —— 节流必须放在解码层（subscribe 第三个参数）
  //   ② 单帧能有 2 万+ 个点（实测 23773）—— 直接全量 setState + 重绘会把页面压垮，
  //      所以这里按步长抽稀到 ~600 点。折线画在屏幕上，600 点和 2 万点肉眼看不出差别。
  const MAX_PATH_PTS = 600;
  useEffect(() => {
    if (!conn || !pathTopic) return;
    return conn.subscribe(
      pathTopic,
      (msg) => {
        const m = msg as {
          poses?: { pose?: { position?: { x: number; y: number; z: number } } }[];
        };
        if (!Array.isArray(m.poses) || m.poses.length === 0) return;
        const total = m.poses.length;
        const step = Math.max(1, Math.floor(total / MAX_PATH_PTS));
        const out: number[] = [];
        for (let i = 0; i < total; i += step) {
          const p = m.poses[i]?.pose?.position;
          if (p) out.push(p.x, p.y, p.z);
        }
        // 抽稀后补上终点，否则线会差一截
        const last = m.poses[total - 1]?.pose?.position;
        if (last) out.push(last.x, last.y, last.z);
        setPlanPath(out);
      },
      200,
    );
  }, [conn, pathTopic]);

  /**
   * ExpTraj 的话题名用 useMemo 提出来，作为 effect 的稳定依赖。
   *
   * 为什么不能直接依赖 `topics`：onChannels 在 advertise 和 unadvertise 时都会
   * 触发，而且每次传的都是新数组。只要话题列表有任何变动，effect 就会
   * cleanup（退订）→ 重新订阅；而 SUPER 只在 replan 那一瞬间发一条 exp_traj，
   * 落在重订阅的窗口里就永远丢了 —— 表现就是“从来没显示过”。
   */
  /**
   * 期望轨迹话题：
   *   SUPER -> /fsm_node/visualization/exp_traj（MarkerArray）
   *   EGO   -> /drone_0_ego_planner_node/optimal_list（单个 Marker）
   * 两个规划器命名不同，依次匹配。
   */
  const expTrajTopic = useMemo(
    () =>
      topics.find((x) => x.topic.includes("visualization/exp_traj"))?.topic ??
      topics.find((x) => x.topic.includes("optimal_list"))?.topic ??
      "",
    [topics],
  );

  /** 诊断用：实际收到的 ExpTraj 消息数与见过的最大折线段数 */
  const [expTrajStat, setExpTrajStat] = useState({ msgs: 0, segs: 0 });
  /** 只在第一次收到时打一条日志，避免刷屏 */
  const expTrajLoggedRef = useRef(false);

  // ── 订阅 SUPER 的期望轨迹（rviz 里的 ExpTraj）──
  // 它是 MarkerArray，不能按 nav_msgs/Path 解析，得从 markers[].points 里取折线。
  // 顺带把 marker 自带的颜色拿过来，与 rviz 显示保持一致。
  // 另：SUPER 是“有订阅者才发”（ros1_interface.hpp 里查 getNumSubscribers），
  // 所以前端一订阅它就会开始填充，rviz 那边同时开着也互不影响。
  useEffect(() => {
    if (!conn || !expTrajTopic) return;
    // 订阅建立时打一条，用于区分“没订阅上”和“订阅了但没数据”
    pushLog(`已订阅 ExpTraj: ${expTrajTopic}`);
    expTrajLoggedRef.current = false;
    return conn.subscribe(expTrajTopic, (msg) => {
      // 两种来源结构不同：
      //   SUPER -> MarkerArray，消息里有 markers[]
      //   EGO   -> 单个 Marker，消息本身就是一条（有 points/type/color）
      // 统一归一化成数组再走后面的解析。
      type Mk = {
        type?: number;
        points?: { x: number; y: number; z: number }[];
        color?: { r: number; g: number; b: number; a: number };
      };
      const m = msg as Mk & { markers?: Mk[] };
      const markers = m.markers;
      const list: Mk[] = Array.isArray(markers)
        ? markers
        : m.points
          ? [{ type: m.type, points: m.points, color: m.color }]
          : [];
      if (list.length === 0) return;
      const out: number[] = [];
      let col: string | undefined;
      for (const mk of list) {
        // SUPER 的轨迹用的是 ARROW（ros1_adapter.hpp 里 line_list.type = Marker::ARROW），
        // 每个 marker 是一小段（points = [起点, 终点]）；航点是 SPHERE 不带线段。
        // 所以三种都要收：ARROW=0 / LINE_STRIP=4 / LINE_LIST=5。
        if (mk.type !== 0 && mk.type !== 4 && mk.type !== 5) continue;
        if (!Array.isArray(mk.points)) continue;
        for (const p of mk.points) {
          // 相邻重复点会让 2D 描边出现断面，顺手去重
          const n = out.length;
          if (n >= 3 && n % 3 === 0) {
            const dx = p.x - out[n - 3];
            const dy = p.y - out[n - 2];
            const dz = p.z - out[n - 1];
            if (dx * dx + dy * dy + dz * dz < 1e-6) continue;
          }
          out.push(p.x, p.y, p.z);
        }
        const c = mk.color;
        if (!col && c && c.a > 0) {
          col = `rgba(${Math.round(c.r * 255)},${Math.round(c.g * 255)},${Math.round(c.b * 255)},${c.a})`;
        }
      }
      // 只在真的有折线时才覆盖。
      // SUPER 每次发布前会先 deleteAllMarkerArray 清场（发一个空 marker），
      // 若把空结果也 setExpTraj 进去，路径刚画出来就被下一条空消息抹掉，
      // 表现成“这条路径从来没显示过”。
      if (out.length >= 6) {
        setExpTraj(out);
        if (col) setExpTrajColor(col);
      }
      setExpTrajStat((p) => ({ msgs: p.msgs + 1, segs: Math.max(p.segs, out.length / 3) }));
      // 只在【真的收到有效轨迹】时记一次。
      // 之前写成“首次收到”，而 SUPER 每次都先发空 marker 清场，
      // 于是这条日志永远记的是那条空消息（markers=1 可用点=0），毫无信息量。
      if (!expTrajLoggedRef.current && out.length >= 6) {
        expTrajLoggedRef.current = true;
        pushLog(
          `ExpTraj 收到有效轨迹：markers=${list.length} 点数=${out.length / 3} 颜色=${col ?? "—"}`,
        );
      }
      // 关键：这里【不能做时间节流】。
      // SUPER 每次发布是连发两条：先 deleteAllMarkerArray 发空 marker 清场，
      // 紧接着发填好的轨迹，**两条间隔不到 1ms**。任何时间节流（哪怕是 15ms）
      // 都会把第二条（真正有数据的那条）丢掉 —— 实测 15ms 节流下收到 93 条
      // 消息、有效 0 条。
      //
      // 改用数据层过滤：空结果不 setState，于是渲染频率自然降到“每次有效
      // replan 一次”（实测约 11 次/秒），压力比之前还小。
    }, 0);
  }, [conn, expTrajTopic, pushLog]);

  const serviceSet = useMemo(() => new Set(services), [services]);

  // ── 网关服务自动刷新 ──
  // foxglove_bridge 只在连接建立的那一刻发一次完整服务列表，之后新增的服务不会推送
  // （实测：连接期间把 gcs_gateway 杀掉再拉起，客户端收不到任何 services 更新）。
  // 于是只要本连接建立得比网关早——仿真刚重启、网关还没起来时最典型——
  // services 里就永远没有 /gcs/*，所有控制按钮都报“网关服务不存在”，
  // 而无论怎么点都没用，只有刷新页面重新连接才会好。
  // 这里在“连上了但拿不到网关服务”时自动重连刷新，直到拿到为止。
  const gwRefreshLoggedRef = useRef(false);
  useEffect(() => {
    if (state !== "open" || !conn) return;
    if (services.some((s) => s.startsWith("/gcs/"))) {
      gwRefreshLoggedRef.current = false;
      return;
    }
    if (!gwRefreshLoggedRef.current) {
      gwRefreshLoggedRef.current = true;
      pushLog("已连上 bridge，但拿不到 /gcs/ 服务（网关起得比本连接晚），正在自动重连刷新…");
    }
    const t = setTimeout(() => connect(), 5000);
    return () => clearTimeout(t);
  }, [state, conn, services, connect, pushLog]);

  const tfFrameCount = useMemo(() => {
    if (topics.length === 0) return 0;
    return tfRef.current.frameIds().length;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [topics, subscribed]);

  const handlePlannerChange = useCallback(
    (p: Planner) => {
      // 切换后要重连刷新话题列表，所以把 connect 挂到 ref 上供 planner_status 回调调用
      connectRef.current = connect;
      // 真正切规划器要走网关：它会停掉旧节点、启新节点（十几秒），
      // 完成后通过 /gcs/planner_status 回推状态。这里只发请求，
      // 不要先 setPlanner —— 否则界面显示的“已切换”和实际不一致。
      if (!conn) {
        pushLog("切换规划器: 未连接");
        return;
      }
      if (!services.includes("/gcs/set_planner")) {
        pushLog("切换规划器: 网关服务 /gcs/set_planner 不存在（gcs_gateway 还没起）");
        return;
      }
      conn
        .callService("/gcs/set_planner", { planner: p })
        .then((r) => {
          const res = r as { success?: boolean; message?: string };
          if (res.success) {
            pushLog(`切换规划器 → ${p.toUpperCase()}: 已受理 ${res.message ?? ""}`, "ok");
          } else {
            // 被拒（飞行中 / 正在切换 / 已是目标）——用红色，别让用户以为是成功
            pushLog(`切换规划器 → ${p.toUpperCase()}: 被拒 — ${res.message ?? ""}`, "error");
          }
        })
        .catch((e) => pushLog(`切换规划器失败: ${String(e)}`, "error"));

      if (waypoints.length > 0) {
        pushLog(`已清空 ${waypoints.length} 个航点（防止旧点被新规划器执行）`);
        setWaypoints([]);
      }
    },
    [conn, services, waypoints.length, pushLog],
  );

  // 订阅规划器切换状态：网关切完之后回推 "ready ego/super"，
  // 前面才会把 planner 真正改掉（而不是点一下就改）。
  useEffect(() => {
    if (!conn) return;
    return conn.subscribe("/gcs/planner_status", (msg) => {
      const d = (msg as { data?: string }).data;
      if (!d) return;
      setPlannerStatus(d);
      const [st, who] = d.split(/\s+/);
      if (st === "ready" && (who === "ego" || who === "super")) {
        if (plannerRef.current !== (who as Planner)) {
          plannerRef.current = who as Planner;
          setPlanner(who as Planner);
          // 规划器一换，整套节点都换了，话题列表也就全变了。
          // foxglove_bridge 不推连接后新增的话题（和它不推新增服务是同一回事），
          // 所以必须重连一次，否则 EGO 的 occupancy_inflate / optimal_list
          // 根本不会出现在 topics 里，界面就永远看不到膨胀点云和轨迹。
          pushLog(`规划器已就绪: ${who.toUpperCase()}，刷新话题列表…`, "ok");
          setTimeout(() => connectRef.current?.(), 600);
        } else {
          pushLog(`规划器已就绪: ${who.toUpperCase()}`);
        }
      } else if (st === "switching") {
        pushLog(`规划器切换中: → ${(who ?? "").toUpperCase()}（约 10s）`);
      } else if (st === "failed") {
        pushLog(`规划器切换失败: ${who ?? ""}（看 /tmp/planner_switch.log）`);
      }
    });
  }, [conn, pushLog]);

  return (
    <div className="app">
      <header className="topbar">
        <span className="brand">
          <img src="/logo.png" alt="AS" className="brand-logo" />
          <span className="title">AS 地面站</span>
        </span>
        <input
          className="url"
          value={url}
          onChange={(e) => setUrl(e.target.value)}
          spellCheck={false}
        />
        <button onClick={connect}>连接</button>
        <button onClick={disconnect}>断开</button>
        <span className={`dot ${state}`} />
        <span className="state">{state === "idle" ? "未连接" : STATE_LABEL[state]}</span>

        <span className="divider" />

        <div className="seg">
          <button className={viewMode === "3d" ? "on" : ""} onClick={() => setViewMode("3d")}>
            3D
          </button>
          <button className={viewMode === "2d" ? "on" : ""} onClick={() => setViewMode("2d")}>
            2D
          </button>
        </div>

        <span className="spacer" />
        <span className="muted">
          话题 {topics.length} · 服务 {services.length} · TF {tfFrameCount} · 航点{" "}
          {waypoints.length}
        </span>
      </header>

      <main className="body">
        <aside className="ctrl-col">
          <ControlPanel
            conn={conn}
            services={serviceSet}
            planner={planner}
            plannerStatus={plannerStatus}
            onPlannerChange={handlePlannerChange}
            cameraMode={cameraMode}
            onCameraModeChange={setCameraMode}
            onLog={pushLog}
            geofence={geofence}
            onGeofenceChange={setGeofence}
            onApplyGeofence={applyGeofence}
            fenceMode={fenceMode}
            onFenceModeChange={setFenceMode}
            currentZ={dronePose?.z}
          />
        </aside>

        <div className="main-col">
          <div className="main-view">
            {viewMode === "3d" ? (
              <ThreeDPanel
                conn={conn}
                rawTopic={rawCloudTopic}
                inflatedTopic={inflatedCloudTopic}
                showRaw={showRawCloud}
                showInflated={showInflatedCloud}
                onShowRawChange={setShowRawCloud}
                onShowInflatedChange={setShowInflatedCloud}
                trajectory={trajectory}
                planPath={planPath}
                expTraj={expTraj}
                expTrajColor={expTrajColor}
                drone={dronePose}
                geofence={geofence}
              />
            ) : (
              <Map2DPanel
                conn={conn}
                poseTopic={poseTopic}
                rawTopic={rawCloudTopic}
                inflatedTopic={inflatedCloudTopic}
                showRawCloud={showRawCloud}
                showInflatedCloud={showInflatedCloud}
                view={map2d}
                onViewChange={setMap2d}
                planner={planner}
                defaultHeight={1.0}
                onWaypointsChange={setWaypoints}
                trajectory={trajectory}
                planPath={planPath}
                expTraj={expTraj}
                expTrajColor={expTrajColor}
                expTrajStat={expTrajStat}
                geofence={geofence}
                onGeofenceChange={setGeofence}
                onApplyGeofence={applyGeofence}
                fenceMode={fenceMode}
                onLog={pushLog}
                onClearTrails={clearTrails}
              />
            )}
          </div>
          <ImagePanel
            conn={conn}
            topics={imageTopics}
            selected={selectedImage}
            onSelectedChange={setSelectedImage}
            detTopic={detTopic}
            onFollow={(cls, mode) => {
              if (!conn) {
                pushLog("框选跟随: 未连接");
                return;
              }
              conn
                .callService("/gcs/start_follow", { target_class: cls, mode })
                .then((r) => {
                  const res = r as { success?: boolean; message?: string };
                  pushLog(`框选跟随(${cls}): ${res.success ? "已受理" : "失败"} ${res.message ?? ""}`);
                })
                .catch((e) => pushLog(`框选跟随失败: ${String(e)}`));
            }}
            onLog={pushLog}
          />
        </div>

        <aside className="sidebar">
          <section className="panel">
            <h2>话题</h2>
            {topics.length === 0 && <p className="muted">未连接或还没有 advertise</p>}
            <ul className="topic-list">
              {topics.map((t) => {
                const st = stats[t.topic];
                const on = subscribed.has(t.topic);
                return (
                  <li key={t.topic} className={on ? "on" : ""}>
                    <button
                      className="topic-btn"
                      onClick={() => toggle(t.topic)}
                      title={`${t.topic}\n${t.schemaName}${st ? `\n${st.hz.toFixed(1)} Hz  解码 ${st.decodeMs.toFixed(2)} ms` : ""}`}
                    >
                      <span className="tname" title={t.topic}>
                        {t.topic}
                      </span>
                      <span className="ttype" title={t.schemaName}>
                        {t.schemaName}
                      </span>
                      <span className="thz">{st ? `${st.hz.toFixed(1)}` : ""}</span>
                    </button>
                  </li>
                );
              })}
            </ul>
          </section>

          <section className="panel">
            <h2>服务</h2>
            {services.length === 0 && <p className="muted">无</p>}
            <ul className="plain">
              {services.map((s) => (
                <li key={s}>{s}</li>
              ))}
            </ul>
          </section>

          <section className="panel">
            <h2>日志</h2>
            {errors.length === 0 && <p className="muted">无</p>}
            <ul className="plain logs">
              {errors.map((e, i) => (
                <li key={i} className={`log-${e.level}`}>
                  <span className="log-time">{e.time}</span> {e.msg}
                </li>
              ))}
            </ul>
          </section>
        </aside>
      </main>
    </div>
  );
}
