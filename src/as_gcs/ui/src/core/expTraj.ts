/**
 * 期望轨迹（ExpTraj）的 Marker 归一化。
 *
 * 两个规划器的消息形态完全不同，这里统一成「一串扁平 xyz」：
 *   SUPER -> /fsm_node/visualization/exp_traj   MarkerArray，轨迹是 ARROW(0)（每段 2 点）
 *   EGO   -> /drone_0_ego_planner_node/optimal_list  单个 Marker
 *
 * ⚠ 关键坑（2026-10-09 实测）：EGO 的 optimal_list 用的是 **SPHERE_LIST(7)**，
 *   不是折线类型。早先只认 ARROW / LINE_STRIP / LINE_LIST，于是 EGO 的消息
 *   被整条 `continue` 掉，`points` 永远是空的 —— 表现就是「切到 EGO 后
 *   一条路径都看不到」，而且极容易被误判成"后端没发"，因为 ROS 侧
 *   `rostopic echo` 明明有数据。
 *
 * 判断依据要落在「能不能连成折线」上，而不是枚举某几个规划器的类型。
 */

/** Marker 的最小结构（只取我们用到的字段） */
export interface MarkerLike {
  type?: number;
  points?: { x: number; y: number; z: number }[];
  color?: { r: number; g: number; b: number; a: number };
}

/**
 * 可以当折线用的 Marker 类型。
 *   0 ARROW / 4 LINE_STRIP / 5 LINE_LIST —— 原生折线
 *   6 CUBE_LIST / 7 SPHERE_LIST / 8 POINTS —— 点序列，按顺序连起来即轨迹
 */
const POLYLINE_TYPES = new Set([0, 4, 5, 6, 7, 8]);

export interface ExpTrajParseResult {
  /** 扁平 [x,y,z, x,y,z, ...]；不足 2 个点时为空 */
  points: number[];
  /** marker 自带的 rgba(...)；没取到时 undefined（由调用方用默认色） */
  color?: string;
  /** 归一化后的 marker 个数（诊断用） */
  markerCount: number;
  /** 被类型 / 点数过滤掉的 marker 个数（诊断用，能区分"没数据"和"类型不认"） */
  skipped: number;
}

/**
 * 把一条 ExpTraj 消息（MarkerArray 或单个 Marker）解析成折线点串。
 *
 * 纯函数：不碰 ROS、不碰 React，可以直接在脚本里喂数据验证。
 */
export function parseExpTrajMarkers(msg: unknown): ExpTrajParseResult {
  const m = msg as MarkerLike & { markers?: MarkerLike[] };
  const markers = m.markers;
  // SUPER 是 MarkerArray（markers[]）；EGO 是单个 Marker（自身带 points）
  const list: MarkerLike[] = Array.isArray(markers)
    ? markers
    : m.points
      ? [{ type: m.type, points: m.points, color: m.color }]
      : [];

  const out: number[] = [];
  let color: string | undefined;
  let skipped = 0;

  for (const mk of list) {
    if (typeof mk.type !== "number" || !POLYLINE_TYPES.has(mk.type)) {
      skipped++;
      continue;
    }
    // 单点 marker（SUPER 的 SPHERE 航点标记）必须排除：
    // 它连不成线，收进来只会把相邻折线"拽歪"。
    if (!Array.isArray(mk.points) || mk.points.length < 2) {
      skipped++;
      continue;
    }
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
    if (!color && c && c.a > 0) {
      color = `rgba(${Math.round(c.r * 255)},${Math.round(c.g * 255)},${Math.round(c.b * 255)},${c.a})`;
    }
  }

  return { points: out, color, markerCount: list.length, skipped };
}
