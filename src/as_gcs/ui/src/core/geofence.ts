/**
 * 安全区（Geofence）共用工具。
 *
 * 放在 core 而不是留在 Map2DPanel 内部：2D 画布的绘制/顶点编辑（Map2DPanel）
 * 和左侧控件面板（ControlPanel）都要用到排序，抽出来避免两边各存一份。
 */

/**
 * 按绕质心的极角排序多边形顶点。
 *
 * 为什么需要：用户随手点几个点，顺序不一定是"沿着轮廓走"的。
 * 直接按点击顺序连线会产生自交多边形（看着像"折叠"了）。
 * 按极角排序后，任意点击顺序都能得到正确的简单多边形（凸多边形完美，
 * 凹多边形也可能被改成凸的——安全区场景下可以接受）。
 *
 * 注意：只用于绘制与下发，**不改变存储顺序**——否则拖拽顶点时索引会跳。
 */
export function sortPolygon(pts: { x: number; y: number }[]): { x: number; y: number }[] {
  if (pts.length < 3) return pts;
  let cx = 0;
  let cy = 0;
  for (const p of pts) {
    cx += p.x;
    cy += p.y;
  }
  cx /= pts.length;
  cy /= pts.length;
  return [...pts].sort(
    (a, b) => Math.atan2(a.y - cy, a.x - cx) - Math.atan2(b.y - cy, b.x - cx),
  );
}

/** 安全区判定的结果 */
export interface FenceVerdict {
  ok: boolean;
  /** 不通过时的原因（直接拿去显示） */
  reason?: string;
}

/**
 * 判断一个点是否落在安全区内。
 *
 * 必须与网关的 checkGeofence 用同一套规则（z 范围 + 射线法），否则会出现
 * “前端显示在区内、网关却拒绝”这种自相矛盾的情况，比不提示还难排查。
 * 对应 as_controller/src/gcs_gateway.cpp 的 checkGeofence。
 *
 * 安全区未启用 或 顶点不足 3 个 → 视为不限制。
 */
export function geofenceCheck(
  g: { enabled: boolean; polygon: { x: number; y: number }[]; zMin: number; zMax: number } | undefined,
  x: number,
  y: number,
  z: number,
): FenceVerdict {
  if (!g || !g.enabled || g.polygon.length < 3) return { ok: true };

  if (z < g.zMin || z > g.zMax) {
    return {
      ok: false,
      reason: `高度 ${z.toFixed(2)}m 超出安全区 [${g.zMin.toFixed(1)}, ${g.zMax.toFixed(1)}]m`,
    };
  }

  // 射线法：从该点向右引射线，数穿过多边形边的次数，奇数为内部
  let inside = false;
  const poly = g.polygon;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const pi = poly[i];
    const pj = poly[j];
    if (
      pi.y > y !== pj.y > y &&
      x < ((pj.x - pi.x) * (y - pi.y)) / (pj.y - pi.y) + pi.x
    ) {
      inside = !inside;
    }
  }
  if (!inside) {
    return { ok: false, reason: `目标点 (${x.toFixed(2)}, ${y.toFixed(2)}) 在安全区外` };
  }
  return { ok: true };
}
