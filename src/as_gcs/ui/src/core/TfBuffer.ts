/**
 * TF 缓冲：对齐 tf2 语义的最小实现
 *
 * 从 robviz 移植。保持 core 目录零 three.js 依赖（Node 端测试/MCP 也要能用），
 * 所以四元数 slerp 自己实现。
 *
 * 关键点：
 *   - 每个 frame 保留时间窗内的样本，二分查找相邻两帧插值
 *   - lookup 走双链上溯找公共祖先，链断开返回 undefined（调用方必须显性化处理，
 *     不能静默——这是调 TF 问题时最重要的信号）
 */

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number]; // x, y, z, w

export interface Transform {
  translation: Vec3;
  rotation: Quat;
}

export type Matrix4 = number[]; // 列主序 16 元素

export const IDENTITY: Transform = {
  translation: [0, 0, 0],
  rotation: [0, 0, 0, 1],
};

interface Sample {
  time: number; // ns
  transform: Transform;
}

interface FrameEntry {
  parent: string;
  samples: Sample[];
}

const DEFAULT_WINDOW_NS = 10_000_000_000; // 10s

export class TfBuffer {
  #frames = new Map<string, FrameEntry>();
  #staticFrames = new Map<string, Transform>();
  #windowNs: number;
  #lastSetStampNs = 0n;

  constructor(windowNs: number = DEFAULT_WINDOW_NS) {
    this.#windowNs = windowNs;
  }

  /** 清空（重连时调用） */
  clear(): void {
    this.#frames.clear();
    this.#staticFrames.clear();
    this.#lastSetStampNs = 0n;
  }

  /** 写入一组变换（/tf 或 /tf_static 解出的消息） */
  setTransforms(
    transforms: { header: { frame_id: string; stamp: unknown }; child_frame_id: string; transform: { translation: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } } }[],
    isStatic: boolean,
    fallbackTimeNs: bigint,
  ): void {
    for (const t of transforms) {
      const parent = stripSlash(t.header.frame_id);
      const child = stripSlash(t.child_frame_id);
      if (!child || child === parent) continue;

      const transform: Transform = {
        translation: [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
        rotation: [
          t.transform.rotation.x,
          t.transform.rotation.y,
          t.transform.rotation.z,
          t.transform.rotation.w,
        ],
      };

      if (isStatic) {
        this.#staticFrames.set(child, transform);
        continue;
      }

      const stamp = toNs(t.header.stamp);
      const time = stamp === 0n ? Number(fallbackTimeNs) : Number(stamp);

      let entry = this.#frames.get(child);
      if (!entry || entry.parent !== parent) {
        entry = { parent, samples: [] };
        this.#frames.set(child, entry);
      }
      entry.samples.push({ time, transform });
    }

    if (this.#lastSetStampNs < fallbackTimeNs) this.#lastSetStampNs = fallbackTimeNs;
    this.#prune();
  }

  /** 已知 frame 列表（含 static） */
  frameIds(): string[] {
    const s = new Set<string>();
    for (const [child, e] of this.#frames) {
      s.add(child);
      s.add(e.parent);
    }
    for (const [child, parent] of this.#staticStaticPairs()) {
      s.add(child);
      s.add(parent);
    }
    return [...s].sort();
  }

  #staticStaticPairs(): [string, string][] {
    // /tf_static 只存了 child→transform，parent 从 /tf 链里也能拿到；
    // 这里没有额外信息，返回空即可（frameIds 已覆盖动态部分）
    return [];
  }

  /**
   * 求 target 系下 source 系的变换。
   * 链断开返回 undefined。
   */
  lookup(targetFrame: string, sourceFrame: string, timeNs: number): Transform | undefined {
    const target = stripSlash(targetFrame);
    const source = stripSlash(sourceFrame);
    if (target === source) return IDENTITY;

    const chainS = this.#chainToRoot(source);
    const chainT = this.#chainToRoot(target);
    if (!chainS || !chainT) return undefined;

    // 找公共祖先
    const inTarget = new Map<string, number>();
    chainT.forEach((f, i) => inTarget.set(f, i));

    let rootIdx = -1;
    let sourceIdx = -1;
    for (let i = 0; i < chainS.length; i++) {
      const idx = inTarget.get(chainS[i]);
      if (idx !== undefined) {
        rootIdx = i;
        sourceIdx = idx;
        break;
      }
    }
    if (rootIdx < 0) return undefined; // 两个不相连的树

    // source → 公共祖先
    let m: Matrix4 | undefined = identityMatrix();
    for (let i = 0; i < rootIdx; i++) {
      const child = chainS[i];
      const t = this.#lookupEdge(child, timeNs);
      if (!t) return undefined;
      m = mulMatrix(m, transformToMatrix(t));
    }
    // 公共祖先 → target（取逆）
    for (let i = sourceIdx - 1; i >= 0; i--) {
      const child = chainT[i];
      const t = this.#lookupEdge(child, timeNs);
      if (!t) return undefined;
      const inv = invertTransform(t);
      m = mulMatrix(m, transformToMatrix(inv));
    }
    return matrixToTransform(m);
  }

  /** 从 frame 上溯到根，返回 [frame, parent, grandparent, ...] */
  #chainToRoot(frame: string): string[] | undefined {
    const chain: string[] = [frame];
    const guard = new Set<string>([frame]);
    let cur = frame;
    for (let depth = 0; depth < 64; depth++) {
      const entry = this.#frames.get(cur);
      const parent = entry?.parent ?? null;
      if (!parent) {
        // 到根了；如果这个 frame 有 static 变换但没动态链，也认为到根
        return chain;
      }
      if (guard.has(parent)) return undefined; // 成环
      guard.add(parent);
      chain.push(parent);
      cur = parent;
    }
    return undefined; // 太深，视为异常
  }

  /** 取 child→parent 的变换（带插值） */
  #lookupEdge(child: string, timeNs: number): Transform | undefined {
    const st = this.#staticOnly(child);
    if (st) return st;

    const entry = this.#frames.get(child);
    if (!entry || entry.samples.length === 0) return undefined;
    const s = entry.samples;
    if (s.length === 1) return s[0].transform;

    // 二分找第一个 time >= timeNs
    let lo = 0;
    let hi = s.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (s[mid].time < timeNs) lo = mid + 1;
      else hi = mid;
    }
    const b = s[lo];
    const a = s[Math.max(0, lo - 1)];
    if (a === b) return b.transform;

    const span = b.time - a.time;
    if (span <= 0) return b.transform;
    const ratio = clamp01((timeNs - a.time) / span);

    return {
      translation: [
        lerp(a.transform.translation[0], b.transform.translation[0], ratio),
        lerp(a.transform.translation[1], b.transform.translation[1], ratio),
        lerp(a.transform.translation[2], b.transform.translation[2], ratio),
      ],
      rotation: slerp(a.transform.rotation, b.transform.rotation, ratio),
    };
  }

  #staticOnly(child: string): Transform | undefined {
    return this.#staticFrames.get(child);
  }

  #prune(): void {
    const cutoff = Number(this.#lastSetStampNs) - this.#windowNs;
    for (const entry of this.#frames.values()) {
      const s = entry.samples;
      if (s.length < 2) continue;
      let drop = 0;
      while (drop < s.length - 1 && s[drop].time < cutoff) drop++;
      if (drop > 0) entry.samples = s.slice(drop);
    }
  }
}

// ---------------- 数学工具（不依赖 three.js） ----------------

export function stripSlash(f: string): string {
  return f.startsWith("/") ? f.slice(1) : f;
}

export function toNs(stamp: unknown): bigint {
  if (!stamp || typeof stamp !== "object") return 0n;
  const s = stamp as { sec?: number; secs?: number; nsec?: number; nsecs?: number };
  const sec = s.sec ?? s.secs ?? 0;
  const nsec = s.nsec ?? s.nsecs ?? 0;
  return BigInt(Math.trunc(sec)) * 1_000_000_000n + BigInt(Math.trunc(nsec));
}

function clamp01(v: number): number {
  return v < 0 ? 0 : v > 1 ? 1 : v;
}

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

export function slerp(a: Quat, b: Quat, t: number): Quat {
  let [ax, ay, az, aw] = a;
  let [bx, by, bz, bw] = b;
  let dot = ax * bx + ay * by + az * bz + aw * bw;
  if (dot < 0) {
    bx = -bx; by = -by; bz = -bz; bw = -bw;
    dot = -dot;
  }
  if (dot > 0.9995) {
    return normalizeQuat([ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t, aw + (bw - aw) * t]);
  }
  const theta0 = Math.acos(Math.min(1, dot));
  const theta = theta0 * t;
  const sinTheta = Math.sin(theta);
  const sinTheta0 = Math.sin(theta0);
  const s0 = Math.cos(theta) - (dot * sinTheta) / sinTheta0;
  const s1 = sinTheta / sinTheta0;
  return normalizeQuat([ax * s0 + bx * s1, ay * s0 + by * s1, az * s0 + bz * s1, aw * s0 + bw * s1]);
}

export function normalizeQuat(q: Quat): Quat {
  const len = Math.hypot(q[0], q[1], q[2], q[3]);
  if (len === 0) return [0, 0, 0, 1];
  return [q[0] / len, q[1] / len, q[2] / len, q[3] / len];
}

export function invertTransform(t: Transform): Transform {
  const [x, y, z, w] = t.rotation;
  const invRot: Quat = [-x, -y, -z, w];
  const r = quatToMatrix(invRot);
  const [tx, ty, tz] = t.translation;
  return {
    translation: [
      -(r[0] * tx + r[4] * ty + r[8] * tz),
      -(r[1] * tx + r[5] * ty + r[9] * tz),
      -(r[2] * tx + r[6] * ty + r[10] * tz),
    ],
    rotation: invRot,
  };
}

/** 列主序 4x4 */
export function quatToMatrix(q: Quat): Matrix4 {
  const [x, y, z, w] = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  return [
    1 - (yy + zz), xy + wz, xz - wy, 0,
    xy - wz, 1 - (xx + zz), yz + wx, 0,
    xz + wy, yz - wx, 1 - (xx + yy), 0,
    0, 0, 0, 1,
  ];
}

export function identityMatrix(): Matrix4 {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

export function transformToMatrix(t: Transform): Matrix4 {
  const m = quatToMatrix(t.rotation);
  m[12] = t.translation[0];
  m[13] = t.translation[1];
  m[14] = t.translation[2];
  return m;
}

export function mulMatrix(a: Matrix4, b: Matrix4): Matrix4 {
  const out = new Array<number>(16);
  for (let col = 0; col < 4; col++) {
    for (let row = 0; row < 4; row++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) sum += a[k * 4 + row] * b[col * 4 + k];
      out[col * 4 + row] = sum;
    }
  }
  return out;
}

export function matrixToTransform(m: Matrix4): Transform {
  const translation: Vec3 = [m[12], m[13], m[14]];
  const trace = m[0] + m[5] + m[10];
  let q: Quat;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    q = [(m[6] - m[9]) / s, (m[8] - m[2]) / s, (m[1] - m[4]) / s, 0.25 * s];
  } else if (m[0] > m[5] && m[0] > m[10]) {
    const s = Math.sqrt(1 + m[0] - m[5] - m[10]) * 2;
    q = [0.25 * s, (m[1] + m[4]) / s, (m[8] + m[2]) / s, (m[6] - m[9]) / s];
  } else if (m[5] > m[10]) {
    const s = Math.sqrt(1 + m[5] - m[0] - m[10]) * 2;
    q = [(m[1] + m[4]) / s, 0.25 * s, (m[6] + m[9]) / s, (m[8] - m[2]) / s];
  } else {
    const s = Math.sqrt(1 + m[10] - m[0] - m[5]) * 2;
    q = [(m[8] + m[2]) / s, (m[6] + m[9]) / s, 0.25 * s, (m[1] - m[4]) / s];
  }
  return { translation, rotation: normalizeQuat(q) };
}
