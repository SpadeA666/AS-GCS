/**
 * 图像面板：显示 ROS 相机画面 + 框选目标跟踪。
 *
 * 消息支持：
 *   sensor_msgs/CompressedImage —— createImageBitmap 原生解码
 *   sensor_msgs/Image           —— 手动转 RGBA（rgb8/bgr8/rgba8/bgra8/mono8/16UC1/32FC1）
 *
 * 框选流程：
 *   在画面上拖出一个矩形 → 若有检测框落在框内，取其 Class；否则让用户手填类别
 *   → 确认卡片 → 调 /gcs/start_follow
 *
 * 带宽：优先选 /compressed 话题。raw 640x480 RGB 一帧 900KB，30Hz 就是 27MB/s。
 */
import { useEffect, useRef, useState } from "react";
import type { FoxgloveConnection } from "../core/FoxgloveConnection.ts";

interface DetBox {
  xmin: number;
  ymin: number;
  xmax: number;
  ymax: number;
  cls: string;
  prob: number;
}

interface Props {
  conn: FoxgloveConnection | undefined;
  topics: string[];
  selected: string;
  onSelectedChange: (t: string) => void;
  /** 检测框话题（yolov8_ros_msgs/BoundingBoxes），可为空 */
  detTopic?: string;
  /** 确认跟随：由父组件调 /gcs/start_follow */
  onFollow?: (targetClass: string, mode: 0 | 1) => void;
  onLog?: (msg: string) => void;
}

interface Stats {
  w: number;
  h: number;
  hz: number;
  ms: number;
  enc: string;
}

interface Sel {
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

export function ImagePanel({
  conn,
  topics,
  selected,
  onSelectedChange,
  detTopic,
  onFollow,
  onLog,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [stats, setStats] = useState<Stats | undefined>(undefined);
  const [err, setErr] = useState("");
  const [det, setDet] = useState<DetBox[]>([]);
  const [sel, setSel] = useState<Sel | undefined>(undefined);
  /** 拖拽起点（屏幕坐标，用于算图像坐标） */
  const dragStartRef = useRef<{ x: number; y: number } | undefined>(undefined);
  /** 拖拽中的框（图像坐标，用于实时绘制） */
  const [dragImg, setDragImg] = useState<Sel | undefined>(undefined);
  const [manualCls, setManualCls] = useState("");
  const busyRef = useRef(false);
  const imgSizeRef = useRef({ w: 0, h: 0 });
  /**
   * canvas 的实际显示尺寸（CSS 像素）。
   *
   * 为什么需要用 JS 量：
   *   .imgpanel-host 是 flex 容器，flex item 会被 blockify，inline-block 的
   *   “包裹内容”语义会丢失；而 canvas 的 max-width:100% 又是相对 stage 的，
   *   两者形成循环依赖，浏览器只能猜。一旦 stage 尺寸 与 canvas 显示尺寸不等，
   *   svg 叠加层（inset:0 + preserveAspectRatio="none"）就会拉伸错位 ——
   *   表现就是“图比例不对”且“拖框不跟鼠标”。
   *   直接量出来写死到 stage 上，两个问题一次性消掉。
   */
  const [disp, setDisp] = useState({ w: 0, h: 0 });
  /**
   * 图像面板的高度：按图像宽高比贴合可用宽度算出。
   * 以前面板高度写死 230px，横向图受高度限制被压得很窄，左右留一大片黑边。
   * 依赖只取 stats（图像内在尺寸，与布局无关），避免与 disp 形成循环。
   */
  const hostRef = useRef<HTMLDivElement>(null);
  const [hostH, setHostH] = useState(0);

  useEffect(() => {
    const host = hostRef.current;
    const iw = stats?.w ?? 0;
    const ih = stats?.h ?? 0;
    if (!host || iw <= 0 || ih <= 0) return;
    const avail = host.clientWidth;
    if (avail <= 0) return;
    const want = Math.round((avail * ih) / iw);
    const cap = Math.round(window.innerHeight * 0.42); // 别把地图挤没
    const next = Math.max(160, Math.min(want, cap));
    setHostH((p) => (Math.abs(p - next) < 2 ? p : next));
  }, [stats?.w, stats?.h]);

  useEffect(() => {
    const c = canvasRef.current;
    if (!c) return;
    const sync = () => {
      const r = c.getBoundingClientRect();
      setDisp((prev) =>
        Math.abs(prev.w - r.width) < 0.5 && Math.abs(prev.h - r.height) < 0.5
          ? prev
          : { w: r.width, h: r.height },
      );
    };
    sync();
    const ro = new ResizeObserver(sync);
    ro.observe(c);
    window.addEventListener("resize", sync);
    return () => {
      ro.disconnect();
      window.removeEventListener("resize", sync);
    };
  });

  // ── 图像订阅与解码 ──
  useEffect(() => {
    if (!conn || !selected) return;
    let frames = 0;
    let t0 = performance.now();
    let last: Stats = { w: 0, h: 0, hz: 0, ms: 0, enc: "" };

    const unsub = conn.subscribe(selected, (msg) => {
      if (busyRef.current) return; // latest-wins，丢弃重叠帧
      busyRef.current = true;
      const t = performance.now();
      const m = msg as {
        format?: string;
        encoding?: string;
        width?: number;
        height?: number;
        data?: Uint8Array | number[];
      };

      const done = (w: number, h: number, enc: string) => {
        last = { w, h, ms: performance.now() - t, hz: last.hz, enc };
        imgSizeRef.current = { w, h };
        frames++;
        const el = (performance.now() - t0) / 1000;
        if (el >= 1) {
          setStats({ ...last, hz: frames / el });
          frames = 0;
          t0 = performance.now();
        }
        busyRef.current = false;
      };

      // CompressedImage
      if (typeof m.format === "string") {
        const fmt = m.format.toLowerCase();
        const mime = fmt.includes("png") ? "image/png" : "image/jpeg";
        const bytes = m.data instanceof Uint8Array ? m.data : new Uint8Array(m.data ?? []);
        const ab = bytes.buffer.slice(
          bytes.byteOffset,
          bytes.byteOffset + bytes.byteLength,
        ) as ArrayBuffer;
        createImageBitmap(new Blob([ab], { type: mime }))
          .then((bmp) => {
            const c = canvasRef.current;
            if (c) {
              if (c.width !== bmp.width || c.height !== bmp.height) {
                c.width = bmp.width;
                c.height = bmp.height;
              }
              c.getContext("2d")?.drawImage(bmp, 0, 0);
            }
            done(bmp.width, bmp.height, fmt.includes("png") ? "png" : "jpeg");
            bmp.close();
          })
          .catch((e) => {
            setErr(`解码失败: ${String(e)}`);
            busyRef.current = false;
          });
        return;
      }

      // 原始 Image
      const w = m.width ?? 0;
      const h = m.height ?? 0;
      const enc = (m.encoding ?? "").toLowerCase();
      const bytes = m.data instanceof Uint8Array ? m.data : new Uint8Array(m.data ?? []);
      if (w <= 0 || h <= 0 || bytes.length === 0) {
        busyRef.current = false;
        return;
      }
      const c = canvasRef.current;
      if (!c) {
        busyRef.current = false;
        return;
      }
      if (c.width !== w || c.height !== h) {
        c.width = w;
        c.height = h;
      }
      const ctx = c.getContext("2d");
      if (ctx) {
        const img = ctx.createImageData(w, h);
        const dst = img.data;
        const n = w * h;
        if (enc === "rgb8" || enc === "bgr8") {
          const swap = enc === "bgr8";
          for (let i = 0; i < n; i++) {
            dst[i * 4] = bytes[i * 3 + (swap ? 2 : 0)];
            dst[i * 4 + 1] = bytes[i * 3 + 1];
            dst[i * 4 + 2] = bytes[i * 3 + (swap ? 0 : 2)];
            dst[i * 4 + 3] = 255;
          }
        } else if (enc === "rgba8" || enc === "bgra8") {
          const swap = enc === "bgra8";
          for (let i = 0; i < n; i++) {
            dst[i * 4] = bytes[i * 4 + (swap ? 2 : 0)];
            dst[i * 4 + 1] = bytes[i * 4 + 1];
            dst[i * 4 + 2] = bytes[i * 4 + (swap ? 0 : 2)];
            dst[i * 4 + 3] = bytes[i * 4 + 3];
          }
        } else if (enc === "mono8") {
          for (let i = 0; i < n; i++) {
            const v = bytes[i];
            dst[i * 4] = dst[i * 4 + 1] = dst[i * 4 + 2] = v;
            dst[i * 4 + 3] = 255;
          }
        } else if (enc === "16uc1" || enc === "32fc1") {
          // 深度：求量程做灰阶归一化
          const src =
            enc === "16uc1"
              ? new Uint16Array(bytes.buffer, bytes.byteOffset, n)
              : new Float32Array(bytes.buffer, bytes.byteOffset, n);
          let lo = Infinity;
          let hi = -Infinity;
          for (let i = 0; i < n; i++) {
            const v = src[i];
            if (v > 0 && Number.isFinite(v)) {
              if (v < lo) lo = v;
              if (v > hi) hi = v;
            }
          }
          const span = hi - lo || 1;
          for (let i = 0; i < n; i++) {
            const v = src[i];
            const g = v > 0 && Number.isFinite(v) ? Math.round((255 * (v - lo)) / span) : 0;
            dst[i * 4] = dst[i * 4 + 1] = dst[i * 4 + 2] = g;
            dst[i * 4 + 3] = 255;
          }
        } else {
          setErr(`暂不支持的 encoding: ${m.encoding}`);
          busyRef.current = false;
          return;
        }
        ctx.putImageData(img, 0, 0);
      }
      done(w, h, enc);
    });

    setErr("");
    return () => {
      unsub();
      busyRef.current = false;
    };
  }, [conn, selected]);

  // ── 检测框订阅 ──
  useEffect(() => {
    setDet([]);
    if (!conn || !detTopic) return;
    return conn.subscribe(detTopic, (msg) => {
      const m = msg as { bounding_boxes?: { Class?: string; probability?: number; xmin?: number; ymin?: number; xmax?: number; ymax?: number }[] };
      if (!Array.isArray(m.bounding_boxes)) return;
      setDet(
        m.bounding_boxes.map((b) => ({
          xmin: b.xmin ?? 0,
          ymin: b.ymin ?? 0,
          xmax: b.xmax ?? 0,
          ymax: b.ymax ?? 0,
          cls: b.Class ?? "",
          prob: b.probability ?? 0,
        })),
      );
    });
  }, [conn, detTopic]);

  // ── 叠加绘制：检测框 + 拖框 ──
  const drawOverlay = () => {
    // 叠加层改用 SVG（见 JSX）浮在 canvas 上方，不参与 canvas 重绘，
    // 否则每帧 putImageData 会把画上去的框擦掉。
  };
  void drawOverlay;

  /** 屏幕坐标 → 图像像素坐标 */
  const clientToImage = (cx: number, cy: number) => {
    const c = canvasRef.current;
    if (!c) return { x: 0, y: 0 };
    const r = c.getBoundingClientRect();
    return {
      x: ((cx - r.left) / Math.max(1, r.width)) * c.width,
      y: ((cy - r.top) / Math.max(1, r.height)) * c.height,
    };
  };

  const onDown = (e: React.MouseEvent) => {
    if (e.button !== 0) return;
    setSel(undefined);
    dragStartRef.current = { x: e.clientX, y: e.clientY };
    const p = clientToImage(e.clientX, e.clientY);
    setDragImg({ x0: p.x, y0: p.y, x1: p.x, y1: p.y });
  };

  const onMove = (e: React.MouseEvent) => {
    const s = dragStartRef.current;
    if (!s) return;
    const a = clientToImage(s.x, s.y);
    const b = clientToImage(e.clientX, e.clientY);
    setDragImg({
      x0: Math.min(a.x, b.x),
      y0: Math.min(a.y, b.y),
      x1: Math.max(a.x, b.x),
      y1: Math.max(a.y, b.y),
    });
  };

  const onUp = () => {
    const s = dragStartRef.current;
    dragStartRef.current = undefined;
    const box = dragImg;
    setDragImg(undefined);
    if (!s || !box) return;
    // 太小的框当误触（像素阈值按图像尺寸缩放）
    const minSide = Math.max(6, imgSizeRef.current.w / 100);
    if (box.x1 - box.x0 < minSide || box.y1 - box.y0 < minSide) return;
    setSel(box);
  };

  /** 选区内的检测框 */
  const hits = sel
    ? det.filter((b) => {
        const cx = (b.xmin + b.xmax) / 2;
        const cy = (b.ymin + b.ymax) / 2;
        return cx >= sel.x0 && cx <= sel.x1 && cy >= sel.y0 && cy <= sel.y1;
      })
    : [];
  const bestHit = hits.length > 0 ? hits.reduce((a, b) => (b.prob > a.prob ? b : a)) : undefined;

  const confirmFollow = () => {
    const cls = bestHit?.cls || manualCls.trim();
    if (!cls) {
      onLog?.("请先框选一个目标，或手动填写目标类别");
      return;
    }
    const mode: 0 | 1 = selected.includes("d435i") || selected.includes("camera_2") ? 0 : 1;
    onFollow?.(cls, mode);
    onLog?.(`框选跟随：class=${cls} mode=${mode === 0 ? "前视" : "下视"}`);
    setSel(undefined);
    setManualCls("");
  };

  const ordered = [...topics].sort((a, b) => {
    const ca = a.includes("compressed") ? 0 : 1;
    const cb = b.includes("compressed") ? 0 : 1;
    return ca - cb || a.localeCompare(b);
  });

  // 显示尺寸换算（叠加层用百分比定位，天然跟着 canvas 缩放）
  const size = imgSizeRef.current;

  return (
    <div className="imgpanel">
      <div className="imgpanel-toolbar">
        <select value={selected} onChange={(e) => onSelectedChange(e.target.value)}>
          {topics.length === 0 && <option value="">（未发现图像话题）</option>}
          {ordered.map((t) => (
            <option key={t} value={t}>
              {t}
            </option>
          ))}
        </select>
        <span className="lbl">检测框</span>
        <span className={detTopic ? "mono" : "muted mono"}>
          {detTopic || "未发现"}
        </span>
        {det.length > 0 && <span className="hint">{det.length} 个</span>}
        <span className="spacer" />
        {stats && (
          <span className="muted mono">
            {stats.w}×{stats.h} · {stats.enc} · {stats.hz.toFixed(1)}Hz · {stats.ms.toFixed(1)}ms
          </span>
        )}
      </div>

      <div
        className="imgpanel-host"
        ref={hostRef}
        style={hostH > 0 ? { height: `${hostH}px` } : undefined}
        onMouseDown={onDown}
        onMouseMove={onMove}
        onMouseUp={onUp}
        onMouseLeave={onUp}
      >
        <div
          className="imgpanel-stage"
          style={disp.w > 0 ? { width: `${disp.w}px`, height: `${disp.h}px` } : undefined}
        >
          <canvas ref={canvasRef} />
          {/* 叠加层：坐标系与 canvas 显示尺寸完全对齐（见 disp 的注释） */}
          {size.w > 0 && (
            <svg className="imgpanel-overlay" viewBox={`0 0 ${size.w} ${size.h}`} preserveAspectRatio="none">
              {det.map((b, i) => (
                <g key={i}>
                  <rect
                    x={b.xmin}
                    y={b.ymin}
                    width={Math.max(0, b.xmax - b.xmin)}
                    height={Math.max(0, b.ymax - b.ymin)}
                    fill="none"
                    stroke={bestHit === b ? "#48c46a" : "#4ea1ff"}
                    strokeWidth={Math.max(1.5, size.w / 400)}
                  />
                  <text
                    x={b.xmin + 3}
                    y={b.ymin - 4}
                    fill={bestHit === b ? "#48c46a" : "#4ea1ff"}
                    fontSize={Math.max(10, size.w / 45)}
                    fontFamily="ui-monospace, monospace"
                  >
                    {b.cls} {b.prob.toFixed(2)}
                  </text>
                </g>
              ))}
              {(sel || dragImg) && (
                <rect
                  x={(sel ?? dragImg)!.x0}
                  y={(sel ?? dragImg)!.y0}
                  width={Math.max(0, (sel ?? dragImg)!.x1 - (sel ?? dragImg)!.x0)}
                  height={Math.max(0, (sel ?? dragImg)!.y1 - (sel ?? dragImg)!.y0)}
                  fill="rgba(224,160,48,0.12)"
                  stroke="#e0a030"
                  strokeWidth={Math.max(2, size.w / 320)}
                  strokeDasharray={sel ? "none" : "6 4"}
                />
              )}
            </svg>
          )}
          {!selected && <div className="imgpanel-empty">选一个图像话题</div>}
          {err && <div className="imgpanel-err">{err}</div>}
        </div>

        {sel && (
          <div className="wp-confirm" style={{ left: 14, top: 14, position: "absolute" }}>
            <div className="wp-confirm-head">
              <b>框选目标</b>
              <span className="muted">{hits.length} 个命中</span>
            </div>
            <div className="wp-confirm-body">
              {bestHit ? (
                <>
                  <div>
                    <span className="k">类别</span> {bestHit.cls}
                  </div>
                  <div>
                    <span className="k">置信</span> {bestHit.prob.toFixed(2)}
                  </div>
                </>
              ) : (
                <>
                  <div className="muted" style={{ marginBottom: 6 }}>
                    选区内没有检测框（仿真里可能没跑 YOLO），手动填类别：
                  </div>
                  <input
                    className="num"
                    style={{ width: "100%" }}
                    value={manualCls}
                    placeholder="如 red_ballon"
                    onChange={(e) => setManualCls(e.target.value)}
                  />
                </>
              )}
            </div>
            <div className="wp-confirm-actions">
              <button className="primary" onClick={confirmFollow}>
                开始跟随
              </button>
              <button onClick={() => setSel(undefined)}>取消</button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
