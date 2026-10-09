/**
 * 模拟前端行为：订阅 ExpTraj 并带上同样的节流参数，看能否收到【有效】轨迹。
 * 用于验证「节流会不会把有数据的那条砍掉」。
 *
 * 用法：npx vite-node scripts/probe_exptraj_throttled.ts [节流ms]
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";

const url = "ws://localhost:8765";
const throttle = Number(process.argv[2] ?? 15);
console.log(`  节流 = ${throttle} ms`);

const conn = new FoxgloveConnection(url, { reconnectMs: 0 });

let subbed = false;
let total = 0;      // 收到的消息总数
let valid = 0;      // 其中含有效折线的
let bestPts = 0;    // 见过的最多点数
let color = "—";

const tick = () => {
  if (subbed) return;
  const t = (conn as unknown as { topics: { topic: string }[] }).topics.find((x) =>
    x.topic.includes("visualization/exp_traj"),
  );
  if (!t) return;
  subbed = true;
  console.log(`  订阅 ${t.topic}`);
  conn.subscribe(
    t.topic,
    (msg) => {
      total++;
      const m = msg as {
        markers?: {
          type?: number;
          points?: { x: number; y: number; z: number }[];
          color?: { r: number; g: number; b: number; a: number };
        }[];
      };
      if (!Array.isArray(m.markers)) return;
      let pts = 0;
      let col: string | undefined;
      for (const mk of m.markers) {
        if (mk.type !== 0 && mk.type !== 4 && mk.type !== 5) continue;
        if (!Array.isArray(mk.points)) continue;
        pts += mk.points.length;
        const c = mk.color;
        if (!col && c && c.a > 0) {
          col = `rgba(${Math.round(c.r * 255)},${Math.round(c.g * 255)},${Math.round(c.b * 255)},${c.a})`;
        }
      }
      if (pts >= 2) {
        valid++;
        if (pts > bestPts) {
          bestPts = pts;
          if (col) color = col;
        }
      }
    },
    throttle,
  );
};
setInterval(tick, 500);

// 触发一次 replan
setTimeout(() => {
  if (!subbed) return;
  console.log("  发点触发 replan…");
  try {
    conn.publish("/move_base_simple/goal", "geometry_msgs/PoseStamped", {
      header: { seq: 0, stamp: { sec: 0, nsec: 0 }, frame_id: "map" },
      pose: {
        position: { x: 0.8, y: 0.5, z: 1.0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
  } catch (e) {
    console.log(`  发点失败: ${String(e)}`);
  }
}, 8000);

setTimeout(() => {
  console.log(`\n  收到消息 ${total} 条，其中有效 ${valid} 条，最大点数 ${bestPts}，颜色 ${color}`);
  process.exit(0);
}, 30000);
