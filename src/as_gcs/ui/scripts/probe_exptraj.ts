/**
 * 一次性排查脚本：模拟前端连接，订阅 SUPER 的 ExpTraj（MarkerArray），
 * 按前端 App.tsx 里那套解析逻辑处理，看能不能真的拿到折线。
 *
 * 用法：npx vite-node scripts/probe_exptraj.ts
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";

const url = process.argv[2] ?? "ws://localhost:8765";
const TARGET = "visualization/exp_traj";

const conn = new FoxgloveConnection(url, { reconnectMs: 0 });

let subbed = false;
let hits = 0;

// topics 要等 advertise 到达才知道，这里轮询 onChannels 结果触发订阅
const trySubscribe = (topics: { topic: string }[]) => {
  if (subbed) return;
  const t = topics.find((x) => x.topic.includes(TARGET));
  if (!t) {
    console.log(`  还没看到含 "${TARGET}" 的话题（当前 ${topics.length} 个）`);
    return;
  }
  subbed = true;
  console.log(`  找到话题: ${t.topic}`);
  conn.subscribe(t.topic, (msg) => {
    hits++;
    const m = msg as {
      markers?: {
        type?: number;
        points?: { x: number; y: number; z: number }[];
        color?: { r: number; g: number; b: number; a: number };
      }[];
    };
    if (!Array.isArray(m.markers)) {
      console.log("    收到消息但 markers 不是数组");
      return;
    }
    let segs = 0;
    let pts = 0;
    let col: string | undefined;
    const flat: number[] = [];
    for (const mk of m.markers) {
      if (mk.type !== 0 && mk.type !== 4 && mk.type !== 5) continue;
      if (!Array.isArray(mk.points)) continue;
      segs++;
      pts += mk.points.length;
      for (const p of mk.points) flat.push(p.x, p.y, p.z);
      const c = mk.color;
      if (!col && c && c.a > 0) {
        col = `rgba(${Math.round(c.r * 255)},${Math.round(c.g * 255)},${Math.round(c.b * 255)},${c.a})`;
      }
    }
    if (hits <= 3) {
      console.log(
        `    #${hits} markers=${m.markers.length} 折线段=${segs} 点数=${pts} 扁平长度=${flat.length} color=${col ?? "—"}`,
      );
    }
  });
};

const connAny = conn as unknown as { topics: { topic: string }[] };
setInterval(() => {
  trySubscribe(connAny.topics);
}, 1000);

// SUPER 只在“真的发生 replan”时才会调 vizExpTraj（super_planner.cpp:765），
// 光订阅不发点是不会收到任何东西的。所以这里主动发一个目标点触发规划。
setTimeout(() => {
  if (!subbed) {
    console.log("  20s 内没订阅上，放弃发点");
    return;
  }
  console.log("  发一个目标点触发 replan…");
  try {
    conn.publish("/move_base_simple/goal", "geometry_msgs/PoseStamped", {
      header: {
        seq: 0,
        stamp: { sec: Math.floor(Date.now() / 1000), nsec: (Date.now() % 1000) * 1e6 },
        frame_id: "map",
      },
      pose: {
        position: { x: 1.5, y: 0.6, z: 1.0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
  } catch (e) {
    console.log(`  发点失败: ${String(e)}`);
  }
}, 8000);

setTimeout(() => {
  console.log(`\n═══ 汇总 ═══`);
  console.log(`  订阅成功: ${subbed}`);
  console.log(`  收到消息: ${hits} 条`);
  process.exit(0);
}, 40000);
