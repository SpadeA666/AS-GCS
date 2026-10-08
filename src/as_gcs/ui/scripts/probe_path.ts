/** 一次性排查：模拟前端订阅 /fsm_node/fsm/path，看能否拿到完整路径 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";
const url = process.argv[2] ?? "ws://localhost:8765";
const conn = new FoxgloveConnection(url, { reconnectMs: 0 });
let subbed = false, hits = 0, best = 0;
const tick = () => {
  if (subbed) return;
  const t = (conn as unknown as { topics: { topic: string }[] }).topics.find((x) =>
    x.topic.includes("fsm/path"),
  );
  if (!t) return;
  subbed = true;
  console.log(`  订阅 ${t.topic}`);
  conn.subscribe(t.topic, (msg) => {
    hits++;
    const m = msg as { poses?: { pose?: { position?: { x: number; y: number; z: number } } }[] };
    if (!Array.isArray(m.poses)) return;
    if (m.poses.length > best) {
      best = m.poses.length;
      const p0 = m.poses[0].pose?.position, p1 = m.poses[m.poses.length - 1].pose?.position;
      console.log(`  poses=${m.poses.length}  首=(${p0?.x.toFixed(2)},${p0?.y.toFixed(2)},${p0?.z.toFixed(2)})  末=(${p1?.x.toFixed(2)},${p1?.y.toFixed(2)},${p1?.z.toFixed(2)})`);
    }
  });
};
setInterval(tick, 1000);
setTimeout(() => {
  console.log(`\n  订阅成功=${subbed}  收到=${hits} 条  最大点数=${best}`);
  process.exit(0);
}, 25000);
