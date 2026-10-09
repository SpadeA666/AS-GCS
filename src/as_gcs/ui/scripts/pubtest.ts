/**
 * 连发 3 次目标点，看第几次才生效 —— 验证"首条丢失"假设。
 * 用法：npx vite-node scripts/pubtest.ts
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";

const conn = new FoxgloveConnection("ws://localhost:8765", { reconnectMs: 0 });

const mk = (x: number) => ({
  header: { seq: 0, stamp: { sec: 0, nsec: 0 }, frame_id: "map" },
  pose: { position: { x, y: 0.6, z: 1.0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
});

const send = (n: number, x: number, next?: () => void) => {
  console.log(`  发第 ${n} 次 (x=${x})…`);
  try {
    conn.publish("/move_base_simple/goal", "geometry_msgs/PoseStamped", mk(x));
  } catch (e) {
    console.log(`    ✗ ${String(e)}`);
  }
  if (next) setTimeout(next, 2000);
};

setTimeout(() => {
  send(1, 1.0, () =>
    send(2, 2.0, () =>
      send(3, 3.0, () => setTimeout(() => process.exit(0), 2500)),
    ),
  );
}, 6000);
