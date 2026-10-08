/**
 * 用真实的 foxglove_bridge 验证 FoxgloveConnection（ROS 1 版）。
 * 用法：npx tsx scripts/probe.ts [ws://host:8765]
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";

const url = process.argv[2] ?? "ws://localhost:8765";
console.log(`连接 ${url}\n`);

const conn = new FoxgloveConnection(url, {
  reconnectMs: 0,
  onStateChange: (s) => console.log("  [state]", s),
  onChannels: (t) =>
    console.log("  [channels]", t.length, t.length ? "→ " + t.map((x) => x.topic).join(", ") : ""),
  onServices: (s) =>
    console.log("  [services]", s.length, s.length ? "→ " + s.map((x) => x.name).join(", ") : ""),
  onError: (e) => console.log("  [error]", e.message),
});

const received = new Map<string, number>();
let decodeErrors = 0;

setTimeout(() => {
  const topics = conn.topics;
  console.log(`\n═══ 话题 ${topics.length} 个 ═══`);
  for (const t of topics.slice(0, 25)) {
    console.log(`  ${t.topic}`);
    console.log(`      ${t.schemaName}  (schema ${t.schema.length} 字节)`);
  }
  if (topics.length > 25) console.log(`  ...还有 ${topics.length - 25} 个`);

  const services = conn.services;
  console.log(`\n═══ 服务 ${services.length} 个 ═══`);
  for (const s of services.slice(0, 15)) {
    console.log(`  ${s.name}  ${s.type}`);
  }

  // 订阅全部话题，看能否真的解出消息
  console.log("\n═══ 订阅全部话题 4 秒 ═══");
  for (const t of topics) {
    try {
      conn.subscribe(t.topic, () => {
        received.set(t.topic, (received.get(t.topic) ?? 0) + 1);
      });
    } catch (e) {
      console.log(`  订阅 ${t.topic} 抛错: ${String(e)}`);
    }
  }

  setTimeout(() => {
    console.log("\n═══ 收到的消息 ═══");
    if (received.size === 0) {
      console.log("  (4 秒内没有收到任何消息——如果确实没有节点在发布，这是正常的)");
    }
    for (const [topic, n] of [...received].sort((a, b) => b[1] - a[1])) {
      console.log(`  ${String(n).padStart(5)}  ${topic}`);
    }
    console.log(`\n解码失败次数: ${decodeErrors}`);
    conn.close();
    process.exit(0);
  }, 4000);
}, 3000);

// 允许脚本在什么话题都没有时也能正常结束
setTimeout(() => {
  console.log("\n[超时兜底] 退出");
  process.exit(0);
}, 20000);
