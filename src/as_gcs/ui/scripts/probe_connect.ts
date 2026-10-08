/**
 * 连接耗时测量：分别记录 WS 握手、advertise 广播、服务广播的时间点。
 * 用法：npx vite-node scripts/probe_connect.ts [ws://localhost:8765]
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";

const url = process.argv[2] ?? "ws://localhost:8765";
const t0 = performance.now();
const ms = () => (performance.now() - t0).toFixed(0).padStart(5);

let channelsAt = 0;
let servicesAt = 0;
let chBytes = 0;
let svcCount = 0;

console.log(`连接 ${url}\n`);

const conn = new FoxgloveConnection(url, {
  reconnectMs: 0,
  onStateChange: (s) => console.log(`  [${ms()}ms] state=${s}`),
  onError: (e) => console.log(`  [${ms()}ms] error: ${e.message}`),
  onChannels: (t) => {
    if (!channelsAt) {
      channelsAt = performance.now() - t0;
      chBytes = t.reduce((a, x) => a + x.schema.length + x.topic.length + x.schemaName.length, 0);
    }
    console.log(`  [${ms()}ms] channels=${t.length}  schema累计=${(chBytes / 1024).toFixed(0)}KB`);
  },
  onServices: (s) => {
    if (!servicesAt) {
      servicesAt = performance.now() - t0;
      svcCount = s.length;
    }
    console.log(`  [${ms()}ms] services=${s.length}`);
  },
});

setTimeout(() => {
  console.log(`\n═══ 汇总 ═══`);
  console.log(`  第一个 advertise 到达:  ${channelsAt.toFixed(0)} ms`);
  console.log(`  schema 文本总量:       ${(chBytes / 1024).toFixed(0)} KB（${svcCount} 个服务未计）`);
  console.log(`  话题数:                ${conn.topics.length}`);
  process.exit(0);
}, 12000);
