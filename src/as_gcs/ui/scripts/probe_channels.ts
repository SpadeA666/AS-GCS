/**
 * 一次性诊断：连接 bridge，把所有含 occupancy / inf_occ / optimal 的话题列出来。
 * 用来判断 EGO 的话题到底有没有出现在 bridge 推的频道列表里。
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";

const url = process.argv[2] ?? "ws://localhost:8765";

const conn = new FoxgloveConnection(url, {
  reconnectMs: 0,
  onChannels: (t) => {
    const hit = t.filter(
      (x) =>
        x.topic.includes("occupancy") ||
        x.topic.includes("inf_occ") ||
        x.topic.includes("optimal") ||
        x.topic.includes("grid_map"),
    );
    console.log(`\n总话题数: ${t.length}`);
    console.log(`匹配 occupancy/inf_occ/optimal/grid_map 的: ${hit.length} 个`);
    for (const h of hit) {
      console.log(`  ${h.topic}   [${h.schemaName}]`);
    }
    const cl = t.filter((x) => x.schemaName === "sensor_msgs/PointCloud2");
    console.log(`\n所有 PointCloud2 话题 (${cl.length} 个):`);
    for (const c of cl) console.log(`  ${c.topic}`);
    process.exit(0);
  },
});

setTimeout(() => {
  console.log("超时：没收到频道列表");
  process.exit(1);
}, 8000);
