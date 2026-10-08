/**
 * 点云探针：连 bridge 订阅点云话题，打印原始帧结构和解析结果。
 * 用法：npx vite-node scripts/probe_cloud.ts [/super_cloud]
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";
import { parsePointCloud2 } from "../src/core/pointcloud.ts";

const target = process.argv[2] ?? "/super_cloud";
console.log(`连接 ws://localhost:8765，目标话题 ${target}\n`);

const conn = new FoxgloveConnection("ws://localhost:8765", {
  reconnectMs: 0,
  onStateChange: (s) => console.log("[state]", s),
  onError: (e) => console.log("[error]", e.message),
});

setTimeout(() => {
  const cands = conn.topics.filter(
    (t) => t.topic.includes("cloud") || t.topic.includes("lidar") || t.topic.includes("points"),
  );
  console.log("═══ 候选点云话题 ═══");
  for (const t of cands) console.log(`  ${t.topic}  [${t.schemaName}]`);

  console.log(`\n═══ 订阅 ${target} ═══`);
  let n = 0;

  conn.subscribe(target, (msg) => {
    n++;
    const m = msg as Record<string, unknown>;
    if (n !== 1) return;

    console.log("\n── 第一帧的原始结构 ──");
    console.log("  顶层字段:", Object.keys(m).join(", "));

    const data = m.data as ArrayBufferView | undefined;
    console.log("  height / width:", m.height, "/", m.width);
    console.log("  point_step / row_step:", m.point_step, "/", m.row_step);
    console.log("  is_bigendian / is_dense:", m.is_bigendian, "/", m.is_dense);
    console.log("  fields:", JSON.stringify(m.fields));
    console.log(
      "  data:",
      data ? `${data.constructor.name} byteLength=${data.byteLength}` : "(缺失!)",
    );
    if (data) {
      console.log("  data.byteOffset:", data.byteOffset);
      console.log(
        "  头 24 字节:",
        Array.from(new Uint8Array(data.buffer, data.byteOffset, Math.min(24, data.byteLength)))
          .map((b) => b.toString(16).padStart(2, "0"))
          .join(" "),
      );
    }

    console.log("\n── 解析结果 ──");
    let parsed;
    try {
      parsed = parsePointCloud2(m as never);
    } catch (e) {
      console.log("  解析抛异常:", String(e));
      return;
    }
    console.log("  点数:", parsed.count);

    if (parsed.count > 0) {
      let minx = Infinity, maxx = -Infinity;
      let miny = Infinity, maxy = -Infinity;
      let minz = Infinity, maxz = -Infinity;
      for (let i = 0; i < parsed.count; i++) {
        const x = parsed.positions[i * 3];
        const y = parsed.positions[i * 3 + 1];
        const z = parsed.positions[i * 3 + 2];
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
        if (z < minz) minz = z;
        if (z > maxz) maxz = z;
      }
      console.log(`  X: ${minx.toFixed(2)} ~ ${maxx.toFixed(2)}`);
      console.log(`  Y: ${miny.toFixed(2)} ~ ${maxy.toFixed(2)}`);
      console.log(`  Z: ${minz.toFixed(2)} ~ ${maxz.toFixed(2)}`);
      console.log(
        "  前 3 个点:",
        Array.from(parsed.positions.slice(0, 9))
          .map((v) => v.toFixed(3))
          .join(", "),
      );
      console.log("  scalar:", parsed.scalarName || "(无)", parsed.scalar ? "有值" : "");
    }
  });

  setTimeout(() => {
    console.log(`\n═══ 6 秒内共收到 ${n} 帧 ═══`);
    process.exit(0);
  }, 6000);
}, 3000);

setTimeout(() => process.exit(0), 20000);
