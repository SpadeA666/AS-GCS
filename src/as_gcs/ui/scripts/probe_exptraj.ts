/**
 * 排查脚本：模拟前端连接，订阅 ExpTraj 话题，
 * 用前端【真实在用的】parseExpTrajMarkers() 解析，看能不能拿到折线。
 *
 * 存在的意义：ROS 侧 `rostopic echo` 有数据、界面却一条线都没有，这种
 * 情况太多了（话题选错 / 消息类型没覆盖 / 空 marker 清场）。这个脚本
 * 直接把「收到什么 → 解析出什么」摊开，一步定位卡在哪一层。
 *
 * 用法：
 *   npx vite-node scripts/probe_exptraj.ts                  # 默认 ws://localhost:8765
 *   npx vite-node scripts/probe_exptraj.ts ws://host:8765
 */
import { FoxgloveConnection } from "../src/core/FoxgloveConnection.ts";
import { parseExpTrajMarkers } from "../src/core/expTraj.ts";

const url = process.argv[2] ?? "ws://localhost:8765";

/** 两个规划器的 ExpTraj 话题名不同，两个都订 —— 谁有数据一目了然 */
const CANDIDATES = ["optimal_list", "visualization/exp_traj"];

const conn = new FoxgloveConnection(url, { reconnectMs: 0 });
const connAny = conn as unknown as { topics: { topic: string }[] };

const subbed = new Set<string>();
let hits = 0;
let bestPts = 0;

const trySubscribe = (topics: { topic: string }[]) => {
  for (const key of CANDIDATES) {
    const t = topics.find((x) => x.topic.includes(key));
    if (!t || subbed.has(t.topic)) continue;
    subbed.add(t.topic);
    console.log(`  订阅: ${t.topic}`);
    conn.subscribe(t.topic, (msg) => {
      hits++;
      const r = parseExpTrajMarkers(msg);
      bestPts = Math.max(bestPts, r.points.length / 3);
      if (hits <= 6) {
        console.log(
          `    #${hits} markers=${r.markerCount} 被过滤=${r.skipped} → 折线点=${r.points.length / 3} color=${r.color ?? "—"}`,
        );
      }
    });
  }
};

setInterval(() => trySubscribe(connAny.topics), 1000);

// 规划器只在「真的发生 replan」时才会发 ExpTraj，光订阅收不到东西。
// 这里主动发一个目标点触发规划。
// 注意：绕开 /gcs/go_to_planner 直接发 —— 网关在未解锁时会拒绝规划目标，
// 用网关验证不了这条链路。
setTimeout(() => {
  if (subbed.size === 0) {
    console.log("  8s 内没订阅上任何 ExpTraj 话题，放弃发点");
    return;
  }
  console.log("  发目标点触发 replan（连发 3 次，避开 advertise 的时序问题）…");
  const send = () => {
    try {
      conn.publish("/move_base_simple/goal", "geometry_msgs/PoseStamped", {
        header: {
          seq: 0,
          stamp: { sec: Math.floor(Date.now() / 1000), nsec: (Date.now() % 1000) * 1e6 },
          frame_id: "map",
        },
        pose: {
          position: { x: -1.0, y: 1.2, z: 0.9 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      });
    } catch (e) {
      console.log(`  发点失败: ${String(e)}`);
    }
  };
  // 第一次 publish 会顺手做 advertise，消息通常丢 —— 多发几次才稳
  send();
  setTimeout(send, 700);
  setTimeout(send, 1400);
}, 8000);

setTimeout(() => {
  console.log(`\n═══ 汇总 ═══`);
  console.log(`  已订阅话题: ${[...subbed].join(", ") || "（无）"}`);
  console.log(`  收到消息: ${hits} 条`);
  console.log(`  解析出的最大折线点数: ${bestPts}`);
  console.log(
    bestPts >= 2
      ? "  ✅ 前端解析层没有问题：拿到就能画出来"
      : hits > 0
        ? "  ⚠ 收到消息但解析不出点 —— 看上面的「被过滤」计数，大概率是类型没覆盖"
        : "  ❌ 一条消息都没收到：话题选错 / 规划器没在跑 / 没触发 replan",
  );
  process.exit(0);
}, 30000);
