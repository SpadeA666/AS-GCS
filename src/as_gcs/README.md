# as_gcs —— AS 地面站

无人机 Web 地面站：**观察建图点云 + 打点控制飞行 + 安全区管理 + 目标跟随**。
跨平台（Linux / Windows 浏览器均可），ROS 侧 + 前端都在本包内。

```
浏览器 ──WebSocket(8765)──> foxglove_bridge ──ROS──> 仿真 / PX4（master 11311）
                                  ↑ 订阅
                            gcs_gateway_node        ← 控制面，14 个 /gcs/ 服务
```

## 快速开始

```bash
bash ui/restart_all.sh              # 一键：停旧 → 起仿真 → 等就位 → 起地面站
# 浏览器 http://localhost:5173/
```

### 换场景

```bash
bash ui/restart_all.sh --list       # 先看有哪些场景
bash ui/restart_all.sh indoor3      # 指定场景重启（默认就是 indoor3）
bash ui/restart_all.sh raicom       # 换回竞赛小场景
```

**当前实测可用的场景**（已校验 world 文件存在）：

| 类别 | 场景 |
|---|---|
| 室内 | `indoor1` `indoor2` `indoor3` `indoor4` `indoor5` |
| 室外 | `outdoor1` `outdoor2` `outdoor3` `outdoor4` `outdoor2_precision_landing` |
| 竞赛 | `raicom` |
| 其他 | `rk` `zhihang1` `zhihang2` |

## 脚本速查（都在 `ui/`）

| 脚本 | 用途 |
|---|---|
| `restart_all.sh [场景]` | **全量重启**：停 → 起仿真 → 等 master → 起地面站 |
| `sim_stable.sh [场景]` | 只起仿真（替代 `raicom.sh`，见下） |
| `start-dev.sh` | 只起/自愈地面站（bridge + gateway + 前端 + watchdog） |
| `cleanup_all.sh` | 停掉仿真与地面站组件（保留前端和 watchdog） |
| `gcs_watchdog.sh` | 常驻守护，master 换代后自动救回 bridge/gateway |
| `launch-gcs.sh` | 桌面快捷方式调用（= `start-dev.sh` + 开浏览器） |

**排障探针**（`ui/scripts/`）：

```bash
npx vite-node scripts/probe_connect.ts              # 连接耗时、话题/服务数
npx vite-node scripts/probe_path.ts                 # fsm/path 能否收到、多少点
npx vite-node scripts/probe_exptraj.ts              # ExpTraj（会主动发点触发规划）
npx vite-node scripts/probe_exptraj_throttled.ts 0  # 带节流验证 ExpTraj
```

## 典型使用流程

```
首次开机 / 想换场景  →  bash ui/restart_all.sh <场景>
之后日常            →  双击桌面「AS 地面站」图标  或  bash ui/start-dev.sh
```

**桌面快捷方式**（`~/桌面/as-gcs.desktop`）**不启动仿真**，只拉起地面站并打开浏览器。

## 为什么不用 `raicom.sh` 启动仿真

`sh/raicom.sh` 是 `set -e` + 裸 `wait` + `trap cleanup EXIT` 的组合：

> **任何一条 launch 抖动 → `wait` 返回非零 → 脚本退出 → EXIT trap 把所有节点一起杀光。**

表现就是"仿真莫名其妙整体消失、bridge/gateway 变成孤儿、地面站连不上"。
`sim_stable.sh` 让每条 launch 各自 `setsid` 独立成会话，谁挂都不牵连别人。

## 设计要点

- **打点分两类**：`规划器点` 走 `/gcs/go_to_planner`（EGO/SUPER 规划），
  `PX4 点` 走 `/gcs/go_to_px4`（直接位置控制）。**两类点在 2D 图上形状不同**，防误发。
- **安全区校验在 ROS 侧**（`set_geofence` → 网关 `checkGeofence`），前端只做可视化与预判。
- **`SetActuator` 用 -1 表示保持原值**，因为 `pwmControl` 三通道必须一起下发。
- **Z / Yaw 的控制来源**由 `nav_mode` 决定（`/gcs/set_nav_mode`），
  与"任务目标是什么"是两件事——goal 里带的永远是用户指定的值。

## 已知限制

- **规划器参数与场景强相关**：`super_planner/config/click_smooth_ros1.yaml` 里的
  `virtual_ceil_height: 1.8` / `virtual_ground_height: -0.6` 是照 raicom 小场景调的。
  **大场景（indoor / outdoor）下规划可能 100% 失败**（`Ill corridor`），
  需要重新调这两个值——这是规划器参数问题，不是地面站问题。
- **`src/SUPER` 的 240MB 素材未入库**（模型/点云/动图），换机器后需要自行补齐。
- 前端用 `vite dev`（:5173，HMR）。**不再维护 4173 生产预览**——那套服务的是
  `dist/` 构建产物，改了源码不重新 build 页面就是旧的，极易误判成"改动没生效"。
