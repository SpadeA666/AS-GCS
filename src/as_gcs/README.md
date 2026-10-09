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
| `launch-gcs.sh` | 桌面快捷方式的实际入口（= `start-dev.sh` + 开浏览器） |
| `install-desktop-icon.sh` | 把快捷方式装到桌面/应用菜单（自动按本机路径改写模板） |
| `as-gcs.desktop` | 快捷方式模板，`__UI_DIR__` 占位由上面脚本替换 |

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

自己装一份：`bash ~/catkin_ws/src/as_gcs/ui/install-desktop-icon.sh`
（别人 clone 后没有这个 .desktop，需要跑一次安装脚本）。

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

## 遥控器接管（joy_rc_bridge）

**为什么需要它**：QGC 自带的 joystick 只发 MAVLINK `MANUAL_CONTROL`，走
`manual_control_input → manual_control_setpoint` 这条链路，**永远不会产生
`input_rc` / `rc_channels` / `manual_control_switches`**。结果就是
`RC_MAP_KILL_SW`、`RC_MAP_ARM_SW`、`RC_MAP_FLTMODE` 这些开关在仿真里全是死的
（QGC v4.2 二进制里也没有任何 RC override 代码，且不支持 passthrough）。

`scripts/joy_rc_bridge.py` 改发 `RC_CHANNELS_OVERRIDE`：

```
TX12 ──USB──> /dev/input/js0 ──> joy_rc_bridge ──> RC_CHANNELS_OVERRIDE
  ──> PX4 input_rc ──> rc_update ──> rc_channels + manual_control_switches
  ──> ManualControl ──> action_request ──> Commander
```

这条链路与真机的物理接收机**完全同构**，所以开关行为、参数都能原样搬到真机。

### 用法

```bash
bash scripts/joy_bridge_ctl.sh start    # 起桥（后台常驻）
bash scripts/joy_bridge_ctl.sh status
bash scripts/joy_bridge_ctl.sh log
bash scripts/joy_bridge_ctl.sh stop
```

**跑桥时必须先在 QGC 里关掉 joystick**（Vehicle Setup → Joystick → 取消 Enable），
否则 QGC 的 `MANUAL_CONTROL` 和桥的 RC 通道会同时被接受（`COM_RC_IN_MODE=2`），
摇杆互相打架。

**⚠️ 重启仿真后遥控器没反应？** 那是 master 换代把桥变成了孤儿：
桥进程还活着，但 rospy 连接已失效，`rc/override` 上不再有发布者。
这**不是**桥的 bug，也不是参数丢了 —— PX4 参数由 `-w sitl_iris_0` 持久化在工作目录，
重启后仍在。

现在 `start-dev.sh` 和 `gcs_watchdog.sh` 都已把桥纳进来（与 bridge/gateway 同级的
“进程 + 注册”双判据），所以 master 换代后会自动救回。手动救：

```bash
bash scripts/joy_bridge_ctl.sh stop && bash scripts/joy_bridge_ctl.sh start
# 或者直接重跑：bash ui/start-dev.sh
```

### 通道映射（对齐实机习惯）

| TX12 | js 编号 | RC 通道 | PX4 参数 |
|---|---|---|---|
| 四摇杆 | Axis 0/1/2/3 | ch1/2/3/4 | `RC_MAP_ROLL/PITCH/THROTTLE/YAW` |
| 开关 **C** | Axis 6 | ch7 | `RC_MAP_FLTMODE` |
| 开关 **F** | Axis 7 | ch8（桥内反向） | `RC_MAP_OFFB_SW` |
| 开关 **E** | Button 0 | ch9 | `RC_MAP_KILL_SW` |
| 开关 **B** | Button 1 | ch10 | `RC_MAP_ARM_SW` |

### 配套 PX4 参数

```
RC_MAP_ROLL=1  RC_MAP_PITCH=2  RC_MAP_THROTTLE=3  RC_MAP_YAW=4
RC_MAP_FLTMODE=7   RC_MAP_OFFB_SW=8
RC_MAP_KILL_SW=9   RC_MAP_ARM_SW=10
COM_FLTMODE1=8     # 自稳 Stabilized（C 低档 -> slot 1）
COM_FLTMODE4=1     # 高度 Altitude  （C 中档 -> slot 4）
COM_FLTMODE6=2     # 定点 Position  （C 高档 -> slot 6）
COM_RC_IN_MODE=2   # 否则 selector 只认 MAVLink 源，会丢掉这些通道
COM_RC_OVERRIDE=3  # AUTO + OFFBOARD 都允许拨杆接管
```

> **注意**：PX4 SITL 里 `RC_MAP_*` 默认全是 `0`（映射**禁用**）。不显式设置的话，
> 即使 RC 通道数据进来了也不会被当成摇杆或开关。

### 已验证（仿真实测）

- **C 开关**：三档 → 模式 `STABILIZED / ALTCTL / POSCTL` 完整循环，重复两轮无误
- **E 开关**：拨到 kill → `system_status` 跃到 `8`（`MAV_STATE_FLIGHT_TERMINATION`，
  即 `manual_lockdown`）；拨回 → 回到 `3`
- **B 开关**：`armed False → True → False` 双向验证
- **F 开关**（offboard）待验：OFFBOARD 需要持续 setpoint 流，
  要等 SUPER / `as_controller` 跑起来才有意义，否则只会走 failsafe

### 附带工具

```bash
# 探测 TX12 每个开关落在哪个 Axis/Button（会打印实时事件 + 汇总）
/usr/bin/python3 scripts/probe_joy.py 60
```
