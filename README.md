# AS-GCS

**无人机自主飞行系统**：PX4 Offboard 控制器 + Web 地面站。

```
浏览器 ──WebSocket──> foxglove_bridge ──ROS──> 仿真 / PX4
                          ↑
                    gcs_gateway_node（控制面）
```

## 包含什么

| 组件 | 路径 | 说明 |
|---|---|---|
| **控制器** | `src/as_controller` | PX4 Offboard 接口封装 + 状态机：起飞、定点、规划器导航、视觉跟踪、投放机构 |
| **地面站** | `src/as_gcs` | 浏览器访问的 Web 地面站：点云建图、打点飞行、安全区、目标跟随、遥控器接管 |

规划器 **SUPER**（主）/ **EGO-Planner**（备选）；定位 **FAST-LIO**；雷达 **Livox Mid360**。

> 本仓库只含这两个核心包。第三方依赖（mavros、foxglove_bridge、faster-lio、Livox 驱动、
> SUPER 等）需要自行准备 —— 完整清单与来源见 **[docs/REPRODUCE.md](docs/REPRODUCE.md)**。

## 快速开始

```bash
# 1. 环境自检（不改动任何东西，先看看缺什么）
bash scripts/install_all.sh --check-only

# 2. 一键安装（装依赖 → 配置 catkin → 编译 → 验证）
bash scripts/install_all.sh

# 3. 起仿真 + 地面站
bash src/as_gcs/ui/restart_all.sh raicom
# 浏览器打开 http://localhost:5173/
```

## 环境要求（摘要）

- **Ubuntu 20.04** + **ROS Noetic**
- **PX4 1.13.2**（跑仿真需要）
- **遥控器：RadioMaster TX12**（OpenTX / EdgeTX，USB Joystick 模式）
  - 换其他型号必须先重新探测通道映射，见复现指南 9.4 节

完整依赖清单见 **[docs/REPRODUCE.md](docs/REPRODUCE.md)** 第二、三节。

## 文档导航

| 文档 | 内容 |
|---|---|
| **[docs/REPRODUCE.md](docs/REPRODUCE.md)** | **复现指南**：环境依赖、一键安装、地面站用法、控制器接口、遥控器配置、给 agent 的说明 |
| [BUILD_GUIDE.md](BUILD_GUIDE.md) | 编译排障：9 个已知编译问题的完整记录 |
| [src/as_gcs/README.md](src/as_gcs/README.md) | 地面站使用、场景切换、排障手册 |

## 用 AI Agent 帮忙配置？

先读复现指南的 **[第十节](docs/REPRODUCE.md#十给-ai-agent-的说明)**。开工前必须确认三件事：

1. **先跑环境自检** —— `bash scripts/install_all.sh --check-only`
2. **问清遥控器型号** —— 非 TX12 必须重新探测通道，否则开关会错位
3. **确认工作空间路径** —— 脚本默认 `~/catkin_ws`，不同则先改路径变量

## 安全提示

- **不要让 agent 主动触发起飞** —— 地面站的起飞按钮会真的解锁，`takeoff()` 在 `main()` 里就执行
- 重启网关前确认 `armed: False` —— 它是 OFFBOARD 的 setpoint 源，飞着时重启会触发 PX4 failsafe
