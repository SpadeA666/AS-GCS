# EGO-Planner 定制修改说明

> 针对 RAICOM 仿真场景的两处定制改动，解决打点目标高度失控与穿越未建图盲区的问题。
> 对应仓库：`SpadeA666/drone-simulation-workspace`

---

## 改动一：打点目标高度强制覆盖（fixed_goal_height）

### 问题

RViz 的 "3D Nav Goal" 工具（EGO 的 `rviz_plugins` 插件）点击时，鼠标射线投影到 Fixed Frame 的 z=0 平面，**发布的 PoseStamped 的 z 恒为 0**。EGO 的 `waypointCallback` 直接使用 RViz 传来的 z 作为目标高度，导致：

- 打点目标高度 = 0（地面）→ 飞机一路飞向地面（越飞越低）
- EGO 仅有的保护是 `z < -0.1 return`（防止打到地底），拦不住 z=0 的目标

SUPER 规划器不存在此问题——它的 FSM 里有 `click_height` 参数，收到打点后**无条件把目标高度覆盖为固定值**。本改动为 EGO 移植了同样的机制。

### 改动内容

| 文件 | 改动 |
|---|---|
| `plan_manage/include/plan_manage/ego_replan_fsm.h` | 新增成员 `double fixed_goal_height_ = -10.0;` |
| `plan_manage/src/ego_replan_fsm.cpp` | ① `init` 读取参数 `fsm/fixed_goal_height`；② `waypointCallback` 中 `fixed_goal_height_ > -5.0` 时强制 `end_wp.z() = fixed_goal_height_` |
| `plan_manage/launch/advanced_param.xml` | 新增 `<param name="fsm/fixed_goal_height" value="0.6" type="double"/>` |
| `plan_manage/launch/advanced_param_xtdrone.xml` | 同上 |

核心代码（`ego_replan_fsm.cpp`）：

```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
if (fixed_goal_height_ > -5.0)   // 仿 SUPER click_height, 覆盖打点目标高度, 防止越飞越低
{
  end_wp.z() = fixed_goal_height_;
  cout << "[zfix] Goal height overridden to " << fixed_goal_height_ << endl;
}
planNextWaypoint(end_wp);
```

### 用法

- 调高度：改 launch 里 `fsm/fixed_goal_height` 的值（无需重编译，重启节点生效）
- 关闭覆盖（用 RViz 原始 z）：把值设为 `-10`（或删除该 param，默认即 -10）
- 验证：启动后打点，终端打印 `[zfix] Goal height overridden to 0.6`

---

## 改动二：未知区域按障碍处理（unknown_as_obstacle）

### 问题

EGO-planner 的地图格子初始化时未知值 `clamp_min_log_ - unknown_flag_` **低于障碍判定阈值** `min_occupancy_log_`，即**未观测区域 = 自由空间，A\* 可直接穿越**。

RAICOM 场景实测：穿孔障碍物挡住雷达对柱子下方的扫描 → 起飞后柱子下方从未建图（未知）→ EGO 认为下方无障碍 → **规划路径直接从柱子中间穿过**。

对比 SUPER：它并非显式保守，而是 `virtual_ground_height` 硬边界（`InfMap::getGridType` 中 `z ≤ ground + 0.3m` 直接判为 OCCUPIED）**凑巧封死了低空**，A\* 无法下探到柱子下方，只能绕行。

本改动让 EGO 获得**显式的未知区域保守策略**，且不牺牲地面附近已知障碍的感知（优于临时抬高 `ground_height` 的做法——那会让 0.5m 以下的真实障碍从地图中消失）。

### 改动内容

| 文件 | 改动 |
|---|---|
| `plan_env/include/plan_env/grid_map.h` | `MappingParameters` 新增成员 `bool unknown_as_obstacle_ = false;` |
| `plan_env/src/grid_map.cpp` | ① 新增辅助函数 `unknownOccupancyValue()`；② 读取参数 `grid_map/unknown_as_obstacle`；③ 7 处 occupancy 初始化/重置改用辅助函数 |
| `plan_manage/launch/advanced_param.xml` | 新增 `<param name="grid_map/unknown_as_obstacle" value="true" type="bool"/>` |
| `plan_manage/launch/advanced_param_xtdrone.xml` | 同上 |

核心代码（`grid_map.cpp`）：

```cpp
// 未知格子的 occupancy 值: 开关开启时视为障碍(>阈值), 关闭时保持 EGO 原始行为(可通行)
inline double unknownOccupancyValue(const MappingParameters &mp) {
  return mp.unknown_as_obstacle_ ? mp.min_occupancy_log_ + 0.1
                                 : mp.clamp_min_log_ - mp.unknown_flag_;
}
```

传导链路（已验证）：未知格子值 > 阈值 → `grid_map.cpp:623` 膨胀生成时标入 inflate buffer → A\* 的 `checkOccupancy`（`getInflateOccupancy`）返回 1 → 直接跳过；同时 ESDF 更新把未知区当障碍 → **轨迹优化同样避让**。前后端两层都保守。

### 用法与副作用

- 开关：launch 里 `grid_map/unknown_as_obstacle`（true = 保守，false = 原始激进行为）
- 副作用：开启后遮挡多的环境会偏保守——墙角、传感器盲区后方的空间被当障碍，路径可能绕远。RAICOM 场地影响小，建议开/关各实测一次权衡
- 与 `ground_height` 的关系：开启本开关后，**`ground_height=0.5` 的临时方案可以撤销**（改回 0 或负值，恢复地面附近完整感知）

---

## 备份文件说明

调试过程中产生的 `.bak_zfix` / `.bak_unk` / `.bak_working` 备份文件**已从版本控制中移除**（工作区保留），并在 `.gitignore` 中忽略 `*.bak_*`，避免误提交。

## 编译方式

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin build plan_env ego_planner   # 两处改动涉及这两个包
```

编译验证：`All 5 packages succeeded`（仅原有格式 warning，与改动无关）。
