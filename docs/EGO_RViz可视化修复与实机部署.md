# EGO-Planner RViz 可视化修复与实机部署文档

> 本文档记录 2026-08-04 对 EGO-Planner 在 RViz 中路径可视化(红点、航点球)缺失问题的诊断、修复,以及如何把同样改动部署到实机板载计算机。
>
> 适用于:仿真环境(当前)+ 实机板载计算机(同一份源码)。

---

## 1. 背景与现象

- 在 RViz 中给 EGO-Planner 打点后,**看不到路径上的红色小点**,也看不到"中途追踪点"(航点球)。
- 对比:SUPER 能看到路径和中途追踪点(`/fsm_node/visualization/goal`、`/fsm_node/fsm/path`),EGO 看不到。
- RViz 显示的消息 `IN SWARM MODE, REFINE DISABLED!` 只是 EGO-Planner 打印一次的日志(`planner_manager.cpp:297`),**不是问题根源**;`plan_success=1` 说明规划本身成功。

## 2. 根因分析

EGO-Planner 的可视化 marker 其实**一直在发布**,问题在:

1. **红色小点(SPHERE_LIST)被注释**:
   `traj_utils/src/planning_visualization.cpp` 的 `displayMarkerList()` 里,球点发布被 `//` 注释,只发布细的 `LINE_STRIP`,所以路径只是细线,没有点。
2. **打点后的航点球被注释**:
   `plan_manage/src/ego_replan_fsm.cpp` 的 `planNextWaypoint()` 里 `displayGoalPoint(next_wp, ...)` 被 `//` 注释,新目标不出球。
3. **RViz 配置里大部分 display 关闭**:
   `spadea/rviz/ego.rviz` 只开了 `optimal_traj`,`goal_point` / `global_list` / `init_list` 都是 `Enabled: false`。
4. **打点工具是 2D**:
   ego.rviz 用的是 `rviz/SetGoal`(2D Nav Goal),SUPER 用的是 `rviz_plugins/Goal3DTool`(3D,可设 z / yaw)。

> 补充:marker 话题名本身**没有配错**。EGO 规划节点用私有 NodeHandle `nh("~")`,所有可视化发布在 `/drone_0_ego_planner_node/` 下,和 ego.rviz 里写的一致;marker 的 `frame_id` 硬编码为 `map`,与 rviz 固定帧一致。

## 3. 改动明细

### 3.1 源码改动(需要重新编译)

| # | 文件 | 改动 | 作用 |
|---|------|------|------|
| 1 | `ego_planner/traj_utils/src/planning_visualization.cpp` `displayMarkerList()` | 取消注释 `if (show_sphere) sphere.points.push_back(pt);` 和 `if (show_sphere) pub.publish(sphere);` | 最优轨迹显示**红色球点**(`optimal_list` 用红色 `(1,0,0,1)`),init 蓝点、global 青点也恢复 |
| 2 | `ego_planner/plan_manage/src/ego_replan_fsm.cpp` `planNextWaypoint()` | 取消注释 `visualization_->displayGoalPoint(next_wp, ...)` | 打点后(含 3D 目标)显示**航点球** |

### 3.2 RViz 配置改动(无需编译,重开 rviz 生效)

文件:`spadea/rviz/ego.rviz`

- **Tools 段**:`rviz/SetGoal` → `rviz_plugins/Goal3DTool`(3D 打点,发布话题仍是 `/move_base_simple/goal`)。
- **Planning 组 Display 状态**(当前最终状态):

| Display | 话题 | 状态 |
|---|---|---|
| goal_point(航点球) | `/drone_0_ego_planner_node/goal_point` | ✅ 开 |
| optimal_traj(最优轨迹红点/线) | `/drone_0_ego_planner_node/optimal_list` | ✅ 开 |
| InitTraj(初始轨迹) | `/drone_0_ego_planner_node/init_list` | ✅ 开 |
| global_path(全局路径) | `/drone_0_ego_planner_node/global_list` | ⬜ 关(可选,按需开) |
| AStar(A* 搜索路径) | `/drone_0_ego_planner_node/a_star_list` | ⬜ 关(噪声大,建议保持关) |

### 3.3 依赖:3D 打点插件

`rviz_plugins` 包(含 `Goal3DTool`)在 `ego_planner/Utils/rviz_plugins` 下,必须编译出 `librviz_plugins.so` 且被 RViz 加载,3D 工具才出现在工具栏。

## 4. 仿真环境操作

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash

# 重新编译(改了两处源码 + 插件)
catkin build traj_utils ego_planner rviz_plugins
source devel/setup.bash

# 重启 ego 规划节点(roslaunch 你的 ego launch,例如 single_run_in_mid.launch)
# 重新打开 rviz(rviz 配置改动要重开才加载)
roslaunch spadea ego.launch
```

验证:

```bash
rostopic list | grep drone_0_ego_planner_node            # 话题存在
rostopic hz /drone_0_ego_planner_node/optimal_list       # 在发布
rostopic echo -n1 /drone_0_ego_planner_node/optimal_list # 内容/点数/frame_id
```

## 5. 实机板载计算机部署

实机用的是**同一份源码**,把改动同步过去 + 重编译 + 重启即可。

### 5.1 同步 3 个文件

```bash
scp <仿真机>:~/catkin_ws/src/ego_planner/traj_utils/src/planning_visualization.cpp  板载:~/catkin_ws/src/ego_planner/traj_utils/src/
scp <仿真机>:~/catkin_ws/src/ego_planner/plan_manage/src/ego_replan_fsm.cpp         板载:~/catkin_ws/src/ego_planner/plan_manage/src/
scp <仿真机>:~/catkin_ws/src/spadea/rviz/ego.rviz                                   板载:~/catkin_ws/src/spadea/rviz/
```

> 若仿真与板载共用 git 仓库,直接 pull + rebase 更省事。

### 5.2 编译

```bash
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash
catkin build rviz_plugins traj_utils ego_planner
source devel/setup.bash
```

### 5.3 重启节点 + 重开 rviz

- 重启 EGO 规划节点(重新 roslaunch 实机的 ego launch)。
- 重新打开 rviz(rviz 配置文件改动必须重开才加载)。

## 6. 命名空间 / 帧检查清单(实机最容易踩的坑)

| 项 | 仿真值 | 实机需确认 |
|---|---|---|
| 规划节点名 | `drone_0_ego_planner_node` | 实机 drone_id / 节点名若不同,rviz 里所有 `/drone_0_ego_planner_node/*` 话题要改成实机实际的 |
| rviz Fixed Frame | `map` | 实机 TF 里**必须有 `map` 帧**。marker 在规划器源码里硬编码 `frame_id="map"`,没有 map 帧就显示不出来;若实机用 `odom`/`world`,把 rviz 固定帧改过去,并保证 TF 完整 |
| Goal3DTool 话题 | `/move_base_simple/goal` | 与 ego 的 goal 订阅话题一致(默认一致) |
| `rviz_plugins` 插件 | 已编译 | 实机必须 `catkin build rviz_plugins` 并 `source devel/setup.bash`,否则工具栏无 3D Goal |

排查命令:

```bash
rostopic list | grep ego_planner_node
rosrun tf tf_echo map base_link        # 确认 map 帧存在且可达
```

## 7. 常见问题排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 打点后什么都没有 | ego 节点没重启(旧二进制) | 重启 ego 节点;确认 `/drone_0_ego_planner_node/optimal_list` 有数据 |
| 只有线没有红点 | `planning_visualization.cpp` 未重新编译 | 重编译 `traj_utils` |
| 工具栏没有 3D Goal | `rviz_plugins` 未编译/未 source | `catkin build rviz_plugins` + `source devel/setup.bash` |
| marker 有数据但显示不出来 | TF 缺 `map` 帧 | 补 `map` 帧;或把 rviz 固定帧改成存在的帧 |

## 8. 相关背景:EGO 乱飞问题(此前诊断,尚未全部解决)

本次可视化修复与 ego 乱飞问题是两件事,但同一环境下需要注意:

- **180° 旋转**:`as_navigation` 的 `ego_planner_pos_cmd_cb` 曾硬编码 180° XY 旋转,已改为参数 `ego_rotate_180` 默认 `false`(`lio_to_mavros` 桥 `lidar_yaw_offset=0` 不旋转 → LIO 系 = PX4 local)。仿真中已默认关闭。
- **z 退化**:`single_run_in_mid.launch` 当前 `odom_topic=/Odometry_highrate`(原始 LIO)。LIO z 在仿真里漂移,会导致 ego 高度不稳。SUPER 用的是 `super_data_fix.py` 的 `/super_odom`(LIO XY/yaw + MAVROS z)+ `/super_cloud`。若要 ego 高度稳定,需把 ego 的 `odom_topic`/`cloud_topic` 指向 `/super_odom`/`/super_cloud`(需 `super_data_fix` 在跑)。
- 快速自检坐标系:悬停时对比 `/Odometry_highrate` 与 `/iris_0/mavros/local_position/odom` 的 `position.x/y` —— 符号一致 → 不旋转;反号 → 需要 180°(`ego_rotate_180=true`)。

---

*文档维护人:项目组 / 最后更新:2026-08-04*
