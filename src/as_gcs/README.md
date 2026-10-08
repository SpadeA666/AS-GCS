# as_gcs

地面站的 ROS 侧。当前只有消息定义与骨架，**尚未编译**。

- `msg/` `srv/` `action/` —— 地面站与网关之间的接口契约
- 网关（`as_controller` 包内的 `gcs_gateway`）与设备管理（`device_manager`）见 PLAN-v2.md

设计要点：
- 打点分两类：`KIND_PLANNER` 走规划器，`KIND_PX4` 直接给 PX4 位置控制
- `SetActuator` 用 -1 表示保持原值，因为 `pwmControl` 三通道必须一起下发
- 安全区（`SetGeofence`）的校验必须在 ROS 侧，前端只做可视化
