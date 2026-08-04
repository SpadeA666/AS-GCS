#!/bin/bash
# raicom.sh — 依次启动 PX4 indoor3 / FAST-LIO Mid360 建图 / map->odom 静态TF
#              / as_navigation octomap / lio_to_mavros / SUPER planner / spadea rviz
# 每条命令间隔 2s，全部在后台运行；Ctrl+C 或脚本退出时自动清理所有子进程

set -e

# 2) 记录子进程 PID，退出时统一清理
PIDS=()
cleanup() {
    echo ""
    echo "[raicom] 正在终止已启动的节点..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# 3) 依次启动，每条间隔 2s
echo "[raicom] (1/7) roslaunch px4 raicom.launch"
roslaunch px4 raicom.launch &
PIDS+=($!)
sleep 2

echo "[raicom] (2/7) roslaunch fast_lio mapping_mid360.launch"
roslaunch fast_lio mapping_mid360.launch &
PIDS+=($!)
sleep 2

echo "[raicom] (3/7) rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
PIDS+=($!)
sleep 1

# echo "[raicom] (3.5/7) rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map world"
# rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map world &
# PIDS+=($!)
# sleep 1

echo "[raicom] (4/7) roslaunch as_navigation octomap.launch"
roslaunch as_navigation octomap.launch &
PIDS+=($!)
sleep 2

echo "[raicom] (5/7) roslaunch lio_to_mavros lio_to_mavros.launch"
roslaunch lio_to_mavros lio_to_mavros.launch &
PIDS+=($!)
sleep 2

echo "[raicom] (5.5/7) python3 super_data_fix (LIO XY + MAVROS z, 修复LIO z退化)"
python3 /home/spadea/catkin_ws/scripts/super_data_fix.py &
PIDS+=($!)
sleep 2

# echo "[raicom] (6/7) roslaunch ego_planner single_run_in_mid.launch"
# roslaunch ego_planner single_run_in_mid.launch &
# PIDS+=($!)
# sleep 2

echo "[raicom] (6/7) roslaunch mission_planner click_demo.launch"
roslaunch mission_planner click_demo.launch &
PIDS+=($!)
sleep 2
# rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link drone &
# PIDS+=($!)
# sleep 2

echo "[raicom] (6.5/7) roslaunch yolov11_ros yolo_v11_double.launch (单目下视 + D435i前视 yolo11)"
roslaunch yolov11_ros yolo_v11_double.launch &
PIDS+=($!)
sleep 2

echo "[raicom] (6.6/7) rqt_image_view 显示 yolo 检测图1 (/yolov11/camera_1/detection_image)"
rqt_image_view /yolov11/camera_1/detection_image &
PIDS+=($!)
sleep 1

# echo "[raicom] (7/7) roslaunch spadea ego.launch"
# roslaunch spadea ego.launch
# PIDS+=($!)

echo "[raicom] (7/7) roslaunch spadea super.launch"
roslaunch spadea super.launch
PIDS+=($!)

# 4) 保持脚本运行，直到后台的 roslaunch 节点退出
wait
