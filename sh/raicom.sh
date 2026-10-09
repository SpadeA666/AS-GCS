#!/bin/bash
# raicom.sh — 依次启动 PX4 indoor3 / FAST-LIO Mid360 建图 / map->odom 静态TF
#              / as_navigation octomap / lio_to_mavros / SUPER planner / spadea rviz
# 每条命令间隔 2s，全部在后台运行；Ctrl+C 或脚本退出时自动清理所有子进程

set -e

# Keep direct execution reproducible from a clean terminal.  PX4 is a ROS
# package overlay for SITL, not part of catkin_ws/devel by itself.
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
source "$HOME/PX4_Firmware/Tools/setup_gazebo.bash" \
  "$HOME/PX4_Firmware/" "$HOME/PX4_Firmware/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4_Firmware:$HOME/PX4_Firmware/Tools/sitl_gazebo"

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

echo "[raicom] (2/7) roslaunch faster_lio mapping_mid360.launch"
roslaunch faster_lio mapping_mid360.launch &
PIDS+=($!)
sleep 2

echo "[raicom] (3/7) rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
PIDS+=($!)
sleep 1

echo "[raicom] (3.5/7) rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map world"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map world &
PIDS+=($!)
sleep 1

# echo "[raicom] (4/7) roslaunch as_navigation octomap.launch"
# roslaunch as_navigation octomap.launch &
# PIDS+=($!)
# sleep 2

echo "[raicom] (5/7) roslaunch lio_to_mavros lio_to_mavros.launch"
roslaunch lio_to_mavros lio_to_mavros.launch &
PIDS+=($!)
sleep 2

echo "[raicom] (5.5/7) python3 super_data_fix (GT真值模式: MAVROS位姿+原始点云, 绕过LIO z退化)"
# 独立subshell: source ROS环境(rospy) + devel PYTHONPATH(CustomMsg) + 系统python3,
# 不污染主环境(避免 ROS_PACKAGE_PATH 被 setup.bash 重置导致 roslaunch px4 找不到包)
# 日志落盘 /tmp/super_data_fix.log 便于排查
nohup bash -c 'source /opt/ros/noetic/setup.bash; \
export PYTHONPATH=/home/spadea/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH; \
exec /usr/bin/python3 /home/spadea/catkin_ws/scripts/super_data_fix.py _use_ground_truth:=true' \
  > /tmp/super_data_fix.log 2>&1 &
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

# echo "[raicom] (6.5/7) roslaunch yolov11_ros yolo_v11_double.launch (单目下视 + D435i前视 yolo11)"
# roslaunch yolov11_ros yolo_v11_double.launch &
# PIDS+=($!)
# sleep 2

# echo "[raicom] (6.6/7) rqt_image_view 显示 yolo 检测图1 (/yolov11/camera_1/detection_image)"
# rqt_image_view /yolov11/camera_1/detection_image &
# PIDS+=($!)
# sleep 1


# echo "[raicom] (7/7) roslaunch spadea ego.launch"
# roslaunch spadea ego.launch
# PIDS+=($!)

# roslaunch ommpc_bridge super_bridge.launch &
# PIDS+=($!)
# sleep 1

# roslaunch ommpc_controller px4_example.launch &
# PIDS+=($!)
# sleep 1

# echo "[raicom] (7/7) roslaunch spadea super.launch"
# roslaunch spadea super.launch
# PIDS+=($!)

# roslaunch exploration_manager exploration.launch 
# PIDS+=($!)

# 4) 保持脚本运行，直到后台的 roslaunch 节点退出
wait
