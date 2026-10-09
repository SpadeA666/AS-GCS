#!/bin/bash
# 稳定启动本地仿真：启动内容与 sh/raicom.sh 一致，但去掉了会让整个仿真级联崩掉的设计。
#
# 用法：
#   bash sim_stable.sh                # 默认 indoor3
#   bash sim_stable.sh raicom         # 指定场景
#   bash sim_stable.sh --list         # 列出可用场景
#
# 为什么不用 raicom.sh：
#   它开头是 `set -e`，结尾是裸的 `wait`，且 `trap cleanup INT TERM EXIT`。
#   `wait` 一旦返回某个子进程的退出码（任一 launch 抖动、退出、被杀），
#   `set -e` 就会立刻结束脚本，而 EXIT trap 随即把【所有】 ROS 节点一起杀掉。
#   结果：任何一条 launch 出问题，整个仿真（Gazebo/PX4/LIO/规划器）全部消失，
#   连带 foxglove_bridge / gcs_gateway 变成孤儿，地面站就“连不上”。
#
#   本脚本让每条 launch 各自 setsid nohup 独立成会话，父脚本拉完就退出：
#   谁挂掉都不影响别人，也没有人会来“统一清理”。

# ── 场景参数：支持位置参数，也兼容 SIM_WORLD=xxx 环境变量 ──
WORLDS="indoor1 indoor2 indoor3 indoor4 indoor5 outdoor1 outdoor2 outdoor3 outdoor4 outdoor2_precision_landing raicom rk zhihang1 zhihang2"

case "$1" in
  --list|-l)
    echo "可用场景（来自 PX4_Firmware/launch/*.launch，已校验 world 文件存在）："
    for w in $WORLDS; do echo "  $w"; done
    echo
    echo "用法：bash sim_stable.sh <场景>"
    exit 0
    ;;
  -h|--help)
    echo "用法：bash sim_stable.sh [场景]      # 默认 indoor3"
    echo "      bash sim_stable.sh --list      # 列出可用场景"
    exit 0
    ;;
  "") ;;
  *)
    if ! echo " $WORLDS " | grep -q " $1 "; then
      echo "未知场景：$1"
      echo "可用：$WORLDS"
      exit 1
    fi
    export SIM_WORLD="$1"
    ;;
esac

export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/noetic/setup.bash
source /home/spadea/catkin_ws/devel/setup.bash
source /home/spadea/PX4_Firmware/Tools/setup_gazebo.bash \
       /home/spadea/PX4_Firmware/ /home/spadea/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/spadea/PX4_Firmware"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/spadea/PX4_Firmware/Tools/sitl_gazebo"

# FASTER-LIO 的 conan 运行时依赖：两个 gflags 的符号命名空间不同，不能混用，
# 所以 conan 的 glog 目录和 gflags 目录都必须进 LD_LIBRARY_PATH。
CONAN_GLOG=$(dirname $(find "$HOME/.conan/data/glog" -name "libglog.so.1" 2>/dev/null | head -1))
CONAN_GFLAGS=$(dirname $(find "$HOME/.conan/data/gflags" -name "libgflags_nothreads.so.2.2" 2>/dev/null | head -1))
if [ -n "$CONAN_GLOG" ] && [ -n "$CONAN_GFLAGS" ]; then
  export LD_LIBRARY_PATH="$CONAN_GLOG:$CONAN_GFLAGS:$LD_LIBRARY_PATH"
else
  echo "警告：未找到 conan 的 glog/gflags，FASTER-LIO 可能起不来"
fi

run() {  # run <名字> <命令...>
  local name="$1"; shift
  setsid nohup "$@" > "/tmp/sim_${name}.log" 2>&1 < /dev/null &
  sleep 0.3
  echo "  → $(printf '%-14s' "$name") pid $!   日志 /tmp/sim_${name}.log"
}

# 场景可切换。默认 indoor3（大场景，190KB world）；想回原来的小场景：
#   SIM_WORLD=raicom bash sim_stable.sh
# 可选值 = PX4_Firmware/launch/ 下现成的 <名字>.launch：
#   indoor1..indoor7 / outdoor1..outdoor4 / warehouse / robocup_indoor / baylands ...
# 这些 launch 与 raicom.launch 同模板，只是 world 不同，iris+Mid360 配置一致。
SIM_WORLD="${SIM_WORLD:-indoor3}"

echo "═══ 启动仿真（场景 ${SIM_WORLD}，各节点独立会话，互不连带）═══"

# 规划器状态文件必须清掉。
# 它描述的是【运行时】状态，而仿真重启后默认跑的永远是 SUPER。
# 不清的话：上次切到 EGO 时留下的 "ready ego" 会被 gcs_gateway 读到
# （网关启动时会跟随这个文件，见 pollPlannerSwitch），于是出现
# 「实际在跑 SUPER，网关却按 EGO 的控制律发指令」这种错位 ——
# 现象就是切换规划器之后行为对不上，很难查。
rm -f /tmp/planner_switch.state

echo "  （已重置规划器状态：→ super）"
run px4          roslaunch px4 "${SIM_WORLD}.launch"
sleep 8
run fasterlio    roslaunch faster_lio mapping_mid360.launch
sleep 3
run tf_map_odom  rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
run tf_map_world rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map world
sleep 2
run lio2mavros   roslaunch lio_to_mavros lio_to_mavros.launch
sleep 2
run datafix bash -c 'source /opt/ros/noetic/setup.bash; export PYTHONPATH=/home/spadea/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH; exec /usr/bin/python3 /home/spadea/catkin_ws/scripts/super_data_fix.py _use_ground_truth:=true'
sleep 2
run super        roslaunch mission_planner click_demo.launch

echo
echo "═══ 已全部拉起 ═══"
echo "  foxglove_bridge / gcs_gateway 由 gcs_watchdog.sh 自动接管，无需手动起"
echo "  等 20~30 秒让 Gazebo / PX4 / LIO 就位后再看话题"
