#!/bin/bash
# 一次性清理脚本：把重复/残留的仿真与地面站进程全部清掉。
#
# 为什么要写成文件：
#   pkill -f <pattern> 会匹配"完整命令行"。如果直接在 shell 里写
#   pkill -f "gzserver"，那么执行这条命令的 bash 自身命令行里就含
#   "gzserver" 字样，pkill 会把自己也杀掉（表现为命令 exit -1 中断）。
#   放进脚本文件后，脚本进程的 cmdline 只有脚本路径，不含这些模式。

export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash 2>/dev/null

echo "── 1. 残留的仿真主脚本 ──"
pkill -9 -f 'raicom\.sh' 2>/dev/null && echo "  killed raicom.sh" || echo "  none"
sleep 2

echo "── 2. ROS 核心 ──"
pkill -9 -f 'roslaunch'      2>/dev/null && echo "  killed roslaunch" || echo "  none"
pkill -9 -f 'rosmaster'      2>/dev/null && echo "  killed rosmaster" || echo "  none"
pkill -9 -f 'rosout'         2>/dev/null
sleep 2

echo "── 3. Gazebo / PX4 ──"
pkill -9 -f 'gzserver'       2>/dev/null && echo "  killed gzserver" || echo "  none"
pkill -9 -f 'gzclient'       2>/dev/null && echo "  killed gzclient" || echo "  none"
pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null && echo "  killed px4" || echo "  none"
pkill -9 -f 'sitl_gazebo'    2>/dev/null
sleep 2

echo "── 4. 导航 / 定位 / 规划 ──"
for pat in 'faster_lio' 'laserMapping' 'lio_to_mavros' 'super_data_fix' 'mission_planner' 'fsm_node' 'asnav' 'ego_planner' 'super_planner' 'mavros'; do
  pkill -9 -f "$pat" 2>/dev/null && echo "  killed $pat"
done
sleep 2

echo "── 5. 地面站组件（watchdog 会在 master 回来后重建）──"
pkill -9 -f 'foxglove_bridge'         2>/dev/null && echo "  killed foxglove_bridge" || echo "  none"
pkill -9 -f 'foxglove_nodelet_manager' 2>/dev/null
pkill -9 -f 'gcs_gateway_node'        2>/dev/null && echo "  killed gcs_gateway" || echo "  none"
# watchdog 本身保留（下面会确认）
sleep 2

# 注意：用 pgrep -f 而不是 ps | grep。
# ps 跟 grep 都会把“执行 grep 的进程自己”数进去（它的命令行里也有那个模式），
# pgrep 则会自动排除自身，数字才是准的。
count() { pgrep -fc "$1" 2>/dev/null || echo 0; }

echo "── 清理结果 ──"
echo "  raicom.sh   : $(count 'raicom\.sh')"
echo "  roslaunch   : $(count 'roslaunch')"
echo "  rosmaster   : $(count 'rosmaster')"
echo "  gzserver    : $(count 'gzserver')"
echo "  px4         : $(count 'px4_sitl_default/bin/px4')"
echo "  gateway     : $(count 'gcs_gateway_node')"
echo "  watchdog    : $(count 'gcs_watchdog')"
echo "  11311       : $(ss -tln 2>/dev/null | grep -q ':11311 ' && echo '仍占用' || echo '已释放')"
echo "  8765        : $(ss -tln 2>/dev/null | grep -q ':8765 ' && echo '仍占用' || echo '已释放')"
echo "  5173        : $(ss -tln 2>/dev/null | grep -q ':5173 ' && echo '仍在跑' || echo '已停')"
