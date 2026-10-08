#!/bin/bash
# restart_all.sh —— 一键「干净重启」仿真 + 地面站
#
# 为什么顺序不能反（这是之前反复踩的坑）：
#   地面站的 foxglove_bridge / gcs_gateway 都挂在 ROS master 上。
#   而 master 是随仿真一起起来的。如果先起地面站、后起仿真，
#   仿真会把 master 换成一茬新的，地面站那两个进程还占着端口、
#   注册表却已经没了 —— 前端于是"连上了却拿不到 /gcs/ 服务"。
#   所以永远是：先仿真 → 等 master 就位 → 再地面站。
#
# 用法：bash restart_all.sh
# 只重启地面站（仿真不动）：bash start-dev.sh

set -u
UI_DIR=/home/spadea/catkin_ws/src/as_gcs/ui

echo "════════ 1/4 停止旧仿真与地面站组件 ════════"
bash "$UI_DIR/cleanup_all.sh"

echo
echo "════════ 2/4 启动仿真（各节点独立会话）════════"
bash "$UI_DIR/sim_stable.sh"

echo
echo "════════ 3/4 等 ROS master 与话题就位 ════════"
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spadea/catkin_ws/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI="http://localhost:11311"

ok=0
for i in $(seq 1 24); do   # 最多等 120 秒
  sleep 5
  n=$(timeout 8 rostopic list 2>/dev/null | wc -l)
  printf "  +%3ds  话题 %s\n" "$((i*5))" "$n"
  if [ "$n" -gt 120 ]; then ok=1; break; fi
done

if [ "$ok" -ne 1 ]; then
  echo
  echo "  ✗ 等待超时。查日志："
  echo "      ls -la /tmp/sim_*.log"
  echo "      tail -30 /tmp/sim_px4.log"
  exit 1
fi
echo "  ✓ 仿真就绪"

echo
echo "════════ 4/4 启动地面站（bridge + gateway + 前端 + 守护）════════"
bash "$UI_DIR/start-dev.sh"

echo
echo "════════ 全部完成 ════════"
echo "  浏览器： http://localhost:5173/"
echo "  看护：   gcs_watchdog.sh 会自动兜住 master 换代导致的失联"
