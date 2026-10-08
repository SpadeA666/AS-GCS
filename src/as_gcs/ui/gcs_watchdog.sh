#!/bin/bash
# gcs_watchdog.sh —— 地面站组件守护
#
# 解决什么问题：
#   仿真由 sh/raicom.sh 拉起，而 raicom.sh 带 `trap cleanup INT TERM EXIT`，
#   退出时会 kill 它启动的 `roslaunch px4 raicom.launch` —— rosmaster 正是
#   由这个 roslaunch 拉起来的。于是每次 raicom.sh 退出/重启，ROS master 都会
#   换成一茬全新的进程。
#
#   而 foxglove_bridge 和 gcs_gateway 是独立进程：master 换代后它们还活着、
#   端口还占着，但已经和新 master 失联（或者进程被连带打死、master 上留着
#   僵尸注册）。这时前端连得上 bridge 也拿不到 /gcs/* 服务。
#
#   这个守护脚本每隔几秒做一次真实健康检查，失效就自动重启对应组件。
#   健康判据见下面 bridge_up / gw_up —— 两条都要满足，缺一不可。
#
# 用法：
#   bash gcs_watchdog.sh &          # 前台放入后台
#   nohup bash gcs_watchdog.sh &    # 或者用 nohup（start-dev.sh 会这么拉起）
#   停止：pkill -f "[g]cs_watchdog"
#
# 日志：/tmp/gcs_watchdog.log

source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spadea/catkin_ws/devel/setup.bash 2>/dev/null

LAUNCH=/home/spadea/catkin_ws/src/foxglove_bridge/ros1_foxglove_bridge/launch/foxglove_bridge.launch
INTERVAL=3

log() { echo "[$(date '+%H:%M:%S')] $*" >> /tmp/gcs_watchdog.log; }

port_up() { ss -tln 2>/dev/null | grep -q ":$1 "; }

# ── 健康判据：进程活着 且 注册在当前 master，两条都必须满足 ──
# 只查端口 / 只查 rosnode list 都会被"僵尸注册"和"master 换代后的孤儿"骗过。
# （另：rosnode ping 的退出码在 noetic 上恒为 0，不能当判据。）
bridge_up() {
  pgrep -f "[f]oxglove_bridge_nodelet" >/dev/null 2>&1 &&
    rosnode list 2>/dev/null | grep -qx "/foxglove_bridge"
}

gw_up() {
  pgrep -f "[g]cs_gateway_node" >/dev/null 2>&1 &&
    rosnode list 2>/dev/null | grep -qx "/gcs_gateway"
}

log "启动（间隔 ${INTERVAL}s）"

while true; do
  # master 不在就什么都别做：bridge/gateway 起了也注册不上，只会白折腾。
  # （raicom.sh 重启期间 master 会短暂消失，这里等它回来。）
  if ! port_up 11311; then
    sleep "$INTERVAL"
    continue
  fi

  if ! bridge_up; then
    log "bridge 失效 → 重启"
    pkill -f "[f]oxglove_bridge" 2>/dev/null
    pkill -f "[f]oxglove_nodelet_manager" 2>/dev/null
    sleep 1
    setsid nohup roslaunch "$LAUNCH" port:=8765 > /tmp/gcs_bridge.log 2>&1 < /dev/null &
    sleep 4
  fi

  if ! gw_up; then
    log "网关失效 → 重启"
    pkill -f "[g]cs_gateway_node" 2>/dev/null
    sleep 1
    setsid nohup rosrun as_controller gcs_gateway_node > /tmp/gcs_gateway.log 2>&1 < /dev/null &
    sleep 4
  fi

  sleep "$INTERVAL"
done
