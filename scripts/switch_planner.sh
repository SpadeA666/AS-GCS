#!/bin/bash
# =============================================================================
# switch_planner.sh —— 在地面站运行中切换规划器（SUPER <-> EGO）
#
# 用法:
#   bash switch_planner.sh ego
#   bash switch_planner.sh super
#
# 由 gcs_gateway 在收到 /gcs/set_planner 时后台调用（不能阻塞网关主循环，
# 那个循环每帧都要发 offboard setpoint 心跳）。
#
# 状态写到 $STATE，网关读它并发到 /gcs/planner_status 话题给前端显示：
#   switching ego     切换进行中
#   ready ego         切换完成
#   failed ego <原因>  切换失败
#
# 两个规划器的组成：
#   SUPER : mission_planner/click_demo.launch（含 fsm_node + mission_planner）
#   EGO   : as_controller/octomap.launch（octomap_server，输入 /cloud_registered）
#         + ego_planner/single_run_in_mid.launch（EGO 主节点，点云输入 /octomap_point_cloud_centers）
#
# 注意：EGO 不用 FAST-LIO 的原始点云，而是用 octomap 栅格化之后的
# /octomap_point_cloud_centers。这是 EGO 这套配置的既定接法。
# =============================================================================

set -uo pipefail

TARGET="${1:-}"
STATE=/tmp/planner_switch.state
LOG=/tmp/planner_switch.log
WS="${AS_GCS_WS:-$HOME/catkin_ws}"

log() { echo "[$(date '+%H:%M:%S')] $*" >> "$LOG"; }

# 每步之间等一会儿，给节点起/停的时间
settle() { sleep "$1"; }

fail() {
  echo "failed $TARGET $*" > "$STATE"
  log "失败: $*"
  exit 1
}

case "$TARGET" in
  ego|super) ;;
  *) echo "用法: $0 {ego|super}" >&2; exit 2 ;;
esac

echo "switching $TARGET" > "$STATE"
log "开始切换到 $TARGET"

# ── 先确保 ROS 环境在（crontab/systemd 拉起来时可能没有）──
set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash 2>/dev/null
# shellcheck disable=SC1091
source "$WS/devel/setup.bash" 2>/dev/null
set -u

if [ "$TARGET" = "ego" ]; then
  # ── SUPER -> EGO ──
  log "停掉 SUPER（mission_planner + fsm_node）"
  pkill -f "[m]ission_planner" 2>/dev/null
  pkill -f "[f]sm_node" 2>/dev/null
  settle 2

  log "启动 octomap_server"
  setsid nohup roslaunch as_controller octomap.launch \
    > /tmp/planner_octomap.log 2>&1 < /dev/null &
  settle 3

  log "启动 EGO（single_run_in_mid.launch）"
  setsid nohup roslaunch ego_planner single_run_in_mid.launch \
    > /tmp/planner_ego.log 2>&1 < /dev/null &
  settle 6

  # ── 验证 ──
  if pgrep -f "[e]go_planner_node" >/dev/null 2>&1 || pgrep -f "[t]raj_server" >/dev/null 2>&1; then
    echo "ready ego" > "$STATE"
    log "EGO 已就绪"
  else
    fail "ego 节点未起来，看 /tmp/planner_ego.log"
  fi

else
  # ── EGO -> SUPER ──
  log "停掉 EGO（plan_manage + traj_server）与 octomap"
  pkill -f "[e]go_planner_node" 2>/dev/null
  pkill -f "[t]raj_server" 2>/dev/null
  pkill -f "[o]ctomap_server" 2>/dev/null
  settle 2

  log "启动 SUPER（mission_planner click_demo.launch）"
  setsid nohup roslaunch mission_planner click_demo.launch \
    > /tmp/planner_super.log 2>&1 < /dev/null &
  settle 6

  if pgrep -f "[f]sm_node" >/dev/null 2>&1; then
    echo "ready super" > "$STATE"
    log "SUPER 已就绪"
  else
    fail "fsm_node 未起来，看 /tmp/planner_super.log"
  fi
fi
