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
  pkill -f "[c]lick_demo" 2>/dev/null   # roslaunch 父进程，不杀会残留
  settle 2

  # ── 清掉可能残留的 EGO / octomap（重要！）──
  # 旧写法只在「切到 SUPER」时清理，切到 EGO 时不清理，
  # 于是每切一次 EGO 就多留一个 octomap_server。多个实例同时发
  # /octomap_point_cloud_centers（各 1Hz），EGO 收到 N 倍点云与 N 份内存，
  # 实测堆到 5 个实例后 EGO 主节点直接 std::bad_alloc 崩掉。
  # 而 rosnode list 还留着僵尸注册、traj_server 也仍在跑，
  # 于是下面那个 `||` 判据照样报 ready —— 极其难发现的坑。
  log "清掉残留的 EGO / octomap"
  # 注意匹配式：用 `roslaunch …` 前缀而不是裸文件名，
  # 否则会把 VSCode 里打开的 `single_run_in_mid.launch` / `octomap.launch`
  # （命令行同样含这些字符串）一起杀掉，丢掉未保存的编辑。
  pkill -f "roslaunch ego_planner single_run_in_mid" 2>/dev/null  # EGO 的 roslaunch 父进程
  pkill -f "[e]go_planner_node" 2>/dev/null
  pkill -f "[t]raj_server" 2>/dev/null
  pkill -f "[p]cl_render_node" 2>/dev/null
  pkill -f "roslaunch as_controller octomap" 2>/dev/null
  pkill -f "[o]ctomap_server" 2>/dev/null
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
  # 必须是 `&&`：主节点崩了但 traj_server 还活着时，
  # 旧写法的 `||` 会误报 ready，而实际打点毫无反应。
  if pgrep -f "[e]go_planner_node" >/dev/null 2>&1 && pgrep -f "[t]raj_server" >/dev/null 2>&1; then
    echo "ready ego" > "$STATE"
    log "EGO 已就绪"
  else
    fail "ego 节点未起来（主节点与 traj_server 需同时在），看 /tmp/planner_ego.log"
  fi

else
  # ── EGO -> SUPER ──
  log "停掉 EGO（plan_manage + traj_server）与 octomap"
  pkill -f "roslaunch ego_planner single_run_in_mid" 2>/dev/null
  pkill -f "[e]go_planner_node" 2>/dev/null
  pkill -f "[t]raj_server" 2>/dev/null
  pkill -f "[p]cl_render_node" 2>/dev/null
  pkill -f "roslaunch as_controller octomap" 2>/dev/null
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
