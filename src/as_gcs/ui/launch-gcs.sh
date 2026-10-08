#!/bin/bash
# AS 地面站 —— 双击启动器
#
# 做三件事：确保 roscore / foxglove_bridge / 前端服务都在跑，然后打开浏览器。
# 已经在跑的服务会跳过，不会重复启动。

# ROS 环境：下面的健康检查（rosnode list）需要它，否则判定会永远失败。
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spadea/catkin_ws/devel/setup.bash 2>/dev/null

UI_DIR=/home/spadea/catkin_ws/src/as_gcs/ui
BRIDGE_LAUNCH=/home/spadea/catkin_ws/src/foxglove_bridge/ros1_foxglove_bridge/launch/foxglove_bridge.launch
WEB_PORT=4173
WS_PORT=8765

notify() {
  # 有 notify-send 就弹通知，没有就只写日志
  command -v notify-send >/dev/null 2>&1 && notify-send "AS 地面站" "$1" || true
  echo "[$(date +%H:%M:%S)] $1" >> /tmp/gcs_launcher.log
}

port_up() {
  ss -tln 2>/dev/null | grep -q ":$1 "
}

# ── 健康判据：进程活着 且 注册在当前 master，两条都要满足 ──
# 只查端口 / 只查 rosnode list 会被两种假象骗过：
#   ① 进程被 kill 了但 master 上注册未清（僵尸注册）
#   ② master 换代后进程还活着但已失联（孤儿）
# 这两种情况下端口都占着、列表也能查到，脚本会误判成“已在运行”。
bridge_up() {
  pgrep -f "[f]oxglove_bridge_nodelet" >/dev/null 2>&1 &&
    rosnode list 2>/dev/null | grep -qx "/foxglove_bridge"
}

gw_up() {
  pgrep -f "[g]cs_gateway_node" >/dev/null 2>&1 &&
    rosnode list 2>/dev/null | grep -qx "/gcs_gateway"
}

# ── 1. ROS master ──
# 用端口判断，不要用 pgrep "roscore"：roscore 实际 exec 的是 rosmaster，
# 进程命令行里压根没有 roscore 这个词，pgrep 永远匹配不到。
if ! port_up 11311; then
  notify "启动 roscore…"
  source /opt/ros/noetic/setup.bash 2>/dev/null
  setsid nohup roscore > /tmp/gcs_roscore.log 2>&1 < /dev/null &
  for _ in $(seq 1 20); do
    port_up 11311 && break
    sleep 0.5
  done
fi

# ── 2. foxglove_bridge ──
if ! bridge_up; then
  notify "启动数据桥 foxglove_bridge..."
  # 清掉可能还占着端口、但已连不上 master 的僵尸 bridge，否则新起的绑不上 8765
  pkill -f "[f]oxglove_bridge" 2>/dev/null
  pkill -f "[f]oxglove_nodelet_manager" 2>/dev/null
  sleep 1
  setsid nohup roslaunch "$BRIDGE_LAUNCH" port:=$WS_PORT > /tmp/gcs_bridge.log 2>&1 < /dev/null &
  for _ in $(seq 1 40); do
    bridge_up && break
    sleep 0.5
  done
fi

# ── 3. gcs_gateway（地面站控制面；取代 asnav_3d_node，两者不可同时跑）──
if ! gw_up; then
  notify "启动地面站网关 gcs_gateway..."
  # 孤儿网关还占着 ASNAV 的那批话题，得先杀掉，否则新的起不来
  pkill -f "[g]cs_gateway_node" 2>/dev/null
  sleep 1
  setsid nohup rosrun as_controller gcs_gateway_node > /tmp/gcs_gateway.log 2>&1 < /dev/null &
  sleep 4
fi

# ── 3.5 watchdog：master 换代后自动救回上面两个组件 ──
if ! pgrep -f "[g]cs_watchdog" >/dev/null 2>&1; then
  setsid nohup bash "$UI_DIR/gcs_watchdog.sh" > /dev/null 2>&1 < /dev/null &
fi

# ── 4. 前端静态服务器 ──
if ! port_up "$WEB_PORT"; then
  # dist/ 不存在或比源码旧，就先构建
  if [ ! -f "$UI_DIR/dist/index.html" ] || [ -n "$(find "$UI_DIR/src" -newer "$UI_DIR/dist/index.html" 2>/dev/null)" ]; then
    notify "重新构建前端…"
    cd "$UI_DIR" || exit 1
    npm run build >> /tmp/gcs_build.log 2>&1
  fi
  setsid nohup npx vite preview --host --port "$WEB_PORT" > /tmp/gcs_ui_prod.log 2>&1 < /dev/null &
  for _ in $(seq 1 30); do
    port_up "$WEB_PORT" && break
    sleep 0.5
  done
fi

# ── 5. 打开浏览器 ──
if port_up "$WEB_PORT"; then
  notify "就绪，正在打开浏览器"
  xdg-open "http://localhost:$WEB_PORT/" >/dev/null 2>&1 &
else
  notify "启动失败，请看 /tmp/gcs_*.log"
fi
