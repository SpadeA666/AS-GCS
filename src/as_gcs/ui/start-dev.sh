#!/bin/bash
# 一键启动地面站开发环境：roscore + foxglove_bridge + gcs_gateway + 前端
# 全部用 setsid 脱离当前会话，关掉终端也不影响。

source /opt/ros/noetic/setup.bash
source /home/spadea/catkin_ws/devel/setup.bash

UI_DIR=/home/spadea/catkin_ws/src/as_gcs/ui
LAUNCH=/home/spadea/catkin_ws/src/foxglove_bridge/ros1_foxglove_bridge/launch/foxglove_bridge.launch

port_up() { ss -tln 2>/dev/null | grep -q ":$1 "; }

# 关键：判断“节点是否注册在当前 master 上”，而不是“端口还在不在”。
# 每次用 raicom.sh 重启仿真，ROS master 都会换一茬（新进程、新注册表）。
# 这时 bridge/gateway 进程还活着、端口还占着，但已经和 master 失联：
# 端口检查会误判成“已在运行”，于是前端连得上却拿不到任何 /gcs/ 服务。
# ── 健康判据（两处坑都踩过了，写成下面这样）──
# 要同时满足两条，缺一不可：
#   ① 进程真的还活着 —— 进程被 kill 后，master 上的注册会残留一段时间（僵尸注册）；
#   ② 注册在当前 master —— master 换代后进程还活着，但已和 master 失联。
# 只查“端口在不在”或“rosnode list 里有没有”会被这两种情况骗过：
# 脚本报“已在运行”，而前端连上了却拿不到任何 /gcs/ 服务。
# （另：rosnode ping 的退出码在 noetic 上恒定是 0，连不存在的节点也返回 0，不能做判据。）
bridge_up() {
  pgrep -f "[f]oxglove_bridge_nodelet" >/dev/null 2>&1 &&
    rosnode list 2>/dev/null | grep -qx "/foxglove_bridge"
}

gw_up() {
  pgrep -f "[g]cs_gateway_node" >/dev/null 2>&1 &&
    rosnode list 2>/dev/null | grep -qx "/gcs_gateway"
}

# ── 1. roscore ──
# 注意：用端口判断，别用 pgrep "roscore" —— roscore 实际 exec 的是 rosmaster，
# 进程命令行里没有 "roscore" 这个词，pgrep 永远匹配不到。
echo "── roscore ──"
if port_up 11311; then
  echo "  已在运行 (:11311)"
else
  setsid nohup roscore > /tmp/gcs_roscore.log 2>&1 < /dev/null &
  for _ in $(seq 1 20); do port_up 11311 && break; sleep 0.5; done
  echo "  已启动"
fi

# ── 2. foxglove_bridge ──
echo "── foxglove_bridge ──"
if bridge_up; then
  echo "  已在运行（注册在当前 master）"
else
  # 端口可能被“连旧 master 的僵尸 bridge”占着，先清掉再起，否则新 bridge 绑不上 8765。
  pkill -f "[f]oxglove_bridge" 2>/dev/null
  pkill -f "[f]oxglove_nodelet_manager" 2>/dev/null
  sleep 1
  setsid nohup roslaunch "$LAUNCH" port:=8765 > /tmp/gcs_bridge.log 2>&1 < /dev/null &
  for _ in $(seq 1 40); do bridge_up && break; sleep 0.5; done
  echo "  已启动"
fi

# ── 3. gcs_gateway（地面站控制面）──
# 取代 asnav_3d_node：两者都实例化 ASNAV 并占用同一批话题，不能同时跑。
echo "── gcs_gateway ──"
if gw_up; then
  echo "  已在运行"
else
  # 同上：孤儿网关还占着 ASNAV 的那批话题，必须先杀掉，否则新网关起不来。
  pkill -f "[g]cs_gateway_node" 2>/dev/null
  sleep 1
  setsid nohup rosrun as_controller gcs_gateway_node > /tmp/gcs_gateway.log 2>&1 < /dev/null &
  for _ in $(seq 1 24); do gw_up && break; sleep 0.5; done
  echo "  已启动"
fi

# ── 4. 前端 ──
echo "── 前端 dev server ──"
if port_up 5173; then
  echo "  已在运行 (:5173)"
else
  cd "$UI_DIR" || exit 1
  setsid nohup npx vite --host > /tmp/gcs_ui.log 2>&1 < /dev/null &
  for _ in $(seq 1 30); do port_up 5173 && break; sleep 0.5; done
  echo "  已启动"
fi

# ── 5. watchdog ──
# 守护 bridge/gateway：raicom.sh 重启会让 master 换代，这两个组件会变孤儿。
# 有了它就不用每次 master 换代后手动重跑本脚本。
echo "── watchdog ──"
if pgrep -f "[g]cs_watchdog" >/dev/null 2>&1; then
  echo "  已在运行"
else
  setsid nohup bash "$UI_DIR/gcs_watchdog.sh" > /dev/null 2>&1 < /dev/null &
  sleep 1
  echo "  已启动（日志 /tmp/gcs_watchdog.log）"
fi

echo
echo "── 状态 ──"
bridge_up && echo "  bridge    ✓ (:8765, 进程与注册都在)" || echo "  bridge    ✗（进程或注册缺失）"
port_up 5173 && echo "  前端      ✓ (:5173)" || echo "  前端      ✗"
if gw_up; then
  echo "  网关      ✓ $(rosservice list 2>/dev/null | grep -c '^/gcs/') 个 /gcs/ 服务（节点响应正常）"
else
  echo "  网关      ✗ 节点无响应（僵尸注册也会出现在列表里，别被数字骗了）"
fi
echo
echo "浏览器打开： http://localhost:5173/"
echo "日志： /tmp/gcs_roscore.log  /tmp/gcs_bridge.log  /tmp/gcs_gateway.log  /tmp/gcs_ui.log"
