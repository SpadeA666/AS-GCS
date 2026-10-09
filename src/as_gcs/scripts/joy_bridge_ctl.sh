#!/bin/bash
# joy_rc_bridge 的启停控制
# 用法: joy_bridge_ctl.sh {start|stop|status|log}

SCRIPT="$HOME/catkin_ws/src/as_gcs/scripts/joy_rc_bridge.py"
LOG=/tmp/joy_bridge.log
PAT="[j]oy_rc_bridge.py"

case "$1" in
  start)
    if pgrep -f "$PAT" >/dev/null; then
      echo "已经在运行 (PID $(pgrep -f "$PAT"))"; exit 0
    fi
    source /opt/ros/noetic/setup.bash
    source "$HOME/catkin_ws/devel/setup.bash" 2>/dev/null
    setsid bash -c "source /opt/ros/noetic/setup.bash; source $HOME/catkin_ws/devel/setup.bash 2>/dev/null; exec /usr/bin/python3 $SCRIPT" > "$LOG" 2>&1 < /dev/null &
    sleep 3
    if pgrep -f "$PAT" >/dev/null; then
      echo "已启动 (PID $(pgrep -f "$PAT"))，日志: $LOG"
    else
      echo "启动失败，看 $LOG"; tail -20 "$LOG"
    fi
    ;;
  stop)
    pkill -f "$PAT" && echo "已停止" || echo "没在运行"
    ;;
  status)
    pgrep -af "$PAT" || echo "没在运行"
    ;;
  log)
    tail -40 "$LOG"
    ;;
  *)
    echo "用法: $0 {start|stop|status|log}"
    ;;
esac
