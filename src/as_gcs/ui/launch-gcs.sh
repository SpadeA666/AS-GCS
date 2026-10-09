#!/bin/bash
# AS 地面站 —— 双击/菜单启动器
#
# 统一走 start-dev.sh（dev server :5173），不再单独维护 4173 生产预览。
#
# 为什么废弃 4173 那套：
#   旧版这里跑的是 `vite preview`，服务的是 dist/ 构建产物 —— 改了源码
#   不重新 `npm run build`，页面永远停在旧版本，极易误判成"改动没生效"。
#   而且它当时是用 npx 拉起 vite（~/.npm/_npx/...），拿的不是项目依赖里的
#   那个版本，行为还可能和 dev 不一致。dev server 有 HMR，改完立刻可见。
#
# 浏览器入口统一为 http://localhost:5173/

UI_DIR=/home/spadea/catkin_ws/src/as_gcs/ui
WEB_PORT=5173

notify() {
  command -v notify-send >/dev/null 2>&1 && notify-send "AS 地面站" "$1" || true
  echo "[$(date +%H:%M:%S)] $1" >> /tmp/gcs_launcher.log
}

notify "启动中…"

# start-dev.sh 负责：bridge + gateway 自愈、拉起 watchdog、启动前端
bash "$UI_DIR/start-dev.sh"

if ss -tln 2>/dev/null | grep -q ":$WEB_PORT "; then
  notify "就绪，正在打开浏览器"
  xdg-open "http://localhost:$WEB_PORT/" >/dev/null 2>&1 &
else
  notify "启动失败，请看 /tmp/gcs_*.log"
fi
