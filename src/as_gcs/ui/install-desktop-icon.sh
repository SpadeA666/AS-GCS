#!/bin/bash
# =============================================================================
# install-desktop-icon.sh —— 把「AS 地面站」装成桌面 / 应用菜单的快捷方式
#
# 用法:
#   bash install-desktop-icon.sh             # 装到应用菜单 + 桌面
#   bash install-desktop-icon.sh --app-only  # 只装应用菜单（不放到桌面）
#   bash install-desktop-icon.sh --remove    # 卸载
#
# 它会读同目录下的 as-gcs.desktop 模板，把 __UI_DIR__ 换成当前实际路径，
# 再写到 ~/.local/share/applications/ 和桌面目录。这样别人 clone 到任何
# 路径都能用，不用手改绝对路径。
# =============================================================================

set -uo pipefail

UI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEMPLATE="$UI_DIR/as-gcs.desktop"
APP_DIR="$HOME/.local/share/applications"
NAME="as-gcs.desktop"
MODE=full

for arg in "$@"; do
  case "$arg" in
    --app-only) MODE=app ;;
    --remove)   MODE=remove ;;
    -h|--help)  grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -14; exit 0 ;;
    *) echo "未知参数: $arg"; exit 2 ;;
  esac
done

# 桌面目录：优先问 xdg，问不到就退回 ~/桌面 / ~/Desktop
detect_desktop() {
  local d
  d=$(xdg-user-dir DESKTOP 2>/dev/null)
  if [ -n "$d" ] && [ -d "$d" ]; then echo "$d"; return; fi
  for d in "$HOME/桌面" "$HOME/Desktop"; do
    [ -d "$d" ] && { echo "$d"; return; }
  done
  echo ""
}
DESKTOP_DIR="$(detect_desktop)"

# ── 卸载 ──
if [ "$MODE" = "remove" ]; then
  rm -f "$APP_DIR/$NAME" && echo "  已删除 $APP_DIR/$NAME"
  [ -n "$DESKTOP_DIR" ] && rm -f "$DESKTOP_DIR/$NAME" && echo "  已删除 $DESKTOP_DIR/$NAME"
  command -v update-desktop-database >/dev/null 2>&1 && \
    update-desktop-database "$APP_DIR" >/dev/null 2>&1
  echo "  卸载完成"
  exit 0
fi

# ── 前置检查 ──
if [ ! -f "$TEMPLATE" ]; then
  echo "  [FAIL] 找不到模板: $TEMPLATE"
  exit 1
fi
if [ ! -x "$UI_DIR/launch-gcs.sh" ]; then
  echo "  [WARN] launch-gcs.sh 不可执行，正在修复"
  chmod +x "$UI_DIR/launch-gcs.sh"
fi
if [ ! -f "$UI_DIR/icon.png" ]; then
  echo "  [WARN] 找不到 icon.png，快捷方式会没有图标"
fi

# ── 生成 .desktop ──
TMP="$(mktemp)"
sed "s|__UI_DIR__|$UI_DIR|g" "$TEMPLATE" > "$TMP"
chmod +x "$TMP"

echo "── 安装到应用菜单 ──"
mkdir -p "$APP_DIR"
cp "$TMP" "$APP_DIR/$NAME"
chmod +x "$APP_DIR/$NAME"
echo "  $APP_DIR/$NAME"
echo "    Exec = $UI_DIR/launch-gcs.sh"
echo "    Icon = $UI_DIR/icon.png"

if [ "$MODE" = "full" ]; then
  echo "── 安装到桌面 ──"
  if [ -z "$DESKTOP_DIR" ]; then
    echo "  [WARN] 没找到桌面目录（xdg-user-dir 与 ~/桌面 都没有），跳过"
    echo "         可手动复制： cp '$APP_DIR/$NAME' ~/桌面/"
  else
    cp "$TMP" "$DESKTOP_DIR/$NAME"
    chmod +x "$DESKTOP_DIR/$NAME"
    # GNOME 需要把 .desktop 标记为"受信任"才允许双击启动
    if command -v gio >/dev/null 2>&1; then
      gio set "$DESKTOP_DIR/$NAME" metadata::trusted true 2>/dev/null || true
    fi
    echo "  $DESKTOP_DIR/$NAME"
  fi
fi

rm -f "$TMP"

# ── 刷新缓存 ──
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$APP_DIR" >/dev/null 2>&1 && echo "── 已刷新应用菜单数据库"
fi

echo
echo "── 完成 ──"
echo "  应用菜单里搜「AS 地面站」即可启动。"
echo "  桌面图标若没马上出现，注销重登一次（GNOME 图标缓存比较粘）。"
echo "  卸载：bash $0 --remove"
