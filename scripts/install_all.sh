#!/bin/bash
# =============================================================================
# install_all.sh —— AS-GCS 一键安装与环境自检
#
# 用法:
#   bash install_all.sh                # 检查 + 装依赖 + 配置 catkin + 编译 + 验证
#   bash install_all.sh --check-only   # 只做环境检查，不改动任何东西（推荐先跑这个）
#   bash install_all.sh --no-build     # 检查 + 装依赖 + 配置，但不编译
#   bash install_all.sh --yes          # 不询问，直接安装（供 agent 使用）
#
# 环境变量（路径与默认值不同时覆盖）:
#   AS_GCS_WS   工作空间路径    默认 $HOME/catkin_ws
#   AS_GCS_PX4  PX4 固件路径    默认 $HOME/PX4_Firmware
#
# 退出码: 0 = 全部通过；非 0 = 有 FAIL 项（数量见退出码）
# =============================================================================

set -uo pipefail

WS="${AS_GCS_WS:-$HOME/catkin_ws}"
PX4_DIR="${AS_GCS_PX4:-$HOME/PX4_Firmware}"

MODE=full
ASSUME_YES=0

for arg in "$@"; do
  case "$arg" in
    --check-only) MODE=check ;;
    --no-build)   MODE=nodeps ;;
    --yes|-y)     ASSUME_YES=1 ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \{0,1\}//' | head -20
      exit 0 ;;
    *) echo "未知参数: $arg（用 --help 看用法）"; exit 2 ;;
  esac
done

# ── 输出助手 ──
OK_N=0; FAIL_N=0; WARN_N=0
ok()   { printf '  \033[32m[OK]\033[0m   %s\n' "$*"; OK_N=$((OK_N+1)); }
fail() { printf '  \033[31m[FAIL]\033[0m %s\n' "$*"; FAIL_N=$((FAIL_N+1)); }
warn() { printf '  \033[33m[WARN]\033[0m %s\n' "$*"; WARN_N=$((WARN_N+1)); }
info() { printf '  \033[36m[--]\033[0m   %s\n' "$*"; }
hdr()  { printf '\n\033[1m═══ %s ═══\033[0m\n' "$*"; }

ask() {
  [ "$ASSUME_YES" = "1" ] && return 0
  local ans
  read -r -p "  $1 [y/N] " ans
  [[ "$ans" =~ ^[Yy]$ ]]
}

# =============================================================================
hdr "1/6 系统与 ROS"
if [ -r /etc/os-release ]; then
  . /etc/os-release
  if [ "${VERSION_ID:-}" = "20.04" ]; then
    ok "Ubuntu ${VERSION_ID} (${PRETTY_NAME})"
  else
    warn "Ubuntu ${VERSION_ID:-未知} —— 本项目在 20.04 上验证，其他版本可能有问题"
  fi
else
  fail "读不到 /etc/os-release"
fi

ARCH=$(uname -m)
info "架构: $ARCH $([ "$ARCH" = "x86_64" ] || echo '（非 x86_64，仿真可能受限，真机端常见 aarch64）')"

if [ -f /opt/ros/noetic/setup.bash ]; then
  ok "ROS Noetic 已安装"
  # ROS 的 setup.bash 里有未绑定变量（ROS_DISTRO 等），会和 set -u 冲突，
  # 所以 source 前后临时关掉。
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  set -u
else
  fail "找不到 /opt/ros/noetic —— 请先装 ROS Noetic"
fi

if command -v catkin >/dev/null 2>&1; then
  ok "catkin_tools 已安装 ($(catkin --version 2>/dev/null | head -1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1))"
else
  fail "catkin_tools 未安装 —— sudo apt install python3-catkin-tools"
fi

# 系统 python3（Anaconda 会抢，所以显式用绝对路径）
if [ -x /usr/bin/python3 ]; then
  PYV=$(/usr/bin/python3 --version 2>&1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
  ok "系统 python3: $PYV"
  case "$PYV" in
    3.8.*) ;;
    *) warn "系统 python3 是 $PYV，本项目在 3.8 上验证" ;;
  esac
else
  fail "找不到 /usr/bin/python3"
fi

if [ -n "${CONDA_PREFIX:-}" ]; then
  warn "检测到 Anaconda 正在激活（$CONDA_PREFIX）—— 它会抢 python3，"
  warn "  本项目的 ROS 脚本一律用 /usr/bin/python3 规避，但别用裸 python3"
fi

# =============================================================================
hdr "2/6 工作空间与 PX4"
if [ -d "$WS/src" ]; then
  ok "工作空间: $WS"
else
  fail "工作空间不存在: $WS （用 AS_GCS_WS=/path/to/ws 指定）"
fi

if [ -d "$WS/src/as_controller" ]; then
  ok "src/as_controller 就位"
else
  fail "src/as_controller 缺失 —— 仓库没拉全？"
fi

if [ -d "$WS/src/as_gcs" ]; then
  ok "src/as_gcs 就位"
else
  fail "src/as_gcs 缺失 —— 仓库没拉全？"
fi

if [ -d "$PX4_DIR" ]; then
  PV=$(git -C "$PX4_DIR" describe --tags 2>/dev/null || echo "未知")
  ok "PX4 固件: $PX4_DIR ($PV)"
  [ "$PV" = "v1.13.2" ] || warn "PX4 版本是 $PV，本项目基于 v1.13.2"
else
  warn "找不到 PX4: $PX4_DIR —— 只跑真机不需要它；跑仿真需要（用 AS_GCS_PX4= 指定）"
fi

# =============================================================================
hdr "3/6 apt 依赖"
APT_MISSING=()
for p in ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-tf2-ros \
         ros-noetic-cv-bridge libceres-dev libeigen3-dev libpcl-dev \
         libopencv-dev protobuf-compiler libprotobuf-dev libarmadillo-dev; do
  if dpkg -l "$p" 2>/dev/null | grep -q '^ii'; then
    :
  else
    APT_MISSING+=("$p")
  fi
done

if [ ${#APT_MISSING[@]} -eq 0 ]; then
  ok "关键 apt 包齐全（12 项）"
else
  warn "缺少 ${#APT_MISSING[@]} 个 apt 包:"
  for p in "${APT_MISSING[@]}"; do info "  $p"; done
  info "安装命令（需 sudo，脚本不代劳）:"
  printf '        sudo apt install -y %s\n' "${APT_MISSING[*]}"
fi

# =============================================================================
hdr "4/6 第三方 ROS 包"
TP_OK=0; TP_MISSING=()
for p in Livox-SDK2 livox_ros_driver2 faster-lio foxglove_bridge \
         SUPER ego_planner yolov8_ros lio_to_mavros_main; do
  if [ -d "$WS/src/$p" ]; then
    TP_OK=$((TP_OK+1))
  else
    TP_MISSING+=("$p")
  fi
done
ok "$TP_OK/8 个第三方包就位"
if [ ${#TP_MISSING[@]} -gt 0 ]; then
  warn "缺少 ${#TP_MISSING[@]} 个（来源见 docs/REPRODUCE.md 第 3.3 节）:"
  for p in "${TP_MISSING[@]}"; do info "  $p"; done
fi

# Livox SDK 静态库
if ls /usr/local/lib/liblivox_lidar_sdk_static.a >/dev/null 2>&1; then
  ok "Livox-SDK2 已安装到 /usr/local"
else
  if [ -d "$WS/src/Livox-SDK2" ]; then
    warn "Livox-SDK2 源码在，但静态库未安装到 /usr/local —— livox_ros_driver2 会编译失败"
    info "  cd $WS/src/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j\$(nproc) && sudo make install"
  else
    warn "Livox-SDK2 缺失"
  fi
fi

# 遥控器
if [ -e /dev/input/js0 ]; then
  JSNAME=$(cat /sys/class/input/js0/device/name 2>/dev/null || echo "未知")
  ok "遥控器已连接: $JSNAME"
  case "$JSNAME" in
    *TX12*) ok "  识别为 RadioMaster TX12 —— 可直接用本仓库默认通道映射" ;;
    *)      warn "  非 TX12 型号 —— 必须重新探测通道映射："
            info "    /usr/bin/python3 $WS/src/as_gcs/scripts/probe_joy.py 60" ;;
  esac
else
  warn "没有 /dev/input/js0 —— 遥控器未接或未切到 USB Joystick 模式（不影响仿真，只影响遥控器接管）"
fi

# =============================================================================
hdr "5/6 依赖安装"
if [ "$MODE" = "check" ]; then
  info "--check-only：跳过安装"
else
  # Python 包
  if /usr/bin/python3 -c "import em" 2>/dev/null; then
    EMPY_V=$(/usr/bin/python3 -c "import em; print(getattr(em,'__version__','?'))" 2>/dev/null || echo "?")
    ok "empy 已装 ($EMPY_V)"
  else
    warn "empy 未装（或版本不对）—— 这是消息生成失败的头号原因"
    if ask "现在 pip 安装 empy==3.3.4 等 Python 包？"; then
      /usr/bin/python3 -m pip install --user "empy==3.3.4" catkin_pkg rosdep rosdistro 2>&1 | tail -3
      /usr/bin/python3 -c "import em" 2>/dev/null && ok "Python 依赖安装完成" || fail "Python 依赖仍不可用"
    fi
  fi

  # 地理围栏数据（漏了 mavros 起不来）
  if [ -f /usr/share/GeographicLib/geoids/egm96-5.pgm ] || \
     [ -d /usr/share/GeographicLib/geoids ] && [ "$(ls /usr/share/GeographicLib/geoids 2>/dev/null | wc -l)" -gt 0 ]; then
    ok "GeographicLib 数据集已存在"
  else
    warn "GeographicLib 数据集缺失 —— mavros 会启动失败"
    if ask "现在运行 install_geographiclib_datasets.sh？（需要 sudo）"; then
      sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh 2>&1 | tail -3
    fi
  fi

  # catkin 配置（路径动态生成，别人机器上也能用）
  if [ -d "$WS/cmake" ]; then
    info "配置 catkin cmake 参数（按 $WS 实际路径）"
    ( cd "$WS" && catkin config --cmake-args \
        -Dprotobuf_DIR="$WS/cmake" \
        -DROS_EDITION=ROS1 \
        -DCeres_DIR=/usr/lib/cmake/Ceres >/dev/null 2>&1 ) \
      && ok "catkin 配置完成" || fail "catkin config 失败"
  else
    warn "没有 $WS/cmake/protobuf-config.cmake —— 跳过 protobuf 桩配置"
    ( cd "$WS" && catkin config --cmake-args -DROS_EDITION=ROS1 -DCeres_DIR=/usr/lib/cmake/Ceres >/dev/null 2>&1 ) \
      && ok "catkin 配置完成（无 protobuf 桩）" || fail "catkin config 失败"
  fi
fi

# =============================================================================
hdr "6/6 编译与验证"
if [ "$MODE" = "check" ]; then
  info "--check-only：跳过编译"
elif [ "$MODE" = "nodeps" ]; then
  info "--no-build：跳过编译"
else
  if ask "现在执行 catkin build？会比较久"; then
    ( cd "$WS" && catkin build 2>&1 | tail -20 )
    if [ "${PIPESTATUS[0]}" -eq 0 ]; then
      ok "catkin build 成功"
    else
      fail "catkin build 失败 —— 报错处理见 BUILD_GUIDE.md"
    fi
  else
    info "已跳过编译"
  fi
fi

# ── 验证：关键产物是否存在 ──
hdr "结果验证"
if [ -f "$WS/devel/setup.bash" ]; then
  ok "devel/setup.bash 存在"
else
  fail "devel/setup.bash 不存在 —— 还没编译成功"
fi

for t in as_gcs/Takeoff as_gcs/SetNavMode as_controller/; do
  if find "$WS/devel" -path "*$t*" -print -quit 2>/dev/null | grep -q .; then
    ok "消息/模块已生成: $t"
  else
    warn "未找到: $t （未编译或编译失败）"
  fi
done

if [ -x "$WS/devel/lib/as_controller/gcs_gateway_node" ]; then
  ok "gcs_gateway_node 已编译"
else
  warn "gcs_gateway_node 未编译 —— 地面站控制面不可用"
fi

# =============================================================================
printf '\n\033[1m═══════ 总结 ═══════\033[0m\n'
printf '  通过 %d 项 / 警告 %d 项 / 失败 %d 项\n' "$OK_N" "$WARN_N" "$FAIL_N"

if [ "$FAIL_N" -eq 0 ]; then
  printf '\n  \033[32m环境就绪。\033[0m 下一步：\n'
  printf '    起仿真+地面站:  bash %s/src/as_gcs/ui/restart_all.sh raicom\n' "$WS"
  printf '    浏览器:         http://localhost:5173/\n'
  printf '    遥控器接管:     见 docs/REPRODUCE.md 第九节\n'
else
  printf '\n  \033[31m有 %d 项失败，先解决上面标 [FAIL] 的问题。\033[0m\n' "$FAIL_N"
fi

if [ "$WARN_N" -gt 0 ]; then
  printf '  %d 项警告不阻塞仿真，但会影响遥控器等可选功能。\n' "$WARN_N"
fi

if [ "$MODE" = "check" ]; then
  printf '\n  （--check-only 模式，未做任何改动）\n'
fi

exit "$FAIL_N"
