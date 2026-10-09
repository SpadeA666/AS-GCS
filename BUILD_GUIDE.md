# catkin_ws 编译指南与常见问题解决方案

> 最后更新: 2026-08-01
> 工作空间: 55 个包全部编译成功
> 构建工具: catkin_tools (`catkin build`)
> ROS 版本: Noetic (Ubuntu 20.04)

---

## 编译前必须的准备工作

### 1. 确认 Python 依赖已安装

工作空间使用 Anaconda Python 3.13,需安装 ROS 相关的 Python 包:

```bash
pip install empy==3.3.4 catkin_pkg rosdep rosdistro
```

> ⚠️ **empy 版本必须是 3.x**,版本 4.x 与 ROS Noetic 的 catkin 不兼容,会导致消息生成时报 `module 'em' has no attribute 'RAW_OPT'` 错误。

### 2. 确认 Livox-SDK2 已安装

`livox_ros_driver2` 需要 Livox SDK 的静态库:

```bash
cd ~/catkin_ws/src/Livox-SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install   # 安装到 /usr/local/lib/liblivox_lidar_sdk_static.a
```

### 3. 确认 catkin_tools 配置

```bash
catkin config --cmake-args \
  -Dprotobuf_DIR=/home/spadea/catkin_ws/cmake \
  -DROS_EDITION=ROS1 \
  -DCeres_DIR=/usr/lib/cmake/Ceres
```

这三行的作用:

| CMake 参数 | 解决的问题 |
|-----------|-----------|
| `-Dprotobuf_DIR=...` | 绕过 Anaconda protobuf v29.3 与系统 protobuf v3.6.1 的冲突 |
| `-DROS_EDITION=ROS1` | 强制 `livox_ros_driver2` 使用 ROS1 模式(而非 ROS2 的 ament) |
| `-DCeres_DIR=...` | 使用系统 libceres-dev (共享库) 避免与 conan glog 链接冲突 |

---

## 编译命令

### 完整重编译(清理后从零开始)

```bash
cd ~/catkin_ws
catkin clean -y
# 如果重新配置了 catkin config 参数,需要执行:
catkin config --cmake-args \
  -Dprotobuf_DIR=/home/spadea/catkin_ws/cmake \
  -DROS_EDITION=ROS1 \
  -DCeres_DIR=/usr/lib/cmake/Ceres
# 执行编译
catkin build
```

### 增量编译(只编译修改过的包)

```bash
cd ~/catkin_ws
catkin build
```

### 单个包编译

```bash
catkin build <package_name>
```

### 使用构建脚本

```bash
~/catkin_ws/scripts/build.sh
```

---

## 已知问题与解决方案汇总

### 问题 1: Anaconda protobuf 与系统 protobuf 冲突

**症状:**
```
CMake Error at /home/spadea/anaconda3/lib/cmake/protobuf/protobuf-targets.cmake:42:
  Some (but not all) targets in this export set were already defined.
  Targets Defined: protobuf::libprotobuf-lite, ...
  Targets not yet defined: protobuf::libupb, ...
```

**原因:** Anaconda3 自带的 protobuf v29.3 CMake 配置文件与系统 protobuf v3.6.1 的目标(target)定义冲突。Gazebo 的 `FindIgnProtobuf.cmake` 先用 MODULE 模式加载了系统的 4 个基础 target,然后 CONFIG 模式又找到 Anaconda 的配置,后者尝试导入包含 7 个 target 的导出集 —— 其中 4 个已存在、3 个不存在,CMake 不允许部分重叠。

**解决方案(二选一):**

**方案 A (推荐):** 创建 CMake 桩文件 + 配置 `protobuf_DIR`
```
cmake/protobuf-config.cmake   (已在工作空间中)
catkin config --cmake-args -Dprotobuf_DIR=/home/spadea/catkin_ws/cmake
```

**方案 B (备选):** 编译时临时隐藏 Anaconda 的 protobuf 配置:
```bash
mv /home/spadea/anaconda3/lib/cmake/protobuf /tmp/protobuf_cmake_backup
catkin build
mv /tmp/protobuf_cmake_backup /home/spadea/anaconda3/lib/cmake/protobuf
```

> ⚠️ 方案 B 中,**不能**将 protobuf 目录重命名为 `protobuf.bak` 并留在 `anaconda3/lib/cmake/` 下,因为 CMake 的 `find_package` CONFIG 模式在搜索时会匹配 `protobuf*` 通配符,仍会找到该目录。

---

### 问题 2: empy 模块版本不兼容

**症状:**
```
AttributeError: module 'em' has no attribute 'RAW_OPT'
```
发生在消息生成包的 make 阶段 (ar_track_alvar_msgs, gazebo_msgs, quadrotor_msgs 等)。

**原因:** `pip install empy` 默认安装 4.x 版本,但 ROS Noetic 的 catkin 依赖 empy 3.x 的 `RAW_OPT` API。

**解决:**
```bash
pip install empy==3.3.4
```

---

### 问题 3: catkin_pkg 缺失

**症状:**
```
ImportError: No module named 'catkin_pkg'
```
或:
```
from catkin_pkg.package import parse_package failed
```

**原因:** catkin_tools 使用 Anaconda 的 Python 3.13,但该 Python 环境缺少 ROS 相关的 Python 包。

**解决:**
```bash
pip install catkin_pkg rosdep rosdistro
```

---

### 问题 4: livox_ros_driver2 在 ROS1 环境下错误使用 ROS2 构建

**症状:**
```
CMake Error: By not providing "Findament_cmake_auto.cmake" in CMAKE_MODULE_PATH
```

**原因:** `livox_ros_driver2/CMakeLists.txt` 通过 `ROS_EDITION` 变量判断 ROS1/ROS2 模式。变量未设置时默认走 ROS2 分支,需要 `ament_cmake_auto` (ROS2 构建工具)。

**解决:**
```bash
catkin config --cmake-args -DROS_EDITION=ROS1
```

---

### 问题 5: Ceres / glog / lzma 链接错误

**症状:**
```
undefined reference to `google::InitVLOG3__'
undefined reference to `google::kLogSiteUninitialized'
undefined reference to `lzma_stream_buffer_decode'
```
发生在 `global_fusion` 和 `camera_models` (VINS-Fusion 子包)的链接阶段。

**原因:** CMake 找到 `/usr/local/lib/libceres.a`(自编译的静态 Ceres)和 conan 安装的 glog 0.6.0。自编译 Ceres 链接了 conan 的 glog,而 conan glog 依赖 `liblzma`,但链接时未包含 `-llzma`。同时 conan glog 的版本(0.6.0)与系统 glog(0.4.0)的符号不兼容。

**解决:**
```bash
catkin config --cmake-args -DCeres_DIR=/usr/lib/cmake/Ceres
```
强制 CMake 使用系统的 `libceres-dev`(共享库 v1.14.0),它正确链接了系统 glog(0.4.0)。

---

### 问题 6: api_library 依赖未声明 (catkin_make → catkin build 迁移问题)

**症状:**
```
fatal error: ar_track_alvar_msgs/AlvarMarkers.h: 没有那个文件或目录
```

**原因:** `api_library` 的头文件 `api_library.h` 包含了 `<ar_track_alvar_msgs/AlvarMarkers.h>`,但在 `package.xml` 和 `CMakeLists.txt` 中都没有声明对 `ar_track_alvar_msgs` 的依赖。用 `catkin_make` 构建时,所有包共享一个 CMake 上下文,include 路径自动可见。`catkin build` 为每个包创建隔离的构建环境,依赖必须显式声明。

**修复内容:**

`package.xml` — 添加:
```xml
<build_depend>ar_track_alvar_msgs</build_depend>
<build_export_depend>ar_track_alvar_msgs</build_export_depend>
<exec_depend>ar_track_alvar_msgs</exec_depend>
```

`CMakeLists.txt` — 添加:
```cmake
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs  # 新增
  ...
)

catkin_package(
  INCLUDE_DIRS include       # 取消注释
  LIBRARIES api             # 取消注释并改为 api
  CATKIN_DEPENDS ar_track_alvar_msgs ...  # 新增
)
```

---

### 问题 7: microuav2025 依赖未声明 + api_library 未导出库

**症状:**
```
/usr/bin/ld: 找不到 -lapi
collect2: error: ld returned 1 exit status
```

**原因:** 两个问题叠加:
1. `microuav2025` 在 `target_link_libraries` 中链接了 `api`(来自 `api_library` 的库),但未在 `package.xml` 和 `CMakeLists.txt` 中声明依赖。
2. `api_library` 的 `catkin_package()` 中 `INCLUDE_DIRS` 和 `LIBRARIES` 被注释掉了,没有对外导出其库。

**修复内容:**

`api_library/CMakeLists.txt`:
```cmake
catkin_package(
  INCLUDE_DIRS include       # 取消注释
  LIBRARIES api              # 取消注释,指定导出的库名
  CATKIN_DEPENDS ar_track_alvar_msgs geometry_msgs mavros_msgs roscpp std_msgs message_runtime
)
```

`microuav2025/package.xml` — 添加 api_library 的 build_depend、build_export_depend、exec_depend。

`microuav2025/CMakeLists.txt` — 添加:
```cmake
find_package(catkin REQUIRED COMPONENTS
  api_library  # 新增
  ...
)

target_link_libraries(microuav2025_node
  ...
  ${api_library_LIBRARIES}  # 替代原始 -lapi
  ...
)
```

---

### 问题 8: catkin_make 与 catkin build 构建空间冲突

**症状:**
```
The build space at '/home/spadea/catkin_ws/build' was previously built by 'catkin_make'.
Please remove the build space or pick a different build space.
```

**原因:** `catkin_make` 和 `catkin build` 使用不同的构建空间结构,不能混用。

**解决:**
```bash
catkin clean -y    # 清除旧的 build 和 devel 空间
catkin build       # 重新使用 catkin_tools 构建
```

---

### 问题 9: SUPER 从独立工作空间迁移到 catkin_ws

**背景:** SUPER 是一个多组件无人机规划系统,原本位于独立的 ROS 工作空间。迁移到 `catkin_ws` 后,使用 `catkin build` 统一编译。

**SUPER 包含的子包:**

| 子包 | 说明 | 构建产物 |
|------|------|---------|
| `super_planner` | 核心规划器 | `libsuper.a` (静态库) + `fsm_node`、`traj_opt_tuning`、`read_replan_log` (可执行文件) |
| `rog_map` | ROG 地图服务 | rog_map 库和节点 |
| `mission_planner` | 任务规划器 | mission_planner 可执行文件 |
| `marsim_render` | 仿真渲染 | marsim_render 可执行文件 |
| `drone_detect` | 无人机检测 | drone_detect 可执行文件 |
| `bspline_opt` | B样条轨迹优化 | bspline_opt 库 |
| `plan_env` | 规划环境 | plan_env 库 |
| `path_searching` | 路径搜索(A*) | path_searching 库 |
| `traj_utils` | 轨迹工具 | traj_utils 库 |
| `pose_utils` | 位姿工具 | pose_utils 库 |
| `uav_utils` | 无人机通用工具 | uav_utils 库 |
| `cmake_utils` | CMake 辅助宏 | cmake_utils 配置 |

> ⚠️ `mars_uav_sim/mars_quadrotor_msgs` 通过 `CATKIN_IGNORE` 跳过编译(该消息包与已有 `quadrotor_msgs` 功能重叠)。

**迁移步骤:**

```bash
# 1. 将 SUPER 代码放入 catkin_ws/src/
cp -r /path/to/SUPER ~/catkin_ws/src/SUPER

# 2. 确认所有包已被 catkin build 识别
catkin list  # 应出现 super_planner, rog_map, mission_planner 等

# 3. 编译(增量编译会自动检测新增包)
cd ~/catkin_ws
catkin build
```

**编译特点:**

- `super_planner` 编译为静态库 (`libsuper.a`) 而非共享库,所有符号直接链接到可执行文件中
- 编译过程中有少量 `-Wsign-compare`、`-Wreorder`、`-Wparentheses` 警告,均为上游代码风格问题,不影响功能
- SUPER 内部的 `rog_map` 和 `mission_planner` 作为独立 catkin 包被 `catkin build` 发现并并行编译
- 依赖 `quadrotor_msgs`(已存在于工作空间)、`rog_map`(SUPER 自带)等

**与独立工作空间的区别:**

| 方面 | 独立工作空间 | catkin_ws 统一编译 |
|------|-------------|-------------------|
| 依赖管理 | 需要手动 source 多个 devel 空间 | 所有包依赖自动解析,一次 source |
| 构建隔离 | 依赖可能跨工作空间隐式满足 | catkin build 严格检查,依赖必须显式声明 |
| 并行编译 | 需要手动管理编译顺序 | catkin build 自动按依赖拓扑排序并行编译 |
| 便利性 | 需切换工作空间 | `source ~/catkin_ws/devel/setup.bash` 后全部可用 |

---

## catkin_make vs catkin build 的关键差异

| 特性 | catkin_make | catkin build |
|------|-------------|--------------|
| 构建隔离 | 所有包共享一个 CMake 上下文 | 每个包独立的 CMake 上下文 |
| 依赖声明 | 宽松,隐式依赖也能工作 | 严格,必须显式声明所有依赖 |
| 并行构建 | 单线程 CMake + 多线程 Make | 多包并行构建 |
| 库导出 | 不需要 catkin_package 导出也能被找到 | MUST 在 catkin_package 中正确导出 INCLUDE_DIRS 和 LIBRARIES |
| 构建空间 | build/ (单目录) | build/<package_name>/ (每包独立) |

**迁移要点:**
- 每个包必须在 `package.xml` 中声明所有直接的 build/exec 依赖
- 每个包必须在 `CMakeLists.txt` 的 `find_package(catkin COMPONENTS ...)` 中包含所有直接依赖
- 提供库的包必须在 `catkin_package()` 中取消 `INCLUDE_DIRS` 和 `LIBRARIES` 的注释,正确导出

---

## 工作空间配置文件清单

| 文件 | 用途 |
|------|------|
| `cmake/protobuf-config.cmake` | Anaconda protobuf 拦截桩文件 |
| `scripts/build.sh` | 便捷编译脚本(自动传递 cmake 参数) |
| `.catkin_tools/profiles/default/config.yaml` | catkin build 配置文件 |
| `src/SUPER/super_planner/` | SUPER 核心规划器包 |
| `src/SUPER/rog_map/` | SUPER ROG 地图模块 |
| `src/SUPER/mission_planner/` | SUPER 任务规划器 |
| `src/SUPER/mars_uav_sim/mars_quadrotor_msgs/CATKIN_IGNORE` | 跳过重复的消息包编译 |

---

## 环境信息

- **OS:** Ubuntu 20.04 (Linux 5.15.0)
- **ROS:** Noetic
- **Python:** Anaconda3 Python 3.13.9 (默认) + 系统 Python 3.8
- **CMake:** 3.16.3
- **GCC:** 10.5.0
- **protobuf (系统):** 3.6.1
- **protobuf (Anaconda):** 29.3
- **Ceres:** 1.14.0 (系统 libceres-dev)
- **Gazebo:** 11 + ignition-transport8
