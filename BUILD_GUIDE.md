# catkin_ws 编译指南与常见问题解决方案

> 最后更新: 2026-10-09
> 构建工具: catkin_tools (`catkin build`)
> ROS 版本: Noetic (Ubuntu 20.04)
> 仓库只含 as_controller 与 as_gcs 两个核心包，
> 其余依赖（mavros、faster_lio、foxglove_bridge、SUPER 等）需自行准备，
> 见 docs/REPRODUCE.md。

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
| `scripts/install_all.sh` | 一键安装与环境自检（`--check-only` 只检查不改动） |
| `.catkin_tools/profiles/default/config.yaml` | catkin build 配置文件 |

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
