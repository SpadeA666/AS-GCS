# AS-GCS 复现指南

> 面向对象：想在本机跑起这套系统的人，以及**用 AI agent 帮忙配置的人**。
> 如果你打算交给 agent 做，请先读 [第十节](#十给-ai-agent-的说明) —— 那里写了开工前必须确认的三件事。

---

## 一、这是什么

一套无人机自主飞行系统，分两部分：

| 组件 | 包名 | 作用 |
|---|---|---|
| **控制器** | `as_controller` | 封装 PX4 Offboard 接口，提供起飞/定点/规划器导航/视觉跟踪等 API，以及状态机 |
| **地面站** | `as_gcs` | Web 地面站（浏览器访问）：点云建图显示、打点飞行、安全区、目标跟随、遥控器接管 |

规划器用 **SUPER**（主）和 **EGO-Planner**（备选）；定位用 **FAST-LIO**；雷达 **Livox Mid360**。

本仓库（AS-GCS）**只包含这两个核心包**。其余第三方依赖需要你自行准备，见第三节。

---

## 二、环境要求

### 2.1 操作系统

- **Ubuntu 20.04**（本仓库在 20.04.6 上验证）
- **ROS Noetic**
- **Python 3.8**（系统 python3）

> ⚠️ 如果装了 Anaconda，注意它会抢占 `python3`。本文档凡是涉及 ROS 的脚本
> 一律用 `/usr/bin/python3` 显式指定，别用裸 `python3`。

### 2.2 硬件

**仿真端**（PC，就是在你笔记本上跑）：

- x86_64，内存 8GB 以上（Gazebo + ROS 比较吃内存）
- 有独立显卡更流畅，但不必须

**真机端**（可选，飞实际飞机时）：

- 机载计算机：**NVIDIA Jetson Orin NX**（aarch64，Ubuntu 20.04 + ROS Noetic）
- 飞控：**PX4 1.13.2**
- 雷达：**Livox Mid360**（相对机体倾斜安装 15°）
- 相机：**D435i**（前视）+ 单目 USB 相机（下视）

### 2.3 遥控器（想用遥控器接管的话必看）

- **本仓库验证用的型号：RadioMaster TX12**，固件 OpenTX / EdgeTX
- 连接方式：**USB Joystick (HID) 模式** —— 插上后会多出 `/dev/input/js0`
- **其他型号也能用**，但通道映射不一样，需要用 `probe_joy.py` 重新探测（见第九节）

> ⚠️ **换遥控器必须先重新探测通道映射**。本仓库的默认映射是按 TX12 写的，
> 直接套用到别的遥控器上会导致开关错位、摇杆反向。探测方法见 9.4。

---

## 三、依赖清单

### 3.1 ROS 与系统包

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-mavros ros-noetic-mavros-extras \
  ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-image-transport \
  python3-catkin-tools python3-rosdep python3-pip \
  libceres-dev libeigen3-dev libpcl-dev libopencv-dev \
  libboost-all-dev libarmadillo-dev \
  protobuf-compiler libprotobuf-dev
```

装完 mavros 后还要装它的地理围栏数据（**这步很容易漏，漏了 mavros 起不来**）：

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 3.2 Python 包

```bash
# 注意 empy 必须是 3.x —— 4.x 与 ROS Noetic 的 catkin 不兼容
pip3 install empy==3.3.4 catkin_pkg rosdep rosdistro
```

### 3.3 第三方 ROS 包（需自行获取）

本仓库不含以下包，请按对应来源放到 `catkin_ws/src/` 下：

| 包 | 用途 | 来源 |
|---|---|---|
| `Livox-SDK2` | Mid360 驱动所需的 SDK | github.com/Livox-SDK/Livox-SDK2 |
| `livox_ros_driver2` | Mid360 的 ROS 驱动 | github.com/Livox-SDK/livox_ros_driver2 |
| `faster-lio` | 激光雷达惯性里程计（定位） | github.com/gaoxiang12/faster-lio |
| `foxglove_bridge` | 地面站的 WebSocket 桥 | github.com/foxglove/ros-foxglove-bridge |
| `SUPER` | 主规划器（本仓库做了定制） | 见下方说明 |
| `ego_planner` | 备选规划器 | github.com/ZJU-FAST-Lab/ego-planner |
| `yolov8_ros` / `yolov11_ros_msgs` | 视觉检测 | 见下方说明 |
| `lio_to_mavros_main` | LIO 位姿 → MAVROS 桥接 | 本项目配套 |

> **SUPER / yolov8_ros / lio_to_mavros_main 是定制过的**，直接用上游版本不保证兼容。
> 复现时建议向项目作者索取这三者的完整副本。

`Livox-SDK2` 还需要先编译安装（`livox_ros_driver2` 依赖它的静态库）：

```bash
cd ~/catkin_ws/src/Livox-SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

### 3.4 PX4 固件（仿真需要）

```bash
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git PX4_Firmware
cd PX4_Firmware
git checkout v1.13.2
git submodule update --init --recursive
bash Tools/setup/ubuntu.sh
```

> 仿真用的机型模型（`iris_mid360_400` 等）在本项目里做过定制
> （加了 Mid360、D435i、下视相机），需要用项目里的 `Tools/sitl_gazebo/models/` 覆盖。

---

## 四、一键安装

```bash
cd ~/catkin_ws
bash scripts/install_all.sh
```

这个脚本会依次做：

1. **环境检查** —— OS 版本、ROS、Python、catkin_tools、关键 apt 包、第三方包是否到位
2. **安装缺失的依赖** —— Python 包、地理围栏数据（系统包只提示不擅自装）
3. **配置 catkin** —— 按你的实际路径生成 cmake 参数
4. **编译** —— `catkin build`
5. **结果验证** —— 检查关键可执行文件与消息是否生成成功

每一阶段都会打印 `[OK] / [FAIL] / [WARN]`，最后给一份总结。**它不会碰你的遥控器配置**，那部分要手动做（第九节）。

---

## 五、手动安装（不想用脚本时）

### 5.1 配置 catkin

参数里的路径要换成你自己的：

```bash
cd ~/catkin_ws
catkin config --cmake-args \
  -Dprotobuf_DIR=$PWD/cmake \
  -DROS_EDITION=ROS1 \
  -DCeres_DIR=/usr/lib/cmake/Ceres
```

这三个参数各自解决的问题：

- `protobuf_DIR` —— 绕过 Anaconda protobuf 与系统 protobuf 的 target 冲突
- `ROS_EDITION=ROS1` —— 强制 `livox_ros_driver2` 走 ROS1 分支（否则它会去找 ROS2 的 `ament_cmake_auto`）
- `Ceres_DIR` —— 用系统 libceres 避免自编译静态 Ceres 的 glog 链接问题

### 5.2 编译

```bash
cd ~/catkin_ws
catkin build
```

更多编译报错的处理见 `BUILD_GUIDE.md`（9 个已知问题的完整排查记录）。

---

## 六、跑起来

### 6.1 仿真 + 地面站（一键）

```bash
bash ~/catkin_ws/src/as_gcs/ui/restart_all.sh [场景]
# 浏览器打开 http://localhost:5173/
```

场景可选：`raicom`（竞赛小场景）、`indoor1`~`indoor5`、`outdoor1`~`outdoor4` 等。
不带参数默认 `indoor3`。用 `--list` 看全部。

**顺序铁律**：先仿真 → 等 ROS master 就位 → 再地面站。`restart_all.sh` 已经帮你排好了。

> **别用 `sh/raicom.sh` 起仿真**。它是 `set -e` + 裸 `wait` + `trap cleanup EXIT` 的组合，
> 任何一条 launch 抖动就会把所有节点一起杀光。用 `sim_stable.sh`（`restart_all.sh` 内部用的就是它）。

### 6.2 只重启地面站

```bash
bash ~/catkin_ws/src/as_gcs/ui/start-dev.sh
```

它会拉起 bridge + gateway + 前端 + watchdog，并自愈。

### 6.3 停止

```bash
bash ~/catkin_ws/src/as_gcs/ui/cleanup_all.sh
```

---

## 七、地面站怎么用

浏览器打开 **http://localhost:5173/**。连接成功的标志是右上角显示已连接、话题列表有数据。

主要功能：

- **2D 地图打点** —— 左键点图下发目标。分两类：**规划器点**（走 SUPER/EGO 规划）和
  **PX4 点**（直接位置控制），两类点在图上形状不同，防止误发
- **安全区** —— 画多边形 + 设 minZ/maxZ，超出的打点会被拦下（ROS 侧校验）
- **3D 视图** —— 点云、轨迹、安全区棱柱、无人机模型
- **图像面板** —— YOLO 检测框与目标跟随
- **遥控器接管** —— 见第九节

**服务清单**（前端按钮对应的接口，`as_gcs/srv/`）：

```
takeoff / land / fly_up / fly_down / go_to_px4 / go_to_planner / set_planner /
set_nav_mode / start_follow / stop_follow / align_target / set_actuator /
set_geofence / emergency_stop / camera_control / yolo_control
```

**打点没反应时按这个顺序查**（详见 `src/as_gcs/README.md` 的诊断手册）：

1. 连上了吗 → 2. `/gcs/*` 服务在吗 → 3. `tail /tmp/gcs_gateway.log` 看网关收到没 →
4. 是不是在安全区外 → 5. 规划器节点在吗 → 6. 规划器的点云源对不对

---

## 八、控制器接口（`as_controller`）

接口定义在 `src/as_controller/include/api_3d.h`，实现在 `src/api_3d.cpp`。

### 8.1 基础飞行

```cpp
bool takeoff(float height);                                  // 起飞到指定高度
bool position(float x, float y, float z, float yaw, float tol = 0.2f);   // 定点
bool positionSmooth(..., float hover_sec = 0.0f);            // 平滑定点
bool flyUp(float height);      bool flyDown(float descend_z);
bool autoLand();
bool controlYaw(float x, float y, float z, float target_yaw, float wait_sec);
```

### 8.2 规划器导航

```cpp
bool navigationSuper(float x, float y, float z, float yaw, float tol = 0.2f,
                     bool stop_at_goal = false, int nav_mode = -1);   // 用 SUPER 规划
bool navigationEgo(...);                                            // 用 EGO 规划
bool navigationSuperContour(...);                                   // 轮廓控制变体
bool navigationSuperRviz(int nav_mode = -1);                        // 接收 rviz 打点
```

### 8.3 视觉相关

```cpp
bool trackYoloDown(float max_vel = 0.25f, int tol = 30, float lock_hold = 0.3f);
bool trackYoloForward(...);
bool trackYoloing(...);
bool arTrackLanding(...);
```

### 8.4 其他

```cpp
bool pwmControl(int ch5, int ch6, int ch7 = 50);   // 投放机构
bool putShoot(...);  bool putShootSimple(...);  bool putShootPlus(...);
void setNavMode(int m);   int getNavMode() const;
void resetSuperGoal();    // 换新目标前必须调一次，否则新目标不会重新规划
```

### 8.5 nav_mode 语义（**注意与头文件注释相反**）

`nav_mode` 决定 Z 和 Yaw 由谁提供：

- `NAV_FULL(0)` —— Z 和 Yaw 都由规划器给
- `NAV_Z_ONLY(1)` —— Z 由规划器，Yaw 用调用方传入
- `NAV_YAW_ONLY(2)` —— Yaw 由规划器，Z 用调用方传入
- `NAV_LEVEL(3)` —— 两者都用调用方传入

> ⚠️ **头文件注释写的是 `bit0 = Z`，但代码里实际是 `bit1 = Z、bit0 = Yaw`。以代码为准。**

---

## 九、遥控器配置

### 9.1 为什么要额外配置

**QGC 自带的 joystick 发不出 RC 通道**。它走的是 `MANUAL_CONTROL → manual_control_setpoint`，
**永远不会产生 `input_rc` / `rc_channels` / `manual_control_switches`**。所以像
`RC_MAP_KILL_SW`（锁桨）这类功能在仿真里全是死的。

本仓库提供了一个桥 `scripts/joy_rc_bridge.py`，改发 `RC_CHANNELS_OVERRIDE`，
走**和真机接收机完全同构**的链路：

```
遥控器 → /dev/input/js0 → joy_rc_bridge → RC_CHANNELS_OVERRIDE
  → PX4 input_rc → rc_update → rc_channels + manual_control_switches
  → ManualControl → action_request → Commander
```

### 9.2 通道映射（TX12 默认）

| TX12 开关 | js 编号 | RC 通道 | PX4 参数 |
|---|---|---|---|
| 四摇杆 | Axis 0/1/2/3 | ch1/2/3/4 | `RC_MAP_ROLL/PITCH/THROTTLE/YAW` |
| 开关 **C** | Axis 6 | ch7 | `RC_MAP_FLTMODE` |
| 开关 **F** | Axis 7 | ch8 | `RC_MAP_OFFB_SW` |
| 开关 **E** | Button 0 | ch9 | `RC_MAP_KILL_SW` |
| 开关 **B** | Button 1 | ch10 | `RC_MAP_ARM_SW` |

> TX12 上 `ch1-8` 会映射成轴、**`ch9` 以上会被固件二值化成按钮**。所以 E/B 只有两态。

### 9.3 PX4 参数

```
RC_MAP_ROLL=1  RC_MAP_PITCH=2  RC_MAP_THROTTLE=3  RC_MAP_YAW=4
RC_MAP_FLTMODE=7   RC_MAP_OFFB_SW=8
RC_MAP_KILL_SW=9   RC_MAP_ARM_SW=10
COM_FLTMODE1=8     # 自稳（C 低档 -> slot 1）
COM_FLTMODE4=1     # 高度（C 中档 -> slot 4）
COM_FLTMODE6=2     # 定点（C 高档 -> slot 6）
COM_RC_IN_MODE=2   # 让 selector 接受 RC 源
COM_RC_OVERRIDE=3  # AUTO + OFFBOARD 都允许拨杆接管
```

> **PX4 SITL 里 `RC_MAP_*` 默认全是 0（映射禁用）**。不显式设置的话，
> 即使 RC 数据进来了也不会被当成摇杆或开关。

### 9.4 换遥控器怎么办

```bash
/usr/bin/python3 ~/catkin_ws/src/as_gcs/scripts/probe_joy.py 60
```

然后依次拨动每个开关、把摇杆推到极限。脚本会打印**每个开关落在哪个 Axis/Button**。
拿到结果后改 `joy_rc_bridge.py` 顶部的 `AXIS_MAP` / `BUTTON_MAP` 即可（加开关只改一行）。

> **探测技巧**：如果分不清哪个开关是哪个，就按固定顺序拨、**每个之间停 3 秒**，
> 脚本输出的时间戳能帮你把它们切开。

### 9.5 启动桥

```bash
bash ~/catkin_ws/src/as_gcs/scripts/joy_bridge_ctl.sh start    # 起
bash ~/catkin_ws/src/as_gcs/scripts/joy_bridge_ctl.sh status   # 查
bash ~/catkin_ws/src/as_gcs/scripts/joy_bridge_ctl.sh stop     # 停
```

`start-dev.sh` 和 `gcs_watchdog.sh` 已把桥纳进来，正常流程不用手动起。

> ⚠️ **跑桥时要在 QGC 里关掉 joystick**（Vehicle Setup → Joystick → 取消 Enable），
> 否则 QGC 的 `MANUAL_CONTROL` 和桥的 RC 通道会同时生效，摇杆打架。

---

## 十、给 AI Agent 的说明

如果你打算把这份指南交给 agent 执行，**请先让它做下面三件事**，再开始配置。

### 10.1 开工前必须确认的三件事

**① 检查环境**

让 agent 先跑一遍环境自检：

```bash
bash ~/catkin_ws/scripts/install_all.sh --check-only
```

它会报告：OS / ROS / Python / catkin_tools / apt 依赖 / 第三方包 的到位情况。
**在环境没摸清之前不要动手改配置** —— 缺哪个包、路径在哪，都靠这一步确定。

**② 问清遥控器型号**

**这一步不能跳过。** 不同遥控器的 joystick 通道映射完全不同：

- 型号是什么？
- 插上后 `/dev/input/js0` 存不存在？
- 是 USB Joystick 模式还是别的模式？

如果是 **RadioMaster TX12**，可以直接用本仓库的默认映射；
**其他型号必须先用 `probe_joy.py` 重新探测**（见 9.4），否则开关会错位。

**③ 确认工作空间路径**

本仓库的所有脚本默认工作空间是 `~/catkin_ws`、PX4 在 `~/PX4_Firmware`。
**路径不同的话，先改脚本里的路径变量再执行**。

### 10.2 常见坑（agent 尤其容易踩）

1. **`pkill -f "xxx"` 会杀掉自己** —— 执行这条命令的 shell，其命令行里就含那个字符串。
   写 `[x]xx` 也躲不掉（路径里可能有完整词）。**必须写进脚本文件跑**。
2. **ROS master 换代会让节点变孤儿** —— 重启仿真后，进程还活着但 rospy 已失联。
   判据要同时满足两条：**进程存在** 且 **`rosnode list` 里有**。只查端口或只查列表都会被骗。
3. **`rosnode ping` 的退出码在 Noetic 上恒为 0**，不能当判据。
4. **绝不要主动让飞机起飞** —— 地面站的起飞按钮会真的解锁，`takeoff()` 在 `main()` 里就执行。
5. **`empy` 必须是 3.x**，4.x 会让消息生成报 `module 'em' has no attribute 'RAW_OPT'`。
6. **`pkill -f "gcs_gateway_node"` 重启网关前要确认 `armed: False`** —— 它是 OFFBOARD 的
   setpoint 源，飞着的时候重启会触发 PX4 failsafe。

### 10.3 相关文档

- `BUILD_GUIDE.md` —— 编译报错的完整排查（9 个已知问题）
- `src/as_gcs/README.md` —— 地面站的使用、场景切换、排障手册
- `src/as_gcs/SKILL.md`（或在 agent 的 skills 目录下）—— 地面站的深度排查手册
