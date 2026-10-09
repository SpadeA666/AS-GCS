/**
 * gcs_gateway —— 地面站任务级控制面
 *
 * 设计要点（读之前请先看这段）：
 *
 *  1. 本节点**取代** `asnav_3d_node`（competition_3d.cpp）。两者都会实例化 ASNAV、
 *     订阅/发布同一批话题，**不能同时运行**。
 *
 *  2. 服务回调**只登记命令，不在回调里执行**。
 *     原因：ASNAV 的方法（position/flyUp/takeoff…）内部是阻塞循环，一跑就是几秒。
 *     若在服务回调里直接调用，会把主循环的 setpointPublish 卡住，
 *     offboard 心跳一断，PX4 会退出 offboard 模式——那是要摔飞机的。
 *     所以：回调登记 → 主循环取走执行（阻塞期间 ASNAV 内部自己维持心跳）。
 *
 *  3. 本节点**不会**主动起飞。takeoff 只在 /gcs/takeoff 被显式调用时执行。
 *
 *  4. 仿真话题带 /iris_0 命名空间（api_3d.cpp 里硬编码）。上真机需要改那里
 *     或做 remap，见 PLAN-v2.md。
 */
#include "api_3d.h"
#include <as_gcs/Takeoff.h>
#include <as_gcs/CancelTakeoff.h>
#include <as_gcs/Land.h>
#include <as_gcs/FlyUp.h>
#include <as_gcs/FlyDown.h>
#include <as_gcs/GoToPx4.h>
#include <as_gcs/SetPlanner.h>
#include <as_gcs/SetNavMode.h>
#include <as_gcs/StartFollow.h>
#include <as_gcs/StopFollow.h>
#include <as_gcs/AlignTarget.h>
#include <as_gcs/SetActuator.h>
#include <as_gcs/SetGeofence.h>
#include <as_gcs/EmergencyStop.h>
#include <as_gcs/GcsHeartbeat.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <locale.h>
#include <std_msgs/String.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace {

enum class CmdType {
  NONE,
  TAKEOFF,
  LAND,
  FLY_UP,
  FLY_DOWN,
  GO_TO_PX4,
  /** 交给 EGO/SUPER 规划器导航（落点 ASNAV::navigationSuper / navigationEgo） */
  GO_TO_PLANNER,
  START_FOLLOW,
  ALIGN_TARGET,
  STOP_FOLLOW,
  ACTUATOR,
  EMERGENCY,
  ABORT,
  SET_PLANNER,
};

struct Command {
  CmdType type = CmdType::NONE;
  float a = 0, b = 0, c = 0, d = 0;
  int i = 0, j = 0, k = 0;
  std::string s;
};

/** 规划器选择：决定 GoTo 时走哪条导航接口 */
enum class Planner { SUPER, EGO };

/**
 * 当前任务。关键：所有任务都是**非阻塞**的——每帧重新设一次目标，
 * 由主循环统一发 setpoint。ASNAV::position() 本身不阻塞（只设目标立即返回），
 * 如果按 takeoff() 那套写法中间夹着“只 spin 不发流”的等待，
 * OFFBOARD 会因为 setpoint 断流而被 PX4 撤销。
 */
enum class Task { IDLE, TAKEOFF_CLIMB, GOTO, GOTO_PLANNER };

/** 起飞前的前置阶段 */
enum class PreFlight { NONE, FLOW, WAIT_OFFBOARD, WAIT_ARM, READY };

}  // namespace

class GcsGateway {
 public:
  explicit GcsGateway(ros::NodeHandle& nh)
      : nh_(nh), uav_(nh) {
    // ── 服务注册：回调一律只登记，不执行 ──
    srv_takeoff_ = nh_.advertiseService("/gcs/takeoff", &GcsGateway::onTakeoff, this);
    srv_cancel_takeoff_ =
        nh_.advertiseService("/gcs/cancel_takeoff", &GcsGateway::onCancelTakeoff, this);
    srv_land_ = nh_.advertiseService("/gcs/land", &GcsGateway::onLand, this);
    srv_fly_up_ = nh_.advertiseService("/gcs/fly_up", &GcsGateway::onFlyUp, this);
    srv_fly_down_ = nh_.advertiseService("/gcs/fly_down", &GcsGateway::onFlyDown, this);
    srv_goto_ = nh_.advertiseService("/gcs/go_to_px4", &GcsGateway::onGoToPx4, this);
    srv_goto_planner_ =
        nh_.advertiseService("/gcs/go_to_planner", &GcsGateway::onGoToPlanner, this);
    srv_planner_ = nh_.advertiseService("/gcs/set_planner", &GcsGateway::onSetPlanner, this);
    srv_nav_mode_ = nh_.advertiseService("/gcs/set_nav_mode", &GcsGateway::onSetNavMode, this);
    srv_start_follow_ = nh_.advertiseService("/gcs/start_follow", &GcsGateway::onStartFollow, this);
    srv_stop_follow_ = nh_.advertiseService("/gcs/stop_follow", &GcsGateway::onStopFollow, this);
    srv_align_ = nh_.advertiseService("/gcs/align_target", &GcsGateway::onAlignTarget, this);
    srv_actuator_ = nh_.advertiseService("/gcs/set_actuator", &GcsGateway::onSetActuator, this);
    srv_geofence_ = nh_.advertiseService("/gcs/set_geofence", &GcsGateway::onSetGeofence, this);
    srv_estop_ = nh_.advertiseService("/gcs/emergency_stop", &GcsGateway::onEmergency, this);

    // 规划器切换状态：latch=true，后连上来的前端也能立即拿到当前状态
    planner_status_pub_ = nh_.advertise<std_msgs::String>("/gcs/planner_status", 1, true);
    {
      std_msgs::String m;
      m.data = "ready " + std::string(planner_ == Planner::SUPER ? "super" : "ego");
      planner_status_pub_.publish(m);
    }

    // 心跳：地面站周期性发，超时则视为失联
    sub_heartbeat_ = nh_.subscribe("/gcs/heartbeat", 5, &GcsGateway::onHeartbeat, this);

    // 解锁 / 模式切换。
    // ASNAV 内部**没有** arm 调用，takeoff() 只是等待 armed && OFFBOARD，
    // 所以必须由网关把飞机先叫醒。话题前缀与 api_3d.cpp 保持一致（仿真带 /iris_0）。
    arming_cli_ = nh_.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");
    set_mode_cli_ = nh_.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
    state_sub_ = nh_.subscribe("/iris_0/mavros/state", 10, &GcsGateway::onState, this);
    pose_sub_ = nh_.subscribe("/iris_0/mavros/local_position/pose", 10, &GcsGateway::onPose, this);

    // 执行机构当前值：pwmControl 三通道必须一起下发，所以必须记住上次的值。
    // M7 默认 50(中位)，若调舵机时不带上当前激光值，会把激光关掉。
    act_[0] = 50;
    act_[1] = 50;
    act_[2] = 50;

    last_heartbeat_ = ros::Time::now();
    ROS_INFO("[gcs_gateway] 已就绪。服务前缀 /gcs/，规划器当前: SUPER");
    ROS_INFO("[gcs_gateway] 注意：本节点会占用 ASNAV，勿与 asnav_3d_node 同时运行");
  }

  void spin() {
    ros::Rate rate(50);
    while (ros::ok()) {
      Command cmd;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd = pending_;
        pending_ = Command{};
      }
      if (cmd.type != CmdType::NONE) beginCommand(cmd);

      // 非阻塞推进：每帧只刷新目标，不在这里等
      if (preflight_ != PreFlight::NONE) runPreflight();
      runTask();
      pollPlannerSwitch();

      // 关键：每帧都发 setpoint。OFFBOARD 靠的就是这个心跳，
      // 一旦中间断流超过 ~0.5s，PX4 就会退出 OFFBOARD 并上锁。
      uav_.setpointPublish();
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  // ────────── 命令受理（只登记/启任务） ──────────

  void beginCommand(const Command& c) {
    switch (c.type) {
      case CmdType::TAKEOFF:
        target_z_ = c.a;
        preflight_ = PreFlight::FLOW;
        preflight_ticks_ = 0;
        task_ = Task::IDLE;
        ROS_WARN("[gcs_gateway] 起飞序列开始，目标高度 %.2f m", c.a);
        break;

      case CmdType::GO_TO_PX4:
        tx_ = c.a;
        ty_ = c.b;
        tz_ = c.c;
        tyaw_ = c.d;
        ttol_ = 0.2f;
        if (!mav_state_.armed) {
          ROS_ERROR("[gcs_gateway] 未解锁，忽略位置目标（先起飞）");
          break;
        }
        task_ = Task::GOTO;
        ROS_INFO("[gcs_gateway] 位置目标 → (%.2f, %.2f, %.2f)", tx_, ty_, tz_);
        break;

      case CmdType::GO_TO_PLANNER:
        tx_ = c.a;
        ty_ = c.b;
        tz_ = c.c;
        tyaw_ = c.d;
        ttol_ = 0.3f;  // 绕障后的收敛容差放宽一点
        if (!mav_state_.armed) {
          ROS_ERROR("[gcs_gateway] 未解锁，忽略规划目标（先起飞）");
          break;
        }
        // 关键：清掉 navigationSuper 内部“只发一次 goal”的门闩。
        // 不清的话，飞机还在飞向旧目标时改打新点，goal 不会被重新发布。
        uav_.resetSuperGoal();
        task_ = Task::GOTO_PLANNER;
        ROS_INFO("[gcs_gateway] 规划目标 → (%.2f, %.2f, %.2f)，规划器 %s", tx_, ty_, tz_,
                 planner_ == Planner::SUPER ? "SUPER" : "EGO");
        break;

      case CmdType::FLY_UP:
        if (!mav_state_.armed) {
          ROS_ERROR("[gcs_gateway] 未解锁，忽略上升");
          break;
        }
        tx_ = cur_x_;
        ty_ = cur_y_;
        tz_ = cur_z_ + c.a;
        tyaw_ = cur_yaw_;
        ttol_ = 0.15f;
        task_ = Task::GOTO;
        ROS_INFO("[gcs_gateway] 上升到 %.2f m", tz_);
        break;

      case CmdType::FLY_DOWN:
        if (!mav_state_.armed) {
          ROS_ERROR("[gcs_gateway] 未解锁，忽略下降");
          break;
        }
        tx_ = cur_x_;
        ty_ = cur_y_;
        tz_ = std::max(0.15f, cur_z_ - c.a);
        tyaw_ = cur_yaw_;
        ttol_ = 0.15f;
        task_ = Task::GOTO;
        ROS_INFO("[gcs_gateway] 下降到 %.2f m", tz_);
        break;

      case CmdType::ACTUATOR: {
        {
          std::lock_guard<std::mutex> lk(mtx_);
          act_[0] = c.i;
          act_[1] = c.j;
          act_[2] = c.k;
        }
        ROS_INFO("[gcs_gateway] 舵机/激光 M5=%d M6=%d M7=%d", c.i, c.j, c.k);
        uav_.pwmControl(c.i, c.j, c.k); // 内部自带短循环 + setpointPublish，安全
        break;
      }

      case CmdType::LAND:
        ROS_WARN("[gcs_gateway] 执行降落");
        task_ = Task::IDLE;
        preflight_ = PreFlight::NONE;
        uav_.autoLand();
        break;

      case CmdType::STOP_FOLLOW:
        ROS_INFO("[gcs_gateway] 停止跟随 / 清目标");
        uav_.reset_target();
        break;

      case CmdType::START_FOLLOW:
        task_ = Task::IDLE;
        if (c.i == 1) {
          ROS_INFO("[gcs_gateway] 下视跟随 class=%s", c.s.c_str());
          uav_.trackYoloDown();
        } else {
          ROS_INFO("[gcs_gateway] 前视跟随 class=%s", c.s.c_str());
          uav_.trackYoloForward(0.01f, 0.005f, 0.005f, 80.0f, 20, 20);
        }
        break;

      case CmdType::ALIGN_TARGET:
        task_ = Task::IDLE;
        ROS_INFO("[gcs_gateway] 目标对齐 class=%s", c.s.c_str());
        uav_.trackYoloing(0.01f, 0.005f, 0.005f, 80.0f, 20, 20);
        break;

      case CmdType::EMERGENCY:
        ROS_ERROR("[gcs_gateway] *** 急停：%s ***", c.s.c_str());
        task_ = Task::IDLE;
        preflight_ = PreFlight::NONE;
        uav_.reset_target();
        uav_.set_mode("AUTO.LOITER");
        break;

      case CmdType::ABORT:
        // 取消进行中的任务（当前用于取消起飞）。
        // 分三种情况，区别在“有没有已经解锁” —— 没解锁就干净退出，
        // 解锁了才需要切模式让飞机保持安全状态。
        if (preflight_ != PreFlight::NONE) {
          const bool was_armed = mav_state_.armed;
          preflight_ = PreFlight::NONE;
          preflight_ticks_ = 0;
          task_ = Task::IDLE;
          if (was_armed) {
            // 已经 arm 了，切 LOITER 让 PX4 自己悬停。
            // 取消后网关不再对该目标负责，留在 OFFBOARD 反而更难处理。
            uav_.set_mode("AUTO.LOITER");
            ROS_WARN("[gcs_gateway] 起飞已取消：序列中止，切 AUTO.LOITER 悬停");
          } else {
            ROS_WARN("[gcs_gateway] 起飞已取消：序列中止（未解锁，未 arm）");
          }
        } else if (task_ == Task::TAKEOFF_CLIMB) {
          task_ = Task::IDLE;
          uav_.set_mode("AUTO.LOITER");
          ROS_WARN("[gcs_gateway] 起飞已取消：停止爬升（当前高度 %.2f m），切 AUTO.LOITER",
                   cur_z_);
        } else {
          ROS_INFO("[gcs_gateway] 取消请求收到，但当前没有进行中的任务");
        }
        break;

      case CmdType::SET_PLANNER:
        // 切换在独立进程里做（要启停 roslaunch，会阻塞好几秒），
        // 所以这里只负责起脚本 + 置 pending，状态由 pollPlannerSwitch() 轮询推进。
        {
          const std::string who = c.s;
          if (!std::ifstream(kSwitchScript.c_str()).good()) {
            ROS_ERROR("[gcs_gateway] 找不到切换脚本: %s", kSwitchScript.c_str());
            publishPlannerStatus("failed", who);
            break;
          }
          // 清掉上一次的状态文件，避免把旧结果当成新结果
          std::remove(kPlannerStateFile);
          const std::string cmd =
              "bash " + kSwitchScript + " " + who + " > /dev/null 2>&1 &\n";
          const int rc = std::system(cmd.c_str());
          if (rc != 0) {
            ROS_ERROR("[gcs_gateway] 拉起切换脚本失败 rc=%d", rc);
            publishPlannerStatus("failed", who);
            break;
          }
          planner_pending_ = who;
          publishPlannerStatus("switching", who);
          ROS_WARN("[gcs_gateway] 已启动规划器切换脚本: -> %s", who.c_str());
        }
        break;

      default:
        break;
    }
  }

  /**
   * 起飞前置：建流 → 切 OFFBOARD → 解锁 → 交给爬升任务。
   * 全程由主循环驱动（每帧仍在发 setpoint），所以才不会断流。
   */
  void runPreflight() {
    preflight_ticks_++;
    switch (preflight_) {
      case PreFlight::FLOW:
        // 关键：先调一次 position() 保持当前位置。
        // ASNAV 的 target_position.type_mask 只在 position() 里赋值，
        // 从没调过的话它是默认 0 —— 那意味着“位置和速度都有效”，
        // PX4 会认为 setpoint 自相矛盾而拒绝进入 OFFBOARD。
        uav_.position(cur_x_, cur_y_, cur_z_, cur_yaw_, 0.5f);
        // 主循环已经在发流了，这里只是等一会儿让它稳定（~1.5s）
        if (preflight_ticks_ > 75) {
          if (mav_state_.mode != "OFFBOARD") {
            mavros_msgs::SetMode m;
            m.request.custom_mode = "OFFBOARD";
            if (set_mode_cli_.call(m) && m.response.mode_sent) {
              ROS_INFO("[gcs_gateway] 已请求 OFFBOARD");
            } else {
              ROS_WARN("[gcs_gateway] OFFBOARD 请求未确认");
            }
          }
          preflight_ = PreFlight::WAIT_OFFBOARD;
          preflight_ticks_ = 0;
        }
        break;

      case PreFlight::WAIT_OFFBOARD:
        // 继续维持 setpoint（含正确的 type_mask），否则流一断更切不过去
        uav_.position(cur_x_, cur_y_, cur_z_, cur_yaw_, 0.5f);
        if (mav_state_.mode == "OFFBOARD") {
          ROS_INFO("[gcs_gateway] OFFBOARD 已生效");
          preflight_ = PreFlight::WAIT_ARM;
          preflight_ticks_ = 0;
        } else if (preflight_ticks_ > 150) { // 3s
          ROS_ERROR("[gcs_gateway] OFFBOARD 切换超时（当前 mode=%s），中止",
                    mav_state_.mode.c_str());
          preflight_ = PreFlight::NONE;
        }
        break;

      case PreFlight::WAIT_ARM:
        uav_.position(cur_x_, cur_y_, cur_z_, cur_yaw_, 0.5f);
        if (!mav_state_.armed && preflight_ticks_ == 1) {
          mavros_msgs::CommandBool a;
          a.request.value = true;
          if (arming_cli_.call(a) && a.response.success) {
            ROS_WARN("[gcs_gateway] *** 已解锁 (ARMED) ***");
          } else {
            ROS_ERROR("[gcs_gateway] 解锁请求失败");
          }
        }
        if (mav_state_.armed) {
          ROS_WARN("[gcs_gateway] 起飞：上升到 %.2f m", target_z_);
          preflight_ = PreFlight::NONE;
          task_ = Task::TAKEOFF_CLIMB;
        } else if (preflight_ticks_ > 150) {
          ROS_ERROR("[gcs_gateway] 解锁超时，中止起飞");
          preflight_ = PreFlight::NONE;
        }
        break;

      default:
        break;
    }
  }

  /** 非阻塞任务推进：每帧重设目标，不看是否阻塞 */
  void runTask() {
    switch (task_) {
      case Task::TAKEOFF_CLIMB:
        uav_.position(0.0f, 0.0f, target_z_, cur_yaw_, 0.15f);
        if (std::fabs(cur_z_ - target_z_) < 0.15f) {
          task_ = Task::IDLE;
          ROS_WARN("[gcs_gateway] 已达目标高度 %.2f m，悬停", target_z_);
        }
        break;

      case Task::GOTO:
        if (uav_.position(tx_, ty_, tz_, tyaw_, ttol_)) {
          task_ = Task::IDLE;
          ROS_INFO("[gcs_gateway] 已到达目标 (%.2f, %.2f, %.2f)", tx_, ty_, tz_);
        }
        break;

      // ── 规划器导航 ──
      // 这是之前缺失的一环：ASNAV 里的 super_cmd_ / ego_cmd_ 只有
      // navigationSuper() / navigationEgo() 会去用，而 gateway 一直只调 position()，
      // 于是规划器算出的轨迹无人执行 —— 表现就是“SUPER 点打下去不飞”。
      // 这两个接口都是非阻塞的（每帧重设目标），内部自己发 goal、跟踪轨迹、
      // 判断到达，可以直接放在 50Hz 主循环里调。
      case Task::GOTO_PLANNER:
        if (planner_ == Planner::SUPER) {
          if (uav_.navigationSuper(tx_, ty_, tz_, tyaw_, ttol_)) {
            // 到达后必须用【位置控制】把当前位置锁住。
            // 只把 task_ 置 IDLE 是不够的：navigationSuper 在 yaw 收尾（以及轨迹完成后
            // 的位置锁存）阶段用的是“零速度”掩码（IGNORE_PX | IGNORE_PY），
            // 那个 setpoint 会一直留在 target_position 里被主循环每帧重发，
            // 飞机在速度控制下没有位置反馈，就会慢慢漂走 —— 即“判定到了却扣不住”。
            uav_.position(cur_x_, cur_y_, cur_z_, cur_yaw_, 0.5f);
            task_ = Task::IDLE;
            ROS_INFO("[gcs_gateway] SUPER 已到达规划目标 (%.2f, %.2f, %.2f)，已位置锁存", tx_, ty_, tz_);
          }
        } else {
          if (uav_.navigationEgo(tx_, ty_, tz_, tyaw_, ttol_)) {
            uav_.position(cur_x_, cur_y_, cur_z_, cur_yaw_, 0.5f);
            task_ = Task::IDLE;
            ROS_INFO("[gcs_gateway] EGO 已到达规划目标 (%.2f, %.2f, %.2f)，已位置锁存", tx_, ty_, tz_);
          }
        }
        break;

      default:
        break;
    }
  }
  // ─────────── 服务回调：只登记 ───────────

  bool onTakeoff(as_gcs::Takeoff::Request& req, as_gcs::Takeoff::Response& res) {
    enqueue([&](Command& c) { c.type = CmdType::TAKEOFF; c.a = req.height; });
    res.success = true;
    res.message = "已受理（起飞为不可逆动作，请确认现场安全）";
    ROS_WARN("[gcs_gateway] 收到起飞请求 h=%.2f", req.height);
    return true;
  }

  bool onCancelTakeoff(as_gcs::CancelTakeoff::Request&, as_gcs::CancelTakeoff::Response& res) {
    // 判断当前是否真有进行中的起飞，好给前端一个准确答复
    const bool in_preflight = (preflight_ != PreFlight::NONE);
    const bool climbing = (task_ == Task::TAKEOFF_CLIMB);

    if (!in_preflight && !climbing) {
      res.success = false;
      res.message = "当前没有进行中的起飞（可能已到达高度，或还没点击起飞）";
      ROS_INFO("[gcs_gateway] 取消起飞：无进行中的序列");
      return true;
    }

    enqueue([&](Command& c) { c.type = CmdType::ABORT; c.s = "cancel_takeoff"; });

    if (in_preflight) {
      res.success = true;
      res.message = mav_state_.armed ? "已受理：将中止起飞序列并切悬停（注意已解锁）"
                                     : "已受理：将中止起飞序列（尚未解锁，不会 arm）";
    } else {
      res.success = true;
      res.message = "已受理：将停止爬升并在当前高度悬停";
    }
    ROS_WARN("[gcs_gateway] 收到取消起飞请求（preflight=%d, climbing=%d）",
             static_cast<int>(in_preflight), static_cast<int>(climbing));
    return true;
  }

  bool onLand(as_gcs::Land::Request&, as_gcs::Land::Response& res) {
    enqueue([&](Command& c) { c.type = CmdType::LAND; });
    res.success = true;
    res.message = "已受理";
    return true;
  }

  bool onFlyUp(as_gcs::FlyUp::Request& req, as_gcs::FlyUp::Response& res) {
    enqueue([&](Command& c) { c.type = CmdType::FLY_UP; c.a = req.delta; });
    res.success = true;
    res.message = "已受理";
    return true;
  }

  bool onFlyDown(as_gcs::FlyDown::Request& req, as_gcs::FlyDown::Response& res) {
    enqueue([&](Command& c) { c.type = CmdType::FLY_DOWN; c.a = req.delta; });
    res.success = true;
    res.message = "已受理";
    return true;
  }

  bool onGoToPx4(as_gcs::GoToPx4::Request& req, as_gcs::GoToPx4::Response& res) {
    if (!checkGeofence(req.position.x, req.position.y, req.position.z, res.message)) {
      res.success = false;
      ROS_WARN("[gcs_gateway] PX4 目标点被安全区拒绝: (%.2f, %.2f, %.2f)", req.position.x,
               req.position.y, req.position.z);
      return true;
    }
    enqueue([&](Command& c) {
      c.type = CmdType::GO_TO_PX4;
      c.a = req.position.x;
      c.b = req.position.y;
      c.c = req.position.z;
      c.d = req.yaw;
    });
    res.success = true;
    res.message = "已受理";
    return true;
  }

  /**
   * 规划器导航：把目标交给 EGO/SUPER（由 set_planner 选的那个）。
   * 与 onGoToPx4 的区别：这里不做直接位置控制，而是让规划器绕障后自己发轨迹，
   * 由 ASNAV::navigationSuper / navigationEgo 跟踪。
   * 复用 GoToPx4.srv 的字段（position/yaw/tol）——语义相同，不另建服务类型。
   */
  bool onGoToPlanner(as_gcs::GoToPx4::Request& req, as_gcs::GoToPx4::Response& res) {
    if (!checkGeofence(req.position.x, req.position.y, req.position.z, res.message)) {
      res.success = false;
      ROS_WARN("[gcs_gateway] 规划目标点被安全区拒绝: (%.2f, %.2f, %.2f)", req.position.x,
               req.position.y, req.position.z);
      return true;
    }
    enqueue([&](Command& c) {
      c.type = CmdType::GO_TO_PLANNER;
      c.a = req.position.x;
      c.b = req.position.y;
      c.c = req.position.z;
      c.d = req.yaw;
    });
    res.success = true;
    res.message = planner_ == Planner::SUPER ? "已受理（SUPER 规划）" : "已受理（EGO 规划）";
    return true;
  }

  /**
   * 设置 Z / Yaw 的控制来源（地面站的两个开关）。写进 ASNAV 的 nav_default_mode_，
   * navigationSuper/Ego 的 nav_mode 形参传 -1 时就会用它。
   *   0 NAV_FULL     两者都由规划器给
   *   1 NAV_Z_ONLY   Z 规划器 / Yaw 用填的
   *   2 NAV_YAW_ONLY Yaw 规划器 / Z 用填的
   *   3 NAV_LEVEL    两者都用填的
   */
  bool onSetNavMode(as_gcs::SetNavMode::Request& req, as_gcs::SetNavMode::Response& res) {
    const int m = req.nav_mode;
    if (m < 0 || m > 3) {
      res.success = false;
      res.message = "nav_mode 必须在 0~3 之间";
      return true;
    }
    uav_.setNavMode(m);
    static const char* kNames[] = {"Z:规划器 Yaw:规划器", "Z:规划器 Yaw:用打点的",
                                   "Z:用打点的 Yaw:规划器", "Z:用打点的 Yaw:用打点的"};
    res.success = true;
    res.message = std::string("已切换: ") + kNames[m];
    ROS_WARN("[gcs_gateway] 导航轴模式 → %d (%s)", m, kNames[m]);
    return true;
  }

  bool onSetPlanner(as_gcs::SetPlanner::Request& req, as_gcs::SetPlanner::Response& res) {
    if (req.planner != "ego" && req.planner != "super") {
      res.success = false;
      res.message = "未知规划器: " + req.planner + "（应为 ego 或 super）";
      return true;
    }

    // 飞行中不允许切：切规划器要停掉正在跑的节点，飞机还在空中时做这个很危险
    if (mav_state_.armed) {
      res.success = false;
      res.message = "飞行中禁止切换规划器，请先降落并上锁";
      ROS_WARN("[gcs_gateway] 拒绝切换规划器：当前已解锁");
      return true;
    }

    if (!planner_pending_.empty()) {
      res.success = false;
      res.message = "正在切换到 " + planner_pending_ + "，请等待完成";
      return true;
    }

    const bool want_ego = (req.planner == "ego");
    const bool already = (want_ego && planner_ == Planner::EGO) ||
                         (!want_ego && planner_ == Planner::SUPER);
    if (already) {
      res.success = true;
      res.message = "当前已经是 " + req.planner + "，无需切换";
      return true;
    }

    enqueue([&](Command& c) {
      c.type = CmdType::SET_PLANNER;
      c.s = req.planner;
    });
    res.success = true;
    res.message = "已受理，正在切换到 " + req.planner + "（停旧节点 + 启新节点，约 10s）";
    ROS_WARN("[gcs_gateway] 收到规划器切换请求: -> %s", req.planner.c_str());
    return true;
  }

  /**
   * 轮询切换脚本写的状态文件，推进 planner_pending_ 状态机并广播给前端。
   * 切换在独立进程（switch_planner.sh）里做，不阻塞主循环的 setpoint 心跳。
   */
  void pollPlannerSwitch() {
    if (planner_pending_.empty()) {
      return;
    }
    std::ifstream f(kPlannerStateFile);
    if (!f.is_open()) {
      return;
    }
    std::string line;
    if (!std::getline(f, line)) {
      return;
    }

    if (line.rfind("ready", 0) == 0) {
      planner_ = (planner_pending_ == "ego") ? Planner::EGO : Planner::SUPER;
      ROS_WARN("[gcs_gateway] 规划器切换完成: %s", planner_pending_.c_str());
      publishPlannerStatus("ready", planner_pending_);
      planner_pending_.clear();
    } else if (line.rfind("failed", 0) == 0) {
      ROS_ERROR("[gcs_gateway] 规划器切换失败: %s", line.c_str());
      publishPlannerStatus("failed", planner_pending_);
      planner_pending_.clear();
    }
  }

  void publishPlannerStatus(const std::string& state, const std::string& who) {
    if (!planner_status_pub_) {
      return;
    }
    std_msgs::String m;
    m.data = state + " " + who;
    planner_status_pub_.publish(m);
  }

  bool onStartFollow(as_gcs::StartFollow::Request& req, as_gcs::StartFollow::Response& res) {
    enqueue([&](Command& c) {
      c.type = CmdType::START_FOLLOW;
      c.i = req.mode;
      c.s = req.target_class;
    });
    res.success = true;
    res.message = "已受理";
    return true;
  }

  bool onStopFollow(as_gcs::StopFollow::Request&, as_gcs::StopFollow::Response& res) {
    enqueue([&](Command& c) { c.type = CmdType::STOP_FOLLOW; });
    res.success = true;
    res.message = "已受理";
    return true;
  }

  bool onAlignTarget(as_gcs::AlignTarget::Request& req, as_gcs::AlignTarget::Response& res) {
    enqueue([&](Command& c) {
      c.type = CmdType::ALIGN_TARGET;
      c.s = req.target_class;
    });
    res.success = true;
    res.message = "已受理";
    return true;
  }

  /** 舵机/激光：-1 表示保持当前值（三通道必须一起下发） */
  bool onSetActuator(as_gcs::SetActuator::Request& req, as_gcs::SetActuator::Response& res) {
    const int ch[3] = {
        req.channel_5 < 0 ? act_[0] : req.channel_5,
        req.channel_6 < 0 ? act_[1] : req.channel_6,
        req.channel_7 < 0 ? act_[2] : req.channel_7,
    };
    for (int i = 0; i < 3; i++) {
      if (ch[i] < 0 || ch[i] > 100) {
        res.success = false;
        res.message = "通道值必须为 0~100 或 -1(保持)";
        return true;
      }
    }
    enqueue([&](Command& c) {
      c.type = CmdType::ACTUATOR;
      c.i = ch[0];
      c.j = ch[1];
      c.k = ch[2];
    });
    res.success = true;
    res.message = "已受理";
    res.applied_5 = ch[0];
    res.applied_6 = ch[1];
    res.applied_7 = ch[2];
    return true;
  }

  bool onSetGeofence(as_gcs::SetGeofence::Request& req, as_gcs::SetGeofence::Response& res) {
    std::lock_guard<std::mutex> lk(mtx_);
    geofence_.enabled = req.enable;
    geofence_.polygon.clear();
    for (const auto& p : req.polygon) geofence_.polygon.emplace_back(p.x, p.y);
    geofence_.z_min = req.z_min;
    geofence_.z_max = req.z_max;

    if (!req.enable) {
      res.success = true;
      res.message = "安全区已关闭";
      ROS_WARN("[gcs_gateway] 安全区关闭");
      return true;
    }
    if (geofence_.polygon.size() < 3) {
      res.success = false;
      res.message = "多边形至少需要 3 个顶点";
      geofence_.enabled = false;
      return true;
    }
    if (geofence_.z_max <= geofence_.z_min) {
      res.success = false;
      res.message = "z_max 必须大于 z_min";
      geofence_.enabled = false;
      return true;
    }
    res.success = true;
    res.message = "安全区已启用（" + std::to_string(geofence_.polygon.size()) +
                  " 边形，高度 " + std::to_string(geofence_.z_min) + "~" +
                  std::to_string(geofence_.z_max) + " m）";
    ROS_WARN("[gcs_gateway] %s", res.message.c_str());
    return true;
  }

  bool onEmergency(as_gcs::EmergencyStop::Request& req, as_gcs::EmergencyStop::Response& res) {
    enqueue([&](Command& c) {
      c.type = CmdType::EMERGENCY;
      c.s = req.action;
    });
    res.success = true;
    res.message = "急停已受理（action=" + req.action + "）";
    ROS_WARN("[gcs_gateway] *** 急停请求 action=%s ***", req.action.c_str());
    return true;
  }

  void onHeartbeat(const as_gcs::GcsHeartbeat::ConstPtr&) {
    last_heartbeat_ = ros::Time::now();
    if (link_lost_) {
      ROS_INFO("[gcs_gateway] 地面站链路恢复");
      link_lost_ = false;
    }
  }

  void onState(const mavros_msgs::State::ConstPtr& msg) {
    mav_state_ = *msg;
    have_state_ = true;
  }

  void onPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    cur_x_ = msg->pose.position.x;
    cur_y_ = msg->pose.position.y;
    cur_z_ = msg->pose.position.z;
    cur_yaw_ = tf::getYaw(msg->pose.orientation);
  }

  /**
   * 叫醒飞机：先建立 setpoint 流 → 切 OFFBOARD → 解锁。
   *
   * 顺序不能颠倒：PX4 切 OFFBOARD 前必须已经在收到 setpoint 流，
   * 否则模式切换会被拒。
   */
  bool prepareOffboard() {
    // 1) 等 mavros 连上飞控
    ros::Rate r(20);
    for (int i = 0; i < 100 && ros::ok(); i++) {
      if (have_state_ && mav_state_.connected) break;
      ROS_INFO_THROTTLE(1.0, "[gcs_gateway] 等待飞控连接…");
      ros::spinOnce();
      r.sleep();
    }
    if (!have_state_ || !mav_state_.connected) {
      ROS_ERROR("[gcs_gateway] 飞控未连接，取消起飞");
      return false;
    }

    // 2) 先建立 2 秒 setpoint 流（OFFBOARD 的前置条件）
    ROS_INFO("[gcs_gateway] 建立 setpoint 流…");
    for (int i = 0; i < 40 && ros::ok(); i++) {
      uav_.setpointPublish();
      ros::spinOnce();
      r.sleep();
    }

    // 3) 切 OFFBOARD
    if (mav_state_.mode != "OFFBOARD") {
      mavros_msgs::SetMode m;
      m.request.custom_mode = "OFFBOARD";
      if (set_mode_cli_.call(m) && m.response.mode_sent) {
        ROS_INFO("[gcs_gateway] 已请求 OFFBOARD");
      } else {
        ROS_WARN("[gcs_gateway] OFFBOARD 切换失败（继续尝试解锁）");
      }
      for (int i = 0; i < 20 && ros::ok(); i++) {
        uav_.setpointPublish();
        ros::spinOnce();
        r.sleep();
      }
    }

    // 4) 解锁
    if (!mav_state_.armed) {
      mavros_msgs::CommandBool a;
      a.request.value = true;
      if (arming_cli_.call(a) && a.response.success) {
        ROS_WARN("[gcs_gateway] *** 已解锁 (ARMED) ***");
      } else {
        ROS_ERROR("[gcs_gateway] 解锁失败，取消起飞");
        return false;
      }
      for (int i = 0; i < 20 && ros::ok(); i++) {
        uav_.setpointPublish();
        ros::spinOnce();
        r.sleep();
      }
    }

    ROS_INFO("[gcs_gateway] 当前: armed=%d mode=%s", (int)mav_state_.armed,
             mav_state_.mode.c_str());
    return true;
  }

  /** 安全区校验：必须在 ROS 侧强制，前端校验只是体验 */
  bool checkGeofence(float x, float y, float z, std::string& why) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!geofence_.enabled) return true;

    if (z < geofence_.z_min || z > geofence_.z_max) {
      why = "高度 " + std::to_string(z) + " 超出安全区 [" + std::to_string(geofence_.z_min) +
            ", " + std::to_string(geofence_.z_max) + "]";
      return false;
    }
    // 射线法判断点是否在多边形内
    bool inside = false;
    const auto& poly = geofence_.polygon;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
      const auto& pi = poly[i];
      const auto& pj = poly[j];
      if (((pi.second > y) != (pj.second > y)) &&
          (x < (pj.first - pi.first) * (y - pi.second) / (pj.second - pi.second) + pi.first)) {
        inside = !inside;
      }
    }
    if (!inside) {
      why = "目标点 (" + std::to_string(x) + ", " + std::to_string(y) + ") 在安全区外";
      return false;
    }
    return true;
  }

  template <typename F>
  void enqueue(F&& fill) {
    std::lock_guard<std::mutex> lk(mtx_);
    fill(pending_);
  }

  // ─────────── 成员 ───────────
  ros::NodeHandle nh_;
  ASNAV uav_;

  ros::ServiceServer srv_takeoff_, srv_cancel_takeoff_, srv_land_, srv_fly_up_, srv_fly_down_;

  // ────────── 规划器切换（SUPER <-> EGO）──────────
  /// 切换脚本路径（在 catkin_ws/scripts/ 下）
  const std::string kSwitchScript =
      std::string(getenv("HOME") ? getenv("HOME") : ".") + "/catkin_ws/scripts/switch_planner.sh";
  /// 切换脚本写的状态文件
  static constexpr const char* kPlannerStateFile = "/tmp/planner_switch.state";
  /// 非空 = 切换进行中，值是目标规划器（ego / super）
  std::string planner_pending_;
  ros::Publisher planner_status_pub_;
  ros::ServiceServer srv_goto_, srv_goto_planner_, srv_planner_, srv_nav_mode_,
      srv_start_follow_, srv_stop_follow_;
  ros::ServiceServer srv_align_, srv_actuator_, srv_geofence_, srv_estop_;
  ros::Subscriber sub_heartbeat_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::ServiceClient arming_cli_;
  ros::ServiceClient set_mode_cli_;
  mavros_msgs::State mav_state_;
  bool have_state_ = false;

  std::mutex mtx_;
  Command pending_;
  Planner planner_ = Planner::SUPER;
  int act_[3] = {50, 50, 50};

  struct Geofence {
    bool enabled = false;
    std::vector<std::pair<float, float>> polygon;
    float z_min = 0.0f;
    float z_max = 3.0f;
  } geofence_;

  ros::Time last_heartbeat_;
  double heartbeat_timeout_ = 0.0;  // 0 = 关闭（避免调试时误触发）
  bool link_lost_ = false;

  // 当前任务与目标（非阻塞推进）
  Task task_ = Task::IDLE;
  PreFlight preflight_ = PreFlight::NONE;
  int preflight_ticks_ = 0;
  float target_z_ = 1.0f;   // 起飞目标高度
  float tx_ = 0, ty_ = 0, tz_ = 0, tyaw_ = 0, ttol_ = 0.2f;

  // 当前实际位置（从 mavros 位姿订阅更新）
  float cur_x_ = 0, cur_y_ = 0, cur_z_ = 0, cur_yaw_ = 0;
};

int main(int argc, char** argv) {
  setlocale(LC_ALL, ""); // 不设的话中文日志会变成 ????
  ros::init(argc, argv, "gcs_gateway");
  ros::NodeHandle nh;

  GcsGateway gw(nh);
  gw.spin();
  return 0;
}
