#include "ruikang.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

// ── 轨迹指令缓存 + PD外环 → 速度指令 → setpoint_raw/local ──
// 控制频率 50Hz (20ms)
// PD外环: v_cmd = kp*pos_err + kv*vel_err + ki*integral + vel_ff
// vel_err = vel_ref - vel_actual  (完整PD阻尼，抑制B样条附近震荡)
// ki 小积分消除稳态误差，确保到达目标点
//
// ── 本版改动说明 ──
// 1. 悬停时长拆分: putpoint1→hovertimes1, putpoint2→hovertimes2, putpoint3→hovertimes3
// 2. putpoint2: 悬停进行到 (hovertimes2 - servo2_pre_trigger) 秒时触发投放，
//    悬停正常计时结束后，不再额外悬停，只做1秒过渡延时(kPutpoint2PostWait)，再飞向下一点
// 3. high_waypoint: 悬停进行到 (high_hovertimes - laser_pre_trigger) 秒时触发激光ON，
//    激光ON后固定 laser_on_duration_ 秒(默认3s)自动OFF，不再依赖 laser_off_point 航点
// 4. 所有进入悬停状态(HOVER_HIGH / HOVER_HIGH_AT_HOVERPOINT)的入口，
//    统一 target_position = current_position，防止用旧航点坐标悬停导致的"归位漂移"
// 5. DIRECT_FLY: 到达判定对齐EGO的debounce模式(连续0.15s在容差内)，
//    去掉direct_fly_locked_死锁；sp到目标后飞机仍有稳态误差时积分推动sp越过目标

PX4RosNavEgoPD::PX4RosNavEgoPD(const ros::NodeHandle &nh_private) : nh_private_(nh_private) {

    // ===== 基础参数 =====
    int point_num;
    nh_private_.param<float>("fly_height",       fly_height,       1.0f);
    nh_private_.param<float>("low_fly_height",   low_fly_height,   0.5f);
    nh_private_.param<float>("high_fly_height",  high_fly_height,  2.0f);
    nh_private_.param<float>("hovertimes",       hovertimes,       2.0f);
    nh_private_.param<float>("high_hovertimes", high_hovertimes, 2.0f);
    nh_private_.param<float>("low_hovertimes",  low_hovertimes,  2.0f);
    nh_private_.param<bool> ("is_auto_land",     is_auto_land,     false);
    nh_private_.param<float>("goal_tolerance",   goal_tolerance,   0.25f);
    nh_private_.param<float>("direct_fly_goal_tolerance", direct_fly_goal_tolerance, 0.3f);
    nh_private_.param<int>  ("putpoint1",        putpoint1,        -1);
    nh_private_.param<int>  ("putpoint2",        putpoint2,        -1);
    nh_private_.param<int>  ("putpoint3",        putpoint3,        -1);
    nh_private_.param<int>  ("low_waypoint",     low_waypoint,     -1);
    nh_private_.param<int>  ("high_waypoint",    high_waypoint,    -1);
    nh_private_.param<int>  ("point_num",        point_num,        0);
    nh_private_.param<float>("direct_fly_kp",    direct_fly_kp,    0.8f);
    nh_private_.param<float>("direct_fly_max_v", direct_fly_max_v, 0.8f);
    nh_private_.param<int>  ("servo2_judge_point", servo2_judge_point,  -1);
    nh_private_.param<int>  ("servo2_judge_angle", servo2_judge_angle,   0);
    nh_private_.param<int>  ("laser_on_point",     laser_on_point,      -1);
    nh_private_.param<int>  ("laser_off_point",    laser_off_point,     -1);
    nh_private_.param<float>("Kp_balloon_y",       Kp_balloon_y,       0.003f);
    nh_private_.param<float>("Kp_balloon_z",       Kp_balloon_z,       0.003f);
    nh_private_.param<int>  ("balloon_tol_px",     balloon_tol_px,     20);
    nh_private_.param<float>("balloon_stab_dist",  balloon_stab_dist,  0.6f);
    nh_private_.param<float>("img_cx",             img_cx,             320.0f);
    nh_private_.param<float>("img_cy",             img_cy,             240.0f);

    // ── 新增参数 ──
    nh_private_.param<float>("hovertimes1",        hovertimes1,        2.0f);
    nh_private_.param<float>("hovertimes2",        hovertimes2,        2.0f);
    nh_private_.param<float>("hovertimes3",        hovertimes3,        hovertimes1);
    nh_private_.param<float>("servo2_pre_trigger", servo2_pre_trigger, 1.0f);
    nh_private_.param<float>("laser_pre_trigger",  laser_pre_trigger,  1.0f);
    nh_private_.param<float>("laser_on_duration",  laser_on_duration_, 3.0f);

    // ── PD 外环参数 ──
    nh_private_.param<double>("pd/kp_outer_xy",       kp_outer_xy_,        2.0);
    nh_private_.param<double>("pd/kv_outer_xy",       kv_outer_xy_,        2.0);
    nh_private_.param<double>("pd/ki_outer_xy",       ki_outer_xy_,        0.1);
    nh_private_.param<double>("pd/outer_max_vel",     outer_max_vel_,      1.0);
    nh_private_.param<double>("pd/outer_max_integral",outer_max_integral_, 0.3);
    nh_private_.param<double>("pd/kp_outer_z",        kp_outer_z_,         2.0);
    nh_private_.param<double>("pd/traj_timeout",      traj_timeout_,       0.5);

    ROS_WARN("PD Params: kp_xy=%.1f kv=%.1f ki=%.1f max_vel=%.1f max_int=%.2f timeout=%.1fs",
            kp_outer_xy_, kv_outer_xy_, ki_outer_xy_,
            outer_max_vel_, outer_max_integral_, traj_timeout_);
    ROS_WARN("goal_tolerance=%.2fm direct_fly_goal_tolerance=%.2fm", goal_tolerance, direct_fly_goal_tolerance);
    ROS_WARN("Hover times: putpoint1=%.1fs putpoint2=%.1fs putpoint3=%.1fs high=%.1fs low=%.1fs",
            hovertimes1, hovertimes2, hovertimes3, high_hovertimes, low_hovertimes);
    ROS_WARN("Servo2: point=%d angle=%d pre_trigger=%.1fs | Laser: point=%d pre_trigger=%.1fs on_duration=%.1fs",
            putpoint2, servo2_judge_angle, servo2_pre_trigger,
            high_waypoint, laser_pre_trigger, laser_on_duration_);
    ROS_WARN("DIRECT_FLY integral push: ki=%.2f max=%.2f", kDirectFlyIntegralKi, kDirectFlyIntegralMax);

    resetPDState();

    for (int i = 0; i < point_num; i++) {
        MissionPoint mp;
        nh_private_.param<float>("point" + to_string(i) + "_x",    mp.x,        0.0f);
        nh_private_.param<float>("point" + to_string(i) + "_y",    mp.y,        0.0f);
        nh_private_.param<float>("point" + to_string(i) + "_z",    mp.z,        fly_height);
        nh_private_.param<int>  ("point" + to_string(i) + "_type", mp.nav_type, 0);
        waypoints.push_back(mp);
    }

    // ── 订阅 ──
    state_sub    = nh_private_.subscribe("/mavros/state",               1, &PX4RosNavEgoPD::StateCallback,   this);
    pose_sub_    = nh_private_.subscribe("/mavros/local_position/pose", 1, &PX4RosNavEgoPD::PoseCallback,    this);
    vel_sub_     = nh_private_.subscribe("/mavros/local_position/velocity_local", 1, &PX4RosNavEgoPD::VelCallback, this);
    ego_cmd_sub_ = nh_private_.subscribe("/position_cmd",               1, &PX4RosNavEgoPD::EgoCmdCallback,  this);
    yolo_sub_    = nh_private_.subscribe("/yolov8/BoundingBoxes",       1, &PX4RosNavEgoPD::YoloCallback,    this);

    // ── 发布 ──
    waypoint_pub   = nh_private_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    servo1_pub_    = nh_global_.advertise<std_msgs::UInt16>("servo1_cmd", 10);
    servo2_pub_    = nh_global_.advertise<std_msgs::UInt16>("servo2_cmd", 10);
    laser_pub_     = nh_global_.advertise<std_msgs::Bool>  ("laser_cmd",  10);

    flightControl     = FlightControl("");
    target_position.z = fly_height;
    last_traj_cmd_time_ = ros::Time::now();

    takeoff_origin_.x = 0.0;
    takeoff_origin_.y = 0.0;
    takeoff_origin_.z = 0.0;

    // 50Hz 控制频率
    cmdloop_timer_ = nh_private_.createTimer(ros::Duration(0.02), &PX4RosNavEgoPD::FlyCmdLooper, this);
    fly_mode = UP;
}

PX4RosNavEgoPD::~PX4RosNavEgoPD() {}

void PX4RosNavEgoPD::resetPDState() {
    error_integral_x_ = 0.0;
    error_integral_y_ = 0.0;
}

void PX4RosNavEgoPD::VelCallback(const geometry_msgs::TwistStamped &msg) {
    current_velocity_ = msg.twist.linear;
}

// ── 接收 ego_planner 轨迹 ──
void PX4RosNavEgoPD::EgoCmdCallback(const quadrotor_msgs::PositionCommand &msg) {
    if (fly_mode != PLANNING) return;

    traj_cmd_.pos_x = msg.position.x;
    traj_cmd_.pos_y = msg.position.y;
    traj_cmd_.pos_z = msg.position.z;
    traj_cmd_.vel_x = msg.velocity.x;
    traj_cmd_.vel_y = msg.velocity.y;
    traj_cmd_.vel_z = msg.velocity.z;
    traj_cmd_.acc_x = msg.acceleration.x;
    traj_cmd_.acc_y = msg.acceleration.y;
    traj_cmd_.acc_z = msg.acceleration.z;
    traj_cmd_.yaw   = msg.yaw;
    traj_cmd_.yaw_dot = msg.yaw_dot;
    traj_cmd_.flag  = msg.trajectory_flag;
    traj_cmd_.stamp = ros::Time::now();

    last_traj_cmd_time_ = traj_cmd_.stamp;
    trajectory_active_ =
        (msg.trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED);
}

// ── 激光自动延迟关闭 ──
void PX4RosNavEgoPD::checkLaserAutoOff() {
    if (!laser_is_on_) return;
    if ((ros::Time::now() - laser_on_time_).toSec() >= laser_on_duration_) {
        std_msgs::Bool l_msg;
        l_msg.data = false;
        laser_pub_.publish(l_msg);
        laser_is_on_ = false;
        ROS_WARN("[Laser] auto OFF after %.1fs", laser_on_duration_);
    }
}

void PX4RosNavEgoPD::handleWaypointReached() {
    bool expected = false;
    if (!waypoint_handling_.compare_exchange_strong(expected, true)) return;

    ROS_WARN("Reached Waypoint %d", waypoint_id);
    resetPDState();
    tol_satisfied_start_ = -1.0;

    if (waypoint_id == low_waypoint) {
        fly_mode = HOVER_HIGH;
        low_waypoint_waiting    = true;
        low_waypoint_wait_start = ros::Time::now().toSec();
        target_position = current_position;
        hover_xy_lock_   = current_position;
        waypoint_handling_ = false;
        return;
    }
    if (waypoint_id == high_waypoint) {
        fly_mode = HOVER_HIGH;
        high_waypoint_waiting    = true;
        high_waypoint_wait_start = ros::Time::now().toSec();
        target_position = current_position;
        hover_xy_lock_   = current_position;
        waypoint_handling_ = false;
        return;
    }

    if (waypoint_id == putpoint1) {
        fly_mode          = HOVER_HIGH_AT_HOVERPOINT;
        start_hover_time  = ros::Time::now().toSec();
        target_position   = current_position;
        is_putpoint1_ = true;
        is_putpoint2_ = false;
        is_putpoint3_ = false;
        waypoint_handling_ = false;
        return;
    }
    if (waypoint_id == putpoint2) {
        fly_mode          = HOVER_HIGH_AT_HOVERPOINT;
        start_hover_time  = ros::Time::now().toSec();
        target_position   = current_position;
        is_putpoint1_ = false;
        is_putpoint2_ = true;
        is_putpoint3_ = false;
        servo2_triggered_this_hover_ = false;
        putpoint2_post_wait_         = false;
        waypoint_handling_ = false;
        return;
    }
    if (waypoint_id == putpoint3) {
        fly_mode          = HOVER_HIGH_AT_HOVERPOINT;
        start_hover_time  = ros::Time::now().toSec();
        target_position   = current_position;
        is_putpoint1_ = false;
        is_putpoint2_ = false;
        is_putpoint3_ = true;
        waypoint_handling_ = false;
        return;
    }

    if (!waypoints.empty() && waypoints.front().nav_type == NAV_BALLOON) {
        balloon_is_stabbing = false;
        balloon_step_dist   = 0.0f;
        balloon_done_time   = 0.0;
        fly_mode            = BALLOON_ALIGN;
        waypoint_handling_  = false;
        return;
    }
    if (!waypoints.empty()) {
        waypoints.pop_front();
        target_position = current_position;
        hover_xy_lock_   = current_position;
        fly_mode = HOVER_HIGH;
    }
    if (waypoints.empty()) {
        target_position = current_position;
        hover_xy_lock_   = current_position;
        if (is_auto_land) { fly_mode = DISAMRED; auto_land(); }
        else fly_mode = HOVER_HIGH;
    }
    waypoint_handling_ = false;
}

// ── 50Hz 主控制循环 ──
void PX4RosNavEgoPD::FlyCmdLooper(const ros::TimerEvent &event) {
    checkLaserAutoOff();

    if (!is_offboard) {
        flightControl.setpoint_raw_local_pos(target_position);
        return;
    }

    switch (fly_mode) {
        case UP:
        case DOWN: {
            geometry_msgs::Point pos_temp = is_initial_takeoff_ ? takeoff_origin_ : hover_xy_lock_;
            pos_temp.z = target_position.z;
            flightControl.setpoint_raw_local_pos(pos_temp);
            break;
        }
        case HOVER_HIGH: {
            geometry_msgs::Point pos_temp = hover_xy_lock_;
            pos_temp.z = target_position.z;
            flightControl.setpoint_raw_local_pos(pos_temp);
            if (low_waypoint_waiting) {
                if (ros::Time::now().toSec() - low_waypoint_wait_start > 1.0) {
                    low_waypoint_waiting = false;
                    fly_mode          = DOWN;
                    target_position.z = low_fly_height;
                    for (auto &wp : waypoints) wp.z = low_fly_height;
                }
                break;
            }
            if (high_waypoint_waiting) {
                if (ros::Time::now().toSec() - high_waypoint_wait_start > 1.0) {
                    high_waypoint_waiting = false;
                    fly_mode          = UP;
                    target_position.z = high_fly_height;
                    for (auto &wp : waypoints) wp.z = high_fly_height;
                }
                break;
            }
            if (!waypoints.empty()) publish_waypoint();
            }
            break;

        // ═══════════════ PLANNING (EGO) ═══════════════
        case PLANNING: {
            double dt = (ros::Time::now() - traj_cmd_.stamp).toSec();

            if (dt > traj_timeout_ || !trajectory_active_) {
                geometry_msgs::Point hover_vel;
                hover_vel.x = 0;
                hover_vel.y = 0;
                hover_vel.z = target_position.z;
                flightControl.setpoint_raw_local_velxy_posz(hover_vel);
                if (dt > traj_timeout_) {
                    ROS_WARN_THROTTLE(1.0, "Traj timeout! dt=%.2fs > %.2fs", dt, traj_timeout_);
                }
                return;
            }

            double err_x = traj_cmd_.pos_x - current_position.x;
            double err_y = traj_cmd_.pos_y - current_position.y;

            double vel_err_x = traj_cmd_.vel_x - current_velocity_.x;
            double vel_err_y = traj_cmd_.vel_y - current_velocity_.y;

            error_integral_x_ += err_x * 0.02;
            error_integral_y_ += err_y * 0.02;
            error_integral_x_ = std::max(-outer_max_integral_,
                                  std::min(error_integral_x_, outer_max_integral_));
            error_integral_y_ = std::max(-outer_max_integral_,
                                  std::min(error_integral_y_, outer_max_integral_));

            double vx_cmd = kp_outer_xy_ * err_x
                          + kv_outer_xy_ * vel_err_x
                          + ki_outer_xy_ * error_integral_x_
                          + traj_cmd_.vel_x;

            double vy_cmd = kp_outer_xy_ * err_y
                          + kv_outer_xy_ * vel_err_y
                          + ki_outer_xy_ * error_integral_y_
                          + traj_cmd_.vel_y;

            double speed = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
            if (speed > outer_max_vel_) {
                double scale = outer_max_vel_ / speed;
                vx_cmd *= scale;
                vy_cmd *= scale;
            }

            geometry_msgs::Point vel_xy_posz;
            vel_xy_posz.x = vx_cmd;
            vel_xy_posz.y = vy_cmd;
            vel_xy_posz.z = target_position.z;

            flightControl.setpoint_raw_local_velxy_posz(vel_xy_posz);
            break;
        }

        // ═══════════════ DIRECT_FLY ═══════════════
        case DIRECT_FLY: {

        float dx = target_position.x - current_position.x;
        float dy = target_position.y - current_position.y;
        float dist_to_final = std::sqrt(dx * dx + dy * dy);

        // 调试日志
        {
            double debounce_elapsed = (tol_satisfied_start_ > 0)
                ? (ros::Time::now().toSec() - tol_satisfied_start_) : -1.0;
            ROS_WARN_THROTTLE(1.0,
                "[DIRECT_FLY] dist=%.2fm tol=%.2fm debounce=%.2fs/%.2fs locked=%d "
                "sp_delta=(%.2f,%.2f) integ=(%.2f,%.2f)",
                dist_to_final, direct_fly_goal_tolerance,
                debounce_elapsed, kPosTolDebounce, (int)direct_fly_locked_,
                target_position.x - virtual_sp_x_, target_position.y - virtual_sp_y_,
                direct_fly_integral_x_, direct_fly_integral_y_);
        }

        float dx_sp = target_position.x - virtual_sp_x_;
        float dy_sp = target_position.y - virtual_sp_y_;
        float dist_to_sp = std::sqrt(dx_sp * dx_sp + dy_sp * dy_sp);

        // 分级减速
        float step_xy;
        if (dist_to_final > 0.5f) {
            step_xy = direct_fly_max_v * 0.02f;
        } else if (dist_to_final > 0.15f) {
            step_xy = 0.6f * 0.02f;
        } else {
            step_xy = 0.2f * 0.02f;
        }

        if (dist_to_sp > step_xy) {
            virtual_sp_x_ += (dx_sp / dist_to_sp) * step_xy;
            virtual_sp_y_ += (dy_sp / dist_to_sp) * step_xy;
            // sp在运动中，清除积分
            direct_fly_integral_x_ = 0.0f;
            direct_fly_integral_y_ = 0.0f;
        } else {
            // sp已到目标点，但飞机可能仍有稳态误差→积分推动sp越过目标
            virtual_sp_x_ = target_position.x;
            virtual_sp_y_ = target_position.y;

            if (dist_to_final > direct_fly_goal_tolerance) {
                // 离目标还有距离，朝目标方向积累积分推动
                direct_fly_integral_x_ += kDirectFlyIntegralKi * dx * 0.02f;
                direct_fly_integral_y_ += kDirectFlyIntegralKi * dy * 0.02f;
                direct_fly_integral_x_ = std::max(-kDirectFlyIntegralMax,
                    std::min(direct_fly_integral_x_, kDirectFlyIntegralMax));
                direct_fly_integral_y_ = std::max(-kDirectFlyIntegralMax,
                    std::min(direct_fly_integral_y_, kDirectFlyIntegralMax));
            } else {
                // 已在容差内，清除积分让debounce判定
                direct_fly_integral_x_ = 0.0f;
                direct_fly_integral_y_ = 0.0f;
            }
        }

        geometry_msgs::Point pos_cmd;
        pos_cmd.x = virtual_sp_x_ + direct_fly_integral_x_;
        pos_cmd.y = virtual_sp_y_ + direct_fly_integral_y_;
        pos_cmd.z = target_position.z;
        flightControl.setpoint_raw_local_pos(pos_cmd);
        break;
    }

        // ═══════════════ HOVER_HIGH_AT_HOVERPOINT ═══════════════
        case HOVER_HIGH_AT_HOVERPOINT: {
            float target_hovertime = hovertimes1;
            if (is_high_hover_) target_hovertime = high_hovertimes;
            if (is_low_hover_)  target_hovertime = low_hovertimes;
            if (is_putpoint1_)  target_hovertime = hovertimes1;
            if (is_putpoint2_)  target_hovertime = hovertimes2;
            if (is_putpoint3_)  target_hovertime = hovertimes3;

            double elapsed = ros::Time::now().toSec() - start_hover_time;

            flightControl.setpoint_raw_local_pos(target_position);

            // putpoint2: 提前触发投放
            if (is_putpoint2_ && !servo2_triggered_this_hover_ &&
                elapsed >= (target_hovertime - servo2_pre_trigger)) {
                std_msgs::UInt16 s2_msg;
                s2_msg.data = static_cast<uint16_t>(servo2_judge_angle);
                ROS_WARN("[Putpoint2] dropping now! angle=%d (elapsed=%.2f/%.2fs)",
                         (int)s2_msg.data, elapsed, target_hovertime);
                servo2_pub_.publish(s2_msg);
                servo2_triggered_this_hover_ = true;
            }

            // high_waypoint: 提前触发激光
            if (is_high_hover_ && !laser_triggered_this_hover_ &&
                elapsed >= (target_hovertime - laser_pre_trigger)) {
                std_msgs::Bool l_msg;
                l_msg.data = true;
                ROS_WARN("[High waypoint] laser ON (elapsed=%.2f/%.2fs)",
                         elapsed, target_hovertime);
                laser_pub_.publish(l_msg);
                laser_triggered_this_hover_ = true;
                laser_is_on_   = true;
                laser_on_time_ = ros::Time::now();
            }

            if (elapsed <= target_hovertime) {
                break;
            }

            // 悬停结束
            if (is_putpoint2_) {
                if (!putpoint2_post_wait_) {
                    putpoint2_post_wait_       = true;
                    putpoint2_post_wait_start_ = ros::Time::now().toSec();
                    ROS_WARN("[Putpoint2] hover done, waiting %.1fs...", kPutpoint2PostWait);
                    break;
                }
                if (ros::Time::now().toSec() - putpoint2_post_wait_start_ < kPutpoint2PostWait) {
                    break;
                }
                putpoint2_post_wait_ = false;
                is_putpoint2_        = false;
            }

            is_high_hover_ = false;
            is_low_hover_  = false;
            is_putpoint1_  = false;
            is_putpoint3_  = false;

            if (!waypoints.empty()) { waypoints.pop_front(); publish_waypoint(); }
            break;
        }

        case BALLOON_ALIGN:
        case BALLOON_STAB:
            if (handleBalloonTask()) {
                waypoints.pop_front();
                fly_mode = HOVER_HIGH;
            }
            break;

        default:
            break;
    }
}

void PX4RosNavEgoPD::publish_waypoint() {
    if (waypoints.empty()) return;
    waypoint_id++;
    MissionPoint wp = waypoints.front();

    tol_satisfied_start_ = -1.0;

    target_position.x = wp.x;
    target_position.y = wp.y;
    target_position.z = wp.z;

    if (wp.nav_type == NAV_EGO) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp    = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = wp.x;
        goal.pose.position.y = wp.y;
        goal.pose.position.z = wp.z;
        goal.pose.orientation.w = 1.0;
        waypoint_pub.publish(goal);

        resetPDState();
        trajectory_active_ = true;
        last_traj_cmd_time_ = ros::Time::now();
        fly_mode = PLANNING;
    } else {
        fly_mode = DIRECT_FLY;
        virtual_sp_x_ = current_position.x;
        virtual_sp_y_ = current_position.y;
        direct_fly_integral_x_ = 0.0f;
        direct_fly_integral_y_ = 0.0f;
        direct_fly_locked_ = false;
    }
    ROS_WARN("Mission Target: %d, Mode: %s", waypoint_id,
             wp.nav_type == NAV_EGO ? "EgoPlanner" :
             wp.nav_type == NAV_BALLOON ? "Balloon" : "Direct");
}

void PX4RosNavEgoPD::PoseCallback(const geometry_msgs::PoseStamped &msg) {
    current_position = msg.pose.position;
    auto &q = msg.pose.orientation;
    current_yaw = std::atan2(
        2.0f * (q.w * q.z + q.x * q.y),
        1.0f - 2.0f * (q.y * q.y + q.z * q.z));

    if (fly_mode == UP && std::abs(current_position.z - target_position.z) < 0.1f) {
        if (is_initial_takeoff_) {
            is_initial_takeoff_ = false;
            fly_mode = HOVER_HIGH;
        } else {
            fly_mode         = HOVER_HIGH_AT_HOVERPOINT;
            start_hover_time = ros::Time::now().toSec();
            target_position  = current_position;
            is_high_hover_   = true;
        }
    }

    if (fly_mode == DOWN && std::abs(current_position.z - target_position.z) < 0.1f) {
        fly_mode         = HOVER_HIGH_AT_HOVERPOINT;
        start_hover_time = ros::Time::now().toSec();
        target_position  = current_position;
        is_low_hover_    = true;
    }

    // ── PLANNING (EGO) 到达判定: debounce ──
    if (fly_mode == PLANNING) {
        float dist = std::sqrt(
            std::pow(target_position.x - current_position.x, 2) +
            std::pow(target_position.y - current_position.y, 2));
        if (dist < goal_tolerance) {
            if (tol_satisfied_start_ < 0) {
                tol_satisfied_start_ = ros::Time::now().toSec();
            } else if (ros::Time::now().toSec() - tol_satisfied_start_ > kPosTolDebounce) {
                tol_satisfied_start_ = -1.0;
                handleWaypointReached();
            }
        } else {
            tol_satisfied_start_ = -1.0;
        }
    }
    // ── DIRECT_FLY 到达判定: 纯debounce，无锁，和EGO一模一样 ──
    else if (fly_mode == DIRECT_FLY) {
        float dist = std::sqrt(
            std::pow(target_position.x - current_position.x, 2) +
            std::pow(target_position.y - current_position.y, 2));
        if (dist < direct_fly_goal_tolerance) {
            if (tol_satisfied_start_ < 0) {
                tol_satisfied_start_ = ros::Time::now().toSec();
            } else if (ros::Time::now().toSec() - tol_satisfied_start_ > kPosTolDebounce) {
                tol_satisfied_start_ = -1.0;
                handleWaypointReached();
            }
        } else {
            tol_satisfied_start_ = -1.0;
        }
    }
}

void PX4RosNavEgoPD::StateCallback(const mavros_msgs::State &msg) {
    current_state = msg;
    if (msg.armed && msg.mode == "OFFBOARD") {
        if (!is_offboard) ROS_WARN("OFFBOARD Detected!");
        is_offboard = true;
    } else {
        is_offboard = false;
    }
}

void PX4RosNavEgoPD::YoloCallback(const yolov8_ros_msgs::BoundingBoxes::ConstPtr &msg) {
    for (const auto &box : msg->bounding_boxes) {
        if (box.Class == "balloon" || box.Class == "ballon" || box.Class == "red_ballon") {
            yolo_box_info.cameraXCenter = (box.xmin + box.xmax) / 2.0f;
            yolo_box_info.cameraYCenter = (box.ymin + box.ymax) / 2.0f;
            last_yolo_time_ = ros::Time::now();
            break;
        }
    }
}

bool PX4RosNavEgoPD::handleBalloonTask() {
    if (!balloon_is_stabbing) {
        if ((ros::Time::now() - last_yolo_time_).toSec() > 2.0) {
            ROS_WARN_THROTTLE(1.0, "[YOLO] target lost >2s, hovering...");
            geometry_msgs::Point hover_pos;
            hover_pos.x = current_position.x;
            hover_pos.y = current_position.y;
            hover_pos.z = target_position.z;
            flightControl.setpoint_raw_local_pos(hover_pos);
            return false;
        }
    }

    if (!balloon_is_stabbing) {
        float err_x = yolo_box_info.cameraXCenter - img_cx;
        float err_y = yolo_box_info.cameraYCenter - img_cy;

        float vy_body   = -Kp_balloon_y * err_x;
        float vz_target =  target_position.z - Kp_balloon_z * err_y;

        vy_body   = std::clamp<float>(vy_body,   -0.5f, 0.5f);
        vz_target = std::clamp<float>(vz_target,
                                      target_position.z - 0.5f,
                                      target_position.z + 0.5f);

        geometry_msgs::Point align_cmd;
        align_cmd.x = -vy_body * std::sin(current_yaw);
        align_cmd.y =  vy_body * std::cos(current_yaw);
        align_cmd.z =  vz_target;

        flightControl.setpoint_raw_local_velxy_posz(align_cmd);

        if (std::abs(err_x) < balloon_tol_px && std::abs(err_y) < balloon_tol_px) {
            ROS_WARN("[BALLOON] aligned! stabbing...");
            balloon_is_stabbing  = true;
            balloon_stab_start_x = current_position.x;
            balloon_stab_start_y = current_position.y;
            balloon_stab_start_z = current_position.z;
            balloon_stab_yaw     = current_yaw;
            balloon_step_dist    = 0.0f;
        }
        return false;
    }

    const float step_size = 0.04f;
    if (balloon_step_dist < balloon_stab_dist) {
        balloon_step_dist += step_size;
    }

    float goal_x = balloon_stab_start_x + balloon_step_dist * std::cos(balloon_stab_yaw);
    float goal_y = balloon_stab_start_y + balloon_step_dist * std::sin(balloon_stab_yaw);

    float err_gx  = goal_x - current_position.x;
    float err_gy  = goal_y - current_position.y;
    float kp_stab = 1.2f;
    float max_v   = 1.0f;

    geometry_msgs::Point stab_cmd;
    stab_cmd.x = err_gx * kp_stab;
    stab_cmd.y = err_gy * kp_stab;
    stab_cmd.z = balloon_stab_start_z;

    float speed = std::sqrt(stab_cmd.x * stab_cmd.x + stab_cmd.y * stab_cmd.y);
    if (speed > max_v) {
        stab_cmd.x = stab_cmd.x / speed * max_v;
        stab_cmd.y = stab_cmd.y / speed * max_v;
    }

    flightControl.setpoint_raw_local_velxy_posz(stab_cmd);

    float dist_traveled = std::sqrt(
        std::pow(current_position.x - balloon_stab_start_x, 2) +
        std::pow(current_position.y - balloon_stab_start_y, 2));

    if (balloon_step_dist >= balloon_stab_dist) {
        ROS_WARN("[BALLOON] done! traveled=%.2fm", dist_traveled);
        balloon_is_stabbing = false;
        balloon_step_dist   = 0.0f;
        return true;
    }
    return false;
}

void PX4RosNavEgoPD::auto_land() {
    flightControl.set_mode("POSCTL");
    ros::Duration(0.5).sleep();
    flightControl.set_mode("AUTO.LAND");
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ruikang_pd_node");
    ros::NodeHandle nh("~");
    PX4RosNavEgoPD node(nh);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
