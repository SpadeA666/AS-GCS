#include "api_3d.h"

#include <algorithm>
#include <cmath>

inline float normalize_angle(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

ASNAV::ASNAV(ros::NodeHandle& nh) : nh_(nh)
{
    // 从参数服务器获取参数
    ros::NodeHandle nh_private("~");
    nh_private.param<bool>("is_auto_land", is_auto_land, true);
    nh_private.param<std::string>("target_class_name", target_class_name, "red_ballon");
    nh_private.param<float>("fly_height", fly_height, 0.5f);
    nh_private.param<float>("descend_z", descend_z, 0.3f);

    // navigationZpro 速度环PI修正参数
    nh_private.param<float>("zpro_kp", zpro_kp_, 2.5f);
    nh_private.param<float>("zpro_ki", zpro_ki_, 0.3f);
    nh_private.param<float>("zpro_max_v", zpro_max_v_, 2.0f);
    nh_private.param<float>("zpro_integral_clamp", zpro_integral_clamp_, 0.5f);
    nh_private.param<float>("zpro_err_thresh", zpro_err_thresh_, 0.5f);
    nh_private.param<float>("zpro_brake_dist", zpro_brake_dist_, 1.2f);
    nh_private.param<float>("zpro_brake_min_v", zpro_brake_min_v_, 0.15f);
    nh_private.param<float>("zpro_accel_ff_gain", zpro_accel_ff_gain_, 0.5f);

    // navigationZplus 参数加载
    nh_private.param<float>("zplus_kp_outer", zplus_kp_outer_, 2.5f);
    nh_private.param<float>("zplus_kv_outer", zplus_kv_outer_, 0.8f);
    nh_private.param<float>("zplus_ki_outer", zplus_ki_outer_, 0.3f);
    nh_private.param<float>("zplus_max_vel", zplus_max_vel_, 2.0f);
    nh_private.param<float>("zplus_max_integral", zplus_max_integral_, 0.5f);
    nh_private.param<float>("zplus_traj_timeout", zplus_traj_timeout_, 0.5f);
    nh_private.param<float>("zplus_max_accel", zplus_max_accel_, 2.5f);

    // positionSmooth 步长控制参数
    nh_private.param<double>("smooth_step_xy", smooth_step_xy_, 0.18f);
    nh_private.param<double>("smooth_slow_step_xy", smooth_slow_step_xy_, 0.01f);
    nh_private.param<double>("smooth_slow_dist", smooth_slow_dist_, 0.5f);

    // navigationSuper 参数加载
    nh_private.param<float>("super_kp_outer", super_kp_outer_, 2.5f);
    nh_private.param<float>("super_kv_outer", super_kv_outer_, 0.8f);
    nh_private.param<float>("super_ki_outer", super_ki_outer_, 0.3f);
    nh_private.param<float>("super_max_vel", super_max_vel_, 2.0f);
    nh_private.param<float>("super_max_vel_z", super_max_vel_z_, 1.0f);
    nh_private.param<float>("super_max_integral", super_max_integral_, 0.5f);
    nh_private.param<float>("super_traj_timeout", super_traj_timeout_, 0.5f);
    nh_private.param<bool>("super_rotate_180", super_rotate_180_, false);
    nh_private.param<float>("super_max_accel", super_max_accel_, 2.5f);
    nh_private.param<bool>("ego_rotate_180", ego_rotate_180_, false);

    // 姿态推力控制接口参数（px4ctrl 风格 / SUPER OMMPC 风格）
    nh_private.param<float>("att_kp", att_kp_, 2.5f);
    nh_private.param<float>("att_kv", att_kv_, 3.0f);
    nh_private.param<float>("att_hover_percentage", att_hover_percentage_, 0.5f);
    nh_private.param<float>("att_max_angle", att_max_angle_, 40.0f);
    nh_private.param<float>("att_kang_r", att_kang_r_, 20.0f);
    nh_private.param<float>("att_kang_p", att_kang_p_, 20.0f);
    nh_private.param<float>("att_kang_y", att_kang_y_, 4.0f);
    nh_private.param<bool>("att_use_planner_yaw", att_use_planner_yaw_, true);

    
    // 初始化订阅和发布
    mavros_state_sub_ = nh_.subscribe("/iris_0/mavros/state", 10, &ASNAV::mavros_state_cb, this);
    mavros_local_position_pose_sub_ = nh_.subscribe("/iris_0/mavros/local_position/pose", 10, &ASNAV::mavros_local_position_pose_cb, this);
    mavros_local_velocity_sub_ = nh_.subscribe("/iris_0/mavros/local_position/velocity_local", 10, &ASNAV::mavros_local_velocity_cb, this);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
    mavros_cmd_command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>("/iris_0/mavros/cmd/command");
    ego_planner_pos_cmd_sub_ = nh_.subscribe("/drone_0_planning/pos_cmd", 10, &ASNAV::ego_planner_pos_cmd_cb, this);
    super_planner_pos_cmd_sub_ = nh_.subscribe("/planning/pos_cmd", 10, &ASNAV::super_planner_pos_cmd_cb, this);
    mavros_setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 10);
    mavros_setpoint_raw_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/iris_0/mavros/setpoint_raw/attitude", 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    super_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    yolo_info_sub_ = nh_.subscribe("/yolov11/BoundingBoxes", 10, &ASNAV::yolo_info_cb, this);
    yolo_d435i_info_sub_ = nh_.subscribe("/yolov11/d435i/BoundingBoxes", 10, &ASNAV::yolo_d435i_info_cb, this);
    ar_pose_sub_ = nh_.subscribe("/ar_pose_marker", 10, &ASNAV::ar_pose_cb, this);

    is_offboard = false;
    ego_rotate_180_ = false;
    goal_sent_ = false;
    super_goal_sent_ = false;
    super_cmd_received_ = false;
    current_position = geometry_msgs::Point();
    target_position = mavros_msgs::PositionTarget();
    start_planning_time = 0;
    finish_planning_time = 0;

    ar_marker_found_ = false;
    ar_target_id_ = 8;
    ar_position_detec_x_ = 0;
    ar_position_detec_y_ = 0;
    ar_position_detec_z_ = 0;

    integral_zpx_ = 0.0;
    integral_zpy_ = 0.0;
    last_zplus_call_time_ = ros::Time(0);
    zplus_traj_elapsed_ = 0.0f;
    last_ego_msg_time_ = ros::Time(0);
    zplus_goal_time_ = ros::Time(0);
    last_zplus_vx_ = 0.0;
    last_zplus_vy_ = 0.0;
    zplus_last_traj_id_ = 0;

    zplus_tol_timing_ = false;
    zplus_tol_entry_time_ = ros::Time(0);
    zplus_holding_ = false;
    zplus_hold_start_time_ = ros::Time(0);

    integral_spx_ = 0.0;
    integral_spy_ = 0.0;
    integral_spz_ = 0.0;
    last_super_call_time_ = ros::Time(0);
    last_super_msg_time_ = ros::Time(0);
    super_traj_elapsed_ = 0.0f;
    super_rviz_mode_ = false;
    super_tol_timing_ = false;
    super_tol_entry_time_ = ros::Time(0);
    super_goal_time_ = ros::Time(0);
    last_super_vx_ = 0.0;
    last_super_vy_ = 0.0;
    last_super_vz_ = 0.0;

    attitude_active_time_ = ros::Time(0);
    att_feedback_valid_ = false;
    att_hover_valid_ = false;
    att_hover_x_ = att_hover_y_ = att_hover_z_ = att_hover_yaw_ = 0.0;
    satt_tol_timing_ = false;
    satt_tol_entry_time_ = ros::Time(0);
    zatt_tol_timing_ = false;
    zatt_tol_entry_time_ = ros::Time(0);
}

ASNAV::~ASNAV()
{
}
// 起飞接口
bool ASNAV::takeoff(float height)
{   
    ros::Rate rate(20);
    while (!current_state.connected && ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "等待飞控连接中.....");
        ros::spinOnce();
        rate.sleep();    
    }
    ROS_INFO("飞控连接成功，准备进入OFFBOARD模式");

    for(int i = 0; i < 100 && ros::ok() ; ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    position(0.0f, 0.0f, height, 0.0f, 0.1f);

    for(int i = 0; i < 100 && ros::ok() ; ++i)
    {
        setpointPublish();
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) 
    {
    setpointPublish();
    start_planning_time = ros::Time::now().toSec();
    if(!is_offboard)
     {
        if(current_state.armed && current_state.mode == "OFFBOARD")
        {
            is_offboard = true;
            ROS_INFO("已切换到OFFBOARD模式");
            ROS_INFO("正在上升到目标高度: %.2f m", height);
        }
        else
        {
            is_offboard = false;
        }
     }
    if (std::fabs(current_position.z - height) < 0.1f)
    {
        ROS_INFO("已达到目标高度: %.2f m", height);
        return true;
    }
    ros::spinOnce();
    rate.sleep();
    }
    return false;
}
// 位置控制接口
bool ASNAV::position(float x, float y, float z, float yaw, float tol)
{
    // if (!is_position_inited)
    // {
    //     return false;
    // }
    // float cos_yaw = cos(initial_yaw);
    // float sin_yaw = sin(initial_yaw);
    // float rotated_x = cos_yaw * x - sin_yaw * y;
    // float rotated_y = sin_yaw * x + cos_yaw * y;
    // float target_x = initial_position.x + rotated_x;
    // float target_y = initial_position.y + rotated_y;
    // float target_z = initial_position.z + z;
    // float target_yaw = initial_yaw + yaw;
    
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame =
    mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target_position.position.x = x;
    target_position.position.y = y;
    target_position.position.z = z;
    target_position.yaw = 0.0f;
    return tolerance(x, y, z) < tol;
}
//位置平滑接口
bool ASNAV::positionSmooth(float target_x, float target_y, float target_z, float tol, float hover_sec)
{
    // 1. 静态变量初始化
    static float last_target_x = -999.0f;
    static float last_target_y = -999.0f;
    static float last_target_z = -999.0f;
    static float locked_yaw = 0.0f;      // 新增：用于锁死起始时刻的角度
    static geometry_msgs::Point virtual_sp;
    
    static bool is_hovering = false;
    static ros::Time hover_start_time;
    static bool first_run = true;

    // 2. 检测新任务
    bool is_new_task = (std::abs(target_x - last_target_x) > 0.05f || 
                        std::abs(target_y - last_target_y) > 0.05f || 
                        std::abs(target_z - last_target_z) > 0.05f);

    if (is_new_task || first_run) 
    {
        // 关键点：记录起始位置和当前的 Yaw
        virtual_sp = current_position;
        
        last_target_x = target_x;
        last_target_y = target_y;
        last_target_z = target_z;
        is_hovering = false;
        first_run = false;
        ROS_INFO("平滑任务启动 [保持航向]: (%.2f, %.2f) -> (%.2f, %.2f) Yaw: %.2f", 
                 virtual_sp.x, virtual_sp.y, target_x, target_y, locked_yaw);
    }

    float dist_to_final = std::sqrt(std::pow(target_x - current_position.x, 2) + 
                                    std::pow(target_y - current_position.y, 2));

    // 3. 构建 MAVROS 消息
    target_position = mavros_msgs::PositionTarget(); 
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    if (!is_hovering) {
        // 掩码设置：我们依然控制位置和绝对偏航(Yaw)，但发送的角度始终不变
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        float dx = target_x - virtual_sp.x;
        float dy = target_y - virtual_sp.y;
        float dz = target_z - virtual_sp.z;
        float dist_sp = std::sqrt(dx * dx + dy * dy);

        // 步长限制
        float base_step_xy = smooth_step_xy_;
        if (dist_to_final < smooth_slow_dist_) {
            base_step_xy = smooth_slow_step_xy_;
        }

        if (dist_sp > base_step_xy) {
            virtual_sp.x += (dx / dist_sp) * base_step_xy;
            virtual_sp.y += (dy / dist_sp) * base_step_xy;
        } else {
            virtual_sp.x = target_x;
            virtual_sp.y = target_y;
        }
        
        // 高度处理略... (保持之前的逻辑)
        virtual_sp.z = (std::abs(dz) > 0.015f) ? (virtual_sp.z + (dz > 0 ? 0.015f : -0.015f)) : target_z;

        target_position.position.x = virtual_sp.x;
        target_position.position.y = virtual_sp.y;
        target_position.position.z = virtual_sp.z;
    } else {
        // 悬停制动阶段，发死最终目标点
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.position.x = target_x;
        target_position.position.y = target_y;
        target_position.position.z = target_z;

        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
    }

    // 重点：无论飞到哪，Yaw 始终发任务开始时锁定的那个值
    target_position.yaw = locked_yaw; 

    // 4. 到达与悬停判断（累计计时，不要求连续在tol内）
    if (dist_to_final < tol && std::abs(target_z - current_position.z) < tol) {
        if (!is_hovering) {
            is_hovering = true;
            hover_start_time = ros::Time::now();
            ROS_INFO("进入刹车区，强制零速闭环...");
        }
    }
    if (is_hovering && (ros::Time::now() - hover_start_time).toSec() >= hover_sec) {
        last_target_x = -999.0f;
        is_hovering = false;
        return true;
    }
    return false;
}
// 导航接口
bool ASNAV::navigation(float x, float y, float z, float yaw, float tol, float hover_sec)
{
    if (!goal_sent_) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        goal_pub_.publish(goal);

        goal_sent_ = true;
        ROS_INFO_THROTTLE(1, "[导航] 目标: x=%.2f y=%.2f z=%.2f yaw=%.2f", x, y, z, yaw);
    }

    // 2. 执行原本的速度控制逻辑（维持 Offboard 飞行）
    // 只要 move_base 还在算速度 (is_as_received)，我们就把速度喂给飞控
    if (ego_cmd_received_)
    {
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // 掩码：控制速度和高度
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        // ego 坐标系(E)与 MAVROS 系(M)差 180°(DIAG: ego_pos ≈ -cur_pos),把 ego 速度取反转到 M 系再发。
        target_position.velocity.x = -ego_cmd_.velocity.x;
        target_position.velocity.y = -ego_cmd_.velocity.y;
        // target_position.velocity.z = ego_cmd_.velocity.z;
        target_position.position.z = z;
        target_position.yaw = current_yaw; //ego_cmd_.yaw; //current_yaw
    }

    bool pos_reached = (tolerance(x, y, z) < tol);

    // float yaw_error = normalize_angle(yaw - current_yaw);

    // float yaw_tol = 0.1f; 
    // bool yaw_reached = (std::abs(yaw_error) < yaw_tol);

    if (pos_reached) 
    {
        goal_sent_ = false;
        // ROS_WARN("开始精定位！");
        // bool arrived = positionSmooth(x, y, z, tol * 0.5f, hover_sec);
        // if (arrived)
        // {
        //     ROS_INFO_THROTTLE(1, "已精确到达目标点");
        // }
        // else 
        // {
        //     ROS_INFO_THROTTLE(1, "[导航→点位控制] 正在精确靠近目标...");
        // }
        // return arrived;
        return true;
    }
    return false;
}
// 导航+精定位接口
bool ASNAV::navigationWithPosition(float x, float y, float z, float yaw, float tol, float hover_sec)
{
    static bool fine_locked = false;

    if (!fine_locked)
    {
        if (navigationSuper(x, y, z, yaw, tol))
        {
            fine_locked = true;
            ROS_WARN("开始精定位！");
        }
        return false;
    }

    if (positionSmooth(x, y, z, tol, hover_sec))
    {
        ROS_INFO("已精确到达目标点");
        fine_locked = false;
        return true;
    }
    return false;
}
// 导航pro改进版(位置外环 + 速度前馈 + Yaw覆盖 + ego停摆容错)正在测试中！！！！！！！！！
bool ASNAV::navigationZpro(float x, float y, float z, float yaw, float tol)
{
    if (!goal_sent_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        goal_pub_.publish(goal);

        goal_sent_ = true;
        // 新任务：清零积分
        integral_err_zx_ = 0.0f;
        integral_err_zy_ = 0.0f;
        ROS_INFO("[Zpro] 新目标 (%.2f, %.2f, %.2f) | Kp=%.2f Ki=%.2f maxV=%.2f brake=%.1fm",
                 x, y, z, zpro_kp_, zpro_ki_, zpro_max_v_, zpro_brake_dist_);
    }

    if (ego_cmd_received_)
    {
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        // ---------- 掩码 ----------
        // XY: 速度模式，Z: 位置模式
        // 加速度前馈始终开启（通过增益控制实际大小，gain=0 等效于关闭）
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFZ |        // Z 不用加速度
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        // ---------- 位置误差 ----------
        float err_x = ego_cmd_.position.x - current_position.x;
        float err_y = ego_cmd_.position.y - current_position.y;

        // ---------- 积分（仅在小误差时生效，防过冲）----------
        if (std::abs(err_x) < zpro_err_thresh_)
            integral_err_zx_ += err_x * 0.01f;          // dt ≈ 100Hz
        else
            integral_err_zx_ *= 0.95f;                   // 大误差时缓慢泄积分

        if (std::abs(err_y) < zpro_err_thresh_)
            integral_err_zy_ += err_y * 0.01f;
        else
            integral_err_zy_ *= 0.95f;

        integral_err_zx_ = std::clamp(integral_err_zx_, -zpro_integral_clamp_, zpro_integral_clamp_);
        integral_err_zy_ = std::clamp(integral_err_zy_, -zpro_integral_clamp_, zpro_integral_clamp_);

        // ---------- 速度指令 = 前馈 + P修正 + I修正 ----------
        float pi_corr_x = zpro_kp_ * err_x + zpro_ki_ * integral_err_zx_;
        float pi_corr_y = zpro_kp_ * err_y + zpro_ki_ * integral_err_zy_;
        float vx_ref = ego_cmd_.velocity.x;
        float vy_ref = ego_cmd_.velocity.y;

        // PI 修正不得反向抵消前馈超过 80%：飞机不能停，必须跟着B样条走
        float max_oppose_x = 0.8f * std::abs(vx_ref);
        float max_oppose_y = 0.8f * std::abs(vy_ref);
        if (vx_ref > 0.01f && pi_corr_x < -max_oppose_x) pi_corr_x = -max_oppose_x;
        if (vx_ref < -0.01f && pi_corr_x > max_oppose_x) pi_corr_x = max_oppose_x;
        if (vy_ref > 0.01f && pi_corr_y < -max_oppose_y) pi_corr_y = -max_oppose_y;
        if (vy_ref < -0.01f && pi_corr_y > max_oppose_y) pi_corr_y = max_oppose_y;

        float vx_cmd = vx_ref + pi_corr_x;
        float vy_cmd = vy_ref + pi_corr_y;

        // ---------- 速度限幅 ----------
        float v_norm = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
        if (v_norm > zpro_max_v_)
        {
            vx_cmd *= zpro_max_v_ / v_norm;
            vy_cmd *= zpro_max_v_ / v_norm;
        }

        // ---------- 距离比例刹车 ----------
        float dist_to_goal = std::sqrt(std::pow(x - current_position.x, 2) +
                                       std::pow(y - current_position.y, 2));
        // 只在轨迹结束后刹车。轨迹活跃时（含弯道）B样条自己控速，刹车不干扰
        float v_ref_norm = std::sqrt(ego_cmd_.velocity.x * ego_cmd_.velocity.x +
                                     ego_cmd_.velocity.y * ego_cmd_.velocity.y);
        bool traj_ended = (v_ref_norm < 0.05f);
        if (dist_to_goal < zpro_brake_dist_ && traj_ended)
        {
            float brake_scale = dist_to_goal / zpro_brake_dist_;
            if (v_norm > 1e-4f)
            {
                float min_scale = zpro_brake_min_v_ / v_norm;
                if (brake_scale < min_scale) brake_scale = min_scale;
            }
            vx_cmd *= brake_scale;
            vy_cmd *= brake_scale;
        }

        // 轨迹结束后限速 0.3，防动能过冲
        if (traj_ended)
        {
            float v_end = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
            if (v_end > 0.3f)
            {
                vx_cmd *= 0.3f / v_end;
                vy_cmd *= 0.3f / v_end;
            }
        }

        target_position.velocity.x = vx_cmd;
        target_position.velocity.y = vy_cmd;

        // ---------- 加速度前馈：连续增益，无死区无阈值 ----------
        float ax_ref = ego_cmd_.acceleration.x;
        float ay_ref = ego_cmd_.acceleration.y;
        float v_norm_ref = std::sqrt(vx_ref * vx_ref + vy_ref * vy_ref);
        float cross_va = vx_ref * ay_ref - vy_ref * ax_ref;
        float a_normal = (v_norm_ref > 0.1f) ? std::abs(cross_va) / v_norm_ref : 0.0f;

        float curv_gain = zpro_accel_ff_gain_ * std::min(a_normal / 2.0f, 1.0f);
        target_position.acceleration_or_force.x = curv_gain * ax_ref;
        target_position.acceleration_or_force.y = curv_gain * ay_ref;

        // 轨迹结束清零积分
        if (traj_ended)
        {
            integral_err_zx_ = 0.0f;
            integral_err_zy_ = 0.0f;
        }

        // Z: 使用接口传入的目标高度（不用ego的）
        target_position.position.z = z;

        target_position.yaw = 0.0f;  // 全程机头正向，死锁为0

        // 调试输出（0.3s 节流）
        float dist = std::sqrt(std::pow(x - current_position.x, 2) +
                               std::pow(y - current_position.y, 2));
        ROS_INFO_THROTTLE(0.3,
            "[Zpro] err=(%.3f,%.3f) inte=(%.2f,%.2f) vref=(%.2f,%.2f) vcmd=(%.2f,%.2f) aN=%.2f cG=%.2f dist=%.2f brk=%s",
            err_x, err_y,
            integral_err_zx_, integral_err_zy_,
            vx_ref, vy_ref, vx_cmd, vy_cmd,
            a_normal, curv_gain,
            dist,
            (dist < zpro_brake_dist_ && traj_ended) ? "ON" : "off");
    }

    // 到达判定
    bool pos_reached = (tolerance(x, y, z) < tol);

    if (pos_reached)
    {
        goal_sent_ = false;
        integral_err_zx_ = 0.0f;
        integral_err_zy_ = 0.0f;
        return true;
    }
    return false;
}
// 速度指令帧间斜率限制：|Δv| ≤ max_acc·dt，last 随调用更新。
// 换 case / 新轨迹切入 / 超时进出时，指令速度变化全部变为斜坡，消除机身"抽一下"。
void ASNAV::slewLimitVel(double& vx, double& vy, double& vz,
                         double& lx, double& ly, double& lz, double dt, float max_acc)
{
    if (max_acc > 0.0f)
    {
        double dv = static_cast<double>(max_acc) * static_cast<double>(dt);
        vx = std::clamp(vx, lx - dv, lx + dv);
        vy = std::clamp(vy, ly - dv, ly + dv);
        vz = std::clamp(vz, lz - dv, lz + dv);
    }
    lx = vx;
    ly = vy;
    lz = vz;
}
// 导航接口（Zplus）：速度误差阻尼 + 位置外环PI + 加速度前馈
bool ASNAV::navigationZplus(float x, float y, float z, float yaw, float tol)
{
    // ====== 计算实际 dt ======
    ros::Time now = ros::Time::now();
    float dt = last_zplus_call_time_.isZero()
                   ? 0.01f
                   : (now - last_zplus_call_time_).toSec();
    dt = std::clamp(dt, 0.005f, 0.1f);
    last_zplus_call_time_ = now;

    const bool yaw_hold = std::isnan(yaw);  // NaN = 不管航向：保持当前 yaw（掩码置 IGNORE_YAW）

    // ====== 新目标检测 ======
    if (!goal_sent_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = now;
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        // NaN yaw 会生成 NaN 四元数污染 ego 的目标解析，替换为 0
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_hold ? 0.0f : yaw);
        goal_pub_.publish(goal);

        goal_sent_ = true;
        zplus_goal_time_ = now;
        last_ego_msg_time_ = now;   // 刷新超时计时：等 ego 重规划期间不进超时刹车（对齐 ruikang publish_waypoint）
        integral_zpx_ = 0.0;
        integral_zpy_ = 0.0;
        // 斜率限制以当前实际速度为种子：case 切换瞬间指令速度与实际速度天然连续
        last_zplus_vx_ = current_velocity.x;
        last_zplus_vy_ = current_velocity.y;
        ROS_INFO("[Zplus] 新目标 (%.2f, %.2f, %.2f)", x, y, z);
    }

    // ====== 轨迹超时保护：ego 断联 → 悬停 ======
    double dt_since_ego =
        last_ego_msg_time_.isZero()
            ? 0.0
            : (now - last_ego_msg_time_).toSec();
    bool traj_timeout = (ego_cmd_received_ && dt_since_ego > zplus_traj_timeout_);

    if (!ego_cmd_received_ || traj_timeout)
    {
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0,
                "[Zplus] Traj timeout! dt=%.2fs > %.2fs → HOVER",
                dt_since_ego, zplus_traj_timeout_);

        // type_mask 同样对齐 ruikang setpoint_raw_local_velxy_posz（对应 ruikang PLANNING
        // 分支 dt > traj_timeout_ 时的超时悬停逻辑）
        target_position.header.stamp = now;
        target_position.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
            (yaw_hold ? mavros_msgs::PositionTarget::IGNORE_YAW : 0);
        // 悬停零速度经斜率限制：进入/退出都是斜坡而非阶跃
        double vx_cmd = 0.0, vy_cmd = 0.0, vz_dmy = 0.0;
        slewLimitVel(vx_cmd, vy_cmd, vz_dmy,
                     last_zplus_vx_, last_zplus_vy_, vz_dmy, dt, zplus_max_accel_);
        target_position.velocity.x = vx_cmd;
        target_position.velocity.y = vy_cmd;
        target_position.position.z = z;
        if (!yaw_hold) target_position.yaw = yaw;
        return false;
    }

    // ====== 位置误差 & 速度误差 ======
    // 【坐标系转换】DIAG 实测 ego 坐标系(E)和 MAVROS 系(M)差 180°(ego_pos ≈ -cur_pos,
    // 且 ego 轨迹终点 +2.285 正好 = M 系的 -2.285,说明 ego 确实在往 -2.3 飞)。
    // 所以把 ego 的位置/速度取反转到 M 系再算误差,否则误差混用两个坐标系必然发散。
    double ego_px = -ego_cmd_.position.x;
    double ego_py = -ego_cmd_.position.y;
    double ego_vx = -ego_cmd_.velocity.x;
    double ego_vy = -ego_cmd_.velocity.y;

    double err_x = ego_px - current_position.x;
    double err_y = ego_py - current_position.y;
    double vel_err_x = ego_vx - current_velocity.x;
    double vel_err_y = ego_vy - current_velocity.y;

    // ====== 新轨迹检测：ego replan 后 trajectory_id 变化 → 清积分 ======
    // 防止旧轨迹攒下的积分把机体拉向新参考（bak 版本曾有的保护，现恢复）
    if (ego_cmd_.trajectory_id != zplus_last_traj_id_)
    {
        zplus_last_traj_id_ = ego_cmd_.trajectory_id;
        integral_zpx_ = 0.0;
        integral_zpy_ = 0.0;
    }

    // ====== 无条件积分（固定步长0.02，与 ruikang 的 error_integral_x_ += err_x * 0.02 完全一致）======
    integral_zpx_ += err_x * 0.02;
    integral_zpy_ += err_y * 0.02;
    integral_zpx_ = std::max(-(double)zplus_max_integral_, std::min(integral_zpx_, (double)zplus_max_integral_));
    integral_zpy_ = std::max(-(double)zplus_max_integral_, std::min(integral_zpy_, (double)zplus_max_integral_));

    // ====== 速度指令 = v_ref + Kp*err_p + Kv*err_v + Ki*∫err_p ======
    // ego 的速度参考同样取反转到 M 系,整条 vcmd 现在就是 M 系速度,直接发。
    double vx_cmd = ego_vx
                  + zplus_kp_outer_ * err_x
                  + zplus_kv_outer_ * vel_err_x
                  + zplus_ki_outer_ * integral_zpx_;

    double vy_cmd = ego_vy
                  + zplus_kp_outer_ * err_y
                  + zplus_kv_outer_ * vel_err_y
                  + zplus_ki_outer_ * integral_zpy_;

    // ====== 速度幅值限幅 ======
    double speed = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
    if (speed > zplus_max_vel_)
    {
        double scale = zplus_max_vel_ / speed;
        vx_cmd *= scale;
        vy_cmd *= scale;
    }

    // ====== 帧间加速度斜率限制（2026-08 修复）======
    // 消除换 case、ego 新轨迹切入、超时悬停进出瞬间的速度阶跃
    double vz_dmy2 = 0.0;
    slewLimitVel(vx_cmd, vy_cmd, vz_dmy2,
                 last_zplus_vx_, last_zplus_vy_, vz_dmy2, dt, zplus_max_accel_);

    // ====== 构建 MAVROS 消息 ======
    // type_mask 与 ruikang.cpp 的 FlightControl::setpoint_raw_local_velxy_posz 逐位对齐：
    // IGNORE_PX+IGNORE_PY+IGNORE_AFX+IGNORE_AFY+IGNORE_AFZ+FORCE+IGNORE_YAW_RATE
    // （之前这里少了 FORCE、少了 IGNORE_AFX/AFY，还多了一个 ruikang 没有的 IGNORE_VZ）
    target_position.header.stamp = now;
    target_position.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
        (yaw_hold ? mavros_msgs::PositionTarget::IGNORE_YAW : 0);

    // 直接透传:DIAG 实测 sent 与飞机实际速度同向,PX4 收到 FRAME_LOCAL_NED 速度不镜像。
    // 取反反而把指向 -2.3 目标的正确 vcmd 反掉了,所以直接发 vcmd。
    target_position.velocity.x = vx_cmd;
    target_position.velocity.y = vy_cmd;
    target_position.position.z = z;
    // NaN yaw = 保持当前航向（掩码 IGNORE_YAW）；给定 yaw 时跟踪目标 yaw
    if (!yaw_hold) target_position.yaw = yaw;

    // ====== 调试输出 ======
    ROS_INFO_THROTTLE(0.3,
        "[Zplus] err_p=(%.3f,%.3f) err_v=(%.3f,%.3f) int=(%.2f,%.2f) "
        "vref=(%.2f,%.2f) vcmd=(%.2f,%.2f) dist=%.2f deb=%s",
        err_x, err_y, vel_err_x, vel_err_y,
        integral_zpx_, integral_zpy_,
        ego_cmd_.velocity.x, ego_cmd_.velocity.y, vx_cmd, vy_cmd,
        tolerance(x, y, z),
        zplus_tol_timing_ ? "ON" : "off");

    // ====== 诊断日志:对比 ego 指令 vs 实际状态(标定速度镜像用) ======
    ROS_INFO_THROTTLE(0.3,
        "[DIAG] ego_pos=(%.3f,%.3f) ego_vel=(%.3f,%.3f) ego_yaw=%.2f | "
        "cur_pos=(%.3f,%.3f) cur_vel=(%.3f,%.3f) cur_yaw=%.2f | "
        "vcmd=(%.3f,%.3f) sent=(%.3f,%.3f)",
        ego_cmd_.position.x, ego_cmd_.position.y,
        ego_cmd_.velocity.x, ego_cmd_.velocity.y, ego_cmd_.yaw,
        current_position.x, current_position.y,
        current_velocity.x, current_velocity.y, current_yaw,
        vx_cmd, vy_cmd,
        target_position.velocity.x, target_position.velocity.y);

    // ====== 到达判定（纯位置 Debounce，与 ruikang 完全一致）======
    // 被动观察，不改变控制器行为 — 始终跟踪 B 样条
    float dist = tolerance(x, y, z);
    const float kDebounceTime = 0.15f;  // 对齐 ruikang 的 kPosTolDebounce = 0.15

    if (dist < tol)
    {
        if (!zplus_tol_timing_)
        {
            zplus_tol_timing_ = true;
            zplus_tol_entry_time_ = now;
            ROS_INFO("[Zplus] 进入容差 dist=%.2f<%.2f, debounce %.2fs...",
                     dist, tol, kDebounceTime);
        }
        else if ((now - zplus_tol_entry_time_).toSec() > kDebounceTime)
        {
            // Debounce 完成 → 直接判定到达（对应 ruikang 对普通航点的处理：
            // handleWaypointReached() 切到 HOVER_HIGH 后，下一个周期 !waypoints.empty()
            // 就立刻 publish_waypoint() 发下一个目标，中间没有固定悬停等待）。
            // 不再进入 0.5s 的强制悬停阶段，避免连续调用时每个航点都"顿一下"。
            ROS_INFO("[Zplus] Debounce 完成, 到达！");
            zplus_tol_timing_ = false;
            goal_sent_ = false;
            integral_zpx_ = 0.0;
            integral_zpy_ = 0.0;
            // 到达帧显式给零速度（经斜率限制）：不把残余速度指令带进下一个 case
            double zx = 0.0, zy = 0.0, zz = 0.0;
            slewLimitVel(zx, zy, zz, last_zplus_vx_, last_zplus_vy_, zz, dt, zplus_max_accel_);
            target_position.velocity.x = zx;
            target_position.velocity.y = zy;
            return true;
        }
    }
    else
    {
        zplus_tol_timing_ = false;
    }
    return false;
}

// 纯跟随 ego 轨迹（点击飞行模式）：不发自己的目标，目标来自 RViz 2D Nav Goal
// 直接给 ego_planner_node，本函数只负责把 /drone_0_planning/pos_cmd 转成 PX4 setpoint。
// 与 navigationZplus 的区别：不发布 /move_base_simple/goal；z 跟随 ego 轨迹而非写死；
// 永不判定"到达"，持续跟随，直到 ego 停止发轨迹（到达目标后 FSM 回 WAIT_TARGET）→ 超时悬停。
bool ASNAV::followEgo()
{
    ros::Time now = ros::Time::now();

    // ====== 轨迹超时保护：ego 断联 → 悬停 ======
    double dt_since_ego =
        last_ego_msg_time_.isZero()
            ? 0.0
            : (now - last_ego_msg_time_).toSec();
    bool traj_timeout = (ego_cmd_received_ && dt_since_ego > zplus_traj_timeout_);

    if (!ego_cmd_received_ || traj_timeout)
    {
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0,
                "[FollowEgo] Traj timeout! dt=%.2fs > %.2fs → HOVER",
                dt_since_ego, zplus_traj_timeout_);

        target_position.header.stamp = now;
        target_position.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.velocity.x = 0.0;
        target_position.velocity.y = 0.0;
        target_position.position.z = ego_cmd_received_ ? ego_cmd_.position.z : current_position.z;
        target_position.yaw = 0.0f;
        return false;
    }

    // ====== 位置误差 & 速度误差（跟踪 ego B 样条参考） ======
    double err_x = ego_cmd_.position.x - current_position.x;
    double err_y = ego_cmd_.position.y - current_position.y;
    double vel_err_x = ego_cmd_.velocity.x - current_velocity.x;
    double vel_err_y = ego_cmd_.velocity.y - current_velocity.y;

    // ====== 新轨迹检测：ego replan 后 trajectory_id 变化 → 清积分 + 退出位置保持 ======
    if (ego_cmd_.trajectory_id != zplus_last_traj_id_)
    {
        zplus_last_traj_id_ = ego_cmd_.trajectory_id;
        integral_zpx_ = 0.0;
        integral_zpy_ = 0.0;
        ego_pos_hold_ = false;
        ego_pos_hold_frames_ = 0;
    }

    // ====== 无条件积分（固定步长 0.02，与 navigationZplus 一致）======
    integral_zpx_ += err_x * 0.02;
    integral_zpy_ += err_y * 0.02;
    integral_zpx_ = std::max(-(double)zplus_max_integral_, std::min(integral_zpx_, (double)zplus_max_integral_));
    integral_zpy_ = std::max(-(double)zplus_max_integral_, std::min(integral_zpy_, (double)zplus_max_integral_));

    // ====== 速度指令 = v_ref + Kp*err_p + Kv*err_v + Ki*∫err_p ======
    double vx_cmd = ego_cmd_.velocity.x
                  + zplus_kp_outer_ * err_x
                  + zplus_kv_outer_ * vel_err_x
                  + zplus_ki_outer_ * integral_zpx_;

    double vy_cmd = ego_cmd_.velocity.y
                  + zplus_kp_outer_ * err_y
                  + zplus_kv_outer_ * vel_err_y
                  + zplus_ki_outer_ * integral_zpy_;

    // ====== 速度幅值限幅 ======
    double speed = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
    if (speed > zplus_max_vel_)
    {
        double scale = zplus_max_vel_ / speed;
        vx_cmd *= scale;
        vy_cmd *= scale;
    }

    // ====== 到点位置保持（zfix-smooth: 仿 ruikang HOVER）======
    // EGO 轨迹末端参考静止+距离近 → 切位置模式, 把最后收敛交给 PX4 位置环,
    // 消除速度模式下外环积分+滞后的慢摆; 新轨迹到达(trajectory_id 变化)自动退出
    {
        double ref_spd = std::sqrt(ego_cmd_.velocity.x * ego_cmd_.velocity.x +
                                   ego_cmd_.velocity.y * ego_cmd_.velocity.y);
        double dist_ref = std::sqrt(err_x * err_x + err_y * err_y);
        if (ref_spd < 0.05 && dist_ref < 0.3)
        {
            if (ego_pos_hold_frames_ < 100) ego_pos_hold_frames_++;
            if (ego_pos_hold_frames_ > 10) ego_pos_hold_ = true;   // 10帧防抖(~0.2s@50Hz)
        }
        else
        {
            ego_pos_hold_frames_ = 0;
            ego_pos_hold_ = false;
        }
    }
    if (ego_pos_hold_)
    {
        target_position.header.stamp = now;
        target_position.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.position.x = ego_cmd_.position.x;
        target_position.position.y = ego_cmd_.position.y;
        target_position.position.z = ego_cmd_.position.z;
        target_position.velocity.x = 0;
        target_position.velocity.y = 0;
        return false;
    }

    // ====== 构建 MAVROS 消息（与 navigationZplus 对齐）======
    target_position.header.stamp = now;
    target_position.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target_position.velocity.x = vx_cmd;
    target_position.velocity.y = vy_cmd;
    target_position.position.z = ego_cmd_.position.z;  // 3D 跟随：高度随 ego 轨迹
    target_position.yaw = 0.0f;

    // ====== 调试输出 ======
    ROS_INFO_THROTTLE(0.3,
        "[FollowEgo] err_p=(%.3f,%.3f) err_v=(%.3f,%.3f) "
        "vref=(%.2f,%.2f) vcmd=(%.2f,%.2f) z=%.2f",
        err_x, err_y, vel_err_x, vel_err_y,
        ego_cmd_.velocity.x, ego_cmd_.velocity.y, vx_cmd, vy_cmd,
        ego_cmd_.position.z);

    return false;  // 持续跟随，永不判定"到达"
}

// ==================== rviz 打点测试接口（完全照搬 cxr_egoctrl_v1.cpp，2026-08 新增） ====================
// 用法：遥控器 position 模式起飞 → rviz 中打点（2D Nav Goal）→ 调用本接口开始跟踪。
// 内部 while(ros::ok()) 阻塞运行，与 cxr_egoctrl_v1 的主循环完全一致：
//   打点前：hover_z 高度悬停（速度环 z 维持）；打点后：规划器出轨迹，用 P 速度跟踪律跟踪。
// 只接受 rviz 点触发的规划器轨迹，永不返回 true（只有节点退出时返回 false）。
bool ASNAV::navigationEgoRviz(float hover_z)
{
    ROS_INFO("[EgoRviz] 等待 EGO 规划器指令（悬停高度 %.2f m），请在 rviz 中打点...", hover_z);
    ros::Rate rate(50.0);

    // 与 cxr_egoctrl_v1 一致：先发 100 帧空 setpoint 初始化 offboard 通道
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    //     setpointPublish();
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    ego_cmd_received_ = false;   // 重新等一个 rviz 点触发的轨迹

    while (ros::ok())
    {
        if (!ego_cmd_received_)   // 未收到 EGO 轨迹 → 保持 hover_z 悬停（照搬 cxr）
        {
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target_position.header.stamp = ros::Time::now();
            target_position.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::FORCE |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.velocity.x = 0;
            target_position.velocity.y = 0;
            target_position.velocity.z = (hover_z - current_position.z) * 1.0f;
            target_position.yaw = current_yaw;
            ROS_INFO_THROTTLE(1.0, "[EgoRviz] 请等待，未收到轨迹，保持 %.2f m 悬停...", hover_z);
        }
        else   // 收到 EGO 轨迹 → 完全照搬 cxr 的 P 速度跟踪律
        {
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target_position.header.stamp = ros::Time::now();
            target_position.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::FORCE |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            // NaN 防护：规划器瞬时下发 NaN/越界值时回退 0，避免 PX4 判 SET_POSITION_TARGET_LOCAL_NED 无效
            float vx_cmd = (ego_cmd_.position.x - current_position.x) * 1.0f;
            float vy_cmd = (ego_cmd_.position.y - current_position.y) * 1.0f;
            float vz_cmd = (ego_cmd_.position.z - current_position.z) * 1.0f;
            if (!std::isfinite(vx_cmd)) vx_cmd = 0.0f;
            if (!std::isfinite(vy_cmd)) vy_cmd = 0.0f;
            if (!std::isfinite(vz_cmd)) vz_cmd = 0.0f;
            target_position.velocity.x = vx_cmd;
            target_position.velocity.y = vy_cmd;
            target_position.velocity.z = vz_cmd;
            target_position.yaw = std::isfinite(ego_cmd_.yaw) ? ego_cmd_.yaw : current_yaw;
            ROS_INFO_THROTTLE(0.5, "[EgoRviz] 跟踪 EGO 轨迹，vel=%.2f m/s",
                std::sqrt(target_position.velocity.x * target_position.velocity.x +
                          target_position.velocity.y * target_position.velocity.y));
        }
        setpointPublish();
        ros::spinOnce();
        rate.sleep();
    }

    return false;   // 永不返回 true
}

// SUPER 版本：完全照搬 cxr_egoctrl_v1.cpp 的跟踪律，跟踪 SUPER 规划器指令（/planning/pos_cmd）
bool ASNAV::navigationSuperRviz(float hover_z)
{
    ROS_INFO("[SuperRviz] 等待 SUPER 规划器指令（悬停高度 %.2f m），请在 rviz 中打点...", hover_z);
    ros::Rate rate(50.0);

    // 与 cxr_egoctrl_v1 一致：先发 100 帧空 setpoint 初始化 offboard 通道
    // 注释掉：空消息 type_mask=0（位置+速度都启用）会被 PX4 判 SET_POSITION_TARGET_LOCAL_NED invalid
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    //     setpointPublish();
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    super_cmd_received_ = false;   // 重新等一个 rviz 点触发的轨迹

    while (ros::ok())
    {
        if (!super_cmd_received_)   // 未收到 SUPER 轨迹 → 保持 hover_z 悬停（照搬 cxr）
        {
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target_position.header.stamp = ros::Time::now();
            target_position.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::FORCE |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.velocity.x = 0;
            target_position.velocity.y = 0;
            target_position.velocity.z = (hover_z - current_position.z) * 1.0f;
            target_position.yaw = current_yaw;
            ROS_INFO_THROTTLE(1.0, "[SuperRviz] 请等待，未收到轨迹，保持 %.2f m 悬停...", hover_z);
        }
        else   // 收到 SUPER 轨迹 → 完全照搬 cxr 的 P 速度跟踪律
        {
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            target_position.header.stamp = ros::Time::now();
            target_position.type_mask =
                mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_PZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::FORCE |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            // NaN 防护：规划器瞬时下发 NaN/越界值时回退 0，避免 PX4 判 SET_POSITION_TARGET_LOCAL_NED 无效
            float vx_cmd = (super_cmd_.position.x - current_position.x) * 1.0f;
            float vy_cmd = (super_cmd_.position.y - current_position.y) * 1.0f;
            float vz_cmd = (super_cmd_.position.z - current_position.z) * 1.0f;
            if (!std::isfinite(vx_cmd)) vx_cmd = 0.0f;
            if (!std::isfinite(vy_cmd)) vy_cmd = 0.0f;
            if (!std::isfinite(vz_cmd)) vz_cmd = 0.0f;
            target_position.velocity.x = vx_cmd;
            target_position.velocity.y = vy_cmd;
            target_position.velocity.z = vz_cmd;
            target_position.yaw = std::isfinite(super_cmd_.yaw) ? super_cmd_.yaw : current_yaw;
            ROS_INFO_THROTTLE(0.5, "[SuperRviz] 跟踪 SUPER 轨迹，vel=%.2f m/s",
                std::sqrt(target_position.velocity.x * target_position.velocity.x +
                          target_position.velocity.y * target_position.velocity.y));
        }
        setpointPublish();
        ros::spinOnce();
        rate.sleep();
    }

    return false;   // 永不返回 true
}

// ====== navigationSuper: SUPER 规划器接口（仿 Zplus 控制器） ======
// 调用方式：在主循环中每帧调用 navigationSuper(goal_x, goal_y, goal_z, goal_yaw, tol)
// 返回 true 表示已到达目标点（debounce 完成）
bool ASNAV::navigationSuper(float x, float y, float z, float yaw, float tol)
{
    // ====== 计算实际 dt ======
    ros::Time now = ros::Time::now();
    float dt = last_super_call_time_.isZero()
                   ? 0.01f
                   : (now - last_super_call_time_).toSec();
    dt = std::clamp(dt, 0.005f, 0.1f);
    last_super_call_time_ = now;

    const bool yaw_hold = std::isnan(yaw);  // NaN = 不管航向：保持当前 yaw（掩码置 IGNORE_YAW）

    // ====== 新目标检测：发布 goal 到 SUPER 的目标话题 ======
    // rviz 测试模式（super_rviz_mode_）下不发 goal，只等用户在 rviz 里打点触发 SUPER 重新规划
    if (!super_goal_sent_ && !super_rviz_mode_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = now;
        goal.header.frame_id = "map";  // SUPER 使用 "map" 坐标系
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        // NaN yaw 会生成 NaN 四元数污染 SUPER 的 yaw 规划，替换为 0
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_hold ? 0.0f : yaw);
        super_goal_pub_.publish(goal);

        super_goal_sent_ = true;
        super_goal_time_ = now;
        last_super_msg_time_ = now;   // 刷新超时计时：等 SUPER 重新出轨迹的空窗不进超时刹车（对齐 ruikang publish_waypoint）
        integral_spx_ = 0.0;
        integral_spy_ = 0.0;
        integral_spz_ = 0.0;
        // 斜率限制以当前实际速度为种子：case 切换瞬间指令速度与实际速度天然连续
        last_super_vx_ = current_velocity.x;
        last_super_vy_ = current_velocity.y;
        last_super_vz_ = current_velocity.z;
        super_slew_timer_ = 0.3f;      // zfix-smooth: 新目标后短暂限幅窗口, 消除换点抽动
        super_pos_hold_ = false;       // zfix-smooth: 新目标退出位置保持
        super_pos_hold_frames_ = 0;
        ROS_INFO("[Super] 新目标 (%.2f, %.2f, %.2f) → 已发布到 /move_base_simple/goal", x, y, z);
    }

    // ====== 轨迹超时保护 & 新鲜轨迹判定 ======
    // SUPER 只在 FOLLOW_TRAJ/EMER_STOP 状态发布 pos_cmd，轨迹结束立即停发；
    // 新 goal 发出到 SUPER 重新出轨迹之间有 0.1~0.3s+ 空窗。
    // ① 超时：断联 → 保持；② 非新鲜：缓存还是旧轨迹尾部（stamp 早于本目标发出时刻）→ 保持，
    // 避免跟踪旧参考、以及新轨迹到达瞬间参考跳变直接打进速度指令（换航点"抽一下"的主因）。
    double dt_since_super =
        last_super_msg_time_.isZero()
            ? 0.0
            : (now - last_super_msg_time_).toSec();
    bool traj_timeout = (super_cmd_received_ && dt_since_super > super_traj_timeout_);
    // rviz 模式：没有"自己发 goal"的基准时刻，轨迹一到就当新鲜（防止一直停在 HOLD）
    bool fresh_cmd = super_rviz_mode_ || (super_cmd_received_ && (super_cmd_.header.stamp > super_goal_time_));

    if (!super_cmd_received_ || traj_timeout || !fresh_cmd)
    {
        super_pos_hold_ = false;       // zfix-smooth: 断供/超时退出位置保持
        super_pos_hold_frames_ = 0;
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0,
                "[Super] Traj timeout! dt=%.2fs > %.2fs → HOLD",
                dt_since_super, super_traj_timeout_);

        // 保持：三轴零速度（经斜率限制，进入/退出都是斜坡而非阶跃）
        target_position.header.stamp = now;
        target_position.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
            (yaw_hold ? mavros_msgs::PositionTarget::IGNORE_YAW : 0);
        double vx_cmd = 0.0, vy_cmd = 0.0, vz_cmd = 0.0;
        slewLimitVel(vx_cmd, vy_cmd, vz_cmd,
                     last_super_vx_, last_super_vy_, last_super_vz_, dt, super_max_accel_);
        target_position.velocity.x = vx_cmd;
        target_position.velocity.y = vy_cmd;
        target_position.velocity.z = vz_cmd;
        if (!yaw_hold) target_position.yaw = yaw;
        return false;
    }

    // ====== 位置误差 & 速度误差（XYZ 三轴） ======
    double ref_px = super_cmd_.position.x;
    double ref_py = super_cmd_.position.y;
    double ref_pz = super_cmd_.position.z;
    double ref_vx = super_cmd_.velocity.x;
    double ref_vy = super_cmd_.velocity.y;
    double ref_vz = super_cmd_.velocity.z;

    double err_x = ref_px - current_position.x;
    double err_y = ref_py - current_position.y;
    double err_z = ref_pz - current_position.z;
    double vel_err_x = ref_vx - current_velocity.x;
    double vel_err_y = ref_vy - current_velocity.y;
    double vel_err_z = ref_vz - current_velocity.z;

    // ====== 无条件积分（固定步长 0.02，与 Zplus 一致）======
    integral_spx_ += err_x * 0.02;
    integral_spy_ += err_y * 0.02;
    integral_spz_ += err_z * 0.02;
    integral_spx_ = std::max(-(double)super_max_integral_, std::min(integral_spx_, (double)super_max_integral_));
    integral_spy_ = std::max(-(double)super_max_integral_, std::min(integral_spy_, (double)super_max_integral_));
    integral_spz_ = std::max(-(double)super_max_integral_, std::min(integral_spz_, (double)super_max_integral_));

    // ====== 速度指令 = v_ref + Kp*err_p + Kv*err_v + Ki*∫err_p（XYZ） ======
    double vx_cmd = ref_vx
                  + super_kp_outer_ * err_x
                  + super_kv_outer_ * vel_err_x
                  + super_ki_outer_ * integral_spx_;

    double vy_cmd = ref_vy
                  + super_kp_outer_ * err_y
                  + super_kv_outer_ * vel_err_y
                  + super_ki_outer_ * integral_spy_;

    double vz_cmd = ref_vz
                  + super_kp_outer_ * err_z
                  + super_kv_outer_ * vel_err_z
                  + super_ki_outer_ * integral_spz_;

    // ====== XY 速度幅值限幅 ======
    double speed = std::sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
    if (speed > super_max_vel_)
    {
        double scale = super_max_vel_ / speed;
        vx_cmd *= scale;
        vy_cmd *= scale;
    }

    // ====== Z 速度限幅 ======
    vz_cmd = std::max(-(double)super_max_vel_z_, std::min(vz_cmd, (double)super_max_vel_z_));

    // ====== 帧间加速度斜率限制（zfix-smooth: 条件化）======
    // 仅在新目标/换点后的短暂窗口(super_slew_timer_)内限幅, 正常跟踪旁路——
    // 持续限幅会与 kv 阻尼竞争, 在到点附近制造慢摆(极限环)
    if (super_slew_timer_ > 0.0f)
    {
        super_slew_timer_ -= dt;
        slewLimitVel(vx_cmd, vy_cmd, vz_cmd,
                     last_super_vx_, last_super_vy_, last_super_vz_, dt, super_max_accel_);
    }
    else
    {
        last_super_vx_ = vx_cmd;
        last_super_vy_ = vy_cmd;
        last_super_vz_ = vz_cmd;
    }

    // ====== 到点位置保持（zfix-smooth: 仿 ruikang HOVER）======
    // SUPER 到达后(WAIT_GOAL)发零速指令: 参考静止+距离近 → 切位置模式,
    // 把最后收敛交给 PX4 位置环, 消除速度模式下外环积分+滞后的慢摆
    if (super_rviz_mode_)
    {
        double ref_spd = std::sqrt(ref_vx * ref_vx + ref_vy * ref_vy + ref_vz * ref_vz);
        double dist_ref = std::sqrt(err_x * err_x + err_y * err_y + err_z * err_z);
        if (super_pos_hold_)
        {
            // zfix-hold: 保持中, 参考明显重新移动(新轨迹)才退出
            if (ref_spd > 0.5) super_pos_hold_ = false;
        }
        else if (ref_spd < 0.1 && dist_ref < 0.5)
        {
            // zfix-hold: 参考接近静止 → 锁存当前位置为保持点(不跟跳动的 ref)
            if (super_pos_hold_frames_ < 100) super_pos_hold_frames_++;
            if (super_pos_hold_frames_ > 5)
            {
                super_pos_hold_ = true;
                super_hold_px_ = current_position.x;
                super_hold_py_ = current_position.y;
                super_hold_pz_ = current_position.z;
            }
        }
        else
        {
            super_pos_hold_frames_ = 0;
        }
    }
    if (super_pos_hold_)
    {
        target_position.header.stamp = now;
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_position.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::FORCE |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
            (yaw_hold ? mavros_msgs::PositionTarget::IGNORE_YAW : 0);
        target_position.position.x = super_hold_px_;
        target_position.position.y = super_hold_py_;
        target_position.position.z = super_hold_pz_;
        target_position.velocity.x = 0;
        target_position.velocity.y = 0;
        target_position.velocity.z = 0;
        return false;
    }

    // ====== 构建 MAVROS 消息（全速度模式：XYZ 速度 + 忽略位置） ======
    target_position.header.stamp = now;
    target_position.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
        (yaw_hold ? mavros_msgs::PositionTarget::IGNORE_YAW : 0);

    target_position.velocity.x = vx_cmd;
    target_position.velocity.y = vy_cmd;
    target_position.velocity.z = vz_cmd;
    // NaN yaw = 保持当前航向（掩码 IGNORE_YAW）；给定 yaw 时跟踪目标 yaw
    // （规划器 yaw 前馈跟踪见新接口 navigationSuperAtt / navigationSuperFF）
    if (!yaw_hold) target_position.yaw = yaw;

    // ====== 调试输出 ======
    ROS_INFO_THROTTLE(0.3,
        "[Super] err_p=(%.3f,%.3f,%.3f) err_v=(%.3f,%.3f,%.3f) int=(%.2f,%.2f,%.2f) "
        "vref=(%.2f,%.2f,%.2f) vcmd=(%.2f,%.2f,%.2f) dist=%.2f deb=%s fresh=%s",
        err_x, err_y, err_z, vel_err_x, vel_err_y, vel_err_z,
        integral_spx_, integral_spy_, integral_spz_,
        ref_vx, ref_vy, ref_vz, vx_cmd, vy_cmd, vz_cmd,
        tolerance(x, y, z),
        super_tol_timing_ ? "ON" : "off",
        fresh_cmd ? "Y" : "N");

    // ====== 诊断日志 ======
    ROS_INFO_THROTTLE(0.3,
        "[DIAG-SUPER] ref_pos=(%.3f,%.3f,%.3f) ref_vel=(%.3f,%.3f,%.3f) | "
        "cur_pos=(%.3f,%.3f,%.3f) cur_vel=(%.3f,%.3f,%.3f) | "
        "vcmd=(%.3f,%.3f,%.3f)",
        ref_px, ref_py, ref_pz, ref_vx, ref_vy, ref_vz,
        current_position.x, current_position.y, current_position.z,
        current_velocity.x, current_velocity.y, current_velocity.z,
        vx_cmd, vy_cmd, vz_cmd);

    // ====== 到达判定（纯位置 Debounce） ======
    // rviz 测试模式：永不判定到达，持续跟踪
    if (super_rviz_mode_)
        return false;

    float dist = tolerance(x, y, z);
    const float kDebounceTime = 0.15f;    // zfix-hold: SUPER receding-horizon 轨迹末端振荡, 飞机停稳点约 0.27m
    const float kArriveTol = std::max(tol, 0.3f);   // zfix-hold: 放宽到达判定, 原 tol=0.2 几乎不满足

    if (dist < kArriveTol)
    {
        if (!super_tol_timing_)
        {
            super_tol_timing_ = true;
            super_tol_entry_time_ = now;
            ROS_INFO("[Super] 进入容差 dist=%.2f<%.2f, debounce %.2fs...",
                     dist, tol, kDebounceTime);
        }
        else if ((now - super_tol_entry_time_).toSec() > kDebounceTime)
        {
            ROS_INFO("[Super] Debounce 完成, 到达！");
            super_tol_timing_ = false;
            super_goal_sent_ = false;
            integral_spx_ = 0.0;
            integral_spy_ = 0.0;
            integral_spz_ = 0.0;
            // 到达帧显式给零速度（经斜率限制）：不把残余速度指令带进下一个 case
            double zx = 0.0, zy = 0.0, zz = 0.0;
            slewLimitVel(zx, zy, zz, last_super_vx_, last_super_vy_, last_super_vz_, dt, super_max_accel_);
            target_position.velocity.x = zx;
            target_position.velocity.y = zy;
            target_position.velocity.z = zz;
            return true;
        }
    }
    else
    {
        super_tol_timing_ = false;
    }
    return false;
}

// ====== navigationSuperRvizPID: 只接收 rviz 打点的 SUPER 测试接口（复用 navigationSuper 控制律） ======
// 与 navigationSuperRviz（cxr 简单 P 律）不同，本接口完整复用 navigationSuper 的
// PID 外环 + 帧间斜率限制 + 轨迹超时保护；但不传坐标、不自己发 goal，
// 只等用户在 rviz 里打点触发 SUPER 重新规划，然后用同一套控制律跟踪。
// 内部 while(ros::ok()) 阻塞运行，永不返回 true（只有节点退出才返回 false）。
bool ASNAV::navigationSuperRvizPID()
{
    ROS_INFO("[SuperRvizPID] 只接收 rviz 打点，跟踪 SUPER 轨迹（navigationSuper 控制律）...");
    ros::Rate rate(50.0);

    super_rviz_mode_ = true;
    super_cmd_received_ = false;   // 重新等一个 rviz 点触发的轨迹
    integral_spx_ = 0.0;
    integral_spy_ = 0.0;
    integral_spz_ = 0.0;
    last_super_call_time_ = ros::Time(0);
    // 斜率限制以当前实际速度为种子，进接口瞬间指令速度连续
    last_super_vx_ = current_velocity.x;
    last_super_vy_ = current_velocity.y;
    last_super_vz_ = current_velocity.z;

    while (ros::ok())
    {
        // rviz 模式下 x/y/z/yaw/tol 参数被忽略：不发 goal、不判到达
        navigationSuper(0.0f, 0.0f, 0.0f, NAN, 0.2f);
        setpointPublish();
        ros::spinOnce();
        rate.sleep();
    }

    super_rviz_mode_ = false;
    return false;   // 永不返回 true
}

// ==================== 姿态推力控制接口（2026-08 新增） ====================
// 控制律完全模仿 ZJU px4ctrl（ego 团队开源 PX4 控制器，姿态+总推力通道）：
//   级联 P→P（位置误差±1m、速度误差±1m/s 饱和）→ 期望加速度 = Kv*vel_err + 前馈加速度
//   → 比力 f = 期望加速度 + (0,0,g)（最小 3 m/s^2 + 倾角限制）
//   → SO(3) 构造期望姿态（zb=f/|f|, xc=(cosψ,sinψ,0), yb=norm(zb×xc), xb=yb×zb）
//   → 归一化推力 = f·(当前机体z轴) / thr2acc，thr2acc = g/悬停油门比例
//   → 发 /mavros/setpoint_raw/attitude（mavros 内部完成 ENU→NED/FRD 转换）
// 断指令时模仿 px4ctrl AUTO_HOVER：冻结当前点为悬停期望，由同一控制器保持。
// navigationSuperFF 进一步模仿 SUPER 团队 OMMPC 的输出接口形态：角速度+推力送 PX4 内环
// （type_mask=IGNORE_ATTITUDE，角速度 = SUPER 平坦输出前馈 + 姿态误差反馈角速度）。

bool ASNAV::computeAttThrust(const tf::Vector3& des_p, const tf::Vector3& des_v,
                             const tf::Vector3& des_a, double des_yaw,
                             tf::Quaternion& q_des, double& thrust)
{
    if (!att_feedback_valid_)
        return false;

    // --- 级联 P→P（px4ctrl computePIDErrorAcc，位置误差±1m、速度误差±1m/s 饱和）---
    static const double kPosErrMax = 1.0, kVelErrMax = 1.0;
    tf::Vector3 p_cur(current_position.x, current_position.y, current_position.z);
    tf::Vector3 v_cur(current_velocity.x, current_velocity.y, current_velocity.z);
    tf::Vector3 acc_err;
    for (int i = 0; i < 3; ++i)
    {
        double pe = std::clamp(des_p[i] - p_cur[i], -kPosErrMax, kPosErrMax);
        double ve = std::clamp(des_v[i] + static_cast<double>(att_kp_) * pe - v_cur[i],
                               -kVelErrMax, kVelErrMax);
        acc_err[i] = static_cast<double>(att_kv_) * ve;
    }

    // --- 比力 f = 期望加速度(含前馈) - 重力（px4ctrl computeLimitedTotalAcc）---
    const double kGravity = 9.81;
    const double kMinCollectiveAcc = 3.0;   // px4ctrl kMinNormalizedCollectiveAcc_
    tf::Vector3 f = acc_err + des_a + tf::Vector3(0.0, 0.0, kGravity);
    if (f.length() < kMinCollectiveAcc)
    {
        f.normalize();
        f *= kMinCollectiveAcc;
    }
    // 倾角限制（px4ctrl：比力限到 max_angle 锥内，保持 z 方向投影）
    double max_ang = static_cast<double>(att_max_angle_) * M_PI / 180.0;
    if (max_ang > 0.0 && max_ang < M_PI_2)
    {
        tf::Vector3 z_unit(0.0, 0.0, 1.0);
        double tilt = z_unit.angle(f);
        if (tilt > max_ang)
        {
            tf::Vector3 rot_axis = z_unit.cross(f);
            if (rot_axis.length() > 1e-6)
            {
                rot_axis.normalize();
                f = tf::quatRotate(tf::Quaternion(rot_axis, max_ang), z_unit)
                    * (f.z() / std::cos(max_ang));
            }
        }
    }

    // --- SO(3) 期望姿态（px4ctrl computeFlatInput 构造）---
    tf::Vector3 zb = f.normalized();
    tf::Vector3 xc(std::cos(des_yaw), std::sin(des_yaw), 0.0);
    tf::Vector3 yc = zb.cross(xc);
    if (yc.length() < 1e-3)  // zb 与 xc 近平行（极端俯仰）退化保护
        yc = tf::Vector3(-std::sin(des_yaw), std::cos(des_yaw), 0.0);
    tf::Vector3 yb = yc.normalized();
    tf::Vector3 xb = yb.cross(zb);
    tf::Matrix3x3 R(xb.x(), yb.x(), zb.x(),
                    xb.y(), yb.y(), zb.y(),
                    xb.z(), yb.z(), zb.z());
    R.getRotation(q_des);

    // --- 归一化推力（px4ctrl computeDesiredCollectiveThrustSignal）---
    // 比力投影到【当前】机体 z 轴，thr2acc = g / 悬停油门比例
    tf::Vector3 body_z = tf::quatRotate(current_attitude_, tf::Vector3(0.0, 0.0, 1.0));
    double acc_norm = f.dot(body_z);
    if (acc_norm < kMinCollectiveAcc) acc_norm = kMinCollectiveAcc;
    double thr2acc = kGravity / std::max(0.05, static_cast<double>(att_hover_percentage_));
    thrust = std::clamp(acc_norm / thr2acc, 0.0, 1.0);
    return true;
}

tf::Vector3 ASNAV::attErrorBodyrates(const tf::Quaternion& q_des) const
{
    // px4ctrl computeFeedBackControlBodyrates：q_e = q_est⁻¹ * q_des，rates = ±2*KAng*q_e.vec()
    const double kMaxRatesFb = 4.0;  // px4ctrl kMaxBodyratesFeedback_
    tf::Quaternion q_e = current_attitude_.inverse() * q_des;
    double sign = q_e.getW() >= 0.0 ? 1.0 : -1.0;
    tf::Vector3 rates(sign * 2.0 * att_kang_r_ * q_e.getX(),
                      sign * 2.0 * att_kang_p_ * q_e.getY(),
                      sign * 2.0 * att_kang_y_ * q_e.getZ());
    rates.setX(std::clamp(rates.x(), -kMaxRatesFb, kMaxRatesFb));
    rates.setY(std::clamp(rates.y(), -kMaxRatesFb, kMaxRatesFb));
    rates.setZ(std::clamp(rates.z(), -kMaxRatesFb, kMaxRatesFb));
    return rates;
}

// ====== navigationSuperAtt：SUPER × px4ctrl 姿态推力接口（2026-08 新增）======
// 与 navigationSuper 的差异：输出通道从"速度"换成"姿态+总推力"（setpoint_raw/attitude），
// 控制律完全模仿 px4ctrl；使用 SUPER pos_cmd 的位置/速度/加速度前馈；
// yaw=NAN 时跟随 SUPER 的 yaw 前馈（att_use_planner_yaw=false 则保持当前 yaw）。
// 目标发布/新鲜轨迹判定与 navigationSuper 一致（共用 super_goal_sent_ / super_goal_time_）。
bool ASNAV::navigationSuperAtt(float x, float y, float z, float yaw, float tol)
{
    ros::Time now = ros::Time::now();
    const bool yaw_hold = std::isnan(yaw);

    // ====== 新目标检测（与 navigationSuper 共用目标状态）======
    if (!super_goal_sent_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = now;
        goal.header.frame_id = "map";  // SUPER 使用 "map" 坐标系
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        // NaN yaw 会生成 NaN 四元数污染 SUPER 的 yaw 规划，替换为 0
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_hold ? 0.0f : yaw);
        super_goal_pub_.publish(goal);

        super_goal_sent_ = true;
        super_goal_time_ = now;
        last_super_msg_time_ = now;   // 刷新超时计时（同 navigationSuper 修复）
        ROS_INFO("[SuperAtt] 新目标 (%.2f, %.2f, %.2f) → 已发布", x, y, z);
    }

    // ====== 超时 & 新鲜轨迹判定（同 navigationSuper）======
    double dt_since_super =
        last_super_msg_time_.isZero()
            ? 0.0
            : (now - last_super_msg_time_).toSec();
    bool traj_timeout = (super_cmd_received_ && dt_since_super > super_traj_timeout_);
    bool fresh_cmd = super_cmd_received_ && (super_cmd_.header.stamp > super_goal_time_);
    bool hover = (!super_cmd_received_ || traj_timeout || !fresh_cmd);

    // ====== 期望状态：跟踪轨迹 或 AUTO_HOVER 冻结（px4ctrl 断指令回退）======
    tf::Vector3 des_p, des_v(0.0, 0.0, 0.0), des_a(0.0, 0.0, 0.0);
    double des_yaw;
    if (hover)
    {
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0, "[SuperAtt] Traj timeout! dt=%.2fs → AUTO_HOVER", dt_since_super);
        if (!att_hover_valid_)
        {
            att_hover_x_ = current_position.x;
            att_hover_y_ = current_position.y;
            att_hover_z_ = current_position.z;
            att_hover_yaw_ = current_yaw;
            att_hover_valid_ = true;
        }
        des_p = tf::Vector3(att_hover_x_, att_hover_y_, att_hover_z_);
        des_yaw = yaw_hold ? att_hover_yaw_ : static_cast<double>(yaw);
    }
    else
    {
        att_hover_valid_ = false;
        des_p = tf::Vector3(super_cmd_.position.x, super_cmd_.position.y, super_cmd_.position.z);
        des_v = tf::Vector3(super_cmd_.velocity.x, super_cmd_.velocity.y, super_cmd_.velocity.z);
        des_a = tf::Vector3(super_cmd_.acceleration.x, super_cmd_.acceleration.y, super_cmd_.acceleration.z);
        des_yaw = yaw_hold
                      ? (att_use_planner_yaw_ ? super_cmd_.yaw : static_cast<double>(current_yaw))
                      : static_cast<double>(yaw);
    }

    // ====== px4ctrl 控制律 → 姿态+推力 ======
    tf::Quaternion q_des;
    double thrust = 0.0;
    if (!computeAttThrust(des_p, des_v, des_a, des_yaw, q_des, thrust))
    {
        ROS_WARN_THROTTLE(1.0, "[SuperAtt] 等待姿态反馈...");
        return false;   // 不更新 target_attitude_，setpointPublish 沿用上一帧
    }

    target_attitude_.header.stamp = now;
    target_attitude_.type_mask =
        mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;   // =7：姿态+推力（px4ctrl 默认通道）
    target_attitude_.orientation.x = q_des.getX();
    target_attitude_.orientation.y = q_des.getY();
    target_attitude_.orientation.z = q_des.getZ();
    target_attitude_.orientation.w = q_des.getW();
    target_attitude_.thrust = thrust;
    attitude_active_time_ = now;

    ROS_INFO_THROTTLE(0.3,
        "[SuperAtt] %s des_p=(%.2f,%.2f,%.2f) des_yaw=%.2f thrust=%.2f dist=%.2f",
        hover ? "HOVER" : "TRACK", des_p.x(), des_p.y(), des_p.z(), des_yaw, thrust,
        tolerance(x, y, z));

    // ====== 到达判定（仅跟踪状态判定；hover 期间不判到达）======
    if (hover) return false;
    float dist = tolerance(x, y, z);
    const float kDebounceTime = 0.15f;

    if (dist < tol)
    {
        if (!satt_tol_timing_)
        {
            satt_tol_timing_ = true;
            satt_tol_entry_time_ = now;
        }
        else if ((now - satt_tol_entry_time_).toSec() > kDebounceTime)
        {
            ROS_INFO("[SuperAtt] Debounce 完成, 到达！");
            satt_tol_timing_ = false;
            super_goal_sent_ = false;
            return true;
        }
    }
    else
    {
        satt_tol_timing_ = false;
    }
    return false;
}

// ====== navigationSuperFF：SUPER 官方 OMMPC 风格接口（角速度+推力 → PX4 内环）======
// SUPER 团队实机控制链路（Science Robotics 2025 论文+官方 issue）：自研 OMMPC(100Hz)
// → 期望角速度(roll/pitch/yaw rate)+推力加速度 → MAVROS → PX4 内环。MPC 本体未开源，
// 本接口按该输出接口形态复现：角速度 = SUPER 平坦输出前馈(super_cmd_.angular_velocity)
// + 姿态误差反馈角速度(px4ctrl computeFeedBackControlBodyrates)；推力与姿态解算同
// navigationSuperAtt（px4ctrl SO(3)+级联P）。type_mask = IGNORE_ATTITUDE → 只发
// body rates + thrust（对应 px4ctrl 的 publish_bodyrate_ctrl 通道）。
bool ASNAV::navigationSuperFF(float x, float y, float z, float yaw, float tol)
{
    ros::Time now = ros::Time::now();
    const bool yaw_hold = std::isnan(yaw);

    // ====== 新目标检测（与 navigationSuper 共用目标状态）======
    if (!super_goal_sent_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = now;
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_hold ? 0.0f : yaw);
        super_goal_pub_.publish(goal);

        super_goal_sent_ = true;
        super_goal_time_ = now;
        last_super_msg_time_ = now;
        ROS_INFO("[SuperFF] 新目标 (%.2f, %.2f, %.2f) → 已发布", x, y, z);
    }

    // ====== 超时 & 新鲜轨迹判定（同 navigationSuper）======
    double dt_since_super =
        last_super_msg_time_.isZero()
            ? 0.0
            : (now - last_super_msg_time_).toSec();
    bool traj_timeout = (super_cmd_received_ && dt_since_super > super_traj_timeout_);
    bool fresh_cmd = super_cmd_received_ && (super_cmd_.header.stamp > super_goal_time_);
    bool hover = (!super_cmd_received_ || traj_timeout || !fresh_cmd);

    // ====== 期望状态：跟踪轨迹 或 AUTO_HOVER 冻结 ======
    tf::Vector3 des_p, des_v(0.0, 0.0, 0.0), des_a(0.0, 0.0, 0.0);
    double des_yaw;
    if (hover)
    {
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0, "[SuperFF] Traj timeout! dt=%.2fs → AUTO_HOVER", dt_since_super);
        if (!att_hover_valid_)
        {
            att_hover_x_ = current_position.x;
            att_hover_y_ = current_position.y;
            att_hover_z_ = current_position.z;
            att_hover_yaw_ = current_yaw;
            att_hover_valid_ = true;
        }
        des_p = tf::Vector3(att_hover_x_, att_hover_y_, att_hover_z_);
        des_yaw = yaw_hold ? att_hover_yaw_ : static_cast<double>(yaw);
    }
    else
    {
        att_hover_valid_ = false;
        des_p = tf::Vector3(super_cmd_.position.x, super_cmd_.position.y, super_cmd_.position.z);
        des_v = tf::Vector3(super_cmd_.velocity.x, super_cmd_.velocity.y, super_cmd_.velocity.z);
        des_a = tf::Vector3(super_cmd_.acceleration.x, super_cmd_.acceleration.y, super_cmd_.acceleration.z);
        des_yaw = yaw_hold
                      ? (att_use_planner_yaw_ ? super_cmd_.yaw : static_cast<double>(current_yaw))
                      : static_cast<double>(yaw);
    }

    // ====== px4ctrl 控制律 → 姿态（用于反馈角速度）+推力 ======
    tf::Quaternion q_des;
    double thrust = 0.0;
    if (!computeAttThrust(des_p, des_v, des_a, des_yaw, q_des, thrust))
    {
        ROS_WARN_THROTTLE(1.0, "[SuperFF] 等待姿态反馈...");
        return false;
    }

    // ====== 角速度 = 姿态误差反馈 + SUPER 平坦输出前馈（OMMPC 输出接口形态）======
    tf::Vector3 rates = attErrorBodyrates(q_des);
    if (!hover)
        rates += tf::Vector3(super_cmd_.angular_velocity.x,
                             super_cmd_.angular_velocity.y,
                             super_cmd_.angular_velocity.z);

    target_attitude_.header.stamp = now;
    target_attitude_.type_mask =
        mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;   // 只发 body rates + thrust
    target_attitude_.body_rate.x = rates.x();
    target_attitude_.body_rate.y = rates.y();
    target_attitude_.body_rate.z = rates.z();
    target_attitude_.thrust = thrust;
    attitude_active_time_ = now;

    ROS_INFO_THROTTLE(0.3,
        "[SuperFF] %s rates=(%.2f,%.2f,%.2f) thrust=%.2f dist=%.2f",
        hover ? "HOVER" : "TRACK", rates.x(), rates.y(), rates.z(), thrust,
        tolerance(x, y, z));

    // ====== 到达判定（仅跟踪状态判定；hover 期间不判到达）======
    if (hover) return false;
    float dist = tolerance(x, y, z);
    const float kDebounceTime = 0.15f;

    if (dist < tol)
    {
        if (!satt_tol_timing_)
        {
            satt_tol_timing_ = true;
            satt_tol_entry_time_ = now;
        }
        else if ((now - satt_tol_entry_time_).toSec() > kDebounceTime)
        {
            ROS_INFO("[SuperFF] Debounce 完成, 到达！");
            satt_tol_timing_ = false;
            super_goal_sent_ = false;
            return true;
        }
    }
    else
    {
        satt_tol_timing_ = false;
    }
    return false;
}

// ====== navigationZplusAtt：EGO × px4ctrl 姿态推力接口（2026-08 新增）======
// 与 navigationZplus 的差异：输出通道换成姿态+总推力，控制律完全模仿 px4ctrl；
// 使用 ego pos_cmd 的位置/速度/加速度前馈（加速度前馈是 Zplus 没有的）。
// ego 坐标转换沿用 Zplus 约定（回调取反一次，这里再取反一次，净效果=原始系）。
bool ASNAV::navigationZplusAtt(float x, float y, float z, float yaw, float tol)
{
    ros::Time now = ros::Time::now();
    const bool yaw_hold = std::isnan(yaw);

    // ====== 新目标检测（与 navigationZplus 共用目标状态）======
    if (!goal_sent_)
    {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = now;
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_hold ? 0.0f : yaw);
        goal_pub_.publish(goal);

        goal_sent_ = true;
        zplus_goal_time_ = now;
        last_ego_msg_time_ = now;   // 刷新超时计时（同 navigationZplus 修复）
        ROS_INFO("[ZplusAtt] 新目标 (%.2f, %.2f, %.2f) → 已发布", x, y, z);
    }

    // ====== 超时判定（ego traj_server 持续发流，无需新鲜度判定）======
    double dt_since_ego =
        last_ego_msg_time_.isZero()
            ? 0.0
            : (now - last_ego_msg_time_).toSec();
    bool traj_timeout = (ego_cmd_received_ && dt_since_ego > zplus_traj_timeout_);
    bool hover = (!ego_cmd_received_ || traj_timeout);

    // ====== 期望状态：跟踪轨迹 或 AUTO_HOVER 冻结 ======
    tf::Vector3 des_p, des_v(0.0, 0.0, 0.0), des_a(0.0, 0.0, 0.0);
    double des_yaw;
    if (hover)
    {
        if (traj_timeout)
            ROS_WARN_THROTTLE(1.0, "[ZplusAtt] Traj timeout! dt=%.2fs → AUTO_HOVER", dt_since_ego);
        if (!att_hover_valid_)
        {
            att_hover_x_ = current_position.x;
            att_hover_y_ = current_position.y;
            att_hover_z_ = current_position.z;
            att_hover_yaw_ = current_yaw;
            att_hover_valid_ = true;
        }
        des_p = tf::Vector3(att_hover_x_, att_hover_y_, att_hover_z_);
        des_yaw = yaw_hold ? att_hover_yaw_ : static_cast<double>(yaw);
    }
    else
    {
        att_hover_valid_ = false;
        // 与 navigationZplus 相同的二次取反约定（净效果 = ego 原始系）
        des_p = tf::Vector3(-ego_cmd_.position.x, -ego_cmd_.position.y, ego_cmd_.position.z);
        des_v = tf::Vector3(-ego_cmd_.velocity.x, -ego_cmd_.velocity.y, ego_cmd_.velocity.z);
        des_a = tf::Vector3(-ego_cmd_.acceleration.x, -ego_cmd_.acceleration.y, ego_cmd_.acceleration.z);
        des_yaw = yaw_hold ? static_cast<double>(current_yaw) : static_cast<double>(yaw);
    }

    // ====== px4ctrl 控制律 → 姿态+推力 ======
    tf::Quaternion q_des;
    double thrust = 0.0;
    if (!computeAttThrust(des_p, des_v, des_a, des_yaw, q_des, thrust))
    {
        ROS_WARN_THROTTLE(1.0, "[ZplusAtt] 等待姿态反馈...");
        return false;
    }

    target_attitude_.header.stamp = now;
    target_attitude_.type_mask =
        mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;   // =7：姿态+推力
    target_attitude_.orientation.x = q_des.getX();
    target_attitude_.orientation.y = q_des.getY();
    target_attitude_.orientation.z = q_des.getZ();
    target_attitude_.orientation.w = q_des.getW();
    target_attitude_.thrust = thrust;
    attitude_active_time_ = now;

    ROS_INFO_THROTTLE(0.3,
        "[ZplusAtt] %s des_p=(%.2f,%.2f,%.2f) thrust=%.2f dist=%.2f",
        hover ? "HOVER" : "TRACK", des_p.x(), des_p.y(), des_p.z(), thrust,
        tolerance(x, y, z));

    // ====== 到达判定（仅跟踪状态判定；hover 期间不判到达）======
    if (hover) return false;
    float dist = tolerance(x, y, z);
    const float kDebounceTime = 0.15f;

    if (dist < tol)
    {
        if (!zatt_tol_timing_)
        {
            zatt_tol_timing_ = true;
            zatt_tol_entry_time_ = now;
        }
        else if ((now - zatt_tol_entry_time_).toSec() > kDebounceTime)
        {
            ROS_INFO("[ZplusAtt] Debounce 完成, 到达！");
            zatt_tol_timing_ = false;
            goal_sent_ = false;
            return true;
        }
    }
    else
    {
        zatt_tol_timing_ = false;
    }
    return false;
}

// 控制Yaw角接口
bool ASNAV::controlYaw(float x, float y, float z, float target_yaw, float wait_sec)
{
    float error = normalize_angle(target_yaw - current_yaw);

    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame =
    mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |                    
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target_position.position.x = x; 
    target_position.position.y = y;
    target_position.position.z = z;
    target_position.yaw = current_yaw; // 实时更新的 Yaw

    if (std::fabs(error) > 0.02f) {
        // 使用定点悬停模式，配合极小的旋转步长 (约 20-30度/秒)
        target_position.position.x = x; // 传入固定的目标点 X
        target_position.position.y = y; // 传入固定的目标点 Y
        target_position.position.z = z; // 传入固定的目标点 Z
        
        // 限制旋转步长 (假设循环 20Hz, 0.05s)
        float max_step = 0.66f; 
        float step = std::min(std::fabs(error), max_step);
        target_position.yaw = current_yaw + (error > 0 ? step : -step);

        return false; // 旋转进行中
    }

    // dropBallon(90, 90);
    
    if (!is_yaw_finished) 
    {
        yaw_finish_time = ros::Time::now();
        is_yaw_finished = true;
        ROS_INFO("偏航到位，开始悬停稳定...");
    }

    // 3. 检查悬停时间是否达到要求
    if ((ros::Time::now() - yaw_finish_time).toSec() < wait_sec) 
    {
        // 旋转到位后，持续发布该点，强制维持悬停
        target_position.position.x = x;
        target_position.position.y = y;
        target_position.yaw = target_yaw;
        return false; // 悬停未结束
    }

    // 4. 全部完成
    is_yaw_finished = false;
    return true; // 可以进入下一阶段
}
// 飞行下降接口
bool ASNAV::flyDown(float descend_z)
{
   position(current_position.x, current_position.y, descend_z, current_yaw);
    if (std::fabs(current_position.z - descend_z) < 0.1f)
    {
     ROS_INFO("已达到目标高度: %.2f m", descend_z);
     return true;
    }
    return false;
}
// 飞行上升接口
bool ASNAV::flyUp(float height)
{
   position(current_position.x, current_position.y, height, current_yaw);
    if (std::fabs(current_position.z - height) < 0.1f)
    {
     ROS_INFO("已达到目标高度: %.2f m", height);
     return true;
    }
    return false;
}
// 自动降落接口
bool ASNAV::autoLand()
{
    if (!is_auto_land) 
    {
        ROS_WARN("自动降落功能未启用");
        return false;
    }
    else
    {
    set_mode("POSCTL");
    ros::Duration(0.5).sleep();
    set_mode("AUTO.LAND");
    ROS_INFO("已切换到AUTO.LAND模式，正在降落...");
    return true;
    }
    return false;
}
// 下视视觉跟随接口
bool ASNAV::trackYoloDown(float max_distance, int tol)
{
    float err_x = yolo_box_info.cameraXCenter - 320.0f;
    float err_y = yolo_box_info.cameraYCenter - 240.0f;

    // 1. PID 控制器计算 (增加 D 项用于刹车)
    float Kp = 0.003f;
    float Ki = 0.0002f;
    float Kd = 0.005f; // 新增：微分系数，用于抑制震荡

    integral_error_x += err_x;
    integral_error_y += err_y;
    // 缩小抗积分饱和的上限，防止累积误差过大导致冲过头
    integral_error_x = std::clamp(integral_error_x, -500.0f, 500.0f);
    integral_error_y = std::clamp(integral_error_y, -500.0f, 500.0f);

    float diff_err_x = err_x - last_err_x;
    float diff_err_y = err_y - last_err_y;

    // 计算基础控制量
    float u_x = Kp * err_x + Ki * integral_error_x + Kd * diff_err_x;
    float u_y = Kp * err_y + Ki * integral_error_y + Kd * diff_err_y;

    // 更新上一次误差
    last_err_x = err_x;
    last_err_y = err_y;

    // 2. 将像素误差映射到机身坐标系 (Body Frame)
    // 假设相机镜头向下：
    // 目标在图像下方 (err_y > 0) -> 无人机需要向后飞 -> body_x 为负
    // 目标在图像右方 (err_x > 0) -> 无人机需要向右飞 -> body_y 为正
    float body_offset_x = -u_y; 
    float body_offset_y = u_x;  // 注：你原代码这里是负的，可能会导致反向，请检查相机安装方向

    // 3. 将机身坐标系旋转到 LOCAL_NED 坐标系
    // 根据无人机当前的偏航角 (current_yaw) 进行 2D 旋转
    float offset_x = body_offset_x * std::cos(current_yaw) - body_offset_y * std::sin(current_yaw);
    float offset_y = body_offset_x * std::sin(current_yaw) + body_offset_y * std::cos(current_yaw);

    // 4. 步长限幅
    float step_mag = std::sqrt(offset_x * offset_x + offset_y * offset_y);
    if (step_mag > max_distance) 
    {
        float scale = max_distance / step_mag;
        offset_x *= scale;
        offset_y *= scale;
    }

    // 5. 发布控制指令
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::FORCE |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target_position.position.x = current_position.x + offset_x;
    target_position.position.y = current_position.y + offset_y;
    target_position.position.z = current_position.z; // 保持当前高度
    target_position.yaw = current_yaw;               // 保持当前航向

    // 6. 成功对齐判定
    if (std::abs(err_x) < tol && std::abs(err_y) < tol)
    {
        ROS_INFO("图像目标已对准");
        // 对准后清除积分，防止下次追踪时带入旧的历史误差
        integral_error_x = 0.0f;
        integral_error_y = 0.0f;
        return true;
    }
    return false;   
}
// 正向视觉跟随接口
bool ASNAV::trackYoloForward(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size)
{
    static bool was_approaching = false;  // 记录是否曾进入逼近状态（用于判断”目标消失=扎破”）

    double data_age = (ros::Time::now() - last_yolo_d435i_time_).toSec();

    // 1. 安全机制：检查目标是否丢失超过 2 秒
    if (data_age > 1.0)
    {
        if (was_approaching)
        {
            // 逼近中目标消失 → 气球已扎破，立即刹车返回
            ROS_INFO("逼近中目标消失(%.1fs)，判定气球已扎破! 刹车!", data_age);
            was_approaching = false;
            target_position.header.stamp = ros::Time::now();
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                        mavros_msgs::PositionTarget::IGNORE_PY |
                                        mavros_msgs::PositionTarget::IGNORE_PZ |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::IGNORE_YAW;
            target_position.velocity.x = 0.0f;
            target_position.velocity.y = 0.0f;
            target_position.velocity.z = 0.0f;
            target_position.yaw_rate = 0.0f;
            return true;
        }

        // 未逼近但数据过期 → 悬停等待，不用旧数据
        ROS_WARN_THROTTLE(1.0, "D435i数据过期(%.1fs)，悬停等待...\"", data_age);
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW;
        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        target_position.yaw_rate = 0.0f;
        return false;
    }

    // 2. 误差计算 (假设相机分辨率为 640x480，请根据实际情况修改 320 和 240)
    float img_center_x = 320.0f;
    float img_center_y = 245.0f;

    // 图像坐标系：右正左负，下正上负
    float err_x = yolo_d435i_box_info_.cameraXCenter - img_center_x;
    float err_y = yolo_d435i_box_info_.cameraYCenter - img_center_y;

    // err_size > 0 表示目标太小（距离太远），err_size < 0 表示目标太大（距离太近）
    float err_size = target_box_height - yolo_d435i_box_info_.boxHeight;

    // 3. P控制计算速度
    float vx = Kp_x * err_size;   // 前后：目标太小→前进，目标太大→后退
    float vy = -(Kp_y * err_x);   // 左右：目标偏右→右侧飞
    float vz = -(Kp_z * err_y);   // 上下：目标偏下→下降 (NED坐标系Z正方向为下)

    // 速度限幅
    vx = std::clamp(vx, -1.0f, 1.0f);
    vy = std::clamp(vy, -1.0f, 1.0f);
    vz = std::clamp(vz, -1.0f, 1.0f);

    // ★ 核心：判断XY是否已对准
    bool is_aligned = (std::abs(err_x) < tol_xy && std::abs(err_y) < tol_xy);

    const char* phase_str = was_approaching ? "逼近中→" : (is_aligned ? "已对准✓" : "对齐中");
    ROS_INFO_THROTTLE(1.0, "跟踪中 | dt=%.2fs | err_x=%+.0f err_y=%+.0f err_size=%+.0f | vx=%.2f vy=%.2f vz=%.2f | %s",
                      data_age, err_x, err_y, err_size,
                      vx, vy, vz, phase_str);

    if (!was_approaching)
    {
        // ═══ 阶段1：初始对齐 ═══
        if (!is_aligned)
        {
            vx = 0.0f;  // 没对准时暂停前进，三轴同时对齐
        }
        else
        {
            was_approaching = true;  // 首次对准 → 锁定，进入逼近阶段
        }
    }
    else
    {
        // ═══ 阶段2：逼近穿刺 ═══
        // 一旦进入就不再回退！不因微小的视角波动重新停下来对齐
        vz = 0.0f;  // 锁定Z轴高度
        // Vx保持计算值（持续向前），Vy保持计算值（持续左右跟踪）
    }

    // 4. 构建 MAVROS 速度控制指令 (BODY_NED)
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW;

    target_position.position.x = 0;
    target_position.position.y = 0;
    target_position.position.z = 0;

    target_position.velocity.x = vx;    // 前后速度（对齐后启用，动态逼近）
    target_position.velocity.y = vy;    // 左右侧飞跟随（持续对齐）
    target_position.velocity.z = vz;    // 上下高度跟随（持续对齐）
    target_position.yaw_rate = 0.0f;

    // 5. 判定到达：XY已对准 且 距离足够近（box高度接近目标值）
    if (is_aligned && std::abs(err_size) < tol_size)
    {
        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        ROS_INFO("目标已到达，气球扎破!");
        was_approaching = false;
        return true;
    }

    return false;
}
// 视觉追踪接口
bool ASNAV::trackYoloing(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size)
{
   if ((ros::Time::now() - last_yolo_time_).toSec() > 2.0)
    {
        ROS_WARN_THROTTLE(1.0, "目标丢失超过2秒，进入悬停模式 (HOVER)");
        target_position.header.stamp = ros::Time::now();
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // 机体坐标系
        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW;
        // 速度清零，保持航向
        target_position.velocity.x = 0.0f;
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        target_position.yaw_rate = 0.0f; 
        return false;
    } 
    // 误差计算 (假设相机分辨率为 640x480，请根据实际情况修改 320 和 240)
    float img_center_x = 320.0f;
    float img_center_y = 240.0f;

    // 图像坐标系：右正左负，下正上负
    float err_x = yolo_box_info.cameraXCenter - img_center_x; 
    float err_y = yolo_box_info.cameraYCenter - img_center_y; 

    float err_size = target_box_height - yolo_box_info.boxHeight;

    // 3. P控制计算速度
    float vx = Kp_x * err_size;  // X轴：前后跟随
    // 调整偏航
    float yaw_rate_val = -(Kp_y * err_x);
    // 目标在右(err_x > 0) -> 飞机向右侧飞 (Vy > 0)
    // float vy = -(Kp_y * err_x);
    // 目标在下(err_y > 0) -> 飞机下降 (NED坐标系中，Z轴正方向为下，Vz > 0)
    float vz = -(Kp_z * err_y);

    // 速度限幅，防止剧烈晃动 (限制在 ±1.0 m/s)
    yaw_rate_val = std::clamp(yaw_rate_val, -1.0f, 1.0f);
    // vy = std::clamp(vy, -1.0f, 1.0f);
    vz = std::clamp(vz, -1.0f, 1.0f);
    vx = std::clamp(vx, -1.0f, 1.0f);

    // 4. 构建 MAVROS 速度控制指令
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; // 注意这里换成了 BODY_NED
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW; // 忽略偏航角，但控制偏航角速度

    target_position.velocity.x = vx;    // 前后速度
    target_position.velocity.y = 0.0f;    // 左右侧飞跟随
    target_position.velocity.z = vz;    // 上下高度跟随
    target_position.yaw_rate = yaw_rate_val;    // 保持机头航向不变
    return false;
}
// 重置指令对象接口
void ASNAV::reset_target()
{
    // 彻底重新构造对象，清空所有 position 和 velocity 残留
    target_position = mavros_msgs::PositionTarget();
    
    // 设置一个“全部忽略”的掩码，确保安全
    target_position.type_mask = 0x7FF; 
    
    // 默认回到局部坐标系
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    
    ROS_INFO("指令对象已重置，消除残留数据");
}

bool ASNAV::interceptBalloon(float charge_speed, float Kp_y, float Kp_z, float pop_box_height)
{
    static float last_seen_box_height = 0.0f;

    // 1. 刺破判定 (因为相机离气球还有近2米时气球就破了，所以判定框的高度要根据实测调整)
    if ((ros::Time::now() - last_yolo_time_).toSec() > 0.5)
    {
        // pop_box_height 是你需要实测的：当两米长的枪尖顶到气球时，气球在画面里的高度是多少？
        if (last_seen_box_height > pop_box_height * 0.8) // 稍微放宽一点判定条件
        {
            ROS_INFO("检测到气球爆炸！刺破成功！开始紧急倒车！");
            last_seen_box_height = 0.0f; 
            target_position.type_mask = 0x7BF;
            target_position.velocity.x = 0; target_position.velocity.y = 0; target_position.velocity.z = 0;
            return true; // 返回 true 触发战术撤退
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "目标跟丢，悬停等待...");
            target_position.type_mask = 0x7BF;
            target_position.velocity.x = 0; target_position.velocity.y = 0; target_position.velocity.z = 0;
            return false;
        }
    }

    float err_x = yolo_box_info.cameraXCenter - 320.0f;
    float err_y = yolo_box_info.cameraYCenter - 270.0f;
    last_seen_box_height = yolo_box_info.boxHeight;

    // 2. 长矛冲锋分级控制策略
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

    // 这个值你需要根据你的相机焦距和实际情况去操场上推一推飞机来测定
    float danger_box_height = 100.0f; 

    if (yolo_box_info.boxHeight < danger_box_height)
    {
        // ==========================================
        // 远距离阶段：常规追踪（允许微调 Yaw 来大范围跟目标）
        // ==========================================
        vx = charge_speed * 0.5f; // 远距离慢慢飞，保证对准
        vy = std::clamp(-(Kp_y * err_x), -1.0f, 1.0f);
        vz = std::clamp(-(Kp_z * err_y), -1.0f, 1.0f);
    }
    else
    {
        // ==========================================
        // 近距离“长矛冲锋”阶段：绝对锁死 Yaw，全速平移追击！
        // ==========================================
        vx = charge_speed; // 全速冲锋！
        
        // 因为不转机头了，所以必须加大左右和上下的 P 控制力度，纯靠侧飞来追踪转动的气球
        vy = std::clamp(-(Kp_y * err_x), -1.0f, 1.0f); 
        vz = std::clamp(-(Kp_z * err_y), -1.0f, 1.0f);
        
        ROS_INFO_THROTTLE(0.5, "进入冲锋范围，已锁死机头，平移追击！");
    }

    // 3. 发布 MAVROS 速度指令
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW;
                                
    target_position.velocity.x = vx;
    target_position.velocity.y = vy;
    target_position.velocity.z = vz;
    target_position.yaw_rate = 0.0f;
    

    return false;
}

bool ASNAV::escapeBackward(float distance, float tol)
{
    static bool is_escaping = false;
    static float escape_target_x, escape_target_y, escape_target_z;

    if (!is_escaping)
    {
        // 计算正后方的绝对坐标
        escape_target_x = current_position.x - distance * std::cos(current_yaw);
        escape_target_y = current_position.y - distance * std::sin(current_yaw);
        escape_target_z = current_position.z;
        is_escaping = true;
        ROS_WARN("启动战术撤退，向后退避 %.1f 米！", distance);
    }

    bool reached = position(escape_target_x, escape_target_y, escape_target_z, current_yaw, tol);
    
    if (reached)
    {
        is_escaping = false;
        return true;
    }
    return false;
}

bool ASNAV::attackBalloon(float charge_speed)
{
    // 内部战术状态：
    // 0 = 索敌与冲锋阶段 (ATTACK)
    // 1 = 战术后退与判定阶段 (RETREAT & ASSESS)
    static int combat_state = 0; 
    static ros::Time retreat_start_time; // 记录开始后退的时间

    // --- 比赛核心参数（必须线下实测） ---
    const float LANCE_OVERSHOOT_HEIGHT = 193.0f; // 杆长设定值：框高于此值说明已越过气球刺空了
    const double POP_CONFIRM_TIME = 2.5;         // 判定戳爆所需的时间：几秒内没看到气球就算爆了
    const float KP_Y = 0.005f;
    const float KP_Z = 0.005f;

    // 当前帧是否能看到气球 (0.3秒内有更新就算看到，过滤掉单帧闪烁)
    bool is_visible = (ros::Time::now() - last_yolo_time_).toSec() < 0.3;

    // ====================================================
    // 状态 0：索敌与冲锋
    // ====================================================
    if (combat_state == 0) 
    {
        if (is_visible)
        {
            // 1. 判定是否刺空（越界）
            if (yolo_box_info.boxHeight > LANCE_OVERSHOOT_HEIGHT)
            {
                ROS_WARN("框高度(%.1f) > 设定杆长，刺偏越界！启动后退重试！", yolo_box_info.boxHeight);
                combat_state = 1;
                retreat_start_time = ros::Time::now(); // 开始计时
                return false;
            }

            // 2. 正常长矛冲锋逻辑（锁死机头，平移追击）
            float err_x = yolo_box_info.cameraXCenter - 320.0f;
            float err_y = yolo_box_info.cameraYCenter - 270.0f;

            float vy = std::clamp(-KP_Y * err_x, -1.0f, 1.0f);
            float vz = std::clamp(-KP_Z * err_y, -1.0f, 1.0f);
            
            target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            target_position.type_mask = 1471; // 忽略位置，使用速度
            target_position.velocity.x = charge_speed; // 向前冲锋
            target_position.velocity.y = vy;
            target_position.velocity.z = vz;
            target_position.yaw_rate = 0.0f; // 绝对锁死机头！
        }
        else
        {
            // 3. 冲锋途中气球突然消失（可能爆了，也可能被机械臂转走了）
            ROS_INFO("目标突然消失，启动后退进行扎破确认...");
            combat_state = 1;
            retreat_start_time = ros::Time::now();
        }
    }
    // ====================================================
    // 状态 1：后退与判定（容错与确认机制）
    // ====================================================
    else if (combat_state == 1) 
    {
        // 1. 持续发送后退指令
        target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        target_position.type_mask = 1471;
        target_position.velocity.x = -0.3f; // 以 0.8m/s 的速度往后退
        target_position.velocity.y = 0.0f;
        target_position.velocity.z = 0.0f;
        target_position.yaw_rate = 0.0f;

        // 2. 判定 A：捕捉到气球身影 -> 继续追踪继续戳
        // 注意：加了 * 0.8f 是个专业技巧（滞回区间），防止它在边界值疯狂左右横跳
        if (is_visible && yolo_box_info.boxHeight < (LANCE_OVERSHOOT_HEIGHT * 0.8f)) 
        {
            ROS_INFO("后退拉开距离后重新捕捉到气球，继续发起冲锋！");
            combat_state = 0; // 切回冲锋状态
            return false;
        }

        // 3. 判定 B：几秒内都没有气球 -> 判定戳爆
        if (!is_visible && (ros::Time::now() - retreat_start_time).toSec() > POP_CONFIRM_TIME)
        {
            ROS_INFO("后退验证完成，%.1f 秒内未见目标，判定气球已扎爆！", POP_CONFIRM_TIME);
            combat_state = 0; // 重置内部状态，为打下一个气球做准备
            
            // 发送一次刹车指令清空速度
            target_position.velocity.x = 0.0f; 
            return true; // 告诉主循环：这个目标搞定了！
        }
    }

    return false;
}
// 投放接口
bool ASNAV::dropBallon(int pwm_5, int pwm_6)

{
    ros::Rate rate(20); // 20Hz 发送频率，每次 0.05 秒
    for(int i = 0; i < 6; ++i )
    {
        // 1. 发送舵机控制指令
        lib_pwm_control(pwm_5, pwm_6);
        mavros_cmd_command_client_.call(lib_ctrl_pwm);
        ROS_INFO_ONCE("投放指令已发送，PWM 5: %d, PWM 6: %d", pwm_5, pwm_6);
        
        // 2. 维持 OFFBOARD 模式的心跳指令 (极为关键)
        setpointPublish(); 
        
        // 3. 延时，让循环总耗时达到 1.5 秒左右，给舵机响应时间
        ros::spinOnce();
        rate.sleep(); 
    }
    return true; // 循环结束后再返回
}
// 投放打靶接口
bool ASNAV::putShoot(float x, float y, float z, float yaw, float tol)
{
    static int shoot_phase = 0;
    static std::string target_letter;     // 悬停时保存的字母 A 或 B
    static ros::Time arrive_time;         // 到达靶点的时间戳
    static bool letter_captured = false;  // 是否已在悬停阶段捕捉到字母

    // ====================================================
    // 阶段 0：导航到目标点 + 悬停3s，期间 YOLO 捕捉 A/B
    // ====================================================
    if (shoot_phase == 0)
    {
        bool nav_done = navigationWithPosition(x, y, z, yaw, tol, 3.0f);
        if (nav_done)
        {
            // ★ 导航完成后（已在投放点上方悬停2秒）才捕捉字母
            // 避免飞行途中经过其他字母导致误捕获
            double yolo_age = (ros::Time::now() - last_yolo_time_).toSec();
            bool yolo_fresh = (yolo_age < 1.0f);

            if (yolo_fresh && !yolo_box_info.Class.empty())
            {
                target_letter = yolo_box_info.Class;
                letter_captured = true;
                ROS_INFO("[投放打靶] 悬停完成，捕捉到字母: %s (YOLO数据%.1fs前)",
                         target_letter.c_str(), yolo_age);
            }
            else
            {
                ROS_WARN("[投放打靶] 悬停完成但未获得新鲜字母 (YOLO年龄=%.1fs, Class=%s)",
                         yolo_age,
                         yolo_box_info.Class.empty() ? "(空)" : yolo_box_info.Class.c_str());
            }

            ROS_INFO("[投放打靶] 导航悬停完成，目标字母: %s，开始投放",
                     letter_captured ? target_letter.c_str() : "未识别");
            shoot_phase = 1;
        }
        return false;
    }

    // ====================================================
    // 阶段 1：投放 dropBallon(0, 100)
    // ====================================================
    if (shoot_phase == 1)
    {
        dropBallon(100, 0);
        ROS_INFO("[投放打靶] 已投放，飞向第一个靶点 (1.2, 1.0, 1)");
        shoot_phase = 2;
        return false;
    }

    // ====================================================
    // 阶段 2：飞到第一个靶点 (0, -2, fly_height)
    // ====================================================
    if (shoot_phase == 2)
    {
        bool arrived = positionSmooth(x, -1.8f, descend_z, tol, 0.0f);
        if (arrived)
        {
            arrive_time = ros::Time::now();
            shoot_phase = 3;
            ROS_INFO("[投放打靶] 到达第一个靶点，等待 YOLO 识别...");
        }
        return false;
    }

    // ====================================================
    // 阶段 3：悬停于第一靶点，等待 D435i 识别并判断
    // ====================================================
    if (shoot_phase == 3)
    {
        position(x, -1.8f, descend_z, 0.0f, tol);

        if (last_yolo_d435i_time_ > arrive_time && !yolo_d435i_box_info_.Class.empty())
        {
            std::string detected = yolo_d435i_box_info_.Class;
            ROS_INFO("[投放打靶] 第一个靶点(D435i)识别: %s (目标: %s)",
                     detected.c_str(), target_letter.c_str());

            if (detected == target_letter)
            {
                ROS_INFO("[投放打靶] 第一个靶点匹配！下降到%.2fm打靶...", descend_z);
                shoot_phase = 51;
                return false;
            }
            else
            {
                ROS_INFO("[投放打靶] 不匹配，飞向第二个靶点 (0, -3, %.2f)", fly_height);
                shoot_phase = 4;
                return false;
            }
        }

        if ((ros::Time::now() - arrive_time).toSec() > 3.0f)
        {
            ROS_WARN("[投放打靶] 第一个靶点D435i识别超时，飞向第二个靶点");
            shoot_phase = 4;
        }
        return false;
    }

    // ====================================================
    // 阶段 4：飞到第二个靶点 (0, -3, fly_height)
    // ====================================================
    if (shoot_phase == 4)
    {
        bool arrived = positionSmooth(x, -2.7f, descend_z, tol, 0.0f);
        if (arrived)
        {
            ROS_INFO("[投放打靶] 到达第二个靶点，下降到%.2fm打靶", descend_z);
            shoot_phase = 52;
        }
        return false;
    }

    // ====================================================
    // 阶段 51：第一靶点降高 (0, -2, descend_z)
    // ====================================================
    if (shoot_phase == 51)
    {
        bool descended = flyDown(fly_height);
        if (descended)
        {
            ROS_INFO("[投放打靶] 已降至%.2fm，激光打靶", descend_z);
            shoot_phase = 6;
        }
        return false;
    }

    // ====================================================
    // 阶段 52：第二靶点降高 (0, -3, descend_z)
    // ====================================================
    if (shoot_phase == 52)
    {
        bool descended = flyDown(fly_height);
        if (descended)
        {
            ROS_INFO("[投放打靶] 已降至%.2fm，激光打靶", descend_z);
            shoot_phase = 6;
        }
        return false;
    }

    // ====================================================
    // 阶段 6：开火打靶 dropBallon(100, 100)
    // ====================================================
    if (shoot_phase == 6)
    {
        dropBallon(100, 100);
        ROS_INFO("[投放打靶] 打靶完成，关闭激光");
        shoot_phase = 7;
        return false;
    }

    // ====================================================
    // 阶段 7：关闭激光 dropBallon(100, 0)
    // ====================================================
    if (shoot_phase == 7)
    {
        dropBallon(100, 0);
        ROS_INFO("[投放打靶] 激光已关闭，任务结束 ✓");
        shoot_phase = 0;
        letter_captured = false;
        return true;
    }

    return false;
}
// 投放优化接口
bool ASNAV::putShootSimple(float x, float y, float z, float yaw, float tol)
{
    static int shoot_phase = 0;
    static std::string target_letter;     // 悬停时保存的字母 A 或 B
    static ros::Time arrive_time;         // 到达靶点的时间戳
    static bool letter_captured = false;  // 是否已在悬停阶段捕捉到字母

    // ====================================================
    // 阶段 0：导航到投放点 + 悬停完成后捕捉 A/B（与 putShoot 相同）
    // ====================================================
    if (shoot_phase == 0)
    {
        bool nav_done = navigationWithPosition(x, y, z, yaw, tol, 2.0f);
        if (nav_done)
        {
            double yolo_age = (ros::Time::now() - last_yolo_time_).toSec();
            bool yolo_fresh = (yolo_age < 1.0f);

            if (yolo_fresh && !yolo_box_info.Class.empty())
            {
                target_letter = yolo_box_info.Class;
                letter_captured = true;
                ROS_INFO("[单靶打靶] 悬停完成，捕捉到字母: %s (YOLO数据%.1fs前)",
                         target_letter.c_str(), yolo_age);
            }
            else
            {
                ROS_WARN("[单靶打靶] 悬停完成但未获得新鲜字母 (YOLO年龄=%.1fs, Class=%s)",
                         yolo_age,
                         yolo_box_info.Class.empty() ? "(空)" : yolo_box_info.Class.c_str());
            }

            ROS_INFO("[单靶打靶] 导航悬停完成，目标字母: %s，开始投放",
                     letter_captured ? target_letter.c_str() : "未识别");
            shoot_phase = 1;
        }
        return false;
    }

    // ====================================================
    // 阶段 1：投放 dropBallon(100, 0)（与 putShoot 相同）
    // ====================================================
    if (shoot_phase == 1)
    {
        dropBallon(100, 0);
        ROS_INFO("[单靶打靶] 已投放，飞向唯一靶点 (%.2f, %.2f, %.2f)", x, y, descend_z);
        shoot_phase = 2;
        return false;
    }

    // ====================================================
    // 阶段 2：飞到唯一靶点
    // ====================================================
    if (shoot_phase == 2)
    {
        bool arrived = position(x, -1.6, descend_z, 0.0f, tol);
        if (arrived)
        {
            arrive_time = ros::Time::now();
            shoot_phase = 3;
            ROS_INFO("[单靶打靶] 到达靶点，等待 D435i 识别...");
        }
        return false;
    }

    // ====================================================
    // 阶段 3：悬停于靶点，等待 D435i 识别到即下降
    // ====================================================
    if (shoot_phase == 3)
    {
        position(x, -1.6, descend_z, 0.0f, tol);

        if (last_yolo_d435i_time_ > arrive_time && !yolo_d435i_box_info_.Class.empty())
        {
            std::string detected = yolo_d435i_box_info_.Class;
            ROS_INFO("[单靶打靶] 靶点(D435i)识别: %s (目标: %s)",
                     detected.c_str(), target_letter.c_str());

            if (detected == target_letter)
            {
                ROS_INFO("[单靶打靶] 靶点匹配！下降到%.2fm打靶...", descend_z);
            }
            else
            {
                ROS_WARN("[单靶打靶] 字母不匹配(%s vs %s)，仍下降打靶",
                         detected.c_str(), target_letter.c_str());
            }
            shoot_phase = 4;
            return false;
        }

        if ((ros::Time::now() - arrive_time).toSec() > 3.0f)
        {
            ROS_WARN("[单靶打靶] D435i识别超时，直接下降打靶");
            shoot_phase = 4;
        }
        return false;
    }

    // ====================================================
    // 阶段 4：下降至打靶高度
    // ====================================================
    if (shoot_phase == 4)
    {
        bool descended = flyDown(fly_height);
        if (descended)
        {
            ROS_INFO("[单靶打靶] 已降至%.2fm，激光打靶", descend_z);
            shoot_phase = 5;
        }
        return false;
    }

    // ====================================================
    // 阶段 5：开火打靶 dropBallon(100, 100)
    // ====================================================
    if (shoot_phase == 5)
    {
        dropBallon(100, 100);
        ROS_INFO("[单靶打靶] 打靶完成，关闭激光");
        shoot_phase = 6;
        return false;
    }

    // ====================================================
    // 阶段 6：关闭激光 dropBallon(100, 0)
    // ====================================================
    if (shoot_phase == 6)
    {
        dropBallon(100, 0);
        ROS_INFO("[单靶打靶] 激光已关闭，任务结束 ✓");
        shoot_phase = 0;
        letter_captured = false;
        return true;
    }

    return false;
}
// AR标签跟踪降落接口
bool ASNAV::arTrackLanding(float ground_z, float altitude, float max_error, float vel_set, float camera_offset_x, float camera_offset_y)
{
    static bool flag_init_position = false;
    static float init_position_z_take_off = 0;
    static float last_position_x = 0;
    static float last_position_y = 0;
    static bool flag_arrive = false;
    static float current_altitude = 1.0;

    // 初始化：使用用户传入的地面Z坐标，而非当前高度（避免飞到ar码上空后记录错误基准）
    if (!flag_init_position && current_position.z != 0)
    {
        init_position_z_take_off = ground_z;
        last_position_x = current_position.x;
        last_position_y = current_position.y;
        current_altitude = altitude;
        flag_init_position = true;
        flag_arrive = false;
        ROS_INFO("[AR降落] 初始化完成，地面Z: %.2f, 跟踪高度: %.2f(相对地面)，检测到任意AR码即跟踪",
                 init_position_z_take_off, altitude);
    }

    if (!flag_init_position) return false;

    // 设置坐标系与航向
    target_position.header.stamp = ros::Time::now();
    target_position.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target_position.yaw = current_yaw;

    if (ar_marker_found_ && !flag_arrive)
    {
        // 相机安装位置偏移修正：摄像头在机头前方0.12m，朝下安装
        // 相机坐标系：相机X→无人机Y(左右)，相机Y→无人机X(前后)
        // offset补偿后，使无人机重心（而非摄像头）对准AR码
        float err_x = ar_position_detec_x_ - camera_offset_x;
        float err_y = ar_position_detec_y_ - camera_offset_y;

        // 识别到ar码且对准（误差在阈值内）
        if (fabs(err_x) < max_error && fabs(err_y) < max_error)
        {
            // 记录当前水平位置，防止风或GPS误差导致飘移
            last_position_x = current_position.x;
            last_position_y = current_position.y;

            // 高度判断：小于0.6m则触发降落
            if (current_altitude < 0.6)
            {
                flag_arrive = true;
                ROS_INFO("[AR降落] 已对准AR码，高度<0.6m，触发最终降落");
            }
            else
            {
                // 递减降低高度
                current_altitude -= 0.01;
                ROS_INFO_THROTTLE(1.0, "[AR降落] 已对准AR码(含相机偏移修正)，当前高度: %.2f m，持续下降中...", current_altitude);
            }

            // 位置控制：保持水平位置，调整高度
            target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                        mavros_msgs::PositionTarget::IGNORE_VY |
                                        mavros_msgs::PositionTarget::IGNORE_VZ |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::FORCE |
                                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.position.x = last_position_x;
            target_position.position.y = last_position_y;
            target_position.position.z = init_position_z_take_off + current_altitude;
        }
        else
        {
            // 未对准，速度控制移动到ar码上方
            // 摄像头朝下安装：摄像头X对应无人机Y(左右)，摄像头Y对应无人机X(前后)
            if (err_x >= max_error)
                target_position.velocity.y = -vel_set;
            else if (err_x <= -max_error)
                target_position.velocity.y = vel_set;
            else
                target_position.velocity.y = 0;

            if (err_y >= max_error)
                target_position.velocity.x = -vel_set;
            else if (err_y <= -max_error)
                target_position.velocity.x = vel_set;
            else
                target_position.velocity.x = 0;

            ROS_INFO_THROTTLE(1.0, "[AR降落] 检测到AR码，未对准 | err_x: %.3f err_y: %.3f (已含offset修正) | 速度 vx: %.2f vy: %.2f",
                              err_x, err_y,
                              target_position.velocity.x, target_position.velocity.y);

            target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                        mavros_msgs::PositionTarget::IGNORE_PY |
                                        mavros_msgs::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                                        mavros_msgs::PositionTarget::FORCE |
                                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            target_position.position.z = init_position_z_take_off + current_altitude;

            // 更新水平位置，确保丢失目标时能保持位置
            last_position_x = current_position.x;
            last_position_y = current_position.y;
        }
    }
    else if (!flag_arrive)
    {
        // 未识别到ar码，保持当前位置悬停
        ROS_INFO_THROTTLE(2.0, "[AR降落] 未检测到AR码，保持悬停 | 当前Z: %.2f 目标Z: %.2f",
                          current_position.z,
                          init_position_z_take_off + current_altitude);

        target_position.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        target_position.velocity.x = 0;
        target_position.velocity.y = 0;
        target_position.position.x = last_position_x;
        target_position.position.y = last_position_y;
        target_position.position.z = init_position_z_take_off + current_altitude;
    }

    // 最终降落：已对准AR码且高度足够低，直接调用autoLand切换降落模式
    if (flag_arrive)
    {
        ROS_INFO("[AR降落] 已对准，调用autoLand切换AUTO.LAND模式降落");
        autoLand();
        flag_init_position = false;
        flag_arrive = false;
        return true;
    }

    return false;
}



// MAVROS状态回调
void ASNAV::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
// MAVROS位置回调
void ASNAV::mavros_local_position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position.x = msg->pose.position.x;
    current_position.y = msg->pose.position.y;
    current_position.z = msg->pose.position.z;

    //提取并转换姿态（四元数转Yaw）
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat); // 注意这里比 pose.pose 少了一层 pose
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = static_cast<float>(yaw);
    current_attitude_ = quat;   // 姿态推力接口用：完整姿态四元数
    att_feedback_valid_ = true;
    
}
// MAVROS速度回调 (用于 Zplus 的 Kv 速度误差阻尼项)
void ASNAV::mavros_local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_velocity.x = msg->twist.linear.x;
    current_velocity.y = msg->twist.linear.y;
    current_velocity.z = msg->twist.linear.z;
}
// ego速度回调
void ASNAV::ego_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    ego_cmd_ = *msg;
    // lio_to_mavros 桥 lidar_yaw_offset=0，位置不旋转 → LIO 系 = PX4 local，默认无需旋转；
    // 仅当需要 180° 补偿时置 ego_rotate_180=true（仿 super_rotate_180）
    if (ego_rotate_180_)
    {
        float px = ego_cmd_.position.x, py = ego_cmd_.position.y;
        ego_cmd_.position.x = -px;  ego_cmd_.position.y = -py;
        float vx = ego_cmd_.velocity.x, vy = ego_cmd_.velocity.y;
        ego_cmd_.velocity.x = -vx;  ego_cmd_.velocity.y = -vy;
    }
    ego_cmd_received_ = true;
    last_ego_msg_time_ = ros::Time::now();
}
// SUPER 规划器位置指令回调
void ASNAV::super_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    super_cmd_ = *msg;
    // SUPER 在 "world" 坐标系下规划，如果 world 系与 MAVROS local 系不一致
    // 可通过 super_rotate_180_ 参数控制是否需要旋转 180°（仅 XY，Z 由 MAVROS 处理）
    if (super_rotate_180_)
    {
        float px = super_cmd_.position.x, py = super_cmd_.position.y;
        super_cmd_.position.x = -px;  super_cmd_.position.y = -py;
        float vx = super_cmd_.velocity.x, vy = super_cmd_.velocity.y;
        super_cmd_.velocity.x = -vx;  super_cmd_.velocity.y = -vy;
    }
    super_cmd_received_ = true;
    last_super_msg_time_ = ros::Time::now();
}
// 判断容差函数
float ASNAV::tolerance(float x, float y, float z) const
{
    return std::sqrt(std::pow(current_position.x - x, 2) +
                     std::pow(current_position.y - y, 2) +
                     std::pow(current_position.z - z, 2));
    ROS_INFO_THROTTLE(1, "可以接受");
}
// 发布目标位置函数
void ASNAV::setpointPublish()
{
    // 姿态类接口（navigationSuperAtt/SuperFF/ZplusAtt）最近 0.2s 内被调用过 → 发姿态推力设定值；
    // 否则发位置/速度设定值。主循环每帧无条件调用本函数，切换接口类型时自动回落到位置通道。
    if (!attitude_active_time_.isZero() &&
        (ros::Time::now() - attitude_active_time_).toSec() < 0.2)
        mavros_setpoint_raw_attitude_pub_.publish(target_attitude_);
    else
        mavros_setpoint_raw_local_pub_.publish(target_position);
}
// 设置飞行模式函数
void ASNAV::set_mode(string mode)
{
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.custom_mode = mode;
    if (set_mode_client_.call(mode_msg) && mode_msg.response.mode_sent)
    {
        ROS_INFO("已切换到模式: %s", mode.c_str());
    }
    else
    {
        ROS_ERROR("切换模式失败: %s", mode.c_str());
    }
}
// yolo回调
void ASNAV::yolo_info_cb(const yolov11_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    if (msg->bounding_boxes.empty())
    {
    // ROS_WARN_THROTTLE(1, "YOLO识别结果为空");
    return;
    }
    // 遍历识别框，只取字母 A/B，忽略干扰物
    bool found_target = false;
    for (const auto& box : msg->bounding_boxes)
    {
        if (box.Class == "A" || box.Class == "B")
        {
            yolo_box_info.Class = box.Class;
            yolo_box_info.cameraXCenter = (box.xmax + box.xmin) * 0.5f;
            yolo_box_info.cameraYCenter = (box.ymax + box.ymin) * 0.5f;
            yolo_box_info.boxHeight = box.ymax - box.ymin;

            last_yolo_time_ = ros::Time::now();

            // ROS_INFO_THROTTLE(1, "成功锁定目标 : %s (%.2f, %.2f)",
            //                   yolo_box_info.Class.c_str(),
            //                   yolo_box_info.cameraXCenter,
            //                   yolo_box_info.cameraYCenter);

            found_target = true;
            break;
        }
    }

    if (!found_target)
    {
        ROS_WARN_THROTTLE(1, "未在当前帧中找到目标A/B");
    }
}
// D435i前视YOLO回调（与单目回调逻辑一致，写入独立的d435i存储区）
void ASNAV::yolo_d435i_info_cb(const yolov11_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    if (msg->bounding_boxes.empty())
    {
        // ROS_WARN_THROTTLE(1, "D435i YOLO识别结果为空");
        return;
    }
    bool found_target = false;
    for (const auto& box : msg->bounding_boxes)
    {
        //睿抗
        if (box.Class == "A" || box.Class == "B")
        {
            yolo_d435i_box_info_.Class = box.Class;
            yolo_d435i_box_info_.cameraXCenter = (box.xmax + box.xmin) * 0.5f;
            yolo_d435i_box_info_.cameraYCenter = (box.ymax + box.ymin) * 0.5f;
            yolo_d435i_box_info_.boxHeight = box.ymax - box.ymin;

            last_yolo_d435i_time_ = ros::Time::now();

            // ROS_INFO_THROTTLE(1, "D435i 锁定目标: %s (%.2f, %.2f)",
            //                   yolo_d435i_box_info_.Class.c_str(),
            //                   yolo_d435i_box_info_.cameraXCenter,
            //                   yolo_d435i_box_info_.cameraYCenter);

            found_target = true;
            break;
        }

        // if (box.Class == target_class_name)
        // {
        //     yolo_d435i_box_info_.Class = box.Class;
        //     yolo_d435i_box_info_.cameraXCenter = (box.xmax + box.xmin) * 0.5f;
        //     yolo_d435i_box_info_.cameraYCenter = (box.ymax + box.ymin) * 0.5f;
        //     yolo_d435i_box_info_.boxHeight = box.ymax - box.ymin;

        //     last_yolo_d435i_time_ = ros::Time::now();

        //     // ROS_INFO_THROTTLE(1, "D435i 锁定目标: %s (%.2f, %.2f)",
        //     //                   yolo_d435i_box_info_.Class.c_str(),
        //     //                   yolo_d435i_box_info_.cameraXCenter,
        //     //                   yolo_d435i_box_info_.cameraYCenter);

        //     found_target = true;
        //     break;
        // }
    }
    if (!found_target)
    {
        ROS_WARN_THROTTLE(1, "D435i 未在当前帧中找到目标%s", target_class_name.c_str());
    }
}
// ar标签回调
void ASNAV::ar_pose_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int count = msg->markers.size();
    if (count != 0)
    {
        // 不匹配特定ID，取第一个检测到的AR码直接使用
        ar_track_alvar_msgs::AlvarMarker marker = msg->markers[0];
        ar_marker_found_ = true;
        // 根据摄像头安装方向进行静态坐标转换
        // 摄像头朝下安装：摄像头X对应无人机Y(左右)，摄像头Y对应无人机X(前后)，摄像头Z对应上下
        ar_position_detec_x_ = marker.pose.pose.position.x;
        ar_position_detec_y_ = marker.pose.pose.position.y;
        ar_position_detec_z_ = marker.pose.pose.position.z;
        ROS_INFO_THROTTLE(1.0, "[AR回调] 检测到AR码 id=%d(忽略ID匹配) | pos=(%.3f, %.3f, %.3f)",
                          marker.id, ar_position_detec_x_, ar_position_detec_y_, ar_position_detec_z_);

        // // ---- 以下为按ID匹配的原始逻辑，如需恢复取消注释 ----
        // bool found = false;
        // for (int i = 0; i < count; i++)
        // {
        //     ar_track_alvar_msgs::AlvarMarker marker = msg->markers[i];
        //     if (marker.id == ar_target_id_)
        //     {
        //         ar_marker_found_ = true;
        //         found = true;
        //         ar_position_detec_x_ = marker.pose.pose.position.x;
        //         ar_position_detec_y_ = marker.pose.pose.position.y;
        //         ar_position_detec_z_ = marker.pose.pose.position.z;
        //         ROS_INFO_THROTTLE(1.0, "[AR回调] 锁定目标AR码 id=%d | pos=(%.3f, %.3f, %.3f)",
        //                           marker.id, ar_position_detec_x_, ar_position_detec_y_, ar_position_detec_z_);
        //         return;
        //     }
        // }
        // ar_marker_found_ = false;
        // if (!found)
        // {
        //     ROS_WARN_THROTTLE(2.0, "[AR回调] 收到%d个AR码，但未找到目标id=%d", count, ar_target_id_);
        // }
        // // ---- 原始逻辑结束 ----
    }
    else
    {
        ar_marker_found_ = false;
        ROS_WARN_THROTTLE(3.0, "[AR回调] /ar_pose_marker 话题为空，未检测到任何AR码");
    }
}
