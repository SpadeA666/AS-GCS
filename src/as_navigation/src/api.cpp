#include "api.h"

ASNAV::ASNAV(ros::NodeHandle& nh) : nh_(nh)
{
    // 从参数服务器获取参数
    nh_.param<float>("height", height, 1.0);
    nh_.param<float>("descend_z", descend_z, 0.5);
    nh_.param<bool>("is_auto_land", is_auto_land, true);


    // 初始化订阅和发布
    mavros_state_sub_ = nh_.subscribe("/iris_0/mavros/state", 10, &ASNAV::mavros_state_cb, this);
    mavros_local_position_pose_sub_ = nh_.subscribe("/iris_0/mavros/local_position/pose", 10, &ASNAV::mavros_local_position_pose_cb, this);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
    planner_cmd_vel_sub_ = nh_.subscribe("/xtdrone/iris_0/cmd_vel_flu", 10, &ASNAV::planner_cmd_vel_cb, this);
    mavros_setpoint_raw_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ac_ = new MoveBaseClient("move_base", true);
    ROS_INFO("等待 move_base 服务启动...");
    ac_->waitForServer(); 
    ROS_INFO("move_base 已连接");

    is_offboard = false;
    is_auto_land = false;
    current_position = geometry_msgs::Point();
    target_position = mavros_msgs::PositionTarget();
    planner_velxy_posz = geometry_msgs::Point();
    start_planning_time = 0;
    finish_planning_time = 0;
    goal_sent_ = false;
}

ASNAV::~ASNAV()
{
}
// 起飞函数，升到指定高度调用position函数发布目标位置，并持续发布直到飞到目标高度或进入OFFBOARD模式
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

    position(0.0f, 0.0f, height, 0.0f, 0.2f);

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
// 定位函数，发布目标位置并判断是否到达目标位置
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
    target_position.yaw = current_yaw;
    return tolerance(x, y, z) < tol;
}
// 导航函数，发布目标位置给move_base并判断是否到达目标位置
bool ASNAV::navigation(float x, float y, float z, float yaw, float tol)
{
    // 1. 如果这是新航点（或者第一次进入），发送 Goal 给 ActionServer
    if (!goal_sent_) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map"; // 或者是你的 odom/world
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = z;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        ac_->sendGoal(goal);
        goal_sent_ = true;
        ROS_INFO("Action 发送目标: (%.2f, %.2f)", x, y);
    }

    // 2. 执行原本的速度控制逻辑（维持 Offboard 飞行）
    // 只要 move_base 还在算速度 (is_as_received)，我们就把速度喂给飞控
    if (is_as_received)
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
                                    mavros_msgs::PositionTarget::IGNORE_YAW;
        target_position.velocity.x = planner_velxy_posz.x;
        target_position.velocity.y = planner_velxy_posz.y;
        target_position.position.z = z;
        // target_position.yaw = current_yaw; 
        // target_position.yaw_rate = planner_velxy_posz.z;
    }

    // 3. 关键：判断是否到达
    // 我们检查 Actionlib 的状态。SUCCEEDED 表示 move_base 觉得自己到了。
    auto state = ac_->getState();
    
    // 如果 move_base 说到了，或者我们的数学计算也觉得到了
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("导航任务完成！状态: %s, 剩余距离: %.2f", state.toString().c_str(), tolerance(x,y,z));
        goal_sent_ = false;  // 重置标志，给下一个 case 使用
        is_as_received = false; // 停止当前速度指令执行
        return true;         // 返回 true，主状态机 mission_num 才会 ++
    }

    return false; // 还没到，继续循环
}
// 控制航向函数，发布目标位置并判断是否到达目标航向
bool ASNAV::controlYaw(float yaw, float tol)
{
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
    static bool position_locked = false;
    static geometry_msgs::Point locked_pos;
    if (!position_locked) 
    {
    locked_pos = current_position;
    position_locked = true;
    }
    target_position.position = locked_pos; // 始终发同一个坐标
    target_position.yaw = yaw;
    float angle_error = std::fabs(current_yaw - yaw);
    if (angle_error > M_PI) angle_error = 2 * M_PI - angle_error;
    return angle_error < tol;
//     if(std::fabs(current_yaw - yaw) < tol)
//     {
//         return true;
//     }
//     return false;
}
// 飞行下降函数，发布目标位置并判断是否到达目标高度
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
// 飞行上升函数，发布目标位置并判断是否到达目标高度
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
// 自动降落函数，发布目标位置并判断是否到达目标高度
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
// MAVROS状态回调函数对飞控状态进行更新
void ASNAV::mavros_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
// MAVROS位置回调函数对当前位置进行更新
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
    
}
// move_base速度回调函数
void ASNAV::planner_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    planner_velxy_posz.x = msg->linear.x;
    planner_velxy_posz.y = msg->linear.y;
    planner_velxy_posz.z = msg->angular.z;
    is_as_received = true;
} 
// 容差函数
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
  
