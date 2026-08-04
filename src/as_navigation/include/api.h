#ifndef ASNAV_API_H
#define ASNAV_API_H

#include "mavros_msgs/CommandBool.h"
#include <mavros_msgs/CommandLong.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
// #include <yolov8_ros_msgs/BoundingBoxes.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h> 

#include "lib_library.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ASNAV
{
    public:
    explicit ASNAV(ros::NodeHandle& nh_);
    ~ASNAV();
    bool takeoff(float height);
    bool position(float x, float y, float z, float yaw, float tol = 0.2f);
    bool navigation(float x, float y, float z, float yaw, float tol = 0.2f);
    bool controlYaw(float yaw, float tol = 0.1f);
    bool flyDown(float descend_z);
    bool flyUp(float height);
    void setpointPublish();
    void set_mode(string mode);
    bool autoLand();

    private:
    //容差函数
    float tolerance(float x, float y, float z) const;
    //mavros回调
    void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void mavros_local_position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    //move_base回调
    void planner_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
    ros::NodeHandle nh_;
    //mavros相关组件
    ros::Subscriber mavros_state_sub_, mavros_local_position_pose_sub_;
    ros::Publisher mavros_setpoint_raw_local_pub_;
    ros::ServiceClient set_mode_client_;
    //move_base相关组件
    ros::Subscriber planner_cmd_vel_sub_;  
    ros::Publisher goal_pub_;
    //mavros变量
    bool is_offboard, is_auto_land;
    geometry_msgs::Point current_position;
    mavros_msgs::PositionTarget target_position;
    mavros_msgs::State current_state;
    float current_yaw;

    //move_base变量
    geometry_msgs::Point planner_velxy_posz;
    MoveBaseClient* ac_;
    bool goal_sent_;
    //其他变量
    float height, descend_z;
    double start_planning_time, finish_planning_time;
    bool is_as_received = false;
};
#endif