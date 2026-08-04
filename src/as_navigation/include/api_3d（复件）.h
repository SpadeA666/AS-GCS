#ifndef ASNAV_API_H
#define ASNAV_API_H

#include "mavros_msgs/CommandBool.h"
#include <mavros_msgs/CommandLong.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <yolov11_ros_msgs/BoundingBoxes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include <actionlib/client/simple_action_client.h> 

#include "lib_library.h"


class ASNAV
{
    public:
    explicit ASNAV(ros::NodeHandle& nh_);
    ~ASNAV();
    bool takeoff(float height);
    bool position(float x, float y, float z, float yaw, float tol = 0.2f);
    bool positionSmooth(float target_x, float target_y, float target_z, float tol, float hover_sec = 0.0f);
    bool navigation(float x, float y, float z, float yaw, float tol = 0.2f, float hover_sec = 1.0f);
    bool navigationWithPosition(float x, float y, float z, float yaw, float tol, float hover_sec);
    bool navigationZpro(float x, float y, float z, float yaw, float tol = 0.2f);
    bool navigationZplus(float x, float y, float z, float yaw, float tol = 0.2f);
    bool navigationSuper(float x, float y, float z, float yaw, float tol = 0.2f);
    bool followEgo();
    bool controlYaw(float x, float y, float z, float target_yaw, float wait_sec);
    bool flyDown(float descend_z);
    bool flyUp(float height);
    void setpointPublish();
    void set_mode(string mode);
    bool autoLand();
    bool trackYoloDown(float max_distance = 0.35f, int tol = 30);
    bool trackYoloForward(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size);
    bool trackYoloing(float Kp_x, float Kp_y, float Kp_z, float target_box_height, int tol_xy, int tol_size);
    void reset_target();
    bool interceptBalloon( float charge_speed, float Kp_y, float Kp_z, float pop_box_height);
    bool escapeBackward(float distance, float tol);
    bool attackBalloon(float charge_speed);
    bool dropBallon(int pwm_5 = 100, int pwm_6 = 100);
    bool putShoot(float x, float y, float z, float yaw, float tol);
    bool putShootSimple(float x, float y, float z, float yaw, float tol);
    bool arTrackLanding(float ground_z = 0.0f, float altitude = 1.0f, float max_error = 0.20f, float vel_set = 0.15f, float camera_offset_x = 0.0f, float camera_offset_y = 0.0f);

    struct yoloBox
    {
        string Class;
        float cameraXCenter, cameraYCenter;
        float boxHeight;
    };

    float fly_height;            
    float descend_z;
    
    private:
    //容差函数
    float tolerance(float x, float y, float z) const;
    //mavros回调
    void mavros_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void mavros_local_position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mavros_local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    //ego回调
    void ego_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    //super回调
    void super_planner_pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    //yolo回调
    void yolo_info_cb(const yolov11_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void yolo_d435i_info_cb(const yolov11_ros_msgs::BoundingBoxes::ConstPtr& msg);  // D435i前视YOLO回调
    //ar标签回调
    void ar_pose_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    ros::NodeHandle nh_;
    //mavros相关组件
    ros::Subscriber mavros_state_sub_, mavros_local_position_pose_sub_, mavros_local_velocity_sub_;
    ros::Publisher mavros_setpoint_raw_local_pub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient mavros_cmd_command_client_;
    //ego相关组件
    ros::Subscriber ego_planner_pos_cmd_sub_;
    ros::Publisher goal_pub_;
    //super相关组件
    ros::Subscriber super_planner_pos_cmd_sub_;
    ros::Publisher super_goal_pub_;
    //yolo相关组件
    ros::Subscriber yolo_info_sub_;
    ros::Subscriber yolo_d435i_info_sub_;  // D435i前视相机YOLO订阅
    //ar标签相关组件
    ros::Subscriber ar_pose_sub_;
    //mavros变量
    bool is_offboard, is_auto_land, is_yaw_finished = false;
    geometry_msgs::Point current_position;
    geometry_msgs::Vector3 current_velocity;
    mavros_msgs::PositionTarget target_position;
    mavros_msgs::State current_state;
    float current_yaw;
    ros::Time yaw_finish_time;

    // mavros_msgs::CommandLong lib_ctrl_pwm;

    // EGO 规划器指令缓存
    quadrotor_msgs::PositionCommand ego_cmd_;
    bool ego_cmd_received_ = false;
    ros::Time ego_stale_time_;   // ego指令最后活跃时间（用于检测ego停摆）
    bool goal_sent_;

    // SUPER 规划器指令缓存
    quadrotor_msgs::PositionCommand super_cmd_;
    bool super_cmd_received_ = false;
    ros::Time last_super_msg_time_;
    bool super_goal_sent_;

    //其他变量
    double start_planning_time, finish_planning_time;

    yoloBox yolo_box_info;            // 单目(下视)YOLO结果
    yoloBox yolo_d435i_box_info_;     // D435i(前视)YOLO结果
    std::string target_class_name;
    float integral_error_x, integral_error_y, last_err_x, last_err_y;
    ros::Time last_yolo_time_;
    ros::Time last_yolo_d435i_time_;  // D435i最后识别时间
    //ar标签跟踪变量
    bool ar_marker_found_;
    int ar_target_id_;
    float ar_position_detec_x_, ar_position_detec_y_, ar_position_detec_z_;

    // navigationZpro 速度环PI修正参数（从launch加载）
    float zpro_kp_;              // 位置误差 → 速度修正 P 增益
    float zpro_ki_;              // 位置误差积分 → 速度修正 I 增益
    float zpro_max_v_;           // 速度指令限幅 (m/s)
    float zpro_integral_clamp_;  // 积分抗饱和上限
    float zpro_err_thresh_;      // 积分生效的误差阈值 (m)
    float zpro_brake_dist_;      // 刹车距离 (m)：距终点此距离内开始比例减速
    float zpro_brake_min_v_;     // 刹车区最低速度 (m/s)：避免完全停住不动
    float zpro_accel_ff_gain_;   // 加速度前馈增益 (0=关, 0.3~0.6=弯道补偿, 1=全量)
    float integral_err_zx_;      // X 方向位置误差积分
    float integral_err_zy_;      // Y 方向位置误差积分

    // navigationZplus 参数（从launch加载）
    float zplus_kp_outer_;       // Kp: 位置误差 → 速度修正
    float zplus_kv_outer_;       // Kv: 速度误差阻尼 (v_ref→v_actual)
    float zplus_ki_outer_;       // Ki: 位置误差积分
    float zplus_max_vel_;        // 速度指令限幅 (m/s)
    float zplus_max_integral_;   // 积分抗饱和上限
    float zplus_traj_timeout_;   // 轨迹超时时间 (s)

    // positionSmooth 步长控制参数（从launch加载）
    double smooth_step_xy_;        // 正常步长 (m/cycle)，默认 0.18
    double smooth_slow_step_xy_;   // 接近目标时的减速步长 (m/cycle)，默认 0.01
    double smooth_slow_dist_;      // 触发减速的距离阈值 (m)，默认 0.5

    // navigationZplus 运行时状态
    double integral_zpx_;         // X 位置误差积分
    double integral_zpy_;         // Y 位置误差积分
    ros::Time last_zplus_call_time_; // 上一帧调用时间 (dt计算)
    float zplus_traj_elapsed_;    // 新轨迹软启动计时器
    ros::Time last_ego_msg_time_; // 最近一次收到ego消息的时间戳

    // navigationZplus Debounce 防穿透 + 位置保持
    ros::Time zplus_tol_entry_time_;  // 进入容差时刻
    bool zplus_tol_timing_;           // 是否正在 debounce 计时
    bool zplus_holding_;              // debounce 完成后是否在位置保持阶段
    ros::Time zplus_hold_start_time_; // 位置保持开始时刻
    float zplus_hold_x_ = 0.0f;
    float zplus_hold_y_ = 0.0f;

    // navigationSuper 参数（从launch加载）
    float super_kp_outer_;       // Kp: 位置误差 → 速度修正
    float super_kv_outer_;       // Kv: 速度误差阻尼 (v_ref→v_actual)
    float super_ki_outer_;       // Ki: 位置误差积分
    float super_max_vel_;        // 速度指令限幅 (m/s) XY
    float super_max_vel_z_;      // Z 速度指令限幅 (m/s)
    float super_max_integral_;   // 积分抗饱和上限
    float super_traj_timeout_;   // 轨迹超时时间 (s)
    bool super_rotate_180_;      // 坐标系是否需要旋转180°(LIO→PX4)

    // navigationSuper 运行时状态
    double integral_spx_;         // X 位置误差积分
    double integral_spy_;         // Y 位置误差积分
    double integral_spz_;         // Z 位置误差积分
    ros::Time last_super_call_time_; // 上一帧调用时间 (dt计算)
    float super_traj_elapsed_;    // 新轨迹软启动计时器

    // navigationSuper Debounce
    ros::Time super_tol_entry_time_;  // 进入容差时刻
    bool super_tol_timing_;           // 是否正在 debounce 计时
};
#endif