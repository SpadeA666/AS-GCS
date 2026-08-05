#ifndef ASNAV_API_H
#define ASNAV_API_H

#include "mavros_msgs/CommandBool.h"
#include <mavros_msgs/CommandLong.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"
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
    // 姿态推力控制接口（2026-08 新增）：px4ctrl 风格姿态+总推力 / SUPER OMMPC 风格角速度+推力
    bool navigationSuperAtt(float x, float y, float z, float yaw, float tol = 0.2f);
    bool navigationSuperFF(float x, float y, float z, float yaw, float tol = 0.2f);
    bool navigationZplusAtt(float x, float y, float z, float yaw, float tol = 0.2f);
    bool followEgo();
    // rviz 打点测试接口（照搬 cxr_egoctrl_v1.cpp）：内部阻塞循环，只认 rviz 点触发的规划器轨迹，永不返回 true
    bool navigationEgoRviz(float hover_z = 1.0f);
    bool navigationSuperRviz(float hover_z = 1.0f);
    // 只接收 rviz 打点的 SUPER 跟踪接口：复用 navigationSuper 控制律，不传坐标、不自己发 goal，永不返回 true
    bool navigationSuperRvizPID();
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
    //速度指令帧间斜率限制（消除换航点/新轨迹切入时的速度阶跃）
    void slewLimitVel(double& vx, double& vy, double& vz,
                      double& lx, double& ly, double& lz, double dt, float max_acc);
    // px4ctrl 风格核心：期望 PVA + yaw → 期望姿态 + 归一化推力
    // （computePIDErrorAcc → computeLimitedTotalAcc → SO(3) 姿态 → 推力归一化）
    bool computeAttThrust(const tf::Vector3& des_p, const tf::Vector3& des_v,
                          const tf::Vector3& des_a, double des_yaw,
                          tf::Quaternion& q_des, double& thrust);
    // px4ctrl computeFeedBackControlBodyrates：姿态误差 → 机体系反馈角速度
    tf::Vector3 attErrorBodyrates(const tf::Quaternion& q_des) const;
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
    bool ego_rotate_180_;        // ego 轨迹是否旋转180°(LIO→PX4)。默认 false：lio_to_mavros 桥 lidar_yaw_offset=0 不旋转，LIO系=PX4 local

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
    float zplus_max_accel_;      // 速度指令帧间加速度限幅 (m/s^2)，0=关闭

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
    ros::Time zplus_goal_time_;   // 最近一次发布 ego 目标的时刻（发新目标时刷新超时计时）
    double last_zplus_vx_;        // 上一帧速度指令（斜率限制用）
    double last_zplus_vy_;
    uint32_t zplus_last_traj_id_; // ego trajectory_id 跟踪（新轨迹检测→清积分）

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
    float super_max_accel_;      // 速度指令帧间加速度限幅 (m/s^2)，0=关闭

    // navigationSuper 运行时状态
    double integral_spx_;         // X 位置误差积分
    double integral_spy_;         // Y 位置误差积分
    double integral_spz_;         // Z 位置误差积分
    ros::Time last_super_call_time_; // 上一帧调用时间 (dt计算)
    float super_traj_elapsed_;    // 新轨迹软启动计时器
    bool super_rviz_mode_;        // navigationSuper 的 rviz 测试模式：不发 goal、不判到达（由 navigationSuperRvizPID 置位）

    // navigationSuper Debounce
    ros::Time super_tol_entry_time_;  // 进入容差时刻
    bool super_tol_timing_;           // 是否正在 debounce 计时

    // 航点切换平顺化（2026-08 修复）
    ros::Time super_goal_time_;       // 最近一次发布 SUPER 目标的时刻（刷新超时计时 + 新鲜轨迹判定）
    double last_super_vx_;            // 上一帧速度指令（斜率限制用）
    float super_slew_timer_ = 0.0f;   // >0 时启用 slewLimit（新目标/换点瞬间，正常跟踪旁路）
    bool super_pos_hold_ = false;     // SUPER 到点位置保持（仿 ruikang HOVER：收敛交给 PX4）
    int super_pos_hold_frames_ = 0;   // 到点判定防抖帧计数
    bool ego_pos_hold_ = false;       // EGO 到点位置保持
    int ego_pos_hold_frames_ = 0;     // 到点判定防抖帧计数
    double last_super_vy_;
    double last_super_vz_;

    // ====== 姿态推力控制接口（px4ctrl 风格 / SUPER OMMPC 风格，2026-08 新增）======
    ros::Publisher mavros_setpoint_raw_attitude_pub_;
    mavros_msgs::AttitudeTarget target_attitude_;
    ros::Time attitude_active_time_;   // 最近一次姿态类接口更新时间（setpointPublish 通道切换依据）
    tf::Quaternion current_attitude_;  // 当前姿态四元数（mavros ENU）
    bool att_feedback_valid_;          // 已收到姿态反馈
    // px4ctrl 风格控制器参数（从launch加载）
    float att_kp_;               // 级联位置环 P（px4ctrl gain.Kp*，默认 2.5）
    float att_kv_;               // 级联速度环 P（px4ctrl gain.Kv*，默认 3.0）
    float att_hover_percentage_; // 悬停油门比例（thr2acc = g/hover_percentage），实机需标定
    float att_max_angle_;        // 倾角限制 (deg)
    float att_kang_r_;           // 姿态误差→反馈角速度增益（px4ctrl KAng，默认 20/20/4）
    float att_kang_p_;
    float att_kang_y_;
    bool  att_use_planner_yaw_;  // yaw=NAN 时是否跟随规划器 yaw 前馈（false=保持当前 yaw）
    // 悬停冻结（对应 px4ctrl AUTO_HOVER：断指令时冻结当前点为期望）
    bool att_hover_valid_;
    double att_hover_x_, att_hover_y_, att_hover_z_, att_hover_yaw_;
    // navigationSuperAtt / navigationSuperFF / navigationZplusAtt 到达 debounce
    ros::Time satt_tol_entry_time_;
    bool satt_tol_timing_;
    ros::Time zatt_tol_entry_time_;
    bool zatt_tol_timing_;
};
#endif