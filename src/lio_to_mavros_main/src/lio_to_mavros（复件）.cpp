#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>
#include <queue>
 
Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;

// 【修改1】声明全局时间戳变量，否则编译报错
ros::Time lidar_timestamp; 

class SlidingWindowAverage {
public:
    SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}

    double addData(double newData) {
        if(!dataQueue.empty()&&fabs(newData-dataQueue.back())>0.01){
            dataQueue = std::queue<double>();
            windowSum = 0.0;
            dataQueue.push(newData);
            windowSum += newData;
        }
        else{            
            dataQueue.push(newData);
            windowSum += newData;
        }

        if (dataQueue.size() > windowSize) {
            windowSum -= dataQueue.front();
            dataQueue.pop();
        }
        windowAvg = windowSum / dataQueue.size();
        return windowAvg;
    }

    int get_size(){
        return dataQueue.size();
    }

    double get_avg(){
        return windowAvg;
    }

private:
    int windowSize;
    double windowSum;
    double windowAvg;
    std::queue<double> dataQueue;
};

int windowSize = 8;
SlidingWindowAverage swa=SlidingWindowAverage(windowSize);

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

void lio_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    lidar_timestamp = msg->header.stamp;
}
 
void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    swa.addData(fromQuaternion2yaw(q_px4_odom));
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_to_mavros");
    ros::NodeHandle nh("~");

    std::string odom_topic;

    nh.param<std::string>("odom_topic", odom_topic, "/Odometry_highrate");
    float lidar_yaw_offset;
    nh.param<float>("lidar_yaw_offset", lidar_yaw_offset, 0.0f);

    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 100, lio_callback);
    ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/iris_0/mavros/odometry/in", 5, px4_odom_callback);

    // vision_pose 发布 (frame_id="odom"，与实机一致)
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/iris_0/mavros/vision_pose/pose", 10);

    ros::Rate rate(50.0);

    bool init_flag = 0;
    Eigen::Quaterniond init_q;
    Eigen::Quaterniond q_lidar_to_body = Eigen::Quaterniond::Identity();

    while(ros::ok()){
        // 初始化逻辑
        
        // 原始代码：
// if(swa.get_size()==windowSize && !init_flag){
//     init_yaw = swa.get_avg();  <-- 这里会计算初始偏航角，导致坐标系旋转
//     init_flag = 1;
//     init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ());
//     ROS_INFO("Initialization Complete. Yaw Offset: %f rad", init_yaw);
// }

        if(swa.get_size()==windowSize && !init_flag){
            // lidar_yaw_offset: 雷达→机体的偏航角，实机=0 / 仿真=π(反装180°)
            init_flag = 1;
            init_q = Eigen::AngleAxisd(lidar_yaw_offset, Eigen::Vector3d::UnitZ());
            q_lidar_to_body = Eigen::Quaterniond(Eigen::AngleAxisd(lidar_yaw_offset, Eigen::Vector3d::UnitZ()));
            ROS_INFO("Init done. lidar_yaw_offset=%.2f rad (%.0f deg)", lidar_yaw_offset, lidar_yaw_offset*180/M_PI);
        }

        if(init_flag){
            geometry_msgs::PoseStamped vision;

            // 1. 位置旋转: PX4 local = Rz(π)*LIO_odom，补偿雷达反装
            p_enu = init_q * p_lidar_body;
            vision.pose.position.x = p_enu[0];
            vision.pose.position.y = p_enu[1];
            vision.pose.position.z = p_enu[2];

            // 2. 姿态: 雷达→机体转换
            Eigen::Quaterniond q_enu = init_q * (q_mav * q_lidar_to_body);

            vision.pose.orientation.x = q_enu.x();
            vision.pose.orientation.y = q_enu.y();
            vision.pose.orientation.z = q_enu.z();
            vision.pose.orientation.w = q_enu.w();

            // 3. frame_id 保持 "odom"，与实机一致
            vision.header.stamp = lidar_timestamp;
            vision.header.frame_id = "odom";

            vision_pub.publish(vision);
        }
 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}