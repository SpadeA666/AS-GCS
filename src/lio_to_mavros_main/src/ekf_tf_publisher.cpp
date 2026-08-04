// ekf_tf_publisher.cpp
// 用 EKF(气压计高度,准确)位姿发布 odom->base_link TF。
// 原因:仿真里 FAST-LIO 的 Z 会被平面地面锁死/漂移,base_link TF 显示高度错误;
//      而 EKF 用气压计,高度准确(local_position/pose 正常)。用它发 TF,显示就对了。
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z));

    tf::quaternionMsgToTF(msg->pose.orientation, q);
    transform.setRotation(q);

    // MAVROS local_position/pose 就在 "map" 系,直接发 map->base_link。
    // (注: 原来发 odom->base_link,会与 rog_map 发的 map->base_link 抢 base_link 父帧导致 TF 跳变)
    br.sendTransform(tf::StampedTransform(
        transform,
        msg->header.stamp,
        "map",
        "base_link"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_tf_publisher");
    ros::NodeHandle nh("~");

    std::string pose_topic;
    nh.param<std::string>("pose_topic", pose_topic, "/iris_0/mavros/local_position/pose");

    ros::Subscriber sub = nh.subscribe(pose_topic, 10, pose_cb);
    ros::spin();
    return 0;
}
