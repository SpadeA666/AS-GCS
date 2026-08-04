#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_ned_to_enu.py
把 PX4/MAVROS 的 NED 系 IMU (/iris_0/mavros/imu/data) 转成 ENU 系, 发布给 FAST-LIO。
解决 FAST-LIO 因 IMU 重力方向反导致 Z 发散(-15万米)的问题。

坐标系变换 NED(x北,y东,z下) -> ENU(x东,y北,z上):
    向量 v_enu = (v_ned.y, v_ned.x, -v_ned.z)
    四元数 q_enu = q_R ⊗ q_ned, 其中 q_R 对应旋转矩阵 R=[[0,1,0],[1,0,0],[0,0,-1]]
                  即绕轴 (1,1,0)/√2 转 π, q_R=(w=0, x=1/√2, y=1/√2, z=0)
"""
import rospy
from sensor_msgs.msg import Imu
import math

SRT2 = 1.0 / math.sqrt(2.0)

# q_R = (w, x, y, z), 绕 (1,1,0)/√2 转 π
QR_W, QR_X, QR_Y, QR_Z = 0.0, SRT2, SRT2, 0.0


def quat_mul(q1, q2):
    """Hamilton 四元数乘法 q1 ⊗ q2, 输入 (w,x,y,z)"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


class ImuNedToEnu:
    def __init__(self):
        sub_topic = rospy.get_param("~input_topic", "/iris_0/mavros/imu/data")
        pub_topic = rospy.get_param("~output_topic", "/imu_enu")
        self.pub = rospy.Publisher(pub_topic, Imu, queue_size=10)
        rospy.Subscriber(sub_topic, Imu, self.cb, queue_size=10)
        rospy.loginfo("[imu_ned_to_enu] %s -> %s", sub_topic, pub_topic)

    def cb(self, msg):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id  # base_link

        # 角速度: NED -> ENU
        wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        out.angular_velocity.x = wy
        out.angular_velocity.y = wx
        out.angular_velocity.z = -wz
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        # 线性加速度: NED -> ENU
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        out.linear_acceleration.x = ay
        out.linear_acceleration.y = ax
        out.linear_acceleration.z = -az
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # 姿态四元数: q_enu = q_R ⊗ q_ned
        qn = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        if qn[0] != 0.0 or qn[1] != 0.0 or qn[2] != 0.0 or qn[3] != 0.0:
            qe = quat_mul((QR_W, QR_X, QR_Y, QR_Z), qn)
            out.orientation.w, out.orientation.x, out.orientation.y, out.orientation.z = qe
        else:
            out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("imu_ned_to_enu")
    ImuNedToEnu()
    rospy.spin()
