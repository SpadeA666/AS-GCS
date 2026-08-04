#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
super_data_fix.py
修复 FAST-LIO 在仿真里 z 退化的问题:
  - LIO odom 的 XY/yaw 正确, 但 z 偏差 ~1m 且漂移
  - MAVROS/PX4 的 z 稳定可靠 (EKF+气压计)
  - LIO 和 MAVROS 的 XY/yaw 实测一致

输出:
  /super_odom   : LIO 的 XY/yaw + MAVROS 的 z
  /super_cloud  : LIO cloud 的 z 实时平移到 MAVROS 基准 (z += MAVROS_z - LIO_z)
"""
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct


class SuperDataFix:
    def __init__(self):
        lio_odom_topic = rospy.get_param("~lio_odom", "/Odometry_highrate")
        mav_odom_topic = rospy.get_param("~mav_odom", "/iris_0/mavros/local_position/odom")
        cloud_topic = rospy.get_param("~cloud_in", "/cloud_registered")

        self.lio_z = 0.0
        self.mav_z = 0.0
        self.have_mav = False
        self.have_lio = False

        self.odom_pub = rospy.Publisher("/super_odom", Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher("/super_cloud", PointCloud2, queue_size=10)

        rospy.Subscriber(lio_odom_topic, Odometry, self.lio_cb, queue_size=10)
        rospy.Subscriber(mav_odom_topic, Odometry, self.mav_cb, queue_size=10)
        rospy.Subscriber(cloud_topic, PointCloud2, self.cloud_cb, queue_size=10)
        rospy.loginfo("[super_data_fix] 已启动: %s + %s -> /super_odom, %s -> /super_cloud",
                      lio_odom_topic, mav_odom_topic, cloud_topic)

    def lio_cb(self, msg):
        self.lio_z = msg.pose.pose.position.z
        self.have_lio = True
        if not self.have_mav:
            return
        # 修正 odom: XY/yaw 用 LIO, z 用 MAVROS
        out = Odometry()
        out = msg
        out.pose.pose.position.z = self.mav_z
        self.odom_pub.publish(out)

    def mav_cb(self, msg):
        self.mav_z = msg.pose.pose.position.z
        self.have_mav = True

    def cloud_cb(self, msg):
        if not (self.have_mav and self.have_lio):
            return
        delta = self.mav_z - self.lio_z
        # 平移 z: 把当前帧 cloud 从 LIO 位姿对齐到 MAVROS 位姿
        fields = msg.fields
        x_off = None
        y_off = None
        z_off = None
        for i, f in enumerate(fields):
            if f.name == 'x': x_off = i
            elif f.name == 'y': y_off = i
            elif f.name == 'z': z_off = i
        if z_off is None:
            return

        pts = pc2.read_points(msg, field_names=None, skip_nans=True)
        new_pts = []
        for p in pts:
            p2 = list(p)
            p2[z_off] += delta
            new_pts.append(tuple(p2))

        out = pc2.create_cloud(msg.header, fields, new_pts)
        out.header.frame_id = msg.header.frame_id
        self.cloud_pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("super_data_fix")
    SuperDataFix()
    rospy.spin()
