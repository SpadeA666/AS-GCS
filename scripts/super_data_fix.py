#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
super_data_fix.py — /super_odom + /super_cloud 数据源
两种模式(rosparam ~use_ground_truth, 默认 false):

[LIO模式] use_ground_truth:=false (实机/原逻辑)
  订阅 /Odometry_highrate(LIO) + /iris_0/mavros/local_position/odom(MAVROS)
  输出: /super_odom = LIO XY/yaw + MAVROS z
        /super_cloud = /cloud_registered 点云 z 按点云帧时间戳对齐平移 delta=mav_z-lio_z
  时间戳对齐: 缓存 (lio_odom_stamp, lio_z, mav_z), 点云帧按自身 stamp 取 delta。

[GT模式] use_ground_truth:=true (仿真专用, 绕过 FAST-LIO z 退化)
  订阅 /livox/lidar2(原始机体系点云) + /iris_0/mavros/local_position/odom(MAVROS ENU位姿)
  输出: /super_odom = MAVROS odom 直接转发(PX4 EKF, z可靠)
        /super_cloud = 点云经 [雷达→机体外参] + [MAVROS位姿] 变换到 map 系
  仿真里 MAVROS 位姿+Gazebo点云都是真值 → 地图 z 稳定, 不再受 LIO z 漂移影响。
"""
import rospy
from collections import deque
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# /livox/lidar2 是 livox_ros_driver2 的 CustomMsg(非PointCloud2), GT模式需解析
try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None

# 雷达→机体外参 (Mid360 include pose (0,0,0.05) + 激光sensor模型内(-0.005,0.005,0.047))
# 2026-09-11: Mid360 倾斜 15° 安装 → 雷达系→机体旋转 = R_y(+15°)，不再用单位阵
LIDAR_T_BODY = np.array([0.005, 0.005, 0.097])
LIDAR_R_BODY = np.array([
    [0.9659258, 0.0, 0.2588190],
    [0.0,       1.0, 0.0],
    [-0.2588190, 0.0, 0.9659258],
])
# 点云降采样(每N取1), 控延迟: 24000点→6000点
GT_DOWNSAMPLE = 4


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SuperDataFix:
    def __init__(self):
        self.use_gt = rospy.get_param("~use_ground_truth", False)
        lio_odom_topic = rospy.get_param("~lio_odom", "/Odometry_highrate")
        mav_odom_topic = rospy.get_param("~mav_odom", "/iris_0/mavros/local_position/odom")
        cloud_topic = rospy.get_param("~cloud_in", "/cloud_registered")
        gt_cloud_topic = rospy.get_param("~gt_cloud", "/livox/lidar2")

        self.odom_pub = rospy.Publisher("/super_odom", Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher("/super_cloud", PointCloud2, queue_size=5)

        if self.use_gt:
            # 混合模式: LIO XY + MAVROS z/姿态 + 原始点云(仿真专用, 绕开LIO z退化与MAVROS XY漂移)
            self.last_lio = None
            self.last_mav = None
            if CustomMsg is None:
                raise RuntimeError("GT模式需要 livox_ros_driver2/CustomMsg, 请 source catkin_ws 环境")
            rospy.Subscriber(lio_odom_topic, Odometry, self.gt_lio_cb, queue_size=10)
            rospy.Subscriber(mav_odom_topic, Odometry, self.gt_mav_cb, queue_size=10)
            rospy.Subscriber(gt_cloud_topic, CustomMsg, self.gt_cloud_cb, queue_size=3)
            rospy.loginfo("[super_data_fix] GT混合模式: LIO(XY) + MAVROS(z/姿态) + %s -> /super_odom,/super_cloud",
                          gt_cloud_topic)
        else:
            self.lio_z = 0.0
            self.mav_z = 0.0
            self.have_mav = False
            self.have_lio = False
            self.odom_history = deque(maxlen=200)  # (stamp, lio_z, mav_z)
            rospy.Subscriber(lio_odom_topic, Odometry, self.lio_cb, queue_size=10)
            rospy.Subscriber(mav_odom_topic, Odometry, self.mav_cb, queue_size=10)
            rospy.Subscriber(cloud_topic, PointCloud2, self.cloud_cb, queue_size=10)
            rospy.loginfo("[super_data_fix] LIO模式: %s + %s -> /super_odom, %s -> /super_cloud",
                          lio_odom_topic, mav_odom_topic, cloud_topic)

    # ================= GT 混合模式 =================
    def _mixed_odom(self):
        """LIO XY + MAVROS z/姿态 → /super_odom (frame=odom)"""
        if self.last_lio is None or self.last_mav is None:
            return None
        lio = self.last_lio.pose.pose
        mav = self.last_mav.pose.pose
        out = Odometry()
        out.header = self.last_mav.header
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x = lio.position.x
        out.pose.pose.position.y = lio.position.y
        out.pose.pose.position.z = mav.position.z
        out.pose.pose.orientation = mav.orientation  # 姿态用 MAVROS(PX4 IMU, 准)
        return out

    def gt_lio_cb(self, msg):
        self.last_lio = msg
        out = self._mixed_odom()
        if out is not None:
            self.odom_pub.publish(out)

    def gt_mav_cb(self, msg):
        self.last_mav = msg
        out = self._mixed_odom()
        if out is not None:
            self.odom_pub.publish(out)

    def gt_cloud_cb(self, msg):
        if self.last_lio is None or self.last_mav is None:
            return
        lio = self.last_lio.pose.pose
        mav = self.last_mav.pose.pose
        R = quat_to_rot(mav.orientation)          # 姿态用 MAVROS
        t = np.array([lio.position.x, lio.position.y, mav.position.z])  # XY用LIO, z用MAVROS

        # CustomMsg: CustomPoint[] points, 每个点含 x/y/z/reflectivity/tag/line (livox_base系)
        n = len(msg.points)
        if n == 0:
            return
        pts = msg.points[::GT_DOWNSAMPLE]
        xyz = np.array([(p.x, p.y, p.z) for p in pts], dtype=np.float64)

        # 机体系 -> 机体: p_body = R_lidar_body * p_lidar + t_lidar_body
        p_body = xyz @ LIDAR_R_BODY.T + LIDAR_T_BODY
        # 机体 -> map(odom): p_map = R_odom * p_body + t_odom
        p_map = p_body @ R.T + t

        out = pc2.create_cloud_xyz32(msg.header, [tuple(r) for r in p_map])
        out.header.frame_id = "odom"
        self.cloud_pub.publish(out)

    # ================= LIO 模式 =================
    def lio_cb(self, msg):
        self.lio_z = msg.pose.pose.position.z
        self.have_lio = True
        if not self.have_mav:
            return
        self.odom_history.append((msg.header.stamp, self.lio_z, self.mav_z))
        out = Odometry()
        out = msg
        out.pose.pose.position.z = self.mav_z
        self.odom_pub.publish(out)

    def mav_cb(self, msg):
        self.mav_z = msg.pose.pose.position.z
        self.have_mav = True

    def _get_delta_for(self, cloud_stamp):
        if not self.odom_history:
            return None
        best = None
        best_dt = float('inf')
        for item in self.odom_history:
            dt = abs((item[0] - cloud_stamp).to_sec())
            if dt < best_dt:
                best_dt = dt
                best = item
        if best is None or best_dt > 1.0:
            return None
        return best[2] - best[1]

    def cloud_cb(self, msg):
        if not (self.have_mav and self.have_lio):
            return
        delta = self._get_delta_for(msg.header.stamp)
        if delta is None:
            return
        fields = msg.fields
        z_off = None
        for i, f in enumerate(fields):
            if f.name == 'z':
                z_off = i
                break
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
