#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_nav_debug.py — 导航控制调试数据记录
订阅 SUPER/EGO 参考指令 + 实际状态 + 下发指令 + 目标点，同步采样落 CSV。
用法:
  1. 先启动规划器/控制器（无人机飞行中）
  2. 运行: python3 record_nav_debug.py [输出文件.csv]
  3. 执行一次"接近目标"的飞行
  4. Ctrl+C 停止，把 CSV 路径给分析方

默认输出: ~/catkin_ws/logs/nav_debug_<时间戳>.csv
"""
import csv
import os
import sys
import time
import datetime
import signal

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand


class NavDebugRecorder:
    def __init__(self, out_path):
        self.out_path = out_path
        self.f = open(out_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "t",                       # 采样时刻 (s, 相对启动)
            "goal_x", "goal_y", "goal_z",
            "ref_px", "ref_py", "ref_pz",   # 规划器参考位置
            "ref_vx", "ref_vy", "ref_vz",   # 规划器参考速度
            "cur_px", "cur_py", "cur_pz",   # 实际位置
            "cur_vx", "cur_vy", "cur_vz",   # 实际速度
            "cmd_vx", "cmd_vy", "cmd_vz",   # 实际下发速度指令
            "cmd_mode",                     # 下发 type_mask (判断速度/位置模式)
            "dist_goal",                    # 到目标点距离
            "dist_ref",                     # 到参考点距离
            "ref_speed",                    # 参考速度模长
            "cmd_age_ms",                   # 参考指令新鲜度
        ])
        self.t0 = time.time()

        # 缓存
        self.goal = None
        self.ref = None      # (x,y,z,vx,vy,vz)
        self.cur_p = (0.0, 0.0, 0.0)
        self.cur_v = (0.0, 0.0, 0.0)
        self.cmd = None      # (vx,vy,vz,mask)
        self.ref_stamp = rospy.Time(0)

    def _t(self):
        return time.time() - self.t0

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def ref_cb(self, msg):
        self.ref = (msg.position.x, msg.position.y, msg.position.z,
                    msg.velocity.x, msg.velocity.y, msg.velocity.z)
        self.ref_stamp = msg.header.stamp

    def pose_cb(self, msg):
        self.cur_p = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def vel_cb(self, msg):
        self.cur_v = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)

    def cmd_cb(self, msg):
        self.cmd = (msg.velocity.x, msg.velocity.y, msg.velocity.z,
                    int(msg.type_mask))

    def pose_sample_cb(self, msg):
        """以 pose 回调驱动采样（MAVROS local_position/pose 高频）"""
        self.pose_cb(msg)
        if self.ref is None:
            return
        gx, gy, gz = (self.goal if self.goal else (0.0, 0.0, 0.0))
        rx, ry, rz, rvx, rvy, rvz = self.ref
        cx, cy, cz = self.cur_p
        cvx, cvy, cvz = self.cur_v
        if self.cmd:
            c_vx, c_vy, c_vz, c_mask = self.cmd
        else:
            c_vx = c_vy = c_vz = c_mask = 0.0
        age_ms = 999.0
        if not self.ref_stamp.is_zero():
            age_ms = (rospy.Time.now() - self.ref_stamp).to_sec() * 1000.0
        dist_goal = ((cx-gx)**2 + (cy-gy)**2 + (cz-gz)**2) ** 0.5
        dist_ref = ((cx-rx)**2 + (cy-ry)**2 + (cz-rz)**2) ** 0.5
        ref_speed = (rvx**2 + rvy**2 + rvz**2) ** 0.5
        self.w.writerow([f"{self._t():.4f}",
                         f"{gx:.3f}", f"{gy:.3f}", f"{gz:.3f}",
                         f"{rx:.3f}", f"{ry:.3f}", f"{rz:.3f}",
                         f"{rvx:.3f}", f"{rvy:.3f}", f"{rvz:.3f}",
                         f"{cx:.3f}", f"{cy:.3f}", f"{cz:.3f}",
                         f"{cvx:.3f}", f"{cvy:.3f}", f"{cvz:.3f}",
                         f"{c_vx:.3f}", f"{c_vy:.3f}", f"{c_vz:.3f}",
                         c_mask,
                         f"{dist_goal:.3f}", f"{dist_ref:.3f}",
                         f"{ref_speed:.3f}", f"{age_ms:.1f}"])

    def run(self):
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.ref_cb)
        rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, self.pose_sample_cb)
        rospy.Subscriber("/iris_0/mavros/local_position/velocity_local", TwistStamped, self.vel_cb)
        rospy.Subscriber("/iris_0/mavros/setpoint_raw/local", PositionTarget, self.cmd_cb)
        rospy.loginfo("[record] 开始记录 -> %s (Ctrl+C 停止)", self.out_path)
        rospy.spin()

    def stop(self):
        self.f.close()
        n = sum(1 for _ in open(self.out_path)) - 1
        rospy.loginfo("[record] 已停止, %d 行 -> %s", n, self.out_path)


def main():
    rospy.init_node("record_nav_debug", anonymous=True)
    log_dir = os.path.expanduser("~/catkin_ws/logs")
    os.makedirs(log_dir, exist_ok=True)
    if len(sys.argv) > 1:
        out = sys.argv[1]
    else:
        out = os.path.join(log_dir, "nav_debug_%s.csv" % datetime.datetime.now().strftime("%m%d_%H%M%S"))
    rec = NavDebugRecorder(out)
    signal.signal(signal.SIGINT, lambda *_: (rec.stop(), sys.exit(0)))
    rec.run()


if __name__ == "__main__":
    main()
