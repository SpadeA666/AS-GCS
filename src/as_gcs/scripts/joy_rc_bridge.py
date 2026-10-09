#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
"""
joy_rc_bridge —— 把 TX12 的 USB joystick 输入灌进 PX4 SITL 的 RC 通道。

为什么需要这个
--------------
QGC 自带的 joystick 只发 MAVLINK MANUAL_CONTROL，走的是
    mavlink_receiver -> manual_control_input -> manual_control_setpoint
这条链路。它永远不会产生 input_rc / rc_channels / manual_control_switches，
所以 RC_MAP_KILL_SW / RC_MAP_ARM_SW / RC_MAP_FLTMODE / RC_MAP_OFFB_SW
这些开关在仿真里全是死的（QGC v4.2 二进制里也没有任何 RC override 代码）。

本节点改发 RC_CHANNELS_OVERRIDE：
    input_rc (RC_INPUT_SOURCE_MAVLINK)
      -> rc_update -> rc_channels + manual_control_switches
      -> manual_control -> action_request
      -> Commander
这条链路和真机上的物理接收机完全同构，所以开关能真正生效，
参数也能原样搬到真机。

通道映射（对齐实机习惯）
----------------------
    Axis 0   -> ch1   Roll
    Axis 1   -> ch2   Pitch
    Axis 2   -> ch3   Throttle
    Axis 3   -> ch4   Yaw
    Axis 6   -> ch7   开关 C  三档正向   低=自稳 / 中=高度 / 高=定点
    Axis 7   -> ch8   开关 F  三档反向   低=offboard
    Button 0 -> ch9   开关 E            高=kill
    Button 1 -> ch10  开关 B            高=解锁

为什么 ch8 要反向
-----------------
PX4 的 get_rc_sw2pos_position() 里，on_th 为负时判据是 value < on_th，
而 value = 0.5*channel+0.5 恒在 [0,1]，负阈值永远不成立 —— 负阈值反向
在 v1.13.2 上是坏的。所以反向只能在这里做。

配套 PX4 参数
------------
    RC_MAP_ROLL=1 RC_MAP_PITCH=2 RC_MAP_THROTTLE=3 RC_MAP_YAW=4
    RC_MAP_FLTMODE=7      RC_MAP_OFFB_SW=8
    RC_MAP_KILL_SW=9      RC_MAP_ARM_SW=10
    COM_FLTMODE1=8        (Stabilized, PWM 1000-1160)
    COM_FLTMODE4=1        (Altitude,   PWM 1480-1640)
    COM_FLTMODE6=2        (Position,   PWM 1800-2000)
    COM_RC_IN_MODE=2      (否则 selector 只认 MAVLink 源，会丢掉这些通道)
    COM_RC_OVERRIDE=3     (AUTO + OFFBOARD 都允许拨杆接管)

用法
----
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    /usr/bin/python3 ~/catkin_ws/src/as_gcs/scripts/joy_rc_bridge.py

    # 可选 ROS 参数
    _js_dev:=/dev/input/js0  _rate:=50  _topic:=/iris_0/mavros/rc/override
    _verbose:=true
"""

import os
import select
import struct
import sys

import rospy
from mavros_msgs.msg import OverrideRCIn

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000
AXIS_SCALE = 32767.0
NCHAN = 18

# (joystick 编号, 目标通道号(1-based), 是否反向)
#
# 反向位的依据：QGC 里 TX12 的配置是 Axis1Rev=true（其余三轴都是 false），
# 也就是 Pitch 轴需要反向。桥直接绕过 QGC，所以必须自己把这个反向带上，
# 否则俯仰会反。
AXIS_MAP = [
    (0, 1, False),   # 右摇杆水平 -> Roll
    (1, 2, True),    # 右摇杆垂直 -> Pitch（反向，对齐 QGC 的 Axis1Rev=true）
    (2, 3, False),   # 左摇杆垂直 -> Throttle
    (3, 4, False),   # 左摇杆水平 -> Yaw
    (6, 7, False),   # 开关 C 三档 -> 模式
    (7, 8, True),    # 开关 F 三档 -> offboard（反向：低轴值 = 高 PWM）
]

BUTTON_MAP = [
    (0, 9, False),   # 开关 E -> kill
    (1, 10, False),  # 开关 B -> 解锁
]


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class JoyRCBridge(object):

    def __init__(self):
        self.js_dev = rospy.get_param("~js_dev", "/dev/input/js0")
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.topic = rospy.get_param("~topic", "/iris_0/mavros/rc/override")
        self.verbose = bool(rospy.get_param("~verbose", True))

        self.ch = [PWM_MID] * NCHAN
        self.seen_axis = set()
        self.seen_button = set()

        self.fd = None
        self.pub = None

    def open_joystick(self):
        try:
            self.fd = os.open(self.js_dev, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            rospy.logfatal("无法打开 %s: %s", self.js_dev, e)
            rospy.logfatal("确认遥控器已切到 USB Joystick 模式（会出现 /dev/input/js0）")
            return False
        rospy.loginfo("已打开 %s", self.js_dev)
        return True

    def handle_axis(self, num, val):
        for js_id, chan, invert in AXIS_MAP:
            if js_id != num:
                continue
            pwm = PWM_MID + int(val / AXIS_SCALE * 500.0)
            if invert:
                pwm = (PWM_MIN + PWM_MAX) - pwm
            self.ch[chan - 1] = clamp(pwm, PWM_MIN, PWM_MAX)
            if self.verbose and num not in self.seen_axis:
                self.seen_axis.add(num)
                rospy.loginfo("Axis %d 已绑定 -> ch%d%s", num, chan, "（反向）" if invert else "")

    def handle_button(self, num, val):
        for js_id, chan, invert in BUTTON_MAP:
            if js_id != num:
                continue
            pwm = PWM_MAX if val else PWM_MIN
            if invert:
                pwm = (PWM_MIN + PWM_MAX) - pwm
            self.ch[chan - 1] = pwm
            if self.verbose and num not in self.seen_button:
                self.seen_button.add(num)
                rospy.loginfo("Button %d 已绑定 -> ch%d", num, chan)

    def drain_events(self, timeout):
        """读空 joystick 事件队列。返回是否读到了数据。"""
        got = False
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return False
        while True:
            try:
                data = os.read(self.fd, 8 * 64)
            except BlockingIOError:
                break
            except OSError as e:
                rospy.logerr("读取 %s 出错: %s", self.js_dev, e)
                break
            if not data:
                break
            got = True
            for i in range(0, len(data) - 7, 8):
                _t, val, typ, num = struct.unpack("<IhBB", data[i:i + 8])
                base = typ & 0x7F
                if base == JS_EVENT_AXIS:
                    self.handle_axis(num, val)
                elif base == JS_EVENT_BUTTON:
                    self.handle_button(num, val)
            # 继续尝试读，直到队列空
            r, _, _ = select.select([self.fd], [], [], 0)
            if not r:
                break
        return got

    def run(self):
        rospy.init_node("joy_rc_bridge", anonymous=False)
        if not self.open_joystick():
            return 1

        self.pub = rospy.Publisher(self.topic, OverrideRCIn, queue_size=1)
        rospy.loginfo("发布 RC override -> %s @ %.0f Hz", self.topic, self.rate_hz)
        rospy.loginfo("等待 joystick 事件（拨一下摇杆/开关即可看到绑定日志）...")

        rate = rospy.Rate(self.rate_hz)
        period = 1.0 / self.rate_hz
        try:
            while not rospy.is_shutdown():
                self.drain_events(period)

                msg = OverrideRCIn()
                msg.channels = list(self.ch)
                self.pub.publish(msg)

                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            if self.fd is not None:
                os.close(self.fd)
            rospy.loginfo("joy_rc_bridge 退出")
        return 0


if __name__ == "__main__":
    try:
        sys.exit(JoyRCBridge().run())
    except rospy.ROSInterruptException:
        sys.exit(0)
