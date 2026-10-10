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
    Axis 6   -> ch7   开关 C  三档正向   低=高度 / 中=定点 / 高=OFFBOARD
    Axis 7   -> ch8   开关 F  二值正向   高=抢回遥控器（见下）
    Button 0 -> ch9   开关 E            高=kill
    Button 1 -> ch10  开关 B            高=解锁

为什么「抢回遥控器」要在本节点里合成为切模式
----------------------------------------
理想做法是给杆 F 配一个「切到 POSCTL」的 PX4 开关，但 **v1.13.2 里
所有这类开关都已废弃**（RC_MAP_POSCTL_SW / RC_MAP_MAN_SW / RC_MAP_MODE_SW
/ RC_MAP_STAB_SW / RC_MAP_ACRO_SW / RC_MAP_RATT_SW 全在
modules/rc_update/params_deprecated.c 里）。实测给 RC_MAP_POSCTL_SW 写 8，
服务返回 success 且回读 value=8，但紧接着再读就变回 0 —— 参数根本不落盘。
（另：RC_MAP_LOITER_SW 是活的，但它切的是 AUTO.LOITER 自动悬停，
摇杆不控位置，拿来当「遥控器控制」是错的。）

所以改为：杆 F 拨到高位时，本节点把**模式通道 ch7 压到 PWM_MID**，
也就是 COM_FLTMODE4 那一档（定点）。这样只靠 RC_MAP_FLTMODE 一条通道
就实现了「一键夺回」，不依赖任何废弃参数。

必须放在发送前合成（build_channels），不能在 handle_axis 里直接写
self.ch[6] —— 那样杆 C 一动就把它覆盖掉了。

配套 PX4 参数
------------
    RC_MAP_ROLL=1 RC_MAP_PITCH=2 RC_MAP_THROTTLE=3 RC_MAP_YAW=4
    RC_MAP_FLTMODE=7      (杆 C；杆 F 的 ch8 不映射给 PX4，由本节点自己用)
    RC_MAP_OFFB_SW=0      (必须关：OFFBOARD 改由杆 C 最高档触发)
    RC_MAP_KILL_SW=9      RC_MAP_ARM_SW=10
    COM_FLTMODE1=1        (Altitude,   PWM 1000-1160) ← 杆 C 最低
    COM_FLTMODE4=2        (Position,   PWM 1480-1640) ← 杆 C 中间；也是夺回目标
    COM_FLTMODE6=7        (Offboard,   PWM 1800-2000) ← 杆 C 最高
    COM_RC_IN_MODE=2      (否则 selector 只认 MAVLink 源，会丢掉这些通道)
    COM_RC_OVERRIDE=3     (AUTO + OFFBOARD 都允许拨杆接管)
    COM_RC_STICK_OV=30    (拨杆接管阈值 %；=0 是越界值，任何抖动都会夺权)

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

# 通道号（1-based）
RC_CH_FLTMODE = 7      # 杆 C：模式三档（自稳 / 高度 / 定点）
RC_CH_TAKEOVER = 8     # 杆 F：地面站 ↔ 遥控器
# 与 PX4 的 RC_*_TH 默认 0.75 对齐：PWM > 1750 视为「拨到高位」
TAKEOVER_ON_PWM = 1750

# F 低位（地面站控制）时把 ch7 压到这个 PWM。
#
# 为什么不是 PWM_MAX(2000)：杆 C 最高档（也是 2000）要占 slot 6 = 定点，
# 两者会撞。于是改用 **slot 5**，单独配 COM_FLTMODE5 = Offboard。
#
# 1719 是按 PX4 的换算公式反推出来的（src/modules/rc_update/rc_update.cpp）：
#   mode_slot = (((((v+1.05)*6 + 1/6) / 2.1) + 1/6) + 1
#   v = (pwm - RC7_TRIM) / ((RC7_MAX - RC7_MIN)/2)     ← 本机实测 1000/2000/1500
# 解得 slot 5 对应 pwm ∈ [1632, 1807)，取中值 1719，两侧各余约 87。
TAKEOVER_OFFBOARD_PWM = 1719

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
    (6, 7, False),   # 开关 C 三档 -> 模式（高度 / 定点 / OFFBOARD）
    (7, 8, True),    # 开关 F -> 抢回遥控器（反向：TX12 上这根杆的轴极性与物理高低位相反）
                     #   2026-10-10 实测后反回：以前是 True，为了“高位=夺回”一度改成 False，
                     #   但实机上杆推高时轴值反而低，所以必须反向才能让“物理高位 = 夺回”。
                     #   夺回逻辑本身不看轴值、只看合成后的 PWM（>=1750），故反向不影响它。
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

    def build_channels(self):
        """把 self.ch 合成实际要发的 18 个通道。

        杆 F = 地面站控制 ↔ 遥控器控制 的【总开关】：
            F 低位 → ch7 压到 TAKEOVER_OFFBOARD_PWM（slot 5 → COM_FLTMODE5 = OFFBOARD）
            F 高位 → ch7 用杆 C 选的档位（自稳 / 高度 / 定点）

        ⚠ 为什么低位要“压到另一个档”而不能不干预：
        PX4 的模式切换是【边沿触发】——只有 mode_slot 变了才会发
        ACTION_SWITCH_MODE。如果只在高位时把 ch7 改成某个值（比如之前的
        压成 1500=定点），那么当杆 C 本来就在中间档（ch7 已经是 1500）时，
        “压成 1500”等于没有变化 → PX4 不触发 → 飞机留在 OFFBOARD 里，
        表现就是“拨了 F 却回不到遥控器的定点”。

        让低位输出 slot 5、高位输出 C 的档位，两侧值天然不同，
        无论 C 停在哪一档，拨 F 都一定会产生一次边沿。

        ⚠ 为什么是 slot 5 而不是 slot 6(2000)：
        slot 6 已经被杆 C 的最高档（定点）占用，两者会撞。

        为什么放在发送前合成而不是 handle_axis：杆 C 每次动都会重写 ch7，
        在那里写会被覆盖。
        """
        out = list(self.ch)
        if out[RC_CH_TAKEOVER - 1] >= TAKEOVER_ON_PWM:
            # F 高位：遥控器控制 → ch7 保持杆 C 选定的档位（自稳/高度/定点），不干预
            pass
        else:
            # F 低位：地面站控制 → ch7 压到 slot 5（COM_FLTMODE5 = OFFBOARD）
            out[RC_CH_FLTMODE - 1] = TAKEOVER_OFFBOARD_PWM
        return out

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
                msg.channels = self.build_channels()
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
