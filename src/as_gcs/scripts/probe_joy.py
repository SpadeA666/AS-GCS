#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-
"""
TX12 (OpenTX Radiomaster TX12 Joystick) 输入映射探测。

读 /dev/input/js0 的 joystick API 事件，实时打印并在结束时汇总：
  - 哪些开关落成了 Button N
  - 哪些开关/旋钮/摇杆落成了 Axis N，以及各自的行程范围

用法:
    /usr/bin/python3 probe_joy.py [录制秒数]     # 默认 60 秒
    JOY_DEV=/dev/input/js1 /usr/bin/python3 probe_joy.py 30

注意: 用 /usr/bin/python3，不要用被 anaconda 抢占的 python3。
"""

import os
import select
import struct
import sys
import time

DEV = os.environ.get("JOY_DEV", "/dev/input/js0")
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


def main():
    try:
        fd = os.open(DEV, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        print("无法打开 %s: %s" % (DEV, e))
        print("提示: 确认遥控器已切到 USB Joystick 模式，且当前用户在 input 组或有 ACL。")
        return 1

    print("# 设备: %s" % DEV, flush=True)
    print("# 录制 %d 秒" % int(DUR), flush=True)
    print("#", flush=True)
    print("# 请依次拨动：", flush=True)
    print("#   1) 每个拨杆开关的【每一个位置】（上/中/下 都要停一下）", flush=True)
    print("#   2) 每个旋钮/电位器的两端", flush=True)
    print("#   3) 四个摇杆各自推到四个方向的极限，然后回中", flush=True)
    print("#", flush=True)

    btns = {}       # num -> 触发次数
    axes = {}       # num -> [min, max]
    init_axes = {}  # num -> 初始值
    n_ev = 0

    t0 = time.time()
    try:
        while time.time() - t0 < DUR:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                data = os.read(fd, 8 * 64)
            except BlockingIOError:
                continue
            if not data:
                continue
            for i in range(0, len(data) - 7, 8):
                _t, val, typ, num = struct.unpack("<IhBB", data[i:i + 8])
                base = typ & 0x7F
                is_init = bool(typ & JS_EVENT_INIT)
                el = time.time() - t0
                n_ev += 1

                if base == JS_EVENT_BUTTON:
                    if not is_init:
                        btns[num] = btns.get(num, 0) + 1
                        print("[%6.2fs] BUTTON %2d -> %s" % (el, num, "按下" if val else "松开"),
                              flush=True)
                elif base == JS_EVENT_AXIS:
                    a = axes.setdefault(num, [val, val])
                    a[0] = min(a[0], val)
                    a[1] = max(a[1], val)
                    if is_init:
                        init_axes[num] = val
                    else:
                        print("[%6.2fs] AXIS   %2d -> %6d" % (el, num, val), flush=True)
    except KeyboardInterrupt:
        print("\n# 手动中断", flush=True)
    finally:
        os.close(fd)

    elapsed = time.time() - t0
    print("", flush=True)
    print("===== 汇总 (录制 %.1fs, %d 个事件) =====" % (elapsed, n_ev), flush=True)

    print("-- 按钮（开关/按键）--", flush=True)
    if btns:
        for n in sorted(btns):
            print("  Button %2d : 触发 %d 次" % (n, btns[n]), flush=True)
    else:
        print("  (没有捕获到按钮事件)", flush=True)

    print("-- 轴（摇杆/旋钮/模拟开关）--", flush=True)
    if axes:
        for n in sorted(axes):
            lo, hi = axes[n]
            span = hi - lo
            if span > 20000:
                kind = "连续摇杆"
            elif span > 3000:
                kind = "旋钮/电位器"
            elif span > 200:
                kind = "开关(两/三档)"
            else:
                kind = "本轮没动"
            print("  Axis %2d   : 范围 %6d ~ %6d  (行程 %6d)  [%s]  初始=%d"
                  % (n, lo, hi, span, kind, init_axes.get(n, 0)), flush=True)
    else:
        print("  (没有捕获到轴事件)", flush=True)

    print("", flush=True)
    print("# 把上面这段贴回去，即可得到完整映射。", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
