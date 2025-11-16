#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Software 테스트 - ActionExecutor 경유 v-ω 업데이트 시퀀스
- 하드웨어 없이 DummyController로 동작 로그만 확인
"""
from __future__ import print_function

import time
import rospy

from litebot.action.action_executor import ActionExecutor


class DummyController(object):
    def __init__(self):
        self.last_cmd = None

    def update_speed_angular(self, v, w):
        self.last_cmd = (v, w)
        print("[DummyController] update_speed_angular v={}, w={}".format(v, w))

    def brake(self):
        self.last_cmd = (0.0, 0.0)
        print("[DummyController] brake")

    # stubs to satisfy executor API
    def drive_forward_distance(self, distance, speed):
        pass

    def drive_backward_distance(self, distance, speed):
        pass

    def drive_circle_distance(self, distance, speed, angular_velocity):
        pass

    def rotate_in_place(self, degrees, ang_speed=1.0):
        pass


def main():
    SPEED = 0.25
    HZ = 20.0
    PERIOD = 1.0 / HZ
    HOLD_SEC = 2.0
    LOOPS = int(HOLD_SEC / PERIOD)

    rospy.init_node("executor_update_sequence", anonymous=False)
    ctrl = DummyController()
    ex = ActionExecutor(ctrl)

    print("[SW Exec] center (2s @20Hz)")
    for _ in range(LOOPS):
        ex.execute(("update_speed_angular", {"speed": SPEED, "angular": 0.0}))
        time.sleep(PERIOD)

    print("[SW Exec] left bias (2s @20Hz)")
    for _ in range(LOOPS):
        ex.execute(("update_speed_angular", {"speed": SPEED, "angular": 0.8}))
        time.sleep(PERIOD)

    print("[SW Exec] right bias (2s @20Hz)")
    for _ in range(LOOPS):
        ex.execute(("update_speed_angular", {"speed": SPEED, "angular": -0.8}))
        time.sleep(PERIOD)

    print("[SW Exec] stop")
    ex.execute(("stop", None))
    print("[SW Exec] done.")


if __name__ == "__main__":
    main()


