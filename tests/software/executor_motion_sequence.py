#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Software 테스트 - ActionExecutor 경유 모션 시퀀스
- 전/후진, 원형 주행, 제자리 회전, 정지
"""
from __future__ import print_function

import time
import rospy

from litebot.action.action_executor import ActionExecutor


class DummyController(object):
    def update_speed_angular(self, v, w):
        print("[DummyController] update v={}, w={}".format(v, w))

    def brake(self):
        print("[DummyController] brake")

    def drive_forward_distance(self, distance, speed):
        print("[DummyController] forward distance={}, speed={}".format(distance, speed))
        time.sleep(0.5)

    def drive_backward_distance(self, distance, speed):
        print("[DummyController] backward distance={}, speed={}".format(distance, speed))
        time.sleep(0.5)

    def drive_circle_distance(self, distance, speed, angular_velocity):
        print("[DummyController] circle distance={}, speed={}, w={}".format(distance, speed, angular_velocity))
        time.sleep(0.5)

    def rotate_in_place(self, degrees, ang_speed=1.0):
        print("[DummyController] rotate {} deg @{} rad/s".format(degrees, ang_speed))
        time.sleep(0.5)

    def is_action_running(self):
        print("[DummyController] is action running")

    def execute_async(self, action):
        print("[DummyController] exectue async")





def main():
    FWD_DIST = 1.0
    BWD_DIST = 0.5
    SPEED = 0.25
    DIA = 0.4
    DIST = 1.0

    rospy.init_node("executor_motion_sequence", anonymous=False)
    ctrl = DummyController()
    ex = ActionExecutor(ctrl)

    print("[SW Motion] drive_forward")
    ex.execute(("drive_forward", (FWD_DIST, SPEED)))

    print("[SW Motion] drive_backward")
    ex.execute(("drive_backward", (BWD_DIST, SPEED)))

    print("[SW Motion] drive_circle left")
    ex.execute(("drive_circle", (DIST, SPEED, DIA, "left")))

    print("[SW Motion] drive_circle right")
    ex.execute(("drive_circle", (DIST, SPEED, DIA, "right")))

    print("[SW Motion] rotate +90 / -90")
    ex.execute(("rotate", 90.0))
    ex.execute(("rotate", -90.0))

    print("[SW Motion] stop")
    ex.execute(("stop", None))
    print("[SW Motion] done.")


if __name__ == "__main__":
    main()


