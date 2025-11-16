#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS 하드웨어 테스트 - ActionExecutor 경유 확인
"""
from __future__ import print_function

import sys

try:
    import rospy
except Exception:
    rospy = None

from litebot.io.ros.ros_controller import ROSController
from litebot.action.action_executor import ActionExecutor


def main():
    if rospy is None:
        print("[ROS Executor HW] rospy unavailable. Run in ROS environment.")
        sys.exit(1)

    # 파라미터
    SPEED = 0.25
    CIRCLE_DIAMETER = 0.4
    CIRCLE_DIST = 1.0

    print("[ROS Executor HW] init controller + executor")
    ctrl = ROSController()
    ex = ActionExecutor(ctrl)
    try:
        # --- 1) v-ω: center (2s 유지) ---
        print("[ROS Executor HW] update_speed_angular (center) - hold 2s @20Hz")
        rate = rospy.Rate(20)
        for _ in range(40):
            ex.execute(("update_speed_angular", {"speed": SPEED, "angular": 0.0}))
            rate.sleep()

        # --- 2) v-ω: left bias (2s 유지) ---
        print("[ROS Executor HW] update_speed_angular (left bias) - hold 2s @20Hz")
        rate = rospy.Rate(20)
        for _ in range(40):
            ex.execute(("update_speed_angular", {"speed": SPEED, "angular": 0.8}))
            rate.sleep()

        # --- 3) v-ω: right bias (2s 유지) ---
        print("[ROS Executor HW] update_speed_angular (right bias) - hold 2s @20Hz")
        rate = rospy.Rate(20)
        for _ in range(40):
            ex.execute(("update_speed_angular", {"speed": SPEED, "angular": -0.8}))
            rate.sleep()

        # --- 4) 원형 주행(좌) ---
        print("[ROS Executor HW] drive_circle via executor (left)")
        ex.execute(("drive_circle", (CIRCLE_DIST, SPEED, CIRCLE_DIAMETER, "left")))
        rospy.sleep(0.5)

        # --- 5) 원형 주행(우) ---
        print("[ROS Executor HW] drive_circle via executor (right)")
        ex.execute(("drive_circle", (CIRCLE_DIST, SPEED, CIRCLE_DIAMETER, "right")))
        rospy.sleep(0.5)

        # --- 6) 제자리 회전 +90 / -90 ---
        print("[ROS Executor HW] rotate +90")
        ex.execute(("rotate", 90.0))
        rospy.sleep(0.5)
        print("[ROS Executor HW] rotate -90")
        ex.execute(("rotate", -90.0))
        rospy.sleep(0.5)

        # --- 7) 정지 ---
        print("[ROS Executor HW] stop")
        ex.execute(("stop", None))
        print("[ROS Executor HW] Done.")
    except KeyboardInterrupt:
        print("\n[ROS Executor HW] Interrupted. Braking...")
        ctrl.brake()
    except Exception as exc:
        print("[ROS Executor HW] Error: {}".format(exc))
        ctrl.brake()


if __name__ == "__main__":
    main()


