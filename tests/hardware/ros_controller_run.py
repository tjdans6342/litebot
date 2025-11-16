#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS 하드웨어 테스트 - 컨트롤러 단독 확인
"""
from __future__ import print_function

import sys

try:
    import rospy
except Exception:
    rospy = None

from litebot.io.ros.ros_controller import ROSController


def main():
    if rospy is None:
        print("[ROS Controller HW] rospy unavailable. Run in ROS environment.")
        sys.exit(1)

    # 파라미터
    FWD_DIST = 1.0
    BWD_DIST = 0.5
    SPEED = 0.25
    ANG_SPEED = 0.6
    CIRCLE_DIAMETER = 0.4
    CIRCLE_DIST = 1.0

    print("[ROS Controller HW] init controller")
    ctrl = ROSController()
    try:
        # --- 1) 직진(거리 기반) ---
        print("[ROS Controller HW] forward {} m @ {} m/s".format(FWD_DIST, SPEED))
        ctrl.drive_forward_distance(FWD_DIST, SPEED)
        rospy.sleep(0.5)

        # --- 2) 후진(거리 기반) ---
        print("[ROS Controller HW] backward {} m @ {} m/s".format(BWD_DIST, SPEED))
        ctrl.drive_backward_distance(BWD_DIST, SPEED)
        rospy.sleep(0.5)

        radius = CIRCLE_DIAMETER / 2.0
        ang_vel = SPEED / max(radius, 1e-6)
        # --- 3) 원형 주행(좌) ---
        print("[ROS Controller HW] circle LEFT dist={}, v={}, dia={}".format(CIRCLE_DIST, SPEED, CIRCLE_DIAMETER))
        ctrl.drive_circle_distance(CIRCLE_DIST, SPEED, ang_vel)
        rospy.sleep(0.5)

        # --- 4) 원형 주행(우) ---
        print("[ROS Controller HW] circle RIGHT dist={}, v={}, dia={}".format(CIRCLE_DIST, SPEED, CIRCLE_DIAMETER))
        ctrl.drive_circle_distance(CIRCLE_DIST, SPEED, -ang_vel)
        rospy.sleep(0.5)

        # --- 5) v-ω 단일 명령 테스트 (2s 유지) ---
        print("[ROS Controller HW] update_speed_angular keep 2s (v={}, w=0.3) @20Hz".format(SPEED))
        rate = rospy.Rate(20)
        for _ in range(40):
            ctrl.update_speed_angular(SPEED, 0.3)
            rate.sleep()

        # --- 6) 제자리 회전 +90 / -90 ---
        print("[ROS Controller HW] rotate +90 @ {} rad/s".format(ANG_SPEED))
        ctrl.rotate_in_place(90.0, ang_speed=ANG_SPEED)
        rospy.sleep(0.5)
        print("[ROS Controller HW] rotate -90 @ {} rad/s".format(ANG_SPEED))
        ctrl.rotate_in_place(-90.0, ang_speed=ANG_SPEED)
        rospy.sleep(0.5)

        # --- 7) 정지 ---
        print("[ROS Controller HW] Brake")
        ctrl.brake()
        print("[ROS Controller HW] Done.")
    except KeyboardInterrupt:
        print("\n[ROS Controller HW] Interrupted. Braking...")
        ctrl.brake()
    except Exception as exc:
        print("[ROS Controller HW] Error: {}".format(exc))
        ctrl.brake()


if __name__ == "__main__":
    main()


