#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ActionExecutor 수동 테스트 스크립트
- pytest 없이 직접 실행하며 동작을 눈으로 확인하는 용도
"""
from __future__ import print_function

import time
import os
import numpy as np
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

    def drive_forward_distance(self, distance, speed):
        print("[DummyController] forward distance={}, speed={}".format(distance, speed))
        time.sleep(max(0.0, min(1.0, distance / max(speed, 1e-6))))

    def drive_backward_distance(self, distance, speed):
        print("[DummyController] backward distance={}, speed={}".format(distance, speed))
        time.sleep(max(0.0, min(1.0, distance / max(speed, 1e-6))))

    def drive_circle_distance(self, distance, speed, angular_velocity):
        print("[DummyController] circle distance={}, speed={}, w={}".format(distance, speed, angular_velocity))
        time.sleep(max(0.0, min(1.0, distance / max(speed, 1e-6))))

    def rotate_in_place(self, degrees, ang_speed=1.0):
        print("[DummyController] rotate_in_place {} deg @{} rad/s".format(degrees, ang_speed))
        # emulate time based rotation (capped for demo)
        duration = abs(degrees) * 3.141592653589793 / 180.0 / max(ang_speed, 1e-6)
        time.sleep(min(duration, 1.0))


def main():
    rospy.init_node("manual_check", anonymous=False)
    ctrl = DummyController()
    ex = ActionExecutor(ctrl)

    # --- 1) v-ω 경로: 큰 combined_err 입력 시 각속도 클램프 확인 ---
    print("\n=== update_speed_angular: large combined_err -> should clamp ===")
    ex.execute(("update_speed_angular", {"speed": 0.3, "angular": 999.0}))

    # --- 2) 제자리 회전 ---
    print("\n=== rotate: 90 deg ===")
    ex.execute(("rotate", 90.0))

    # --- 3) 전진(거리 기반) ---
    print("\n=== drive_forward: 1.0 m @ 0.2 m/s ===")
    ex.execute(("drive_forward", (1.0, 0.2)))

    # --- 4) 원형 주행(좌) ---
    print("\n=== drive_circle: d=1.0, v=0.2, dia=0.4, left ===")
    ex.execute(("drive_circle", (1.0, 0.2, 0.4, "left")))

    # --- 5) QR 커맨드(로깅 경로 확인용) ---
    print("\n=== qr_command: TEST123 ===")
    ex.execute(("qr_command", "TEST123"))

    # --- 6) 캡처: 파일 저장 경로 확인 ---
    print("\n=== capture: save to tests/_out/sample.jpg ===")
    out_dir = os.path.join("tests", "_out")
    if not os.path.exists(out_dir):
        try:
            os.makedirs(out_dir)
        except Exception:
            pass
    dummy_image = (np.zeros((100, 160, 3)) + 255).astype("uint8")  # white image
    out_path = os.path.join(out_dir, "sample.jpg")
    ex.execute(("capture", {"image": dummy_image, "save_path": out_path}))

    # --- 7) 정지 ---
    print("\n=== stop ===")
    ex.execute(("stop", None))

    print("\n[OK] manual_check finished.")


if __name__ == "__main__":
    main()


