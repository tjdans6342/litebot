#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 하드웨어 테스트 - 컨트롤러 단독 확인 (현재 스텁 출력)
"""
from __future__ import print_function

import time

from litebot.io.tiki.tiki_controller import TikiController


def main():
    # 파라미터
    FWD_DIST = 1.0
    BWD_DIST = 0.5
    SPEED = 0.25
    ANG_SPEED = 0.6
    CIRCLE_DIAMETER = 0.4
    CIRCLE_DIST = 1.0

    print("[Tiki Controller HW] init controller")
    ctrl = TikiController()
    try:
        # --- 1) 직진(거리 기반) ---
        print("[Tiki Controller HW] forward {} m @ {} m/s".format(FWD_DIST, SPEED))
        ctrl.drive_forward_distance(FWD_DIST, SPEED)
        time.sleep(0.5)

        # --- 2) 후진(거리 기반) ---
        print("[Tiki Controller HW] backward {} m @ {} m/s".format(BWD_DIST, SPEED))
        ctrl.drive_backward_distance(BWD_DIST, SPEED)
        time.sleep(0.5)

        radius = CIRCLE_DIAMETER / 2.0
        ang_vel = SPEED / max(radius, 1e-6)
        # --- 3) 원형 주행(좌) ---
        print("[Tiki Controller HW] circle LEFT dist={}, v={}, dia={}".format(CIRCLE_DIST, SPEED, CIRCLE_DIAMETER))
        ctrl.drive_circle_distance(CIRCLE_DIST, SPEED, ang_vel)
        time.sleep(0.5)

        # --- 4) 원형 주행(우) ---
        print("[Tiki Controller HW] circle RIGHT dist={}, v={}, dia={}".format(CIRCLE_DIST, SPEED, CIRCLE_DIAMETER))
        ctrl.drive_circle_distance(CIRCLE_DIST, SPEED, -ang_vel)
        time.sleep(0.5)

        # --- 5) v-ω 단일 명령 테스트 (2s 유지) ---
        print("[Tiki Controller HW] update_speed_angular keep 2s (v={}, w=0.3) @20Hz".format(SPEED))
        for _ in range(40):
            ctrl.update_speed_angular(SPEED, 0.3)
            time.sleep(0.05)

        # --- 6) 제자리 회전 +90 / -90 ---
        print("[Tiki Controller HW] rotate +90 @ {} rad/s".format(ANG_SPEED))
        ctrl.rotate_in_place(90.0, ang_speed=ANG_SPEED)
        time.sleep(0.5)
        print("[Tiki Controller HW] rotate -90 @ {} rad/s".format(ANG_SPEED))
        ctrl.rotate_in_place(-90.0, ang_speed=ANG_SPEED)
        time.sleep(0.5)

        # --- 7) 정지 ---
        print("[Tiki Controller HW] Brake")
        ctrl.brake()
        print("[Tiki Controller HW] Done.")
    except KeyboardInterrupt:
        print("\n[Tiki Controller HW] Interrupted. Braking...")
        ctrl.brake()
    except Exception as exc:
        print("[Tiki Controller HW] Error: {}".format(exc))
        ctrl.brake()


if __name__ == "__main__":
    main()


