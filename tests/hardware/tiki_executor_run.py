#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 하드웨어 테스트 - ActionExecutor 경유 확인 (현재 스텁 출력)
"""
from __future__ import print_function

import time

from litebot.io.tiki.tiki_controller import TikiController
from litebot.action.action_executor import ActionExecutor


def main():
    # 파라미터
    SPEED = 0.25
    CIRCLE_DIAMETER = 0.4
    CIRCLE_DIST = 1.0

    print("[Tiki Executor HW] init controller + executor")
    ctrl = TikiController()
    ex = ActionExecutor(ctrl)
    try:
        # --- 1) v-ω: center (2s 유지) ---
        print("[Tiki Executor HW] center - hold 2s @20Hz")
        for _ in range(40):
            ex.execute(("update_speed_angular", {"speed": SPEED, "angular": 0.0}))
            time.sleep(0.05)

        # --- 2) v-ω: left bias (2s 유지) ---
        print("[Tiki Executor HW] left bias - hold 2s @20Hz")
        for _ in range(40):
            ex.execute(("update_speed_angular", {"speed": SPEED, "angular": 0.8}))
            time.sleep(0.05)

        # --- 3) v-ω: right bias (2s 유지) ---
        print("[Tiki Executor HW] right bias - hold 2s @20Hz")
        for _ in range(40):
            ex.execute(("update_speed_angular", {"speed": SPEED, "angular": -0.8}))
            time.sleep(0.05)

        # --- 4) 원형 주행(좌/우) ---
        print("[Tiki Executor HW] circle left via executor")
        ex.execute(("drive_circle", (CIRCLE_DIST, SPEED, CIRCLE_DIAMETER, "left")))
        time.sleep(0.5)
        print("[Tiki Executor HW] circle right via executor")
        ex.execute(("drive_circle", (CIRCLE_DIST, SPEED, CIRCLE_DIAMETER, "right")))
        time.sleep(0.5)

        # --- 5) 제자리 회전 +90 / -90 ---
        print("[Tiki Executor HW] rotate +90")
        ex.execute(("rotate", 90.0))
        time.sleep(0.5)
        print("[Tiki Executor HW] rotate -90")
        ex.execute(("rotate", -90.0))
        time.sleep(0.5)

        # --- 6) 정지 ---
        print("[Tiki Executor HW] stop")
        ex.execute(("stop", None))
        print("[Tiki Executor HW] Done.")
    except KeyboardInterrupt:
        print("\n[Tiki Executor HW] Interrupted. Braking...")
        ctrl.brake()
    except Exception as exc:
        print("[Tiki Executor HW] Error: {}".format(exc))
        ctrl.brake()


if __name__ == "__main__":
    main()


