#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Software 테스트 - 캡처/QR 경로 확인
"""
from __future__ import print_function

import os
import time
import numpy as np

from litebot.action.action_executor import ActionExecutor


class DummyController(object):
    def update_speed_angular(self, v, w): pass
    def brake(self): pass
    def drive_forward_distance(self, distance, speed): pass
    def drive_backward_distance(self, distance, speed): pass
    def drive_circle_distance(self, distance, speed, angular_velocity): pass
    def rotate_in_place(self, degrees, ang_speed=1.0): pass


def main():
    ex = ActionExecutor(DummyController())

    print("[SW Capture/QR] qr_command")
    ex.execute(("qr_command", "TEST-QR-001"))

    print("[SW Capture/QR] capture to tests/_out/capture.jpg")
    out_dir = os.path.join("tests", "_out")
    try:
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
    except Exception:
        pass
    image = (np.zeros((120, 160, 3)) + 200).astype("uint8")
    out_path = os.path.join(out_dir, "capture.jpg")
    ex.execute(("capture", {"image": image, "save_path": out_path}))

    print("[SW Capture/QR] done.")


if __name__ == "__main__":
    main()


