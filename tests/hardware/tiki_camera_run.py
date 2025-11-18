#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 하드웨어 테스트 - 카메라 입력 확인
"""
from __future__ import print_function

import os
import sys
import time

try:
    import cv2
except Exception:
    cv2 = None

# Tiki 카메라 구현이 있을 경우를 가정
try:
    from litebot.io.tiki.tiki_camera import TikiCamera  # 존재하지 않으면 안내만 출력
except Exception:
    TikiCamera = None


def main():
    if TikiCamera is None:
        print("[Tiki Camera HW] litebot.io.tiki.tiki_camera.TikiCamera 가 없습니다.")
        print("[Tiki Camera HW] Tiki 카메라 모듈이 준비되면 이 스크립트로 프레임을 확인할 수 있습니다.")
        sys.exit(1)

    cam = TikiCamera()
    print("[Tiki Camera HW] initialized TikiCamera")

    save_dir = os.path.join("tests", "_out")
    try:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
    except Exception:
        pass

    saved = False
    frames = 0
    print("[Tiki Camera HW] reading frames for ~5 seconds...")
    start = time.time()
    while (time.time() - start) < 5.0:
        frame = cam.get_frame() if hasattr(cam, "get_frame") else None
        if frame is not None:
            frames += 1
            h, w = frame.shape[:2]
            print("[TikiCamera] frame {}: {}x{}".format(frames, w, h))
            if not saved and cv2 is not None:
                out_path = os.path.join(save_dir, "tiki_cam_sample.jpg")
                try:
                    cv2.imwrite(out_path, frame)
                    print("[Tiki Camera HW] saved sample to {}".format(out_path))
                    saved = True
                except Exception as e:
                    print("[Tiki Camera HW] save failed: {}".format(e))
        time.sleep(0.1)

    print("[Tiki Camera HW] total frames: {}".format(frames))
    print("[Tiki Camera HW] done.")


if __name__ == "__main__":
    main()


