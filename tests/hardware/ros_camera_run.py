#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS 하드웨어 테스트 - 카메라 입력 확인
- roscore 및 카메라 토픽이 살아있는 환경에서 실행합니다.
"""
from __future__ import print_function

import os
import sys

try:
    import rospy
except Exception:
    rospy = None

try:
    import cv2
except Exception:
    cv2 = None

from litebot.io.ros.ros_camera import ROSCamera


def main():
    if rospy is None:
        print("[ROS Camera HW] rospy unavailable. Run in ROS environment.")
        sys.exit(1)

    rospy.init_node("ros_camera_run", anonymous=False)


    cam = ROSCamera()  # 기본 토픽: /usb_cam/image_raw/compressed
    print("[ROS Camera HW] Subscribed to {}".format(cam.topic_name))

    save_dir = os.path.join("tests", "_out")
    try:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
    except Exception:
        pass

    rate = rospy.Rate(10)  # 10 Hz
    saved = False
    frames = 0
    print("[ROS Camera HW] reading frames for ~5 seconds...")
    start = rospy.Time.now()
    while (rospy.Time.now() - start).to_sec() < 5.0 and not rospy.is_shutdown():
        frame = cam.get_frame()
        if frame is not None:
            frames += 1
            h, w = frame.shape[:2]
            print("[ROSCamera] frame {}: {}x{}".format(frames, w, h))
            if not saved and cv2 is not None:
                out_path = os.path.join(save_dir, "ros_cam_sample.jpg")
                try:
                    cv2.imwrite(out_path, frame)
                    print("[ROS Camera HW] saved sample to {}".format(out_path))
                    saved = True
                except Exception as e:
                    print("[ROS Camera HW] save failed: {}".format(e))
        rate.sleep()

    print("[ROS Camera HW] total frames: {}".format(frames))
    print("[ROS Camera HW] done.")


if __name__ == "__main__":
    main()


