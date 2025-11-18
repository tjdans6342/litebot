#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
통합 테스트 - LiteBot으로 주행/관측/트리거를 묶고,
트리거 결과가 'aruco'일 때 현재 프레임을 캡처합니다.
"""
from __future__ import print_function

import os
import sys
import time
from datetime import datetime

import cv2

from litebot.bot import LiteBot


def _ensure_out_dir():
    out_dir = os.path.join("tests", "_out")
    try:
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
    except Exception:
        pass
    return out_dir


def main():
    # mode 인자 받기 (기본값: "ros")
    mode = sys.argv[1] if len(sys.argv) > 1 else "ros"
    
    # ROS 모드인 경우에만 rospy 초기화
    if mode == "ros":
        import rospy
        rospy.init_node("litebot_aruco_capture", anonymous=False)
        rate = rospy.Rate(20)
        is_shutdown = rospy.is_shutdown
        log_func = rospy.loginfo
    else:
        # Tiki 모드인 경우
        rate = None  # time.sleep으로 대체
        is_shutdown = lambda: False
        log_func = print
    
    litebot = LiteBot(mode=mode)
    save_dir = _ensure_out_dir()

    while not is_shutdown():
        # 1. 프레임 캡처
        litebot.frame = litebot.camera.get_frame()

        # 프레임이 없으면 처리 중단
        if litebot.frame is None:
            if mode == "ros":
                rate.sleep()
            else:
                time.sleep(0.05)  # 20Hz
            continue

        # 2. 이미지 처리
        litebot.images = litebot.image_processor.get_images(litebot.frame)
        if litebot.images is None:
            litebot.images = {}

        # 3. 감지 수행 (필요 항목만 추가)
        observations = {
            "lane": litebot.observer.observe_lines(litebot.images.get("hough", litebot.frame)),
            "aruco": litebot.observer.observe_aruco(litebot.images.get("original", litebot.frame)),
            # 다른 감지는 필요 시 확장
        }

        # 4. 트리거 매니저가 적절한 액션과 우세 트리거명을 반환
        # 주의: bot.py의 새로운 구조에서는 trigger_managers를 사용하지만,
        # 여기서는 기존 방식(단일 trigger_manager)을 유지합니다.
        motor_obs = {
            "lane": observations["lane"],
            "aruco": observations["aruco"],
        }
        action, source = litebot.trigger_managers["motor"].step(motor_obs)

        # 5. 액션 실행: 우세 트리거가 아루코면 캡처, 그 외는 정상 실행
        if action:
            cmd, val = action
            if source != 'lane':
                log_func("[LiteBot] source=%s action=%s value=%s", source, action[0], action[1])
            litebot.action_executor.execute(action)
            
            # ArUco 마커가 감지되면 이미지 저장
            if source == "aruco":
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                marker = observations.get("aruco")
                marker_id = marker.get("id") if marker else None
                if marker_id is not None:
                    filename = "aruco_{}_{}.jpg".format(marker_id, timestamp)
                else:
                    filename = "aruco_{}.jpg".format(timestamp)
                
                save_path = os.path.join(save_dir, filename)
                cv2.imwrite(save_path, litebot.frame)
                log_func("[LiteBot] ArUco marker detected (ID: %s) and image saved: %s", marker_id, save_path)

        if mode == "ros":
            rate.sleep()
        else:
            time.sleep(0.05)  # 20Hz


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[LiteBot] Interrupted by user")
    except Exception as e:
        print("[LiteBot] Error: {}".format(e))
        import traceback
        traceback.print_exc()


# # ROS 모드 (기본값)
# python tests/integration/capture_aruco.py

# # 또는 명시적으로
# python tests/integration/capture_aruco.py ros

# # Tiki 모드
# python tests/integration/capture_aruco.py tiki