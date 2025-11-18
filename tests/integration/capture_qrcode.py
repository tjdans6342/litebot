#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
통합 테스트 - LiteBot으로 QR 코드를 감지하고,
QR 코드가 감지되면 현재 프레임을 tests/out에 저장합니다.
QR 코드 트리거와 Motor 관련 트리거만 동작합니다.
"""
from __future__ import print_function

import os
import sys
import time
from datetime import datetime

import cv2

from litebot.bot import LiteBot
from litebot.core.trigger_manager import TriggerManager
from litebot.core.trigger_qrcode import QrcodeTrigger
from litebot.core.trigger_lane import LaneTrigger
from litebot.core.trigger_aruco import ArucoTrigger
from litebot.core.trigger_pothole import PotholeTrigger


def _ensure_out_dir():
    """tests/_out 디렉터리가 없으면 생성"""
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
        rospy.init_node("litebot_qrcode_capture", anonymous=False)
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

    # QR 코드 트리거만 사용하는 TriggerManager (리소스 독립)
    qr_trigger_manager = TriggerManager(resource=None)
    qr_trigger_manager.triggers = [QrcodeTrigger()]

    # Motor 관련 트리거만 사용하는 TriggerManager (모터 리소스)
    motor_trigger_manager = TriggerManager(resource=litebot.controller)
    motor_trigger_manager.triggers = [
        LaneTrigger(),
        ArucoTrigger(),
        PotholeTrigger()
    ]

    # 기존 trigger_managers를 커스터마이즈된 것으로 교체
    litebot.trigger_managers = {
        None: qr_trigger_manager,
        "motor": motor_trigger_manager
    }

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

        # 3. 감지 수행
        qr_code_result = litebot.observer.observe_qr_codes(
            litebot.images.get("original", litebot.frame)
        )
        # QrcodeTrigger는 리스트를 기대하므로 리스트로 감싸기
        default_obs = {
            "qr_codes": [qr_code_result] if qr_code_result else None,
        }
        motor_obs = {
            "lane": litebot.observer.observe_lines(
                litebot.images.get("hough")
            ),
            "aruco": litebot.observer.observe_aruco(
                litebot.images.get("original", litebot.frame)
            ),
            "pothole": litebot.observer.observe_pothole(
                litebot.images.get("binary")
            ),
        }

        # 4. 트리거 매니저에서 액션 수집
        actions = {}
        sources = {}

        # QR 코드 트리거 체크
        qr_action, qr_source = qr_trigger_manager.step(default_obs)
        if qr_action:
            actions[None] = qr_action
            sources[None] = qr_source

        # Motor 관련 트리거 체크
        motor_action, motor_source = motor_trigger_manager.step(motor_obs)
        if motor_action:
            actions["motor"] = motor_action
            sources["motor"] = motor_source

        # 5. 액션 실행
        for resource_type, action in actions.items():
            litebot.action_executor.execute(action)
            
            # QR 코드가 감지되면 이미지 저장
            if resource_type is None and sources.get(None) == "qrcode":
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                qr_code_value = qr_code_result  # 위에서 저장한 원본 결과 사용
                if qr_code_value:
                    # QR 코드 값에 따라 파일명 생성
                    qr_label = str(qr_code_value).replace(" ", "_").replace("/", "_")
                    filename = "qrcode_{}_{}.jpg".format(qr_label, timestamp)
                else:
                    filename = "qrcode_{}.jpg".format(timestamp)
                
                save_path = os.path.join(save_dir, filename)
                cv2.imwrite(save_path, litebot.frame)
                log_func("[LiteBot] QR code detected (%s) and image saved: %s", qr_code_value, save_path)
            
            # 액션 로그 출력 (lane 제외)
            source = sources.get(resource_type)
            if source and source != 'lane':
                log_func(
                    "[LiteBot] source=%s action=%s value=%s",
                    source, action[0], action[1]
                )

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
# python tests/integration/capture_qrcode.py

# # 또는 명시적으로
# python tests/integration/capture_qrcode.py ros

# # Tiki 모드
# python tests/integration/capture_qrcode.py tiki