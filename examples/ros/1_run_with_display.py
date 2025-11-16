#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS 모드의 가장 단순한 LiteBot 실행 예제.
# TODO: to include hough image
"""
import os
import sys

import rospy

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from litebot.bot import LiteBot  # noqa: E402
from litebot.utils.display_utils import display_images

IMAGE_TARGETS = [
    "original",
    "bev",
    "filtered",
    "gray",
    # "blur",
    # "binary",
    "canny",
    "hough",
]

def main():
    rospy.init_node("litebot_simple_runner", anonymous=False)
    litebot = LiteBot(mode="ros")
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # 1. 프레임 캡처
        litebot.frame = litebot.camera.get_frame()

        # 프레임이 없으면 처리 중단
        if litebot.frame is None:
            continue
        
        # 2. 이미지 처리
        litebot.images = litebot.image_processor.get_images(litebot.frame)
        
        # images가 None이면 빈 딕셔너리로 처리
        if litebot.images is None:
            litebot.images = {}
        display_images(litebot.images, image_targets=IMAGE_TARGETS)
        
        # 3. 감지 수행
        observations = {
            "lane": litebot.observer.observe_lines(litebot.images["hough"]),
            # "aruco": litebot.observer.observe_aruco(litebot.images["original"]),
            # "pothole": litebot.observer.observe_pothole(litebot.images["binary"]),
            # "qr_codes": litebot.observer.observe_qr_codes(litebot.images["original"]),
            # 필요한 경우 다른 감지 추가
        }
        
        # 4. 트리거 매니저가 적절한 액션과 우세 트리거명을 반환
        action, source = litebot.trigger_manager.step(observations)
        
        # 5. 액션 실행
        if action:
            litebot.action_executor.execute(action)
            rospy.loginfo("[LiteBot] source=%s action=%s value=%s", source, action[0], action[1])

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

