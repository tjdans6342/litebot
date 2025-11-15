#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ROS 모드에서 LiteBot를 실행하는 가장 간단한 엔트리 스크립트
"""
import sys
import os
import time
import rospy

# 프로젝트 루트를 Python path에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from ai.detection_bridge import DetectionBridge
from litebot.bot import LiteBot
from litebot.utils.display_utils import (
    display_images,
    annotate_lane_info,
    visualize_hough_image
)


DISPLAY_IMAGE_TARGETS = [
    ("Original", "original"),
    ("BEV", "bev"),
    ("Filtered", "filtered"),
    ("gray", "gray"),
    # ("Blurred", "blur"),
    # ("binary", "binary"),
    ("Canny", "canny"),
    ("Hough", "hough"),
]


def main():
    """
        LiteBot를 ROS 모드로 실행
    """
    rospy.init_node("litebot_runner", anonymous=False)
    bot = LiteBot(mode="ros")
    rate = rospy.Rate(20)
    try:
        bridge = DetectionBridge()
        rospy.loginfo("[LiteBot] DetectionBridge initialized.")
    except Exception as exc:
        rospy.logwarn("[LiteBot] DetectionBridge unavailable: %s", exc)
        bridge = None
    last_log_check = 0.0

    while not rospy.is_shutdown():
        observations, action = bot.step() # observations: 관찰 결과, action: 액션

        if observations is None:
            rate.sleep()
            continue

        if bot.images:
            lane_info = observations.get("lane")
            if lane_info and lane_info.get("exist_lines", True) and "hough" in bot.images:
                visualized = visualize_hough_image(bot.images["hough"], lane_info)
                bot.images["hough"] = annotate_lane_info(visualized, lane_info)
            display_images(bot.images, image_targets=DISPLAY_IMAGE_TARGETS)

        if action:
            rospy.loginfo("[LiteBot] action=%s value=%s", action[0], action[1])


        # 객체 감지 로그 확인  
        if bridge and time.time() - last_log_check >= 3.0:
            for e in bridge.read_new_logs():
                # e.timestamp: 로그 생성 시간
                # e.image_name: 이미지 파일명
                # e.counts: 객체 감지 결과 (각 클래스별 개수)
                # e.total_counts: 객체 감지 결과 총 개수
                rospy.loginfo("[DetectionBridge] %s %s %s %s", e.timestamp, e.image_name, e.counts, e.total_counts)
            last_log_check = time.time()

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
