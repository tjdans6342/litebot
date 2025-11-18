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
        resource_obs_pairs, actions, sources = bot.step() # resource_obs_pairs: 리소스별 관찰 결과, actions: 리소스별 액션, sources: 리소스별 트리거명

        if resource_obs_pairs is None:
            rate.sleep()
            continue

        if bot.images:
            # motor 관찰 결과에서 lane 정보 가져오기
            motor_obs = None
            for resource_type, obs in resource_obs_pairs:
                if resource_type == "motor":
                    motor_obs = obs
                    break
            
            if motor_obs:
                lane_info = motor_obs.get("lane")
                if lane_info and lane_info.get("exist_lines", True) and "hough" in bot.images:
                    visualized = visualize_hough_image(bot.images["hough"], lane_info)
                    bot.images["hough"] = annotate_lane_info(visualized, lane_info)
            display_images(bot.images, image_targets=DISPLAY_IMAGE_TARGETS)

        # 모든 리소스별 액션 로그 출력
        for resource_type, action in actions.items():
            if action:
                rospy.loginfo("[LiteBot] resource=%s action=%s value=%s", resource_type, action[0], action[1])


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
