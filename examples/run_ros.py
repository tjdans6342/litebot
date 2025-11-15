#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ROS 모드에서 LiteBot를 실행하는 가장 간단한 엔트리 스크립트
"""
import sys
import os
import rospy

# 프로젝트 루트를 Python path에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

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

    while not rospy.is_shutdown():
        observations, action = bot.step()

        if observations is None:
            rate.sleep()
            continue

        if bot.images:
            lane_info = observations.get("lane")
            if lane_info and "hough" in bot.images:
                visualized = visualize_hough_image(bot.images["hough"], lane_info)
                bot.images["hough"] = annotate_lane_info(visualized, lane_info)
            display_images(bot.images, image_targets=DISPLAY_IMAGE_TARGETS)

        if action:
            rospy.loginfo("[LiteBot] action=%s value=%s", action[0], action[1])

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
