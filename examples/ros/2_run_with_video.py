#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LiteBot 주행 + 비디오 캡처 예제.
"""
import os
import sys

import rospy

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from litebot.bot import LiteBot  # noqa: E402
from litebot.analysis.video_recorder import VideoRecorder  # noqa: E402
from litebot.analysis.analysis_manager import AnalysisManager


def main():
    rospy.init_node("litebot_video_runner", anonymous=False)
    bot = LiteBot(mode="ros")
    rate = rospy.Rate(20)

    recorder = VideoRecorder()
    # recorder.set_save_path("recordings/ros_run.avi")
    recorder.start_recording()
    rospy.loginfo("[LiteBot] Video recording started: %s", recorder.save_path)

    try:
        while not rospy.is_shutdown():
            observations, action, source = bot.step()

            if bot.frame is not None:
                manager = AnalysisManager()
                grid_image = manager.make_images_grid(list(bot.images.values()))
                try:
                    recorder.add_frame(grid_image)
                except Exception as exc:
                    rospy.logwarn("[LiteBot] failed to add frame to recorder: %s", exc)

            if observations is None:
                rate.sleep()
                continue

            if action:
                rospy.loginfo("[LiteBot] action=%s value=%s", action[0], action[1])

            rate.sleep()
    finally:
        recorder.stop_recording()
        rospy.loginfo("[LiteBot] Video recording finished.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

